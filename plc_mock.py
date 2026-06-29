import time
import argparse
import threading
from dataclasses import dataclass
from config_parser import load_config
from mqtt_manager import FlexibleProtocolMQTTManager
from mqtt_message_binding import ComponentMessage, custom_component_mapping
from mqtt_data_exchange import PUBLISH_TOPICS, SUBSCRIBE_TOPICS, apply_exchange_config
from struct_utils import print_byte_by_byte
from tcp_framed_transport import TcpFramedClient

from multiprocessing import Process, Lock
from multiprocessing.sharedctypes import Value, Array

def _struct_endianness(topic_def, config=None):
    default_endianness = config.mqttSettings.endianness if config is not None else "LE"
    endianness = str(topic_def.get("endianness", default_endianness)).upper()
    return "<" if endianness == "LE" else ">"


@dataclass
class PerformanceMetrics:
    report_interval: float
    rx_count: int = 0
    tx_count: int = 0
    rx_bytes: int = 0
    tx_bytes: int = 0
    malformed_count: int = 0
    tx_drop_count: int = 0
    latency_count: int = 0
    latency_total_ms: float = 0.0
    latency_min_ms: float | None = None
    latency_max_ms: float | None = None

    def __post_init__(self):
        self._lock = threading.Lock()
        self._last_report = time.monotonic()
        self._last_rx_count = 0
        self._last_tx_count = 0
        self._last_rx_bytes = 0
        self._last_tx_bytes = 0

    def record_rx(self, payload_size: int, latency_ms: float | None = None) -> None:
        with self._lock:
            self.rx_count += 1
            self.rx_bytes += payload_size
            if latency_ms is not None and latency_ms >= 0:
                self.latency_count += 1
                self.latency_total_ms += latency_ms
                self.latency_min_ms = latency_ms if self.latency_min_ms is None else min(self.latency_min_ms, latency_ms)
                self.latency_max_ms = latency_ms if self.latency_max_ms is None else max(self.latency_max_ms, latency_ms)

    def record_tx(self, payload_size: int, sent: bool) -> None:
        with self._lock:
            if sent:
                self.tx_count += 1
                self.tx_bytes += payload_size
            else:
                self.tx_drop_count += 1

    def record_malformed(self) -> None:
        with self._lock:
            self.malformed_count += 1

    def maybe_report(self) -> None:
        now = time.monotonic()
        with self._lock:
            elapsed = now - self._last_report
            if elapsed < self.report_interval:
                return

            rx_delta = self.rx_count - self._last_rx_count
            tx_delta = self.tx_count - self._last_tx_count
            rx_byte_delta = self.rx_bytes - self._last_rx_bytes
            tx_byte_delta = self.tx_bytes - self._last_tx_bytes
            avg_latency = (
                self.latency_total_ms / self.latency_count
                if self.latency_count
                else None
            )

            latency_text = "n/a"
            if avg_latency is not None:
                latency_text = (
                    f"avg {avg_latency:.2f} ms, "
                    f"min {self.latency_min_ms:.2f} ms, "
                    f"max {self.latency_max_ms:.2f} ms"
                )

            print(
                "[METRICS] "
                f"rx {self.rx_count} ({rx_delta / elapsed:.1f}/s, {rx_byte_delta / elapsed:.0f} B/s) | "
                f"tx {self.tx_count} ({tx_delta / elapsed:.1f}/s, {tx_byte_delta / elapsed:.0f} B/s) | "
                f"malformed {self.malformed_count} | tx_dropped {self.tx_drop_count} | "
                f"latency {latency_text}"
            )

            self._last_report = now
            self._last_rx_count = self.rx_count
            self._last_tx_count = self.tx_count
            self._last_rx_bytes = self.rx_bytes
            self._last_tx_bytes = self.tx_bytes


def _extract_latency_ms(payload: bytes, topic_def, config=None) -> float | None:
    custom_ids = topic_def.get("custom_ids", [])
    if not custom_ids or custom_ids[-1] != "clock" or len(payload) < 8:
        return None

    byteorder = "little" if _struct_endianness(topic_def, config) == "<" else "big"
    sent_ns = int.from_bytes(payload[-8:], byteorder=byteorder, signed=False)
    if sent_ns <= 0:
        return None
    return (time.time_ns() - sent_ns) / 1_000_000.0

def main():
    parser = argparse.ArgumentParser(description="Mock PLC Client for testing MQTT data exchange")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    parser.add_argument("--exchange-config", help="Optional JSON file overriding MQTT/TCP exchange settings")
    parser.add_argument("--send-interval", type=float, default=2.0, help="Seconds between output command publishes")
    parser.add_argument("--metrics-interval", type=float, default=5.0, help="Seconds between performance reports")
    parser.add_argument("--dump-bytes", action="store_true", help="Print each received payload byte")
    args = parser.parse_args()

    # Load configuration to get MQTT settings and use as a state container
    config = load_config(args.config)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)

    transport = config.mqttSettings.transport
    broker_url = config.mqttSettings.brokerUrl
    broker_port = config.mqttSettings.port
    prefix = config.mqttSettings.topicPrefix or ""
    metrics = PerformanceMetrics(report_interval=args.metrics_interval)

    # Reconstruct the mappings that the simulator uses
    # Simulator publishes to /plcsim/input (from PLC's perspective, this is input)
    input_topic = f"{prefix}input".replace("//", "/")
    input_ids = PUBLISH_TOPICS["input"]["custom_ids"]
    input_mapping = custom_component_mapping(config, input_ids)
    input_msg = ComponentMessage(input_mapping, endianness=_struct_endianness(PUBLISH_TOPICS["input"], config))

    # Simulator subscribes to /plcsim/output (from PLC's perspective, this is output)
    output_topic = f"{prefix}output".replace("//", "/")
    output_ids = SUBSCRIBE_TOPICS["output"]["custom_ids"]
    output_mapping = custom_component_mapping(config, output_ids)
    output_msg = ComponentMessage(output_mapping, endianness=_struct_endianness(SUBSCRIBE_TOPICS["output"], config))

    mqtt = None
    tcp = None
    lock = threading.Lock()
    lock2 = Lock()
    botton_array = Array("c",[0 for i in range(9)],lock=lock2)

    def handle_input(raw):
        payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
        # if tcp:
            # remove first 8 bytes (magic + payload length)
            # if len(payload) < 8:
                # metrics.record_malformed()
                # print(
                    # f"[<- RECV {input_topic}] Ignoring payload: "
                    # f"expected at least 4 bytes, got {len(payload)}"
                # )
                # return
            # payload = payload[8:]

        if args.dump_bytes:
            print_byte_by_byte(payload)
        if len(payload) != input_msg.payload_size:
            metrics.record_malformed()
            print(
                f"[<- RECV {input_topic}] Ignoring payload: "
                f"expected {input_msg.payload_size} bytes, got {len(payload)}"
            )
            return

        metrics.record_rx(len(payload), _extract_latency_ms(payload, PUBLISH_TOPICS["input"], config))
        with lock:
            input_msg.deserialize(payload, config)
            
            # Print state of some interesting components
            sensor_1 = next((s for s in config.sensors if s.id == 'sensor_1_2_1'), None)
            sensor_2 = next((s for s in config.sensors if s.id == 'sensor_1_3_1'), None)
            button_0 = next((b for b in config.buttons if b.id == 'button_1_1_1'), None)
            button_1 = next((b for b in config.buttons if b.id == 'button_1_1_2'), None)
            button_2 = next((b for b in config.buttons if b.id == 'button_1_1_3'), None)
            button_3 = next((b for b in config.buttons if b.id == 'button_1_2_1'), None)
            button_4 = next((b for b in config.buttons if b.id == 'button_1_2_2'), None)
            button_5 = next((b for b in config.buttons if b.id == 'button_1_2_3'), None)
            button_6 = next((b for b in config.buttons if b.id == 'button_1_3_1'), None)
            button_7 = next((b for b in config.buttons if b.id == 'button_1_3_2'), None)
            button_8 = next((b for b in config.buttons if b.id == 'button_1_3_3'), None)
            belt_1 = next((b for b in config.belts if b.id == 'belt_1_1_1'), None)
            belt_2 = next((b for b in config.belts if b.id == 'belt_1_2_1'), None)
            belt_3 = next((b for b in config.belts if b.id == 'belt_1_3_1'), None)
            
            s1_state = sensor_1.sensor_states if sensor_1 else 'N/A'
            s2_state = sensor_2.sensor_states if sensor_2 else 'N/A'
            btn_0_state = button_0.sensor_states if button_0 else 'N/A'
            btn_1_state = button_1.sensor_states if button_1 else 'N/A'
            btn_2_state = button_2.sensor_states if button_2 else 'N/A'
            btn_3_state = button_3.sensor_states if button_3 else 'N/A'
            btn_4_state = button_4.sensor_states if button_4 else 'N/A'
            btn_5_state = button_5.sensor_states if button_5 else 'N/A'
            btn_6_state = button_6.sensor_states if button_6 else 'N/A'
            btn_7_state = button_7.sensor_states if button_7 else 'N/A'
            btn_8_state = button_8.sensor_states if button_8 else 'N/A'
            b1_speed = belt_1.speed if belt_1 else 'N/A'
            b1_text = f"{b1_speed:.2f}" if isinstance(b1_speed, (int, float)) else str(b1_speed)
            
            print(f"[<- RECV {input_topic}] S1: {s1_state} | S2: {s2_state} | BTN:  {btn_0_state} | {btn_1_state} | {btn_2_state} | {btn_3_state} | {btn_4_state} | {btn_5_state} | {btn_6_state} | {btn_7_state} | {btn_8_state} | B1 Spd: {b1_text}")

    print(f"Starting Mock PLC Client...")
    if transport == "mqtt":
        mqtt = FlexibleProtocolMQTTManager(broker_url=broker_url, port=broker_port)
        mqtt.register_callback(input_topic, handle_input)
        print(f"Connecting to MQTT {broker_url}:{broker_port}")
        mqtt.start()
    elif transport == "tcp":
        tcp_host = config.mqttSettings.tcpServerHost
        tcp_port = config.mqttSettings.tcpPort
        tcp = TcpFramedClient(
            tcp_host,
            tcp_port,
            handle_input,
            length_endianness=config.mqttSettings.endianness,
        )
        print(f"Connecting to TCP {tcp_host}:{tcp_port}")
        tcp.start()
    else:
        print(f"[ERROR] Unsupported exchange transport: {transport}")
        return

    # Wait for connection
    deadline = time.time() + 5.0
    def is_connected():
        if mqtt is not None:
            return mqtt.client.is_connected()
        if tcp is not None:
            return tcp.connected
        return False

    while not is_connected() and time.time() < deadline:
        time.sleep(0.1)

    if not is_connected():
        print("[ERROR] Could not connect. Exiting.")
        if mqtt is not None:
            mqtt.stop()
        if tcp is not None:
            tcp.stop()
        return

    print("Connected! Press Ctrl+C to exit.")
    print(f"Listening on: {input_topic}")
    print(f"Publishing to: {output_topic}")

    try:
        counter = 0
        while True:
            # Every ~2 seconds, toggle belt 1_1_1 speed and publish it back
            with lock:
                belt_1_1_1 = next((b for b in config.belts if b.id == 'belt_1_1_1'), None)
                if belt_1_1_1:
                    
                    belt_1_1_1.speed = 2.0 if (counter % 2 == 0) else False
                    speed_val = belt_1_1_1.speed
                else:
                    speed_val = 0.0
                led_1_1_1 = next((b for b in config.leds if b.id == 'led_1_1_1'), None)
                led_1_1_2 = next((b for b in config.leds if b.id == 'led_1_1_2'), None)
                if led_1_1_1:
                    led_1_1_1.isOn = True if (counter % 2 == 0) else False
                if led_1_1_2:
                    led_1_1_2.isOn = False if (counter % 2 == 0) else True
                payload = output_msg.serialize(config)

            if mqtt is not None:
                mqtt.publish(output_topic, payload)
                sent = mqtt.client.is_connected()
            elif tcp is not None:
                # add payload length in first word
                # MAGIC_MARKER = (0x12345678).to_bytes(4, byteorder="big")
                # length_bytes = len(payload).to_bytes(4, byteorder="big")

                # payload = MAGIC_MARKER + length_bytes + payload
                # print(f"[TCPSync] Payload length: {length_bytes}")

                sent = tcp.send(payload)
            else:
                sent = False

            metrics.record_tx(len(payload), sent)
            print(f"[-> SEND {output_topic}] Sent command -> B1 Spd: {speed_val:.2f}" if sent else f"[-> SEND {output_topic}] Dropped command: not connected")
            
            counter += 1
            metrics.maybe_report()
            time.sleep(args.send_interval)
            
    except KeyboardInterrupt:
        print("\nStopping Mock PLC...")
    finally:
        metrics.maybe_report()
        if mqtt is not None:
            mqtt.stop()
        if tcp is not None:
            tcp.stop()

if __name__ == "__main__":
    main()
