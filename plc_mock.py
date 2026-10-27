"""
plc_mock.py - Mock PLC client for testing the simulator's MQTT/TCP data
exchange. Exercises the structured input/output ComponentMessage channels
and two of the data-visualization metrics layouts (MetricLayout1 and
MetricLayout3 in mqtt_data_exchange.py) on the SAME connection - the
values sent are this client's own measured values (the send-loop period,
the last output-send duration, the round-trip latency computed from the
simulator's clock field, the accumulated send count, the belt speed and
LED states just set), not synthetic demo data. See _build_layout1_sample()
and _build_layout3_sample() below.

Over TCP, every frame this client sends carries its own --node-id and an
explicit FrameType (IO_MAPPING for the output channel, METRIC_LAYOUT_1 /
METRIC_LAYOUT_3 for the two metrics layouts, see METRICS_CHANNELS) in the
header - see FrameType/ParsedFrame in tcp_framed_transport.py. Over MQTT
there is no such header, so each layout is disambiguated by its own topic
(metrics vs metric3); node id / frame type only apply to the TCP path.
"""
import time
import argparse
import threading
from dataclasses import dataclass
from config_parser import load_config
from mqtt_manager import FlexibleProtocolMQTTManager
from mqtt_message_binding import ComponentMessage, MetricsMessage, belt_feedback_mapping, custom_component_mapping
from mqtt_data_exchange import (
    METRICS_CHANNELS,
    MetricLayout1,
    MetricLayout1Perf,
    MetricLayout1Results,
    MetricLayout3,
    MetricLayout3Test123,
    PUBLISH_TOPICS,
    SUBSCRIBE_TOPICS,
    apply_exchange_config,
)
from struct_utils import print_byte_by_byte
from tcp_framed_transport import FrameType, ParsedFrame, TcpFramedClient

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


def _build_layout1_sample(
    loop_period_ms: float,
    rtt_latency_ms: float,
    send_elapsed_us: float,
    tx_count: int,
    sent_ok: bool,
    belt_speed: float,
) -> MetricLayout1:
    """This client's own measured values as a MetricLayout1 instance (see
    metric_layout.py / mqtt_data_exchange.py) - MetricsMessage.serialize()
    packs it straight to the wire, no hand-keyed (group, name) dict. Field
    names say what each number actually is: loop_period_ms is the interval
    between send-loop iterations (~= --send-interval), rtt_latency_ms is
    the round-trip latency computed in handle_input(), send_elapsed_us is
    how long building + sending the last output payload took."""
    return MetricLayout1(
        perf=MetricLayout1Perf(
            scan_time_ms=loop_period_ms,
            cycle_time_ms=rtt_latency_ms,
            fn_elapsed_us=send_elapsed_us,
            sample_count=tx_count,
        ),
        results=MetricLayout1Results(
            last_result_code=0 if sent_ok else 1,
            # Only the first component is meaningful (the belt's scalar
            # speed); this client doesn't track a separate Y component.
            belt_1_velocity=(belt_speed, 0.0),
        ),
    )


def _build_layout3_sample(led_on: bool, led2_on: bool, value: float) -> MetricLayout3:
    """A second metric layout (frame_type METRIC_LAYOUT_3, topic
    /plcsim/metric3) this client also reports, to exercise a node sending
    more than one layout over the same connection. serialize() takes the
    top-level MetricLayout3 instance - not the MetricLayout3Test123 group
    on its own."""
    return MetricLayout3(
        test123=MetricLayout3Test123(led=led_on, led2=led2_on, v=value),
    )


def main():
    parser = argparse.ArgumentParser(description="Mock PLC Client for testing MQTT data exchange")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    parser.add_argument("--exchange-config", help="Optional JSON file overriding MQTT/TCP exchange settings")
    parser.add_argument("--send-interval", type=float, default=2.0, help="Seconds between output command publishes")
    parser.add_argument("--metrics-interval", type=float, default=5.0, help="Seconds between performance reports")
    parser.add_argument("--dump-bytes", action="store_true", help="Print each received payload byte")
    parser.add_argument(
        "--node-id", type=int, default=0,
        help="This client's identity in the TCP frame header's Node ID byte (0-255, TCP transport only)",
    )
    parser.add_argument("--print-feedback", action="store_true",
                        help="Print every received belt feedback frame (needs mqttSettings.beltFeedback on the simulator)")
    args = parser.parse_args()
    if not (0 <= args.node_id <= 0xFF):
        parser.error("--node-id must be between 0 and 255")

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
    input_mapping = custom_component_mapping(config, input_ids, direction="out")
    input_msg = ComponentMessage(input_mapping, endianness=_struct_endianness(PUBLISH_TOPICS["input"], config))

    # Simulator subscribes to /plcsim/output (from PLC's perspective, this is output)
    output_topic = f"{prefix}output".replace("//", "/")
    output_ids = SUBSCRIBE_TOPICS["output"]["custom_ids"]
    output_mapping = custom_component_mapping(config, output_ids, direction="in")
    output_msg = ComponentMessage(output_mapping, endianness=_struct_endianness(SUBSCRIBE_TOPICS["output"], config))

    # Metrics channels - same connection as the output channel above.
    # from_layout() derives each wire format from its layout dataclass
    # (mqtt_data_exchange.py). This client reports its OWN measured values
    # through them (see _build_layout1_sample() / _build_layout3_sample()
    # and the send loop below), not synthetic demo data.
    metrics_topic_suffix = METRICS_CHANNELS[FrameType.METRIC_LAYOUT_1]["topic_suffix"]
    metrics_topic = f"{prefix}{metrics_topic_suffix}".replace("//", "/")
    metrics_msg = MetricsMessage.from_layout(MetricLayout1, endianness=_struct_endianness({}, config))

    # A second layout this client also reports (METRIC_LAYOUT_3) - its own
    # frame_type over TCP, its own topic over MQTT (metric3, not metrics).
    metric3_topic_suffix = METRICS_CHANNELS[FrameType.METRIC_LAYOUT_3]["topic_suffix"]
    metric3_topic = f"{prefix}{metric3_topic_suffix}".replace("//", "/")
    metric3_msg = MetricsMessage.from_layout(MetricLayout3, endianness=_struct_endianness({}, config))

    # Belt drive feedback (actualSpeed + encoderCount per belt) - only sent
    # by a simulator running with mqttSettings.beltFeedback enabled.
    feedback_topic = f"{prefix}belt_feedback".replace("//", "/")
    feedback_msg = ComponentMessage(belt_feedback_mapping(config), endianness=_struct_endianness({}, config))

    mqtt = None
    tcp = None
    lock = threading.Lock()
    # Written by handle_input() (nonlocal) whenever a received frame
    # carries a valid clock timestamp; read by the send loop below as the
    # most recently observed round-trip latency.
    last_latency_ms = 0.0

    def handle_input(raw):
        nonlocal last_latency_ms
        # TcpFramedClient hands back a ParsedFrame (node_id, frame_type,
        # payload) - see tcp_framed_transport.py; MQTT has no such header,
        # so mqtt.register_callback below only ever calls this with raw
        # payload bytes (or str) instead. Either way this only ever
        # processes the "input" topic, which is always IO_MAPPING - a
        # differently-typed TCP frame here would mean something is
        # misconfigured (e.g. pointed at the wrong port), so it's dropped
        # rather than fed to input_msg.deserialize() as if it matched.
        if isinstance(raw, ParsedFrame) and raw.frame_type == FrameType.BELT_FEEDBACK:
            handle_belt_feedback(raw.payload)
            return
        if isinstance(raw, ParsedFrame):
            if raw.frame_type != FrameType.IO_MAPPING:
                print(f"[<- RECV {input_topic}] Ignoring frame_type={raw.frame_type} from node {raw.node_id} "
                      f"(expected IO_MAPPING={FrameType.IO_MAPPING}).")
                return
            payload = raw.payload
        else:
            payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()

        if args.dump_bytes:
            print_byte_by_byte(payload)
        if len(payload) != input_msg.payload_size:
            metrics.record_malformed()
            print(
                f"[<- RECV {input_topic}] Ignoring payload: "
                f"expected {input_msg.payload_size} bytes, got {len(payload)}"
            )
            return

        latency_ms = _extract_latency_ms(payload, PUBLISH_TOPICS["input"], config)
        if latency_ms is not None:
            last_latency_ms = latency_ms
        metrics.record_rx(len(payload), latency_ms)
        with lock:
            input_msg.deserialize(payload, config)

            # Print state of some interesting components.
            def _state(collection, comp_id):
                comp = next((c for c in collection if c.id == comp_id), None)
                return comp.sensor_states if comp else 'N/A'

            sensor_ids = ['sensor_1_2_1', 'sensor_1_3_1']
            button_ids = [
                'button_1_1_1', 'button_1_1_2', 'button_1_1_3',
                'button_1_2_1', 'button_1_2_2', 'button_1_2_3',
                'button_1_3_1', 'button_1_3_2', 'button_1_3_3',
            ]
            sensor_text = " | ".join(_state(config.sensors, sid) for sid in sensor_ids)
            button_text = " | ".join(str(_state(config.buttons, bid)) for bid in button_ids)

            belt_1 = next((b for b in config.belts if b.id == 'belt_1_1_1'), None)
            b1_speed = belt_1.speed if belt_1 else 'N/A'
            b1_text = f"{b1_speed:.2f}" if isinstance(b1_speed, (int, float)) else str(b1_speed)

            print(f"[<- RECV {input_topic}] S: {sensor_text} | BTN: {button_text} | B1 Spd: {b1_text}")

    def handle_belt_feedback(raw):
        payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
        if len(payload) != feedback_msg.payload_size:
            metrics.record_malformed()
            print(f"[<- RECV {feedback_topic}] Ignoring payload: "
                  f"expected {feedback_msg.payload_size} bytes, got {len(payload)}")
            return
        metrics.record_rx(len(payload))
        with lock:
            feedback_msg.deserialize(payload, config)
            belt_1 = next((b for b in config.belts if b.id == 'belt_1_1_1'), None)
            if belt_1 is not None and args.print_feedback:
                print(f"[<- RECV {feedback_topic}] belt_1_1_1 actual {belt_1.actualSpeed:.3f} m/s "
                      f"(cmd {belt_1.speed:.2f}) encoder {belt_1.encoderCount}")

    print(f"Starting Mock PLC Client...")
    if transport == "mqtt":
        mqtt = FlexibleProtocolMQTTManager(broker_url=broker_url, port=broker_port)
        mqtt.register_callback(input_topic, handle_input)
        mqtt.register_callback(feedback_topic, handle_belt_feedback)
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
    print(f"Reporting metrics on: {metrics_topic} (layout 1), {metric3_topic} (layout 3)")

    try:
        counter = 0
        last_loop_start = time.perf_counter()
        while True:
            loop_start = time.perf_counter()
            loop_period_ms = (loop_start - last_loop_start) * 1000.0
            last_loop_start = loop_start

            # Every ~2 seconds, toggle belt 1_1_1 speed and publish it back
            send_start = time.perf_counter()
            with lock:
                belt_1_1_1 = next((b for b in config.belts if b.id == 'belt_1_1_1'), None)
                if belt_1_1_1:

                    belt_1_1_1.speed = 2.0 if (counter % 2 == 0) else 0.0
                    speed_val = belt_1_1_1.speed
                else:
                    speed_val = 0.0
                led_1_1_1 = next((b for b in config.leds if b.id == 'led_1_1_1'), None)
                led_1_1_2 = next((b for b in config.leds if b.id == 'led_1_1_2'), None)
                if led_1_1_1:
                    led_1_1_1.isOn = True if (counter % 2 == 0) else False
                if led_1_1_2:
                    led_1_1_2.isOn = False if (counter % 2 == 0) else True
                # Captured under the lock - the send loop reads these below,
                # off-lock, while the sim mutates config from another thread.
                led_1_on = bool(led_1_1_1.isOn) if led_1_1_1 else False
                led_2_on = bool(led_1_1_2.isOn) if led_1_1_2 else False
                payload = output_msg.serialize(config)

            if mqtt is not None:
                mqtt.publish(output_topic, payload)
                sent = mqtt.client.is_connected()
            elif tcp is not None:
                # TcpFramedClient.send() wraps the payload with the
                # magic-marker header/node-id/frame-type/length/trailer
                # itself (see TcpFrameCodec.pack() in
                # tcp_framed_transport.py) - no manual framing needed here.
                sent = tcp.send(payload, frame_type=FrameType.IO_MAPPING, node_id=args.node_id)
            else:
                sent = False
            send_elapsed_us = (time.perf_counter() - send_start) * 1_000_000.0

            metrics.record_tx(len(payload), sent)
            print(f"[-> SEND {output_topic}] Sent command -> B1 Spd: {speed_val:.2f}" if sent else f"[-> SEND {output_topic}] Dropped command: not connected")

            # Report this client's own measured values through the metrics
            # channel, over the SAME connection as the output payload above.
            # serialize() takes the MetricLayout1 instance directly.
            metrics_payload = metrics_msg.serialize(
                _build_layout1_sample(
                    loop_period_ms=loop_period_ms,
                    rtt_latency_ms=last_latency_ms,
                    send_elapsed_us=send_elapsed_us,
                    tx_count=metrics.tx_count,
                    sent_ok=sent,
                    belt_speed=speed_val,
                )
            )
            if mqtt is not None:
                mqtt.publish(metrics_topic, metrics_payload)
            elif tcp is not None:
                tcp.send(metrics_payload, frame_type=FrameType.METRIC_LAYOUT_1, node_id=args.node_id)

            # The second layout (METRIC_LAYOUT_3) - its own topic over MQTT,
            # its own frame_type over TCP.
            metric3_payload = metric3_msg.serialize(
                _build_layout3_sample(led_on=led_1_on, led2_on=led_2_on, value=speed_val)
            )
            if mqtt is not None:
                mqtt.publish(metric3_topic, metric3_payload)
            elif tcp is not None:
                tcp.send(metric3_payload, frame_type=FrameType.METRIC_LAYOUT_3, node_id=args.node_id)

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
