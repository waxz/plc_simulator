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

Control: every --scan seconds the PLC program in plc_logic.py runs on the
latest inputs and the resulting belt speeds and LED states are sent back -
LineController for a layout with several lines (per-line Start/Stop/Reset,
feeders holding parcels at the merge), else DeliveryController (one line).
It works with any layout: the I/O channels come from
mqtt_data_exchange.io_topics(), the same function the simulator uses, so a
layout designed in the editor needs no channel setup.

Monitoring: with --bridge HOST:PORT it also sends, over a second TCP
connection, what a real PLC would report to the Foxglove middleware
(plc_foxglove_bridge.py): its output image (IO_MAPPING), the input image it
works on (INPUT_IMAGE), its metrics frames and a LAYOUT_CHECK. Without a
simulator it keeps scanning (inputs stay as they are) as long as the bridge
is there.

    python main.py my_layout.json --exchange-config tcp_exchange_config.json
    python plc_foxglove_bridge.py my_layout.json
    python plc_mock.py my_layout.json --exchange-config tcp_exchange_config.json --bridge 127.0.0.1:5002
"""
import time
import argparse
import threading
from dataclasses import dataclass
from config_parser import load_config
from mqtt_manager import FlexibleProtocolMQTTManager
from mqtt_message_binding import ComponentMessage, MetricsMessage, custom_component_mapping
from mqtt_data_exchange import (
    METRICS_CHANNELS,
    MetricLayout1,
    MetricLayout1Perf,
    MetricLayout1Results,
    MetricLayout3,
    MetricLayout3Test123,
    apply_exchange_config,
    io_topics,
)
from plc_logic import make_controller
import io_layout
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


class PlcImage:
    """A PLC's I/O frames for a layout - the same channels the simulator
    builds (io_topics): `input_msg` (simulator -> PLC: sensors, buttons,
    feedback, the clock) and `output_msg` (PLC -> simulator: belts' and
    devices' commands, lamps). `send(plc_config)` is the output frame's
    bytes from a PLC's copy of the layout; `receive(payload, plc_config)`
    reads an input frame into it. device_check.py drives the simulator
    through these bytes, exactly as this mock PLC does over TCP."""

    def __init__(self, config):
        publish_topics, subscribe_topics = io_topics(config)
        self.input_ids = publish_topics["input"]["custom_ids"]
        self.input_mapping = custom_component_mapping(config, self.input_ids, direction="out")
        self.input_msg = ComponentMessage(self.input_mapping,
                                          endianness=_struct_endianness(publish_topics["input"], config))
        self.output_ids = subscribe_topics["output"]["custom_ids"]
        self.output_mapping = custom_component_mapping(config, self.output_ids, direction="in")
        self.output_msg = ComponentMessage(self.output_mapping,
                                           endianness=_struct_endianness(subscribe_topics["output"], config))

    def send(self, plc_config) -> bytes:
        return self.output_msg.serialize(plc_config)

    def receive(self, payload: bytes, plc_config) -> None:
        self.input_msg.deserialize(payload, plc_config)


def make_program(config, program: str = "auto", jam_time: float = 5.0, autostart: bool = False,
                 align: str = "head", land_offsets=None, max_length: float = 1.2,
                 induct_speed: float = 1.0):
    """The PLC program for a layout (--program): plc_merge's MergeTrainController
    for a merge line whose inductions take slots, else plc_logic's."""
    shows_slots = any(getattr(t, "role", "load") in ("slot", "sync") for t in getattr(config, "trackedLoads", []))
    if program == "merge-train" or (program == "auto" and shows_slots):
        from plc_merge import MergeTrainController
        return MergeTrainController(config, jam_time=jam_time, autostart=autostart, align=align,
                                    land_offsets=land_offsets, max_length=max_length,
                                    induct_speed=induct_speed)
    return make_controller(config, jam_time=jam_time, autostart=autostart)


def main():
    parser = argparse.ArgumentParser(description="Mock PLC Client for testing MQTT data exchange")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    parser.add_argument("--exchange-config", help="Optional JSON file overriding MQTT/TCP exchange settings")
    parser.add_argument("--send-interval", type=float, default=2.0,
                        help="Seconds between metrics reports and status lines")
    parser.add_argument("--scan", type=float, default=0.05,
                        help="PLC scan time: seconds between control steps / output commands (default 0.05)")
    parser.add_argument("--jam-time", type=float, default=5.0,
                        help="Seconds a sensor may stay blocked before it is a jam (belts stop)")
    parser.add_argument("--autostart", action="store_true", help="Run the belts without pressing Start")
    parser.add_argument("--align", choices=("head", "center"), default="head",
                        help="merge-train: a load's front to its car's front target (head: it asks as the gate "
                             "sees it), or centre to centre (center: it asks once its length is measured)")
    parser.add_argument("--induct-speed", type=float, default=1.0, metavar="M/S",
                        help="merge-train: the metering belts (human-assisted induction) and a section while "
                             "its load crosses the gate (default 1 m/s); past the gate the section speeds up")
    parser.add_argument("--max-length", type=float, default=1.2, metavar="METRES",
                        help="merge-train: a load longer than this (the gate still blocked) gets no car and is "
                             "held until it is taken away (default 1.2 m, a car)")
    parser.add_argument("--land-offset", action="append", default=[], metavar="FEED=METRES",
                        help="merge-train: how far downstream of its port a feed's loads land (commissioned: "
                             "python plc_merge.py LAYOUT --commission 120 prints them); repeat per feed")
    parser.add_argument("--program", choices=("auto", "lines", "merge-train"), default="auto",
                        help="The PLC program: merge-train (plc_merge.py: a merge line's inductions by a slot "
                             "train), lines (plc_logic.py), auto: merge-train when the layout shows slots "
                             "(a trackedLoads row of role slot or sync), else lines")
    parser.add_argument("--show-layout", action="store_true",
                        help="Print the input/output byte layouts (and the matching PLC STRUCTs)")
    parser.add_argument("--metrics-interval", type=float, default=5.0, help="Seconds between performance reports")
    parser.add_argument("--dump-bytes", action="store_true", help="Print each received payload byte")
    parser.add_argument(
        "--node-id", type=int, default=0,
        help="This client's identity in the TCP frame header's Node ID byte (0-255, TCP transport only)",
    )
    parser.add_argument("--print-feedback", action="store_true",
                        help="Print the drive feedback of every belt that reports it (Belt.feedback), each input frame")
    parser.add_argument("--bridge", metavar="HOST:PORT",
                        help="Also send this PLC's I/O images and metrics to plc_foxglove_bridge.py (e.g. 127.0.0.1:5002)")
    parser.add_argument("--bridge-interval", type=float, default=0.1,
                        help="Seconds between I/O image frames to the bridge (default 0.1)")
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

    # Reconstruct the mappings that the simulator uses (io_topics: the same
    # channels for this layout as MQTTSyncEngine builds).
    publish_topics, subscribe_topics = io_topics(config)
    image = PlcImage(config)
    # Simulator publishes to /plcsim/input (from PLC's perspective, this is input)
    input_topic = f"{prefix}input".replace("//", "/")
    input_ids, input_mapping, input_msg = image.input_ids, image.input_mapping, image.input_msg
    # Simulator subscribes to /plcsim/output (from PLC's perspective, this is output)
    output_topic = f"{prefix}output".replace("//", "/")
    output_ids, output_mapping, output_msg = image.output_ids, image.output_mapping, image.output_msg

    # The PLC program: reads buttons/sensors from `config` (filled by
    # input_msg.deserialize), writes belt speeds/LEDs into it (sent by
    # output_msg.serialize).
    from plc_merge import parse_offsets
    controller = make_program(config, args.program, jam_time=args.jam_time, autostart=args.autostart,
                              align=args.align, land_offsets=parse_offsets(args.land_offset),
                              max_length=args.max_length, induct_speed=args.induct_speed)
    print(f"[PLC] Inputs: {', '.join(i for i in input_ids if i != 'clock')}")
    print(f"[PLC] Outputs: {', '.join(output_ids)}")
    print(f"[PLC] Program: {type(controller).__name__}")
    print(f"[PLC] Roles: " + ", ".join(f"{k}={v}" for k, v in controller.roles().items() if v))
    print(f"[PLC] Input  frame: {io_layout.describe(input_mapping)}")
    print(f"[PLC] Output frame: {io_layout.describe(output_mapping)}")
    if args.show_layout:
        for title, mapping, type_name in (("input  (simulator -> PLC)", input_mapping, "Sim_Input"),
                                          ("output (PLC -> simulator)", output_mapping, "Sim_Output")):
            print(io_layout.format_layout(mapping, title))
            print(io_layout.st_struct(type_name, mapping))
    endian = _struct_endianness({}, config)
    mine = (io_layout.layout_fingerprint(input_mapping), input_msg.payload_size,
            io_layout.layout_fingerprint(output_mapping), output_msg.payload_size)
    layout_verdict = None  # set when the simulator answers the LAYOUT_CHECK
    size_warned_at = -1e9

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

    mqtt = None
    tcp = None
    lock = threading.Lock()
    # Written by handle_input() (nonlocal) whenever a received frame
    # carries a valid clock timestamp; read by the send loop below as the
    # most recently observed round-trip latency.
    last_latency_ms = 0.0

    def handle_layout_check(payload):
        nonlocal layout_verdict
        reply = io_layout.unpack_layout_check(payload, endian)
        if reply is None:
            print("[PLC] Ignoring a malformed LAYOUT_CHECK reply.")
            return
        problems = io_layout.compare_layout_check(mine, reply[:4])
        layout_verdict = not problems and reply[4] == io_layout.CHECK_MATCH
        if layout_verdict:
            print(f"[PLC] I/O layout check: OK - input {mine[1]} bytes {mine[0]:08X}, "
                  f"output {mine[3]} bytes {mine[2]:08X} on both sides")
        else:
            print("[PLC] I/O layout MISMATCH with the simulator: " + "; ".join(problems or ["simulator says so"])
                  + ". Use the same layout file on both sides (python io_layout.py <layout> --st).")

    def handle_input(raw):
        nonlocal last_latency_ms, size_warned_at
        if isinstance(raw, ParsedFrame) and raw.frame_type == FrameType.LAYOUT_CHECK:
            handle_layout_check(raw.payload)
            return
        # TcpFramedClient hands back a ParsedFrame (node_id, frame_type,
        # payload) - see tcp_framed_transport.py; MQTT has no such header,
        # so mqtt.register_callback below only ever calls this with raw
        # payload bytes (or str) instead. Either way this only ever
        # processes the "input" topic, which is always IO_MAPPING - a
        # differently-typed TCP frame here would mean something is
        # misconfigured (e.g. pointed at the wrong port), so it's dropped
        # rather than fed to input_msg.deserialize() as if it matched.
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
            now = time.monotonic()
            if now - size_warned_at >= 5.0:  # not 100 lines a second
                size_warned_at = now
                print(f"[<- RECV {input_topic}] Ignoring frames: expected {input_msg.payload_size} bytes, "
                      f"got {len(payload)} - the simulator runs a different I/O layout.")
            return

        latency_ms = _extract_latency_ms(payload, publish_topics["input"], config)
        if latency_ms is not None:
            last_latency_ms = latency_ms
        metrics.record_rx(len(payload), latency_ms)
        with lock:
            # Buttons/sensors land in `config`; the control loop below acts on them.
            # (So does the drive feedback of every belt with `feedback` on.)
            input_msg.deserialize(payload, config)
            if args.print_feedback:
                for belt in config.belts:
                    if belt.feedback:
                        print(f"[<- RECV {input_topic}] {belt.id} actual {belt.actualSpeed:.3f} m/s "
                              f"(cmd {belt.speed:.2f}) encoder {belt.encoderCount}"
                              f"{' FAULT' if belt.fault else ''}")

    bridge = None
    if args.bridge:
        bridge_host, _, bridge_port = args.bridge.rpartition(":")
        if not bridge_host or not bridge_port.isdigit():
            parser.error("--bridge must be HOST:PORT, e.g. 127.0.0.1:5002")

        def handle_bridge_frame(frame):
            if frame.frame_type == FrameType.LAYOUT_CHECK:
                reply = io_layout.unpack_layout_check(frame.payload, endian)
                problems = io_layout.compare_layout_check(mine, reply[:4]) if reply else ["bad reply"]
                print("[PLC] Bridge layout check: " + ("OK" if not problems else "MISMATCH - " + "; ".join(problems)))

        bridge = TcpFramedClient(bridge_host, int(bridge_port), handle_bridge_frame,
                                 length_endianness=config.mqttSettings.endianness)
        bridge.start()
        print(f"[PLC] Reporting to the Foxglove bridge at {bridge_host}:{bridge_port}")
    bridge_checked = False
    next_bridge_send = 0.0

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

    if not is_connected() and bridge is None:
        print("[ERROR] Could not connect. Exiting.")
        if mqtt is not None:
            mqtt.stop()
        if tcp is not None:
            tcp.stop()
        return
    if not is_connected():
        print("[PLC] No simulator yet - scanning anyway and reporting to the bridge (still trying to connect).")
    else:
        print("Connected! Press Ctrl+C to exit.")
    sim_checked = False   # LAYOUT_CHECK sent on this simulator connection
    print(f"Listening on: {input_topic}")
    print(f"Publishing to: {output_topic}")
    print(f"Reporting metrics on: {metrics_topic} (layout 1), {metric3_topic} (layout 3)")

    try:
        last_loop_start = time.perf_counter()
        next_report = time.monotonic() + args.send_interval
        was_sent = True
        while True:
            loop_start = time.perf_counter()
            loop_period_ms = (loop_start - last_loop_start) * 1000.0
            last_loop_start = loop_start

            # One PLC scan: logic on the latest inputs, then send the outputs.
            send_start = time.perf_counter()
            with lock:
                events = controller.step()
                first_belt = config.belts[0] if config.belts else None
                speed_val = float(first_belt.speed) if first_belt is not None else 0.0
                # Captured under the lock - the receive thread mutates config.
                led_1_on = bool(config.leds[0].isOn) if len(config.leds) > 0 else False
                led_2_on = bool(config.leds[1].isOn) if len(config.leds) > 1 else False
                payload = output_msg.serialize(config)
                status = controller.summary()
                # The I/O image this scan worked on, for the bridge (the
                # simulator's input frame layout, as the PLC last saw it).
                input_image = input_msg.serialize(config) if bridge is not None else b""

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

            if tcp is not None and tcp.connected and not sim_checked:
                # Ask the simulator to compare I/O layouts (answered in handle_layout_check).
                sim_checked = tcp.send(io_layout.pack_layout_check(input_mapping, output_mapping, endianness=endian),
                                       frame_type=FrameType.LAYOUT_CHECK, node_id=args.node_id)
            elif tcp is not None and not tcp.connected:
                sim_checked = False

            if bridge is not None and bridge.connected:
                if not bridge_checked:
                    bridge.send(io_layout.pack_layout_check(input_mapping, output_mapping, endianness=endian),
                                frame_type=FrameType.LAYOUT_CHECK, node_id=args.node_id)
                    bridge_checked = True
                if time.monotonic() >= next_bridge_send:
                    next_bridge_send = time.monotonic() + args.bridge_interval
                    bridge.send(payload, frame_type=FrameType.IO_MAPPING, node_id=args.node_id)
                    bridge.send(input_image, frame_type=FrameType.INPUT_IMAGE, node_id=args.node_id)
            elif bridge is not None:
                bridge_checked = False   # check again after a reconnect

            for event in events:
                print(f"[PLC] {event}")
            if sent != was_sent:
                print(f"[-> SEND {output_topic}] " + ("Sending commands again." if sent else
                                                       "Dropping commands: not connected"))
                was_sent = sent

            if time.monotonic() >= next_report:
                next_report += args.send_interval
                print(f"[PLC] {status}")
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
                if bridge is not None:
                    bridge.send(metrics_payload, frame_type=FrameType.METRIC_LAYOUT_1, node_id=args.node_id)

                # The second layout (METRIC_LAYOUT_3) - its own topic over MQTT,
                # its own frame_type over TCP.
                metric3_payload = metric3_msg.serialize(
                    _build_layout3_sample(led_on=led_1_on, led2_on=led_2_on, value=speed_val)
                )
                if mqtt is not None:
                    mqtt.publish(metric3_topic, metric3_payload)
                elif tcp is not None:
                    tcp.send(metric3_payload, frame_type=FrameType.METRIC_LAYOUT_3, node_id=args.node_id)
                if bridge is not None:
                    bridge.send(metric3_payload, frame_type=FrameType.METRIC_LAYOUT_3, node_id=args.node_id)

            metrics.maybe_report()
            time.sleep(max(0.0, args.scan - (time.perf_counter() - loop_start)))

    except KeyboardInterrupt:
        print("\nStopping Mock PLC...")
    finally:
        metrics.maybe_report()
        if mqtt is not None:
            mqtt.stop()
        if tcp is not None:
            tcp.stop()
        if bridge is not None:
            bridge.stop()

if __name__ == "__main__":
    main()
