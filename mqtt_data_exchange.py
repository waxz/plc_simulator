"""
mqtt_data_exchange.py - MQTT data exchange engine for the conveyor simulator.

Manages publish (simulator -> PLC) and subscribe (PLC -> simulator) channels
using ComponentMessage for binary serialization/deserialization. Each channel
has its own mapping, topic, and rate-limited interval.

Usage in main.py:
    config_lock = threading.Lock()  # shared with PhysicsEngine(config, lock=config_lock)
    metrics_store = MetricsStore()  # read by the web UI's Live Data tab
    mqtt = MQTTSyncEngine(config, lock=config_lock, metrics_store=metrics_store)
    # in game loop:
    mqtt.sync()
    # on shutdown:
    mqtt.stop()
"""

from paho.mqtt import packettypes
import json
import struct
import time
import threading
from dataclasses import dataclass
from typing import Dict, List, Any, Optional, Tuple

from mqtt_manager import FlexibleProtocolMQTTManager
from config_parser import SimulatorConfig
from metrics_store import MetricsStore
from mqtt_message_binding import (
    ByteType,
    ComponentMessage,
    MetricsMessage,
    belt_feedback_mapping,
    custom_component_mapping,
    known_component_ids,
    verify_component_list,
)
import io_layout
from metric_layout import f32, u8, u16, u32, u64, bit, Vec2, Vec3
from channel_registry import load_channel_registry
from tcp_framed_transport import FrameType, ParsedFrame, TcpFramedServer


# ---------------------------------------------------------------------------
# Default topic configurations.
# Each entry maps a topic string to its interval (ms) and custom_ids list.
# These are used when mqttSettings.topics is empty or doesn't cover bindings.
# ---------------------------------------------------------------------------
PUBLISH_TOPICS: Dict[str, Dict[str, Any]] = {
    'input': {
        "interval": 10,
        "custom_ids": [
            'button_1_1_1', 'button_1_1_2', 'button_1_1_3', 'sensor_1_1_1', 'button_1_2_1', 'button_1_2_2',
            'button_1_2_3', 'sensor_1_2_1', 'button_1_3_1', 'button_1_3_2', 'button_1_3_3', 'sensor_1_3_1',
            'clock',
        ],
    },
}

SUBSCRIBE_TOPICS: Dict[str, Dict[str, Any]] = {
    'output': {
        "interval": 10,
        "custom_ids": [
            'led_1_1_1', 'led_1_1_2', 'led_1_1_3', 'belt_1_1_1', 'belt_1_1_2', 'belt_1_1_3',
            'belt_1_1_4', 'belt_1_1_5', 'led_1_2_1', 'led_1_2_2', 'led_1_2_3', 'belt_1_2_1',
            'belt_1_2_2', 'belt_1_2_3', 'belt_1_2_4', 'belt_1_2_5', 'led_1_3_1', 'led_1_3_2',
            'led_1_3_3', 'belt_1_3_1', 'belt_1_3_2', 'belt_1_3_3', 'belt_1_3_4', 'belt_1_3_5',
        ],
    },
}


# The SIMATIC AX TcpClient's send / receive buffers are 256 bytes: a frame
# (10 header + payload + 4 trailer) carries 242 payload bytes at most.
PLC_MAX_PAYLOAD = 242


def io_topics(config: SimulatorConfig) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
    """The IO-mapping channels for a layout: (publish_topics, subscribe_topics).

    PUBLISH_TOPICS/SUBSCRIBE_TOPICS above name test_config.json's components.
    A layout that has all of them keeps exactly that wire layout; any other
    layout (e.g. one designed in the layout editor, with ids like sensor_1,
    button_1, belt_1) gets channels derived from its own components:
      input  (simulator -> PLC): every button, then every sensor, then every
                                 machine's feedback, then clock
      output (PLC -> simulator): every LED, then every belt, then every
                                 machine's command
    in the layout's order (machines: stoppers, robot arms, elevators - see
    mqtt_message_binding.MACHINE_FIELDS). The simulator (MQTTSyncEngine)
    and the PLC side (plc_mock.py) both call this, so they always agree on
    the byte layout."""
    legacy = PUBLISH_TOPICS["input"]["custom_ids"] + SUBSCRIBE_TOPICS["output"]["custom_ids"]
    if set(legacy) <= known_component_ids(config):
        return PUBLISH_TOPICS, SUBSCRIBE_TOPICS
    machines = [m.id for m in (*config.stoppers, *config.robots, *config.elevators, *config.scanners,
                               *config.diverters, *config.turntables, *config.accumulators, *config.doors,
                               *config.safety, *(s for s in config.sinks if s.capacity))]
    inputs = [b.id for b in config.buttons] + [s.id for s in config.sensors] + machines + ["clock"]
    outputs = [l.id for l in config.leds] + [b.id for b in config.belts] + machines
    return ({"input": {**PUBLISH_TOPICS["input"], "custom_ids": inputs}},
            {"output": {**SUBSCRIBE_TOPICS["output"], "custom_ids": outputs}})


# ---------------------------------------------------------------------------
# Data-visualization panel metrics: inbound-only, pre-declared metric
# frames, riding the SAME MQTT connection / TcpFramedServer as the
# "output" channel rather than a second server on a separate port (see
# MetricsMessage in mqtt_message_binding.py). Client-reported metrics must
# be declared here ahead of time, same as belts/sensors/buttons are in
# PUBLISH_TOPICS/SUBSCRIBE_TOPICS above - there is no free-form/dynamically
# named metric support over this wire format.
#
# Each layout is a dataclass whose fields are one dataclass per metric
# *group* (see metric_layout.py) - the wire (ByteType, "group/name") list
# is derived from the typed fields by layout_mapping(), so the class is
# the single source of truth. Field widths come from the f32/u16/u32/...
# aliases; a Vec2/Vec3 field packs as name[0], name[1], ...
#
# Different nodes can report DIFFERENT metric data - each distinct layout
# gets its own FrameType.METRIC_LAYOUT_* value (tcp_framed_transport.py)
# and its own entry in METRICS_CHANNELS below. A node picks which layout
# it's sending by which frame_type it tags its frames with (TCP) or which
# topic it publishes to (MQTT) - see plc_mock.py for a real sender. The
# web editor (mqtt_data_exchange_codegen.py) generates these classes.
METRICS_INTERVAL_MS = 50


# Layout 1 - this project's default: performance counters + a result code
# + a belt velocity vector. See plc_mock.py's send loop.
@dataclass
class MetricLayout1Perf:
    scan_time_ms: f32
    cycle_time_ms: f32
    fn_elapsed_us: f32
    sample_count: u32


@dataclass
class MetricLayout1Results:
    last_result_code: u16
    belt_1_velocity: Vec2


@dataclass
class MetricLayout1:
    perf: MetricLayout1Perf
    results: MetricLayout1Results


# Layout 2 - a deliberately differently-shaped example (environmental/
# condition-monitoring style fields instead of perf/results), proving a
# second node type can report a completely different metric set over the
# same connection without colliding with layout 1's group/name space.
@dataclass
class MetricLayout2Env:
    temperature_c: f32
    humidity_pct: f32
    vibration_alarm_code: u16


@dataclass
class MetricLayout2:
    env: MetricLayout2Env


@dataclass
class MetricLayout3Test123:
    led: bit
    led2: bit
    v: f32


@dataclass
class MetricLayout3:
    test123: MetricLayout3Test123


# frame_type -> {topic_suffix, layout} for every declared metrics layout.
# MQTT has no frame_type header byte (its topic string already
# disambiguates), so each layout gets its own topic; TCP instead routes by
# the frame's frame_type directly (see MQTTSyncEngine._handle_tcp_payload).
METRICS_CHANNELS: Dict[int, Dict[str, Any]] = {
    FrameType.METRIC_LAYOUT_1: {"topic_suffix": 'metrics', "layout": MetricLayout1},
    FrameType.METRIC_LAYOUT_2: {"topic_suffix": 'metrics2', "layout": MetricLayout2},
    FrameType.METRIC_LAYOUT_3: {"topic_suffix": 'metric3', "layout": MetricLayout3},
}


def _struct_endianness(topic_def: Dict[str, Any], config: SimulatorConfig | None = None) -> str:
    """Return the struct byte-order prefix for PLC payload values."""
    default_endianness = config.mqttSettings.endianness if config is not None else "LE"
    endianness = str(topic_def.get("endianness", default_endianness)).upper()
    return "<" if endianness == "LE" else ">"


def apply_exchange_config(config: SimulatorConfig, file_path: str) -> None:
    """Overlay exchange settings from a small JSON file onto the simulator config."""
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    settings_data = data.get("mqttSettings", data)
    merged = config.mqttSettings.model_dump()
    merged.update(settings_data)
    config.mqttSettings = type(config.mqttSettings)(**merged)


class _TopicChannel:
    """Internal helper: one publish or subscribe channel with rate limiting.
    `frame_type` is what a publish channel's frames are tagged with on TCP.

    Scheduled with perf_counter(), not monotonic(): on Windows monotonic()
    only advances every ~15.6 ms, which capped a 10 ms channel at ~64 Hz."""

    def __init__(self, topic: str, message: ComponentMessage, interval_ms: int,
                 frame_type: int = FrameType.IO_MAPPING):
        self.topic = topic
        self.message = message
        self.interval_s = interval_ms / 1000.0
        self.last_time = 0.0
        self.frame_type = int(frame_type)

    def is_due(self) -> bool:
        """Return True if enough time has elapsed since the last send/recv."""
        return (time.perf_counter() - self.last_time) >= self.interval_s

    def mark(self) -> None:
        self.last_time = time.perf_counter()

    def mark_periodic(self) -> None:
        """Advance the schedule by exactly one interval, so publishing keeps
        the configured rate on average instead of drifting late by up to one
        caller tick each period. Resets to now if more than one interval
        behind (first send, or the caller stalled)."""
        now = time.perf_counter()
        if now - self.last_time < 2.0 * self.interval_s:
            self.last_time += self.interval_s
        else:
            self.last_time = now


class MQTTSyncEngine:
    """
    Owns the MQTT connection and manages publish/subscribe channels.

    Constructor accepts a SimulatorConfig directly (not a file path) so that
    main.py can share the same live config object with the physics engine and
    renderer.
    """

    def __init__(
        self,
        config: SimulatorConfig,
        lock: Optional[threading.Lock] = None,
        metrics_store: Optional[MetricsStore] = None,
        channel_registry_path: Optional[str] = None,
        show_layout: bool = False,
    ):
        self.config = config
        self._enabled = config.mqttSettings.enabled
        self._transport = config.mqttSettings.transport
        self._pub_channels: List[_TopicChannel] = []
        self._sub_channels: List[_TopicChannel] = []
        # Fed by the data-visualization panel's metrics channels (see
        # METRICS_CHANNELS above) - kept empty when no store was given, so
        # callers that don't care about metrics pay nothing for it. Keyed
        # by frame_type since different nodes can report differently-
        # shaped metric data, one _TopicChannel per declared layout.
        self.metrics_store = metrics_store
        self._metrics_channels: Dict[int, _TopicChannel] = {}
        self._mqtt: FlexibleProtocolMQTTManager | None = None
        self._tcp: TcpFramedServer | None = None
        # This simulator's own identity on the TCP frame protocol's Node ID
        # byte - tags every frame it sends (see FrameType/ParsedFrame in
        # tcp_framed_transport.py). A connecting client (plc_mock.py) tags
        # its own frames with its own separate node id.
        self._node_id = config.mqttSettings.nodeId
        # Pass the SAME lock main.py gives PhysicsEngine so serialize()/
        # deserialize() here can't interleave with physics's own reads/
        # writes of config.belts/config.sensors on the main thread (see
        # PhysicsEngine.__init__'s docstring for the race this closes).
        # Falls back to a private lock if none is given, so this class still
        # works correctly standalone (just without that extra guarantee).
        self._lock = lock if lock is not None else threading.Lock()
        # Inbound IO frames applied to config / rejected while applying.
        self.rx_frames = 0
        self.rx_errors = 0
        # Inbound IO frames of the wrong size (the PLC's layout differs) -
        # counted, and reported at most every few seconds.
        self.rx_size_errors = 0
        self._size_warned_at = -1e9
        # Result of the last FrameType.LAYOUT_CHECK from a client: None (none
        # yet), True (layouts match) or False - and per TCP connection.
        self.layout_match: Optional[bool] = None
        self.layout_match_by_conn: Dict[int, bool] = {}
        # Output frames ignored: from a node other than tcpOutputOwner, or
        # (tcpRequireLayoutCheck) from a connection without a matching check.
        self.rx_ignored = 0

        if not self._enabled:
            print("[DataExchange] Exchange disabled in config. Skipping.")
            return

        prefix = config.mqttSettings.topicPrefix or ""

        # Everything below iterates over these three dicts, never the
        # module-level PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS
        # directly - so a caller that doesn't pass channel_registry_path
        # gets exactly today's hardcoded channels, and one that does gets
        # the same three loops run against registry-derived definitions
        # instead (see channel_registry.load_channel_registry).
        if channel_registry_path is not None:
            publish_topics, subscribe_topics, metrics_channels = load_channel_registry(channel_registry_path)
        else:
            publish_topics, subscribe_topics = io_topics(config)
            metrics_channels = METRICS_CHANNELS

        if self._transport == "mqtt":
            broker_url = config.mqttSettings.brokerUrl
            broker_port = config.mqttSettings.port
            self._mqtt = FlexibleProtocolMQTTManager(
                broker_url=broker_url,
                port=broker_port,
            )
        elif self._transport != "tcp":
            raise ValueError(f"Unsupported exchange transport: {self._transport}")


        # Build publish channels
        for topic_suffix, topic_def in publish_topics.items():
            full_topic = f"{prefix}{topic_suffix}".replace("//", "/")
            interval_ms = topic_def.get("interval", 100)

            # A registry-derived topic_def already carries a resolved
            # mapping (loaded from its CSV); the legacy hardcoded dicts
            # carry "custom_ids" instead and still go through
            # custom_component_mapping() exactly as before.
            mapping = topic_def.get("mapping")
            if mapping is None:
                mapping = custom_component_mapping(config, topic_def["custom_ids"], direction="out")
            verify_component_list(mapping, config)
            message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def, config))
            if show_layout:
                print(io_layout.format_layout(mapping, f"[DataExchange] {full_topic} (simulator -> PLC)"))

            frame_type = topic_def.get("frame_type", FrameType.IO_MAPPING)
            channel = _TopicChannel(full_topic, message, interval_ms, frame_type=frame_type)
            self._pub_channels.append(channel)
            print(f"[DataExchange] Publish channel: {full_topic}  "
                  f"({len(mapping)} entries, {io_layout.describe(mapping)}, {interval_ms} ms, "
                  f"frame_type={channel.frame_type})")

        # Belt drive feedback (actualSpeed + encoderCount per belt) as its own
        # frame type. Opt-in: a PLC that deserializes every frame into its
        # IO_MAPPING struct without checking the frame type byte would be
        # corrupted by it.
        if config.mqttSettings.beltFeedback and config.belts:
            mapping = belt_feedback_mapping(config)
            message = ComponentMessage(mapping, endianness=_struct_endianness({}, config))
            full_topic = f"{prefix}belt_feedback".replace("//", "/")
            channel = _TopicChannel(full_topic, message, config.mqttSettings.beltFeedbackIntervalMs,
                                    frame_type=FrameType.BELT_FEEDBACK)
            self._pub_channels.append(channel)
            print(f"[DataExchange] Belt feedback channel: {full_topic}  "
                  f"({len(config.belts)} belts, {message.payload_size} payload bytes, "
                  f"{config.mqttSettings.beltFeedbackIntervalMs} ms, frame_type={channel.frame_type})")

        # Build subscribe channels and register callbacks
        for topic_suffix, topic_def in subscribe_topics.items():
            full_topic = f"{prefix}{topic_suffix}".replace("//", "/")
            interval_ms = topic_def.get("interval", 100)

            mapping = topic_def.get("mapping")
            if mapping is None:
                mapping = custom_component_mapping(config, topic_def["custom_ids"], direction="in")
            verify_component_list(mapping, config)
            message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def, config))
            if show_layout:
                print(io_layout.format_layout(mapping, f"[DataExchange] {full_topic} (PLC -> simulator)"))

            channel = _TopicChannel(full_topic, message, interval_ms)
            self._sub_channels.append(channel)

            # Register the deserialization callback
            if self._mqtt is not None:
                self._mqtt.register_callback(
                    full_topic,
                    self._make_recv_handler(channel),
                )
            print(f"[DataExchange] Subscribe channel: {full_topic}  "
                  f"({len(mapping)} entries, {io_layout.describe(mapping)}, {interval_ms} ms)")

        # Metrics channels - inbound-only, like "output" above, but target
        # self.metrics_store instead of self.config (see MetricsMessage).
        # One per declared layout in METRICS_CHANNELS, keyed by frame_type
        # (TCP's routing key) - a node reports layout 2's fields by tagging
        # its frames FrameType.METRIC_LAYOUT_2, or (MQTT) by publishing to
        # that layout's own topic. Skipped entirely when no store was
        # given, so this class still works exactly as before for callers
        # that don't pass one.
        if self.metrics_store is not None:
            for frame_type, channel_def in metrics_channels.items():
                metrics_topic = f"{prefix}{channel_def['topic_suffix']}".replace("//", "/")
                interval_ms = channel_def.get("interval", METRICS_INTERVAL_MS)
                # Hardcoded METRICS_CHANNELS carries "layout" (a metric
                # layout dataclass, metric_layout.py) - from_layout() derives
                # the wire (ByteType, path) list from its typed fields. The
                # registry path (channel_registry.py) carries "mapping" (an
                # already-resolved (ByteType, path) list) instead.
                endianness = _struct_endianness({}, config)
                if "layout" in channel_def:
                    metrics_message = MetricsMessage.from_layout(channel_def["layout"], endianness=endianness)
                else:
                    metrics_message = MetricsMessage(channel_def["mapping"], endianness=endianness)
                channel = _TopicChannel(metrics_topic, metrics_message, interval_ms)
                self._metrics_channels[frame_type] = channel
                if self._mqtt is not None:
                    self._mqtt.register_callback(
                        metrics_topic,
                        self._make_metrics_recv_handler(channel),
                    )
                print(f"[DataExchange] Metrics channel (frame_type={frame_type}): {metrics_topic}  "
                      f"({len(metrics_message.mapping)} entries, {metrics_message.payload_size} payload bytes)")

        if self._mqtt is not None:
            self._mqtt.start()
            print(f"[MQTTSync] Connecting to {broker_url}:{broker_port} ...")

        if self._transport == "tcp":
            bind_host = config.mqttSettings.tcpBindHost
            tcp_port = config.mqttSettings.tcpPort
            settings = config.mqttSettings
            self._tcp = TcpFramedServer(
                bind_host,
                tcp_port,
                self._handle_tcp_payload,
                length_endianness=settings.tcpLengthEndianness or settings.endianness,
                rx_timeout=settings.tcpRxTimeoutMs / 1000.0 if settings.tcpRxTimeoutMs else None,
                max_payload=settings.tcpMaxPayload,
            )
            self._warn_frame_sizes()
            self._tcp.start()
            print(f"[TCPSync] Simulator server listening on {bind_host}:{tcp_port} ...")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def sync(self) -> None:
        """
        Called once per frame from the game loop.
        Publishes all due outbound channels. Inbound channels are updated
        asynchronously by MQTT callbacks.
        """
        if not self._enabled:
            return

        for ch in self._pub_channels:
            if not ch.is_due():
                continue
            try:
                # Lock held only around serialize() - it returns a plain,
                # immutable `bytes` payload with no further dependency on
                # `self.config`, so the actual network write below doesn't
                # need to happen while holding the lock. Previously the lock
                # was held across the send too, which meant a stalled/slow
                # client's blocking sendall() (up to RECV_TIMEOUT_S) also
                # blocked PhysicsEngine.update() and every other client's
                # inbound deserialize() for that same duration - one bad
                # connection could stall the whole app, not just its own
                # channel.
                with self._lock:
                    payload = ch.message.serialize(self.config)
            except Exception as exc:
                print(f"[MQTTSync] Error serializing message: {exc}")
                # Still mark the channel even on failure - otherwise is_due()
                # stays True forever and a single broken channel (e.g. a
                # mapped component deleted at runtime) retries and fails on
                # literally every frame instead of respecting its configured
                # interval like every other channel does.
                ch.mark()
                continue

            if self._mqtt is not None:
                self._mqtt.publish(ch.topic, payload)
            elif self._tcp is not None:
                # Non-blocking: queued for each client's writer thread.
                self._tcp.send(payload, frame_type=ch.frame_type, node_id=self._node_id)
            ch.mark_periodic()

    def stop(self) -> None:
        """Cleanly disconnect from the broker."""
        if self._mqtt is not None:
            self._mqtt.stop()
            print("[MQTTSync] Disconnected.")
        if self._tcp is not None:
            self._tcp.stop()
            print("[TCPSync] Stopped.")

    def stats(self) -> Dict[str, int]:
        """Exchange health counters: inbound frames applied/rejected, plus
        (TCP) connected clients and frames sent/dropped by slow clients."""
        result = {"rx_frames": self.rx_frames, "rx_errors": self.rx_errors,
                  "rx_size_errors": self.rx_size_errors, "rx_ignored": self.rx_ignored}
        if self._tcp is not None:
            result.update(self._tcp.stats())
        return result

    @property
    def connected(self) -> bool:
        if self._mqtt is not None:
            return self._mqtt.client.is_connected()
        if self._tcp is not None:
            return self._tcp.connected
        return False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _make_recv_handler(self, channel: _TopicChannel):
        """Create a closure that deserializes incoming payloads into self.config."""
        def handler(raw):
            payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
            if len(payload) != channel.message.payload_size:
                self.rx_size_errors += 1
                now = time.monotonic()
                if now - self._size_warned_at >= 5.0:
                    self._size_warned_at = now
                    print(f"[DataExchange] Ignoring {channel.topic} frames: expected "
                          f"{channel.message.payload_size} bytes, got {len(payload)} "
                          f"({self.rx_size_errors} so far). The PLC's I/O layout differs from this "
                          f"layout's ({io_layout.describe(channel.message.mapping)}) - compare with "
                          f"`python io_layout.py <layout.json> --st`.")
                return

            try:
                with self._lock:
                    channel.message.deserialize(payload, self.config)
            except Exception as exc:
                # e.g. a mapped component deleted in the editor at runtime.
                # Drop this one frame; letting it propagate would kill the
                # client's whole TCP connection (its recv loop's handler).
                self.rx_errors += 1
                print(f"[MQTTSync] Error applying {channel.topic} payload: {exc}")
                return
            self.rx_frames += 1
            channel.mark()
        return handler

    def _make_metrics_recv_handler(self, channel: _TopicChannel):
        """Like _make_recv_handler, but deserializes into self.metrics_store
        instead of self.config - metrics don't touch config.belts/sensors
        at all, so this deliberately does NOT take self._lock (that lock
        exists solely to guard concurrent config access; MetricsStore has
        its own internal lock and holding an unrelated one here would only
        add needless contention with PhysicsEngine.update()).

        `node_id` defaults to 0 so this same handler works unchanged as an
        MQTT topic callback (which only ever calls it with one argument,
        the payload - MQTT has no per-message node id) - the TCP path
        below passes the frame's real node_id explicitly."""
        def handler(raw, node_id: int = 0):
            payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
            if len(payload) != channel.message.payload_size:
                print(
                    f"[MQTTSync] Ignoring {channel.topic} payload: "
                    f"expected {channel.message.payload_size} bytes, got {len(payload)}"
                )
                return
            channel.message.deserialize(payload, self.metrics_store, node_id=node_id)
            channel.mark()
        return handler

    def _warn_frame_sizes(self) -> None:
        """The SIMATIC AX TcpClient (tcp_client_lib.st) sends and receives
        through 256-byte buffers: a frame's payload must stay within 242
        bytes. Warn when an I/O frame of this layout doesn't."""
        for channel in (*self._pub_channels, *self._sub_channels):
            size = channel.message.payload_size
            if size > PLC_MAX_PAYLOAD:
                print(f"[TCPSync] Warning: the {channel.topic} frame has a {size}-byte payload - over the "
                      f"{PLC_MAX_PAYLOAD} bytes the AX TcpClient's 256-byte buffers can take; raise them "
                      f"on the PLC (or set tcpMaxPayload to see it dropped here).")

    def io_layout_check(self) -> Optional[tuple]:
        """(input fingerprint, input size, output fingerprint, output size)
        of the IO-mapping channels, or None without both."""
        pub = next((c for c in self._pub_channels if c.frame_type == FrameType.IO_MAPPING), None)
        sub = self._sub_channels[0] if self._sub_channels else None
        if pub is None or sub is None:
            return None
        return (io_layout.layout_fingerprint(pub.message.mapping), pub.message.payload_size,
                io_layout.layout_fingerprint(sub.message.mapping), sub.message.payload_size)

    def _handle_layout_check(self, frame: ParsedFrame) -> None:
        """A PLC sent its layout fingerprints (io_layout.pack_layout_check):
        compare with ours, log the result, and answer with ours and the
        verdict so the PLC can log it too."""
        endian = _struct_endianness({}, self.config)
        theirs = io_layout.unpack_layout_check(frame.payload, endian)
        mine = self.io_layout_check()
        if theirs is None or mine is None:
            print(f"[DataExchange] Ignoring LAYOUT_CHECK from node {frame.node_id}: "
                  f"{'bad payload' if theirs is None else 'no IO channels here'}.")
            return
        problems = io_layout.compare_layout_check(mine, theirs[:4])
        self.layout_match = not problems
        if frame.conn is not None:
            self.layout_match_by_conn[frame.conn] = not problems
        if problems:
            print(f"[DataExchange] I/O layout MISMATCH with node {frame.node_id}: " + "; ".join(problems))
        else:
            print(f"[DataExchange] I/O layout check with node {frame.node_id}: OK "
                  f"(input {mine[1]} bytes {mine[0]:08X}, output {mine[3]} bytes {mine[2]:08X})")
        reply = struct.pack(endian + io_layout._CHECK, *mine,
                            io_layout.CHECK_MISMATCH if problems else io_layout.CHECK_MATCH)
        if self._tcp is not None:
            # To the client that asked only: a PLC decodes whatever frame it
            # receives as its input image (it doesn't check the frame type),
            # so a broadcast reply would corrupt every other client's inputs.
            self._tcp.send_to(frame.conn, reply, frame_type=FrameType.LAYOUT_CHECK, node_id=self._node_id)

    def _handle_tcp_payload(self, frame: ParsedFrame) -> None:
        """Routes by the frame's explicit FrameType byte (see
        tcp_framed_transport.py) - not by guessing from payload size, which
        used to be ambiguous whenever two channels' payload sizes happened
        to collide. A METRIC_LAYOUT_* frame_type also selects WHICH
        metrics layout/mapping applies (see METRICS_CHANNELS), since
        different nodes can report differently-shaped metric data."""
        metrics_channel = self._metrics_channels.get(frame.frame_type)
        if metrics_channel is not None:
            self._make_metrics_recv_handler(metrics_channel)(frame.payload, frame.node_id)
            return

        if frame.frame_type == FrameType.LAYOUT_CHECK:
            self._handle_layout_check(frame)
            return

        if frame.frame_type == FrameType.IO_MAPPING:
            settings = self.config.mqttSettings
            if (settings.tcpOutputOwner is not None and frame.node_id != settings.tcpOutputOwner) or (
                    settings.tcpRequireLayoutCheck and not self.layout_match_by_conn.get(frame.conn, False)):
                self.rx_ignored += 1
                return
            if not self._sub_channels:
                print("[TCPSync] Ignoring payload because no subscribe channels are configured.")
                return
            if len(self._sub_channels) > 1:
                print("[TCPSync] Multiple subscribe channels configured; using the first channel.")
            self._make_recv_handler(self._sub_channels[0])(frame.payload)
            return

        print(f"[TCPSync] Ignoring frame with unrecognized frame_type={frame.frame_type} from node {frame.node_id}.")
