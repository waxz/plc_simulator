"""
plc_foxglove_bridge.py - middleware between a PLC and Foxglove.

A TCP server for the PLC: it receives the PLC's raw binary frames (the same
framing as the simulator link, tcp_framed_transport.py), decodes them with
the layout the PLC was built for (io_layout / ComponentMessage /
MetricsMessage - the layout JSON from the editor), composes JSON messages
and publishes them on a Foxglove WebSocket server (foxglove-sdk), so
Foxglove can plot and inspect the PLC live: open Foxglove, "Open connection"
-> Foxglove WebSocket -> ws://localhost:8765.

Frames from the PLC (frame type in the header):
  IO_MAPPING       its output image: belt commands, lamps   -> /plc/outputs
  INPUT_IMAGE      the input image it works on: sensors,     -> /plc/inputs
                   buttons, its clock stamp (image age)
  METRIC_LAYOUT_n  its metrics layouts                       -> /plc/metrics/<name>
  LAYOUT_CHECK     answered like the simulator does: the PLC learns whether
                   both sides use the same I/O layout
  JSON             a UTF-8 JSON object, published as is   -> /plc/json/<topic>
                   (its "topic" member, default "data"): no packing on the
                   PLC side and no layout needed to read it
  RECORD_SCHEMA    a PLC record's field names and types, once per
                   connection; then
  RECORD_VALUES    its values only, raw - turned into the  -> /plc/json/<topic>
                   same JSON message (plc_records.py): the cheapest way
                   for a PLC to show its own values
Composed here:
  /plc/lines       per line (L1, L2, ...): state RUN / HOLD / STOP / ERROR
                   from its lamps and belts, belts moving, sensors blocked
  /plc/events      foxglove.Log: layout check, line state changes, PLC
                   connects / goes quiet
  /bridge/stats    frames per type and node, rates, bytes, malformed frames

    python plc_foxglove_bridge.py my_layout.json [--listen 0.0.0.0:5002] [--foxglove 127.0.0.1:8765]

plc_mock.py sends exactly these frames with --bridge 127.0.0.1:5002; a real
PLC sends its output STRUCT as IO_MAPPING and its input STRUCT as INPUT_IMAGE
(python io_layout.py my_layout.json --st prints both STRUCTs).
"""
import argparse
import copy
import json
import re
import struct
import threading
import time
from collections import defaultdict
from typing import Any, Dict, List, Optional, Tuple

import foxglove
from foxglove.channels import LogChannel
from foxglove.messages import Log, LogLevel, Timestamp

import devices
import io_layout
import plc_records
from config_parser import SimulatorConfig, load_config
from mqtt_data_exchange import METRICS_CHANNELS, apply_exchange_config, io_topics
from mqtt_message_binding import (
    ByteType, ComponentMessage, MetricsMessage, custom_component_mapping, find_component_by_id, parse_path,
)
from plc_logic import find_lines
from tcp_framed_transport import FrameType, ParsedFrame, TcpFramedServer

QUIET_AFTER_S = 3.0   # no frame from a PLC for this long: "PLC quiet" event


# ---------------------------------------------------------------------------
# Binary -> fields
# ---------------------------------------------------------------------------

def decode_fields(message, payload: bytes) -> List[Tuple[str, Any]]:
    """[(path, value)] of a ComponentMessage / MetricsMessage payload, in
    layout order: bits as bools, numbers as numbers, padding left out - the
    same walk as their deserialize(), without touching any config."""
    values = list(struct.unpack(message.format_string, payload))
    out: List[Tuple[str, Any]] = []
    index, bits, shift = 0, 0, 0
    for byte_type, path in message.mapping:
        if byte_type == ByteType.BIT:
            if shift == 0:
                bits = values[index]
                index += 1
            value = bool((bits >> shift) & 1)
            shift = (shift + 1) % 8
            if not path.startswith("padding"):
                out.append((path, value))
            continue
        shift = 0
        if byte_type == ByteType.PADDING:
            continue
        value = values[index]
        index += 1
        if not path.startswith("padding"):
            out.append((path, value))
    return out


KINDS = (("belts", "belts"), ("sensors", "sensors"), ("buttons", "buttons"), ("leds", "leds"),
         ("sources", "sources"), ("sinks", "sinks"))


class Composer:
    """Decoded fields -> the JSON messages, using the layout for kinds,
    labels and button contacts (an NC STOP reads 1 when not pressed)."""

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self.kind_of: Dict[str, str] = {}
        for attr, kind in KINDS:
            for component in getattr(config, attr):
                self.kind_of[component.id] = kind
        self.lines = {name: {k: [c.id for c in v] for k, v in parts.items()}
                      for name, parts in find_lines(config).items()}

    def components(self, fields: List[Tuple[str, Any]]) -> Dict[str, Dict[str, dict]]:
        """{kind: {id: {variable: value}}}; a bit vector (sensor_states[i])
        becomes a "0101" string like the layout's sensor_states."""
        out: Dict[str, Dict[str, dict]] = defaultdict(dict)
        vectors: Dict[Tuple[str, str], Dict[int, Any]] = defaultdict(dict)
        for path, value in fields:
            component, variable, index = parse_path(path)
            if component == "clock":
                out["clock"][variable] = value
                continue
            if index is not None:
                vectors[(component, variable)][index] = value
                continue
            out[self.kind_of.get(component, "other")].setdefault(component, {})[variable] = value
        for (component, variable), parts in vectors.items():
            bits = "".join("1" if parts[i] else "0" for i in sorted(parts))
            out[self.kind_of.get(component, "other")].setdefault(component, {})[variable] = bits
        return out

    def _label(self, cid: str) -> str:
        component = find_component_by_id(cid, self.config)
        return (getattr(component, "label", "") or cid) if component not in (None, True) else cid

    def outputs(self, fields) -> dict:
        parts = self.components(fields)
        belts = {}
        for cid, values in parts.get("belts", {}).items():
            speed = float(values.get("speed", 0.0))
            run = bool(values.get("run", abs(speed) > 1e-9))
            entry = {"speed": speed, "run": run, "moving": run and abs(speed) > 1e-9}
            if "reverse" in values:
                entry["reverse"] = bool(values["reverse"])
            belts[cid] = entry
        leds = {cid: {"on": bool(v.get("isOn", False)), "label": self._label(cid)}
                for cid, v in parts.get("leds", {}).items()}
        return {"belts": belts, "leds": leds}

    def inputs(self, fields) -> dict:
        parts = self.components(fields)
        sensors = {}
        for cid, values in parts.get("sensors", {}).items():
            bits = str(values.get("sensor_states", "0"))
            sensor = find_component_by_id(cid, self.config)
            blocked_char = "0" if getattr(sensor, "invert", False) else "1"
            entry = {"blocked": blocked_char in bits, "label": self._label(cid)}
            if len(bits) > 1:
                entry["beams"] = bits
                entry["beams_blocked"] = bits.count(blocked_char)
            sensors[cid] = entry
        buttons = {}
        for cid, values in parts.get("buttons", {}).items():
            raw = values.get("sensor_states", "0")
            button = find_component_by_id(cid, self.config)
            pressed = devices.is_pressed_signal(button, raw) if button not in (None, True) else raw in ("1", True)
            buttons[cid] = {"pressed": bool(pressed), "label": self._label(cid)}
        msg = {"sensors": sensors, "buttons": buttons}
        # The image's clock field: when the sender stamped it (time.time_ns()
        # at serialize) - how old the image is when it gets here.
        clock = parts.get("clock", {})
        if "nanoseconds" in clock and clock["nanoseconds"]:
            msg["image_clock_ns"] = int(clock["nanoseconds"])
            msg["image_age_ms"] = (time.time_ns() - int(clock["nanoseconds"])) / 1e6
        return msg

    def line_states(self, outputs: Optional[dict], inputs: Optional[dict]) -> dict:
        """Per line: its lamps and belts as one state - ERROR (an error/jam/
        fault lamp), RUN (auto/run lamp, belts moving), HOLD (auto lamp, belts
        stopped: a feeder waiting at the merge), STOP."""
        lines = {}
        for name, parts in self.lines.items():
            leds = {cid: (outputs or {}).get("leds", {}).get(cid, {}).get("on", False) for cid in parts["leds"]}
            words = {cid: self._label(cid).lower() for cid in parts["leds"]}
            error = any(on for cid, on in leds.items() if any(w in words[cid] for w in ("error", "jam", "fault", "alarm")))
            auto = any(on for cid, on in leds.items() if any(w in words[cid] for w in ("auto", "run", "ok", "green")))
            belts = [(outputs or {}).get("belts", {}).get(cid, {}) for cid in parts["belts"]]
            moving = sum(1 for b in belts if b.get("moving"))
            blocked = [cid for cid in parts["sensors"] if (inputs or {}).get("sensors", {}).get(cid, {}).get("blocked")]
            state = "ERROR" if error else ("RUN" if auto and moving else "HOLD" if auto else "STOP")
            lines[name] = {"state": state, "state_code": ["STOP", "HOLD", "RUN", "ERROR"].index(state),
                           "belts_moving": moving, "belts_total": len(parts["belts"]),
                           "sensors_blocked": blocked}
        return {"lines": lines}


def json_schema(sample: Any) -> dict:
    """A JSON schema for a composed message, from an example of it (so
    Foxglove can offer the field names)."""
    if isinstance(sample, bool):
        return {"type": "boolean"}
    if isinstance(sample, int):
        return {"type": "integer"}
    if isinstance(sample, float):
        return {"type": "number"}
    if isinstance(sample, str):
        return {"type": "string"}
    if isinstance(sample, list):
        return {"type": "array", "items": json_schema(sample[0]) if sample else {}}
    if isinstance(sample, dict):
        return {"type": "object", "properties": {k: json_schema(v) for k, v in sample.items()}}
    return {}


class _MetricsDict:
    """A MetricsStore stand-in: MetricsMessage.deserialize() -> {group: {name: value}}."""

    def __init__(self):
        self.data: Dict[str, Dict[str, Any]] = defaultdict(dict)

    def add_sample(self, name, value, group="", kind="scalar"):
        self.data[group.split("/", 1)[-1]][name] = value


# ---------------------------------------------------------------------------
# The bridge
# ---------------------------------------------------------------------------

class PlcFoxgloveBridge:
    def __init__(self, config: SimulatorConfig, listen: Tuple[str, int] = ("0.0.0.0", 5002),
                 foxglove_at: Tuple[str, int] = ("127.0.0.1", 8765), name: str = "PLC bridge",
                 start_foxglove: bool = True):
        self.config = config
        endian = "<" if str(config.mqttSettings.endianness).upper() == "LE" else ">"
        self.endian = endian
        publish, subscribe = io_topics(config)
        self.input_mapping = custom_component_mapping(config, publish["input"]["custom_ids"], direction="out")
        self.output_mapping = custom_component_mapping(config, subscribe["output"]["custom_ids"], direction="in")
        self.input_msg = ComponentMessage(self.input_mapping, endianness=endian)
        self.output_msg = ComponentMessage(self.output_mapping, endianness=endian)
        self.metrics = {ft: (ch["topic_suffix"].strip("/").replace("/", "_"),
                             MetricsMessage.from_layout(ch["layout"], endianness=endian))
                        for ft, ch in METRICS_CHANNELS.items()}
        self.mine = (io_layout.layout_fingerprint(self.input_mapping), self.input_msg.payload_size,
                     io_layout.layout_fingerprint(self.output_mapping), self.output_msg.payload_size)
        self.composer = Composer(config)

        # Its own context: channels and server belong to this bridge only.
        self.context = ctx = foxglove.Context()
        self.server_fg = (foxglove.start_server(name=name, host=foxglove_at[0], port=foxglove_at[1], context=ctx)
                          if start_foxglove else None)
        zero_out = self.composer.outputs(decode_fields(self.output_msg, bytes(self.output_msg.payload_size)))
        zero_in = self.composer.inputs(decode_fields(self.input_msg, bytes(self.input_msg.payload_size)))
        zero_in.setdefault("image_clock_ns", 0)
        zero_in.setdefault("image_age_ms", 0.0)
        self.ch_outputs = foxglove.Channel("/plc/outputs", schema=json_schema(self._stamp(zero_out)), context=ctx)
        self.ch_inputs = foxglove.Channel("/plc/inputs", schema=json_schema(self._stamp(zero_in)), context=ctx)
        self.ch_lines = foxglove.Channel(
            "/plc/lines", schema=json_schema(self._stamp(self.composer.line_states(zero_out, zero_in))), context=ctx)
        self.ch_metrics = {ft: foxglove.Channel(f"/plc/metrics/{name_}", context=ctx)
                           for ft, (name_, _m) in self.metrics.items()}
        self.ch_stats = foxglove.Channel("/bridge/stats", context=ctx)
        self.ch_events = LogChannel("/plc/events", context=ctx)
        self.ch_json: Dict[str, Any] = {}     # /plc/json/<topic>, made on a topic's first message
        self.records: Dict[Tuple[Any, int, int], plc_records.Schema] = {}   # (connection, node, slot)
        self.values_without_schema = 0

        self._lock = threading.Lock()
        self.last_outputs: Optional[dict] = None
        self.last_inputs: Optional[dict] = None
        self.last_lines: Dict[str, str] = {}
        self.layout_ok: Dict[int, bool] = {}
        self.counts: Dict[Tuple[int, str], int] = defaultdict(int)
        self.bytes_in = 0
        self.malformed = 0
        self.last_frame_at: Dict[int, float] = {}
        self.quiet: Dict[int, bool] = {}
        self._stats_prev = (time.monotonic(), 0)
        self.last_stats: dict = {}   # refreshed once a second by _tick

        self.tcp = TcpFramedServer(listen[0], listen[1], self.on_frame,
                                   length_endianness=config.mqttSettings.endianness)
        self.listen = listen
        self.foxglove_at = foxglove_at
        self._stop = threading.Event()
        self._ticker: Optional[threading.Thread] = None

    @staticmethod
    def _stamp(msg: dict) -> dict:
        ns = time.time_ns()
        return {"timestamp": {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}, **msg}

    def start(self):
        self.tcp.start()
        self._ticker = threading.Thread(target=self._tick, daemon=True)
        self._ticker.start()
        self.event(LogLevel.Info, f"Listening for PLC frames on {self.listen[0]}:{self.listen[1]}; "
                                  f"Foxglove at ws://{self.foxglove_at[0]}:{self.foxglove_at[1]}")

    def stop(self):
        self._stop.set()
        self.tcp.stop()
        if self.server_fg is not None:
            self.server_fg.stop()

    def event(self, level, text: str):
        ns = time.time_ns()
        print(f"[Bridge] {text}")
        self.ch_events.log(Log(timestamp=Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000),
                               level=level, message=text, name="plc_bridge"), log_time=ns)

    # -- frames --------------------------------------------------------------------

    def on_frame(self, frame: ParsedFrame):
        node = frame.node_id
        kind = FrameType(frame.frame_type).name if frame.frame_type in FrameType._value2member_map_ \
            else f"type_{frame.frame_type}"
        with self._lock:
            self.counts[(node, kind)] += 1
            self.bytes_in += len(frame.payload)
            self.last_frame_at[node] = time.monotonic()
            if self.quiet.get(node, True):
                self.quiet[node] = False
                self.event(LogLevel.Info, f"PLC node {node} is sending")
        try:
            if frame.frame_type == FrameType.IO_MAPPING:
                self._io(frame, self.output_msg, "outputs")
            elif frame.frame_type == FrameType.INPUT_IMAGE:
                self._io(frame, self.input_msg, "inputs")
            elif frame.frame_type == FrameType.LAYOUT_CHECK:
                self._layout_check(frame)
            elif frame.frame_type == FrameType.JSON:
                self._json(frame)
            elif frame.frame_type == FrameType.RECORD_SCHEMA:
                self._record_schema(frame)
            elif frame.frame_type == FrameType.RECORD_VALUES:
                self._record_values(frame)
            elif frame.frame_type in self.metrics:
                self._metrics(frame)
        except Exception as exc:  # noqa: BLE001 - one bad frame must not stop the bridge
            with self._lock:
                self.malformed += 1
            print(f"[Bridge] Bad frame {kind} from node {node}: {exc}")

    def _io(self, frame: ParsedFrame, message: ComponentMessage, which: str):
        if len(frame.payload) != message.payload_size:
            with self._lock:
                self.malformed += 1
            return
        fields = decode_fields(message, frame.payload)
        if which == "outputs":
            msg = self.composer.outputs(fields)
            self.last_outputs = msg
            self.ch_outputs.log(self._stamp({"node": frame.node_id, **msg}))
        else:
            msg = self.composer.inputs(fields)
            self.last_inputs = msg
            self.ch_inputs.log(self._stamp({"node": frame.node_id, **msg}))
        lines = self.composer.line_states(self.last_outputs, self.last_inputs)
        self.ch_lines.log(self._stamp(lines))
        for name, info in lines["lines"].items():
            if self.last_lines.get(name) != info["state"]:
                if name in self.last_lines:
                    level = LogLevel.Error if info["state"] == "ERROR" else LogLevel.Info
                    self.event(level, f"{name}: {self.last_lines[name]} -> {info['state']}")
                self.last_lines[name] = info["state"]

    def _layout_check(self, frame: ParsedFrame):
        theirs = io_layout.unpack_layout_check(frame.payload, self.endian)
        if theirs is None:
            with self._lock:
                self.malformed += 1
            return
        problems = io_layout.compare_layout_check(self.mine, theirs[:4])
        self.layout_ok[frame.node_id] = not problems
        if problems:
            self.event(LogLevel.Error, f"I/O layout MISMATCH with PLC node {frame.node_id}: " + "; ".join(problems))
        else:
            self.event(LogLevel.Info, f"I/O layout check with PLC node {frame.node_id}: OK "
                                      f"(input {self.mine[1]} bytes, output {self.mine[3]} bytes)")
        reply = struct.pack(self.endian + io_layout._CHECK, *self.mine,
                            io_layout.CHECK_MISMATCH if problems else io_layout.CHECK_MATCH)
        self.tcp.send_to(frame.conn, reply, frame_type=FrameType.LAYOUT_CHECK, node_id=frame.node_id)

    def _json(self, frame: ParsedFrame):
        msg = json.loads(frame.payload.decode("utf-8"))
        if not isinstance(msg, dict):
            raise ValueError("a JSON frame holds an object")
        self._publish_json(str(msg.pop("topic", "data")), frame.node_id, msg)

    def _publish_json(self, topic: str, node: int, msg: dict):
        topic = re.sub(r"[^A-Za-z0-9_/-]", "_", topic).strip("/") or "data"
        channel = self.ch_json.get(topic)
        if channel is None:
            channel = self.ch_json[topic] = foxglove.Channel(f"/plc/json/{topic}", context=self.context)
        channel.log(self._stamp({"node": node, **msg}))

    def _record_schema(self, frame: ParsedFrame):
        schema = plc_records.parse_schema(frame.payload)
        key = (frame.conn, frame.node_id, schema.slot)
        if key not in self.records or self.records[key].fields != schema.fields:
            self.event(LogLevel.Info, f"Record '{schema.topic}' from PLC node {frame.node_id}: "
                                      f"{len(schema.fields)} fields")
        self.records[key] = schema

    def _record_values(self, frame: ParsedFrame):
        schema = self.records.get((frame.conn, frame.node_id, plc_records.values_slot(frame.payload)))
        if schema is None:
            with self._lock:
                self.values_without_schema += 1    # sent before its schema: dropped, not malformed
            return
        self._publish_json(schema.topic, frame.node_id, plc_records.parse_values(frame.payload, schema))

    def _metrics(self, frame: ParsedFrame):
        name, message = self.metrics[frame.frame_type]
        if len(frame.payload) != message.payload_size:
            with self._lock:
                self.malformed += 1
            return
        store = _MetricsDict()
        message.deserialize(frame.payload, store, node_id=frame.node_id)
        self.ch_metrics[frame.frame_type].log(self._stamp({"node": frame.node_id, **store.data}))

    # -- once a second -------------------------------------------------------------

    def stats(self) -> dict:
        """Counters since start, and the frame rate since the last call
        (the once-a-second tick)."""
        with self._lock:
            now = time.monotonic()
            frames = sum(self.counts.values())
            t0, f0 = self._stats_prev
            rate = (frames - f0) / (now - t0) if now > t0 else 0.0
            self._stats_prev = (now, frames)
            nodes = {}
            for (node, kind), n in self.counts.items():
                entry = nodes.setdefault(str(node), {"frames": {}, "last_frame_age_s": None, "layout_ok": None})
                entry["frames"][kind] = n
                entry["last_frame_age_s"] = round(now - self.last_frame_at.get(node, now), 3)
                entry["layout_ok"] = self.layout_ok.get(node)
            return {"frames_total": frames, "frames_per_s": round(rate, 2), "bytes_in": self.bytes_in,
                    "malformed": self.malformed, "values_without_schema": self.values_without_schema,
                    "nodes": nodes}

    def _tick(self):
        while not self._stop.wait(1.0):
            self.last_stats = self.stats()
            self.ch_stats.log(self._stamp(self.last_stats))
            now = time.monotonic()
            for node, at in list(self.last_frame_at.items()):
                if now - at > QUIET_AFTER_S and not self.quiet.get(node):
                    self.quiet[node] = True
                    self.event(LogLevel.Warning, f"PLC node {node} quiet for {now - at:.1f} s")


def _host_port(text: str, default_host: str) -> Tuple[str, int]:
    host, _, port = text.rpartition(":")
    return (host or default_host), int(port)


def main():
    parser = argparse.ArgumentParser(description="TCP server for PLC frames -> JSON on a Foxglove WebSocket server")
    parser.add_argument("config", help="The layout JSON the PLC was built for (its I/O layout)")
    parser.add_argument("--exchange-config", help="Optional JSON overriding the exchange settings (endianness, ...)")
    parser.add_argument("--listen", default="0.0.0.0:5002", help="TCP address the PLC connects to (default 0.0.0.0:5002)")
    parser.add_argument("--foxglove", default="127.0.0.1:8765",
                        help="Foxglove WebSocket server address (default 127.0.0.1:8765)")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)
    bridge = PlcFoxgloveBridge(copy.deepcopy(config), listen=_host_port(args.listen, "0.0.0.0"),
                               foxglove_at=_host_port(args.foxglove, "127.0.0.1"))
    print(f"[Bridge] Input image {bridge.input_msg.payload_size} bytes: {io_layout.describe(bridge.input_mapping)}")
    print(f"[Bridge] Output image {bridge.output_msg.payload_size} bytes: {io_layout.describe(bridge.output_mapping)}")
    bridge.start()
    print("[Bridge] Topics: /plc/outputs /plc/inputs /plc/lines /plc/metrics/* /plc/json/* /plc/events /bridge/stats. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(5.0)
            s = bridge.last_stats
            if not s:
                continue
            print(f"[Bridge] {s['frames_total']} frames ({s['frames_per_s']}/s), {s['malformed']} malformed, "
                  + ", ".join(f"node {n}: " + " ".join(f"{k}={v}" for k, v in e['frames'].items())
                              for n, e in s["nodes"].items()))
    except KeyboardInterrupt:
        print("\n[Bridge] Stopping.")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
