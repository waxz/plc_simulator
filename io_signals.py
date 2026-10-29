"""
io_signals.py - one component's PLC signals, by name: what the designer's
Data Exchange section shows (live from the running simulator) and what an
operator may set by hand there.

The signals are the component's part of the PLC data exchange, taken from
the same mapping the frames are built from (mqtt_message_binding.
paddng_component), so the list matches the wire exactly:

  command   PLC -> simulator  (a belt's speed / run / reverse, a stopper's
                               release, a diverter's divert, a lamp, ...)
  feedback  simulator -> PLC  (a sensor's output, a belt's actualSpeed,
                               a stopper's isUp, ...)

A component's `control` (config_parser.CommandSource / ControlMode /
SensorControl) says where its commands come from: the PLC exchange
("plc"), its built-in controller ("auto", machines) or the operator
("manual"). Only in "manual" may they be set by hand (apply()); a sensor in
"manual" is forced: its output is the value set here.

  signals(component)          -> [Signal, ...] (no config needed)
  read(config, ids)           -> {id: {"control": ..., "values": {name: value}}}
  apply(config, id, name, v)  -> None, or why not (a text)
  control_options(component)  -> ["auto", "plc", "manual"], ... or []
"""
import typing
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import config_parser as cp
from io_layout import PLC_TYPES
from mqtt_message_binding import (_MACHINE_KIND, ByteType, _compact_int, _int_to_matching_shape, _read_bit,
                                  _write_bit, find_component_by_id, paddng_component, parse_path)

_LIST_OF = {cp.Belt: "belts", cp.Sensor: "sensors", cp.ControlButton: "buttons", cp.LedIndicator: "leds"}
_WIDTH = {ByteType.BYTE: 8, ByteType.UINT16: 16, ByteType.UINT32: 32, ByteType.UINT64: 64}


@dataclass(frozen=True)
class Signal:
    name: str               # the model field ("speed", "sensor_states", "isUp", ...)
    byte_type: ByteType
    count: int = 1          # bits of an indexed field (a laser banner's beams)
    indexed: bool = False   # on the wire as name[i] bits
    direction: str = "feedback"   # "command" (PLC -> simulator) or "feedback"
    settable: bool = False  # may be set by hand in "manual"

    @property
    def plc_type(self) -> str:
        base = PLC_TYPES.get(self.byte_type, self.byte_type.name)
        return f"ARRAY[0..{self.count - 1}] OF {base}" if self.count > 1 else base


def _list_name(component) -> Optional[str]:
    for cls, name in _LIST_OF.items():
        if isinstance(component, cls):
            return name
    return _MACHINE_KIND.get(type(component).__name__)


def _rows(component, direction: str) -> List[Signal]:
    """The component's fields in one direction's frame, in wire order, a
    banner's beams (name[i]) as one signal."""
    name = _list_name(component)
    if name is None:
        return []
    config = cp.SimulatorConfig()
    getattr(config, name).append(component)
    out: Dict[str, list] = {}
    for byte_type, path in paddng_component(config, sensor_id=[component.id], direction=direction):
        cid, field, index = parse_path(path)
        if cid != component.id:                 # padding
            continue
        row = out.setdefault(field, [byte_type, 0, index is not None])
        row[1] += 1
    return [Signal(field, bt, count, indexed) for field, (bt, count, indexed) in out.items()]


def signals(component) -> List[Signal]:
    """Every PLC signal of a component, commands first. A plain sink and a
    source have none; a pallet neither."""
    if isinstance(component, cp.Sink) and not component.capacity:
        return []
    if isinstance(component, (cp.Sensor, cp.ControlButton)):
        # Inputs of the PLC: their output state is the feedback. A sensor's
        # can be forced by hand; a button is the operator's (press it).
        settable = isinstance(component, cp.Sensor)
        return [Signal(s.name, s.byte_type, s.count, s.indexed, "feedback", settable)
                for s in _rows(component, "out")]
    if isinstance(component, cp.LedIndicator):
        return [Signal(s.name, s.byte_type, s.count, s.indexed, "command", True) for s in _rows(component, "in")]
    commands = [Signal(s.name, s.byte_type, s.count, s.indexed, "command", True) for s in _rows(component, "in")]
    feedback = [Signal(s.name, s.byte_type, s.count, s.indexed, "feedback", False) for s in _rows(component, "out")]
    return commands + feedback


def control_options(component) -> List[str]:
    """The values its `control` may take (from the model's type), or []."""
    field = type(component).model_fields.get("control")
    return list(typing.get_args(field.annotation)) if field is not None else []


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return round(value, 4)
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def value_of(component, signal: Signal) -> Any:
    """A signal's value as the PLC sees it: a bool, a number, or a banner's
    beams as a "0101..." string (beam 0 first)."""
    raw = getattr(component, signal.name, None)
    if signal.indexed:
        if signal.count == 1:
            return _read_bit(raw, 0)
        return "".join("1" if _read_bit(raw, i) else "0" for i in range(signal.count))
    if signal.byte_type == ByteType.BIT:
        return bool(raw)
    if signal.byte_type == ByteType.FLOAT:
        return _jsonable(float(raw or 0.0))
    if isinstance(raw, (str, bool)) or raw is None:
        return _compact_int(raw, signal.byte_type)
    return int(raw)


def read(config, ids) -> Dict[str, dict]:
    """The live signal values of the components `ids` (unknown ones left out)."""
    out = {}
    for cid in ids:
        try:
            component = find_component_by_id(cid, config)
        except ValueError:
            continue
        out[cid] = {"control": getattr(component, "control", None),
                    "values": {s.name: value_of(component, s) for s in signals(component)}}
    return out


def apply(config, component_id: str, name: str, value: Any) -> Optional[str]:
    """Set a component's `control`, or one of its settable signals while it
    is in "manual". Returns None when done, else why not."""
    try:
        component = find_component_by_id(component_id, config)
    except ValueError:
        return f"no component {component_id!r}"
    if name == "control":
        options = control_options(component)
        if value not in options:
            return f"control of {component_id} is one of {', '.join(options) or 'nothing'}"
        component.control = value
        return None
    signal = next((s for s in signals(component) if s.name == name), None)
    if signal is None or not signal.settable:
        return f"{component_id} has no settable signal {name!r}"
    if getattr(component, "control", None) != "manual":
        return f"{component_id} is not in manual (control {getattr(component, 'control', None)!r})"
    current = getattr(component, name, None)
    try:
        if signal.indexed:
            bits = [bool(value)] if signal.count == 1 else [ch == "1" for ch in str(value).ljust(signal.count, "0")]
            for i, bit in enumerate(bits[:signal.count]):
                current = _write_bit(current, i, bit)
            new = current
        elif signal.byte_type == ByteType.BIT:
            new = bool(value)
        elif signal.byte_type == ByteType.FLOAT:
            new = float(value)
        else:
            number = int(value) & ((1 << _WIDTH.get(signal.byte_type, 64)) - 1)
            new = _int_to_matching_shape(number, current) if isinstance(current, (str, bool)) else number
    except (TypeError, ValueError):
        return f"{value!r} is not a {signal.plc_type}"
    setattr(component, name, new)
    return None
