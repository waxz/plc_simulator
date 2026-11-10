"""
device_kinds.py - every kind of machine / logistics device on the PLC
exchange, declared once.

A DeviceKind says, for one SimulatorConfig list (stoppers, diverters, ...):
its model class, its PLC commands (PLC -> simulator) and feedback
(simulator -> PLC) as (bits, values), the name of its STRUCTs on the PLC
side (st_export.py: <stName>_Command / <stName>_Feedback) and which of its
components are on the exchange. Everything else derives from DEVICE_KINDS:

  mqtt_message_binding   the frame mapping (MACHINE_FIELDS, machine_fields,
                         find_component_by_id, known_component_ids)
  mqtt_data_exchange     io_topics: which ids go into the I/O frames, in
                         DEVICE_KINDS order (the wire order)
  st_export.py           the PLC STRUCTs and serializers
  io_signals.py          the inspector's / designer's PLC I/O
  sim_process.py         the fields mirrored to the simulator window

Adding a kind of device
-----------------------
  1. its model in config_parser.py, a list for it in SimulatorConfig and a
     `control: ControlMode = "plc"` field when it takes commands
  2. one register(DeviceKind(...)) call below - append it: the order is the
     wire order, so existing frames keep their layout
  3. what it does in the simulation (machines.py / logistics.py read its
     command fields and write its feedback fields)
  4. what the mock PLC does with it: a handler in plc_logic.DEVICE_PROGRAMS
  5. how it looks: layout_editor.py (canvas item, catalogue entry),
     logistics_draw.py / visualizer.py, scene3d.py + view3d_web
Then regenerate the AX side: python st_export.py <layout> --out ...
"""
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, Iterator, List, Optional, Tuple

import config_parser as cp


class ByteType(Enum):
    BIT = "bit"
    BYTE = "byte"
    UINT16 = "uint16"
    UINT32 = "uint32"
    UINT64 = "uint64"
    FLOAT = "float"
    PADDING = "padding"


Field = Tuple[ByteType, str]
Fields = Tuple[List[Field], List[Field]]          # (bits, values): bits are packed first
NONE: Fields = ([], [])


def bits(*names: str) -> List[Field]:
    return [(ByteType.BIT, n) for n in names]


@dataclass(frozen=True)
class DeviceKind:
    list_name: str                     # the SimulatorConfig list: "diverters"
    model: type                        # config_parser.Diverter
    st_name: str                       # PLC STRUCTs: Diverter_Command / Diverter_Feedback
    commands: Fields = NONE            # PLC -> simulator
    feedback: Fields = NONE            # simulator -> PLC
    runtime: Tuple[str, ...] = ()      # further fields the simulator window mirrors (drawing)
    on_exchange: Optional[Callable[[object], bool]] = None    # which ones are on it (None: all)
    doc: str = ""

    def exchanged(self, component) -> bool:
        return self.on_exchange is None or bool(self.on_exchange(component))

    def fields(self, direction: str) -> Fields:
        """("in": commands, "out": feedback) - the channel convention."""
        return self.feedback if direction == "out" else self.commands

    @property
    def signal_names(self) -> Tuple[str, ...]:
        names = [n for side in (self.commands, self.feedback) for group in side for _bt, n in group]
        return tuple(dict.fromkeys(names))


DEVICE_KINDS: Dict[str, DeviceKind] = {}       # list name -> kind, in wire order
_BY_MODEL: Dict[type, DeviceKind] = {}


def register(kind: DeviceKind) -> DeviceKind:
    if kind.list_name in DEVICE_KINDS:
        raise ValueError(f"device kind {kind.list_name!r} is already registered")
    if kind.list_name not in cp.SimulatorConfig.model_fields:
        raise ValueError(f"SimulatorConfig has no list {kind.list_name!r}")
    DEVICE_KINDS[kind.list_name] = kind
    _BY_MODEL[kind.model] = kind
    return kind


def kind_of(component) -> Optional[DeviceKind]:
    return _BY_MODEL.get(type(component))


def exchanged_devices(config) -> Iterator[object]:
    """Every device on the exchange, in wire order (kind by kind, each in
    layout order)."""
    for kind in DEVICE_KINDS.values():
        for component in getattr(config, kind.list_name, []):
            if kind.exchanged(component):
                yield component


# ---------------------------------------------------------------------------
# The kinds. Append new ones at the end: the order is the I/O frames' order.
# ---------------------------------------------------------------------------

register(DeviceKind(
    "stoppers", cp.Stopper, "Stopper",
    commands=(bits("release"), []),
    feedback=(bits("isUp", "occupied"), []),
    doc="A pop-up stopper: release lowers it."))

register(DeviceKind(
    "robots", cp.RobotArm, "Robot",
    # FANUC UOP style: enable, start, hold, fault reset, gripper + program
    # (PNS), speed override and a target pose in; status bits, cycle count,
    # fault code, the six joints and the tool centre point out.
    commands=(bits("start", "enable", "hold", "faultReset", "gripperClose"),
              [(ByteType.BYTE, "program"), (ByteType.BYTE, "speedOverride"), (ByteType.FLOAT, "targetX"),
               (ByteType.FLOAT, "targetY"), (ByteType.FLOAT, "targetZ"), (ByteType.FLOAT, "targetYaw")]),
    feedback=(bits("busy", "holding", "partAtPick", "placeReady", "ready", "atHome", "inPosition", "fault"),
              [(ByteType.UINT16, "cycles"), (ByteType.BYTE, "faultCode"),
               *[(ByteType.FLOAT, f"j{i}") for i in range(1, 7)],
               (ByteType.FLOAT, "tcpX"), (ByteType.FLOAT, "tcpY"), (ByteType.FLOAT, "tcpZ")]),
    runtime=("toolX", "toolY", "phase", "phaseT"),
    doc="A 6-axis robot (FANUC UOP style)."))

register(DeviceKind(
    "elevators", cp.Elevator, "Elevator",
    commands=([], [(ByteType.BYTE, "targetLevel")]),
    feedback=(bits("moving", "atLevel", "occupied"), [(ByteType.BYTE, "level")]),
    runtime=("position", "liftVelocity", "motorSpeed"),
    doc="A lift between floor levels."))

register(DeviceKind(
    "scanners", cp.Scanner, "Scanner",
    feedback=(bits("noRead", "present"),
              [(ByteType.UINT32, "code"), (ByteType.BYTE, "destination"), (ByteType.UINT16, "readCount"),
               (ByteType.FLOAT, "weight"), (ByteType.FLOAT, "measuredLength"),
               (ByteType.FLOAT, "measuredWidth"), (ByteType.FLOAT, "measuredHeight")]),
    doc="A barcode / RFID / DWS scanner: reads each parcel once."))

register(DeviceKind(
    "diverters", cp.Diverter, "Diverter",
    commands=(bits("divert"), [(ByteType.FLOAT, "commandSpeed"), (ByteType.FLOAT, "commandAngle")]),
    feedback=(bits("active", "present"), [(ByteType.UINT32, "presentCode"), (ByteType.UINT16, "diverted")]),
    doc="A pusher / pop-up wheel diverter: divert at a speed and direction."))

register(DeviceKind(
    "turntables", cp.Turntable, "Turntable",
    commands=([], [(ByteType.BYTE, "targetIndex")]),
    feedback=(bits("turning", "inPosition", "occupied"), [(ByteType.BYTE, "index")]),
    runtime=("angle", "turnVelocity", "motorSpeed"),
    doc="A turntable: turns its belt between positions."))

register(DeviceKind(
    "transfers", cp.Transfer, "Transfer",
    commands=(bits("transfer"), []),
    feedback=(bits("present", "centered", "up"), [(ByteType.UINT16, "transferred")]),
    runtime=("lift", "motorSpeed"),
    doc="A 90-degree chain transfer of a roller line."))

register(DeviceKind(
    "accumulators", cp.Accumulator, "Accumulator",
    commands=(bits("release"), []),
    feedback=(bits("full"), [(ByteType.UINT16, "zones")]),
    doc="A zero-pressure accumulation conveyor."))

register(DeviceKind(
    "doors", cp.GuardDoor, "GuardDoor",
    commands=(bits("lock"), []),
    feedback=(bits("closed", "locked"), []),
    doc="A guard door in a safety fence."))

register(DeviceKind(
    "safety", cp.SafetyCircuit, "SafetyRelay",
    commands=(bits("reset"), []),
    feedback=(bits("ok", "tripped"), []),
    doc="A safety relay: e-stops, guard doors, light curtains."))

register(DeviceKind(
    "sinks", cp.Sink, "Chute",
    commands=(bits("empty"), []),
    feedback=(bits("full"), [(ByteType.UINT16, "count")]),
    on_exchange=lambda sink: bool(sink.capacity),
    doc="A chute (a sink with a capacity); plain sinks are not on the exchange."))
