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
    # A kind whose fields depend on the instance (a typed value): its fields
    # (component, direction) -> Fields, and its STRUCT's name (component) -> str.
    fields_of: Optional[Callable[[object, str], Fields]] = None
    st_name_of: Optional[Callable[[object], str]] = None

    def exchanged(self, component) -> bool:
        return self.on_exchange is None or bool(self.on_exchange(component))

    def fields(self, direction: str, component=None) -> Fields:
        """("in": commands, "out": feedback) - the channel convention; the
        component's own when the kind's fields depend on it."""
        if self.fields_of is not None and component is not None:
            return self.fields_of(component, direction)
        return self.feedback if direction == "out" else self.commands

    def struct_name(self, component=None) -> str:
        """The ST STRUCTs' name stem (Name_Command / Name_Feedback)."""
        if self.st_name_of is not None and component is not None:
            return self.st_name_of(component)
        return self.st_name

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

DIVERTER_COMMANDS: Fields = (bits("enable"), [(ByteType.FLOAT, "commandSpeed"), (ByteType.FLOAT, "commandAngle")])
DIVERTER_FEEDBACK: Fields = (bits("active", "present"), [(ByteType.UINT32, "presentCode"),
                                                         (ByteType.UINT16, "diverted")])


def wheel_rows(d) -> int:
    """How many rows of wheels a diverter steers one by one (1: the whole deck as one): with row
    control, a row per wheel along the travel (equipment_geometry.wheel_columns)."""
    if getattr(d, "diverterType", "") != "wheels" or not getattr(d, "rowControl", False):
        return 1
    import equipment_geometry
    return len(equipment_geometry.grid_of(d)[0])


def _diverter_fields(d, direction: str) -> Fields:
    """A deck of rows: enable, then each row's speed and angle (ARRAYs on the PLC side)."""
    if direction == "out":
        return DIVERTER_FEEDBACK
    n = wheel_rows(d)
    if n == 1:
        return DIVERTER_COMMANDS
    return (bits("enable"), [(ByteType.FLOAT, f"rowSpeeds[{i}]") for i in range(n)]
            + [(ByteType.FLOAT, f"rowAngles[{i}]") for i in range(n)])


def _diverter_struct(d) -> str:
    n = wheel_rows(d)
    return "Diverter" if n == 1 else f"WheelDeck{n}"


register(DeviceKind(
    "diverters", cp.Diverter, "Diverter",
    commands=DIVERTER_COMMANDS, feedback=DIVERTER_FEEDBACK,
    runtime=("wheelAngle", "rowSpeeds", "rowAngles", "rowWheelAngles"),   # the window draws the wheels so
    fields_of=_diverter_fields, st_name_of=_diverter_struct,
    doc="A pusher / pop-up wheel diverter: enable, a speed and a direction (deg to the travel); "
        "a deck of wheel rows: a speed and a direction per row."))

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


# A PLC value's data type -> its wire type (ST: BOOL, BYTE, UINT, UDINT, ULINT, REAL).
VALUE_TYPES = {"BOOL": ByteType.BIT, "BYTE": ByteType.BYTE, "UINT": ByteType.UINT16, "UDINT": ByteType.UINT32,
               "ULINT": ByteType.UINT64, "REAL": ByteType.FLOAT}


def _value_fields(value, direction: str) -> Fields:
    if direction == "out":
        return NONE
    wire = VALUE_TYPES[value.dataType]
    return (bits("value"), []) if wire is ByteType.BIT else ([], [(wire, "value")])


def _value_struct(value) -> str:
    """PlcValue (a REAL, as before types), PlcValueUdint, PlcValueBool, ..."""
    return "PlcValue" if value.dataType == "REAL" else "PlcValue" + value.dataType.capitalize()


register(DeviceKind(
    "plcValues", cp.PlcValue, "PlcValue",
    commands=([], [(ByteType.FLOAT, "value")]),
    fields_of=_value_fields, st_name_of=_value_struct,
    doc="A value the PLC shows and plots, of its data type: drives nothing."))


register(DeviceKind(
    "watchdogs", cp.LinkWatchdog, "LinkWatchdog",
    commands=([], [(ByteType.UINT16, "plcHeartbeat"), (ByteType.UINT16, "simHeartbeatEcho")]),
    feedback=(bits("plcAlive"), [(ByteType.UINT16, "simHeartbeat"), (ByteType.FLOAT, "latencyMs")]),
    runtime=("tripped", "trips", "latencyMaxMs"),
    doc="The PLC link's heartbeat both ways: the simulator stops the PLC's commands when the PLC's stops."))


register(DeviceKind(
    "trackedLoads", cp.TrackedLoadView, "TrackedLoad",
    commands=(bits("active"), [(ByteType.UINT32, "loadId"), (ByteType.FLOAT, "loadX"), (ByteType.FLOAT, "loadY"),
                               (ByteType.FLOAT, "loadYaw"), (ByteType.FLOAT, "loadLength"),
                               (ByteType.FLOAT, "loadWidth"), (ByteType.UINT16, "frame")]),
    feedback=(bits("matched"), [(ByteType.FLOAT, "alongError"), (ByteType.FLOAT, "lateralError")]),
    runtime=("lengthError", "widthError"),
    doc="A load the PLC tracks: checked against the parcels; its tags plotted in the 3D page."))
