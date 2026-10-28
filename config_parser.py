import json
from typing import List, Optional, Literal, Dict, Any, Tuple, Union
from pydantic import BaseModel, Field, PrivateAttr


class DriveConfig(BaseModel):
    """Motor/VFD model for one belt (see drive.py). A belt without one is an
    ideal drive: its surface speed equals the commanded `Belt.speed` instantly,
    exactly the pre-drive behavior."""
    accel: float = 2.0        # m/s^2, ramp-up limit
    decel: float = 2.0        # m/s^2, ramp-down limit
    tau: float = 0.05         # s, first-order lag after the ramp (0 = none)
    speedMax: float = 5.0     # m/s, |actual speed| clamp
    countsPerM: float = 10000.0  # encoder resolution
    # Overload: parcels on the belt heavier than maxLoad (kg) for longer than
    # overloadTime (s) trip the drive (fault, motor coasts down) until the
    # run command is removed. None = no overload protection.
    maxLoad: Optional[float] = None
    overloadTime: float = 1.0
    # Feedback status: atSpeed when |actual - setpoint| <= atSpeedTolerance
    # (m/s) while running; running when |actual| > runningThreshold (m/s).
    atSpeedTolerance: float = 0.05
    runningThreshold: float = 0.01


class PathNode(BaseModel):
    """An anchor point of a path belt's centerline, in the belt's local
    frame (metres, rotated by Belt.rotation, offset by Belt.x/y). `bulge`
    shapes the segment from this node to the next, as in DXF polylines:
    0 = straight, tan(sweep/4) = arc (positive turns left / CCW,
    1 = semicircle)."""
    x: float
    y: float
    bulge: float = 0.0


class Belt(BaseModel):
    id: str
    # "path": a drawn belt - centerline through `path` anchors (lines and
    # arcs), offset by beltWidth on both sides (cad/path.py).
    type: Literal["linear", "curved", "path"]
    shape: Literal["quadrilateral", "arc", "rectangle", "path"] = "rectangle"
    x: float
    y: float
    length: float = 0.0
    beltWidth: float
    rotation: float
    radius: float = 0.0
    startAngle: float = 0.0
    endAngle: float = 0.0
    speed: float
    directionAngle: float
    direction: int = 1
    color: str
    vertexs: Optional[List[Dict[str, float]]] = None
    path: Optional[List[PathNode]] = None
    closed: bool = False      # path belt: last anchor joins the first (a loop conveyor)
    # Path belt: every sharp corner between two straight segments is rounded
    # with this radius when the geometry is built (cad/path.belt_local_path),
    # so `path` keeps the corners you drew and moving one re-rounds it.
    cornerRadius: float = 0.0
    speedBinding: Optional[str] = None
    directionAngleBinding: Optional[str] = None
    directionBinding: Optional[str] = None
    transition_momentum_duration: float = 0.6
    # How the PLC commands the motor (devices.commanded_speed):
    #   "speed" - `speed` is the signed setpoint (negative = reverse);
    #             on the wire: speed + directionAngle REALs
    #   "vfd"   - like a frequency converter: `run` and `reverse` command
    #             bits plus `speed` as the magnitude; on the wire: run BOOL,
    #             reverse BOOL (with the other bits), speed REAL
    commandFormat: Literal["speed", "vfd"] = "speed"
    run: bool = False
    reverse: bool = False
    # `speed` is the commanded setpoint (written by the PLC). The drive model
    # turns it into `actualSpeed`, which is what physics uses.
    drive: Optional[DriveConfig] = None
    friction: float = 0.5     # belt/parcel friction coefficient (friction traction)
    # Floor level (0 = ground, 1 = the level above, ...): a belt moves only
    # what is on its level. An elevator's platform belt changes level with
    # the platform (-1 while travelling: it moves nothing).
    level: int = 0
    # Runtime outputs (drive.py) - the motor feedback: actualSpeed (FLOAT),
    # encoderCount (UINT32) and the status bits running, atSpeed, reversing,
    # fault. Never saved into layout files.
    actualSpeed: float = Field(default=0.0, exclude=True)
    encoderCount: int = Field(default=0, exclude=True)
    running: bool = Field(default=False, exclude=True)
    atSpeed: bool = Field(default=False, exclude=True)
    reversing: bool = Field(default=False, exclude=True)
    fault: bool = Field(default=False, exclude=True)
    load: float = Field(default=0.0, exclude=True)       # kg of parcels on the belt
    _overload_s: float = PrivateAttr(default=0.0)
    _drive_ref: float = PrivateAttr(default=0.0)
    _encoder_frac: float = PrivateAttr(default=0.0)
    _safety_stop: bool = PrivateAttr(default=False)     # a safety circuit holds it stopped
    _zone_hold: bool = PrivateAttr(default=False)       # an accumulation zone holds its parcel
    _geom: Optional[tuple] = PrivateAttr(default=None)  # surface.py geometry cache


# How a component's on/off (or multi-bit) state is packed onto the wire by
# the auto-generating mapping builders (paddng_component/
# custom_component_mapping in mqtt_message_binding.py) - "bit" (the
# default) is the original behavior: one BIT entry per state bit, packed
# 8-to-a-byte with padding auto-inserted where needed. Any other value
# packs the WHOLE value into a single compact register instead ("word" is
# conventionally a 16-bit PLC register, hence "uint16" here) - which one
# fits depends on how many bits the component actually needs (see
# mqtt_message_binding.sensor_state_bit_count and
# verify_component_list's width guard). This is a property of the
# component's own declared config, not a per-channel authoring choice -
# nothing outside config_parser.py/mqtt_message_binding.py decides it.
PackFormat = Literal["bit", "byte", "uint16", "uint32", "float"]


class Sensor(BaseModel):
    id: str
    x: float
    y: float
    width: float
    height: float
    rotation: float
    isActive: bool = False
    label: str
    color: str
    stateBinding: Optional[str] = None
    sensorType: Literal["ir", "laser_banner"] = "ir"
    laserCount: Optional[int] = None
    beamDistance: Optional[float] = None
    wordBinding: Optional[str] = None
    detectedWord: Optional[int] = 0
    sensor_states: Optional[Union[bool, int, str]] = None
    packFormat: PackFormat = "bit"
    # Signal conditioning, like a real sensor's timer/output settings:
    #   onDelay  - the output switches on only after the beam has been
    #              blocked this long (s): ignores flicker, gaps in a parcel
    #   offDelay - the output stays on this long after the beam clears (s):
    #              stretches short pulses so a slow PLC scan still sees them
    #   invert   - light-on / NC output: 1 while the beam is clear, 0 when
    #              blocked (the default is dark-on: 1 when blocked)
    # For a laser banner they apply to each beam.
    onDelay: float = 0.0
    offDelay: float = 0.0
    invert: bool = False
    level: int = 0            # sees only what is on this floor level
    # Runtime: detections (beam(s) becoming blocked, after onDelay) since
    # start - wraps when packed - so the PLC can count a pulse shorter than
    # the exchange interval; blockedBeams = beams blocked now. Never saved.
    risingEdges: int = Field(default=0, exclude=True)
    blockedBeams: int = Field(default=0, exclude=True)
    _filter: Optional[list] = PrivateAttr(default=None)  # per beam: [output, timer]

class Source(BaseModel):
    id: str
    x: float
    y: float
    rotation: float
    interval: float
    minWidth: float
    maxWidth: float
    minHeight: float
    maxHeight: float
    label: str
    colorScheme: Optional[List[str]] = None
    triggerBinding: Optional[str] = None
    intervalBinding: Optional[str] = None
    minMass: float = 1.0      # kg, parcel mass range (friction traction)
    maxMass: float = 1.0
    level: int = 0            # floor level of the parcels it makes
    # Parcel identity (logistics.py): each parcel gets the next barcode, a
    # destination drawn from `destinations` (weighted by
    # `destinationWeights`, equal when empty; none when `destinations` is
    # empty) and, with probability `noReadRate`, an unreadable label.
    destinations: List[int] = Field(default_factory=list)
    destinationWeights: List[float] = Field(default_factory=list)
    noReadRate: float = 0.0
    barcodeStart: int = 1000001

class Sink(BaseModel):
    """Where parcels leave the layout. With a `capacity` it is a chute /
    bin (logistics.py): it holds that many, then reports `full` and parcels
    back up in front of it until it is emptied - after `emptyTime` s by
    itself (0 = only on command), or by the PLC's `empty` (rising edge).
    Only chutes (capacity > 0) are on the PLC exchange."""
    id: str
    x: float
    y: float
    width: float
    height: float
    label: str
    capacity: int = 0
    emptyTime: float = 0.0
    empty: bool = False                                  # command
    count: int = Field(default=0, exclude=True)          # feedback: parcels in it now
    full: bool = Field(default=False, exclude=True)

class ControlButton(BaseModel):
    id: str
    x: float
    y: float
    width: float = 0.6
    height: float = 0.4
    rotation: float = 0.0
    label: str
    color: str = "#3b82f6"
    sensor_states: Optional[Union[bool, int, str]] = None
    stateBinding: Optional[str] = None
    packFormat: PackFormat = "bit"
    # "momentary": pressed while held (push button); "latching": each press
    # toggles (selector switch, emergency stop) - see devices.py.
    buttonType: Literal["momentary", "latching"] = "momentary"
    # NC contact (typical for STOP / emergency stop): the signal is 1 while
    # NOT pressed, 0 when pressed, so a broken wire reads as "stop".
    normallyClosed: bool = False
    # Keyboard key that presses it in the simulator window (e.g. "1", "q").
    key: Optional[str] = None
    pressed: bool = Field(default=False, exclude=True)  # runtime: the operator's action

class LedIndicator(BaseModel):
    id: str
    x: float
    y: float
    radius: float = 0.2
    label: str
    color: str = "#22c55e"
    isOn: bool = False
    stateBinding: Optional[str] = None
    # "bit": the PLC switches it on/off (isOn). "byte": the PLC sends a mode
    # (`mode`): 0 off, 1 on, 2 blink slow (1 Hz), 3 blink fast (4 Hz); the lamp
    # (isOn) follows it in the simulator (devices.update_led).
    packFormat: PackFormat = "bit"
    mode: int = Field(default=0, exclude=True)

# ---------------------------------------------------------------------------
# Machines (machines.py runs them in the simulation). Each one either runs
# itself (control "auto": a small built-in controller, so a layout works
# without a PLC) or follows the PLC's command fields (control "plc") - its
# feedback fields go to the PLC either way.
# ---------------------------------------------------------------------------

ControlMode = Literal["auto", "plc"]


class Pallet(BaseModel):
    """A carrier that rides the belts like a parcel (the same belt drive,
    sensors and collisions) and is never taken by a sink. Parcels put on it -
    by a robot arm, or made by a source right above it - ride along as its
    load; a sink it crosses takes the load and leaves the pallet. `x`, `y`,
    `rotation` are where it starts (and returns to on Reset)."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 0.8        # m, along its travel (local x)
    height: float = 0.6       # m, across
    mass: float = 8.0         # kg, empty
    capacity: int = 1         # loads it carries at most
    label: str = ""
    color: str = "#b45309"
    level: int = 0
    # Runtime: loads on it now. Never saved.
    loadCount: int = Field(default=0, exclude=True)


class Stopper(BaseModel):
    """A pop-up stop at a belt: while up it is a bar across the belt that
    holds a pallet (or parcel) - typically before an intersection, a robot
    or an elevator. `rotation` is the direction of travel it stops; the bar
    is `width` across it and `depth` deep along it, centred at (x, y).

    Command (PLC -> simulator, control "plc"): `release` - 1 lowers it.
    Feedback (simulator -> PLC): `isUp` (actual position, after
    `actuationTime`) and `occupied` (something waiting at it).
    control "auto": it holds whatever arrives for `holdTime` s, then lowers
    for `releaseTime` s."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    depth: float = 0.05
    label: str = ""
    color: str = "#dc2626"
    level: int = 0
    control: ControlMode = "auto"
    actuationTime: float = 0.15
    holdTime: float = 2.0
    releaseTime: float = 1.0
    release: bool = False                                  # command
    isUp: bool = Field(default=True, exclude=True)         # feedback
    occupied: bool = Field(default=False, exclude=True)    # feedback


class RobotArm(BaseModel):
    """A 6-axis industrial robot (FANUC M-10iA / M-20iA style, robot6.py) on
    a pedestal at (x, y), with a gripper pointing down. Each pick-and-place
    cycle (program 1) it moves above the pick point, straight down, grips
    the parcel there (a pallet's load first, else a parcel on the belt),
    lifts it, carries it over the place point, sets it down - onto a pallet
    with room there as its load, else onto the belt / floor - and goes back
    to its perch above the middle. Both points must be within `reach`. One
    cycle takes `cycleTime` s at `speedOverride` 100 % (reach, carry,
    return: a third each).

    `placeTarget` "pallet": it only sets parts down on a pallet (waits for
    one with room at the place point); "any": on a pallet if there is one,
    else wherever the place point is.

    PLC interface (control "plc"), like a FANUC robot's UOP signals:
      commands  `enable` (ENBL), `start` (a program begins when it turns
                on), `hold` (*HOLD: motion pauses), `faultReset`,
                `program` (BYTE, PNS: 1 pick and place, 2 go to the perch,
                3 move the tool to targetX/Y/Z/Yaw), `speedOverride` (BYTE,
                1..100 %), `gripperClose` (grip / let go where the tool is,
                outside program 1) and the target (REALs, world metres,
                yaw deg)
      feedback  `ready` (CMDENBL: enabled, no fault, idle), `busy`,
                `atHome` (ATPERCH), `inPosition`, `holding`, `partAtPick`,
                `placeReady`, `fault` + `faultCode` (BYTE: 1 out of reach,
                2 joint limit), `cycles` (UINT16, wraps), the joints j1..j6
                (deg) and the tool centre point tcpX/Y/Z (m) as REALs
    control "auto": a cycle starts whenever a part is at the pick point and
    the place point is ready. Scripts drive it through
    machines.MachineRuntime.robot(id) (RobotHandle: move_to, grip, ...)."""
    id: str
    x: float
    y: float
    pickX: float
    pickY: float
    placeX: float
    placeY: float
    reach: float = 2.5
    cycleTime: float = 3.0
    pickRadius: float = 0.4   # m: how close to the pick point a part's centre must be
    label: str = ""
    color: str = "#7c3aed"
    level: int = 0
    pickLevel: int = 0
    placeLevel: int = 0
    placeTarget: Literal["any", "pallet"] = "any"
    control: ControlMode = "auto"
    mountHeight: float = 0.3  # m: pedestal under the arm
    toolLength: float = 0.18  # m: flange to the gripper pads (TCP)
    # Commands (PLC -> robot)
    start: bool = False
    enable: bool = True
    hold: bool = False
    faultReset: bool = False
    gripperClose: bool = False
    program: int = 1
    speedOverride: int = 100
    targetX: float = 0.0
    targetY: float = 0.0
    targetZ: float = 1.2
    targetYaw: float = 0.0
    # Feedback (robot -> PLC)
    busy: bool = Field(default=False, exclude=True)
    holding: bool = Field(default=False, exclude=True)
    partAtPick: bool = Field(default=False, exclude=True)
    placeReady: bool = Field(default=False, exclude=True)
    cycles: int = Field(default=0, exclude=True)
    ready: bool = Field(default=False, exclude=True)
    atHome: bool = Field(default=False, exclude=True)
    inPosition: bool = Field(default=True, exclude=True)
    fault: bool = Field(default=False, exclude=True)
    faultCode: int = Field(default=0, exclude=True)
    j1: float = Field(default=0.0, exclude=True)
    j2: float = Field(default=0.0, exclude=True)
    j3: float = Field(default=0.0, exclude=True)
    j4: float = Field(default=0.0, exclude=True)
    j5: float = Field(default=90.0, exclude=True)
    j6: float = Field(default=0.0, exclude=True)
    tcpX: float = Field(default=0.0, exclude=True)
    tcpY: float = Field(default=0.0, exclude=True)
    tcpZ: float = Field(default=0.0, exclude=True)
    # Runtime pose for drawing: the tool (gripper) position.
    toolX: float = Field(default=0.0, exclude=True)
    toolY: float = Field(default=0.0, exclude=True)
    # Runtime: the cycle's phase ("idle", "to_pick", "to_place", "home") and
    # how far through it (0..1) - the 3D view lifts the tool from these.
    phase: str = Field(default="idle", exclude=True)
    phaseT: float = Field(default=0.0, exclude=True)


class Elevator(BaseModel):
    """A lift between floor levels: a platform with its own conveyor - the
    belt `beltId`, an ordinary belt the elevator carries (put it on the
    elevator's footprint; the PLC or the auto mode runs it like any belt).
    Whatever is on the platform travels with it and arrives on the new
    level. `levels` floors (0 .. levels-1), `levelHeight` m apart, moving
    at `liftSpeed` m/s.

    Command (control "plc"): `targetLevel` (BYTE).
    Feedback: `level` (the floor it is at, or last left), `moving`,
    `atLevel` (stopped at a floor) and `occupied` (something on the
    platform).
    control "auto": loaded at `autoFrom` it carries the load to `autoTo`,
    runs its belt until the platform is empty, and goes back."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.4        # m, along the platform conveyor
    height: float = 1.0       # m, across
    beltId: Optional[str] = None
    levels: int = 2
    levelHeight: float = 1.0
    liftSpeed: float = 0.5
    label: str = ""
    color: str = "#0891b2"
    control: ControlMode = "auto"
    autoFrom: int = 0
    autoTo: int = 1
    targetLevel: int = 0                                   # command
    level: int = Field(default=0, exclude=True)            # feedback
    moving: bool = Field(default=False, exclude=True)
    atLevel: bool = Field(default=True, exclude=True)
    occupied: bool = Field(default=False, exclude=True)
    position: float = Field(default=0.0, exclude=True)     # runtime: height in levels (0.5 = halfway)


# ---------------------------------------------------------------------------
# Logistics equipment (logistics.py runs them): identification, sorting,
# accumulation and safety. Like the machines above, each works by itself
# (control "auto") or follows the PLC (control "plc").
# ---------------------------------------------------------------------------

class Scanner(BaseModel):
    """An identification point over a belt: reads each parcel once as its
    centre enters the read zone (`width` along the travel, `height` across,
    `rotation` the travel direction). `scannerType`:
      "barcode"  a camera / laser scanner - parcels with an unreadable label
                 give a no-read
      "rfid"     a reader gate - always reads
      "dws"      a dimension-weight-scan station: reads like a barcode
                 scanner and also measures weight and size
    Feedback per read: `code` (UDINT barcode, 0 on a no-read), `destination`
    (BYTE, as the warehouse system would answer for that code), `noRead`,
    `readCount` (UINT, +1 per parcel: a new read when it changes), `present`
    (a parcel in the zone now) and, for "dws", `weight` (kg) and
    `measuredLength` / `measuredWidth` / `measuredHeight` (m)."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 0.4
    height: float = 1.0
    scannerType: Literal["barcode", "rfid", "dws"] = "barcode"
    label: str = ""
    color: str = "#0ea5e9"
    level: int = 0
    code: int = Field(default=0, exclude=True)
    destination: int = Field(default=0, exclude=True)
    noRead: bool = Field(default=False, exclude=True)
    readCount: int = Field(default=0, exclude=True)
    present: bool = Field(default=False, exclude=True)
    weight: float = Field(default=0.0, exclude=True)
    measuredLength: float = Field(default=0.0, exclude=True)
    measuredWidth: float = Field(default=0.0, exclude=True)
    measuredHeight: float = Field(default=0.0, exclude=True)


class Diverter(BaseModel):
    """Sends parcels off the belt onto a branch: while it is active, parcels
    in its zone (`width` along the travel, `height` across, `rotation` the
    travel) are steered at `divertAngle` degrees to the travel - +90 / -90
    for a pusher (left / right), about +-30 for pop-up wheels - at `speed`.

    control "auto": active while a readable parcel for `destination` is in
    the zone (a sorter's own tracking). control "plc": active while the
    PLC's `divert` is on. Feedback: `active`, `present` (a parcel in the
    zone), `presentCode` (UDINT, its barcode; 0 for a no-read) and
    `diverted` (UINT count)."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    height: float = 1.0
    diverterType: Literal["pusher", "wheels"] = "pusher"
    divertAngle: float = 90.0
    speed: float = 1.0
    destination: int = 1
    label: str = ""
    color: str = "#f97316"
    level: int = 0
    control: ControlMode = "auto"
    divert: bool = False                                     # command
    active: bool = Field(default=False, exclude=True)        # feedback
    present: bool = Field(default=False, exclude=True)
    presentCode: int = Field(default=0, exclude=True)
    diverted: int = Field(default=0, exclude=True)


class Turntable(BaseModel):
    """A turntable / 90-degree transfer: a round platform carrying its own
    belt (`beltId`, an ordinary straight belt on the table) that turns
    between `angles` (degrees added to the belt's own heading) at
    `turnSpeed` deg/s. Whatever is on the platform turns with it.

    Command (control "plc"): `targetIndex` (BYTE, into `angles`).
    Feedback: `index`, `turning`, `inPosition`, `occupied`.
    control "auto": at `autoFrom` it runs its belt until a parcel is on
    board, turns to `autoTo`, runs the belt until the table is empty, and
    turns back."""
    id: str
    x: float
    y: float
    radius: float = 0.8
    beltId: Optional[str] = None
    angles: List[float] = Field(default_factory=lambda: [0.0, 90.0])
    turnSpeed: float = 90.0
    label: str = ""
    color: str = "#64748b"
    level: int = 0
    control: ControlMode = "auto"
    autoFrom: int = 0
    autoTo: int = 1
    targetIndex: int = 0                                     # command
    index: int = Field(default=0, exclude=True)              # feedback
    turning: bool = Field(default=False, exclude=True)
    inPosition: bool = Field(default=True, exclude=True)
    occupied: bool = Field(default=False, exclude=True)
    angle: float = Field(default=0.0, exclude=True)          # runtime: the table's turn now (deg)


class Accumulator(BaseModel):
    """A zero-pressure accumulation conveyor: the belts `beltIds` (in flow
    order) are its zones, each with a photo-eye at its end. A zone runs
    while it is empty, or while the zone after it is free ("singulation":
    one parcel per zone, gaps kept) or moving ("train": a held queue moves
    off together). The last zone discharges while `release` is on.

    Command (control "plc"): `release`. control "auto": always released.
    Feedback: `zones` (UINT bit mask, bit 0 = first zone, set = occupied),
    `full` (every zone occupied)."""
    id: str
    x: float
    y: float
    beltIds: List[str] = Field(default_factory=list)
    mode: Literal["singulation", "train"] = "singulation"
    label: str = ""
    color: str = "#22c55e"
    control: ControlMode = "auto"
    release: bool = True                                     # command
    zones: int = Field(default=0, exclude=True)              # feedback
    full: bool = Field(default=False, exclude=True)


class GuardDoor(BaseModel):
    """A door in a safety fence: the operator opens and closes it (click it
    in the simulator or the 3D view). The PLC can lock it shut (`lock`).
    Feedback: `closed`, `locked`. A safety circuit that lists it trips when
    it opens."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    label: str = ""
    color: str = "#facc15"
    lock: bool = False                                       # command
    closed: bool = Field(default=True, exclude=True)         # feedback
    locked: bool = Field(default=False, exclude=True)


class SafetyCircuit(BaseModel):
    """A safety relay: it stops the belts `beltIds` (every belt when empty)
    - whatever the PLC commands - when one of its emergency stops
    (`stopButtonIds`, buttons: pressed = trip) is pressed, a guard door
    (`doorIds`) opens or a light curtain (`curtainIds`, sensors) is
    interrupted. The belts stay stopped until every cause is clear and it is
    reset: the PLC's `reset` or the operator's reset button
    (`resetButtonId`), rising edge. `x`, `y` place its symbol.
    Feedback: `ok` (the belts may run), `tripped` (waiting for a reset)."""
    id: str
    x: float
    y: float
    stopButtonIds: List[str] = Field(default_factory=list)
    doorIds: List[str] = Field(default_factory=list)
    curtainIds: List[str] = Field(default_factory=list)
    beltIds: List[str] = Field(default_factory=list)
    resetButtonId: Optional[str] = None
    label: str = ""
    color: str = "#eab308"
    reset: bool = False                                      # command
    ok: bool = Field(default=True, exclude=True)             # feedback
    tripped: bool = Field(default=False, exclude=True)


class TopicComponent(BaseModel):
    id: str
    type: Literal["belt", "sensor", "item", "laser_banner", "button", "led"]

class TopicConfig(BaseModel):
    id: str
    topic: str
    interval: int
    direction: Literal["in", "out"]
    format: Literal["json", "raw", "string", "binary_struct"]
    endianness: Literal["LE", "BE"]
    components: List[TopicComponent] = Field(default_factory=list)

class MqttSettings(BaseModel):
    enabled: bool = False
    transport: Literal["mqtt", "tcp"] = "mqtt"
    brokerUrl: str = "ws://localhost"
    port: int = 5000
    topicPrefix: str = ""
    tcpBindHost: str = "0.0.0.0"
    tcpServerHost: str = "127.0.0.1"
    tcpPort: int = 5001
    endianness: Literal["LE", "BE"] = "LE"
    topics: List[TopicConfig] = Field(default_factory=list)
    # This simulator's own identity on the TCP frame protocol's Node ID
    # byte (see FrameType/ParsedFrame in tcp_framed_transport.py) - tags
    # every frame the simulator itself sends. A connecting client (e.g.
    # plc_mock.py) has its own separate node id, not this one.
    nodeId: int = 0
    # Publish belt actualSpeed/encoderCount as a BELT_FEEDBACK frame. Off by
    # default: a PLC that doesn't check the frame type byte would deserialize
    # this frame into its IO_MAPPING input struct.
    beltFeedback: bool = False
    beltFeedbackIntervalMs: int = 10

class BeltConnection(BaseModel):
    """Design-time joint: `fromBelt`'s outlet meets `toBelt`'s inlet, with
    matching heading (see cad/constraints.py). Kept when either belt moves in
    the layout editor. Physics ignores it - parcels transfer by geometry."""
    fromBelt: str
    toBelt: str


class BackgroundDrawing(BaseModel):
    """A DXF floor plan shown behind the layout in the editor (cad/dxf_io.py).
    `path` is relative to the layout file when possible. Only what passes the
    filters below is shown, snapped to and picked from."""
    path: str
    scale: float = 0.001      # metres per drawing unit (0.001 = drawing in mm)
    offsetX: float = 0.0      # metres, applied after scaling
    offsetY: float = 0.0
    visible: bool = True
    # Keep only this window of the drawing: (min_x, min_y, max_x, max_y) in
    # drawing units (before scale/offset), e.g. one sheet of a drawing set.
    clip: Optional[Tuple[float, float, float, float]] = None
    skipTypes: List[str] = Field(default_factory=list)     # "DIMENSION", "HATCH", "LEADER"
    hiddenLayers: List[str] = Field(default_factory=list)


class SketchEntity(BaseModel):
    """Construction geometry drawn in the layout editor (not a belt): a
    line/polyline whose segments may be arcs (`bulges`, DXF convention, one
    per point). Belts are made from it with Tools > Belt from Sketch/Drawing.
    World metres. Physics ignores it."""
    points: List[Tuple[float, float]]
    bulges: List[float] = Field(default_factory=list)
    closed: bool = False


class SimulatorConfig(BaseModel):
    version: str = "1.0.0"
    savedAt: int = 0
    belts: List[Belt] = Field(default_factory=list)
    sensors: List[Sensor] = Field(default_factory=list)
    sources: List[Source] = Field(default_factory=list)
    sinks: List[Sink] = Field(default_factory=list)
    buttons: List[ControlButton] = Field(default_factory=list)
    leds: List[LedIndicator] = Field(default_factory=list)
    pallets: List[Pallet] = Field(default_factory=list)
    stoppers: List[Stopper] = Field(default_factory=list)
    robots: List[RobotArm] = Field(default_factory=list)
    elevators: List[Elevator] = Field(default_factory=list)
    scanners: List[Scanner] = Field(default_factory=list)
    diverters: List[Diverter] = Field(default_factory=list)
    turntables: List[Turntable] = Field(default_factory=list)
    accumulators: List[Accumulator] = Field(default_factory=list)
    doors: List[GuardDoor] = Field(default_factory=list)
    safety: List[SafetyCircuit] = Field(default_factory=list)
    mqttSettings: MqttSettings = Field(default_factory=MqttSettings)
    connections: List[BeltConnection] = Field(default_factory=list)
    sketch: List[SketchEntity] = Field(default_factory=list)
    background: Optional[BackgroundDrawing] = None
    simulatorBackend: Literal["matter", "rapier"] = "matter"
    simulationSteps: int = 4
    kineticFriction: bool = False
    collisionEnabled: bool = True
    gridSnap: bool = True
    gridSize: float = 0.5
    useSurfaceVelocity: bool = False
    # "kinematic": legacy velocity override (useSurfaceVelocity) / impulse model.
    # "friction": parcels dragged by force-limited joints (traction.py).
    tractionModel: Literal["kinematic", "friction"] = "kinematic"
    floorFriction: float = 0.6
    # Fixed physics step (s). Sensors and drives are evaluated every step.
    physicsDt: float = 0.004
    maxStepsPerAdvance: int = 50
    # Runtime: simulated time in ns, mappable as clock/simNanoseconds. Never saved.
    simTimeNs: int = Field(default=0, exclude=True)

def load_config(file_path: str) -> SimulatorConfig:
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return SimulatorConfig(**data)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        config = load_config(sys.argv[1])
        print(f"Loaded config with {len(config.belts)} belts, {len(config.sensors)} sensors.")
