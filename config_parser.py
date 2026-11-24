import json
from typing import Annotated, List, Optional, Literal, Dict, Any, Tuple, Union
import re

from pydantic import BaseModel, BeforeValidator, Field, PrivateAttr, field_validator, model_serializer, model_validator


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
    # The drive train (drivetrain.py): inverter -> motor -> gearbox ->
    # chain / belt transmission -> drive drum or rollers. With gearRatio and
    # drumDiameter set, the surface speed follows from the motor speed:
    # v = f * motorRatedSpeed / motorRatedFrequency / (gearRatio *
    # transmissionRatio) / 60 * pi * drumDiameter. A belt commanded in Hz
    # (commandFormat "frequency") always has a train: missing values are
    # derived from its design speed (see drivetrain.train_of).
    motorRatedSpeed: float = 1450.0       # rpm at motorRatedFrequency (4-pole motor, rated slip)
    motorRatedFrequency: float = 50.0     # Hz
    gearRatio: Optional[float] = None     # gearbox i = motor rpm / output shaft rpm
    transmissionRatio: float = 1.0        # chain / belt after the gearbox (1 = drum on the gearbox shaft)
    drumDiameter: Optional[float] = None  # m, the drive drum (belt) or a roller (roller conveyor)
    maxFrequency: float = 50.0            # Hz, the inverter's output limit: limits the surface speed
    # The inverter's ramps, in seconds from 0 to motorRatedFrequency (as a
    # VFD sets them): with a train they replace accel / decel.
    rampUpTime: Optional[float] = None
    rampDownTime: Optional[float] = None
    # An encoder on the motor shaft, counts per revolution; None = a
    # measuring wheel on the surface, countsPerM.
    encoderCountsPerRev: Optional[int] = None
    # The motor's limits, on its shaft (drivetrain.motor_limits converts them
    # through the train to the surface, the hoist or the table): no faster
    # than motorMaxSpeed (rpm), no ramp steeper than motorMaxAccel (rpm/s),
    # and a belt's stop (setpoint 0: run off, speed 0, a hold, a safety
    # stop) brakes at motorMaxAccel - the quick stop - not on decel /
    # rampDownTime. They need a train (gearRatio and drumDiameter, a Hz
    # belt, or a machine's drive). None = no limit.
    motorMaxSpeed: Optional[float] = None
    motorMaxAccel: Optional[float] = None


class PathNode(BaseModel):
    """An anchor point of a path belt's centerline, in the belt's local
    frame (metres, rotated by Belt.rotation, offset by Belt.x/y). `bulge`
    shapes the segment from this node to the next, as in DXF polylines:
    0 = straight, tan(sweep/4) = arc (positive turns left / CCW,
    1 = semicircle)."""
    x: float
    y: float
    bulge: float = 0.0


# Where a component's command fields come from (its `control`):
#   "plc"    - the PLC data exchange writes them (the default): nothing
#              moves without a PLC (plc_mock.py's program or a real one)
#   "manual" - the operator sets them by hand (the simulator window's
#              inspector, PLC I/O); frames from the PLC leave them alone,
#              so a component can be tested on its own
# A sensor's "command" is its own output, forced by hand in "manual"
# (SensorControl). Layouts saved with the former "auto" (a built-in
# controller) load as "plc": that logic is the PLC program's now.
def _from_auto(value):
    return "plc" if value == "auto" else value


CommandSource = Annotated[Literal["plc", "manual"], BeforeValidator(_from_auto)]
SensorControl = Literal["sim", "manual"]


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
    transition_momentum_duration: float = 0.6   # handoff "blend" only: the seconds a parcel takes to follow
    # How the PLC commands the motor (devices.commanded_speed):
    #   "speed" - `speed` is the signed setpoint (negative = reverse);
    #             on the wire: speed + directionAngle REALs
    #   "vfd"   - like a frequency converter: `run` and `reverse` command
    #             bits plus `speed` as the magnitude; on the wire: run BOOL,
    #             reverse BOOL (with the other bits), speed REAL
    #   "frequency" - a real frequency inverter: `run`, `reverse` and
    #             `frequency` (Hz); the surface speed follows from the drive
    #             train (drivetrain.py, DriveConfig). `speed` is then the
    #             design speed the PLC converts to Hz. On the wire: run BOOL,
    #             reverse BOOL, frequency REAL; feedback adds outputFrequency
    #             (Hz) and motorSpeed (rpm)
    commandFormat: Literal["speed", "vfd", "frequency"] = "speed"
    # The drive's feedback to the PLC, in the input frame with every other
    # device's: running, atSpeed, reversing, fault (bits), actualSpeed (REAL,
    # m/s), encoderCount (UDINT); a "frequency" belt adds outputFrequency (Hz)
    # and motorSpeed (rpm). What a real drive or encoder reports - a PLC that
    # tracks loads by travel takes it from here. Off: the belt reports nothing.
    feedback: bool = False
    control: CommandSource = "plc"
    run: bool = False
    reverse: bool = False
    frequency: float = 0.0          # Hz, the "frequency" command (magnitude; `reverse` sets the sign)
    # `speed` is the commanded setpoint (written by the PLC). The drive model
    # turns it into `actualSpeed`, which is what physics uses.
    drive: Optional[DriveConfig] = None
    # Side guides (rails) along its open side edges (kinematic model, handoff
    # "friction"): a parcel reaching past such an edge over nothing is pushed
    # back on - not where another belt adjoins (a merge opening, an outlet).
    guides: bool = True
    friction: float = 0.5     # belt/parcel friction coefficient mu: the friction traction model's cap, and the
                              # kinematic model's hand-off (handoff "friction": a parcel's velocity follows the
                              # belt by at most mu g - a speed step slips dv^2 / 2 mu g; 0: no slip)
    # What carries the load: a belt, or driven rollers (pallets: a roller
    # conveyor, `rollerPitch` m between roller centres). It moves things
    # the same way; it is drawn as rollers (2D, designer, 3D).
    surface: Literal["belt", "roller"] = "belt"
    rollerPitch: float = 0.125
    # Floor level (0 = ground, 1 = the level above, ...): a belt moves only
    # what is on its level. An elevator's platform belt changes level with
    # the platform (-1 while travelling: it moves nothing).
    level: int = 0
    # Where loads enter / leave it, set by hand (None: from its shape, ax_geometry.py):
    # [x, y] in the belt's own frame (like vertexs: turned by rotation, offset by x / y),
    # so they move with it. What a PLC program's path (start -> end) is made from.
    inletPoint: Optional[List[float]] = None
    outletPoint: Optional[List[float]] = None
    # Runtime outputs (drive.py) - the motor feedback: actualSpeed (FLOAT),
    # encoderCount (UINT32) and the status bits running, atSpeed, reversing,
    # fault. Never saved into layout files.
    actualSpeed: float = Field(default=0.0, exclude=True)
    encoderCount: int = Field(default=0, exclude=True)
    running: bool = Field(default=False, exclude=True)
    atSpeed: bool = Field(default=False, exclude=True)
    reversing: bool = Field(default=False, exclude=True)
    fault: bool = Field(default=False, exclude=True)
    outputFrequency: float = Field(default=0.0, exclude=True)  # Hz, the inverter's ramped output (signed)
    motorSpeed: float = Field(default=0.0, exclude=True)       # rpm, from the surface speed through the train
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
    # "laser_banner": a head above the belt shining `laserCount` lasers
    # straight down, one every `beamDistance` m along its long side (across
    # the belt; `rotation` = the travel): a bit per beam, set while a parcel
    # is under it - the parcel's width, and over time its length / shape.
    sensorType: Literal["ir", "laser_banner"] = "ir"
    laserCount: Optional[int] = None
    beamDistance: Optional[float] = None
    wordBinding: Optional[str] = None
    detectedWord: Optional[int] = 0
    sensor_states: Optional[Union[bool, int, str]] = None
    packFormat: PackFormat = "bit"
    # "sim": the simulation decides the output; "manual": it stays as forced
    # by hand (the designer's Data Exchange section), parcels or not.
    control: SensorControl = "sim"
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

SOURCE_CLEAR_GAP_M = 0.2   # the automatic detection area: the largest parcel and this much each way


class Source(BaseModel):
    """Makes a parcel every `interval` s at (x, y), heading `rotation` - but
    only while its **detection area** is clear: a rectangle centred on the
    source, turned with it, `clearLength` m along its heading and
    `clearWidth` m across it. While any parcel (or pallet) of its level is
    in the area it waits, and makes the parcel as soon as the area clears
    (an interval shorter than the line can take no longer piles parcels up
    at the start of the belt). 0 = automatic: the largest parcel it makes,
    plus SOURCE_CLEAR_GAP_M before and after it along the heading."""
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
    clearLength: float = 0.0  # m, its detection area along its heading (0: the largest parcel + 2 x 0.2 m)
    clearWidth: float = 0.0   # m, ... across it (0: the largest parcel)

    def clear_area(self) -> Tuple[float, float]:
        """Its detection area (along, across its heading), m."""
        largest = max(self.maxWidth, self.maxHeight, self.minWidth, self.minHeight)
        along = self.clearLength if self.clearLength > 0 else largest + 2.0 * SOURCE_CLEAR_GAP_M
        across = self.clearWidth if self.clearWidth > 0 else largest
        return along, across

class Sink(BaseModel):
    """Where parcels leave the layout. With a `capacity` it is a chute /
    bin (logistics.py): it holds that many, then reports `full` and parcels
    back up in front of it until it is emptied - after `emptyTime` s by
    itself (0 = only on command), or by the PLC's `empty` (rising edge).
    Only chutes (capacity > 0) are on the PLC exchange.
    `fillMode` (a chute's): "count" - each load is counted and deleted,
    `full` at the capacity; "keep" - each load stays in the bin (set into
    its next slot), `full` at the capacity, the bin's loads removed when it
    is emptied; "delete" - each load is deleted (counted, the count wraps),
    never `full`."""
    id: str
    x: float
    y: float
    width: float
    height: float
    label: str
    capacity: int = 0
    emptyTime: float = 0.0
    fillMode: Literal["count", "keep", "delete"] = "count"
    control: CommandSource = "plc"
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
    # NC contact (typical for STOP / emergency stop): how the button is wired,
    # shown on the panel. The simulator applies it; the PLC gets the final
    # state either way (1 while pressed), see devices.contact.
    normallyClosed: bool = False
    # Keyboard key that presses it in the simulator window (e.g. "1", "q").
    key: Optional[str] = None
    pressed: bool = Field(default=False, exclude=True)  # runtime: the operator's action
    # The control panel this button is an element of (control_panel.py),
    # None for a button placed on its own. A panel's buttons are derived
    # from its elements and not saved separately.
    panel: Optional[str] = None

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
    control: CommandSource = "plc"
    mode: int = Field(default=0, exclude=True)
    # What it is: a pilot lamp, or a buzzer / horn (an output the PLC
    # switches the same way; the simulator shows it sounding).
    indicator: Literal["lamp", "buzzer"] = "lamp"
    panel: Optional[str] = None     # the control panel it belongs to, as ControlButton.panel


class PanelElement(BaseModel):
    """One device on a control panel: a push button, a pilot lamp or a
    buzzer. It becomes a ControlButton / LedIndicator with the id
    `<panel id>_<name>` (control_panel.py)."""
    kind: Literal["button", "lamp", "buzzer"] = "button"
    name: str                           # unique on the panel, a valid identifier (start, stop, run, ...)
    label: str = ""                     # the legend plate (the name in capitals when empty)
    color: str = "#22c55e"
    buttonType: Literal["momentary", "latching"] = "momentary"   # buttons
    normallyClosed: bool = False                                  # buttons: NC contact (STOP, E-STOP)
    key: Optional[str] = None                                     # buttons: keyboard key in the simulator
    packFormat: PackFormat = "bit"      # lamps: "byte" for blink modes

    @field_validator("name")
    @classmethod
    def _identifier(cls, name: str) -> str:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name or ""):
            raise ValueError(f"panel element name {name!r}: letters, digits and _ (a PLC identifier)")
        return name


class ControlPanel(BaseModel):
    """An operator control panel: an enclosure whose push buttons, lamps and
    buzzer are its `elements`, laid out on its face (control_panel.py)."""
    id: str
    x: float
    y: float
    width: float = 1.2
    height: float = 0.6
    rotation: float = 0.0
    label: str = ""
    color: str = "#475569"
    control: CommandSource = "plc"      # its lamps and buzzer: from the PLC or set by hand
    elements: List[PanelElement] = Field(default_factory=list)

    @field_validator("elements")
    @classmethod
    def _unique_names(cls, elements: List[PanelElement]) -> List[PanelElement]:
        names = [e.name for e in elements]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"panel elements named twice: {', '.join(duplicates)}")
        return elements

# ---------------------------------------------------------------------------
# Machines (machines.py runs them in the simulation). Each one follows the
# PLC's command fields (control "plc", the default: nothing moves without a
# PLC - plc_mock.py's program, plc_logic.py, or a real one) or the ones set
# by hand (control "manual", see CommandSource) - its feedback fields go to
# the PLC either way.
# ---------------------------------------------------------------------------

ControlMode = CommandSource


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
    plc_mock.py's program holds whatever arrives for `holdTime` s, then
    lowers it for `releaseTime` s."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    depth: float = 0.05
    label: str = ""
    color: str = "#dc2626"
    level: int = 0
    control: ControlMode = "plc"
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
    plc_mock.py's program starts a cycle when its pick sensor sees a part and
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
    # The IR sensor (a Sensor id) at the pick point that tells the PLC a part
    # is there - a robot has no part detection of its own on the PLC side.
    # The PLC program starts a cycle when it is blocked. None: the PLC
    # program uses the robot's own `partAtPick`.
    pickSensorId: Optional[str] = None
    label: str = ""
    color: str = "#7c3aed"
    level: int = 0
    pickLevel: int = 0
    placeLevel: int = 0
    placeTarget: Literal["any", "pallet"] = "any"
    control: ControlMode = "plc"
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
    elevator's footprint; the PLC runs it like any belt).
    Whatever is on the platform travels with it and arrives on the new
    level. `levels` floors (0 .. levels-1), `levelHeight` m apart, moving
    at `liftSpeed` m/s.

    With a `drive` (drivetrain.py) the lift is a hoist: motor, gearbox,
    chain and drum / sprocket; it travels at the train's speed at the rated
    frequency (`liftSpeed` is then the design speed missing train values
    are derived from), accelerating and braking on the inverter's ramps.

    Command (control "plc"): `targetLevel` (BYTE).
    Feedback: `level` (the floor it is at, or last left), `moving`,
    `atLevel` (stopped at a floor) and `occupied` (something on the
    platform).
    plc_mock.py's program: loaded at `autoFrom` it carries the load to
    `autoTo`, runs its belt until the platform is empty, and goes back."""
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
    control: ControlMode = "plc"
    autoFrom: int = 0
    autoTo: int = 1
    targetLevel: int = 0                                   # command
    level: int = Field(default=0, exclude=True)            # feedback
    moving: bool = Field(default=False, exclude=True)
    atLevel: bool = Field(default=True, exclude=True)
    occupied: bool = Field(default=False, exclude=True)
    position: float = Field(default=0.0, exclude=True)     # runtime: height in levels (0.5 = halfway)
    drive: Optional[DriveConfig] = None                    # the hoist's drive train (None: liftSpeed, no ramps)
    liftVelocity: float = Field(default=0.0, exclude=True)  # runtime: m/s, signed (up +)
    motorSpeed: float = Field(default=0.0, exclude=True)    # runtime: rpm, with a drive train


# ---------------------------------------------------------------------------
# Logistics equipment (logistics.py runs them): identification, sorting,
# accumulation and safety. Like the machines above, each follows the PLC
# (control "plc", the default) or commands set by hand ("manual").
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
    """Sends parcels onto a branch. Its zone: `width` along the travel,
    `height` across, `rotation` the travel; `divertAngle` the branch's
    direction (deg to the travel: +90 / -90 a pusher's left / right, about
    +-30 for wheels), `speed` its design speed.

    Three commands: `enable`, `commandSpeed` (m/s) and `commandAngle` (deg
    to the travel, + left; logistics.divert_angle).

    A pusher works over a belt: while `enable` is on it pushes what is in
    its zone at `commandSpeed`, at `commandAngle`; off, the belt carries it.

    Pop-up wheels ("wheels") are a conveyor of their own, with two motors:
    one turns the wheels (the speed: `commandSpeed`), one swivels them (the
    direction: `commandAngle`, at most 60 deg each way, 0 = straight on).
    The parcels in the zone always go the way the wheels point at the
    wheels' speed - straight through at angle 0, onto the branch at
    divertAngle - and `enable` off stops the wheels: what is on them stands
    (logistics.diverter_motion). Where they point now: `wheelAngle`.

    Row control (`rowControl`, wheels only): the deck is a two-dimensional
    array of wheels - rows across the travel, one after another along it
    (as many as the module has wheels along the travel:
    `wheelRows`, or fitted to the deck: 6 on a 1 m deck; row 0 first; each
    row's strip from midway to the row before to midway to the row after).
    The wheels: `wheelRows` rows of `wheelColumns` wheels at
    `wheelPitchAlong` / `wheelPitchAcross` m (0: fitted to the deck). Every wheel turns on its own motor; each row's
    wheels swivel together on one motor, mounted along the travel. Then the
    commands are `enable` and per row a speed and an angle (`rowSpeeds[i]`,
    `rowAngles[i]`), and a parcel goes the average way of the rows under it
    (logistics: its footprint sampled). Where each row points now:
    `rowWheelAngles`. The deck carries the parcels itself: no belt under it.

    At the simulation's start / reset: `commandSpeed` = `speed`,
    `commandAngle` = a pusher's divertAngle, the wheels' 0 (straight);
    plc_mock.py's program enables them and steers to divertAngle for a code
    tracked to `destination` (a layout saying `divert` for `enable` loads).
    `drive` (optional): the wheels' / the pusher's motor, as a machine's
    (drivetrain.py; the drum is the wheel or roller): the parcels go at
    `actualSpeed`, which ramps to `commandSpeed` while active and to 0 when
    not, within the motor's limits. Without one: at `commandSpeed` at once.
    Feedback: `active`, `present` (a parcel in the zone), `presentCode`
    (UDINT, its barcode; 0 for a no-read) and `diverted` (UINT count)."""
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
    control: ControlMode = "plc"
    enable: bool = False                                     # command: a pusher pushes, wheels turn
    # Where its zone begins and ends for the AX program (world x, y; None: the ends of its
    # box along its travel) - set in the designer by input, drag or a pick (ax_geometry.zone_ends)
    axStart: Optional[List[float]] = None
    axEnd: Optional[List[float]] = None
    commandSpeed: float = Field(default=0.0, exclude=True)   # command: m/s
    commandAngle: float = Field(default=0.0, exclude=True)   # command: deg to the travel (+ left)
    rowControl: bool = False                                 # wheels: a swivel motor per wheel row
    # the wheels (equipment_geometry.wheel_grid): rows along the travel (each across it), wheels in
    # a row, and their pitch along / across, m - 0: fitted to the deck
    wheelRows: int = 0
    wheelColumns: int = 0
    wheelPitchAlong: float = 0.0
    wheelPitchAcross: float = 0.0
    rowSpeeds: List[float] = Field(default_factory=list, exclude=True)    # command per row (rows > 1): m/s
    rowAngles: List[float] = Field(default_factory=list, exclude=True)    # command per row: deg to the travel
    rowWheelAngles: List[float] = Field(default_factory=list, exclude=True)   # runtime: where each row points
    rowActualSpeeds: List[float] = Field(default_factory=list, exclude=True)  # runtime: each row's speed
    drive: Optional[DriveConfig] = None                      # the wheels' / pusher's motor (None: instant)
    actualSpeed: float = Field(default=0.0, exclude=True)    # runtime: the speed the parcels go at (m/s)
    wheelAngle: float = Field(default=0.0, exclude=True)     # runtime: where pop-up wheels point (deg)
    active: bool = Field(default=False, exclude=True)        # feedback
    present: bool = Field(default=False, exclude=True)
    presentCode: int = Field(default=0, exclude=True)
    diverted: int = Field(default=0, exclude=True)

    @model_validator(mode="before")
    @classmethod
    def _divert_is_enable(cls, data):
        """Layouts written before the enable command: their `divert`."""
        if isinstance(data, dict) and "divert" in data:
            data = dict(data)
            data.setdefault("enable", data.pop("divert"))
        if isinstance(data, dict) and "rows" in data:             # a deck of rows, as first written
            data = dict(data)
            data.setdefault("rowControl", int(data.pop("rows") or 1) > 1)
        return data


class Turntable(BaseModel):
    """A turntable / 90-degree transfer: a round platform carrying its own
    belt (`beltId`, an ordinary straight belt on the table) that turns
    between `angles` (degrees added to the belt's own heading) at
    `turnSpeed` deg/s. Whatever is on the platform turns with it.

    Command (control "plc"): `targetIndex` (BYTE, into `angles`).
    Feedback: `index`, `turning`, `inPosition`, `occupied`.
    plc_mock.py's program: at `autoFrom` it runs its belt until a parcel is
    on board, turns to `autoTo`, runs the belt until the table is empty,
    and turns back."""
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
    control: ControlMode = "plc"
    autoFrom: int = 0
    autoTo: int = 1
    targetIndex: int = 0                                     # command
    index: int = Field(default=0, exclude=True)              # feedback
    turning: bool = Field(default=False, exclude=True)
    inPosition: bool = Field(default=True, exclude=True)
    occupied: bool = Field(default=False, exclude=True)
    angle: float = Field(default=0.0, exclude=True)          # runtime: the table's turn now (deg)
    # The slewing drive (drivetrain.py): motor, gearbox, pinion on the
    # slewing ring (transmissionRatio = ring teeth / pinion teeth). With it
    # the table turns at the drive's speed and ramps; turnSpeed is the design
    # speed missing values derive from.
    drive: Optional[DriveConfig] = None
    turnVelocity: float = Field(default=0.0, exclude=True)   # runtime: deg/s, signed
    motorSpeed: float = Field(default=0.0, exclude=True)     # runtime: rpm


class Transfer(BaseModel):
    """A right-angle (90 degree) transfer - the chain / belt transfer of a
    pallet line - at a corner or junction of roller conveyors: a pallet
    rolling in along `rotation` stops centred on it (when `stop`), chain
    strands lift it off the rollers and carry it at `crossAngle` degrees to
    its travel (+90 left, -90 right) at `speed` onto the crossing conveyor,
    without turning it, then drop. The zone is `width` along the travel and
    `height` across, centred at (x, y).

    It lifts and runs while `transfer` is on (plc_mock.py's program: once a
    pallet stands centred, until it has left; at a junction off for those
    that go straight - then set `stop` false).
    Feedback: `present` (something in the zone), `centered` (stopped on the
    transfer, ready), `up` (lifted), `transferred` (UINT count)."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    height: float = 1.0
    crossAngle: float = 90.0
    speed: float = 0.5
    liftTime: float = 0.4
    stop: bool = True
    label: str = ""
    color: str = "#a3a3a3"
    level: int = 0
    control: ControlMode = "plc"
    transfer: bool = False                                   # command
    present: bool = Field(default=False, exclude=True)       # feedback
    centered: bool = Field(default=False, exclude=True)
    up: bool = Field(default=False, exclude=True)
    transferred: int = Field(default=0, exclude=True)
    # The chain drive (drivetrain.py): motor, gearbox, chain sprocket
    # (drumDiameter). With it the chains carry at the drive's speed; `speed`
    # is the design speed missing values derive from.
    drive: Optional[DriveConfig] = None
    motorSpeed: float = Field(default=0.0, exclude=True)     # runtime: rpm, while carrying
    lift: float = Field(default=0.0, exclude=True)           # runtime: 0 down .. 1 up (drawing)


class Accumulator(BaseModel):
    """A zero-pressure accumulation conveyor: the belts `beltIds` (in flow
    order) are its zones, each with a photo-eye at its end. A zone runs
    while it is empty, or while the zone after it is free ("singulation":
    one parcel per zone, gaps kept) or moving ("train": a held queue moves
    off together). The last zone discharges while `release` is on.

    Command (control "plc"): `release` (plc_mock.py's program: while the
    line runs).
    Feedback: `zones` (UINT bit mask, bit 0 = first zone, set = occupied),
    `full` (every zone occupied)."""
    id: str
    x: float
    y: float
    beltIds: List[str] = Field(default_factory=list)
    mode: Literal["singulation", "train"] = "singulation"
    label: str = ""
    color: str = "#22c55e"
    control: ControlMode = "plc"
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
    control: CommandSource = "plc"
    lock: bool = False                                       # command
    closed: bool = Field(default=True, exclude=True)         # feedback
    locked: bool = Field(default=False, exclude=True)


# A PLC value's data types: what the exchange's wire carries (device_kinds.VALUE_TYPES).
PlcDataType = Literal["BOOL", "BYTE", "UINT", "UDINT", "ULINT", "REAL"]


class PlcValue(BaseModel):
    """A value the PLC shows: one it writes (`value`, in its output frame)
    that drives nothing - no motor, no lamp - of its `dataType` (BOOL,
    BYTE, UINT, UDINT, ULINT, REAL; its ST struct follows the type). Its
    tag monitors list it and the 3D page's charts plot it (over time, or two
    of them as an x/y trajectory). Use it for what the PLC computes: a
    tracked position, a controller's error, a state number, a scan time.
    The designer's Exchange Table edits them (with the tracked-load slots):
    a row's place is its place in the frame. `unit` and `decimals` are for
    showing it; `x`, `y` are no longer used (it is not drawn)."""
    id: str
    x: float
    y: float
    rotation: float = 0.0
    width: float = 1.0
    height: float = 0.35
    label: str = ""
    unit: str = ""
    decimals: int = 2
    color: str = "#a78bfa"
    level: int = 0
    control: CommandSource = "plc"
    dataType: PlcDataType = "REAL"
    value: Union[float, int, bool] = Field(default=0.0, exclude=True)   # command: the PLC's value


class LinkWatchdog(BaseModel):
    """The PLC link's heartbeat and watchdog, both ways (logistics.py).
    The PLC counts `plcHeartbeat` up every scan (UINT, wraps); the simulator
    counts `simHeartbeat` up every physics step - it stops while the
    simulation is paused or stalls - and reports `plcAlive` while the PLC's
    heartbeat moves. Once the PLC's heartbeat has moved (armed), it standing
    still for `timeoutMs` trips the watchdog: every command of the PLC is put
    at rest (belts stopped, bits off) until it moves again - as a drive's
    own communication watchdog stops it when its PLC is gone. The PLC
    watches `simHeartbeat` the same way (the plant is not advancing: hold).

    Latency: the PLC echoes `simHeartbeatEcho`, the simHeartbeat of the input
    its outputs were computed from. When they arrive the simulator is some
    steps further: `latencyMs` = (simHeartbeat - echo) steps x the step, in
    simulated ms - the age of the plant state the PLC's commands answer,
    the whole loop (the input frame's way, the PLC's scan, the output's way
    back). 0 until an echo comes. `latencyMaxMs`: the longest since start."""
    id: str
    x: float
    y: float
    width: float = 1.4
    height: float = 0.4
    label: str = ""
    color: str = "#22c55e"
    timeoutMs: int = 1000
    control: CommandSource = "plc"
    plcHeartbeat: int = Field(default=0, exclude=True)        # command: the PLC's counter
    simHeartbeatEcho: int = Field(default=0, exclude=True)    # command: the simHeartbeat its outputs are of
    simHeartbeat: int = Field(default=0, exclude=True)        # feedback: the simulator's counter
    plcAlive: bool = Field(default=False, exclude=True)       # feedback: the PLC's counter moves
    latencyMs: float = Field(default=0.0, exclude=True)       # feedback: the loop's age, simulated ms
    latencyMaxMs: float = Field(default=0.0, exclude=True)    # runtime: the longest
    tripped: bool = Field(default=False, exclude=True)        # runtime: the PLC's commands at rest
    trips: int = Field(default=0, exclude=True)               # runtime: how often


class TrackedLoadView(BaseModel):
    """One load the PLC tracks, as it shows it: a slot the PLC fills from its
    load table every scan (one per load it can show at a time). It drives
    nothing; the simulator checks it (logistics.py, tracking): against the
    parcel nearest it as the parcels were at the simulation step the PLC's
    values come from - `frame`, the watchdog's `simHeartbeat` in the input
    frame the PLC computed them from - so the exchange's delay is no error.
    It reports, per slot, the error along the load's travel and across it.
    Positions in the plant (m), the heading in degrees, as every angle on
    the exchange. Its tags are shown and plotted in the 3D page's tag
    monitor (view "Tracked loads"). `x`, `y` place it in the designer.

    Its `role` (the layout's, not on the exchange: the frame is the same for
    each) says what the box is, and how the simulator draws it ("Track boxes"
    in the 3D view, [B] in the window):
      load  a load the PLC tracks (checked against the parcels)
      slot  a slot on a merge line: one a load asked for, or was granted
            (drawn, never checked: no parcel is in it yet)
      sync  the load a sync section is driving into its slot (checked)"""
    id: str
    x: float
    y: float
    width: float = 1.0
    height: float = 0.3
    label: str = ""
    color: str = "#f472b6"
    level: int = 0
    control: CommandSource = "plc"
    role: Literal["load", "slot", "sync"] = "load"            # what the box is (drawn by it; slots not checked)
    active: bool = Field(default=False, exclude=True)         # command: the slot shows a load
    loadId: int = Field(default=0, exclude=True)              # command: the PLC's load id
    loadX: float = Field(default=0.0, exclude=True)           # command: its centre, m
    loadY: float = Field(default=0.0, exclude=True)
    loadYaw: float = Field(default=0.0, exclude=True)         # command: its travel's heading, deg
    loadLength: float = Field(default=0.0, exclude=True)      # command: along the travel, m
    loadWidth: float = Field(default=0.0, exclude=True)       # command: across it, m
    frame: int = Field(default=0, exclude=True)               # command: the simHeartbeat its values are of
    matched: bool = Field(default=False, exclude=True)        # feedback: a parcel was found for it
    alongError: float = Field(default=0.0, exclude=True)      # feedback: PLC - parcel along the travel, m
    lateralError: float = Field(default=0.0, exclude=True)    # feedback: PLC - parcel across it (+ left), m
    lengthError: float = Field(default=0.0, exclude=True)     # runtime: PLC - parcel length (its own box), m
    widthError: float = Field(default=0.0, exclude=True)      # runtime: PLC - parcel width, m


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
    control: CommandSource = "plc"
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
    # (What a device reports to the PLC is the device's own - a belt's drive
    # feedback is Belt.feedback - and travels in the I/O frames: these
    # settings are only the transport's.)
    # TCP exchange robustness (tcp_framed_transport.py), all opt-in:
    #   tcpLengthEndianness   byte order of the frame's length field (None:
    #                         the payload's `endianness`, as always)
    #   tcpRxTimeoutMs        close a client silent this long (0: never)
    #   tcpMaxPayload         don't send frames with a longer payload (0: no
    #                         limit) - the AX TcpClient's frames hold 4110
    #                         bytes: payload 4096 at most
    #   tcpOutputOwner        accept the PLC output frame (IO_MAPPING) only
    #                         from this node id (None: from any client)
    #   tcpRequireLayoutCheck accept it only from a connection whose
    #                         LAYOUT_CHECK matched
    tcpLengthEndianness: Optional[Literal["LE", "BE"]] = None
    tcpRxTimeoutMs: int = 0
    tcpMaxPayload: int = 0
    tcpOutputOwner: Optional[int] = None
    tcpRequireLayoutCheck: bool = False

class BeltConnection(BaseModel):
    """Design-time joint: `fromBelt`'s outlet meets `toBelt`'s inlet, with
    matching heading (see cad/constraints.py). Kept when either belt moves in
    the layout editor. Physics ignores it - parcels transfer by geometry."""
    fromBelt: str
    toBelt: str


class PlcTag(BaseModel):
    """Your entry for one PLC signal in the designer's tag table (io_table.py):
    `key` is the signal, "<component>/<signal>"; an empty `name` / `table` /
    `comment` keeps the generated one (the component id, the "Default tag
    table", the component's label)."""
    key: str
    name: str = ""
    table: str = ""
    comment: str = ""


class AxExport(BaseModel):
    """How this layout's PLC exchange is exported as SIMATIC AX ST
    (ax_export.py): set once - in the designer (File > Export to SIMATIC
    AX) or with st_export.py --save - and every later export, from the
    designer, st_export.py or the AX project's generate_st_serde.py, writes
    the same file the same way. Paths are relative to the layout file."""
    output: str = ""                # the .st file (e.g. ../ax_dev/src/demo/sim_exchange/sim_io.st)
    namespace: str = ""             # NAMESPACE around the ST ("" = none)
    prefix: str = ""                # Station_1: Sim_Input -> Station_1_Input, ...
    frames: List[str] = Field(default_factory=list)    # [] = every frame
    tags: bool = True               # the PLC tags STRUCT, ReadTags / WriteTags
    bitArrays: bool = False         # a laser banner's beams as ARRAY OF BOOL (else ARRAY OF BYTE)
    maxPayload: int = 4096          # refuse a bigger frame (the AX TcpClient's 4110-byte frames); 0 = no limit
    vectors: str = ""               # also an AxUnit golden-vector test file ("" = none)
    # Device kinds whose tags are arrays in the PLC tags STRUCT (plc_tags.tag_path): per wire
    # struct type an ARRAY of it, the device's index its place in its kind's list (layout
    # order), named in <prefix>_TagIndex - so a program can loop over them ("belts",
    # "trackedLoads"). [] = every tag a variable of its own.
    tagArrays: List[str] = Field(default_factory=list)


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
    transfers: List[Transfer] = Field(default_factory=list)
    doors: List[GuardDoor] = Field(default_factory=list)
    safety: List[SafetyCircuit] = Field(default_factory=list)
    controlPanels: List[ControlPanel] = Field(default_factory=list)
    plcValues: List[PlcValue] = Field(default_factory=list)  # values the PLC shows / plots (drive nothing)
    watchdogs: List[LinkWatchdog] = Field(default_factory=list)  # the PLC link's heartbeat both ways
    trackedLoads: List[TrackedLoadView] = Field(default_factory=list)  # loads the PLC tracks, drawn and checked
    mqttSettings: MqttSettings = Field(default_factory=MqttSettings)
    connections: List[BeltConnection] = Field(default_factory=list)
    plcTags: List[PlcTag] = Field(default_factory=list)     # tag names / tables / comments (io_table.py)
    axExport: Optional[AxExport] = None                      # SIMATIC AX ST export settings (ax_export.py)
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
    # How a kinematic parcel takes up a belt's velocity (physics.drive_toward_target):
    #   "friction"  at most the belt's friction x g (Coulomb friction: it
    #               slides until the speeds match, a speed step costs dv^2 / 2 mu g
    #               of slip, a joint between equal belts none; with the two-belt drive and side guides);
    #   "blend"     over the belt's transition_momentum_duration, restarted at
    #               every hand-off (the former model: a 0.6 s blend lags a faster
    #               belt by up to ~90 mm on short belts).
    #   "scaled"    the blend's smooth ease, split: the momentum across the new
    #               belt over its transition_momentum_duration (a side merge slides
    #               in as with "blend"), the speed along it over that x |dv along|
    #               / |v_belt| (a 15 % speed step ~0.08 s instead of 0.6; an equal
    #               belt none): the merges' look, little slip - the default
    handoff: Literal["friction", "blend", "scaled"] = "scaled"
    floorFriction: float = 0.6
    # Fixed physics step (s). Sensors and drives are evaluated every step.
    physicsDt: float = 0.004
    maxStepsPerAdvance: int = 50
    # Runtime: simulated time in ns, mappable as clock/simNanoseconds. Never saved.
    simTimeNs: int = Field(default=0, exclude=True)

    @model_validator(mode="after")
    def _expand_control_panels(self):
        """A control panel's buttons and lamps are derived from its elements:
        built here, whenever a layout is loaded (control_panel.py)."""
        if self.controlPanels or any(b.panel for b in self.buttons) or any(l.panel for l in self.leds):
            import control_panel
            control_panel.refresh(self)
        return self

    @model_serializer(mode="wrap")
    def _without_panel_children(self, handler):
        """Saved without the derived panel buttons / lamps: the panel's
        elements are the source; loading builds them again."""
        data = handler(self)
        if isinstance(data, dict):
            for key, models in (("buttons", self.buttons), ("leds", self.leds)):
                if key in data and len(data[key]) == len(models):
                    data[key] = [d for d, m in zip(data[key], models) if not m.panel]
            if not self.plcTags:
                data.pop("plcTags", None)          # untouched tags: nothing to save
            if self.axExport is None:
                data.pop("axExport", None)
        return data

def load_config(file_path: str) -> SimulatorConfig:
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return SimulatorConfig(**data)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        config = load_config(sys.argv[1])
        print(f"Loaded config with {len(config.belts)} belts, {len(config.sensors)} sensors.")
