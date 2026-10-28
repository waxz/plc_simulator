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

class Sink(BaseModel):
    id: str
    x: float
    y: float
    width: float
    height: float
    label: str

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
