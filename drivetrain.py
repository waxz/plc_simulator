"""
drivetrain.py - the mechanics between a conveyor's frequency inverter and its
surface: an induction motor, a gearbox, an optional chain / belt transmission,
and the drive drum (belt conveyor) or the rollers (roller conveyor).

    motor speed      n_motor = f * n_rated / f_rated          [rpm]  (V/f, rated slip)
    output shaft     n_out   = n_motor / (i_gear * i_trans)    [rpm]
    surface speed    v       = n_out / 60 * pi * d             [m/s]  (d: drum / roller diameter)

so the surface speed is proportional to the inverter frequency:
v = f * speed_per_hz. A PLC commanding a belt with commandFormat "frequency"
sends f (Hz), as to a real inverter, and computes it from the speed it wants
with the same mechanical data: frequency_for_speed().

Example: a 4-pole motor (1450 rpm at 50 Hz), gearbox i = 20, a 100 mm drum:
1450 / 20 = 72.5 rpm at the drum, * pi * 0.1 m / 60 s = 0.380 m/s at 50 Hz.

The data come from the belt's DriveConfig (config_parser.py): motorRatedSpeed,
motorRatedFrequency, gearRatio, transmissionRatio, drumDiameter,
maxFrequency. A "frequency" belt without gearRatio / drumDiameter gets a
default train: a 100 mm drum and the gear ratio that gives the belt's design
speed (`Belt.speed`, else 1 m/s) at the rated frequency.
"""
import math
from dataclasses import dataclass
from typing import Optional

DEFAULT_DRUM_DIAMETER = 0.1     # m
DEFAULT_DESIGN_SPEED = 1.0      # m/s at the rated frequency, when the belt has no speed of its own


@dataclass(frozen=True)
class Train:
    motor_rated_speed: float     # rpm at motor_rated_frequency
    motor_rated_frequency: float # Hz
    gear_ratio: float            # motor rpm / gearbox output rpm
    transmission_ratio: float    # gearbox output rpm / drum rpm (chain, belt; 1 = drum on the shaft)
    drum_diameter: float         # m
    max_frequency: float         # Hz, the inverter's output limit

    @property
    def speed_per_hz(self) -> float:
        """Surface speed (m/s) per Hz of inverter output."""
        return self.motor_speed(1.0) / (self.gear_ratio * self.transmission_ratio) / 60.0 * math.pi * self.drum_diameter

    @property
    def max_speed(self) -> float:
        """Surface speed (m/s) at the inverter's maximum frequency."""
        return self.speed_for_frequency(self.max_frequency)

    def motor_speed(self, frequency: float) -> float:
        """Motor speed (rpm) at this output frequency (Hz)."""
        return frequency * self.motor_rated_speed / self.motor_rated_frequency

    def output_speed(self, frequency: float) -> float:
        """Gearbox output shaft speed (rpm) at this output frequency."""
        return self.motor_speed(frequency) / self.gear_ratio

    def speed_for_frequency(self, frequency: float) -> float:
        """Surface speed (m/s) at this output frequency (Hz); the sign follows."""
        return frequency * self.speed_per_hz

    def frequency_for_speed(self, speed: float) -> float:
        """The output frequency (Hz) for this surface speed (m/s); the sign follows."""
        return speed / self.speed_per_hz

    def motor_speed_for_surface(self, speed: float) -> float:
        """Motor speed (rpm) while the surface runs at `speed` (m/s)."""
        return self.motor_speed(self.frequency_for_speed(speed))

    def turn_speed(self, frequency: float) -> float:
        """A rotary output (a turntable on its slewing ring): degrees per
        second of the driven part at this frequency."""
        return self.output_speed(frequency) / self.transmission_ratio * 6.0


_KIND_OF_CLASS = {}      # a pydantic model class -> its machine_kind (by its fields: fixed per class)


def _kind_by_fields(has) -> str:
    if has("diverterType"):
        return "diverter"
    if has("liftSpeed"):
        return "lift"
    if has("turnSpeed"):
        return "turntable"
    if has("crossAngle"):
        return "transfer"
    return "belt"


def machine_kind(component) -> str:
    """What the drive moves: "lift" (an elevator's hoist), "turntable" (a
    slewing drive, rotary), "transfer" (a transfer's chains), "diverter"
    (its wheels' or pusher's motor) or "belt".
    Called for every belt every physics step: decided once per model class
    (its fields), as hasattr on a pydantic model costs a raised and caught
    AttributeError per missing name."""
    cls = type(component)
    kind = _KIND_OF_CLASS.get(cls)
    if kind is None:
        fields = getattr(cls, "model_fields", None)
        if not isinstance(fields, dict):                 # not a pydantic model (a test's stand-in): ask it
            return _kind_by_fields(lambda name: hasattr(component, name))
        kind = _KIND_OF_CLASS[cls] = _kind_by_fields(lambda name: name in fields)
    return kind


def is_lift(component) -> bool:
    return machine_kind(component) == "lift"


def is_rotary(component) -> bool:
    """A turntable: its speed in degrees per second, no drum."""
    return machine_kind(component) == "turntable"


def design_speed(component) -> float:
    """The speed the component is designed for: a belt's or a transfer's
    `speed` and an elevator's `liftSpeed` (m/s), a turntable's `turnSpeed`
    (deg/s)."""
    kind = machine_kind(component)
    value = {"lift": "liftSpeed", "turntable": "turnSpeed"}.get(kind, "speed")
    return abs(float(getattr(component, value, 0.0) or 0.0))


def train_of(component, assume_drive: bool = False) -> Optional[Train]:
    """The component's drive train, or None. A belt has one with gearRatio
    and drumDiameter set, or when commanded in Hz; a machine (elevator,
    turntable, transfer, diverter) as soon as it has a `drive`. Missing
    values are derived from the design speed at the rated frequency.
    `assume_drive`: one without a drive yet, as it would be (the designer's
    display) - a belt commanded in m/s too, from its speed now (the designer
    pins that train when a drive value is first edited: pin_belt_train)."""
    drive = getattr(component, "drive", None)
    rotary = is_rotary(component)
    if drive is not None and drive.gearRatio and (drive.drumDiameter or rotary):
        return Train(drive.motorRatedSpeed, drive.motorRatedFrequency, drive.gearRatio,
                     drive.transmissionRatio or 1.0, drive.drumDiameter or DEFAULT_DRUM_DIAMETER, drive.maxFrequency)
    if machine_kind(component) != "belt":
        if drive is None and not assume_drive:
            return None
    elif getattr(component, "commandFormat", "speed") != "frequency" and not assume_drive:
        return None
    rated_speed = drive.motorRatedSpeed if drive is not None else 1450.0
    rated_frequency = drive.motorRatedFrequency if drive is not None else 50.0
    max_frequency = drive.maxFrequency if drive is not None else rated_frequency
    transmission = (drive.transmissionRatio or 1.0) if drive is not None else 1.0
    gear = drive.gearRatio if drive is not None else None
    diameter = drive.drumDiameter if drive is not None else None
    design = design_speed(component) or (90.0 if rotary else DEFAULT_DESIGN_SPEED)
    if rotary:
        # The gear ratio that turns the table at its design speed (deg/s = rpm * 6).
        gear = gear or rated_speed / transmission / (design / 6.0)
        return Train(rated_speed, rated_frequency, gear, transmission, diameter or DEFAULT_DRUM_DIAMETER,
                     max_frequency)
    # The missing one of gear ratio / drum diameter: the design speed at the rated frequency.
    if gear:
        drum_rpm = rated_speed / transmission / gear
        diameter = design * 60.0 / (math.pi * drum_rpm)
    else:
        diameter = diameter or DEFAULT_DRUM_DIAMETER
        drum_rpm = design / (math.pi * diameter) * 60.0
        gear = rated_speed / transmission / drum_rpm
    return Train(rated_speed, rated_frequency, gear, transmission, diameter, max_frequency)


def pin_belt_train(belt) -> None:
    """A belt commanded in m/s gets a train only from explicit gearRatio and
    drumDiameter (its `speed` is the PLC's command, so nothing may be derived
    from it while it runs). The designer pins them, from the speed it has
    now, when a drive value is first edited - then the motor's limits in rpm
    act on it."""
    if machine_kind(belt) != "belt" or getattr(belt, "commandFormat", "speed") == "frequency":
        return
    drive = belt.drive
    if drive is None or (drive.gearRatio and drive.drumDiameter):
        return
    train = train_of(belt, assume_drive=True)
    drive.gearRatio, drive.drumDiameter = train.gear_ratio, train.drum_diameter


def limits_text(component) -> str:
    """The motor's limits and what they are at the output, for the designer."""
    drive = getattr(component, "drive", None)
    if drive is None or not (drive.motorMaxSpeed or drive.motorMaxAccel):
        return ""
    vmax, amax = motor_limits(component)
    unit = "\u00b0/s" if is_rotary(component) else "m/s"
    parts = []
    if drive.motorMaxSpeed:
        parts.append(f"max {drive.motorMaxSpeed:g} rpm = {vmax:.3f} {unit}")
    if drive.motorMaxAccel:
        parts.append(f"{drive.motorMaxAccel:g} rpm/s = {amax:.2f} {unit}\u00b2")
    return "\nmotor: " + ", ".join(parts)


# Common induction-motor nameplates (rated speed at rated frequency, with
# rated slip): (name, rpm, Hz). The designer offers them for a drive.
MOTOR_PRESETS = [
    ("1450 rpm, 50 Hz", 1450.0, 50.0),    # 4-pole
    ("2900 rpm, 50 Hz", 2900.0, 50.0),    # 2-pole
    ("960 rpm, 50 Hz", 960.0, 50.0),      # 6-pole
    ("1750 rpm, 60 Hz", 1750.0, 60.0),    # 4-pole
    ("3500 rpm, 60 Hz", 3500.0, 60.0),    # 2-pole
]
CUSTOM_MOTOR = "Custom"


def motor_preset(rated_speed: float, rated_frequency: float) -> str:
    """The preset with this nameplate, or CUSTOM_MOTOR."""
    for name, rpm, hz in MOTOR_PRESETS:
        if abs(rpm - rated_speed) < 1e-6 and abs(hz - rated_frequency) < 1e-6:
            return name
    return CUSTOM_MOTOR


def _speed_at(component, train: Train, frequency: float) -> float:
    return train.turn_speed(frequency) if is_rotary(component) else train.speed_for_frequency(frequency)


def motor_limits(component) -> tuple:
    """(max speed, max accel) in the component's unit - m/s and m/s^2, a
    turntable deg/s and deg/s^2 - from its motor's motorMaxSpeed (rpm) and
    motorMaxAccel (rpm/s) through its train; inf where unset or without a
    train (an rpm means nothing without the gearing)."""
    drive = getattr(component, "drive", None)
    train = train_of(component) if drive is not None else None
    if train is None:
        return math.inf, math.inf
    per_rpm = abs(_speed_at(component, train, 1.0) / train.motor_speed(1.0))
    vmax = drive.motorMaxSpeed * per_rpm if drive.motorMaxSpeed else math.inf
    amax = drive.motorMaxAccel * per_rpm if drive.motorMaxAccel else math.inf
    return vmax, amax


def machine_speed(component) -> float:
    """The speed a machine moves at - m/s, a turntable deg/s: its drive's at
    the rated frequency (no higher than the max frequency nor the motor's
    max speed), else its own speed setting."""
    train = train_of(component)
    if train is None:
        return design_speed(component)
    return min(_speed_at(component, train, min(train.motor_rated_frequency, train.max_frequency)),
               motor_limits(component)[0])


def machine_ramps(component) -> tuple:
    """(accel, decel) per second^2 in the machine's unit (m or deg): from
    the inverter's ramp times (0 to the rated frequency), inf without; none
    steeper than the motor's max acceleration."""
    drive = getattr(component, "drive", None)
    train = train_of(component)
    if drive is None or train is None:
        return math.inf, math.inf
    rated = _speed_at(component, train, train.motor_rated_frequency)
    accel = rated / drive.rampUpTime if drive.rampUpTime else math.inf
    decel = rated / drive.rampDownTime if drive.rampDownTime else math.inf
    amax = motor_limits(component)[1]
    return min(accel, amax), min(decel, amax)


def motor_speed_of(component, velocity: float) -> float:
    """Motor rpm while the machine moves at `velocity` (its unit, signed)."""
    train = train_of(component)
    if train is None:
        return 0.0
    per_hz = _speed_at(component, train, 1.0)
    return train.motor_speed(velocity / per_hz) if per_hz else 0.0


def travel_time(component, distance: float) -> float:
    """Seconds to move `distance` (m, a turntable deg) from standstill to
    standstill with the drive's ramps (a triangle when too short for full
    speed)."""
    v = machine_speed(component)
    if v <= 0.0:
        return math.inf
    accel, decel = machine_ramps(component)
    if accel == math.inf and decel == math.inf:
        return distance / v
    ramp_distance = v * v / 2.0 * (1.0 / accel + 1.0 / decel)
    if ramp_distance <= distance:
        return v / accel + v / decel + (distance - ramp_distance) / v
    peak = math.sqrt(2.0 * distance / (1.0 / accel + 1.0 / decel))
    return peak / accel + peak / decel


def ramp_velocity(velocity: float, left: float, vmax: float, accel: float, decel: float, h: float) -> float:
    """The next speed (magnitude) of a positioning move: up to vmax on the
    accel ramp, then braking so it stops after `left` more
    (v <= sqrt(2 * decel * left)); always some progress, so it never creeps
    to a halt short of the target."""
    reachable = min(vmax, math.sqrt(2.0 * decel * left)) if decel != math.inf else vmax
    if reachable > velocity:
        velocity = reachable if accel == math.inf else min(reachable, velocity + accel * h)
    else:
        velocity = reachable if decel == math.inf else max(reachable, velocity - decel * h)
    return max(velocity, min(vmax, 1e-3))


# An elevator's names for the same.
lift_speed = machine_speed
lift_ramps = machine_ramps
lift_time = travel_time


def summary(component) -> tuple:
    """(text, warning) describing the drive train in words, for the
    designer: the chain from the inverter to the surface at the rated
    frequency, the reach at the max frequency, and the design speed in Hz
    (a machine: its speed and a typical move's time).
    warning is "" or why the design speed is out of reach."""
    if machine_kind(component) != "belt":
        return _machine_summary(component)
    belt = component
    train = train_of(belt)
    if train is None:
        return (f"{getattr(belt, 'speed', 0.0):g} m/s, set directly (an ideal drive). Edit a drive value to "
                f"model the motor: its gear ratio and drum are then taken from this speed."), ""
    rated = train.motor_rated_frequency
    carrier = "rollers" if getattr(belt, "surface", "belt") == "roller" else "drum"
    text = (f"{rated:g} Hz \u2192 motor {train.motor_speed(rated):.0f} rpm \u2192 "
            f"{train.output_speed(rated) / train.transmission_ratio:.1f} rpm at the {carrier} \u2192 "
            f"{train.speed_for_frequency(rated):.3f} m/s\n"
            f"max {train.max_speed:.3f} m/s at {train.max_frequency:g} Hz")
    design = abs(float(getattr(belt, "speed", 0.0) or 0.0))
    warning = ""
    if design > 0.0:
        needed = train.frequency_for_speed(design)
        text += f"; design {design:g} m/s = {needed:.1f} Hz"
        if needed > train.max_frequency + 1e-9:
            warning = (f"The design speed {design:g} m/s needs {needed:.1f} Hz, above the inverter's "
                       f"max {train.max_frequency:g} Hz: the belt only reaches {train.max_speed:.3f} m/s.")
    return text + limits_text(belt), warning


_MACHINE_WORDS = {
    # kind: (what is driven, speed unit, own speed field, a typical move: (label, distance))
    "lift": ("drum", "m/s", "liftSpeed", None),
    "transfer": ("sprocket", "m/s", "speed", None),
    "turntable": ("slewing ring", "\u00b0/s", "turnSpeed", ("90\u00b0 turn", 90.0)),
    "diverter": ("wheel", "m/s", "speed", None),
}


def _machine_summary(component) -> tuple:
    kind = machine_kind(component)
    part, unit, field, move = _MACHINE_WORDS[kind]
    train = train_of(component)
    if train is None:
        return (f"{field} {getattr(component, field):g} {unit}, set directly. Edit the drive's values to "
                f"derive it from motor, gearbox and {part} instead."), ""
    rated = min(train.motor_rated_frequency, train.max_frequency)
    v = machine_speed(component)
    if kind == "turntable":
        result = f"table {train.output_speed(rated) / train.transmission_ratio:.2f} rpm = {v:.1f} {unit}"
    else:
        word = {"lift": "lift", "diverter": "divert"}.get(kind, "cross")
        result = (f"{train.output_speed(rated) / train.transmission_ratio:.1f} rpm at the {part} \u2192 "
                  f"{word} {v:.3f} {unit}")
    text = f"{rated:g} Hz \u2192 motor {train.motor_speed(rated):.0f} rpm \u2192 {result}"
    if kind == "lift":
        move = (f"level to level ({component.levelHeight:g} m)", component.levelHeight)
    if move is not None:
        text += f"\n{move[0]}: {travel_time(component, move[1]):.1f} s"
    text += limits_text(component)
    warning = ""
    if train.max_frequency < train.motor_rated_frequency:
        warning = (f"The inverter's max {train.max_frequency:g} Hz is below the rated "
                   f"{train.motor_rated_frequency:g} Hz: it runs at {v:.3f} {unit}.")
    return text, warning
