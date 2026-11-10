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


def train_of(belt) -> Optional[Train]:
    """The belt's drive train, or None (no train data and not commanded in Hz)."""
    drive = getattr(belt, "drive", None)
    if drive is not None and drive.gearRatio and drive.drumDiameter:
        return Train(drive.motorRatedSpeed, drive.motorRatedFrequency, drive.gearRatio,
                     drive.transmissionRatio or 1.0, drive.drumDiameter, drive.maxFrequency)
    if getattr(belt, "commandFormat", "speed") != "frequency":
        return None
    rated_speed = drive.motorRatedSpeed if drive is not None else 1450.0
    rated_frequency = drive.motorRatedFrequency if drive is not None else 50.0
    max_frequency = drive.maxFrequency if drive is not None else rated_frequency
    transmission = (drive.transmissionRatio or 1.0) if drive is not None else 1.0
    gear = drive.gearRatio if drive is not None else None
    diameter = drive.drumDiameter if drive is not None else None
    # The missing one of gear ratio / drum diameter: the design speed at the rated frequency.
    design = abs(float(getattr(belt, "speed", 0.0) or 0.0)) or DEFAULT_DESIGN_SPEED
    if gear:
        drum_rpm = rated_speed / transmission / gear
        diameter = design * 60.0 / (math.pi * drum_rpm)
    else:
        diameter = diameter or DEFAULT_DRUM_DIAMETER
        drum_rpm = design / (math.pi * diameter) * 60.0
        gear = rated_speed / transmission / drum_rpm
    return Train(rated_speed, rated_frequency, gear, transmission, diameter, max_frequency)


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


def summary(belt) -> tuple:
    """(text, warning) describing the belt's drive train in words, for the
    designer: the chain from the inverter to the surface at the rated
    frequency, the reach at the max frequency, and the design speed in Hz.
    warning is "" or why the design speed is out of reach."""
    train = train_of(belt)
    if train is None:
        return "", ""
    rated = train.motor_rated_frequency
    carrier = "rollers" if getattr(belt, "surface", "belt") == "roller" else "drum"
    text = (f"{rated:g} Hz → motor {train.motor_speed(rated):.0f} rpm → "
            f"{train.output_speed(rated) / train.transmission_ratio:.1f} rpm at the {carrier} → "
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
    return text, warning
