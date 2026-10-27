"""
drive.py - belt motor/VFD model.

`Belt.speed` is the commanded setpoint (written by the PLC over the exchange).
step_drive() turns it into the belt's real surface speed, `Belt.actualSpeed`,
and integrates an encoder count, `Belt.encoderCount`:

    ref    += clamp(setpoint - ref, -decel*h, +accel*h)   VFD ramp
    actual += (ref - actual) * (1 - exp(-h / tau))        drive/mechanical lag
    actual  = clamp(actual, -speedMax, speedMax)
    encoder = (encoder + counts moved) mod 2**32          UINT32, wraps like hardware

A belt with no `drive` config is ideal: actualSpeed == speed every step, so
layouts without drive settings behave exactly as before this model existed.
"""
import math

from config_parser import Belt

ENCODER_MODULUS = 1 << 32


def _ramp_limit(ref: float, setpoint: float, accel: float, decel: float) -> float:
    """Per-second rate limit for moving `ref` toward `setpoint`: `decel` while
    the speed magnitude is shrinking (including through zero), else `accel`."""
    shrinking = abs(setpoint) < abs(ref) or (setpoint * ref < 0.0)
    return decel if shrinking else accel


def step_drive(belt: Belt, h: float) -> None:
    """Advance one belt's drive by `h` seconds."""
    drive = belt.drive
    setpoint = float(belt.speed or 0.0)

    if drive is None:
        belt._drive_ref = setpoint
        actual = setpoint
    else:
        ref = belt._drive_ref
        rate = _ramp_limit(ref, setpoint, drive.accel, drive.decel)
        max_delta = rate * h if rate > 0.0 else math.inf
        ref += max(-max_delta, min(max_delta, setpoint - ref))
        belt._drive_ref = ref

        actual = belt.actualSpeed
        if drive.tau > 0.0:
            actual += (ref - actual) * (1.0 - math.exp(-h / drive.tau))
        else:
            actual = ref
        actual = max(-drive.speedMax, min(drive.speedMax, actual))

    belt.actualSpeed = actual

    counts_per_m = drive.countsPerM if drive is not None else 10000.0
    moved = actual * h * counts_per_m + belt._encoder_frac
    whole = math.floor(moved)
    belt._encoder_frac = moved - whole
    belt.encoderCount = (belt.encoderCount + int(whole)) % ENCODER_MODULUS
