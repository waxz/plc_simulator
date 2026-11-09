"""
drive.py - belt motor/VFD model.

The setpoint is devices.commanded_speed(belt): `Belt.speed` (signed) for
commandFormat "speed", or the run / reverse / speed command of a "vfd"
belt; 0 while the drive is faulted. step_drive() turns it into the belt's
real surface speed, `Belt.actualSpeed`, integrates an encoder count,
`Belt.encoderCount`, and sets the status bits the PLC gets as feedback:

    ref    += clamp(setpoint - ref, -decel*h, +accel*h)   VFD ramp
    actual += (ref - actual) * (1 - exp(-h / tau))        drive/mechanical lag
    actual  = clamp(actual, -speedMax, speedMax)
    encoder = (encoder + counts moved) mod 2**32          UINT32, wraps like hardware

    running   |actual| > runningThreshold
    atSpeed   running and |actual - setpoint| <= atSpeedTolerance
    reversing actual < 0 (or, starting up, the setpoint is)
    fault     overload: the parcels on the belt (Belt.load, kg, from
              physics.py) weighed more than drive.maxLoad for longer than
              drive.overloadTime. The motor coasts to a stop (setpoint 0,
              no ramp limit); the fault resets when the run command is
              removed (run off, or speed 0) and the load is back in limit.
              inject_fault() trips it on purpose (tests, demos).

A belt with no `drive` config is ideal: actualSpeed == setpoint every step,
so layouts without drive settings behave exactly as before this model existed.

With a drive train (drivetrain.py: gearRatio and drumDiameter set, or any
belt commanded in Hz) the mechanics apply on top:

    setpoint  clamped to the surface speed at the inverter's maxFrequency
    ramp      rampUpTime / rampDownTime (s from 0 to the rated frequency)
              replace accel / decel when set
    outputFrequency  the inverter's ramped output, Hz (signed)
    motorSpeed       the motor's speed from the surface speed, rpm (signed)
    encoder   on the motor shaft when encoderCountsPerRev is set:
              counts += motor revolutions * encoderCountsPerRev
"""
import math

import devices
import drivetrain
from config_parser import Belt

ENCODER_MODULUS = 1 << 32


def _ramp_limit(ref: float, setpoint: float, accel: float, decel: float) -> float:
    """Per-second rate limit for moving `ref` toward `setpoint`: `decel` while
    the speed magnitude is shrinking (including through zero), else `accel`."""
    shrinking = abs(setpoint) < abs(ref) or (setpoint * ref < 0.0)
    return decel if shrinking else accel


def _run_commanded(belt: Belt) -> bool:
    if belt.commandFormat == "frequency":
        return bool(belt.run) and abs(float(belt.frequency or 0.0)) > 0.0
    if belt.commandFormat == "vfd":
        return bool(belt.run) and abs(float(belt.speed or 0.0)) > 0.0
    return abs(float(belt.speed or 0.0)) > 0.0


def _ramp_rates(drive, train) -> tuple:
    """(accel, decel) in m/s^2: the inverter's ramp times when the belt has a
    train and they are set, else the drive's accel / decel."""
    accel, decel = drive.accel, drive.decel
    if train is not None:
        rated = train.speed_for_frequency(train.motor_rated_frequency)
        if drive.rampUpTime:
            accel = rated / drive.rampUpTime
        if drive.rampDownTime:
            decel = rated / drive.rampDownTime
    return accel, decel


def inject_fault(belt: Belt) -> None:
    """Trip the drive as if it had detected a fault."""
    belt.fault = True


def _update_fault(belt: Belt, h: float) -> None:
    drive = belt.drive
    overloaded = drive is not None and drive.maxLoad is not None and belt.load > drive.maxLoad
    if overloaded and _run_commanded(belt):
        belt._overload_s += h
        if belt._overload_s >= drive.overloadTime:
            belt.fault = True
    else:
        belt._overload_s = 0.0
    if belt.fault and not _run_commanded(belt) and not overloaded:
        belt.fault = False  # reset: run command removed, load back in limit


def step_drive(belt: Belt, h: float) -> None:
    """Advance one belt's drive by `h` seconds."""
    drive = belt.drive
    train = drivetrain.train_of(belt)
    _update_fault(belt, h)
    setpoint = devices.commanded_speed(belt)
    if belt._safety_stop or belt._zone_hold:
        setpoint = 0.0          # a safety circuit / accumulation zone holds it, whatever is commanded
    if train is not None:
        limit = train.max_speed  # the inverter goes no higher than maxFrequency
        setpoint = max(-limit, min(limit, setpoint))

    if drive is None:
        belt._drive_ref = setpoint
        actual = setpoint
    else:
        ref = belt._drive_ref
        if belt.fault:
            ref = 0.0  # coast: the inverter output is off, only the lag slows the belt
        else:
            accel, decel = _ramp_rates(drive, train)
            rate = _ramp_limit(ref, setpoint, accel, decel)
            max_delta = rate * h if rate > 0.0 else math.inf
            ref += max(-max_delta, min(max_delta, setpoint - ref))
        belt._drive_ref = ref

        actual = belt.actualSpeed
        tau = max(drive.tau, 0.5) if belt.fault else drive.tau
        if tau > 0.0:
            actual += (ref - actual) * (1.0 - math.exp(-h / tau))
        else:
            actual = ref
        actual = max(-drive.speedMax, min(drive.speedMax, actual))

    belt.actualSpeed = actual
    if train is not None:
        belt.outputFrequency = train.frequency_for_speed(belt._drive_ref)
        belt.motorSpeed = train.motor_speed_for_surface(actual)
    else:
        belt.outputFrequency = 0.0
        belt.motorSpeed = 0.0

    threshold = drive.runningThreshold if drive is not None else 0.01
    tolerance = drive.atSpeedTolerance if drive is not None else 0.05
    belt.running = abs(actual) > threshold
    belt.atSpeed = belt.running and abs(actual - setpoint) <= tolerance
    belt.reversing = actual < -threshold or (not belt.running and setpoint < 0.0)

    if train is not None and drive is not None and drive.encoderCountsPerRev:
        moved = belt.motorSpeed / 60.0 * h * drive.encoderCountsPerRev + belt._encoder_frac
    else:
        counts_per_m = drive.countsPerM if drive is not None else 10000.0
        moved = actual * h * counts_per_m + belt._encoder_frac
    whole = math.floor(moved)
    belt._encoder_frac = moved - whole
    belt.encoderCount = (belt.encoderCount + int(whole)) % ENCODER_MODULUS
