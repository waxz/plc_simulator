"""
devices.py - how the simulated field devices behave, in one place (no Qt,
no pygame): what the simulator window, the physics step and tests call.

Push buttons (ControlButton)
  press() / release() are the operator's hand. A "momentary" button is
  pressed while held; a "latching" one (selector, emergency stop) toggles on
  each press and ignores release. The signal the PLC gets, `sensor_states`,
  is the final state: 1 while pressed. Whether the button is wired normally
  open (NO) or normally closed (NC, usual for STOP / E-stop) is the
  simulator's detail: the PLC program never needs to know it.

LEDs (LedIndicator)
  packFormat "bit": the PLC writes isOn. packFormat "byte": the PLC writes
  a mode - 0 off, 1 on, 2 blink 1 Hz, 3 blink 4 Hz - and update_led() turns
  it into the lamp state (isOn) at the current time.

Motors (Belt)
  commanded_speed(): the setpoint the drive ramps to. commandFormat "speed":
  `speed` (signed). "vfd": `speed` magnitude, only while `run`, negative
  when `reverse`. "frequency": `frequency` (Hz) through the drive train
  (drivetrain.py), only while `run`, negative when `reverse`. 0 while the
  drive is faulted. drive.py turns it into the actual speed and the
  feedback (actualSpeed, encoderCount, running, atSpeed, reversing, fault;
  outputFrequency and motorSpeed with a drive train).

Sensors (Sensor)
  condition(): the sensor's output stage on top of the raw beam geometry
  (physics.py) - on/off delays per beam, light-on/dark-on (invert), the
  detection counter (risingEdges) and blockedBeams.
"""
from typing import List, Sequence

import drivetrain
from config_parser import Belt, ControlButton, LedIndicator, Sensor

BLINK_HZ = {2: 1.0, 3: 4.0}


# -- push buttons -------------------------------------------------------------

def contact(button: ControlButton) -> bool:
    """The signal the PLC sees: TRUE while the button is pressed (a latching
    one: while latched in). Whether the button is wired normally open or
    normally closed (`normallyClosed`, e.g. STOP and E-STOP) is the
    simulator's detail - the panel shows it - and is applied here, so the
    PLC program only ever reads the final pressed / not pressed state."""
    return bool(button.pressed)


def sync_button(button: ControlButton) -> None:
    button.sensor_states = contact(button)


def press(button: ControlButton) -> None:
    """The operator pushes the button."""
    if button.buttonType == "latching":
        button.pressed = not button.pressed
    else:
        button.pressed = True
    sync_button(button)


def release(button: ControlButton) -> None:
    """The operator lets go (no effect on a latching button)."""
    if button.buttonType != "latching":
        button.pressed = False
    sync_button(button)


def is_pressed_signal(button: ControlButton, signal) -> bool:
    """PLC side: does this received signal mean "pressed"? The signal is
    the final state (see contact): TRUE is pressed, NO or NC alike."""
    if isinstance(signal, str):
        return signal.strip() not in ("", "0", "false", "False")
    return bool(signal)


# -- LEDs ---------------------------------------------------------------------

def lamp_on(led: LedIndicator, t: float) -> bool:
    if led.packFormat != "byte":
        return bool(led.isOn)
    mode = int(led.mode or 0)
    if mode in BLINK_HZ:
        return (t * BLINK_HZ[mode]) % 1.0 < 0.5
    return mode == 1


def update_led(led: LedIndicator, t: float) -> None:
    """Mode LEDs: set the lamp (isOn) for time t (s)."""
    if led.packFormat == "byte":
        led.isOn = lamp_on(led, t)


# -- motors -------------------------------------------------------------------

def commanded_speed(belt: Belt) -> float:
    """The signed speed setpoint the drive follows (m/s)."""
    if belt.fault:
        return 0.0
    if belt.commandFormat == "frequency":
        if not belt.run:
            return 0.0
        magnitude = drivetrain.train_of(belt).speed_for_frequency(abs(float(belt.frequency or 0.0)))
        return -magnitude if belt.reverse else magnitude
    if belt.commandFormat == "vfd":
        if not belt.run:
            return 0.0
        magnitude = abs(float(belt.speed or 0.0))
        return -magnitude if belt.reverse else magnitude
    return float(belt.speed or 0.0)


# -- sensors ------------------------------------------------------------------

def condition(sensor: Sensor, raw: Sequence[bool], h: float) -> str:
    """Output stage of a sensor for one step of `h` seconds: `raw` is each
    beam's blocked state from geometry (one for an IR sensor). Applies
    onDelay / offDelay per beam, counts detections, then invert. Returns
    the new sensor_states ("0"/"1" per beam) and updates risingEdges and
    blockedBeams."""
    if sensor._filter is None or len(sensor._filter) != len(raw):
        sensor._filter = [[False, 0.0] for _ in raw]
    was_detecting = any(f[0] for f in sensor._filter)
    on_delay, off_delay = max(0.0, sensor.onDelay), max(0.0, sensor.offDelay)
    for f, blocked in zip(sensor._filter, raw):
        output, timer = f
        if blocked == output:
            f[1] = 0.0           # agrees with the output: nothing pending
            continue
        timer += h
        delay = on_delay if blocked else off_delay
        if timer >= delay - 1e-12:
            f[0], f[1] = blocked, 0.0
        else:
            f[1] = timer
    outputs: List[bool] = [f[0] for f in sensor._filter]
    if any(outputs) and not was_detecting:
        sensor.risingEdges += 1
    sensor.blockedBeams = sum(1 for b in raw if b)
    return "".join("1" if (o != bool(sensor.invert)) else "0" for o in outputs)
