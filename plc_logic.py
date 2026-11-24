"""
plc_logic.py - a small, generic PLC program for any conveyor layout.

DeliveryController works on the PLC side of the exchange: it reads the I/O
image the simulator sends (buttons and sensors) and writes the commands the
simulator applies (belt speeds and LEDs) into the same SimulatorConfig
object that ComponentMessage serializes - so it runs unchanged over TCP or
MQTT (plc_mock.py) or directly in a test.

Behaviour, once per scan (step()):
  Buttons (user input), picked by label/id; each reads 1 while pressed,
  NO or NC contact alike (the contact type is the simulator's detail):
    "start"            -> run
    "stop"             -> stop
    "reset"            -> clear a jam
    no such labels     -> one button toggles run/stop; with two or more the
                          first starts and the second stops
  Sensors (parcel triggers): each rising edge is a parcel passing; counted
    per sensor. The sensor nearest a sink counts deliveries. A sensor that
    stays blocked longer than `jam_time` is a jam: every belt stops until
    Start/Reset is pressed with the sensor clear.
  Sensors with `invert` (light-on output) read 0 when blocked.
  Laser banners (lasers shining down, a bit per beam): the PLC measures each
    parcel passing under one (BannerProfile) - its width from the blocked
    beams, its length from how far the belt moved while any beam was
    blocked - and logs "banner_1: parcel 0.42 m long x 0.36 m wide".
  Belts: running, no jam, no drive fault -> each belt's design speed (its
    speed in the layout file, 1 m/s where that is 0; negative = reverse),
    else stopped. commandFormat "speed": `speed` (signed); "vfd": `run`,
    `reverse` and `speed` (magnitude); "frequency": `run`, `reverse` and
    `frequency`, the Hz that give the design speed through the belt's drive
    train (drivetrain.py).
  Drive faults (Belt.fault in the drive feedback: a belt with `feedback` on reports it in the input frame):
    stop the line; it can be started again once every fault has cleared
    (a drive resets when its run command is removed).
  LEDs, picked by label/id: "run"/"ok"/"auto" (else the first LED) shows
    running, "jam"/"fault"/"alarm"/"error" (else the second LED) a jam or a
    drive fault.

Machines with control "plc" (machines.py: stoppers, robot arms, elevators)
are run by MachineControl while the line runs: a stopper holds what
arrives for its holdTime and then lets it pass, a robot starts a cycle
when its pick sensor sees a part and there is somewhere to put it, an
elevator loads, lifts, unloads and comes back (running its platform belt
only for that). Stopped, every stopper stays up.

step() returns the events of that scan as text, for logs.

LineController (below) does the same per line for layouts with several
lines - a main line and feeders merging into it, each with its own panel -
and holds feeder parcels at the merge; make_controller() picks the one that
fits a layout.
"""
import math
import re
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set

import devices
import drivetrain
import equipment_geometry
import logistics
import machines
from config_parser import SimulatorConfig
from device_kinds import DEVICE_KINDS, wheel_rows

DEFAULT_SPEED = 1.0  # m/s for belts whose layout speed is 0


def is_on(value) -> bool:
    """A state as received: bool, int, or "0"/"1" text (any 1 in a bit
    string - e.g. one blocked beam of a laser banner - is on)."""
    if isinstance(value, str):
        text = value.strip()
        return "1" in text or text.lower() in ("true", "yes", "on")
    return bool(value)


def occupied(sensor) -> bool:
    """Is anything in front of the sensor? A light-on (invert) output reads 0
    for a blocked beam."""
    value = sensor.sensor_states
    if isinstance(value, str) and set(value.strip()) <= {"0", "1"} and value.strip():
        return ("0" if sensor.invert else "1") in value
    return is_on(value) != bool(sensor.invert)


def part_at_pick(config: SimulatorConfig, robot) -> bool:
    """A part waits at the robot's pick point: its pick sensor (an IR sensor
    wired to the PLC, `pickSensorId`) is blocked - or, without one, the
    robot's own `partAtPick`."""
    if robot.pickSensorId:
        sensor = next((s for s in config.sensors if s.id == robot.pickSensorId), None)
        return sensor is not None and occupied(sensor)
    return bool(robot.partAtPick)


def waiting_sensors(config: SimulatorConfig) -> Set[str]:
    """Sensors where a part waits on purpose (a robot's pick sensor): blocked
    for long is not a jam there."""
    return {r.pickSensorId for r in config.robots if r.pickSensorId}


def _pick(components, words, fallback_index: Optional[int]):
    for c in components:
        text = f"{getattr(c, 'label', '')} {c.id}".lower()
        if any(w in text for w in words):
            return c
    if fallback_index is not None and len(components) > fallback_index:
        return components[fallback_index]
    return None


def _design_speed(belt) -> float:
    """Signed running speed from the layout file (1 m/s where it is 0); a vfd
    belt's `reverse` flag makes it negative."""
    speed = float(belt.speed or 0.0)
    if abs(speed) < 1e-9:
        speed = DEFAULT_SPEED
    if belt.commandFormat in ("vfd", "frequency") and belt.reverse:
        speed = -abs(speed)
    return speed


def _command_frequency(belt, run: bool, nominal: float) -> None:
    """Command a belt that takes Hz (commandFormat "frequency") as a PLC does
    with the drive's nameplate data: run / reverse, and the inverter
    frequency that gives `nominal` m/s at the drum (drivetrain.py)."""
    belt.run = run
    belt.reverse = nominal < 0
    belt.frequency = drivetrain.train_of(belt).frequency_for_speed(abs(nominal))


CHUTE_SWAP_S = 5.0     # a full chute without an empty time: emptied this long after it fills
LOADED_AFTER_S = 0.5   # elevator: a part on the platform this long is on board (its belt stops)


# ---------------------------------------------------------------------------
# Device programs: what this PLC does with each kind of device
# (device_kinds.DEVICE_KINDS), once per scan, from the device's feedback to
# its commands. One function per kind, registered with @device_program:
# adding a kind of device to the PLC is one more of these. They run in the
# registry's order (scanners before diverters: the reads feed the tracking).
# ---------------------------------------------------------------------------

@dataclass
class Scan:
    """What a device program gets each scan."""
    running: bool                  # the line runs (started, no jam / fault)
    dt: float                      # s since the last scan
    speeds: Dict[str, float]       # belt id -> its running speed (platform belts)
    events: List[str]              # log lines of this scan
    reset: bool                    # Start / Reset pressed this scan


DeviceProgram = Callable[["MachineControl", object, Scan], None]
DEVICE_PROGRAMS: Dict[str, DeviceProgram] = {}


def device_program(list_name: str):
    """Register the PLC program for one kind of device (its SimulatorConfig
    list name, as in device_kinds.DEVICE_KINDS)."""
    def register(fn: DeviceProgram) -> DeviceProgram:
        DEVICE_PROGRAMS[list_name] = fn
        return fn
    return register


def _run_platform_belt(control: "MachineControl", belt_id: Optional[str], run: bool, scan: Scan) -> None:
    belt = control.belts.get(belt_id) if belt_id else None
    if belt is None:
        return
    nominal = scan.speeds.get(belt.id, DEFAULT_SPEED)
    if belt.commandFormat == "frequency":
        _command_frequency(belt, run, nominal)
    elif belt.commandFormat == "vfd":
        belt.run, belt.speed = run, abs(nominal)
    else:
        belt.speed = nominal if run else 0.0


@device_program("stoppers")
def _stopper(control, stopper, scan: Scan) -> None:
    """Hold what arrives for holdTime, then let it pass; stopped: stay up."""
    state = control.state(stopper)
    release = scan.running and not machines.stopper_wants_up(stopper, state, scan.dt)
    if release != stopper.release:
        scan.events.append(f"stopper {stopper.id} {'released' if release else 'up'}")
    stopper.release = release


@device_program("robots")
def _robot(control, robot, scan: Scan) -> None:
    """Program 1 (pick and place) when its pick sensor sees a part and the
    place point is ready; a fault (e.g. a point out of reach) is reset with
    the line's reset."""
    robot.enable, robot.program = True, 1
    part = part_at_pick(control.config, robot)
    robot.start = scan.running and robot.ready and part and robot.placeReady
    robot.faultReset = robot.fault and scan.reset


@device_program("elevators")
def _elevator(control, elevator, scan: Scan) -> None:
    """Load at autoFrom, lift to autoTo, unload, come back - running its
    platform belt only for that."""
    state = control.state(elevator)
    on_board = elevator.occupied and elevator.atLevel
    state["on_s"] = state.get("on_s", 0.0) + scan.dt if on_board else 0.0
    state["loaded"] = state["on_s"] >= LOADED_AFTER_S
    target, run = machines.elevator_auto(elevator, state, scan.dt)
    if not scan.running:
        run = False
    elif target != elevator.targetLevel:
        scan.events.append(f"elevator {elevator.id} to level {target}")
        elevator.targetLevel = target
    _run_platform_belt(control, elevator.beltId, run, scan)


@device_program("scanners")
def _scanner(control, scanner, scan: Scan) -> None:
    """Tracking: each read gives code -> destination (what a warehouse
    system answers)."""
    if control.reads.get(scanner.id) != scanner.readCount:
        control.reads[scanner.id] = scanner.readCount
        if scanner.code and scanner.readCount:
            control.track[scanner.code] = scanner.destination
            if len(control.track) > 4096:                       # forget the oldest
                control.track.pop(next(iter(control.track)))


@device_program("diverters")
def _diverter(control, d, scan: Scan) -> None:
    """Divert the code in its zone when it is tracked to its destination.
    A pusher: enable then, at the layout's speed and divertAngle. Wheels:
    enabled while running, straight on (angle 0) and to divertAngle for
    the destination, at a speed that keeps the parcels' pace along the
    travel (the design speed is the diverting one: speed x cos(divertAngle)
    / cos(angle))."""
    fire = scan.running and d.presentCode != 0 and control.track.get(d.presentCode) == d.destination
    state = control.state(d)
    if fire and not state.get("fire"):
        scan.events.append(f"diverter {d.id}: {d.presentCode} -> destination {d.destination}")
    state["fire"] = fire
    if d.diverterType == "wheels":
        d.enable = scan.running
        d.commandAngle = equipment_geometry.wheel_steer(d.divertAngle) if fire else 0.0
        d.commandSpeed = d.speed * math.cos(math.radians(equipment_geometry.wheel_steer(d.divertAngle))) / \
            math.cos(math.radians(d.commandAngle))
        n = wheel_rows(d)
        if n > 1:                                               # a deck of rows: all of them so
            d.rowAngles, d.rowSpeeds = [d.commandAngle] * n, [d.commandSpeed] * n
    else:
        d.enable = fire
        d.commandSpeed, d.commandAngle = d.speed, d.divertAngle



@device_program("turntables")
def _turntable(control, table, scan: Scan) -> None:
    """Load at autoFrom, turn to autoTo, unload, turn back."""
    target, run = logistics.turntable_auto(table, control.state(table), scan.dt)
    if scan.running and target != table.targetIndex:
        scan.events.append(f"turntable {table.id} to position {target}")
        table.targetIndex = target
    _run_platform_belt(control, table.beltId, run and scan.running, scan)


@device_program("transfers")
def _transfer(control, t, scan: Scan) -> None:
    """Lift once a pallet stands centred; keep running until it has left."""
    transfer = scan.running and (t.centered or (t.transfer and t.present))
    if transfer and not t.transfer:
        scan.events.append(f"transfer {t.id}")
    t.transfer = transfer


@device_program("accumulators")
def _accumulator(control, acc, scan: Scan) -> None:
    """Released while the line runs."""
    acc.release = scan.running


@device_program("doors")
def _door(control, door, scan: Scan) -> None:
    """Locked while the line runs."""
    door.lock = scan.running


@device_program("safety")
def _safety(control, circuit, scan: Scan) -> None:
    """Reset with the line's Start / Reset."""
    circuit.reset = scan.reset


@device_program("sinks")
def _chute(control, sink, scan: Scan) -> None:
    """A full chute without an empty time is emptied a while after it fills
    (the operator swapping the bin)."""
    if sink.emptyTime:
        return
    state = control.state(sink)
    state["full_s"] = state.get("full_s", 0.0) + scan.dt if sink.full else 0.0
    sink.empty = state["full_s"] >= CHUTE_SWAP_S


PLC_VALUE_TURN_S = 10.0     # the mock's test signal: one turn of the circle


@device_program("plcValues")
def _plc_value(control, value, scan: Scan) -> None:
    """A value the PLC shows (it drives nothing): this mock writes a test
    signal, so the charts have something to draw - the PLC values in pairs
    trace a circle (the first of a pair cos, the second sin, one turn in
    PLC_VALUE_TURN_S, radius 1), while the line runs."""
    state = control.state(value)
    if scan.running:
        state["t"] = state.get("t", 0.0) + scan.dt
    ids = [v.id for v in control.config.plcValues]
    index = ids.index(value.id) if value.id in ids else 0
    angle = 2.0 * math.pi * state.get("t", 0.0) / PLC_VALUE_TURN_S
    signal = math.cos(angle) if index % 2 == 0 else math.sin(angle)
    kind = getattr(value, "dataType", "REAL")
    if kind == "REAL":
        value.value = signal
    elif kind == "BOOL":
        value.value = signal > 0.0
    else:                                       # an integer: the time it ran, in tenths of a second
        value.value = int(state.get("t", 0.0) * 10.0)


@device_program("trackedLoads")
def _tracked_load(control, slot, scan: Scan) -> None:
    """A slot for a load the PLC tracks: this mock tracks no loads (it sees
    only its I/O image, not the parcels), so every slot stays empty. A real
    PLC program fills them (the AX merge demo: MergeLine.ShowLoads)."""
    slot.active = False


class MachineControl:
    """The PLC side of the machines and logistics devices: once per scan,
    after the belts' own outputs, the program of each kind of device
    (DEVICE_PROGRAMS) writes the commands of its devices on control "plc"
    from their feedback."""

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self.belts = {b.id: b for b in config.belts}
        self.track: Dict[int, int] = {}      # scanner tracking: code -> destination
        self.reads: Dict[str, int] = {}      # scanner id -> the readCount seen last
        self._states: Dict[str, dict] = {}
        self._last: Optional[float] = None
        self.reset_requested = False    # Start / Reset pressed this scan: robot fault reset, safety reset
        self.own_programs: set = set()  # device ids another program drives (plc_merge's wheel sorters)

    def state(self, device) -> dict:
        """A device program's own memory for one device."""
        return self._states.setdefault(device.id, {})

    def step(self, now: float, running: bool, speeds: Dict[str, float], events: List[str]) -> None:
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        scan = Scan(running, dt, speeds, events, self.reset_requested)
        for name, kind in DEVICE_KINDS.items():
            program = DEVICE_PROGRAMS.get(name)
            if program is None:
                continue
            for device in getattr(self.config, name, []):
                if kind.exchanged(device) and getattr(device, "control", "plc") == "plc"                         and device.id not in self.own_programs:     # driven by a program of its own
                    program(self, device, scan)
        self.reset_requested = False


@dataclass
class SensorTrack:
    occupied: bool = False
    since: float = 0.0
    count: int = 0


def beam_bits(sensor) -> List[bool]:
    """A laser banner's beams as received, True = blocked (a light-on /
    invert output reads 0 for a blocked beam)."""
    value = sensor.sensor_states
    if isinstance(value, int) and not isinstance(value, bool):     # off the wire: bit i = beam i
        count = int(sensor.laserCount or 16)
        return [bool(value >> i & 1) != bool(sensor.invert) for i in range(count)]
    if not isinstance(value, str) or not value.strip() or not set(value.strip()) <= {"0", "1"}:
        return [occupied(sensor)]
    return [(c == "1") != bool(sensor.invert) for c in value.strip()]          # in the simulation: "0110..."


def belt_at(config: SimulatorConfig, x: float, y: float):
    """The straight belt under (x, y), or None."""
    for belt in config.belts:
        if belt.type != "linear" or belt.vertexs:
            continue
        a = math.radians(belt.rotation)
        u = (x - belt.x) * math.cos(a) + (y - belt.y) * math.sin(a)
        v = -(x - belt.x) * math.sin(a) + (y - belt.y) * math.cos(a)
        if abs(u) <= belt.length / 2 and abs(v) <= belt.beltWidth / 2:
            return belt
    return None


@dataclass
class BannerProfile:
    """A PLC measuring parcels with a laser banner (lasers shining down, one
    every `pitch` across the belt): while a parcel is under it, the widest
    span of blocked beams is its width and the distance the belt moves (its
    commanded speed x the scan time) its length. `measured` is the last
    parcel's (length, width), m."""
    pitch: float
    belt: Optional[str]                    # the belt under the banner: its speed is the parcel's
    first: int = -1
    last: int = -1
    length: float = 0.0
    measured: Optional[tuple] = None
    count: int = 0

    def step(self, bits: List[bool], speed: float, dt: float) -> Optional[tuple]:
        """One scan; (length, width) when a parcel has just passed."""
        blocked = [i for i, b in enumerate(bits) if b]
        if blocked:
            self.first = blocked[0] if self.first < 0 else min(self.first, blocked[0])
            self.last = max(self.last, blocked[-1])
            self.length += abs(speed) * dt
            return None
        if self.first < 0:
            return None
        # n blocked beams: the parcel spans between n - 1 and n + 1 pitches; n is the estimate
        width = (self.last - self.first + 1) * self.pitch
        self.measured = (round(self.length, 3), round(width, 3))
        self.count += 1
        self.first, self.last, self.length = -1, -1, 0.0
        return self.measured


def banner_profiles(config: SimulatorConfig) -> Dict[str, BannerProfile]:
    profiles = {}
    for sensor in config.sensors:
        if sensor.sensorType == "laser_banner":
            belt = belt_at(config, sensor.x, sensor.y)
            profiles[sensor.id] = BannerProfile(float(sensor.beamDistance or 0.05), belt.id if belt else None)
    return profiles


def measure_banners(profiles: Dict[str, BannerProfile], config: SimulatorConfig, dt: float,
                    speed_of: Callable[[Optional[str]], float], events: List[str]) -> None:
    """One scan of every banner's profile measurement; a parcel that has
    passed is logged with its size."""
    for sensor in config.sensors:
        profile = profiles.get(sensor.id)
        if profile is None:
            continue
        size = profile.step(beam_bits(sensor), speed_of(profile.belt), dt)
        if size is not None:
            events.append(f"{sensor.id}: parcel {size[0]:.2f} m long x {size[1]:.2f} m wide")


@dataclass
class DeliveryController:
    config: SimulatorConfig
    jam_time: float = 5.0
    autostart: bool = False
    running: bool = False
    jammed: Optional[str] = None           # id of the blocked sensor
    delivered: int = 0
    sensors: Dict[str, SensorTrack] = field(default_factory=dict)
    faulted: Set[str] = field(default_factory=set)  # belts whose drive reports a fault

    def __post_init__(self):
        cfg = self.config
        buttons = list(cfg.buttons)
        self.start_button = _pick(buttons, ("start",), None)
        self.stop_button = _pick(buttons, ("stop",), None)
        self.reset_button = _pick(buttons, ("reset", "ack"), None)
        if self.start_button is None and self.stop_button is None:
            if len(buttons) == 1:
                self.toggle_button = buttons[0]
            else:
                self.toggle_button = None
                self.start_button = buttons[0] if buttons else None
                self.stop_button = buttons[1] if len(buttons) > 1 else None
        else:
            self.toggle_button = None
        leds = list(cfg.leds)
        self.run_led = _pick(leds, ("run", "ok", "auto", "green"), 0)
        self.jam_led = _pick([l for l in leds if l is not self.run_led], ("jam", "fault", "alarm", "error"), 0)
        self.speeds = {b.id: _design_speed(b) for b in cfg.belts}
        self.delivery_sensor = self._sensor_nearest_a_sink()
        self._pressed = {b.id: False for b in buttons}
        self.sensors = {s.id: SensorTrack() for s in cfg.sensors}
        self.banners = banner_profiles(cfg)
        self._scan_at: Optional[float] = None
        self.running = self.autostart
        self.machines = MachineControl(cfg)
        self._write_outputs()

    def _sensor_nearest_a_sink(self):
        best, best_d = None, math.inf
        for sensor in self.config.sensors:
            for sink in self.config.sinks:
                d = math.hypot(sensor.x - sink.x, sensor.y - sink.y)
                if d < best_d:
                    best, best_d = sensor.id, d
        return best

    def roles(self) -> Dict[str, Optional[str]]:
        def ident(c):
            return c.id if c is not None else None
        return {"start": ident(self.start_button), "stop": ident(self.stop_button),
                "reset": ident(self.reset_button), "toggle": ident(getattr(self, "toggle_button", None)),
                "run_led": ident(self.run_led), "jam_led": ident(self.jam_led),
                "delivery_sensor": self.delivery_sensor}

    # -- the scan ------------------------------------------------------------------

    def step(self, now: Optional[float] = None) -> List[str]:
        now = time.monotonic() if now is None else now
        events: List[str] = []
        self._buttons(events)
        self._sensors(now, events)
        dt = 0.0 if self._scan_at is None else max(0.0, now - self._scan_at)
        self._scan_at = now
        moving = self.running and self.jammed is None and not self.faulted
        measure_banners(self.banners, self.config, dt,
                        lambda belt: self.speeds.get(belt, 0.0) if moving and belt else 0.0, events)
        self._drive_faults(events)
        self._write_outputs()
        self.machines.step(now, self.running and self.jammed is None and not self.faulted, self.speeds, events)
        return events

    def _buttons(self, events: List[str]):
        for button in self.config.buttons:
            pressed = devices.is_pressed_signal(button, button.sensor_states)
            edge = pressed and not self._pressed.get(button.id, False)
            self._pressed[button.id] = pressed
            if not edge:
                continue
            if button is self.toggle_button:
                self._set_running(not self.running, button.id, events)
            elif button is self.start_button:
                self._clear_jam(button.id, events)
                self.machines.reset_requested = True
                self._set_running(True, button.id, events)
            elif button is self.stop_button:
                self._set_running(False, button.id, events)
            elif button is self.reset_button:
                self._clear_jam(button.id, events)
                self.machines.reset_requested = True
            else:
                events.append(f"button {button.id} pressed")

    def _set_running(self, running: bool, by: str, events: List[str]):
        if running and self.jammed is not None:
            events.append(f"{by}: can't start, sensor {self.jammed} is still jammed")
            return
        if running and self.faulted:
            events.append(f"{by}: can't start, drive fault on {', '.join(sorted(self.faulted))}")
            return
        if running != self.running:
            self.running = running
            events.append(f"{'START' if running else 'STOP'} ({by})")

    def _clear_jam(self, by: str, events: List[str]):
        if self.jammed is None:
            return
        if self.sensors[self.jammed].occupied:
            events.append(f"{by}: jam at {self.jammed} not cleared - remove the parcel first")
            return
        events.append(f"jam at {self.jammed} cleared ({by})")
        self.jammed = None

    def _sensors(self, now: float, events: List[str]):
        for sensor in self.config.sensors:
            track = self.sensors.setdefault(sensor.id, SensorTrack())
            blocked = occupied(sensor)
            if blocked and not track.occupied:
                track.count += 1
                track.since = now
                if sensor.id == self.delivery_sensor:
                    self.delivered += 1
                    events.append(f"parcel at {sensor.id} (#{track.count}) - delivered {self.delivered}")
                else:
                    events.append(f"parcel at {sensor.id} (#{track.count})")
            elif blocked and sensor.id in waiting_sensors(self.config):
                track.since = now   # a part waiting for the robot: not a jam
            track.occupied = blocked
            if blocked and self.jammed is None and self.running and now - track.since > self.jam_time:
                self.jammed = sensor.id
                events.append(f"JAM at {sensor.id}: blocked for {now - track.since:.1f} s - belts stopped")

    def _drive_faults(self, events: List[str]):
        for belt in self.config.belts:
            if belt.fault and belt.id not in self.faulted:
                self.faulted.add(belt.id)
                events.append(f"FAULT on {belt.id}: drive tripped - line stopped")
                self.running = False
            elif not belt.fault and belt.id in self.faulted:
                self.faulted.discard(belt.id)
                events.append(f"drive {belt.id} fault cleared - press Start")

    def _write_outputs(self):
        run = self.running and self.jammed is None and not self.faulted
        for belt in self.config.belts:
            nominal = self.speeds.get(belt.id, DEFAULT_SPEED)
            if belt.commandFormat == "frequency":
                _command_frequency(belt, run, nominal)
            elif belt.commandFormat == "vfd":
                belt.run = run
                belt.reverse = nominal < 0
                belt.speed = abs(nominal)
            else:
                belt.speed = nominal if run else 0.0
        if self.run_led is not None:
            self.run_led.isOn = run
        if self.jam_led is not None:
            self.jam_led.isOn = self.jammed is not None or bool(self.faulted)

    def summary(self) -> str:
        counts = ", ".join(f"{sid} {t.count}" for sid, t in self.sensors.items()) or "no sensors"
        if self.faulted:
            state = "FAULT " + ", ".join(sorted(self.faulted))
        elif self.jammed:
            state = "JAM at " + self.jammed
        else:
            state = "RUN" if self.running else "STOP"
        return f"{state} | parcels: {counts} | delivered {self.delivered}"


# ---------------------------------------------------------------------------
# Line control: several conveyor lines in one layout
# ---------------------------------------------------------------------------
#
# A plant layout usually has more than one line: a main line to the outfeed
# and feeder lines merging into it, each with its own operator panel (START /
# STOP / RESET buttons, AUTO / MANUAL / ERROR lamps) and sensors. Lines are
# found from the names: a label like "L2 START" / "L2 BANNER", or an id like
# belt_1_2_3 (station 1, line 2, item 3).
#
# LineController, once per scan:
#   per line   START / STOP / RESET act on that line only; a jam (a sensor
#              blocked longer than jam_time) or a drive fault stops that line.
#   main line  the line that reaches a sink. Feeders can only be started while
#              it runs.
#   merging    a feeder brings its parcels up to its end sensor (the one nearest
#              the main line) and HOLDS them there - its belts stop - while the
#              main line isn't moving parcels (stopped, jammed, fault) or while
#              the main line's merge sensor sees a parcel (the main line has
#              priority at the merge). The hold latches (a parcel coasting past
#              the sensor doesn't release it) and releases on its own once the
#              main line moves and the merge is free. A held parcel is not a jam.
#   lamps      AUTO/RUN/OK/GREEN = running, MANUAL/STOP/IDLE = stopped without
#              an error, ERROR/JAM/FAULT/ALARM = jam or drive fault, HOLD (if a
#              lamp is named so) = holding at the merge.

_LINE_LABEL = re.compile(r"\bL(\d+)\b", re.IGNORECASE)
_LINE_ID = re.compile(r"_(\d+)_(\d+)_\d+$")
DELIVERY_SENSOR_RANGE = 5.0  # m: a main-line sensor this close to a sink counts deliveries
MERGE_SENSOR_RANGE = 8.0  # m: a main-line sensor this close to a feeder's end sensor watches the merge


def line_of(component) -> Optional[str]:
    """"L<n>" from a label like "L2 START", else from an id like belt_1_2_3."""
    label = getattr(component, "label", "") or ""
    m = _LINE_LABEL.search(label)
    if m:
        return f"L{int(m.group(1))}"
    m = _LINE_ID.search(component.id)
    return f"L{int(m.group(2))}" if m else None


def find_lines(config: SimulatorConfig) -> Dict[str, Dict[str, list]]:
    """{line: {"belts": [...], "sensors": [...], "buttons": [...], "leds": [...]}}
    for every line named in the layout."""
    lines: Dict[str, Dict[str, list]] = {}
    for kind in ("belts", "sensors", "buttons", "leds"):
        for component in getattr(config, kind):
            key = line_of(component)
            if key is not None:
                lines.setdefault(key, {"belts": [], "sensors": [], "buttons": [], "leds": []})[kind].append(component)
    return dict(sorted(lines.items(), key=lambda kv: int(kv[0][1:])))


@dataclass
class Line:
    name: str
    belts: list
    sensors: list
    buttons: list
    leds: list
    running: bool = False
    jammed: Optional[str] = None
    faulted: Set[str] = field(default_factory=set)
    holding: bool = False
    yields: int = 0                        # feeder: times it gave way to a parcel on the main line
    hold_logged: bool = False              # feeder: its current hold was reported (main line not moving)
    feeds: Optional[str] = None            # the line it merges into (feeders)
    hold_sensor: Optional[str] = None      # feeder: its end sensor, where parcels wait
    merge_sensor: Optional[str] = None     # feeder: the main line's sensor at the merge

    @property
    def state(self) -> str:
        if self.faulted:
            return "FAULT"
        if self.jammed:
            return "JAM"
        if not self.running:
            return "STOP"
        return "HOLD" if self.holding else "RUN"

    @property
    def moving(self) -> bool:
        """Its belts run (parcels move on it)."""
        return self.running and not self.jammed and not self.faulted and not self.holding


def _distance_to_belts(x: float, y: float, belts) -> float:
    return min((math.hypot(b.x - x, b.y - y) for b in belts), default=math.inf)


class LineController:
    """Per-line control with feeder merges - see the comment above. Same
    interface as DeliveryController (step / roles / summary / speeds)."""

    def __init__(self, config: SimulatorConfig, jam_time: float = 5.0, autostart: bool = False):
        self.config = config
        self.jam_time = jam_time
        self.delivered = 0
        groups = find_lines(config)
        if not groups:
            raise ValueError("No lines found (labels like 'L1 START' or ids like belt_1_1_1).")
        self.lines: Dict[str, Line] = {name: Line(name, **parts) for name, parts in groups.items()}
        self.main = self._main_line()
        for line in self.lines.values():
            if line.name != self.main.name:
                self._plan_merge(line)
        self.speeds = {b.id: _design_speed(b) for b in config.belts}
        self.machines = MachineControl(config)
        self.sensors = {s.id: SensorTrack() for s in config.sensors}
        self.banners = banner_profiles(config)
        self._scan_at: Optional[float] = None
        self.delivery_sensor = self._delivery_sensor()
        self._pressed = {b.id: False for b in config.buttons}
        for line in self.lines.values():
            line.running = autostart
        self._write_outputs()

    # -- structure -----------------------------------------------------------------

    def _main_line(self) -> Line:
        """The line whose belts come nearest a sink (else the first line)."""
        best, best_d = None, math.inf
        for line in self.lines.values():
            for sink in self.config.sinks:
                d = _distance_to_belts(sink.x, sink.y, line.belts)
                if d < best_d:
                    best, best_d = line, d
        return best or next(iter(self.lines.values()))

    def _delivery_sensor(self) -> Optional[str]:
        """A main-line sensor near a sink counts deliveries (none if there is no such sensor)."""
        best, best_d = None, DELIVERY_SENSOR_RANGE
        for sensor in self.main.sensors:
            for sink in self.config.sinks:
                d = math.hypot(sensor.x - sink.x, sensor.y - sink.y)
                if d <= best_d:
                    best, best_d = sensor.id, d
        return best

    def _plan_merge(self, feeder: Line):
        feeder.feeds = self.main.name
        if not feeder.sensors:
            return
        end = min(feeder.sensors, key=lambda s: _distance_to_belts(s.x, s.y, self.main.belts))
        feeder.hold_sensor = end.id
        near = [s for s in self.main.sensors if math.hypot(s.x - end.x, s.y - end.y) <= MERGE_SENSOR_RANGE]
        if near:
            feeder.merge_sensor = min(near, key=lambda s: math.hypot(s.x - end.x, s.y - end.y)).id

    def line_of(self, component) -> Optional[Line]:
        key = line_of(component)
        return self.lines.get(key) if key else None

    def roles(self) -> Dict[str, Optional[str]]:
        out = {"main_line": self.main.name, "delivery_sensor": self.delivery_sensor}
        for line in self.lines.values():
            if line.feeds:
                out[f"{line.name} merge"] = (f"into {line.feeds}, holds at {line.hold_sensor}"
                                             + (f", yields to {line.merge_sensor}" if line.merge_sensor else ""))
        return out

    # -- the scan ------------------------------------------------------------------

    def step(self, now: Optional[float] = None) -> List[str]:
        now = time.monotonic() if now is None else now
        events: List[str] = []
        # Sensors and drives first, so a Reset pressed in the scan a jam
        # clears sees the sensor clear.
        self._sensors(now, events)
        dt = 0.0 if self._scan_at is None else max(0.0, now - self._scan_at)
        self._scan_at = now
        belts = {b.id: b for b in self.config.belts}

        def speed_of(belt_id):
            line = self.line_of(belts[belt_id]) if belt_id in belts else None
            return self.speeds.get(belt_id, 0.0) if line is not None and line.moving else 0.0
        measure_banners(self.banners, self.config, dt, speed_of, events)
        self._drive_faults(events)
        self._buttons(events)
        self._merges(events)
        self._write_outputs()
        self.machines.step(now, any(line.moving for line in self.lines.values()), self.speeds, events)
        return events

    def _buttons(self, events: List[str]):
        for button in self.config.buttons:
            pressed = devices.is_pressed_signal(button, button.sensor_states)
            edge = pressed and not self._pressed.get(button.id, False)
            self._pressed[button.id] = pressed
            if not edge:
                continue
            line = self.line_of(button)
            text = f"{getattr(button, 'label', '')} {button.id}".lower()
            targets = [line] if line is not None else list(self.lines.values())
            for target in targets:
                if "start" in text:
                    self._clear_jam(target, button.id, events)
                    self._start(target, button.id, events)
                elif "stop" in text:
                    self._stop(target, button.id, events)
                elif "reset" in text or "ack" in text:
                    self._clear_jam(target, button.id, events)
                    self.machines.reset_requested = True
                else:
                    events.append(f"button {button.id} pressed")
                    break

    def _start(self, line: Line, by: str, events: List[str]):
        if line.running:
            return
        if line.jammed is not None:
            events.append(f"{line.name} ({by}): can't start, sensor {line.jammed} is still jammed")
        elif line.faulted:
            events.append(f"{line.name} ({by}): can't start, drive fault on {', '.join(sorted(line.faulted))}")
        elif line.feeds and not self.lines[line.feeds].running:
            events.append(f"{line.name} ({by}): can't start - start {line.feeds}, the line it feeds, first")
        else:
            line.running = True
            events.append(f"{line.name} START ({by})")

    def _stop(self, line: Line, by: str, events: List[str]):
        if line.running:
            line.running = False
            events.append(f"{line.name} STOP ({by})")

    def _clear_jam(self, line: Line, by: str, events: List[str]):
        if line.jammed is None:
            return
        if self.sensors[line.jammed].occupied:
            events.append(f"{line.name} ({by}): jam at {line.jammed} not cleared - remove the parcel first")
            return
        events.append(f"{line.name}: jam at {line.jammed} cleared ({by})")
        line.jammed = None

    def _sensors(self, now: float, events: List[str]):
        for sensor in self.config.sensors:
            track = self.sensors.setdefault(sensor.id, SensorTrack())
            blocked = occupied(sensor)
            if blocked and not track.occupied:
                track.count += 1
                track.since = now
                if sensor.id == self.delivery_sensor:
                    self.delivered += 1
                    events.append(f"parcel at {sensor.id} (#{track.count}) - delivered {self.delivered}")
                else:
                    events.append(f"parcel at {sensor.id} (#{track.count})")
            elif blocked and (not self._moving_past(sensor) or sensor.id in waiting_sensors(self.config)):
                track.since = now   # waiting on purpose (holding / stopped / for the robot): not a jam
            track.occupied = blocked
            line = self.line_of(sensor)
            if (line is not None and blocked and line.jammed is None and line.moving
                    and now - track.since > self.jam_time):
                line.jammed = sensor.id
                events.append(f"{line.name} JAM at {sensor.id}: blocked for {now - track.since:.1f} s - line stopped")

    def _moving_past(self, sensor) -> bool:
        line = self.line_of(sensor)
        return line is None or line.moving

    def _drive_faults(self, events: List[str]):
        for line in self.lines.values():
            for belt in line.belts:
                if belt.fault and belt.id not in line.faulted:
                    line.faulted.add(belt.id)
                    line.running = False
                    events.append(f"{line.name} FAULT on {belt.id}: drive tripped - line stopped")
                elif not belt.fault and belt.id in line.faulted:
                    line.faulted.discard(belt.id)
                    events.append(f"{line.name}: drive {belt.id} fault cleared - press Start")

    def _merges(self, events: List[str]):
        for line in self.lines.values():
            if not line.feeds:
                continue
            target = self.lines[line.feeds]
            at_merge = line.hold_sensor is not None and self.sensors[line.hold_sensor].occupied
            busy = line.merge_sensor is not None and self.sensors[line.merge_sensor].occupied
            # A hold latches: the parcel coasts on past the sensor while the
            # belts run down, and must not be pushed on when the sensor clears -
            # only when the main line moves and the merge is free again.
            waiting = line.running and (at_merge or line.holding)
            blocked_target = waiting and not target.moving
            hold = blocked_target or (waiting and busy)
            if hold == line.holding:
                continue
            line.holding = hold
            # Giving way to a parcel at the merge is routine (counted, not logged);
            # holding because the main line stopped is worth a line in the log.
            if hold and not blocked_target:
                line.yields += 1
            elif hold:
                events.append(f"{line.name} HOLD at {line.hold_sensor} ({target.name} {target.state.lower()})")
            elif line.hold_logged:
                events.append(f"{line.name} released into {target.name}")
            line.hold_logged = hold and blocked_target

    def _write_outputs(self):
        for line in self.lines.values():
            for belt in line.belts:
                nominal = self.speeds.get(belt.id, DEFAULT_SPEED)
                if belt.commandFormat == "frequency":
                    _command_frequency(belt, line.moving, nominal)
                elif belt.commandFormat == "vfd":
                    belt.run = line.moving
                    belt.reverse = nominal < 0
                    belt.speed = abs(nominal)
                else:
                    belt.speed = nominal if line.moving else 0.0
            for led in line.leds:
                text = f"{getattr(led, 'label', '')} {led.id}".lower()
                error = line.jammed is not None or bool(line.faulted)
                if any(w in text for w in ("error", "jam", "fault", "alarm")):
                    led.isOn = error
                elif "hold" in text:
                    led.isOn = line.holding
                elif any(w in text for w in ("manual", "stop", "idle")):
                    led.isOn = not line.running and not error
                elif any(w in text for w in ("auto", "run", "ok", "green")):
                    led.isOn = line.running and not error

    def summary(self) -> str:
        lines = " | ".join(f"{line.name} {line.state}" + (f" (gave way {line.yields}x)" if line.yields else "")
                           for line in self.lines.values())
        counts = ", ".join(f"{sid} {t.count}" for sid, t in self.sensors.items()) or "no sensors"
        return f"{lines} | parcels: {counts} | delivered {self.delivered}"


def make_controller(config: SimulatorConfig, jam_time: float = 5.0, autostart: bool = False):
    """LineController for a layout with two or more lines, else the one-line
    DeliveryController."""
    if len(find_lines(config)) >= 2:
        return LineController(config, jam_time=jam_time, autostart=autostart)
    return DeliveryController(config, jam_time=jam_time, autostart=autostart)


@device_program("watchdogs")
def _watchdog(control, w, scan: Scan) -> None:
    """The PLC side of the link's heartbeat: counted up every scan (the
    simulator stops this program's commands when it stands still); the
    simulator's heartbeat of the input this scan answers, echoed (latency)."""
    w.plcHeartbeat = (int(w.plcHeartbeat) + 1) & 0xFFFF
    w.simHeartbeatEcho = int(w.simHeartbeat) & 0xFFFF
