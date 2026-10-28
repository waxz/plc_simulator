"""
plc_logic.py - a small, generic PLC program for any conveyor layout.

DeliveryController works on the PLC side of the exchange: it reads the I/O
image the simulator sends (buttons and sensors) and writes the commands the
simulator applies (belt speeds and LEDs) into the same SimulatorConfig
object that ComponentMessage serializes - so it runs unchanged over TCP or
MQTT (plc_mock.py) or directly in a test.

Behaviour, once per scan (step()):
  Buttons (user input), picked by label/id; an NC contact
  (ControlButton.normallyClosed, e.g. STOP) reads 0 when pressed:
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
  Belts: running, no jam, no drive fault -> each belt's design speed (its
    speed in the layout file, 1 m/s where that is 0; negative = reverse),
    else stopped. commandFormat "speed": `speed` (signed); "vfd": `run`,
    `reverse` and `speed` (magnitude).
  Drive faults (Belt.fault in the motor feedback, BELT_FEEDBACK frame):
    stop the line; it can be started again once every fault has cleared
    (a drive resets when its run command is removed).
  LEDs, picked by label/id: "run"/"ok"/"auto" (else the first LED) shows
    running, "jam"/"fault"/"alarm"/"error" (else the second LED) a jam or a
    drive fault.

Machines with control "plc" (machines.py: stoppers, robot arms, elevators)
are run by MachineControl, with the same decisions as their built-in auto
mode, while the line runs: a stopper holds what arrives for its holdTime and
then lets it pass, a robot starts a cycle when a part waits and there is
somewhere to put it, an elevator loads, lifts, unloads and comes back
(running its platform belt only for that). Stopped, every stopper stays up.

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
from typing import Dict, List, Optional, Set

import devices
import machines
from config_parser import SimulatorConfig

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
    if belt.commandFormat == "vfd" and belt.reverse:
        speed = -abs(speed)
    return speed


LOADED_AFTER_S = 0.5   # elevator: a part on the platform this long is on board (its belt stops)


class MachineControl:
    """The PLC side of control "plc" machines: writes their commands
    (stopper release, robot start, elevator targetLevel and platform belt)
    from their feedback, once per scan, after the belts' own outputs."""

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self._stoppers: Dict[str, dict] = {}
        self._elevators: Dict[str, dict] = {}
        self._last: Optional[float] = None

    def step(self, now: float, running: bool, speeds: Dict[str, float], events: List[str]) -> None:
        dt = 0.0 if self._last is None else max(0.0, now - self._last)
        self._last = now
        cfg = self.config
        for stopper in cfg.stoppers:
            if stopper.control != "plc":
                continue
            state = self._stoppers.setdefault(stopper.id, {})
            release = running and not machines.stopper_wants_up(stopper, state, dt)
            if release != stopper.release:
                events.append(f"stopper {stopper.id} {'released' if release else 'up'}")
            stopper.release = release
        for robot in cfg.robots:
            if robot.control == "plc":
                robot.start = running and not robot.busy and robot.partAtPick and robot.placeReady
        belts = {b.id: b for b in cfg.belts}
        for elevator in cfg.elevators:
            if elevator.control != "plc":
                continue
            state = self._elevators.setdefault(elevator.id, {})
            on_board = elevator.occupied and elevator.atLevel
            state["on_s"] = state.get("on_s", 0.0) + dt if on_board else 0.0
            state["loaded"] = state["on_s"] >= LOADED_AFTER_S
            target, run = machines.elevator_auto(elevator, state, dt)
            if not running:
                run = False
            elif target != elevator.targetLevel:
                events.append(f"elevator {elevator.id} to level {target}")
                elevator.targetLevel = target
            belt = belts.get(elevator.beltId) if elevator.beltId else None
            if belt is not None:
                nominal = speeds.get(belt.id, DEFAULT_SPEED)
                if belt.commandFormat == "vfd":
                    belt.run = run
                    belt.speed = abs(nominal)
                else:
                    belt.speed = nominal if run else 0.0


@dataclass
class SensorTrack:
    occupied: bool = False
    since: float = 0.0
    count: int = 0


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
                self._set_running(True, button.id, events)
            elif button is self.stop_button:
                self._set_running(False, button.id, events)
            elif button is self.reset_button:
                self._clear_jam(button.id, events)
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
            if belt.commandFormat == "vfd":
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
            elif blocked and not self._moving_past(sensor):
                track.since = now   # waiting on purpose (holding / stopped): not a jam
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
                if belt.commandFormat == "vfd":
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
