"""
plc_logic.py - a small, generic PLC program for any conveyor layout.

DeliveryController works on the PLC side of the exchange: it reads the I/O
image the simulator sends (buttons and sensors) and writes the commands the
simulator applies (belt speeds and LEDs) into the same SimulatorConfig
object that ComponentMessage serializes - so it runs unchanged over TCP or
MQTT (plc_mock.py) or directly in a test.

Behaviour, once per scan (step()):
  Buttons (user input), picked by label/id:
    "start"            -> run
    "stop"             -> stop
    "reset"            -> clear a jam
    no such labels     -> one button toggles run/stop; with two or more the
                          first starts and the second stops
  Sensors (parcel triggers): each rising edge is a parcel passing; counted
    per sensor. The sensor nearest a sink counts deliveries. A sensor that
    stays blocked longer than `jam_time` is a jam: every belt stops until
    Start/Reset is pressed with the sensor clear.
  Belts: running and no jam -> each belt's design speed (its speed in the
    layout file, 1 m/s where that is 0), else 0.
  LEDs, picked by label/id: "run"/"ok" (else the first LED) shows running,
    "jam"/"fault"/"alarm"/"error" (else the second LED) shows a jam.

step() returns the events of that scan as text, for logs.
"""
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from config_parser import SimulatorConfig

DEFAULT_SPEED = 1.0  # m/s for belts whose layout speed is 0


def is_on(value) -> bool:
    """A button/sensor/LED state as received: bool, int, or "0"/"1" text."""
    if isinstance(value, str):
        return value.strip() not in ("", "0", "false", "False")
    return bool(value)


def _pick(components, words, fallback_index: Optional[int]):
    for c in components:
        text = f"{getattr(c, 'label', '')} {c.id}".lower()
        if any(w in text for w in words):
            return c
    if fallback_index is not None and len(components) > fallback_index:
        return components[fallback_index]
    return None


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
        self.run_led = _pick(leds, ("run", "ok", "green"), 0)
        self.jam_led = _pick([l for l in leds if l is not self.run_led], ("jam", "fault", "alarm", "error"), 0)
        self.speeds = {b.id: (b.speed if abs(b.speed) > 1e-9 else DEFAULT_SPEED) for b in cfg.belts}
        self.delivery_sensor = self._sensor_nearest_a_sink()
        self._pressed = {b.id: False for b in buttons}
        self.sensors = {s.id: SensorTrack() for s in cfg.sensors}
        self.running = self.autostart
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
        self._write_outputs()
        return events

    def _buttons(self, events: List[str]):
        for button in self.config.buttons:
            pressed = is_on(button.sensor_states)
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
            occupied = is_on(sensor.sensor_states)
            if occupied and not track.occupied:
                track.count += 1
                track.since = now
                if sensor.id == self.delivery_sensor:
                    self.delivered += 1
                    events.append(f"parcel at {sensor.id} (#{track.count}) - delivered {self.delivered}")
                else:
                    events.append(f"parcel at {sensor.id} (#{track.count})")
            track.occupied = occupied
            if occupied and self.jammed is None and self.running and now - track.since > self.jam_time:
                self.jammed = sensor.id
                events.append(f"JAM at {sensor.id}: blocked for {now - track.since:.1f} s - belts stopped")

    def _write_outputs(self):
        run = self.running and self.jammed is None
        for belt in self.config.belts:
            belt.speed = self.speeds.get(belt.id, DEFAULT_SPEED) if run else 0.0
        if self.run_led is not None:
            self.run_led.isOn = run
        if self.jam_led is not None:
            self.jam_led.isOn = self.jammed is not None

    def summary(self) -> str:
        counts = ", ".join(f"{sid} {t.count}" for sid, t in self.sensors.items()) or "no sensors"
        state = "JAM at " + self.jammed if self.jammed else ("RUN" if self.running else "STOP")
        return f"{state} | parcels: {counts} | delivered {self.delivered}"
