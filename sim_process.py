"""
sim_process.py - the simulation in its own process, the window in another.

A thread (sim_runner.py) already keeps the simulation from waiting for the
window, but the two still share one Python interpreter lock: while the
window draws, physics doesn't run. Here the simulation - physics, the PLC
exchange (its TCP server, so the PLC connects to this process) and, with
--web-ui, the web dashboard - runs in a child process with a CPU core of
its own. The window process keeps a mirror of the layout and a
PhysicsProxy that looks like the PhysicsEngine to the renderer.

Each frame the window asks for one snapshot (pipelined: it never waits for
the child) - the parcels as ParcelViews plus the runtime state of belts,
sensors, lamps and buttons, taken under the physics lock so it is
consistent - and applies it to its mirror. Commands go the other way:
play / pause, speed, button press / release, clear parcels, reset.

    python main.py layout.json                   # the default for a window
    python main.py layout.json --no-sim-process  # simulation thread in the window's process
"""
import multiprocessing
import threading
import time
from typing import Dict, List, Optional

import devices
from config_parser import SimulatorConfig, load_config
from mqtt_data_exchange import apply_exchange_config
from physics import ParcelView

# Runtime fields copied from the simulation to the window's mirror.
BELT_FIELDS = ("speed", "run", "reverse", "actualSpeed", "running", "fault", "atSpeed", "encoderCount", "load",
               "level")
# Machines' runtime fields (machines.py), per config list.
MACHINE_FIELDS = {
    "pallets": ("loadCount",),
    "stoppers": ("isUp", "occupied", "release"),
    "robots": ("busy", "holding", "partAtPick", "placeReady", "cycles", "toolX", "toolY", "start", "phase", "phaseT",
               "ready", "atHome", "inPosition", "fault", "faultCode", "j1", "j2", "j3", "j4", "j5", "j6",
               "tcpX", "tcpY", "tcpZ"),
    "elevators": ("level", "moving", "atLevel", "occupied", "position", "targetLevel"),
    "scanners": ("code", "destination", "noRead", "readCount", "present", "weight", "measuredLength",
                 "measuredWidth", "measuredHeight"),
    "diverters": ("active", "present", "presentCode", "diverted", "divert"),
    "turntables": ("index", "turning", "inPosition", "occupied", "angle", "targetIndex"),
    "accumulators": ("zones", "full", "release"),
    "doors": ("closed", "locked", "lock"),
    "safety": ("ok", "tripped"),
    "sinks": ("count", "full"),
}
SNAPSHOT_TIMEOUT_S = 5.0


def _component_state(config: SimulatorConfig) -> dict:
    return {
        "belts": {b.id: tuple(getattr(b, f, None) for f in BELT_FIELDS) for b in config.belts},
        "sensors": {s.id: s.sensor_states for s in config.sensors},
        "buttons": {b.id: b.sensor_states for b in config.buttons},
        "leds": {l.id: l.isOn for l in config.leds},
        "machines": {kind: {c.id: tuple(getattr(c, f) for f in fields) for c in getattr(config, kind)}
                     for kind, fields in MACHINE_FIELDS.items()},
    }


def _apply_component_state(config: SimulatorConfig, state: dict) -> None:
    belts = state["belts"]
    for belt in config.belts:
        values = belts.get(belt.id)
        if values is not None:
            for field, value in zip(BELT_FIELDS, values):
                if value is not None:
                    setattr(belt, field, value)
    for kind, attr in (("sensors", "sensor_states"), ("buttons", "sensor_states"), ("leds", "isOn")):
        values = state[kind]
        for component in getattr(config, kind):
            if component.id in values:
                setattr(component, attr, values[component.id])
    for kind, fields in MACHINE_FIELDS.items():
        values = state.get("machines", {}).get(kind, {})
        for component in getattr(config, kind):
            row = values.get(component.id)
            if row is not None:
                for field, value in zip(fields, row):
                    setattr(component, field, value)


# ---------------------------------------------------------------------------
# The child process
# ---------------------------------------------------------------------------

def _opt_out_of_power_throttling() -> bool:
    """Windows 11 runs a process without a window of its own as background
    work ("EcoQoS": efficiency cores / lower clocks) - measured, it made the
    simulation process ~20 % slower than the same simulation in the
    window's process. Ask Windows to run it at full speed instead
    (SetProcessInformation / ProcessPowerThrottling). Best effort: False
    where that isn't available (other systems, older Windows)."""
    import sys
    if sys.platform != "win32":
        return False
    try:
        import ctypes
        from ctypes import wintypes

        class PowerThrottlingState(ctypes.Structure):
            _fields_ = [("Version", wintypes.ULONG), ("ControlMask", wintypes.ULONG),
                        ("StateMask", wintypes.ULONG)]

        process_power_throttling = 4                 # PROCESS_INFORMATION_CLASS.ProcessPowerThrottling
        execution_speed = 0x1                        # PROCESS_POWER_THROTTLING_EXECUTION_SPEED
        state = PowerThrottlingState(1, execution_speed, 0)   # control it, and turn it off
        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        kernel32.SetProcessInformation.argtypes = (wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD)
        kernel32.SetProcessInformation.restype = wintypes.BOOL
        ok = kernel32.SetProcessInformation(kernel32.GetCurrentProcess(), process_power_throttling,
                                            ctypes.byref(state), ctypes.sizeof(state))
        return bool(ok)
    except Exception:  # noqa: BLE001 - an optimisation: never a reason not to simulate
        return False


def simulation_main(conn, settings: dict) -> None:
    """Entry point of the simulation process: build everything from the
    layout file, then serve commands and snapshot requests until "quit"
    (or the window process goes away)."""
    from main import build_engine, start_web_ui        # here: main imports pygame for the window side
    from metrics_store import MetricsStore
    from physics import PhysicsEngine
    from sim_runner import SimulationRunner

    _opt_out_of_power_throttling()
    config = load_config(settings["config"])
    if settings.get("exchange_config"):
        apply_exchange_config(config, settings["exchange_config"])
    lock = threading.Lock()
    physics = PhysicsEngine(config, lock=lock)
    metrics_store = MetricsStore()
    exchange = build_engine(config, lock, metrics_store, channel_registry_path=settings.get("channel_registry"),
                            show_layout=settings.get("show_layout", False))
    if settings.get("web_ui"):
        start_web_ui(settings["config"], settings.get("channel_registry"), metrics_store,
                     host=settings.get("web_host", "127.0.0.1"), port=settings.get("web_port", 5050))
    runner = SimulationRunner(physics, exchange, tick_s=settings.get("tick_s", 0.002), speed=settings.get("speed", 1.0))
    runner.playing = False
    runner.start()
    by_id = {c.id: c for c in config.buttons}
    try:
        conn.send(("ready", None))
        while True:
            if not conn.poll(0.5):
                continue
            cmd, arg = conn.recv()
            if cmd == "snapshot":
                with lock:
                    snap = {"parcels": physics.parcel_views(), "state": _component_state(config),
                            "sim_time": physics.sim_time, "delivered": dict(physics.delivered),
                            "dropped_time": physics.dropped_time}
                snap["exchange"] = exchange.stats() if getattr(exchange, "_enabled", False) else None
                snap["playing"], snap["speed"] = runner.playing, runner.speed
                conn.send(("snapshot", snap))
            elif cmd == "play":
                runner.playing = bool(arg)
            elif cmd == "speed":
                runner.speed = arg
            elif cmd == "button":
                button_id, pressed = arg
                button = by_id.get(button_id)
                if button is not None:
                    with lock:
                        (devices.press if pressed else devices.release)(button)
            elif cmd == "door":
                with lock:
                    physics.logistics.toggle_door(arg)
            elif cmd == "clear":
                physics.clear_parcels()
            elif cmd == "reset":
                physics.reset()
            elif cmd == "quit":
                break
    except (EOFError, BrokenPipeError, ConnectionResetError, KeyboardInterrupt):
        pass                                   # the window process is gone
    finally:
        runner.stop()
        exchange.stop()
        try:
            conn.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# The window side
# ---------------------------------------------------------------------------

class _ExchangeView:
    """What the status bar needs of the exchange: its last stats."""

    def __init__(self):
        self._enabled = False
        self._stats: dict = {}

    def stats(self) -> dict:
        return self._stats


class PhysicsProxy:
    """Stands in for the PhysicsEngine in the window process; the real one
    runs in a child process (simulation_main). Call poll() once per frame."""

    def __init__(self, config: SimulatorConfig, settings: dict):
        self.config = config                   # the window's mirror of the layout
        self.settings = settings
        self.sim_time = 0.0
        self.delivered: Dict[str, int] = {}
        self.dropped_time = 0.0
        self.exchange = _ExchangeView()
        self._views: List[ParcelView] = []
        self._pending = False                  # a snapshot request is out
        self._playing: Optional[bool] = None
        self._speed = object()                 # nothing sent yet
        self.snapshots = 0
        self.process: Optional[multiprocessing.Process] = None
        self._conn = None

    # -- lifecycle -----------------------------------------------------------------

    def start(self, timeout: float = 60.0) -> None:
        ctx = multiprocessing.get_context("spawn")
        self._conn, child = ctx.Pipe()
        self.process = ctx.Process(target=simulation_main, args=(child, self.settings),
                                   name="Simulation", daemon=True)
        self.process.start()
        child.close()
        end = time.monotonic() + timeout
        while not self._conn.poll(0.2):
            if not self.process.is_alive() or time.monotonic() > end:
                code = self.process.exitcode
                self.stop()
                raise RuntimeError("the simulation process did not start"
                                   + (f" (exit code {code}, see its output above)" if code is not None else ""))
        try:
            msg = self._conn.recv()
        except (EOFError, OSError):
            code = self.process.exitcode
            self.stop()
            raise RuntimeError("the simulation process stopped while starting"
                               + (f" (exit code {code}, see its output above)" if code is not None else ""))
        if msg[0] != "ready":
            raise RuntimeError(f"unexpected first message from the simulation process: {msg!r}")

    def stop(self) -> None:
        if self._conn is not None:
            try:
                self._conn.send(("quit", None))
            except (OSError, BrokenPipeError):
                pass
        if self.process is not None:
            self.process.join(timeout=5)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2)
        if self._conn is not None:
            self._conn.close()
            self._conn = None

    @property
    def alive(self) -> bool:
        return self.process is not None and self.process.is_alive()

    # -- per frame -----------------------------------------------------------------

    def _send(self, cmd: str, arg=None) -> None:
        try:
            self._conn.send((cmd, arg))
        except (OSError, BrokenPipeError):
            pass                               # the simulation process is gone; alive says so

    def poll(self, wait: float = 0.0) -> bool:
        """Take the newest snapshot if one has arrived, and ask for the next
        one. Returns True when a snapshot was applied. `wait` > 0 blocks up
        to that long for it (tests, start-up)."""
        got = False
        try:
            while self._pending and self._conn.poll(wait if not got else 0):
                kind, snap = self._conn.recv()
                if kind == "snapshot":
                    self._apply(snap)
                    self._pending = False
                    got = True
        except (EOFError, OSError):
            return False
        if not self._pending:
            self._send("snapshot")
            self._pending = True
        return got

    def _apply(self, snap: dict) -> None:
        self._views = snap["parcels"]
        _apply_component_state(self.config, snap["state"])
        self.sim_time = snap["sim_time"]
        self.delivered = snap["delivered"]
        self.dropped_time = snap["dropped_time"]
        stats = snap.get("exchange")
        self.exchange._enabled = stats is not None
        self.exchange._stats = stats or {}
        self.snapshots += 1

    # -- the PhysicsEngine interface the renderer uses -----------------------------

    def parcel_views(self) -> List[ParcelView]:
        return self._views

    @property
    def parcels(self) -> List[ParcelView]:
        return self._views

    def set_playing(self, playing: bool) -> None:
        if playing != self._playing:
            self._playing = playing
            self._send("play", playing)

    def set_speed(self, speed: Optional[float]) -> None:
        if speed != self._speed:
            self._speed = speed
            self._send("speed", speed)

    def set_button(self, button_id: str, pressed: bool) -> None:
        self._send("button", (button_id, pressed))

    def toggle_door(self, door_id: str) -> None:
        """The operator opens / closes a guard door (logistics.py)."""
        self._send("door", door_id)

    def clear_parcels(self) -> None:
        self._send("clear")
        self._views = []

    def reset(self) -> None:
        self._send("reset")
        self._views = []


def wait_for(proxy: PhysicsProxy, predicate, timeout: float = 10.0) -> bool:
    """Poll the proxy until predicate() holds (tests, scripts)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        proxy.poll(wait=0.05)
        if predicate():
            return True
    return False
