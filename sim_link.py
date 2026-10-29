"""
sim_link.py - one simulator window and one designer at a time.

Each app listens on a localhost port and notes it, with its pid, in
<role>_instance.json next to the hmi settings file (sim_instance.json,
designer_instance.json). Before starting the other app, each first asks the
running one:

  designer  Run in Simulator  -> the simulator reloads the saved layout
                                 (request_reload; main.py runs layouts in a loop)
  simulator Edit in Designer  -> the designer comes to the front with the
                                 layout (request_open_designer)

Only if none answers is a new process started. Two simulators would also
fight over the PLC exchange port.

The designer's Data Exchange section also reads a component's PLC signals
live from the simulator and sets them by hand (io_signals.py):

  request_io(ids)                 -> the values, or None (no simulator)
  request_set(id, name, value)    -> queued for the simulator's next frame

Protocol: one JSON line each way.
  -> {"cmd": "reload" | "open", "config": "<layout.json>", "source": ...}
  <- {"ok": true, "pid": 1234}
  -> {"cmd": "io", "ids": ["belt_1", ...]}
  <- {"ok": true, "pid": 1234, "config": ..., "source": ..., "values": {...}}
  -> {"cmd": "set", "id": "div_1", "name": "divert", "value": true}
  <- {"ok": true, "pid": 1234}
"""
import json
import os
import socket
import sys
import threading
from typing import Optional

import hmi_style as hmi

TIMEOUT_S = 2.0
IO_TIMEOUT_S = 0.3          # the designer polls io several times a second: never hang it
COMMANDS = {"sim": "reload", "designer": "open"}


def registry_path(role: str = "sim") -> str:
    return os.path.join(os.path.dirname(os.path.abspath(hmi.settings_path())), f"{role}_instance.json")


def _read_registry(role: str) -> Optional[dict]:
    try:
        with open(registry_path(role), "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) and "port" in data else None
    except (OSError, ValueError):
        return None


def same_layout(a: Optional[str], b: Optional[str]) -> bool:
    return bool(a and b) and os.path.normcase(os.path.abspath(a)) == os.path.normcase(os.path.abspath(b))


class InstanceLink:
    """An app's listener: a thread that queues requested layouts;
    `take()` hands the app's own loop the latest one (once), or None."""

    def __init__(self, role: str, config_path: Optional[str] = None, source: Optional[str] = None):
        self.role = role
        self.config_path = os.path.abspath(config_path) if config_path else None
        self.source = os.path.abspath(source) if source else None
        self._pending: Optional[str] = None
        self.pending_source: Optional[str] = None       # the layout file a run copy was made from
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self.port = self._sock.getsockname()[1]
        self._closed = False
        threading.Thread(target=self._serve, name=f"{role}-link", daemon=True).start()
        self._write_registry()

    def _write_registry(self):
        path = registry_path(self.role)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"pid": os.getpid(), "port": self.port, "config": self.config_path,
                           "source": self.source}, f)
        except OSError as exc:
            print(f"[{self.role}-link] Could not write {path}: {exc}")

    def set_config(self, config_path: Optional[str], source: Optional[str] = None):
        self.config_path = os.path.abspath(config_path) if config_path else None
        self.source = os.path.abspath(source) if source else None
        self._write_registry()

    def _handle(self, request: dict) -> dict:
        """The reply to one request (on the listener thread)."""
        if request.get("cmd") == COMMANDS[self.role] and request.get("config"):
            with self._lock:
                self._pending = os.path.abspath(request["config"])
                source = request.get("source")
                self.pending_source = os.path.abspath(source) if source else None
            return {"ok": True}
        return {"ok": False}

    def _serve(self):
        while not self._closed:
            try:
                conn, _addr = self._sock.accept()
            except OSError:
                return
            with conn:
                try:
                    conn.settimeout(TIMEOUT_S)
                    request = json.loads(conn.makefile("r", encoding="utf-8").readline() or "{}")
                    reply = self._handle(request if isinstance(request, dict) else {})
                    reply["pid"] = os.getpid()
                    conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
                except (OSError, ValueError):
                    pass

    def take(self) -> Optional[str]:
        with self._lock:
            path, self._pending = self._pending, None
        return path

    def close(self):
        self._closed = True
        try:
            self._sock.close()
        except OSError:
            pass
        entry = _read_registry(self.role)
        if entry and entry.get("pid") == os.getpid():
            try:
                os.remove(registry_path(self.role))
            except OSError:
                pass


class SimulatorLink(InstanceLink):
    """The simulator's listener: reload requests, plus the Data Exchange
    section's reads (answered here, from `config` - the layout the window
    shows) and hand-set values (queued: take_commands() hands them to the
    window loop, which applies them to the simulation)."""

    def __init__(self, config_path: Optional[str], source: Optional[str] = None):
        self.config = None          # set by the window loop (main.run_pygame)
        self._commands: list = []
        super().__init__("sim", config_path, source)

    def take_reload(self) -> Optional[str]:
        return self.take()

    def take_commands(self) -> list:
        """[(component id, signal name, value), ...] set since the last call."""
        with self._lock:
            commands, self._commands = self._commands, []
        return commands

    def _handle(self, request: dict) -> dict:
        cmd = request.get("cmd")
        if cmd == "io":
            import io_signals
            config = self.config
            ids = [str(i) for i in request.get("ids") or []]
            values = io_signals.read(config, ids) if config is not None else {}
            return {"ok": config is not None, "config": self.config_path, "source": self.source,
                    "values": values}
        if cmd == "set" and request.get("id") and request.get("name"):
            with self._lock:
                self._commands.append((str(request["id"]), str(request["name"]), request.get("value")))
            return {"ok": True}
        return super()._handle(request)


def _exchange(entry: dict, request: dict, timeout: float) -> Optional[dict]:
    """Send one request to the app noted in `entry`; its reply, or None."""
    try:
        with socket.create_connection(("127.0.0.1", int(entry["port"])), timeout=timeout) as conn:
            conn.settimeout(timeout)
            conn.sendall((json.dumps(request) + "\n").encode("utf-8"))
            reply = json.loads(conn.makefile("r", encoding="utf-8").readline() or "{}")
    except (OSError, ValueError, KeyError, TypeError):
        return None
    return reply if isinstance(reply, dict) else None


def _request(role: str, config_path: str, source: Optional[str] = None) -> Optional[int]:
    entry = _read_registry(role)
    if entry is None:
        return None
    if sys.platform == "win32" and entry.get("pid"):
        # Let that app bring its window to the front (Windows only allows it
        # when the app in front - this one, just clicked - says so).
        try:
            import ctypes
            ctypes.windll.user32.AllowSetForegroundWindow(int(entry["pid"]))
        except (OSError, AttributeError, ValueError):
            pass
    request = {"cmd": COMMANDS[role], "config": os.path.abspath(config_path)}
    if source:
        request["source"] = os.path.abspath(source)
    reply = _exchange(entry, request, TIMEOUT_S)
    return reply.get("pid") if reply and reply.get("ok") else None


def request_reload(config_path: str, source: Optional[str] = None) -> Optional[int]:
    """Ask the running simulator to reload `config_path` - a run copy of the
    layout file `source`, when given (its Edit in Designer opens that one).
    Its pid if it agreed, None if no simulator is running (or it didn't
    answer)."""
    return _request("sim", config_path, source)


def request_open_designer(config_path: str) -> Optional[int]:
    """Ask the running designer to show `config_path` (front window, opening
    it if another layout is open). Its pid, or None if none is running."""
    return _request("designer", config_path)


def request_io(ids, timeout: float = IO_TIMEOUT_S) -> Optional[dict]:
    """The running simulator's live PLC signals of the components `ids`
    (io_signals.read), with the layout it runs ("config", and "source" when
    that is a run copy); None if no simulator answers."""
    entry = _read_registry("sim")
    if entry is None:
        return None
    reply = _exchange(entry, {"cmd": "io", "ids": list(ids)}, timeout)
    return reply if reply and reply.get("ok") else None


def request_set(component_id: str, name: str, value, timeout: float = IO_TIMEOUT_S) -> bool:
    """Ask the running simulator to set a component's `control` or one of
    its signals by hand (io_signals.apply, on its next frame)."""
    entry = _read_registry("sim")
    if entry is None:
        return False
    reply = _exchange(entry, {"cmd": "set", "id": component_id, "name": name, "value": value}, timeout)
    return bool(reply and reply.get("ok"))


def runs_layout(reply: Optional[dict], *paths: Optional[str]) -> bool:
    """Whether a request_io reply comes from a simulator running one of
    these layout files (as itself or as its run copy)."""
    if not reply:
        return False
    return any(same_layout(reply.get(key), path) for key in ("config", "source") for path in paths)
