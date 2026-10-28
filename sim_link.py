"""
sim_link.py - one simulator window at a time.

The simulator window (main.py) listens on a localhost port and notes it, with
its pid and layout, in sim_instance.json next to the hmi settings file. The
designer's Run in Simulator first asks that simulator to reload the saved
layout (request_reload); only if none answers does it start a new one. Two
simulators would also fight over the PLC exchange port.

Protocol: one JSON line each way.
  -> {"cmd": "reload", "config": "<layout.json>"}
  <- {"ok": true, "pid": 1234}
"""
import json
import os
import socket
import threading
from typing import Optional

import hmi_style as hmi

TIMEOUT_S = 2.0


def registry_path() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(hmi.settings_path())), "sim_instance.json")


def _read_registry() -> Optional[dict]:
    try:
        with open(registry_path(), "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) and "port" in data else None
    except (OSError, ValueError):
        return None


class SimulatorLink:
    """The simulator's side: a listener thread; `take_reload()` hands the
    main loop a layout to switch to (once), or None."""

    def __init__(self, config_path: Optional[str]):
        self.config_path = os.path.abspath(config_path) if config_path else None
        self._pending: Optional[str] = None
        self._lock = threading.Lock()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self.port = self._sock.getsockname()[1]
        self._closed = False
        threading.Thread(target=self._serve, name="sim-link", daemon=True).start()
        self._write_registry()

    def _write_registry(self):
        path = registry_path()
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump({"pid": os.getpid(), "port": self.port, "config": self.config_path}, f)
        except OSError as exc:
            print(f"[SimLink] Could not write {path}: {exc}")

    def set_config(self, config_path: str):
        self.config_path = os.path.abspath(config_path)
        self._write_registry()

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
                    ok = request.get("cmd") == "reload" and bool(request.get("config"))
                    if ok:
                        with self._lock:
                            self._pending = os.path.abspath(request["config"])
                    conn.sendall((json.dumps({"ok": ok, "pid": os.getpid()}) + "\n").encode("utf-8"))
                except (OSError, ValueError):
                    pass

    def take_reload(self) -> Optional[str]:
        with self._lock:
            path, self._pending = self._pending, None
        return path

    def close(self):
        self._closed = True
        try:
            self._sock.close()
        except OSError:
            pass
        entry = _read_registry()
        if entry and entry.get("pid") == os.getpid():
            try:
                os.remove(registry_path())
            except OSError:
                pass


def request_reload(config_path: str) -> Optional[int]:
    """Ask the running simulator to reload `config_path`. Its pid if it
    agreed, None if no simulator is running (or it didn't answer)."""
    entry = _read_registry()
    if entry is None:
        return None
    try:
        with socket.create_connection(("127.0.0.1", int(entry["port"])), timeout=TIMEOUT_S) as conn:
            conn.settimeout(TIMEOUT_S)
            conn.sendall((json.dumps({"cmd": "reload", "config": os.path.abspath(config_path)}) + "\n")
                         .encode("utf-8"))
            reply = json.loads(conn.makefile("r", encoding="utf-8").readline() or "{}")
    except (OSError, ValueError):
        return None
    return reply.get("pid") if reply.get("ok") else None
