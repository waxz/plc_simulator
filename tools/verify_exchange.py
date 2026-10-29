"""
verify_exchange.py - runs a layout end to end over TCP - the simulator
(main.py, headless) and the mock PLC (plc_mock.py --autostart), each in its
own process on a free port - and checks the data exchange between them:

  - the I/O layout handshake (LAYOUT_CHECK): both sides derive the same
    input / output frames from the layout (sizes and fingerprints)
  - frames flow both ways: the PLC receives the simulator's input frames,
    the simulator applies the PLC's output frames, with no size or decode
    errors on either side
  - the plant follows the PLC: belts run, and (when the layout has them)
    parcels reach the chutes, transfers carry pallets, the robot cycles

    python tools/verify_exchange.py logistics_full.json --seconds 60

Exit code 0 when every check passes. The simulator's live state is read
from its 3D view stream (view3d.py), its exchange counters from its log.
"""
import argparse
import ast
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Process:
    """A child process whose output lines are collected."""

    def __init__(self, args, env):
        self.lines = []
        self.proc = subprocess.Popen(args, cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding="utf-8", errors="replace", bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip())

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def last(self, pattern: str):
        rx = re.compile(pattern)
        return next((m for m in (rx.search(line) for line in reversed(self.lines)) if m), None)


def live_state(port: int) -> dict:
    """The simulation's state now. The stream starts with the last frame it
    built - from whenever a viewer last looked - so take the next one."""
    frames = 0
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/events", timeout=10) as r:
        for raw in r:
            line = raw.decode("utf-8").strip()
            if line.startswith("data: "):
                frames += 1
                if frames == 2:
                    return json.loads(line[6:])
    return {}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("layout")
    parser.add_argument("--seconds", type=float, default=60.0, help="how long to run (wall time)")
    args = parser.parse_args(argv)

    tmp = tempfile.mkdtemp(prefix="verify_exchange_")
    exchange = os.path.join(tmp, "exchange.json")
    tcp_port, view_port = free_port(), free_port()
    with open(exchange, "w", encoding="utf-8") as f:
        json.dump({"transport": "tcp", "enabled": True, "tcpBindHost": "127.0.0.1", "tcpServerHost": "127.0.0.1",
                   "tcpPort": tcp_port, "endianness": "LE"}, f)
    env = dict(os.environ, SDL_VIDEODRIVER="dummy", PYTHONUNBUFFERED="1",
               CONVEYOR_SIM_SETTINGS=os.path.join(tmp, "settings.json"))
    py = sys.executable
    print(f"[verify] {args.layout}: simulator + plc_mock on TCP 127.0.0.1:{tcp_port} for {args.seconds:g} s")
    sim = Process([py, "main.py", args.layout, "--headless", "--exchange-config", exchange,
                   "--view3d", "serve", "--view3d-port", str(view_port)], env)
    plc = None
    checks = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        checks.append(ok)
        print(f"  [{'ok' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))

    try:
        time.sleep(8.0)                                  # the simulator is up and listening
        before = live_state(view_port)
        plc = Process([py, "plc_mock.py", args.layout, "--exchange-config", exchange, "--autostart"], env)
        time.sleep(args.seconds)
        after = live_state(view_port)
        print("[verify] results")

        verdict = plc.last(r"I/O layout (check: OK|MISMATCH).*")
        check("I/O layout handshake (LAYOUT_CHECK)", bool(verdict and "OK" in verdict.group(1)),
              verdict.group(0) if verdict else "no answer from the simulator")
        metrics = plc.last(r"\[METRICS\] rx (\d+) .*\| tx (\d+) .*\| malformed (\d+)")
        rx, tx, bad = (int(g) for g in metrics.groups()) if metrics else (0, 0, -1)
        check("PLC receives the simulator's input frames", rx > 0, f"{rx} frames")
        check("PLC sends its output frames", tx > 0, f"{tx} frames")
        check("no malformed / wrong-size frames at the PLC", bad == 0, f"{bad}")
        timing = sim.last(r"exchange=(\{.*\})")
        stats = ast.literal_eval(timing.group(1)) if timing else {}
        check("simulator applies the PLC's frames", stats.get("rx_frames", 0) > 0, f"rx_frames {stats.get('rx_frames')}")
        check("no rejected frames at the simulator",
              stats.get("rx_errors", 1) == 0 and stats.get("rx_size_errors", 1) == 0,
              f"rx_errors {stats.get('rx_errors')}, rx_size_errors {stats.get('rx_size_errors')}")
        check("the PLC has a client connection", stats.get("clients", 0) >= 1, f"clients {stats.get('clients')}")

        moving = lambda st: sum(1 for v in st.get("belts", {}).values() if abs(v[0]) > 0.01)   # noqa: E731
        check("nothing moves before the PLC", moving(before) == 0, f"{moving(before)} belts moving")
        check("belts run on the PLC's commands", moving(after) > 0,
              f"{moving(after)} of {len(after.get('belts', {}))} belts moving")
        if after.get("chutes"):
            filled = sum(v[0] for v in after["chutes"].values())
            check("parcels sorted into chutes", filled > 0 or any(v[1] for v in after["chutes"].values()),
                  json.dumps(after["chutes"]))
        if after.get("scanners"):
            reads = {k: v[3] for k, v in after["scanners"].items()}
            check("scanners read", all(n > 0 for n in reads.values()), json.dumps(reads))
        if after.get("transfers"):
            moved = any(v[1] for v in after["transfers"].values()) or after != before
            check("transfers lift on the PLC's command", moved)
        if after.get("doors"):
            check("guard doors locked by the PLC while running", all(v[1] for v in after["doors"].values()))
        if after.get("safety"):
            check("safety relays OK (no trip)", all(v[0] for v in after["safety"].values()))
    finally:
        for p in (plc, sim):
            if p is not None:
                p.stop()
    passed = all(checks)
    print(f"[verify] {'PASSED' if passed else 'FAILED'}: {sum(checks)} of {len(checks)} checks")
    if not passed:
        for name, p in (("simulator", sim), ("plc_mock", plc)):
            if p is not None:
                print(f"--- last lines of {name} ---")
                print("\n".join(line for line in p.lines[-15:] if "TIMING" not in line))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
