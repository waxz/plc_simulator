"""
tools/mock_soak.py - run a layout under plc_mock.py and report what goes wrong.

For each layout: the simulator headless (main.py --headless, its TCP
exchange on a free local port) and plc_mock.py (--autostart) as its PLC,
for --seconds of wall time. Then a summary from what both printed:

  sim / real    simulated seconds per wall second (--speed max: the throughput)
  delivered     loads delivered (the [TIMING] lines' exchange stats and the mock's)
  stalls        [STALL] lines (sim_runner / stall_watch)
  errors        tracebacks, "Error", "Exception", layout mismatches, size errors

    python tools/mock_soak.py merge_demo.json complete_plant.json --seconds 60
    python tools/mock_soak.py --all --seconds 30 --speed max
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAYOUTS = ("merge_demo.json", "station_1.json", "logistics_demo.json", "machines_demo.json",
           "complete_plant.json", "logistics_full.json")
PROBLEM = re.compile(r"Traceback|Error|Exception|MISMATCH|mismatch|Ignoring .* frames|\[STALL\]")


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _pump(proc, sink):
    for line in proc.stdout:
        sink.append(line.rstrip())


def soak(layout: str, seconds: float, speed: str) -> dict:
    tmp = tempfile.mkdtemp()
    exchange = os.path.join(tmp, "exchange.json")
    with open(exchange, "w", encoding="utf-8") as f:
        json.dump({"transport": "tcp", "enabled": True, "tcpBindHost": "127.0.0.1", "tcpServerHost": "127.0.0.1",
                   "tcpPort": free_port(), "endianness": "LE"}, f)
    env = dict(os.environ, PYTHONUNBUFFERED="1", SDL_VIDEODRIVER="dummy")
    sim_out, plc_out = [], []
    sim = subprocess.Popen([sys.executable, "main.py", layout, "--headless", "--exchange-config", exchange,
                            "--speed", speed], cwd=HERE, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, encoding="utf-8", errors="replace")
    threading.Thread(target=_pump, args=(sim, sim_out), daemon=True).start()
    time.sleep(4.0)                                   # the simulator's server up
    plc = subprocess.Popen([sys.executable, "plc_mock.py", layout, "--exchange-config", exchange, "--autostart",
                            "--send-interval", "5"], cwd=HERE, env=env, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    threading.Thread(target=_pump, args=(plc, plc_out), daemon=True).start()
    started = time.monotonic()
    while time.monotonic() - started < seconds and sim.poll() is None and plc.poll() is None:
        time.sleep(0.5)
    for proc in (plc, sim):
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
    time.sleep(0.3)
    timing = [l for l in sim_out if l.startswith("[TIMING]")]
    last = timing[-1] if timing else ""
    ratio = re.search(r"ratio=([\d.]+)x", last)
    sim_t = re.search(r"sim=([\d.]+)s", last)
    dropped = re.search(r"dropped=([\d.]+)s", last)
    problems = [l for l in sim_out + plc_out if PROBLEM.search(l)]
    return {"layout": layout, "ratio": float(ratio.group(1)) if ratio else None,
            "sim_s": float(sim_t.group(1)) if sim_t else None, "dropped_s": float(dropped.group(1)) if dropped else None,
            "stalls": [l for l in sim_out if "[STALL]" in l], "problems": problems,
            "exit": (sim.returncode, plc.returncode), "sim_tail": sim_out[-8:], "plc_tail": plc_out[-8:],
            "timing": timing}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("layouts", nargs="*")
    ap.add_argument("--all", action="store_true", help="every demo layout")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--speed", default="1", help="the simulator's --speed (1, 4, max)")
    ap.add_argument("--verbose", action="store_true", help="print each run's last lines")
    args = ap.parse_args(argv)
    layouts = list(LAYOUTS) if args.all or not args.layouts else args.layouts
    bad = 0
    for layout in layouts:
        r = soak(layout, args.seconds, args.speed)
        flag = "ok" if not r["problems"] else "PROBLEMS"
        bad += bool(r["problems"])
        print(f"{layout:<22} sim {r['sim_s']} s  ratio {r['ratio']}x  dropped {r['dropped_s']} s  "
              f"stalls {len(r['stalls'])}  {flag}", flush=True)
        for line in r["problems"][:12]:
            print(f"    {line}")
        if args.verbose:
            for line in r["timing"][-3:] + r["plc_tail"][-4:]:
                print(f"    | {line}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
