"""
sim_query.py - ask a running simulator what it is doing (its query API,
view3d.py /api): for debugging by hand or by an agent. Read-only; the
standard library only.

    python sim_query.py state                       # time, health, the PLC link, faults, chutes, watchdogs, lamps
    python sim_query.py tags panel infeed_run       # PLC tags now (names or substrings; none: every tag)
    python sim_query.py tags --area Q               # the PLC's outputs
    python sim_query.py history chute_2_full infeed_run --seconds 30   # over time (sim time)
    python sim_query.py components chute_2 div_1    # their PLC signals now, and their control
    python sim_query.py parcels                     # every parcel: where, how fast, barcode, destination
    python sim_query.py tracking                    # the PLC's tracked loads against the parcels (mm)
    python sim_query.py state --json                # the raw JSON

The simulator serves it on 127.0.0.1 (port 8765, or the next free one if
another simulator has it: this finds it). --port picks one.
"""
import argparse
import json
import sys
import urllib.error
import urllib.parse
import urllib.request

DEFAULT_PORT = 8765
SEARCH = 12                 # ports tried from DEFAULT_PORT


def _get(port: int, path: str, timeout: float = 3.0):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=timeout) as res:
        return json.loads(res.read())


def find_port(port=None) -> int:
    for p in ([port] if port else range(DEFAULT_PORT, DEFAULT_PORT + SEARCH)):
        try:
            if _get(p, "/api", timeout=1.0).get("simulator"):
                return p
        except (OSError, ValueError, urllib.error.URLError):
            continue
    raise SystemExit("no simulator answers on 127.0.0.1 (is it running? python main.py <layout>)")


def query(path: str, port=None, **params):
    params = {k: v for k, v in params.items() if v not in (None, "", [])}
    qs = ("?" + urllib.parse.urlencode(params)) if params else ""
    return _get(find_port(port), path + qs)


def _print_state(s: dict) -> None:
    if "error" in s:
        print(s["error"])
        return
    t = s.get("timing") or {}
    print(f"layout     {s.get('layout')}")
    print(f"sim time   {s.get('simTime')} s   {'RUNNING' if s.get('playing') else 'PAUSED'}   speed "
          f"{s.get('speed') or 'max'}   sim/real {t.get('ratio', 0):.2f}   health {s.get('health')}")
    if t:
        print(f"timing     ticks {t.get('tick_hz', 0):.0f}/s, tick max {t.get('tick_ms_max', 0):.1f} ms, stalls "
              f"{t.get('stall_count', 0)} (longest {t.get('longest_stall_s', 0):.2f} s), dropped {t.get('dropped_s', 0)} s")
    link = s.get("plcLink") or {}
    if link:
        print(f"PLC link   clients {link.get('clients')}, frames in {link.get('rx_frames')}, out {link.get('sent')}, "
              f"last frame {link.get('rx_age_ms')} ms ago, errors {link.get('rx_errors')}, size errors "
              f"{link.get('rx_size_errors')}")
    for w in s.get("watchdogs", []):
        print(f"watchdog   {w['id']}: {'TRIPPED' if w['tripped'] else ('PLC alive' if w['plcAlive'] else 'not armed')}"
              f" ({w['trips']} trips), heartbeats PLC {w['plcHeartbeat']} sim {w['simHeartbeat']}, latency "
              f"{w.get('latencyMs', 0.0):.0f} ms (max {w.get('latencyMaxMs', 0.0):.0f}, simulated)")
    for c in s.get("chutes", []):
        print(f"chute      {c['id']}: {c['count']}/{c['capacity']} {'FULL' if c['full'] else 'not full'} "
              f"({c['fillMode']})")
    print(f"lamps      " + ", ".join(f"{k}={v}" for k, v in (s.get("lamps") or {}).items()))
    print(f"belts      {len(s.get('beltsRunning', []))} running; faults: {', '.join(s.get('beltFaults') or []) or 'none'}")
    if s.get("manual"):
        print(f"manual     {', '.join(s['manual'])} (the PLC's commands ignored)")
    print(f"parcels    {s.get('parcels')} on the plant; delivered {s.get('delivered')}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("what", choices=("state", "tags", "history", "components", "parcels", "tracking"))
    ap.add_argument("match", nargs="*", help="names or substrings (tags, history, components)")
    ap.add_argument("--area", choices=("I", "Q"), help="tags: inputs (I) or outputs (Q) only")
    ap.add_argument("--seconds", type=float, default=30.0, help="history: how far back (sim time)")
    ap.add_argument("--port", type=int)
    ap.add_argument("--json", action="store_true", help="print the raw JSON")
    args = ap.parse_args(argv)
    match = ",".join(args.match)
    if args.what == "state":
        data = query("/api/state", args.port)
    elif args.what == "tags":
        data = query("/api/tags", args.port, match=match, area=args.area)
    elif args.what == "history":
        data = query("/api/history", args.port, match=match, area=args.area, seconds=args.seconds)
    elif args.what == "components":
        data = query("/api/components", args.port, match=match)
    elif args.what == "tracking":
        data = query("/api/tracking", args.port)
    else:
        data = query("/api/parcels", args.port)
    if args.json:
        print(json.dumps(data, indent=2))
        return 0
    if "error" in data:
        print(data["error"])
        return 1
    if args.what == "state":
        _print_state(data)
    elif args.what == "tags":
        print(f"sim time {data.get('simTime')} s")
        for t in data["tags"]:
            print(f"{t['name']:<28} {t['address']:<16} {t['type']:<10} {t['value']:<18} {t['comment']}")
    elif args.what == "history":
        names = data["names"]
        print("sim_t     " + " ".join(f"{n[:14]:>14}" for n in names))
        prev = None
        for row in data["samples"]:
            if row[1:] != prev:                  # only the changes
                print(f"{row[0]:<9.2f} " + " ".join(f"{v:>14g}" for v in row[1:]))
                prev = row[1:]
    elif args.what == "components":
        for cid, info in data["components"].items():
            values = ", ".join(f"{k}={v}" for k, v in info.get("values", {}).items())
            print(f"{cid:<20} [{info.get('control')}] {values}")
    elif args.what == "tracking":
        print(f"{data['samples']} checks of {data['loads']} loads: along max {data['maxAlongMm']} mm "
              f"(rms {data['rmsAlongMm']}), across max {data['maxLateralMm']} mm, length max "
              f"{data['maxLengthErrorMm']} mm, width max {data['maxWidthErrorMm']} mm; unmatched "
              f"{data['unmatched']}, rematched {data['rematched']}, stale {data['stale']}")
        for r in data.get("recentLoads", []):
            print(f"  load {r['loadId']:<6} parcel {r.get('parcel')!s:<10} along max {r.get('maxAlong')} "
                  f"across max {r.get('maxLateral')}  length {r.get('length')}  width {r.get('width')}")
    else:
        for p in data["parcels"]:
            print(f"{p['id']:<12} {p['kind']:<7} x {p['x']:8.3f} y {p['y']:8.3f}  {p['speed']:5.2f} m/s  "
                  f"barcode {p['barcode']}  dest {p['destination']}")
        print(f"{len(data['parcels'])} parcels")
    return 0


if __name__ == "__main__":
    sys.exit(main())
