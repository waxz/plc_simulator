"""
sim_study.py - a study: run a layout with its PLC program headless, faster
than real time, over seeds and parameter variants, and measure it. Generic:
any layout, any mock-PLC program (plc_mock.make_program), any parameter.

    python sim_study.py merge_demo.json --seconds 300 --warmup 60 --seeds 1,2,3
    python sim_study.py merge_demo.json --sweep "belts.belt_[2345]*.drive.accel=2,3,4" --jobs 4
    python sim_study.py station_1.json --set "sources.*.interval=1.5" --json out.json --csv runs.csv

What it measures (in the window after --warmup; per run, then over the seeds
of a variant: mean, min and max):

  flow       items delivered per sink and per minute, in total
  wip        items on the plant: mean and max
  lead time  an item's time on the plant, first seen to delivered: mean, p50, p95
  routes     where items went by their destination: {destination: {sink: n}}
             (an item with no destination: "-")
  sources    items each source made; the share of time it was held (its spot
             not clear: the plant could not take more)
  belts      the share of time each belt ran (> 0.01 m/s), and the mean over all
  events     the program's events by kind (the text before ":")
  program    whatever the program counts (its metrics(): the merge train's grants,
             landings, empty cars, each induction's misses, a sorter's turns ...)
  speed      simulated seconds per wall second

Parameters (--set once, --sweep over values; several --sweep: every
combination): LIST.ID.ATTR[.ATTR...]=VALUE - LIST a SimulatorConfig list
(belts, sources, sensors, diverters, ...), ID an id or a glob (*, belt_2*),
ATTR a field, nested ones with dots (drive.accel); "config.ATTR=VALUE" for
the layout itself. Values are JSON (2, 1.5, true, "x") or plain text.
"""
import argparse
import csv
import fnmatch
import itertools
import json
import math
import os
import random
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))


# -- parameters ------------------------------------------------------------------------------------

def parse_value(text: str) -> Any:
    try:
        return json.loads(text)
    except ValueError:
        return text


def parse_setting(text: str) -> Tuple[str, Any]:
    path, sep, value = text.partition("=")
    if not sep or not path.strip():
        raise ValueError(f"{text!r}: expected PATH=VALUE")
    return path.strip(), parse_value(value.strip())


def apply_setting(config, path: str, value: Any) -> int:
    """Set PATH to VALUE in the layout (see the module's doc): the number of fields set."""
    parts = path.split(".")
    if parts[0] == "config":
        targets, attrs = [config], parts[1:]
    else:
        items = getattr(config, parts[0], None)
        if not isinstance(items, list) or len(parts) < 3:
            raise ValueError(f"{path}: expected LIST.ID.ATTR (LIST a layout list: belts, sources, ...)")
        targets = [i for i in items if fnmatch.fnmatchcase(str(getattr(i, "id", "")), parts[1])]
        attrs = parts[2:]
    count = 0
    for target in targets:
        obj = target
        for a in attrs[:-1]:
            obj = getattr(obj, a)
            if obj is None:
                break
        if obj is None or not hasattr(obj, attrs[-1]):
            continue
        setattr(obj, attrs[-1], value)
        count += 1
    if count == 0:
        raise ValueError(f"{path}: nothing in the layout has it")
    return count


def variants(sets: List[str], sweeps: List[str]) -> List[Dict[str, Any]]:
    """Every combination of the sweeps' values, each with the fixed settings."""
    fixed = dict(parse_setting(s) for s in sets)
    axes = []
    for s in sweeps:
        path, _sep, values = s.partition("=")
        axes.append([(path.strip(), parse_value(v.strip())) for v in values.split(",") if v.strip()])
    out = []
    for combo in itertools.product(*axes) if axes else [()]:
        v = dict(fixed)
        v.update(dict(combo))
        out.append(v)
    return out


# -- one run ---------------------------------------------------------------------------------------

def _percentile(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    values = sorted(values)
    return values[min(len(values) - 1, int(round(q * (len(values) - 1))))]


def run(layout: str, settings: Dict[str, Any], seed: int = 1, seconds: float = 300.0, warmup: float = 60.0,
        program: str = "auto", scan: float = 0.05, dt: float = 0.01) -> Dict[str, Any]:
    """One simulation, measured (see the module's doc)."""
    sys.path.insert(0, HERE)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    from config_parser import load_config
    from physics import PhysicsEngine
    import plc_mock
    random.seed(seed)
    config = load_config(layout)
    for path, value in settings.items():
        apply_setting(config, path, value)
    engine = PhysicsEngine(config)
    ctrl = plc_mock.make_program(config, program, autostart=True)
    sinks = {s.id: s for s in config.sinks}

    window = {"on": False}
    first_seen: Dict[str, float] = {}
    lead: List[float] = []
    routes: Dict[str, Dict[str, int]] = {}
    delivered: Dict[str, int] = {}

    def sink_at(x, y) -> str:
        for s in config.sinks:
            if abs(x - s.x) <= s.width / 2 + 0.3 and abs(y - s.y) <= s.height / 2 + 0.3:
                return s.id
        return "removed"

    take = engine.handle_sinks

    def counted_sinks(*a, **k):
        before = [(p, tuple(p.body.position)) for p in engine.parcels]
        result = take(*a, **k)
        alive = {id(p) for p in engine.parcels}
        for p, (x, y) in before:
            if id(p) in alive or not window["on"]:
                continue
            sink = sink_at(x, y)
            delivered[sink] = delivered.get(sink, 0) + 1
            dest = getattr(p, "destination", None)
            key = "-" if dest in (None, 0) else str(dest)
            routes.setdefault(key, {})
            routes[key][sink] = routes[key].get(sink, 0) + 1
            if p.id in first_seen:
                lead.append(engine.sim_time - first_seen[p.id])
        return result

    engine.handle_sinks = counted_sinks
    steps, per_scan = int(seconds / dt), max(1, int(round(scan / dt)))
    sample_every = max(1, int(round(0.5 / dt)))
    wip: List[int] = []
    belt_on = {b.id: 0 for b in config.belts}
    held = {s.id: 0 for s in config.sources}
    samples = 0
    events: Dict[str, int] = {}
    made0: Dict[str, int] = {}
    metrics0: Dict[str, Any] = {}
    t0 = time.perf_counter()
    for i in range(steps):
        engine.update(dt)
        now = engine.sim_time
        if i % per_scan == 0:
            for e in ctrl.step(now) or []:
                if window["on"]:
                    kind = str(e).split(":", 1)[0].strip()[:60]
                    events[kind] = events.get(kind, 0) + 1
        for p in engine.parcels:
            first_seen.setdefault(p.id, now)
        if not window["on"] and now >= warmup:
            window["on"] = True
            made0 = dict(getattr(engine.logistics, "_serial", {}))
            metrics0 = ctrl.metrics() if hasattr(ctrl, "metrics") else {}
        if window["on"] and i % sample_every == 0:
            samples += 1
            wip.append(len(engine.parcels))
            for b in config.belts:
                if abs(float(b.actualSpeed or 0.0)) > 0.01:
                    belt_on[b.id] += 1
            for s in config.sources:
                if engine.source_timers.get(s.id, 0.0) >= s.interval:
                    held[s.id] += 1
    wall = time.perf_counter() - t0
    minutes = max(seconds - warmup, 1e-9) / 60.0
    made = getattr(engine.logistics, "_serial", {})
    program_metrics: Dict[str, Any] = {}
    if hasattr(ctrl, "metrics"):
        for k, v in ctrl.metrics().items():
            base = metrics0.get(k)
            program_metrics[k] = v - base if isinstance(v, (int, float)) and isinstance(base, (int, float)) else v
    total = sum(n for s, n in delivered.items() if s in sinks)
    out = {
        "seed": seed, "settings": settings, "seconds": seconds, "warmup": warmup,
        "flow": {"per_min": total / minutes, "total": total,
                 "sinks": {s: {"n": n, "per_min": n / minutes} for s, n in sorted(delivered.items())}},
        "wip": {"mean": statistics.fmean(wip) if wip else 0.0, "max": max(wip, default=0)},
        "lead_s": {"mean": statistics.fmean(lead) if lead else None, "p50": _percentile(lead, 0.5),
                   "p95": _percentile(lead, 0.95), "n": len(lead)},
        "routes": routes,
        "sources": {s.id: {"made": made.get(s.id, 0) - made0.get(s.id, 0),
                           "held": held[s.id] / samples if samples else 0.0} for s in config.sources},
        "belts": {"running_mean": statistics.fmean(v / samples for v in belt_on.values()) if samples and belt_on
                  else 0.0, "running": {k: v / samples for k, v in belt_on.items()} if samples else {}},
        "events": dict(sorted(events.items(), key=lambda kv: -kv[1])),
        "program": program_metrics,
        "headline": [f"program.{k}" for k in getattr(ctrl, "HEADLINE", ())],   # its own default columns
        "speed": seconds / wall if wall > 0 else math.inf,
    }
    return out


def _run_job(job):
    return run(**job)


# -- many runs -------------------------------------------------------------------------------------

def flatten(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        else:
            out[key] = v
    return out


def aggregate(runs: List[Dict[str, Any]]) -> Dict[str, Dict[str, float]]:
    """Each numeric field over the runs (the seeds of a variant): mean, min, max."""
    flat = [flatten({k: v for k, v in r.items() if k not in ("settings", "seed", "headline")}) for r in runs]
    keys = sorted({k for f in flat for k, v in f.items() if isinstance(v, (int, float)) and not isinstance(v, bool)})
    out = {}
    for k in keys:
        values = [f[k] for f in flat if isinstance(f.get(k), (int, float)) and f[k] is not None]
        if values:
            out[k] = {"mean": statistics.fmean(values), "min": min(values), "max": max(values)}
    return out


def study(layout: str, settings_list: List[Dict[str, Any]], seeds: List[int], jobs: int = 1,
          **run_args) -> List[Dict[str, Any]]:
    """Every variant x seed; per variant its runs and their aggregate."""
    work = [dict(layout=layout, settings=s, seed=seed, **run_args) for s in settings_list for seed in seeds]
    if jobs > 1 and len(work) > 1:
        with ProcessPoolExecutor(max_workers=jobs) as pool:
            results = list(pool.map(_run_job, work))
    else:
        results = [_run_job(w) for w in work]
    out = []
    for i, s in enumerate(settings_list):
        runs = results[i * len(seeds):(i + 1) * len(seeds)]
        out.append({"settings": s, "runs": runs, "aggregate": aggregate(runs)})
    return out


def _fmt(x, digits=1) -> str:
    if x is None:
        return "-"
    return f"{x:.{digits}f}" if isinstance(x, float) else str(x)


def print_table(results: List[Dict[str, Any]], keys: List[str]) -> None:
    head = ["variant", "flow/min", "wip", "lead p95 s"] + keys + ["sim/real"]
    rows = []
    for r in results:
        a = r["aggregate"]

        def m(k, digits=1):
            v = a.get(k)
            if v is None:
                return "-"
            return _fmt(v["mean"], digits) if v["min"] == v["max"] else \
                f"{_fmt(v['mean'], digits)} ({_fmt(v['min'], digits)}-{_fmt(v['max'], digits)})"

        name = ", ".join(f"{k}={v}" for k, v in r["settings"].items()) or "as laid out"
        rows.append([name, m("flow.per_min"), m("wip.mean"), m("lead_s.p95", 1)] +
                    [m(k, 1) for k in keys] + [m("speed", 1)])
    widths = [max(len(str(x)) for x in col) for col in zip(head, *rows)]
    for line in [head] + rows:
        print("  ".join(str(x).ljust(w) for x, w in zip(line, widths)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("layout")
    ap.add_argument("--seconds", type=float, default=300.0, help="simulated seconds per run (default 300)")
    ap.add_argument("--warmup", type=float, default=60.0, help="not measured: the first seconds (default 60)")
    ap.add_argument("--seeds", default="1", help="comma-separated seeds (default 1)")
    ap.add_argument("--program", default="auto", help="the mock PLC's program (plc_mock --program; default auto)")
    ap.add_argument("--scan", type=float, default=0.05, help="PLC scan, s (default 0.05)")
    ap.add_argument("--dt", type=float, default=0.01, help="physics step, s (default 0.01)")
    ap.add_argument("--set", action="append", default=[], metavar="PATH=VALUE", help="a fixed parameter")
    ap.add_argument("--sweep", action="append", default=[], metavar="PATH=V1,V2", help="a parameter over values")
    ap.add_argument("--metric", action="append", default=[], metavar="KEY",
                    help="a column to show (a flattened key, e.g. program.train.empty_cars, "
                         "sources.src_1.held); default: the program's headline (its HEADLINE)")
    ap.add_argument("--jobs", type=int, default=1, help="runs in parallel (processes)")
    ap.add_argument("--json", help="write every run and the aggregates here")
    ap.add_argument("--csv", help="write one row per run here (flattened)")
    args = ap.parse_args(argv)
    layout = args.layout if os.path.isabs(args.layout) else os.path.abspath(args.layout)
    seeds = [int(s) for s in args.seeds.split(",") if s.strip()]
    results = study(layout, variants(args.set, args.sweep), seeds, jobs=args.jobs, seconds=args.seconds,
                    warmup=args.warmup, program=args.program, scan=args.scan, dt=args.dt)
    keys = args.metric or list(results[0]["runs"][0].get("headline", []))
    print_table(results, keys)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=1)
    if args.csv:
        rows = [flatten({k: v for k, v in run.items() if k != "settings"} | {"settings": run["settings"]})
                for r in results for run in r["runs"]]
        cols = sorted({k for row in rows for k in row})
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
