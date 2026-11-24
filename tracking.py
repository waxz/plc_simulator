"""
tracking.py - checks the loads a PLC tracks against the parcels
(config_parser.TrackedLoadView, the "trackedLoads" device kind).

Every scan the PLC fills its slots with the loads it tracks: id, centre,
heading, length and width, and `frame` - the watchdog's `simHeartbeat` in
the input frame it computed them from. The simulator keeps the parcels'
poses of its last SNAPSHOTS steps by that counter (record), so a slot is
compared with the parcels as they were at the step the PLC saw (check):
the exchange's delay (a frame each way) is no error.

The parcel a load is: the one it was matched to before, if it is still
there; else the nearest within MATCH_RADIUS_M not matched to another load.
A load that drifts is so reported as a growing error, not matched to the
next parcel. The errors, in the load's own frame (its heading):
  along     PLC - parcel along the travel (+: the PLC has it ahead)
  lateral   PLC - parcel across it (+: the PLC has it further left)
  length / width   PLC - the parcel's own length / width (its box: along
                   the travel it was made for, and across), whatever it
                   has turned since
Each slot gets `matched`, `alongError`, `lateralError` back (its feedback);
stats() sums them up for the query API (/api/tracking) and the tests.
"""
import math
from collections import OrderedDict
from typing import Dict, Optional

SNAPSHOTS = 512             # simulation steps of parcel poses kept (2 s at 4 ms)
MATCH_RADIUS_M = 0.5        # a load further than this from every free parcel is unmatched
RECENT_LOADS = 20           # loads listed in report()


class TrackingCheck:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self._snapshots: "OrderedDict[int, list]" = OrderedDict()
        self._matched: Dict[int, str] = {}          # PLC load id -> parcel id
        self._last: Dict[str, tuple] = {}           # slot id -> (loadId, frame) checked last
        self.loads: Dict[int, dict] = {}            # PLC load id -> its worst errors so far
        self.samples = 0
        self.unmatched = 0
        self.stale = 0                              # a frame no longer (or not yet) kept
        self.rematched = 0                          # a load matched to another parcel than before
        self.max_along = 0.0
        self.max_lateral = 0.0
        self._sum_along2 = 0.0

    # -- every step ----------------------------------------------------------------------------

    def record(self, frame: int, parcels) -> None:
        """The parcels' poses at the step whose simHeartbeat is `frame`."""
        snap = []
        for p in parcels:
            if getattr(p, "binned_in", None) is not None or getattr(p, "kind", "parcel") != "parcel":
                continue
            x, y = p.body.position
            snap.append((p, float(x), float(y), float(p.body.angle)))
        self._snapshots[int(frame) & 0xFFFF] = snap
        self._snapshots.move_to_end(int(frame) & 0xFFFF)
        while len(self._snapshots) > SNAPSHOTS:
            self._snapshots.popitem(last=False)

    def check(self, slots) -> None:
        """Each slot the PLC filled since the last check: its errors."""
        active_ids = {int(s.loadId) for s in slots if s.active}
        for s in slots:
            if not s.active:
                s.matched, s.alongError, s.lateralError = False, 0.0, 0.0
                s.lengthError = s.widthError = 0.0
                self._last.pop(s.id, None)
                continue
            key = (int(s.loadId), int(s.frame) & 0xFFFF)
            if self._last.get(s.id) == key:
                continue                            # the same values: already checked
            self._last[s.id] = key
            snap = self._snapshots.get(key[1])
            if snap is None:
                self.stale += 1
                continue
            self._check_slot(s, snap, active_ids)
        for load_id in [i for i in self._matched if i not in active_ids]:
            del self._matched[load_id]              # finished (or no longer shown): free its parcel

    def _check_slot(self, s, snap, active_ids) -> None:
        load_id = int(s.loadId)
        taken = {pid for lid, pid in self._matched.items() if lid != load_id and lid in active_ids}
        found = None
        previous = self._matched.get(load_id)
        if previous is not None:
            found = next((e for e in snap if e[0].id == previous), None)
        if found is None:
            best = MATCH_RADIUS_M
            for e in snap:
                if e[0].id in taken:
                    continue
                d = math.hypot(e[1] - s.loadX, e[2] - s.loadY)
                if d <= best:
                    best, found = d, e
            if found is not None:
                if previous is not None and previous != found[0].id:
                    self.rematched += 1
                self._matched[load_id] = found[0].id
        if found is None:
            s.matched, s.alongError, s.lateralError = False, 0.0, 0.0
            s.lengthError = s.widthError = 0.0
            self.unmatched += 1
            return
        parcel, px, py, angle = found
        yaw = math.radians(s.loadYaw)
        c, sn = math.cos(yaw), math.sin(yaw)
        dx, dy = s.loadX - px, s.loadY - py
        along, lateral = dx * c + dy * sn, -dx * sn + dy * c
        length, width = box_size(parcel)
        s.matched, s.alongError, s.lateralError = True, along, lateral
        e_len, e_wid = s.loadLength - length, s.loadWidth - width
        s.lengthError, s.widthError = e_len, e_wid
        self.samples += 1
        self._sum_along2 += along * along
        self.max_along = max(self.max_along, abs(along))
        self.max_lateral = max(self.max_lateral, abs(lateral))
        rec = self.loads.setdefault(load_id, {"parcel": parcel.id, "barcode": getattr(parcel, "barcode", 0),
                                              "samples": 0, "maxAlong": 0.0, "maxLateral": 0.0})
        rec["parcel"], rec["samples"] = parcel.id, rec["samples"] + 1
        rec["maxAlong"] = max(rec["maxAlong"], abs(along))
        rec["maxLateral"] = max(rec["maxLateral"], abs(lateral))
        rec["length"], rec["width"] = e_len, e_wid          # the latest: the PLC's shape after its last measurement
        rec["along"], rec["lateral"] = along, lateral

    # -- the summary -----------------------------------------------------------------------------

    def stats(self) -> dict:
        n = self.samples
        return {"samples": n, "loads": len(self.loads), "unmatched": self.unmatched, "stale": self.stale,
                "rematched": self.rematched,
                "maxAlongMm": round(self.max_along * 1000.0, 2),
                "rmsAlongMm": round(math.sqrt(self._sum_along2 / n) * 1000.0, 2) if n else 0.0,
                "maxLateralMm": round(self.max_lateral * 1000.0, 2),
                # a load's shape as the PLC has it last (after its eyes measured it), the worst of the loads
                "maxLengthErrorMm": round(max((abs(r["length"]) for r in self.loads.values()), default=0.0) * 1000.0, 2),
                "maxWidthErrorMm": round(max((abs(r["width"]) for r in self.loads.values()), default=0.0) * 1000.0, 2)}

    def load(self, load_id: int) -> Optional[dict]:
        return self.loads.get(int(load_id))

    def report(self, recent: int = RECENT_LOADS) -> dict:
        """stats() and the last `recent` loads checked, by PLC load id (the
        query API's /api/tracking; it crosses to the window's process)."""
        out = self.stats()
        ids = sorted(self.loads)[-recent:]
        out["recentLoads"] = [{"loadId": i, **{k: (round(v * 1000.0, 2) if isinstance(v, float) else v)
                                               for k, v in self.loads[i].items()}} for i in ids]
        out["units"] = "mm (the loads' along / lateral / length / width: PLC - parcel)"
        return out


def report_of(engine) -> Optional[dict]:
    """The tracking report of a PhysicsEngine (its logistics runtime), or of
    the window's mirror of one (sim_process: carried in its snapshot)."""
    mirrored = getattr(engine, "tracking", None)
    if isinstance(mirrored, dict):
        return mirrored
    logistics = getattr(engine, "logistics", None)
    check = getattr(logistics, "tracking", None)
    if check is None or not getattr(getattr(engine, "config", None), "trackedLoads", None):
        return None
    return check.report()


def box_size(parcel):
    """The parcel's own length (along its body's x: the travel it was made
    for) and width: what an eye and a banner measure while it runs square."""
    xs = [v.x for v in parcel.shape.get_vertices()]
    ys = [v.y for v in parcel.shape.get_vertices()]
    return max(xs) - min(xs), max(ys) - min(ys)
