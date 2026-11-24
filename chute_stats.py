"""
chute_stats.py - each chute's (sink's) statistics for the simulator window and
the 3D view: how many loads it took since the last reset, and how many a
minute and an hour.

  total      loads delivered into it since the reset
  per_min    loads in the last minute of simulated time (over the time since
             the reset while that is shorter)
  per_hour   loads in the last hour of simulated time - over the time since
             the reset while that is shorter: projected to an hour

Fed every frame with the simulation's time and its `delivered` counts (per
sink, physics.PhysicsEngine.delivered); a simulation reset (its time or a
count going back) starts it again by itself. The rates are in simulated time,
so they hold at any simulation speed.
"""
from collections import deque
from typing import Dict, Iterable, List, Optional

MINUTE_S = 60.0
HOUR_S = 3600.0


class ChuteStats:
    def __init__(self):
        self.reset_at: Optional[float] = None
        self.last: Dict[str, int] = {}
        self.totals: Dict[str, int] = {}
        self.events: Dict[str, deque] = {}          # id -> (sim time, how many)
        self.now = 0.0

    def reset(self, sim_time: float, delivered: Optional[Dict[str, int]] = None) -> None:
        """Start counting again (the Reset Stats button): the chutes themselves are not touched."""
        self.reset_at = float(sim_time)
        if delivered is not None:
            self.last = dict(delivered)
        self.totals = {}
        self.events = {}

    def update(self, sim_time: float, delivered: Dict[str, int]) -> None:
        sim_time = float(sim_time)
        if self.reset_at is None or sim_time < self.now - 1e-9 or \
                any(n < self.last.get(k, 0) for k, n in delivered.items()):
            self.reset(sim_time, delivered)          # the start, or the simulation was reset
        for sink_id, n in delivered.items():
            new = int(n) - int(self.last.get(sink_id, 0))
            if new > 0:
                self.totals[sink_id] = self.totals.get(sink_id, 0) + new
                self.events.setdefault(sink_id, deque()).append((sim_time, new))
        self.last = dict(delivered)
        self.now = sim_time
        for queue in self.events.values():
            while queue and queue[0][0] < sim_time - HOUR_S:
                queue.popleft()

    def row(self, sink_id: str) -> dict:
        elapsed = max(self.now - (self.reset_at if self.reset_at is not None else self.now), 0.0)
        queue = self.events.get(sink_id, ())

        def rate(window: float) -> float:
            span = min(elapsed, window)
            if span < 1.0:
                return 0.0
            count = sum(n for t, n in queue if t >= self.now - span)
            return count * window / span

        return {"id": sink_id, "total": self.totals.get(sink_id, 0),
                "perMin": round(rate(MINUTE_S), 1), "perHour": round(rate(HOUR_S), 0),
                "since": round(elapsed, 1)}

    def rows(self, sink_ids: Iterable[str]) -> List[dict]:
        return [self.row(i) for i in sink_ids]


def text(row: dict) -> str:
    """One chute's numbers on one line: "123 | 12.0/min | 720/h"."""
    return f"{row['total']} | {row['perMin']:.1f}/min | {row['perHour']:.0f}/h"
