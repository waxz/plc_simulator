"""
metrics_store.py - thread-safe, bounded, grouped storage for arbitrary
client-reported metrics (performance counters, cycle times, function
elapsed times, function results, vector values).

Ingestion (metrics_ingest.py) runs on a background network thread and
calls add_sample()/add_samples(); the Qt data-visualization panel reads
from the main/GUI thread via groups()/names()/series()/table_rows(). Every
method takes the same lock, and reads return copies, so a caller never
holds a reference into state that could mutate underneath it.
"""
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, List, Optional, Tuple, Union

MetricValue = Union[float, int, bool, str, List[float]]

DEFAULT_GROUP = "default"
VALID_KINDS = ("scalar", "vector", "text")


def infer_kind(value: Any) -> str:
    """Best-effort kind for a value that arrived with no explicit `kind` -
    a list is a vector, bool/str is text (checked before numbers since
    bool is a subclass of int in Python), everything else is a scalar."""
    if isinstance(value, list):
        return "vector"
    if isinstance(value, (bool, str)):
        return "text"
    return "scalar"


@dataclass(frozen=True)
class MetricSample:
    ts: float
    value: MetricValue


class MetricSeries:
    """One named metric's bounded history. Not thread-safe on its own -
    callers go through MetricsStore's lock."""

    def __init__(self, kind: str, maxlen: int):
        self.kind = kind
        self.samples: Deque[MetricSample] = deque(maxlen=maxlen)

    def append(self, sample: MetricSample) -> None:
        self.samples.append(sample)

    @property
    def latest(self) -> Optional[MetricSample]:
        return self.samples[-1] if self.samples else None


class MetricsStore:
    """Thread-safe, bounded, grouped store of named metric series."""

    def __init__(self, max_samples_per_series: int = 2000):
        self._lock = threading.Lock()
        self._max_samples = max_samples_per_series
        self._groups: Dict[str, Dict[str, MetricSeries]] = {}

    def add_sample(
        self,
        name: str,
        value: MetricValue,
        group: str = DEFAULT_GROUP,
        kind: Optional[str] = None,
        ts: Optional[float] = None,
    ) -> None:
        if not name:
            raise ValueError("metric name must be non-empty")
        group = group or DEFAULT_GROUP
        kind = kind or infer_kind(value)
        if kind not in VALID_KINDS:
            raise ValueError(f"unknown metric kind {kind!r}")
        sample = MetricSample(ts=ts if ts is not None else time.time(), value=value)
        with self._lock:
            series_by_name = self._groups.setdefault(group, {})
            series = series_by_name.get(name)
            if series is None:
                series = MetricSeries(kind=kind, maxlen=self._max_samples)
                series_by_name[name] = series
            elif kind != series.kind:
                # A metric can legitimately change shape over a session
                # (e.g. re-registered with a different value type) - adopt
                # the new kind going forward rather than raising, since
                # rejecting the sample would silently drop real client
                # data over a client-side quirk this store can't control.
                series.kind = kind
            series.append(sample)

    def add_samples(self, samples: List[dict]) -> int:
        """Add a batch of {"group", "name", "value", "kind", "ts"} dicts
        (group/kind/ts optional). Returns how many were accepted; a
        malformed entry in the batch is skipped, not fatal to the rest -
        one bad sample in a client's batch shouldn't discard the good
        ones sent alongside it."""
        accepted = 0
        for entry in samples:
            if not isinstance(entry, dict):
                continue
            try:
                name = entry["name"]
                value = entry["value"]
            except KeyError:
                continue
            try:
                self.add_sample(
                    name=name,
                    value=value,
                    group=entry.get("group", DEFAULT_GROUP),
                    kind=entry.get("kind"),
                    ts=entry.get("ts"),
                )
                accepted += 1
            except (ValueError, TypeError):
                continue
        return accepted

    def groups(self) -> List[str]:
        with self._lock:
            return sorted(self._groups.keys())

    def names(self, group: str) -> List[str]:
        with self._lock:
            return sorted(self._groups.get(group, {}).keys())

    def kind_of(self, group: str, name: str) -> Optional[str]:
        with self._lock:
            series = self._groups.get(group, {}).get(name)
            return series.kind if series else None

    def series(self, group: str, name: str) -> List[MetricSample]:
        """Snapshot copy of one series' current samples, oldest first."""
        with self._lock:
            series = self._groups.get(group, {}).get(name)
            return list(series.samples) if series else []

    def latest(self, group: str, name: str) -> Optional[MetricSample]:
        with self._lock:
            series = self._groups.get(group, {}).get(name)
            return series.latest if series else None

    def table_rows(
        self, group: Optional[str] = None, limit: Optional[int] = None
    ) -> List[Tuple[float, str, str, str, MetricValue]]:
        """Flattened (ts, group, name, kind, value) rows across every
        series (or just `group`), newest first - the CSV-table view's data
        source."""
        rows: List[Tuple[float, str, str, str, MetricValue]] = []
        with self._lock:
            if group is None:
                items = list(self._groups.items())
            elif group in self._groups:
                items = [(group, self._groups[group])]
            else:
                items = []
            for group_name, series_by_name in items:
                for metric_name, series in series_by_name.items():
                    for sample in series.samples:
                        rows.append((sample.ts, group_name, metric_name, series.kind, sample.value))
        rows.sort(key=lambda row: row[0], reverse=True)
        if limit is not None:
            rows = rows[:limit]
        return rows

    def latest_rows(
        self, group: Optional[str] = None
    ) -> List[Tuple[float, str, str, str, MetricValue]]:
        """One row per known metric (or just those in `group`), using each
        metric's most recent sample - the "latest value" table view's data
        source, as opposed to table_rows()'s full scrolling history.
        Sorted by (group, name) rather than newest-first, since this is a
        snapshot dashboard - rows shouldn't reorder every refresh just
        because different metrics happen to update at different rates."""
        rows: List[Tuple[float, str, str, str, MetricValue]] = []
        with self._lock:
            if group is None:
                items = list(self._groups.items())
            elif group in self._groups:
                items = [(group, self._groups[group])]
            else:
                items = []
            for group_name, series_by_name in items:
                for metric_name, series in series_by_name.items():
                    latest = series.latest
                    if latest is not None:
                        rows.append((latest.ts, group_name, metric_name, series.kind, latest.value))
        rows.sort(key=lambda row: (row[1], row[2]))
        return rows

    def clear(self) -> None:
        with self._lock:
            self._groups.clear()
