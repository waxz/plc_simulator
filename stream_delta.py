"""
stream_delta.py - a live state stream sent as what changed (the 3D view's
/events, view3d.py; any state of the same shape).

A state is a dict. Each of its top-level values is one of:

  a dict    {name: value}         only the entries that changed are sent,
                                  and the names that went
  rows      [[id, field1, ...]]   a list of rows, each keyed by its first
                                  element: a new row whole; a known row as
                                  only the fields that changed; the ids
                                  that went
  anything else                    sent again when it changed

A delta is {"Δ": {...}} next to the keys that are always sent ("t", "w",
"layout", ...: ALWAYS). The first frame to each client is a whole state
(a key frame): the stream is per connection and ordered (TCP), so a delta
always applies to the state the client has. Nothing here knows about
parcels, belts or the 3D view: a new key or kind of row is carried as it
is.

    enc = DeltaEncoder()
    msg = enc.encode(state)          # the whole state first, then deltas
    state = apply(state, msg)        # the client's side (view3d_web/delta.js does the same)
"""
from typing import Any, Dict, Optional

ALWAYS = ("t", "w", "layout")     # sent in every frame, whole


def _is_rows(value: Any) -> bool:
    return isinstance(value, list) and bool(value) and all(
        isinstance(r, list) and r and isinstance(r[0], (str, int)) for r in value)


def _rows_delta(prev: list, cur: list) -> Optional[dict]:
    before = {r[0]: r for r in prev}
    out: Dict[str, list] = {}
    seen = set()
    for row in cur:
        key = row[0]
        seen.add(key)
        old = before.get(key)
        if old is None or len(old) != len(row):
            out.setdefault("n", []).append(row)                       # new (or reshaped): whole
        elif old != row:
            change = [key]
            for i in range(1, len(row)):
                if row[i] != old[i]:
                    change += [i, row[i]]
            out.setdefault("c", []).append(change)                    # [id, index, value, index, value, ...]
    gone = [k for k in before if k not in seen]
    if gone:
        out["x"] = gone
    order = [r[0] for r in cur]
    if [r[0] for r in prev if r[0] in seen] + [k for k in order if k not in before] != order:
        out["o"] = order                                              # the rows' order changed
    return out or None


def delta(prev: Dict[str, Any], cur: Dict[str, Any]) -> Dict[str, Any]:
    """What changed from `prev` to `cur`, per top-level key: {"s": {key: whole value},
    "d": {key: {"u": {name: value}, "x": [names]}}, "r": {key: rows delta}, "x": [keys gone]}."""
    out: Dict[str, Any] = {}
    for key, value in cur.items():
        if key in ALWAYS:
            continue
        old = prev.get(key, None)
        if key in prev and old == value:
            continue
        if isinstance(value, dict) and isinstance(old, dict):
            upd = {k: v for k, v in value.items() if k not in old or old[k] != v}
            gone = [k for k in old if k not in value]
            entry = {}
            if upd:
                entry["u"] = upd
            if gone:
                entry["x"] = gone
            if entry:
                out.setdefault("d", {})[key] = entry
        elif _is_rows(value) and isinstance(old, list) and (not old or _is_rows(old)):
            rows = _rows_delta(old, value)
            if rows:
                out.setdefault("r", {})[key] = rows
        else:
            out.setdefault("s", {})[key] = value
    gone = [k for k in prev if k not in cur and k not in ALWAYS]
    if gone:
        out["x"] = gone
    return out


class DeltaEncoder:
    """One per client: its first frame whole (a key frame), then what changed since the last."""

    def __init__(self, keyframe_every: int = 0):
        self.last: Optional[Dict[str, Any]] = None
        self.count = 0
        self.keyframe_every = keyframe_every       # 0: only the first

    def encode(self, state: Dict[str, Any]) -> Dict[str, Any]:
        self.count += 1
        key = self.last is None or (self.keyframe_every and self.count % self.keyframe_every == 0)
        if key or state.get("layout") != self.last.get("layout"):
            msg = dict(state)
        else:
            msg = {k: state[k] for k in ALWAYS if k in state}
            msg["Δ"] = delta(self.last, state)
        self.last = state
        return msg


def apply(state: Optional[Dict[str, Any]], msg: Dict[str, Any]) -> Dict[str, Any]:
    """The client's side: the whole state after this message."""
    if "Δ" not in msg:
        return dict(msg)
    out = dict(state or {})
    for k in ALWAYS:
        if k in msg:
            out[k] = msg[k]
    d = msg["Δ"]
    out.update(d.get("s", {}))
    for key, entry in d.get("d", {}).items():
        value = dict(out.get(key) or {})
        value.update(entry.get("u", {}))
        for name in entry.get("x", []):
            value.pop(name, None)
        out[key] = value
    for key, rows in d.get("r", {}).items():
        by_id = {r[0]: list(r) for r in (out.get(key) or [])}
        for row in rows.get("n", []):
            by_id[row[0]] = list(row)
        for change in rows.get("c", []):
            row = by_id[change[0]]
            for i in range(1, len(change), 2):
                row[change[i]] = change[i + 1]
        for gone in rows.get("x", []):
            by_id.pop(gone, None)
        order = rows.get("o") or [r[0] for r in (out.get(key) or []) if r[0] in by_id] + \
            [k for k in by_id if k not in {r[0] for r in (out.get(key) or [])}]
        out[key] = [by_id[k] for k in order if k in by_id]
    for key in d.get("x", []):
        out.pop(key, None)
    return out
