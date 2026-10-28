"""
surface.py - belt surface geometry and surface velocity, per belt type.

Shared by both traction models (physics.py's kinematic path and
traction.py's friction path), so "where is a belt" and "how fast is its
surface moving at this point" have one answer. A new belt type (merge,
roller zone, ...) adds a branch here and nothing else needs to know.
"""
import bisect
import math
import threading
from contextlib import contextmanager
from typing import Optional, Sequence, Tuple

import shapely
from shapely.geometry import Point as ShapelyPoint

from cad.path import belt_world_path, flatten, footprint, segments
import devices
from config_parser import Belt
from math_utils import get_belt_polygon, point_in_arc, point_in_polygon


def belt_surface_speed(belt: Belt) -> float:
    """Signed surface speed physics should use: the drive's actual speed when
    the belt has a drive model, else the commanded speed (ideal drive) -
    devices.commanded_speed: the signed speed, or the vfd run/reverse/speed
    command; 0 while faulted."""
    if belt.drive is not None:
        return belt.actualSpeed
    return devices.commanded_speed(belt)


def _geometry_signature(belt: Belt) -> tuple:
    verts = tuple((p["x"], p["y"]) for p in belt.vertexs) if belt.vertexs else None
    path = tuple((n.x, n.y, n.bulge) for n in belt.path) if belt.path else None
    return (belt.type, belt.shape, belt.x, belt.y, belt.length, belt.beltWidth, belt.rotation,
            belt.radius, belt.startAngle, belt.endAngle, verts, path, belt.closed, belt.cornerRadius)


class _PathGeometry:
    """World-space data a path belt needs per point query: the prepared
    footprint (fast containment), the flattened centerline with cumulative
    distances (to find the nearest centerline piece), and each piece's
    direction and curvature."""

    def __init__(self, belt: Belt):
        pts, bulges = belt_world_path(belt)
        segs = segments(pts, bulges, belt.closed)
        self.polygon = footprint(pts, bulges, belt.closed, belt.beltWidth)
        shapely.prepare(self.polygon)
        flat, self.curvature, self.segment_index = flatten(segs, with_index=True)
        self.segments = segs
        self.line = shapely.LineString(flat)
        self.cumulative = [0.0]
        self.directions = []
        for (x0, y0), (x1, y1) in zip(flat, flat[1:]):
            d = math.hypot(x1 - x0, y1 - y0)
            self.cumulative.append(self.cumulative[-1] + d)
            self.directions.append(((x1 - x0) / d, (y1 - y0) / d) if d > 0 else (1.0, 0.0))

    def velocity(self, px: float, py: float, v: float):
        """Surface velocity at (px, py) for centerline speed v. Straight
        pieces move uniformly along their direction; arc pieces turn
        rigidly about the arc center (outer edge faster), exactly like a
        curved belt - so parcels keep their orientation through a curve."""
        dist = self.line.project(ShapelyPoint(px, py))
        i = min(max(bisect.bisect_right(self.cumulative, dist) - 1, 0), len(self.directions) - 1)
        seg = self.segments[self.segment_index[i]]
        if seg.kind == "arc":
            w = v * self.curvature[i]
            cx, cy = seg.center
            return -w * (py - cy), w * (px - cx), w
        dx, dy = self.directions[i]
        return v * dx, v * dy, 0.0


_frame = threading.local()   # .geom: {id(belt): (bounds, polygon)} during one physics step


@contextmanager
def geometry_frame(belts: Sequence[Belt]):
    """Within one physics step belts don't move: look each belt's geometry
    up once here, and every point query in the step (thousands with a
    crowded line - each parcel against each belt, several times) reuses it
    instead of rebuilding the cache key from the belt's fields. Per thread:
    the GUI thread keeps checking the belts as they are."""
    previous = getattr(_frame, "geom", None)
    _frame.geom = {id(b): _geometry(b) for b in belts}
    try:
        yield
    finally:
        _frame.geom = previous


def _geometry(belt: Belt) -> Tuple[Tuple[float, float, float, float], Optional[list]]:
    """(bounds, polygon-or-None), cached on the belt and rebuilt only when its
    geometry fields change (the editors move/resize belts at runtime).
    Recomputing the rotated polygon per point query dominated step time."""
    frame = getattr(_frame, "geom", None)
    if frame is not None:
        hit = frame.get(id(belt))
        if hit is not None:
            return hit
    sig = _geometry_signature(belt)
    cached = belt._geom
    if cached is not None and cached[0] == sig:
        return cached[1], cached[2]
    if belt.type == "linear":
        polygon = get_belt_polygon(belt)
        xs = [v[0] for v in polygon]
        ys = [v[1] for v in polygon]
        bounds = (min(xs), min(ys), max(xs), max(ys))
    elif belt.type == "path":
        polygon = _PathGeometry(belt)
        bounds = polygon.polygon.bounds if not polygon.polygon.is_empty else (belt.x, belt.y, belt.x, belt.y)
    else:
        polygon = None
        outer_r = belt.radius + belt.beltWidth / 2.0
        bounds = (belt.x - outer_r, belt.y - outer_r, belt.x + outer_r, belt.y + outer_r)
    belt._geom = (sig, bounds, polygon)
    return bounds, polygon


def belt_bounds(belt: Belt) -> Tuple[float, float, float, float]:
    return _geometry(belt)[0]


def belt_contains(belt: Belt, px: float, py: float) -> bool:
    (min_x, min_y, max_x, max_y), polygon = _geometry(belt)
    if not (min_x <= px <= max_x and min_y <= py <= max_y):
        return False
    if belt.type == "linear":
        return point_in_polygon(px, py, polygon)
    if belt.type == "path":
        return bool(shapely.contains_xy(polygon.polygon, px, py))
    if belt.type == "curved":
        return point_in_arc(px, py, belt.x, belt.y, belt.radius, belt.beltWidth, belt.startAngle, belt.endAngle)
    return False


def curved_direction(belt: Belt) -> int:
    """Curved belts run CCW when directionAngle > 0, else CW (existing rule)."""
    return 1 if getattr(belt, "directionAngle", 0) > 0 else -1


def surface_velocity(belt: Belt, px: float, py: float) -> Optional[Tuple[float, float, float]]:
    """(vx, vy, omega) of the belt surface at world point (px, py). `omega` is
    the surface's rotation rate (0 for linear belts). None at a curved belt's
    exact center, where the direction is undefined."""
    speed = belt_surface_speed(belt)
    if belt.type == "path":
        geom = _geometry(belt)[1]
        if geom.polygon.is_empty:
            return None
        # Signed speed along anchor order (directionAngle ~180 runs it backwards);
        # travelling an arc backwards also flips its turn.
        v = speed if math.cos(math.radians(belt.directionAngle)) >= 0 else -speed
        return geom.velocity(px, py, v)
    if belt.type == "linear":
        rad = math.radians(belt.rotation + belt.directionAngle)
        return speed * math.cos(rad), speed * math.sin(rad), 0.0
    if belt.type == "curved":
        dx = px - belt.x
        dy = py - belt.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return None
        w = curved_direction(belt) * speed / belt.radius if belt.radius > 0 else 0.0
        return -w * dy, w * dx, w
    return None
