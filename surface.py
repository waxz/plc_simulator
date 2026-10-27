"""
surface.py - belt surface geometry and surface velocity, per belt type.

Shared by both traction models (physics.py's kinematic path and
traction.py's friction path), so "where is a belt" and "how fast is its
surface moving at this point" have one answer. A new belt type (merge,
roller zone, ...) adds a branch here and nothing else needs to know.
"""
import math
from typing import Optional, Tuple

from config_parser import Belt
from math_utils import get_belt_polygon, point_in_arc, point_in_polygon


def belt_surface_speed(belt: Belt) -> float:
    """Signed surface speed physics should use: the drive's actual speed when
    the belt has a drive model, else the commanded speed (ideal drive)."""
    if belt.drive is not None:
        return belt.actualSpeed
    return belt.speed


def _geometry_signature(belt: Belt) -> tuple:
    verts = tuple((p["x"], p["y"]) for p in belt.vertexs) if belt.vertexs else None
    return (belt.type, belt.shape, belt.x, belt.y, belt.length, belt.beltWidth, belt.rotation,
            belt.radius, belt.startAngle, belt.endAngle, verts)


def _geometry(belt: Belt) -> Tuple[Tuple[float, float, float, float], Optional[list]]:
    """(bounds, polygon-or-None), cached on the belt and rebuilt only when its
    geometry fields change (the editors move/resize belts at runtime).
    Recomputing the rotated polygon per point query dominated step time."""
    sig = _geometry_signature(belt)
    cached = belt._geom
    if cached is not None and cached[0] == sig:
        return cached[1], cached[2]
    if belt.type == "linear":
        polygon = get_belt_polygon(belt)
        xs = [v[0] for v in polygon]
        ys = [v[1] for v in polygon]
        bounds = (min(xs), min(ys), max(xs), max(ys))
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
