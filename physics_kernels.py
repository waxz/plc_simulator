"""
physics_kernels.py - the per-parcel geometry of a physics step as compiled
loops over all parcels at once (numba), instead of a Python call per parcel
per belt / sensor.

  belts_under_all   which belts hold each parcel's center (point in a belt
                    polygon / arc band, layout order; path belts are only
                    flagged - their exact test stays with shapely)
  banner_beams      a laser banner's beams blocked by nearby parcels
  boxes_overlap     an IR sensor's box against nearby parcels (SAT)

Each kernel repeats the arithmetic of the Python code it replaces
(math_utils.point_in_polygon / point_in_arc, PhysicsEngine._banner_beams /
_polygons_overlap) operation for operation, so a simulation gives the same
numbers with or without it (test_physics checks; a 40 s run of
test_config.json is bit-identical). Trigonometry of parcel angles stays in
Python's math (callers pass cos / sin), as numba's may differ in the last
bit.

numba compiles on first use (cached in __pycache__ afterwards). Without
numba - or with CONVEYOR_SIM_KERNELS=0 - AVAILABLE is False and physics.py
runs its Python code instead.
"""
import math
import os

import numpy as np

try:
    from numba import njit
    AVAILABLE = os.environ.get("CONVEYOR_SIM_KERNELS", "1") != "0"
except ImportError:                                    # pragma: no cover - numba is a dependency
    AVAILABLE = False

    def njit(*_args, **_kwargs):
        return lambda f: f

KIND_POLYGON, KIND_ARC, KIND_PATH = 0, 1, 2
MAX_VERTS = 8          # linear belts: rectangle or quadrilateral (4); room to spare
MAX_HITS = 8           # belts under one point (overlaps at transfers: 2-3)


# ---------------------------------------------------------------------------
# Belts under each parcel
# ---------------------------------------------------------------------------

class BeltArrays:
    """The belts' shapes as arrays for belts_under_all, built from the same
    geometry surface.py uses (rebuild when a belt moves)."""

    def __init__(self, belts):
        from math_utils import get_belt_polygon
        from surface import belt_bounds
        n = len(belts)
        self.kind = np.zeros(n, np.int64)
        self.bbox = np.zeros((n, 4))
        self.polys = np.zeros((n, MAX_VERTS, 2))
        self.nverts = np.zeros(n, np.int64)
        self.arcs = np.zeros((n, 6))          # cx, cy, radius, width, startAngle, endAngle
        for k, belt in enumerate(belts):
            self.bbox[k] = belt_bounds(belt)
            if belt.type == "linear":
                poly = get_belt_polygon(belt)
                if len(poly) > MAX_VERTS:
                    self.kind[k] = KIND_PATH          # unusual polygon: exact test in Python
                    continue
                self.kind[k] = KIND_POLYGON
                self.nverts[k] = len(poly)
                self.polys[k, :len(poly)] = poly
            elif belt.type == "curved":
                self.kind[k] = KIND_ARC
                self.arcs[k] = (belt.x, belt.y, belt.radius, belt.beltWidth, belt.startAngle, belt.endAngle)
            else:
                self.kind[k] = KIND_PATH


@njit(cache=True)
def _point_in_polygon(x, y, poly, n):
    # math_utils.point_in_polygon, operation for operation.
    inside = False
    p1x = poly[0, 0]
    p1y = poly[0, 1]
    xinters = 0.0
    for i in range(1, n + 1):
        p2x = poly[i % n, 0]
        p2y = poly[i % n, 1]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x = p2x
        p1y = p2y
    return inside


@njit(cache=True)
def _point_in_arc(x, y, cx, cy, radius, thickness, start_angle, end_angle):
    # math_utils.point_in_arc, operation for operation.
    dx = x - cx
    dy = y - cy
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < (radius - thickness / 2.0) or dist > (radius + thickness / 2.0):
        return False
    angle = math.degrees(math.atan2(dy, dx))
    if angle < 0:
        angle += 360.0
    if start_angle <= end_angle:
        return start_angle <= angle <= end_angle
    return angle >= start_angle or angle <= end_angle


@njit(cache=True)
def _belts_under_kernel(xy, kind, bbox, polys, nverts, arcs, hits, counts):
    n_belts = kind.shape[0]
    for i in range(xy.shape[0]):
        x = xy[i, 0]
        y = xy[i, 1]
        c = 0
        for k in range(n_belts):
            if not (bbox[k, 0] <= x <= bbox[k, 2] and bbox[k, 1] <= y <= bbox[k, 3]):
                continue
            if kind[k] == 0:
                ok = _point_in_polygon(x, y, polys[k], nverts[k])
            elif kind[k] == 1:
                ok = _point_in_arc(x, y, arcs[k, 0], arcs[k, 1], arcs[k, 2], arcs[k, 3], arcs[k, 4], arcs[k, 5])
            else:
                ok = True                      # path belt: a candidate, checked exactly by the caller
            if ok and c < hits.shape[1]:
                hits[i, c] = k
                c += 1
        counts[i] = c


def belts_under_all(xy: np.ndarray, arrays: BeltArrays):
    """(hits, counts): for parcel i, belt indices hits[i, :counts[i]] in
    layout order (path belts included as candidates)."""
    n = xy.shape[0]
    hits = np.empty((n, MAX_HITS), np.int64)
    counts = np.empty(n, np.int64)
    if n:
        _belts_under_kernel(xy, arrays.kind, arrays.bbox, arrays.polys, arrays.nverts, arrays.arcs, hits, counts)
    return hits, counts


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------

@njit(cache=True)
def banner_beams(sx, sy, cos_a, sin_a, half_w, laser_count, beam_dist, verts, out):
    """PhysicsEngine._banner_beams for the parcels in `verts` (n x 4 x 2
    world corners), operation for operation; out[i] = beam i blocked."""
    eps = 1e-9
    mid = (laser_count - 1) / 2.0
    for i in range(laser_count):
        out[i] = False
    lu = np.empty(4)
    lv = np.empty(4)
    for p in range(verts.shape[0]):
        vmin = math.inf
        vmax = -math.inf
        for j in range(4):
            dx = verts[p, j, 0] - sx
            dy = verts[p, j, 1] - sy
            lu[j] = dx * cos_a + dy * sin_a
            lv[j] = -dx * sin_a + dy * cos_a
            vmin = min(vmin, lv[j])
            vmax = max(vmax, lv[j])
        first = max(0, math.ceil((vmin - eps) / beam_dist + mid))
        last = min(laser_count - 1, math.floor((vmax + eps) / beam_dist + mid))
        for i in range(first, last + 1):
            if out[i]:
                continue
            b = (i - mid) * beam_dist
            lo = math.inf
            hi = -math.inf
            for j in range(4):
                u0 = lu[j]
                v0 = lv[j]
                u1 = lu[(j + 1) % 4]
                v1 = lv[(j + 1) % 4]
                if min(v0, v1) - eps > b or max(v0, v1) + eps < b:
                    continue
                if abs(v1 - v0) <= eps:
                    lo = min(lo, min(u0, u1))
                    hi = max(hi, max(u0, u1))
                else:
                    u = u0 + (b - v0) / (v1 - v0) * (u1 - u0)
                    lo = min(lo, u)
                    hi = max(hi, u)
            if lo <= hi and hi >= -half_w - eps and lo <= half_w + eps:
                out[i] = True


@njit(cache=True)
def _separated(a, b, axis_x, axis_y):
    min_a = math.inf
    max_a = -math.inf
    for j in range(a.shape[0]):
        d = a[j, 0] * axis_x + a[j, 1] * axis_y
        min_a = min(min_a, d)
        max_a = max(max_a, d)
    min_b = math.inf
    max_b = -math.inf
    for j in range(b.shape[0]):
        d = b[j, 0] * axis_x + b[j, 1] * axis_y
        min_b = min(min_b, d)
        max_b = max(max_b, d)
    return max_a < min_b or max_b < min_a


@njit(cache=True)
def boxes_overlap(box, verts):
    """PhysicsEngine._polygons_overlap(box, parcel) for any parcel in `verts`
    (n x 4 x 2), operation for operation."""
    for p in range(verts.shape[0]):
        b = verts[p]
        hit = True
        for which in range(2):
            poly = box if which == 0 else b
            m = poly.shape[0]
            for j in range(m):
                ex = poly[(j + 1) % m, 0] - poly[j, 0]
                ey = poly[(j + 1) % m, 1] - poly[j, 1]
                length = math.hypot(ex, ey)
                if length < 1e-9:
                    continue
                if _separated(box, b, -ey / length, ex / length):
                    hit = False
                    break
            if not hit:
                break
        if hit:
            return True
    return False
