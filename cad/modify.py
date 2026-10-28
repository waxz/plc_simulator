"""
cad/modify.py - CAD-style modify operations on sketch geometry.

A shape is (points, bulges, closed), the polyline-with-arcs form of
SketchEntity / DrawingShape (bulge = tan(sweep / 4), 0 = straight). Arcs stay
exact through every operation.

  trim(shape, pick, cutters)        remove the piece between the nearest
                                    crossings on either side of the pick
  extend(shape, pick, boundaries)   lengthen the end nearest the pick to the
                                    first boundary it meets (lines straight
                                    on, arcs around their circle)
  break_at(shape, point)            split in two at the nearest point
  offset(shape, distance, side)     parallel copy on the side of a point
  fillet(a, pick_a, b, pick_b, r)   join two lines' ends with a tangent arc
  explode(shape)                    one shape per segment
  move / rotate / mirror / scale    transforms (mirror flips every arc)

Positions along a shape are parameters s = segment index + t (t in 0..1),
so they sort in drawing order. Functions raise ModifyError with a reason a
user can act on.
"""
import math
from typing import Iterable, List, Optional, Sequence, Tuple

from cad.path import Segment, bulge_to_segment, fillet_corners, reverse_polyline, segments

Point = Tuple[float, float]
Shape = Tuple[List[Point], List[float], bool]
EPS = 1e-9
TOL = 1e-7          # metres / parameter: crossings this close are the same


class ModifyError(ValueError):
    pass


def as_shape(entity) -> Shape:
    """(points, bulges, closed) from a SketchEntity / DrawingShape / tuple."""
    if isinstance(entity, tuple):
        pts, bul, closed = entity
    else:
        pts, bul, closed = entity.points, entity.bulges, entity.closed
    pts = [(float(x), float(y)) for x, y in pts]
    bul = [float(b) for b in bul] + [0.0] * (len(pts) - len(bul))
    return pts, bul[:len(pts)], bool(closed)


def _segs(shape: Shape) -> List[Segment]:
    pts, bul, closed = shape
    return segments(pts, bul, closed)


# ---------------------------------------------------------------------------
# Points on segments
# ---------------------------------------------------------------------------

def _arc_param(seg: Segment, x: float, y: float) -> float:
    """t along an arc segment of the point at angle atan2(y - cy, x - cx)
    (may be > 1: beyond its end, going the same way round)."""
    a = math.atan2(y - seg.center[1], x - seg.center[0])
    d = (a - seg.start_angle) if seg.sweep > 0 else (seg.start_angle - a)
    d %= 2 * math.pi
    if d > 2 * math.pi - 1e-9:
        d = 0.0
    return d / abs(seg.sweep)


def _nearest_t(seg: Segment, p: Point) -> Tuple[float, float]:
    """(t, distance) of the point on the segment nearest p."""
    if seg.kind == "line":
        (x0, y0), (x1, y1) = seg.p0, seg.p1
        dx, dy = x1 - x0, y1 - y0
        n = dx * dx + dy * dy
        t = 0.0 if n < EPS else max(0.0, min(1.0, ((p[0] - x0) * dx + (p[1] - y0) * dy) / n))
    else:
        t = _arc_param(seg, *p)
        if t > 1.0:                                    # outside the arc: the nearer end
            span = 2 * math.pi / abs(seg.sweep)
            t = 1.0 if (t - 1.0) < (span - t) else 0.0
    q = seg.point_at(t)
    return t, math.hypot(q[0] - p[0], q[1] - p[1])


def nearest_param(shape: Shape, p: Point) -> Tuple[float, float]:
    """(s, distance) of the point on the shape nearest p."""
    best = (0.0, math.inf)
    for i, seg in enumerate(_segs(shape)):
        t, d = _nearest_t(seg, p)
        if d < best[1]:
            best = (i + t, d)
    return best


def point_at(shape: Shape, s: float) -> Point:
    segs = _segs(shape)
    i = min(int(s), len(segs) - 1)
    return segs[i].point_at(s - i)


# ---------------------------------------------------------------------------
# Crossings
# ---------------------------------------------------------------------------

def _line_line(a: Segment, b: Segment, a_ray: bool = False):
    (x1, y1), (x2, y2) = a.p0, a.p1
    (x3, y3), (x4, y4) = b.p0, b.p1
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < EPS:
        return []
    ta = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    tb = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den
    ok_a = ta >= -TOL if a_ray else -TOL <= ta <= 1 + TOL
    return [(ta, tb)] if ok_a and -TOL <= tb <= 1 + TOL else []


def _circle_points_line(seg: Segment, line: Segment):
    """Points where the line through `line` meets the circle of `seg`, with
    the line's t."""
    (x0, y0), (x1, y1) = line.p0, line.p1
    cx, cy, r = seg.center[0], seg.center[1], seg.radius
    dx, dy = x1 - x0, y1 - y0
    fx, fy = x0 - cx, y0 - cy
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4 * a * c
    if a < EPS or disc < -EPS:
        return []
    disc = math.sqrt(max(0.0, disc))
    ts = {(-b - disc) / (2 * a), (-b + disc) / (2 * a)}
    return [(t, (x0 + dx * t, y0 + dy * t)) for t in ts]


def _circle_circle(a: Segment, b: Segment):
    (x0, y0), r0 = a.center, a.radius
    (x1, y1), r1 = b.center, b.radius
    d = math.hypot(x1 - x0, y1 - y0)
    if d < EPS or d > r0 + r1 + TOL or d < abs(r0 - r1) - TOL:
        return []
    l = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, r0 * r0 - l * l))
    mx, my = x0 + l * (x1 - x0) / d, y0 + l * (y1 - y0) / d
    return [(mx + h * (y1 - y0) / d, my - h * (x1 - x0) / d), (mx - h * (y1 - y0) / d, my + h * (x1 - x0) / d)]


def crossings(a: Segment, b: Segment, a_ray: bool = False, a_full_circle: bool = False) -> List[Tuple[float, float]]:
    """[(ta, tb)] where segment a meets segment b. `a_ray`: a line goes on
    past its end (ta > 1); `a_full_circle`: an arc goes on round its circle
    (ta up to 2 pi / |sweep|)."""
    out = []
    if a.kind == "line" and b.kind == "line":
        return _line_line(a, b, a_ray)
    if a.kind == "line" or b.kind == "line":
        line, arc = (a, b) if a.kind == "line" else (b, a)
        for t_line, p in _circle_points_line(arc, line):
            t_arc = _arc_param(arc, *p)
            if line is a:
                ok_a = t_line >= -TOL if a_ray else -TOL <= t_line <= 1 + TOL
                if ok_a and t_arc <= 1 + TOL:
                    out.append((t_line, t_arc))
            else:
                limit = 2 * math.pi / abs(arc.sweep) if a_full_circle else 1 + TOL
                if -TOL <= t_line <= 1 + TOL and t_arc <= limit:
                    out.append((t_arc, t_line))
        return out
    limit = 2 * math.pi / abs(a.sweep) if a_full_circle else 1 + TOL
    for p in _circle_circle(a, b):
        ta, tb = _arc_param(a, *p), _arc_param(b, *p)
        if ta <= limit and tb <= 1 + TOL:
            out.append((ta, tb))
    return out


def cut_params(shape: Shape, cutters: Iterable[Shape]) -> List[float]:
    """Sorted parameters along `shape` where any cutter crosses it."""
    found = []
    mine = _segs(shape)
    for cutter in cutters:
        for b in _segs(cutter):
            for i, a in enumerate(mine):
                for ta, _tb in crossings(a, b):
                    found.append(i + min(1.0, max(0.0, ta)))
    found.sort()
    out = []
    for s in found:
        if not out or s - out[-1] > TOL:
            out.append(s)
    return out


# ---------------------------------------------------------------------------
# Pieces of a shape
# ---------------------------------------------------------------------------

def piece(shape: Shape, s0: float, s1: float) -> Shape:
    """The part of a shape from s0 to s1 (s1 may pass the end of a closed
    shape: it wraps round), as an open shape."""
    segs = _segs(shape)
    n = len(segs)
    pts: List[Point] = []
    bul: List[float] = []
    s = s0
    while s1 - s > TOL:
        i = int(math.floor(s + TOL))
        seg = segs[i % n]
        ta = s - i
        tb = min(1.0, s1 - i)
        a, b = seg.point_at(max(0.0, ta)), seg.point_at(tb)
        if not pts:
            pts.append(a)
        pts.append(b)
        bul.append(math.tan(seg.sweep * (tb - ta) / 4.0) if seg.kind == "arc" else 0.0)
        s = i + tb
    bul.append(0.0)
    if len(pts) < 2:
        raise ModifyError("Nothing would be left of it.")
    return pts, bul, False


def trim(shape: Shape, pick: Point, cutters: Iterable[Shape]) -> List[Shape]:
    """Remove the piece of `shape` under `pick` - between the nearest
    crossings with `cutters` on either side (or an open end). Returns what
    is left (0, 1 or 2 shapes)."""
    shape = as_shape(shape)
    n = len(_segs(shape))
    cuts = cut_params(shape, [as_shape(c) for c in cutters])
    s_pick, _d = nearest_param(shape, pick)
    if not shape[2]:
        inner = [c for c in cuts if TOL < c < n - TOL]
        if not inner:
            raise ModifyError("Nothing crosses it here - nothing to trim to (Delete removes it).")
        lo = max((c for c in inner if c < s_pick), default=0.0)
        hi = min((c for c in inner if c > s_pick), default=float(n))
        left = []
        if lo > TOL:
            left.append(piece(shape, 0.0, lo))
        if hi < n - TOL:
            left.append(piece(shape, hi, float(n)))
        return left
    if len(cuts) < 2:
        raise ModifyError("A closed shape needs two crossings to trim between.")
    lo = max((c for c in cuts if c < s_pick), default=cuts[-1] - n)
    hi = min((c for c in cuts if c > s_pick), default=cuts[0] + n)
    return [piece(shape, hi, lo + n if lo + n > hi else lo + 2 * n)]


def break_at(shape: Shape, point: Point) -> List[Shape]:
    """Split at the point nearest `point`: two shapes (open), or one open
    shape starting and ending there (closed)."""
    shape = as_shape(shape)
    n = len(_segs(shape))
    s, _d = nearest_param(shape, point)
    if shape[2]:
        return [piece(shape, s, s + n)]
    if s < TOL or s > n - TOL:
        raise ModifyError("That is an end already - pick a point along it.")
    return [piece(shape, 0.0, s), piece(shape, s, float(n))]


def extend(shape: Shape, pick: Point, boundaries: Iterable[Shape]) -> Shape:
    """Lengthen the end of an open shape nearest `pick` to the first
    boundary it reaches: a line goes straight on, an arc on round its
    circle."""
    shape = as_shape(shape)
    pts, bul, closed = shape
    if closed:
        raise ModifyError("A closed shape has no end to extend.")
    n = len(_segs(shape))
    s_pick, _d = nearest_param(shape, pick)
    at_start = s_pick < n / 2.0
    if at_start:
        pts, bul = reverse_polyline(pts, bul)
    end = segments(pts, bul, False)[-1]
    best = None
    for boundary in boundaries:
        for b in _segs(as_shape(boundary)):
            for ta, _tb in crossings(end, b, a_ray=end.kind == "line", a_full_circle=end.kind == "arc"):
                if ta > 1 + TOL and (best is None or ta < best):
                    best = ta
    if best is None:
        raise ModifyError("Nothing to extend to in that direction.")
    new_end = end.point_at(best)
    pts = list(pts[:-1]) + [new_end]
    bul = list(bul)
    if end.kind == "arc":
        bul[-2] = math.tan(end.sweep * best / 4.0)
    if at_start:
        pts, bul = reverse_polyline(pts, bul)
    return list(pts), list(bul), False


# ---------------------------------------------------------------------------
# Offset, fillet, explode
# ---------------------------------------------------------------------------

def _left_normal(seg: Segment, t: float) -> Tuple[float, float]:
    h = seg.heading_at(t)
    return -math.sin(h), math.cos(h)


def offset(shape: Shape, distance: float, side: Point) -> Shape:
    """A parallel copy `distance` away, on the side of `side`. Straight
    neighbours meet at a mitred corner, an arc becomes a concentric arc;
    where offset pieces no longer touch they are joined by a straight line."""
    if distance <= 0:
        raise ModifyError("The offset distance must be more than 0.")
    shape = as_shape(shape)
    segs = _segs(shape)
    s, _d = nearest_param(shape, side)
    i = min(int(s), len(segs) - 1)
    q = segs[i].point_at(s - i)
    nx, ny = _left_normal(segs[i], s - i)
    sign = 1.0 if (side[0] - q[0]) * nx + (side[1] - q[1]) * ny >= 0 else -1.0
    d = sign * distance
    moved: List[Segment] = []
    for seg in segs:
        if seg.kind == "line":
            nx, ny = _left_normal(seg, 0.0)
            moved.append(Segment("line", (seg.p0[0] + nx * d, seg.p0[1] + ny * d),
                                 (seg.p1[0] + nx * d, seg.p1[1] + ny * d)))
        else:
            r = seg.radius - d if seg.sweep > 0 else seg.radius + d      # left of a CCW arc is its inside
            if r <= TOL:
                raise ModifyError(f"An arc of radius {seg.radius:.3f} m can't be offset {distance:.3f} m inwards.")
            c = seg.center
            a0, a1 = seg.start_angle, seg.start_angle + seg.sweep
            moved.append(Segment("arc", (c[0] + r * math.cos(a0), c[1] + r * math.sin(a0)),
                                 (c[0] + r * math.cos(a1), c[1] + r * math.sin(a1)), c, r, a0, seg.sweep))
    closed = shape[2]
    # Mitre straight neighbours (the infinite lines' crossing).
    joints = range(len(moved)) if closed else range(len(moved) - 1)
    for j in joints:
        a, b = moved[j], moved[(j + 1) % len(moved)]
        if a.kind == "line" and b.kind == "line":
            (x1, y1), (x2, y2), (x3, y3), (x4, y4) = a.p0, a.p1, b.p0, b.p1
            den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(den) > EPS:
                t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
                p = (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
                moved[j] = Segment("line", a.p0, p)
                moved[(j + 1) % len(moved)] = Segment("line", p, b.p1)
    pts: List[Point] = [moved[0].p0]
    bul: List[float] = []
    for j, seg in enumerate(moved):
        if math.hypot(seg.p0[0] - pts[-1][0], seg.p0[1] - pts[-1][1]) > TOL:
            bul.append(0.0)                    # a gap: join with a straight line
            pts.append(seg.p0)
        bul.append(math.tan(seg.sweep / 4.0) if seg.kind == "arc" else 0.0)
        pts.append(seg.p1)
    if closed:
        if math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) <= TOL:
            pts.pop()
        else:
            bul.append(0.0)
        return pts, bul[:len(pts)], True
    bul.append(0.0)
    return pts, bul, False


def explode(shape: Shape) -> List[Shape]:
    shape = as_shape(shape)
    return [([seg.p0, seg.p1], [math.tan(seg.sweep / 4.0) if seg.kind == "arc" else 0.0, 0.0], False)
            for seg in _segs(shape)]


def fillet(a: Shape, pick_a: Point, b: Shape, pick_b: Point, radius: float) -> Shape:
    """Join two open shapes at the crossing of the straight end segments
    nearest the picks, with a tangent arc of `radius` (0 = a sharp corner):
    one shape. Each keeps the side of the crossing its pick is on."""
    a, b = as_shape(a), as_shape(b)
    if a[2] or b[2]:
        raise ModifyError("Fillet joins the ends of two open lines.")
    ends = []
    for shape, pick in ((a, pick_a), (b, pick_b)):
        segs = _segs(shape)
        s, _d = nearest_param(shape, pick)
        i = min(int(s), len(segs) - 1)
        if segs[i].kind != "line":
            raise ModifyError("Fillet works on straight lines - pick a straight segment.")
        if i not in (0, len(segs) - 1):
            raise ModifyError("Pick a line's end segment (or Explode the polyline first).")
        ends.append((shape, segs[i], i, pick))
    (sa, la, ia, pa), (sb, lb, ib, pb) = ends
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = la.p0, la.p1, lb.p0, lb.p1
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < EPS:
        raise ModifyError("The lines are parallel - they never meet.")
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    corner = (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    def towards_corner(shape, seg, i, pick):
        """The shape's points oriented to end at the corner, keeping the
        part on the pick's side of it."""
        pts, bul, _c = shape
        n = len(pts) - 1
        # which end of the picked segment does the kept part run away from?
        tp, _ = _nearest_t(seg, pick)
        tc = _line_param(seg, corner)
        keep_start_side = tp < tc            # pick lies between p0 and the corner
        if n == 1:
            far = seg.p0 if keep_start_side else seg.p1
            return [far, corner], [0.0, 0.0]
        if i == n - 1 and keep_start_side:                 # last segment, kept towards its start
            return list(pts[:-1]) + [corner], list(bul[:-1]) + [0.0]
        if i == 0 and not keep_start_side:                 # first segment, kept towards its end
            rp, rb = reverse_polyline(pts, bul)
            return list(rp[:-1]) + [corner], list(rb[:-1]) + [0.0]
        raise ModifyError("Pick each line on the side you want to keep, at the end that meets the other.")

    pa_pts, pa_bul = towards_corner(sa, la, ia, pa)
    pb_pts, pb_bul = towards_corner(sb, lb, ib, pb)
    pb_pts, pb_bul = reverse_polyline(pb_pts, pb_bul)       # now starts at the corner
    pts = pa_pts + pb_pts[1:]
    bul = pa_bul[:-1] + [0.0] + pb_bul[1:]
    if radius > 0:
        k = len(pa_pts) - 1                                  # the corner's index
        rounded, rbul = fillet_corners([pts[k - 1], pts[k], pts[k + 1]], [0.0, 0.0, 0.0], False, radius)
        if len(rounded) == 4:
            pts = pts[:k] + rounded[1:3] + pts[k + 1:]
            bul = bul[:k] + [rbul[1], 0.0] + bul[k + 1:]
    return pts, bul, False


def _line_param(seg: Segment, p: Point) -> float:
    (x0, y0), (x1, y1) = seg.p0, seg.p1
    dx, dy = x1 - x0, y1 - y0
    n = dx * dx + dy * dy
    return ((p[0] - x0) * dx + (p[1] - y0) * dy) / n if n > EPS else 0.0


# ---------------------------------------------------------------------------
# Transforms
# ---------------------------------------------------------------------------

def move(shape: Shape, dx: float, dy: float) -> Shape:
    pts, bul, closed = as_shape(shape)
    return [(x + dx, y + dy) for x, y in pts], bul, closed


def rotate(shape: Shape, center: Point, angle_rad: float) -> Shape:
    pts, bul, closed = as_shape(shape)
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    cx, cy = center
    return [(cx + (x - cx) * c - (y - cy) * s, cy + (x - cx) * s + (y - cy) * c) for x, y in pts], bul, closed


def mirror(shape: Shape, a: Point, b: Point) -> Shape:
    """Mirrored about the line through a and b (arcs turn the other way)."""
    pts, bul, closed = as_shape(shape)
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    if n < EPS:
        raise ModifyError("Pick two different points for the mirror line.")
    out = []
    for x, y in pts:
        t = ((x - a[0]) * dx + (y - a[1]) * dy) / n
        fx, fy = a[0] + t * dx, a[1] + t * dy
        out.append((2 * fx - x, 2 * fy - y))
    return out, [-v for v in bul], closed


def scale(shape: Shape, center: Point, factor: float) -> Shape:
    if factor <= 0:
        raise ModifyError("The scale factor must be more than 0.")
    pts, bul, closed = as_shape(shape)
    cx, cy = center
    return [(cx + (x - cx) * factor, cy + (y - cy) * factor) for x, y in pts], bul, closed
