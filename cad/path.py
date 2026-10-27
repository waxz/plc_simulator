"""
cad/path.py - geometry of drawn ("path") belts. No Qt.

A path belt's centerline is a list of anchor points; the segment from each
anchor to the next is a straight line (bulge 0) or an exact circular arc
(bulge = tan(sweep / 4), the DXF polyline convention: positive = turns left
/ CCW, 1 = semicircle). A closed path also joins the last anchor back to
the first. The belt surface is that centerline offset by half the belt
width on each side (Shapely buffer), so a closed loop is an annulus.

Everything here works on plain (x, y) tuples and bulge lists in the belt's
local frame; belt_world_path() maps a Belt's local anchors to the world.

Building blocks used by the drawing tool:
  tangent_bulge()    - arc leaving a point along a heading, ending at a click
  fillet_corners()   - round every sharp corner with a radius
  snap_angle()       - polar tracking (Shift)
  parse_point_input()- typed "x,y", "@dx,dy", "len<angle", "@len<angle", "len"
  preset()           - triangle / rectangle / polygon / circle loops, U, L, S
"""
import math
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from shapely.geometry import LinearRing, LineString, Polygon

Point = Tuple[float, float]

# Chord error used when flattening arcs (metres). 1 mm keeps sensor/footprint
# geometry far below any physical tolerance.
FLATTEN_TOLERANCE = 0.001


@dataclass
class Segment:
    kind: str                    # "line" | "arc"
    p0: Point
    p1: Point
    center: Optional[Point] = None
    radius: float = 0.0
    start_angle: float = 0.0     # radians, at p0
    sweep: float = 0.0           # radians, + CCW / - CW

    @property
    def length(self) -> float:
        if self.kind == "arc":
            return abs(self.sweep) * self.radius
        return math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])

    def heading_at(self, t: float) -> float:
        """Direction of travel (radians) at fraction t in [0, 1]."""
        if self.kind == "arc":
            a = self.start_angle + self.sweep * t
            return a + (math.pi / 2 if self.sweep > 0 else -math.pi / 2)
        return math.atan2(self.p1[1] - self.p0[1], self.p1[0] - self.p0[0])

    def point_at(self, t: float) -> Point:
        if self.kind == "arc":
            a = self.start_angle + self.sweep * t
            return (self.center[0] + self.radius * math.cos(a), self.center[1] + self.radius * math.sin(a))
        return (self.p0[0] + (self.p1[0] - self.p0[0]) * t, self.p0[1] + (self.p1[1] - self.p0[1]) * t)

    @property
    def curvature(self) -> float:
        """Signed 1/radius (+ = turning left); 0 for lines."""
        if self.kind == "arc" and self.radius > 0:
            return math.copysign(1.0 / self.radius, self.sweep)
        return 0.0


# ---------------------------------------------------------------------------
# Bulge arithmetic
# ---------------------------------------------------------------------------

def bulge_to_segment(p0: Point, p1: Point, bulge: float) -> Segment:
    chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if abs(bulge) < 1e-12 or chord < 1e-12:
        return Segment("line", p0, p1)
    sweep = 4.0 * math.atan(bulge)
    radius = chord / (2.0 * abs(math.sin(sweep / 2.0)))
    # Center: from the chord midpoint, perpendicular (left for CCW) by the sagitta offset.
    mx, my = (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0
    ux, uy = (p1[0] - p0[0]) / chord, (p1[1] - p0[1]) / chord
    offset = radius * math.cos(sweep / 2.0) * math.copysign(1.0, sweep)
    center = (mx - uy * offset, my + ux * offset)
    start_angle = math.atan2(p0[1] - center[1], p0[0] - center[0])
    return Segment("arc", p0, p1, center, radius, start_angle, sweep)


def tangent_bulge(p0: Point, heading: float, p1: Point) -> float:
    """Bulge of the arc that leaves p0 along `heading` (radians) and ends at
    p1. The turn is twice the angle between the heading and the chord."""
    chord_angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    delta = math.atan2(math.sin(chord_angle - heading), math.cos(chord_angle - heading))
    sweep = 2.0 * delta
    if abs(sweep) >= 2 * math.pi - 1e-6:
        return 0.0
    return math.tan(sweep / 4.0)


def segments(points: Sequence[Point], bulges: Sequence[float], closed: bool) -> List[Segment]:
    n = len(points)
    count = n if closed else n - 1
    return [bulge_to_segment(points[i], points[(i + 1) % n], bulges[i] if i < len(bulges) else 0.0)
            for i in range(max(count, 0))]


def path_length(segs: Sequence[Segment]) -> float:
    return sum(s.length for s in segs)


def flatten(segs: Sequence[Segment], tolerance: float = FLATTEN_TOLERANCE, with_index: bool = False):
    """Polyline through the centerline plus, per polyline piece, the signed
    curvature of the segment it came from (and, with `with_index`, that
    segment's index)."""
    pts: List[Point] = []
    curv: List[float] = []
    index: List[int] = []
    for k, seg in enumerate(segs):
        if not pts:
            pts.append(seg.p0)
        if seg.kind == "arc":
            # Chord error e = r(1 - cos(step/2))  ->  step = 2 acos(1 - e/r)
            step = 2.0 * math.acos(max(-1.0, 1.0 - tolerance / seg.radius)) if seg.radius > tolerance else math.pi / 8
            n = max(2, int(math.ceil(abs(seg.sweep) / max(step, 1e-6))))
            for i in range(1, n + 1):
                pts.append(seg.point_at(i / n))
                curv.append(seg.curvature)
                index.append(k)
        else:
            pts.append(seg.p1)
            curv.append(0.0)
            index.append(k)
    return (pts, curv, index) if with_index else (pts, curv)


def centerline(points: Sequence[Point], bulges: Sequence[float], closed: bool,
               tolerance: float = FLATTEN_TOLERANCE):
    pts, _ = flatten(segments(points, bulges, closed), tolerance)
    if closed and len(pts) >= 4:
        return LinearRing(pts[:-1] if pts[0] == pts[-1] else pts)
    return LineString(pts)


def footprint(points: Sequence[Point], bulges: Sequence[float], closed: bool, width: float,
              tolerance: float = FLATTEN_TOLERANCE) -> Polygon:
    """The belt surface: centerline offset by width/2 on each side. Straight
    corners are mitred (a real conveyor's corner), ends are cut square."""
    if len(points) < 2 or width <= 0:
        return Polygon()
    line = centerline(points, bulges, closed, tolerance)
    shape = line.buffer(width / 2.0, cap_style="flat", join_style="mitre", mitre_limit=3.0)
    if shape.geom_type == "MultiPolygon":
        shape = max(shape.geoms, key=lambda g: g.area)
    return shape


def start_heading(segs: Sequence[Segment]) -> float:
    return segs[0].heading_at(0.0)


def end_heading(segs: Sequence[Segment]) -> float:
    return segs[-1].heading_at(1.0)


# ---------------------------------------------------------------------------
# Corner fillets
# ---------------------------------------------------------------------------

def fillet_corners(points: Sequence[Point], bulges: Sequence[float], closed: bool,
                   radius: float) -> Tuple[List[Point], List[float]]:
    """Replace every sharp corner between two straight segments by a
    tangent arc of `radius` (shrunk where a segment is too short to fit it).
    Corners next to an arc are left as they are."""
    pts, bul = list(points), list(bulges) + [0.0] * (len(points) - len(bulges))
    n = len(pts)
    if radius <= 0 or n < 3:
        return pts, bul
    corners = range(n) if closed else range(1, n - 1)
    out_pts: List[Point] = []
    out_bul: List[float] = []
    plan = {}
    for i in corners:
        prev_i, next_i = (i - 1) % n, (i + 1) % n
        if abs(bul[prev_i]) > 1e-12 or abs(bul[i]) > 1e-12:
            continue
        a, b, c = pts[prev_i], pts[i], pts[next_i]
        h_in = math.atan2(b[1] - a[1], b[0] - a[0])
        h_out = math.atan2(c[1] - b[1], c[0] - b[0])
        turn = math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in))
        if abs(turn) < 1e-6 or abs(turn) > math.pi - 1e-6:
            continue
        len_in = math.hypot(b[0] - a[0], b[1] - a[1])
        len_out = math.hypot(c[0] - b[0], c[1] - b[1])
        # A segment may lose at most half its length to each of its two corners.
        tangent = min(radius * math.tan(abs(turn) / 2.0), len_in / 2.0, len_out / 2.0)
        if tangent < 1e-9:
            continue
        t_in = (b[0] - tangent * math.cos(h_in), b[1] - tangent * math.sin(h_in))
        t_out = (b[0] + tangent * math.cos(h_out), b[1] + tangent * math.sin(h_out))
        plan[i] = (t_in, t_out, math.tan(turn / 4.0))
    for i in range(n):
        if i in plan:
            t_in, t_out, bulge = plan[i]
            out_pts += [t_in, t_out]
            out_bul += [bulge, bul[i]]
        else:
            out_pts.append(pts[i])
            out_bul.append(bul[i])
    return out_pts, out_bul


# ---------------------------------------------------------------------------
# Drawing-tool helpers
# ---------------------------------------------------------------------------

def snap_angle(origin: Point, p: Point, step_deg: float = 15.0) -> Point:
    """Keep the distance from `origin`, round the direction to `step_deg`."""
    dx, dy = p[0] - origin[0], p[1] - origin[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-12:
        return p
    step = math.radians(step_deg)
    angle = round(math.atan2(dy, dx) / step) * step
    return origin[0] + dist * math.cos(angle), origin[1] + dist * math.sin(angle)


_NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def parse_point_input(text: str, last: Optional[Point], cursor_heading: Optional[float] = None) -> Optional[Point]:
    """Typed coordinates, CAD style (metres, degrees CCW from +X):
        x,y         absolute
        @dx,dy      relative to the last anchor
        len<angle   (or @len<angle) polar, relative to the last anchor
        len         along the current cursor direction from the last anchor
    Returns None if the text isn't one of these (or needs a last anchor)."""
    t = text.strip().replace(" ", "")
    m = re.fullmatch(rf"({_NUM}),({_NUM})", t)
    if m:
        return float(m.group(1)), float(m.group(2))
    m = re.fullmatch(rf"@({_NUM}),({_NUM})", t)
    if m and last is not None:
        return last[0] + float(m.group(1)), last[1] + float(m.group(2))
    m = re.fullmatch(rf"@?({_NUM})<({_NUM})", t)
    if m and last is not None:
        d, a = float(m.group(1)), math.radians(float(m.group(2)))
        return last[0] + d * math.cos(a), last[1] + d * math.sin(a)
    m = re.fullmatch(_NUM, t)
    if m and last is not None and cursor_heading is not None:
        d = float(t)
        return last[0] + d * math.cos(cursor_heading), last[1] + d * math.sin(cursor_heading)
    return None


PRESETS = {
    # name: (label, closed, parameters it uses)
    "triangle": ("Triangle loop", True, ("size",)),
    "rectangle": ("Rectangle loop", True, ("size", "size_b")),
    "polygon": ("Regular polygon loop", True, ("size", "sides")),
    "circle": ("Circle loop", True, ("size",)),
    "u_turn": ("U-turn", False, ("size", "size_b")),
    "l_bend": ("L-bend (90 deg)", False, ("size", "size_b")),
    "s_bend": ("S-bend", False, ("size", "size_b")),
}


def preset(kind: str, size: float, size_b: float = 0.0, sides: int = 6,
           corner_radius: float = 0.0) -> Tuple[List[Point], List[float], bool]:
    """Anchors (centered on the origin), bulges and `closed` for a shape.
      triangle   equilateral, side `size`
      rectangle  `size` x `size_b`
      polygon    regular, `sides` sides, circumradius `size`
      circle     diameter `size` (two semicircle arcs)
      u_turn     legs `size` long, `size_b` apart (semicircle between)
      l_bend     legs `size` and `size_b`, 90 deg left turn
      s_bend     lateral offset `size_b` over forward length `size`
    Corners of straight-sided shapes are rounded by `corner_radius`."""
    if kind == "triangle":
        r = size / math.sqrt(3.0)
        pts = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a))) for a in (-90, 30, 150)]
        bul, closed = [0.0] * 3, True
    elif kind == "rectangle":
        w, h = size / 2.0, (size_b or size) / 2.0
        pts, bul, closed = [(-w, -h), (w, -h), (w, h), (-w, h)], [0.0] * 4, True
    elif kind == "polygon":
        n = max(3, int(sides))
        pts = [(size * math.cos(2 * math.pi * i / n - math.pi / 2), size * math.sin(2 * math.pi * i / n - math.pi / 2))
               for i in range(n)]
        bul, closed = [0.0] * n, True
    elif kind == "circle":
        r = size / 2.0
        return [(0.0, -r), (0.0, r)], [1.0, 1.0], True
    elif kind == "u_turn":
        gap = size_b or size / 2.0
        pts = [(-size / 2.0, -gap / 2.0), (size / 2.0, -gap / 2.0), (size / 2.0, gap / 2.0), (-size / 2.0, gap / 2.0)]
        return pts, [0.0, 1.0, 0.0, 0.0], False
    elif kind == "l_bend":
        b = size_b or size
        pts, bul, closed = [(-size / 2.0, -b / 2.0), (size / 2.0, -b / 2.0), (size / 2.0, b / 2.0)], [0.0] * 3, False
    elif kind == "s_bend":
        # Two opposite tangent arcs joined by their inflection point.
        off = size_b or size / 2.0
        half = (size / 2.0, off / 2.0)
        sweep = 2.0 * math.atan2(half[1], half[0])  # arc from 0 heading to the inflection
        pts = [(-size / 2.0, -off / 2.0), (0.0, 0.0), (size / 2.0, off / 2.0)]
        return pts, [math.tan(sweep / 4.0), -math.tan(sweep / 4.0), 0.0], False
    else:
        raise ValueError(f"unknown shape preset '{kind}'")
    if corner_radius > 0:
        pts, bul = fillet_corners(pts, bul, closed, corner_radius)
    return pts, bul, closed


def recenter(points: Sequence[Point]) -> Tuple[Point, List[Point]]:
    """(center, points relative to it): center = middle of the anchors' bounding box."""
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    return (cx, cy), [(x - cx, y - cy) for x, y in points]


# ---------------------------------------------------------------------------
# Belt <-> path
# ---------------------------------------------------------------------------

def belt_local_path(belt) -> Tuple[List[Point], List[float]]:
    nodes = belt.path or []
    return [(n.x, n.y) for n in nodes], [n.bulge for n in nodes]


def belt_world_path(belt) -> Tuple[List[Point], List[float]]:
    """Anchors in world coordinates (rotation + offset applied; a rotation
    keeps every bulge unchanged)."""
    pts, bul = belt_local_path(belt)
    c, s = math.cos(math.radians(belt.rotation)), math.sin(math.radians(belt.rotation))
    return [(belt.x + x * c - y * s, belt.y + x * s + y * c) for x, y in pts], bul


def path_flow_sign(belt) -> float:
    """+1 if parcels travel in anchor order, -1 if reversed (negative speed,
    or directionAngle pointing backwards)."""
    sign = -1.0 if (belt.speed or 0.0) < 0 else 1.0
    if math.cos(math.radians(belt.directionAngle)) < 0:
        sign = -sign
    return sign
