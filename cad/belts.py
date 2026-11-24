"""
cad/belts.py - build Belt models from drawn or picked geometry (world metres).

  outline_belt()  a polygon belt (shape "quadrilateral", any vertex count):
                  the drawn/picked outline IS the belt surface. Parcels move
                  in one direction across it (rotation + directionAngle).
  path_belt()     a belt along a centerline (cad/path.py), offset by width.
  linear_belt() / curved_belt()   native rectangle / arc belts.
  belt_from_target()  Belt from Sketch/Drawing: a picked line chain or closed
                  area -> a belt of the chosen type (auto / linear / curved /
                  path) and direction.

Outline and path belts store their geometry relative to the belt's x/y (the
middle of the anchors' bounding box), like every other belt, so moving and
rotating them in the editor works the same way.
"""
import math
from typing import List, Optional, Sequence, Tuple

from shapely.geometry import Point as ShapelyPoint, Polygon

from cad.path import belt_world_path, fillet_corners, recenter, segments
from config_parser import Belt, BeltConnection, PathNode

Point = Tuple[float, float]


def default_flow_heading(points: Sequence[Point]) -> float:
    """Direction along the outline's long axis (degrees): the long side of
    its minimum rotated rectangle, pointing toward +X (or +Y if vertical)."""
    rect = list(Polygon(points).minimum_rotated_rectangle.exterior.coords)
    edges = [(rect[i], rect[i + 1]) for i in range(2)]
    (a, b) = max(edges, key=lambda e: math.hypot(e[1][0] - e[0][0], e[1][1] - e[0][1]))
    heading = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0
    if 90.0 < heading <= 270.0:
        heading = (heading + 180.0) % 360.0
    return heading


def outline_belt(belt_id: str, points: Sequence[Point], flow_heading: Optional[float] = None,
                 speed: float = 1.0, color: str = "#3b82f6") -> Belt:
    if len(points) < 3:
        raise ValueError("A belt outline needs at least 3 corners.")
    poly = Polygon(points)
    if not poly.is_valid or poly.area <= 1e-9:
        raise ValueError("The outline crosses itself or has no area.")
    if poly.exterior.is_ccw is False:
        points = list(reversed(points))  # consistent winding for the editor's key points
    (cx, cy), local = recenter(points)
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    heading = default_flow_heading(points) if flow_heading is None else flow_heading % 360.0
    return Belt(id=belt_id, type="linear", shape="quadrilateral", x=cx, y=cy, rotation=0.0,
                length=max(xs) - min(xs), beltWidth=max(ys) - min(ys),
                vertexs=[{"x": x, "y": y} for x, y in local], speed=speed, directionAngle=heading, color=color)


# ---------------------------------------------------------------------------
# Cut belts: straight, with either end cut at an angle (a triangle-like end)
# ---------------------------------------------------------------------------
#
# The last belt of a side induction ends along the main line's edge (a merge
# belt: its END is cut), and a branch leaving a line's side starts along that
# edge (its START is cut), so a load moves across the whole edge at once. Such
# a belt is an outline belt (4 corners) kept in a fixed form, so the designer
# shows and edits it by its numbers:
#   length  - its centreline: from the middle of its start edge to the middle
#             of its end edge (Belt.length)
#   width   - Belt.beltWidth
#   start / end angle - each edge's angle to the travel, degrees in (0, 180):
#             90 a square edge; 45 the edge running 45 degrees to the right of
#             the travel (its left side longer at the end, shorter at the start)
# The belt's x, y is the middle of its centreline and its rotation the travel
# (directionAngle 0), so it moves and rotates like any belt. A 4-corner belt of
# that shape drawn or edited by hand is recognised too (cut_of) and can be put
# back into this form (make_cut_belt), with nothing moving.

MITRE_TOLERANCE = 1e-6
_SHAPE_TOLERANCE = 1e-4          # m: corners this close count as on the form
LOOSE_DEG = 1.5                  # a hand-drawn side this close to the travel (cut_of(loose=True))


def _cot(deg: float) -> float:
    return math.cos(math.radians(deg)) / math.sin(math.radians(deg))


def mitre_corners(length: float, width: float, end_deg: float, start_deg: float = 90.0) -> List[dict]:
    """The 4 local corners (CCW): start right, end right, end left, start left."""
    for name, deg in (("end", end_deg), ("start", start_deg)):
        if not 1.0 <= deg <= 179.0:
            raise ValueError(f"The {name} angle must be between 1 and 179 degrees (90: a square edge).")
    half, w2 = length / 2.0, width / 2.0
    ce, cs = _cot(end_deg), _cot(start_deg)
    right, left = length - w2 * ce + w2 * cs, length + w2 * ce - w2 * cs
    if min(right, left) < -MITRE_TOLERANCE:
        need = w2 * abs(ce - cs)
        raise ValueError(f"A {width:g} m wide belt with these cut ends needs at least {need:.3f} m of length "
                         f"(then one side is 0: a triangle).")
    return [{"x": -half - w2 * cs, "y": -w2}, {"x": half - w2 * ce, "y": -w2},
            {"x": half + w2 * ce, "y": w2}, {"x": -half + w2 * cs, "y": w2}]


def mitred_belt(belt_id: str, start: Point, heading: float, length: float, width: float, end_deg: float,
                speed: float = 1.0, color: str = "#3b82f6", start_deg: float = 90.0) -> Belt:
    """A cut belt from `start` (the middle of its start edge), travelling at
    `heading` degrees, its end cut at `end_deg` and its start at `start_deg`."""
    a = math.radians(heading)
    cx, cy = start[0] + math.cos(a) * length / 2.0, start[1] + math.sin(a) * length / 2.0
    return Belt(id=belt_id, type="linear", shape="quadrilateral", x=cx, y=cy, rotation=heading,
                length=length, beltWidth=width, vertexs=mitre_corners(length, width, end_deg, start_deg),
                speed=speed, directionAngle=0.0, color=color)


def cut_of(belt: Belt, loose: bool = False) -> Optional[dict]:
    """A 4-corner outline belt whose two sides run along its travel (parallel,
    one width apart) - a cut belt, however it was drawn or edited - as
    {start, end (x, y: the middles of its start and end edges), heading,
    length, width, start_deg, end_deg, canonical, exact}; None for any other belt.
    `loose`: sides within LOOSE_DEG of the travel count too (a hand-drawn
    belt); make_cut_belt then squares them (`exact` False: corners move)."""
    v = belt.vertexs
    if belt.shape != "quadrilateral" or not v or len(v) != 4:
        return None
    h = belt.rotation + belt.directionAngle
    a = math.radians(h)
    ux, uy = math.cos(a), math.sin(a)
    r = math.radians(belt.rotation)
    world = [(belt.x + p["x"] * math.cos(r) - p["y"] * math.sin(r),
              belt.y + p["x"] * math.sin(r) + p["y"] * math.cos(r)) for p in v]
    uv = [(x * ux + y * uy, -x * uy + y * ux) for x, y in world]
    edges = [(uv[i], uv[(i + 1) % 4]) for i in range(4)]
    def parallel(p, q):
        if abs(p[1] - q[1]) < _SHAPE_TOLERANCE:
            return True
        return loose and math.degrees(math.atan2(abs(q[1] - p[1]), abs(q[0] - p[0]))) < LOOSE_DEG

    along = [k for k, (p, q) in enumerate(edges) if parallel(p, q)]
    exact = all(abs(edges[k][0][1] - edges[k][1][1]) < _SHAPE_TOLERANCE for k in along)
    if len(along) != 2 or abs(along[0] - along[1]) != 2:
        return None
    v_lo, v_hi = sorted((edges[k][0][1] + edges[k][1][1]) / 2.0 for k in along)
    width = v_hi - v_lo
    if width <= _SHAPE_TOLERANCE:
        return None
    vc = (v_lo + v_hi) / 2.0
    ends = []
    for k in [i for i in range(4) if i not in along]:
        (u0, v0), (u1, v1) = edges[k]
        if abs(v1 - v0) < _SHAPE_TOLERANCE:
            return None
        u_mid = u0 + (u1 - u0) * (vc - v0) / (v1 - v0)            # where the centreline crosses this edge
        deg = math.degrees(math.atan2(v1 - v0, u1 - u0)) % 180.0   # the edge's angle to the travel
        ends.append((u_mid, deg))
    (us, ds), (ue, de) = sorted(ends)
    if ue - us <= _SHAPE_TOLERANCE:
        return None

    def point(u):
        return u * ux - vc * uy, u * uy + vc * ux

    canonical = (abs(belt.directionAngle) < MITRE_TOLERANCE and abs(belt.length - (ue - us)) < 1e-6
                 and abs(belt.beltWidth - width) < 1e-6
                 and math.dist((belt.x, belt.y), point((us + ue) / 2.0)) < 1e-6)
    return {"start": point(us), "end": point(ue), "heading": h % 360.0, "length": ue - us, "width": width,
            "start_deg": ds, "end_deg": de, "canonical": canonical and exact, "exact": exact}


def mitre_of(belt: Belt) -> Optional[float]:
    """The end angle (degrees) of a cut belt, None for any other belt."""
    cut = cut_of(belt)
    return None if cut is None else cut["end_deg"]


def make_cut_belt(belt: Belt, loose: bool = False) -> bool:
    """Put a cut belt drawn or edited by hand into the cut-belt form (x, y the
    middle of its centreline, rotation its travel, length and width its own);
    nothing moves - except, with `loose`, a side drawn slightly off the travel
    is squared to it. False when it is not a cut belt."""
    cut = cut_of(belt, loose)
    if cut is None:
        return False
    if not cut["canonical"]:
        h = cut["heading"]
        (sx, sy), (ex, ey) = cut["start"], cut["end"]
        belt.x, belt.y = (sx + ex) / 2.0, (sy + ey) / 2.0
        belt.rotation, belt.directionAngle = (h if h <= 180.0 else h - 360.0), 0.0
        belt.length, belt.beltWidth = cut["length"], cut["width"]
        belt.vertexs = mitre_corners(cut["length"], cut["width"], cut["end_deg"], cut["start_deg"])
    return True


def set_mitre(belt: Belt, length: Optional[float] = None, width: Optional[float] = None,
              end_deg: Optional[float] = None, start_deg: Optional[float] = None) -> None:
    """Change a cut belt's length, width or angles, keeping the middle of its
    start edge where it is (it stays joined to the belt feeding it)."""
    if not make_cut_belt(belt):
        raise ValueError(f"{belt.id} is not a cut belt.")
    cut = cut_of(belt)
    new_len = belt.length if length is None else length
    new_w = belt.beltWidth if width is None else width
    corners = mitre_corners(new_len, new_w, cut["end_deg"] if end_deg is None else end_deg,
                            cut["start_deg"] if start_deg is None else start_deg)     # raises before any change
    a = math.radians(belt.rotation)
    sx, sy = cut["start"]
    belt.length, belt.beltWidth, belt.vertexs = new_len, new_w, corners
    belt.x, belt.y = sx + math.cos(a) * new_len / 2.0, sy + math.sin(a) * new_len / 2.0


def path_belt(belt_id: str, points: Sequence[Point], bulges: Sequence[float], closed: bool, width: float,
              speed: float = 1.0, color: str = "#3b82f6", corner_radius: float = 0.0) -> Belt:
    if len(points) < 2:
        raise ValueError("A belt path needs at least 2 anchors.")
    (cx, cy), local = recenter(points)
    return Belt(id=belt_id, type="path", shape="path", x=cx, y=cy, rotation=0.0, beltWidth=width,
                speed=speed, directionAngle=0.0, color=color, closed=closed, cornerRadius=corner_radius,
                path=[PathNode(x=x, y=y, bulge=b) for (x, y), b in zip(local, bulges)])


# ---------------------------------------------------------------------------
# Belt from a picked target, with a chosen type and direction
# ---------------------------------------------------------------------------
#
# A target is what Belt from Sketch/Drawing picked:
#   ("region", shapely Polygon)               a closed area
#   ("line", (points, bulges, closed))        a chain of joined lines/arcs
#
# kind:
#   "auto"    line: straight -> linear, arcs of one circle -> curved, else path;
#             area: arc-shaped (bounded by concentric arcs) -> curved, else linear
#   "linear"  parcels move one way: a straight line -> rectangle belt; an
#             area -> polygon belt with that outline
#   "curved"  parcels turn about a center: an arc (or circle) -> curved belt
#             along it; an arc-shaped area (ring sector, ring, disc) -> curved
#             belt filling it
#   "path"    along a centerline: any line chain; the middle arc of an
#             arc-shaped area
# direction:
#   "forward"  line: in the picked chain's order; area: linear along its long
#              axis (toward +X/+Y), curved counter-clockwise
#   "reverse"  the opposite
#   "across+" / "across-"  linear only: 90 deg left / right of forward

KINDS = ("auto", "linear", "curved", "path")
DIRECTIONS = ("forward", "reverse", "across+", "across-")
_TURN = {"forward": 0.0, "reverse": 180.0, "across+": 90.0, "across-": -90.0}


def _turn(direction: str, kind: str) -> float:
    if direction not in _TURN:
        raise ValueError(f"Unknown direction {direction!r}.")
    if direction.startswith("across") and kind != "linear":
        raise ValueError("Across only applies to linear belts; use Forward or Reverse.")
    return _TURN[direction]


def linear_belt(belt_id: str, p0: Point, p1: Point, width: float, direction: str = "forward",
                speed: float = 1.0, color: str = "#3b82f6") -> Belt:
    """Rectangle belt with p0 -> p1 as its centerline."""
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if length <= 1e-9:
        raise ValueError("The line has no length.")
    return Belt(id=belt_id, type="linear", shape="rectangle", x=(p0[0] + p1[0]) / 2, y=(p0[1] + p1[1]) / 2,
                length=length, beltWidth=width, rotation=math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])),
                speed=speed, directionAngle=_turn(direction, "linear"), color=color)


def curved_belt(belt_id: str, center: Point, radius: float, width: float, start_deg: float, sweep_deg: float,
                ccw: bool = True, speed: float = 1.0, color: str = "#3b82f6") -> Belt:
    """Curved belt: centerline radius, CCW from start_deg through sweep_deg
    (360 = full ring). Parcels run CCW if `ccw`, else CW."""
    if radius <= 0 or width <= 0:
        raise ValueError("A curved belt needs a radius and a width.")
    if width > 2 * radius + 1e-9:
        raise ValueError(f"Belt width {width:.3f} m is more than the arc's diameter {2 * radius:.3f} m.")
    if sweep_deg >= 360.0 - 1e-6:
        start, end = 0.0, 360.0
    else:
        start = start_deg % 360.0
        end = (start + sweep_deg) % 360.0
    return Belt(id=belt_id, type="curved", shape="arc", x=center[0], y=center[1], radius=radius,
                beltWidth=width, rotation=0.0, startAngle=start, endAngle=end, speed=speed,
                directionAngle=90.0 if ccw else -90.0, color=color)


def curved_belt_from_segment(belt_id: str, seg, width: float, speed: float = 1.0,
                             color: str = "#3b82f6") -> Belt:
    """The curved belt along an arc segment (cad/path.Segment), parcels
    running from its p0 to its p1 (CCW for a positive sweep, else CW)."""
    sweep = math.degrees(seg.sweep)
    if sweep > 0:      # CCW from p0
        first = math.degrees(seg.start_angle)
    else:              # CW: the same arc is CCW from p1; parcels run CW
        first = math.degrees(math.atan2(seg.p1[1] - seg.center[1], seg.p1[0] - seg.center[0]))
    first %= 360.0
    if first > 360.0 - 1e-9:    # -0.0000001 deg, not 359.9999999
        first = 0.0
    return curved_belt(belt_id, seg.center, seg.radius, width, first, abs(sweep), ccw=sweep > 0,
                       speed=speed, color=color)


def segment_belts(new_id, points: Sequence[Point], bulges: Sequence[float], closed: bool, width: float,
                  corner_radius: float = 0.0, speed: float = 1.0, color: str = "#3b82f6") -> List[Belt]:
    """A drawn conveyor line as separate belts, each with its own drive: a
    linear belt per straight run and a curved belt per arc (sharp corners
    first rounded by `corner_radius`), in flow order (parcels run from the
    first point to the last). `new_id()` gives each belt its id. Connect
    them with belt_chain_connections. ValueError with a user-facing reason
    when a curve is too tight for the width."""
    pts, bul = list(points), list(bulges)
    if corner_radius > 0:
        pts, bul = fillet_corners(pts, bul, closed, corner_radius)
    belts: List[Belt] = []
    for seg in segments(pts, bul, closed):
        if seg.length < 1e-6:
            continue
        if seg.kind == "line":
            belts.append(linear_belt(new_id(), seg.p0, seg.p1, width, speed=speed, color=color))
            continue
        if width > 2 * seg.radius + 1e-9:
            raise ValueError(f"A curve of radius {seg.radius:.3f} m is too tight for a {width:.3f} m belt: "
                             f"use a Corner R of at least {width / 2:.3f} m, or a narrower belt.")
        belts.append(curved_belt_from_segment(new_id(), seg, width, speed=speed, color=color))
    if not belts:
        raise ValueError("The path has no length.")
    return belts


def divide_segment(seg, parts: int):
    """A line or arc segment (cad/path.Segment) cut into `parts` equal
    pieces, in its own direction."""
    from cad.path import Segment
    if parts < 1:
        raise ValueError("Divide into at least one part.")
    pieces = []
    for k in range(parts):
        t0, t1 = k / parts, (k + 1) / parts
        p0, p1 = seg.point_at(t0), seg.point_at(t1)
        if seg.kind == "arc":
            pieces.append(Segment("arc", p0, p1, seg.center, seg.radius, seg.start_angle + seg.sweep * t0,
                                  seg.sweep / parts))
        else:
            pieces.append(Segment("line", p0, p1))
    return pieces


def reversed_segment(seg):
    """The same segment, run the other way."""
    from cad.path import Segment
    if seg.kind == "arc":
        return Segment("arc", seg.p1, seg.p0, seg.center, seg.radius, seg.start_angle + seg.sweep, -seg.sweep)
    return Segment("line", seg.p1, seg.p0)


# A driven roller conveyor as the catalogue's: roller pitch, speed, colour.
ROLLER_PITCH = 0.125     # m between roller centres
ROLLER_SPEED = 0.3       # m/s
ROLLER_COLOR = "#4b5563"


def as_roller(belt: Belt) -> Belt:
    """The conveyor as a driven roller conveyor (the designer's Surface
    Roller). Rollers are straight or curved: ValueError for a path or a
    polygon outline."""
    if belt.type not in ("linear", "curved") or belt.vertexs:
        raise ValueError("A roller conveyor is straight or curved: pick a straight line, an arc or an area, "
                         "or set Type to Linear / Curved.")
    belt.surface, belt.rollerPitch = "roller", ROLLER_PITCH
    belt.speed, belt.color = ROLLER_SPEED, ROLLER_COLOR
    return belt


def divided_belts(new_id, seg, parts: int, width: float, reverse: bool = False, speed: float = 1.0,
                  color: str = "#3b82f6", surface: str = "belt", roller_pitch: float = ROLLER_PITCH) -> List[Belt]:
    """A line or arc as `parts` belts of equal length, each with its own
    drive, in flow order (from its start; `reverse`: from its end) - linear
    belts along a line, curved belts along an arc; `surface` "roller" makes
    them roller conveyors. `new_id()` gives each its id. Connect them with
    belt_chain_connections. ValueError with the reason when the arc is too
    tight for the width."""
    if reverse:
        seg = reversed_segment(seg)
    if seg.length < 1e-6:
        raise ValueError("The line has no length.")
    if seg.kind == "arc" and width > 2 * seg.radius + 1e-9:
        raise ValueError(f"An arc of radius {seg.radius:.3f} m is too tight for a {width:.3f} m wide conveyor.")
    belts = []
    for piece in divide_segment(seg, parts):
        if piece.kind == "arc":
            belt = curved_belt_from_segment(new_id(), piece, width, speed=speed, color=color)
        else:
            belt = linear_belt(new_id(), piece.p0, piece.p1, width, speed=speed, color=color)
        if surface == "roller":
            belt.surface, belt.rollerPitch = "roller", roller_pitch
        belts.append(belt)
    return belts


def belt_chain_connections(belts: Sequence[Belt], closed: bool = False) -> List[BeltConnection]:
    """Each belt's outlet joined to the next one's inlet (and the last to the
    first for a loop)."""
    pairs = list(zip(belts, belts[1:]))
    if closed and len(belts) > 1:
        pairs.append((belts[-1], belts[0]))
    return [BeltConnection(fromBelt=a.id, toBelt=b.id) for a, b in pairs]


def straight_chain(points: Sequence[Point], bulges: Sequence[float], closed: bool,
                   tol: float = 1e-6) -> Optional[Tuple[Point, Point]]:
    """(start, end) if the chain is one straight run (collinear pieces, no
    arcs, no doubling back), else None."""
    if closed or len(points) < 2 or any(abs(b) > 1e-12 for b in list(bulges)[:len(points) - 1]):
        return None
    p0, p1 = points[0], points[-1]
    length = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    if length <= tol:
        return None
    ux, uy = (p1[0] - p0[0]) / length, (p1[1] - p0[1]) / length
    last_t = -tol
    for x, y in points:
        t = (x - p0[0]) * ux + (y - p0[1]) * uy
        if abs(-(x - p0[0]) * uy + (y - p0[1]) * ux) > tol * max(1.0, length) or t < last_t - tol:
            return None
        last_t = t
    return p0, p1


def chain_arc(points: Sequence[Point], bulges: Sequence[float], closed: bool, tol: float = 1e-6):
    """(center, radius, start_deg, sweep_deg, ccw) if every piece of the chain
    is an arc of one circle turning the same way; start/sweep are CCW (as
    curved belts store them) and `ccw` says which way the chain runs."""
    segs = segments(points, bulges, closed)
    if not segs or any(s.kind != "arc" for s in segs):
        return None
    c, r = segs[0].center, segs[0].radius
    sign = 1 if segs[0].sweep > 0 else -1
    for s in segs:
        if (math.hypot(s.center[0] - c[0], s.center[1] - c[1]) > tol * max(1.0, r)
                or abs(s.radius - r) > tol * max(1.0, r) or (1 if s.sweep > 0 else -1) != sign):
            return None
    sweep = math.degrees(sum(abs(s.sweep) for s in segs))
    if sweep > 360.0 + 1e-6:
        return None
    first = math.degrees(segs[0].start_angle)
    start = first if sign > 0 else first - sweep
    return c, r, start % 360.0, min(sweep, 360.0), sign > 0


def _angular_span(angles: Sequence[float]) -> Tuple[float, float]:
    """(start, sweep) degrees of the smallest CCW arc covering all angles."""
    a = sorted(x % 360.0 for x in angles)
    gap, i = max(((a[(i + 1) % len(a)] - a[i]) % 360.0, i) for i in range(len(a)))
    return a[(i + 1) % len(a)], 360.0 - gap


def arc_sector(polygon, shapes, tol: float = 0.01):
    """(center, r_in, r_out, start_deg, sweep_deg) if the area is bounded by
    concentric arcs - a ring sector, a full ring, a pie slice or a disc -
    judged from the exact arcs of `shapes` lying on its boundary; else None."""
    boundary = polygon.boundary
    minx, miny, maxx, maxy = polygon.bounds
    reach = max(maxx - minx, maxy - miny)  # an arc can bulge well beyond its two points
    arcs = []
    for shape in shapes:
        if not shape.points:
            continue
        xs = [p[0] for p in shape.points]
        ys = [p[1] for p in shape.points]
        if min(xs) > maxx + reach or max(xs) < minx - reach or min(ys) > maxy + reach or max(ys) < miny - reach:
            continue
        for seg in segments(shape.points, shape.bulges, shape.closed):
            if seg.kind == "arc" and all(boundary.distance(ShapelyPoint(seg.point_at(t))) <= tol
                                         for t in (0.0, 0.25, 0.5, 0.75, 1.0)):
                arcs.append(seg)
    if not arcs:
        return None
    groups = []  # [center, radii, total sweep]
    for seg in arcs:
        for g in groups:
            if math.hypot(seg.center[0] - g[0][0], seg.center[1] - g[0][1]) <= tol:
                g[1].append(seg.radius)
                g[2] += abs(seg.sweep)
                break
        else:
            groups.append([seg.center, [seg.radius], abs(seg.sweep)])
    center, radii, _ = max(groups, key=lambda g: g[2])
    r_out = max(radii)
    inner = [r for r in radii if r < r_out - tol]
    r_in = max(inner) if inner else 0.0
    c = ShapelyPoint(center)
    if polygon.buffer(tol).contains(c) or (polygon.interiors and Polygon(polygon.exterior).contains(c)):
        start, sweep = 0.0, 360.0  # a disc, or a ring around the center
    else:
        angles = [math.degrees(math.atan2(y - center[1], x - center[0])) for x, y in list(polygon.exterior.coords)[:-1]
                  if math.hypot(x - center[0], y - center[1]) > tol]
        if not angles:
            return None
        start, sweep = _angular_span(angles)
    expected = math.radians(sweep) / 2.0 * (r_out ** 2 - r_in ** 2)
    if expected <= 0 or abs(expected - polygon.area) > 0.03 * expected + tol * r_out:
        return None  # arcs on the boundary, but the area is not that sector
    return center, r_in, r_out, start, sweep


def belt_from_target(belt_id: str, target, kind: str = "auto", direction: str = "forward",
                     width: float = 1.0, shapes=(), tol: float = 0.01) -> Belt:
    """The belt for a picked target (see the comment above). `shapes` are the
    drawing's exact shapes (to recognise arc-shaped areas). Raises ValueError
    with a user-facing reason when the type doesn't fit the target."""
    if kind not in KINDS:
        raise ValueError(f"Unknown belt type {kind!r}.")
    what, geom = target
    if what == "line":
        points, bulges, closed = geom
        if kind in ("auto", "linear"):
            run = straight_chain(points, bulges, closed)
            if run is not None:
                return linear_belt(belt_id, run[0], run[1], width, direction)
            if kind == "linear":
                raise ValueError("Linear needs a straight line; this one bends or turns. Use Path or Curved.")
        if kind in ("auto", "curved"):
            arc = chain_arc(points, bulges, closed)
            if arc is not None:
                center, r, start, sweep, ccw = arc
                forward = _turn(direction, "curved") == 0.0
                return curved_belt(belt_id, center, r, width, start, sweep, ccw if forward else not ccw)
            if kind == "curved":
                raise ValueError("Curved needs an arc or a circle (arcs of one circle). Use Path for mixed lines/arcs.")
        belt = path_belt(belt_id, points, bulges, closed, width)
        belt.directionAngle = _turn(direction, "path")
        return belt

    polygon = geom
    if kind in ("auto", "curved", "path"):
        sector = arc_sector(polygon, shapes, tol)
        if sector is not None:
            center, r_in, r_out, start, sweep = sector
            if kind == "path":
                pts, bul, closed = _arc_centerline(center, (r_in + r_out) / 2.0, start, sweep)
                belt = path_belt(belt_id, pts, bul, closed, r_out - r_in)
                belt.directionAngle = _turn(direction, "path")
                return belt
            ccw = _turn(direction, "curved") == 0.0
            return curved_belt(belt_id, center, (r_in + r_out) / 2.0, r_out - r_in, start, sweep, ccw)
        if kind == "curved":
            raise ValueError("Curved needs an arc-shaped area (bounded by concentric arcs). Use Linear for this area.")
        if kind == "path":
            raise ValueError("Path follows a line: click on a line or arc, or use Linear for this area.")
    from cad.snap import polygon_vertices  # here: cad.snap is the picker side, imported lazily
    corners = polygon_vertices(polygon)
    heading = (default_flow_heading(corners) + _turn(direction, "linear")) % 360.0
    return outline_belt(belt_id, corners, flow_heading=heading)


def _arc_centerline(center: Point, radius: float, start_deg: float, sweep_deg: float):
    """Points/bulges of a CCW arc (or full circle) for a path belt."""
    def at(deg):
        return (center[0] + radius * math.cos(math.radians(deg)), center[1] + radius * math.sin(math.radians(deg)))
    if sweep_deg >= 360.0 - 1e-6:
        return [at(start_deg), at(start_deg + 180.0)], [1.0, 1.0], True
    n = max(1, math.ceil(sweep_deg / 179.0))  # one bulge covers < 360 deg; keep each piece under 180
    step = sweep_deg / n
    pts = [at(start_deg + i * step) for i in range(n + 1)]
    return pts, [math.tan(math.radians(step) / 4.0)] * n + [0.0], False


def flow_arrows(belt: Belt) -> List[Tuple[float, float, float]]:
    """(x, y, heading_deg) markers showing where parcels go, for previews."""
    if belt.type == "linear":
        heading = belt.rotation + belt.directionAngle
        if belt.vertexs:
            n = len(belt.vertexs)
            return [(belt.x + sum(v["x"] for v in belt.vertexs) / n, belt.y + sum(v["y"] for v in belt.vertexs) / n,
                     heading)]
        return [(belt.x, belt.y, heading)]
    if belt.type == "curved":
        sweep = (belt.endAngle - belt.startAngle) % 360.0 or 360.0
        ccw = belt.directionAngle > 0
        out = []
        for frac in ((0.125, 0.625) if sweep >= 359.999 else (0.5,)):
            a = math.radians(belt.startAngle + belt.rotation + sweep * frac)
            out.append((belt.x + belt.radius * math.cos(a), belt.y + belt.radius * math.sin(a),
                        math.degrees(a) + (90.0 if ccw else -90.0)))
        return out
    if belt.type == "path" and belt.path:
        pts, bulges = belt_world_path(belt)
        segs = segments(pts, bulges, belt.closed)
        backwards = math.cos(math.radians(belt.directionAngle)) < 0
        out = []
        for frac in ((0.25, 0.75) if belt.closed else (0.5,)):
            seg = segs[min(int(frac * len(segs)), len(segs) - 1)]
            x, y = seg.point_at(0.5)
            out.append((x, y, math.degrees(seg.heading_at(0.5)) + (180.0 if backwards else 0.0)))
        return out
    return []
