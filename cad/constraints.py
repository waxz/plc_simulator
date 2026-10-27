"""
cad/constraints.py - geometric constraints between belts, solved with
planegcs (FreeCAD's 2D sketch solver).

Each belt becomes sketch geometry along its centerline: a line of fixed
length for a linear belt, an arc of fixed radius and sweep for a curved one.
A connection A -> B adds "A's outlet is coincident with B's inlet" plus
heading continuity (parallel lines, or line/arc or arc/arc tangency).

  solve_layout()  - re-establish every connection after belts moved, holding
                    a chosen set of belts fixed (the ones the user dragged).
  connect_belts() - attach B to A's outlet (A stays put).
  fillet_belts()  - join two straight belts with a tangent curved belt of a
                    given radius, trimming both straights to meet it.

Results are plain dicts of Belt geometry fields, so the caller (the editor)
can apply them through its undo stack. Nothing here imports Qt.
"""
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Sequence, Set

from planegcs import SolveStatus, Sketch

from config_parser import Belt, BeltConnection
from cad.ports import (
    arc_angles,
    belt_ports,
    curved_ccw,
    heading_difference,
    linear_axis_sign,
)

GEOMETRY_FIELDS = ("x", "y", "rotation", "length", "radius", "startAngle", "endAngle")
POSITION_TOLERANCE = 1e-6   # metres
HEADING_TOLERANCE = 1e-3    # degrees
MIN_TRIMMED_LENGTH = 0.05   # metres - a fillet may not shorten a straight below this


class ConstraintError(ValueError):
    """The requested geometry can't be built (bad selection, parallel belts,
    radius too large, solver failed). The message is meant for the user."""


def belt_geometry(belt: Belt) -> Dict[str, float]:
    return {f: getattr(belt, f) for f in GEOMETRY_FIELDS}


def _copy_with(belt: Belt, geometry: Dict[str, float]) -> Belt:
    return belt.model_copy(update=geometry)


# ---------------------------------------------------------------------------
# Rigid pre-alignment (gives the solver a near-exact starting point)
# ---------------------------------------------------------------------------

def _rotate_about(px: float, py: float, cx: float, cy: float, deg: float) -> tuple:
    rad = math.radians(deg)
    c, s = math.cos(rad), math.sin(rad)
    dx, dy = px - cx, py - cy
    return cx + dx * c - dy * s, cy + dx * s + dy * c


def _rigid_move(belt: Belt, pivot: tuple, turn_deg: float, shift: tuple) -> Dict[str, float]:
    """Geometry of `belt` rotated by `turn_deg` about `pivot`, then shifted.
    A curved belt's rotation is folded into its absolute arc angles."""
    x, y = _rotate_about(belt.x, belt.y, pivot[0], pivot[1], turn_deg)
    geom = belt_geometry(belt)
    geom["x"], geom["y"] = x + shift[0], y + shift[1]
    if belt.type == "curved":
        start, end = arc_angles(belt)
        geom["startAngle"] = (start + turn_deg) % 360.0
        geom["endAngle"] = (end + turn_deg) % 360.0
        geom["rotation"] = 0.0
    else:
        geom["rotation"] = (belt.rotation + turn_deg) % 360.0
    return geom


def _align_inlet_to(belt: Belt, port) -> Dict[str, float]:
    """Move `belt` rigidly so its inlet sits on `port` with the same heading."""
    own = belt_ports(belt).inlet
    turn = port.heading - own.heading
    rx, ry = _rotate_about(own.x, own.y, belt.x, belt.y, turn)
    return _rigid_move(belt, (belt.x, belt.y), turn, (port.x - rx, port.y - ry))


def _align_outlet_to(belt: Belt, port) -> Dict[str, float]:
    own = belt_ports(belt).outlet
    turn = port.heading - own.heading
    rx, ry = _rotate_about(own.x, own.y, belt.x, belt.y, turn)
    return _rigid_move(belt, (belt.x, belt.y), turn, (port.x - rx, port.y - ry))


# ---------------------------------------------------------------------------
# Sketch construction
# ---------------------------------------------------------------------------

@dataclass
class _Entity:
    belt: Belt
    kind: str            # "line" | "arc"
    geo: object          # LineId | ArcId
    inlet: object        # PointId
    outlet: object       # PointId
    points: dict         # name -> PointId, for read-back


def _add_belt(sketch: Sketch, belt: Belt, fixed: bool) -> _Entity:
    ports = belt_ports(belt)
    if belt.type == "curved":
        start, end = arc_angles(belt)
        r = belt.radius
        center = (belt.x, belt.y)
        s_xy = (center[0] + r * math.cos(math.radians(start)), center[1] + r * math.sin(math.radians(start)))
        e_xy = (center[0] + r * math.cos(math.radians(end)), center[1] + r * math.sin(math.radians(end)))
        add = sketch.add_fixed_point if fixed else sketch.add_point
        c = add(*center)
        s = add(*s_xy)
        e = sketch.add_point(*e_xy)
        arc = sketch.add_arc_cse(c, s, e, r, math.radians(start), math.radians(end))
        sketch.set_arc_radius(arc, r)
        sketch.set_arc_angle(arc, math.radians(end - start))
        inlet, outlet = (s, e) if curved_ccw(belt) else (e, s)
        return _Entity(belt, "arc", arc, inlet, outlet, {"center": c, "start": s, "end": e})

    add = sketch.add_fixed_point if fixed else sketch.add_point
    p_in = add(ports.inlet.x, ports.inlet.y)
    p_out = add(ports.outlet.x, ports.outlet.y)
    line = sketch.add_line(p_in, p_out)
    if not fixed:
        sketch.set_p2p_distance(p_in, p_out, belt.length)
    return _Entity(belt, "line", line, p_in, p_out, {"inlet": p_in, "outlet": p_out})


def _connect(sketch: Sketch, a: _Entity, b: _Entity) -> None:
    sketch.coincident(a.outlet, b.inlet)
    if a.kind == "line" and b.kind == "line":
        sketch.parallel(a.geo, b.geo)
    elif a.kind == "line":
        sketch.tangent_line_arc(a.geo, b.geo)
    elif b.kind == "line":
        sketch.tangent_line_arc(b.geo, a.geo)
    else:
        sketch.tangent_arc_arc(a.geo, b.geo)


def _read_back(sketch: Sketch, ent: _Entity) -> Dict[str, float]:
    belt = ent.belt
    geom = belt_geometry(belt)
    if ent.kind == "arc":
        info = sketch.get_arc(ent.geo)
        start = math.degrees(info.start_angle) % 360.0
        sweep = math.degrees(info.end_angle - info.start_angle)
        geom.update(x=info.center[0], y=info.center[1], radius=info.radius,
                    startAngle=start, endAngle=(start + sweep) % 360.0, rotation=0.0)
        return geom
    (ix, iy), (ox, oy) = sketch.get_point(ent.inlet), sketch.get_point(ent.outlet)
    heading = math.degrees(math.atan2(oy - iy, ox - ix))
    rotation = heading if linear_axis_sign(belt) > 0 else heading + 180.0
    geom.update(x=(ix + ox) / 2.0, y=(iy + oy) / 2.0, rotation=rotation % 360.0,
                length=math.hypot(ox - ix, oy - iy))
    return geom


def _check_connections(belts: Dict[str, Belt], connections: Iterable[BeltConnection]) -> None:
    for conn in connections:
        a, b = belt_ports(belts[conn.fromBelt]).outlet, belt_ports(belts[conn.toBelt]).inlet
        gap = math.hypot(a.x - b.x, a.y - b.y)
        turn = heading_difference(a.heading, b.heading)
        if gap > POSITION_TOLERANCE * 100 or turn > HEADING_TOLERANCE * 100:
            raise ConstraintError(
                f"Could not keep {conn.fromBelt} -> {conn.toBelt} connected "
                f"(gap {gap * 1000:.1f} mm, angle {turn:.2f} deg). The belts may be over-constrained."
            )


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def validate_connection(a: Belt, b: Belt) -> None:
    for belt in (a, b):
        if belt_ports(belt) is None:
            raise ConstraintError(
                f"{belt.id} has no inlet/outlet (custom-shaped belt, or parcels run across it)."
            )
    if a.id == b.id:
        raise ConstraintError("A belt can't be connected to itself.")


def solve_layout(belts: Sequence[Belt], connections: Sequence[BeltConnection],
                 fixed_ids: Iterable[str]) -> Dict[str, Dict[str, float]]:
    """New geometry for every non-fixed belt that takes part in a connection.

    Belts in `fixed_ids` don't move. In each connected group with no fixed
    belt, the first belt (config order) is held instead, so a group never
    floats. Returns {belt_id: geometry} for belts whose geometry changed."""
    by_id = {b.id: b for b in belts}
    conns = [c for c in connections if c.fromBelt in by_id and c.toBelt in by_id]
    if not conns:
        return {}
    for c in conns:
        validate_connection(by_id[c.fromBelt], by_id[c.toBelt])

    neighbours: Dict[str, List[tuple]] = {}
    for c in conns:
        neighbours.setdefault(c.fromBelt, []).append((c.toBelt, "down"))
        neighbours.setdefault(c.toBelt, []).append((c.fromBelt, "up"))

    fixed: Set[str] = {i for i in fixed_ids if i in neighbours}
    order = [b.id for b in belts if b.id in neighbours]
    seen: Set[str] = set()
    for belt_id in order:  # one anchor per group without a fixed belt
        if belt_id in seen:
            continue
        group, queue = set(), deque([belt_id])
        while queue:
            cur = queue.popleft()
            if cur in group:
                continue
            group.add(cur)
            queue.extend(n for n, _ in neighbours[cur])
        seen |= group
        if not group & fixed:
            fixed.add(belt_id)

    # Rigid pre-alignment, breadth-first out from the fixed belts.
    working = {i: by_id[i] for i in neighbours}
    placed = set(fixed)
    queue = deque(fixed)
    while queue:
        cur = queue.popleft()
        for other, rel in neighbours[cur]:
            if other in placed:
                continue
            src = belt_ports(working[cur])
            geom = (_align_inlet_to(working[other], src.outlet) if rel == "down"
                    else _align_outlet_to(working[other], src.inlet))
            working[other] = _copy_with(working[other], geom)
            placed.add(other)
            queue.append(other)

    sketch = Sketch()
    entities = {i: _add_belt(sketch, working[i], i in fixed) for i in order}
    for c in conns:
        _connect(sketch, entities[c.fromBelt], entities[c.toBelt])
    status = sketch.solve()
    if status != SolveStatus.Success:
        raise ConstraintError(f"The constraint solver could not satisfy the connections ({status.name}).")

    result: Dict[str, Dict[str, float]] = {}
    solved = dict(by_id)
    for belt_id, ent in entities.items():
        if belt_id in fixed:
            continue
        geom = _read_back(sketch, ent)
        solved[belt_id] = _copy_with(by_id[belt_id], geom)
        if any(abs(geom[f] - getattr(by_id[belt_id], f)) > 1e-9 for f in GEOMETRY_FIELDS):
            result[belt_id] = geom
    _check_connections(solved, conns)
    return result


def connect_belts(a: Belt, b: Belt, belts: Sequence[Belt],
                  connections: Sequence[BeltConnection]) -> Dict[str, Dict[str, float]]:
    """Connect a's outlet to b's inlet, holding `a` (and every belt not
    connected to b) in place. Returns new geometry per moved belt."""
    validate_connection(a, b)
    if any(c.fromBelt == a.id and c.toBelt == b.id for c in connections):
        raise ConstraintError(f"{a.id} is already connected to {b.id}.")
    new_conns = list(connections) + [BeltConnection(fromBelt=a.id, toBelt=b.id)]
    return solve_layout(belts, new_conns, fixed_ids=[a.id])


@dataclass
class FilletResult:
    upstream: Dict[str, float]     # new geometry for the upstream straight
    downstream: Dict[str, float]   # new geometry for the downstream straight
    curve: Belt                    # the new curved belt between them
    connections: List[BeltConnection]


def _line_intersection(p, d, q, e):
    """Intersection of p + t*d and q + s*e, or None if parallel."""
    den = d[0] * e[1] - d[1] * e[0]
    if abs(den) < 1e-9:
        return None
    t = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
    return p[0] + t * d[0], p[1] + t * d[1]


def fillet_belts(a: Belt, b: Belt, radius: float, curve_id: str) -> FilletResult:
    """Join straight belt `a` (upstream) to straight belt `b` (downstream)
    with a curved belt of `radius` metres, tangent to both. `a` keeps its
    inlet and `b` its outlet; their facing ends are trimmed or extended to
    meet the curve."""
    validate_connection(a, b)
    if a.type != "linear" or b.type != "linear":
        raise ConstraintError("Fillet needs two straight belts.")
    if radius <= 0:
        raise ConstraintError("Radius must be positive.")
    pa, pb = belt_ports(a), belt_ports(b)
    ha = (math.cos(math.radians(pa.inlet.heading)), math.sin(math.radians(pa.inlet.heading)))
    hb = (math.cos(math.radians(pb.inlet.heading)), math.sin(math.radians(pb.inlet.heading)))
    corner = _line_intersection((pa.inlet.x, pa.inlet.y), ha, (pb.outlet.x, pb.outlet.y), hb)
    if corner is None:
        raise ConstraintError(f"{a.id} and {b.id} are parallel - there is no corner to round.")
    turn = math.degrees(math.atan2(ha[0] * hb[1] - ha[1] * hb[0], ha[0] * hb[0] + ha[1] * hb[1]))
    if abs(turn) < 0.5 or abs(turn) > 179.5:
        raise ConstraintError(f"{a.id} and {b.id} are (anti)parallel - nothing to round.")
    left = turn > 0

    # Closed-form starting guess; the solver then enforces it exactly.
    tangent = radius * math.tan(math.radians(abs(turn)) / 2.0)
    a_out = (corner[0] - tangent * ha[0], corner[1] - tangent * ha[1])
    b_in = (corner[0] + tangent * hb[0], corner[1] + tangent * hb[1])
    na = (-ha[1], ha[0]) if left else (ha[1], -ha[0])
    center = (a_out[0] + radius * na[0], a_out[1] + radius * na[1])

    reach_a = (corner[0] - pa.inlet.x) * ha[0] + (corner[1] - pa.inlet.y) * ha[1]
    reach_b = (pb.outlet.x - corner[0]) * hb[0] + (pb.outlet.y - corner[1]) * hb[1]
    if reach_a <= 0 or reach_b <= 0:
        raise ConstraintError(f"{a.id} doesn't flow toward {b.id}'s line - check the flow directions.")

    sketch = Sketch()
    a_in = sketch.add_fixed_point(pa.inlet.x, pa.inlet.y)
    a_end = sketch.add_point(*a_out)
    b_start = sketch.add_point(*b_in)
    b_out = sketch.add_fixed_point(pb.outlet.x, pb.outlet.y)
    ref_a = sketch.add_line(sketch.add_fixed_point(pa.inlet.x, pa.inlet.y),
                            sketch.add_fixed_point(pa.inlet.x + ha[0], pa.inlet.y + ha[1]))
    ref_b = sketch.add_line(sketch.add_fixed_point(pb.outlet.x, pb.outlet.y),
                            sketch.add_fixed_point(pb.outlet.x + hb[0], pb.outlet.y + hb[1]))
    line_a = sketch.add_line(a_in, a_end)
    line_b = sketch.add_line(b_start, b_out)
    sketch.point_on_line(a_end, ref_a)
    sketch.point_on_line(b_start, ref_b)

    # planegcs arcs run CCW from start to end; a right turn runs the arc backwards.
    first, second = (a_out, b_in) if left else (b_in, a_out)
    ang1 = math.atan2(first[1] - center[1], first[0] - center[0])
    ang2 = math.atan2(second[1] - center[1], second[0] - center[0])
    while ang2 <= ang1:
        ang2 += 2 * math.pi
    c = sketch.add_point(*center)
    s = sketch.add_point(*first)
    e = sketch.add_point(*second)
    arc = sketch.add_arc_cse(c, s, e, radius, ang1, ang2)
    sketch.set_arc_radius(arc, radius)
    sketch.coincident(a_end, s if left else e)
    sketch.coincident(b_start, e if left else s)
    sketch.tangent_line_arc(line_a, arc)
    sketch.tangent_line_arc(line_b, arc)
    status = sketch.solve()
    if status != SolveStatus.Success:
        raise ConstraintError(f"The constraint solver could not fit the curve ({status.name}).")

    a_end_xy, b_start_xy = sketch.get_point(a_end), sketch.get_point(b_start)
    new_a_len = math.hypot(a_end_xy[0] - pa.inlet.x, a_end_xy[1] - pa.inlet.y)
    new_b_len = math.hypot(pb.outlet.x - b_start_xy[0], pb.outlet.y - b_start_xy[1])
    # The trimmed end must still lie downstream of the fixed end.
    if ((a_end_xy[0] - pa.inlet.x) * ha[0] + (a_end_xy[1] - pa.inlet.y) * ha[1] < MIN_TRIMMED_LENGTH
            or (pb.outlet.x - b_start_xy[0]) * hb[0] + (pb.outlet.y - b_start_xy[1]) * hb[1] < MIN_TRIMMED_LENGTH):
        raise ConstraintError(f"Radius {radius:g} m is too large for these belts - it would use up a whole straight.")

    def straight(belt, inlet_xy, outlet_xy, length):
        geom = belt_geometry(belt)
        geom.update(x=(inlet_xy[0] + outlet_xy[0]) / 2.0, y=(inlet_xy[1] + outlet_xy[1]) / 2.0, length=length)
        return geom

    info = sketch.get_arc(arc)
    start = math.degrees(info.start_angle) % 360.0
    end = (start + math.degrees(info.end_angle - info.start_angle)) % 360.0
    curve = Belt(
        id=curve_id, type="curved", shape="arc", x=info.center[0], y=info.center[1],
        radius=info.radius, startAngle=start, endAngle=end, rotation=0.0,
        beltWidth=a.beltWidth, speed=abs(a.speed), directionAngle=90.0 if left else -90.0,
        color=a.color, friction=a.friction, drive=a.drive,
        transition_momentum_duration=a.transition_momentum_duration,
    )
    return FilletResult(
        upstream=straight(a, (pa.inlet.x, pa.inlet.y), a_end_xy, new_a_len),
        downstream=straight(b, b_start_xy, (pb.outlet.x, pb.outlet.y), new_b_len),
        curve=curve,
        connections=[BeltConnection(fromBelt=a.id, toBelt=curve_id),
                     BeltConnection(fromBelt=curve_id, toBelt=b.id)],
    )
