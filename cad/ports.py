"""
cad/ports.py - where parcels enter and leave a belt.

Every linear (rectangle) or curved belt has an inlet and an outlet port on
its centerline: a point plus the flow heading there. Flow direction comes
from the same fields physics uses (`rotation + directionAngle`, the sign of
`speed`, and for curved belts the sign of `directionAngle`), so ports always
agree with how parcels actually move. A belt whose flow runs across its
length (a cross transfer) or a custom polygon belt has no ports.

Angles here are world degrees, CCW from +X. A curved belt's arc is
`startAngle..endAngle` CCW around (x, y), turned by `rotation` - the editor
draws it that way; the solver (constraints.py) writes `rotation = 0` back.
"""
import math
from dataclasses import dataclass
from typing import Optional

from cad.path import belt_world_path, end_heading, flatten, path_flow_sign, segments, start_heading
from config_parser import Belt


@dataclass(frozen=True)
class Port:
    x: float
    y: float
    heading: float  # degrees, direction parcels travel at this point


@dataclass(frozen=True)
class BeltPorts:
    inlet: Port
    outlet: Port


def flow_sign(belt: Belt) -> float:
    """+1 for forward flow, -1 for a belt commanded backwards. A belt at
    speed 0 (normal at design time) counts as forward."""
    return -1.0 if (belt.speed or 0.0) < 0.0 else 1.0


def is_portable(belt: Belt) -> bool:
    if belt.type == "path":
        # A loop has no inlet/outlet.
        return not belt.closed and belt.path is not None and len(belt.path) >= 2
    if belt.type == "curved":
        return belt.radius > 0.0
    if getattr(belt, "shape", "rectangle") == "quadrilateral":
        return False
    return linear_axis_sign(belt) != 0.0


def linear_axis_sign(belt: Belt) -> float:
    """+1 if parcels run toward the belt's +length end, -1 toward the -length
    end, 0 if they run mostly across it (no inlet/outlet along its length)."""
    along = math.cos(math.radians(belt.directionAngle)) * flow_sign(belt)
    if abs(along) < 0.5:  # more than 60 degrees off the length axis
        return 0.0
    return 1.0 if along > 0 else -1.0


def arc_angles(belt: Belt) -> tuple[float, float]:
    """(start, end) world angles of a curved belt in degrees, with end
    following start CCW (end in (start, start + 360])."""
    start = belt.startAngle + belt.rotation
    sweep = (belt.endAngle - belt.startAngle) % 360.0 or 360.0
    return start, start + sweep


def curved_ccw(belt: Belt) -> bool:
    """True if parcels travel CCW around the arc (physics: directionAngle > 0
    means CCW, flipped by a negative speed)."""
    ccw = belt.directionAngle > 0
    return ccw if flow_sign(belt) > 0 else not ccw


def belt_ports(belt: Belt) -> Optional[BeltPorts]:
    if not is_portable(belt):
        return None
    if belt.type == "path":
        pts, bulges = belt_world_path(belt)
        segs = segments(pts, bulges, False)
        first = Port(pts[0][0], pts[0][1], math.degrees(start_heading(segs)))
        last = Port(pts[-1][0], pts[-1][1], math.degrees(end_heading(segs)))
        if path_flow_sign(belt) > 0:
            return BeltPorts(first, last)
        return BeltPorts(Port(last.x, last.y, last.heading + 180.0), Port(first.x, first.y, first.heading + 180.0))
    if belt.type == "curved":
        start, end = arc_angles(belt)
        r = belt.radius

        def at(angle: float) -> tuple[float, float]:
            rad = math.radians(angle)
            return belt.x + r * math.cos(rad), belt.y + r * math.sin(rad)

        if curved_ccw(belt):
            (ix, iy), (ox, oy) = at(start), at(end)
            return BeltPorts(Port(ix, iy, start + 90.0), Port(ox, oy, end + 90.0))
        (ix, iy), (ox, oy) = at(end), at(start)
        return BeltPorts(Port(ix, iy, end - 90.0), Port(ox, oy, start - 90.0))

    sign = linear_axis_sign(belt)
    axis = math.radians(belt.rotation)
    ux, uy = math.cos(axis), math.sin(axis)
    half = belt.length / 2.0
    heading = belt.rotation if sign > 0 else belt.rotation + 180.0
    inlet = Port(belt.x - sign * half * ux, belt.y - sign * half * uy, heading)
    outlet = Port(belt.x + sign * half * ux, belt.y + sign * half * uy, heading)
    return BeltPorts(inlet, outlet)


def heading_difference(a: float, b: float) -> float:
    """Smallest absolute difference between two headings, degrees in [0, 180]."""
    d = (a - b) % 360.0
    return min(d, 360.0 - d)


def centerline_points(belt: Belt, segments_per_90deg: int = 16) -> list[tuple[float, float]]:
    """Centerline from one end to the other (not necessarily in flow order)."""
    if belt.type == "path":
        pts, bulges = belt_world_path(belt)
        return flatten(segments(pts, bulges, belt.closed))[0]
    if belt.type == "curved":
        start, end = arc_angles(belt)
        n = max(2, int(math.ceil((end - start) / 90.0 * segments_per_90deg)) + 1)
        pts = []
        for i in range(n):
            rad = math.radians(start + (end - start) * i / (n - 1))
            pts.append((belt.x + belt.radius * math.cos(rad), belt.y + belt.radius * math.sin(rad)))
        return pts
    axis = math.radians(belt.rotation)
    ux, uy = math.cos(axis), math.sin(axis)
    half = belt.length / 2.0
    return [(belt.x - half * ux, belt.y - half * uy), (belt.x + half * ux, belt.y + half * uy)]
