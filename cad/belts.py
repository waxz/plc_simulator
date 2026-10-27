"""
cad/belts.py - build Belt models from drawn or picked geometry (world metres).

  outline_belt()  a polygon belt (shape "quadrilateral", any vertex count):
                  the drawn/picked outline IS the belt surface. Parcels move
                  in one direction across it (rotation + directionAngle).
  path_belt()     a belt along a centerline (cad/path.py), offset by width.

Both store their geometry relative to the belt's x/y (the middle of the
anchors' bounding box), like every other belt, so moving and rotating them
in the editor works the same way.
"""
import math
from typing import Optional, Sequence, Tuple

from shapely.geometry import Polygon

from cad.path import recenter
from config_parser import Belt, PathNode

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


def path_belt(belt_id: str, points: Sequence[Point], bulges: Sequence[float], closed: bool, width: float,
              speed: float = 1.0, color: str = "#3b82f6") -> Belt:
    if len(points) < 2:
        raise ValueError("A belt path needs at least 2 anchors.")
    (cx, cy), local = recenter(points)
    return Belt(id=belt_id, type="path", shape="path", x=cx, y=cy, rotation=0.0, beltWidth=width,
                speed=speed, directionAngle=0.0, color=color, closed=closed,
                path=[PathNode(x=x, y=y, bulge=b) for (x, y), b in zip(local, bulges)])
