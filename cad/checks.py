"""
cad/checks.py - layout design checks on belt/sensor footprints, with Shapely.

check_layout(config) returns a list of Issue, most severe first:

  error    broken connection   - a saved connection's ports no longer meet
                                 (gap or heading mismatch), or names a belt
                                 that doesn't exist
  error    belt overlap        - two belts that aren't connected overlap
                                 (parcels would be driven by both)
  warning  open outlet         - parcels leaving a belt land on no belt and
                                 no sink
  warning  sensor off belt     - a sensor that no belt passes under
  warning  source off belt     - a source that drops parcels onto the floor

Footprints use the same geometry the simulator does (math_utils polygons,
arcs buffered to the belt width), so "overlap" here means overlap in physics.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from shapely import STRtree
from shapely.geometry import LineString, Point, Polygon, box

from config_parser import Belt, SimulatorConfig
from cad.ports import belt_ports, centerline_points, heading_difference
from math_utils import get_belt_polygon, get_rect_vertices

OVERLAP_AREA_TOLERANCE = 0.005    # m^2 - ignore slivers from touching edges
CONNECTION_GAP_TOLERANCE = 0.005  # m
CONNECTION_ANGLE_TOLERANCE = 1.0  # degrees
OUTLET_PROBE = 0.05               # m - how far past an outlet a parcel lands
TRANSFER_OVERLAP_FRACTION = 0.15  # overlaps below this share of the smaller belt are transfer zones
STACKED_OVERLAP_FRACTION = 0.5    # overlaps above it are always errors


@dataclass
class Issue:
    severity: str                 # "error" | "warning"
    kind: str
    message: str
    item_ids: List[str] = field(default_factory=list)
    location: Optional[Tuple[float, float]] = None


def belt_footprint(belt: Belt) -> Polygon:
    if belt.type == "curved":
        line = LineString(centerline_points(belt, segments_per_90deg=32))
        return line.buffer(belt.beltWidth / 2.0, cap_style="flat", join_style="round")
    return Polygon(get_belt_polygon(belt))


def _is_transfer(overlap, a: Belt, b: Belt) -> bool:
    """An overlap at the end of a belt is a transfer zone (one belt's end
    laid slightly over the next, how parcels hand over in the simulator),
    not a design error: true if a port of either belt lies within one belt
    width of the overlap, or - for side merges and custom-shaped belts,
    which have no ports - if the overlap is a small fraction of the smaller
    belt (an end laid over a neighbour, not two belts stacked)."""
    smaller = min(belt_footprint(a).area, belt_footprint(b).area)
    fraction = overlap.area / smaller if smaller > 0 else 1.0
    if fraction < TRANSFER_OVERLAP_FRACTION:
        return True
    if fraction > STACKED_OVERLAP_FRACTION:
        return False  # belts lying on top of each other, ports or not
    for belt in (a, b):
        ports = belt_ports(belt)
        if ports is None:
            continue
        for port in (ports.inlet, ports.outlet):
            if overlap.distance(Point(port.x, port.y)) <= belt.beltWidth:
                return True
    return False


def _rect(x, y, w, h, rotation) -> Polygon:
    return Polygon(get_rect_vertices(x, y, w, h, rotation))


def check_layout(config: SimulatorConfig) -> List[Issue]:
    issues: List[Issue] = []
    belts = {b.id: b for b in config.belts}
    footprints = {b.id: belt_footprint(b) for b in config.belts}
    ids = list(footprints)
    tree = STRtree([footprints[i] for i in ids])
    connected_pairs = {frozenset((c.fromBelt, c.toBelt)) for c in config.connections}
    connected_from = {c.fromBelt for c in config.connections if c.toBelt in belts}

    # Saved connections still hold?
    for conn in config.connections:
        missing = [i for i in (conn.fromBelt, conn.toBelt) if i not in belts]
        if missing:
            issues.append(Issue("error", "broken connection",
                                f"Connection {conn.fromBelt} -> {conn.toBelt} refers to missing belt(s) {', '.join(missing)}.",
                                [i for i in (conn.fromBelt, conn.toBelt) if i in belts]))
            continue
        pa, pb = belt_ports(belts[conn.fromBelt]), belt_ports(belts[conn.toBelt])
        if pa is None or pb is None:
            issues.append(Issue("error", "broken connection",
                                f"{conn.fromBelt} -> {conn.toBelt}: one belt no longer has an inlet/outlet.",
                                [conn.fromBelt, conn.toBelt]))
            continue
        gap = math.hypot(pa.outlet.x - pb.inlet.x, pa.outlet.y - pb.inlet.y)
        turn = heading_difference(pa.outlet.heading, pb.inlet.heading)
        if gap > CONNECTION_GAP_TOLERANCE or turn > CONNECTION_ANGLE_TOLERANCE:
            issues.append(Issue("error", "broken connection",
                                f"{conn.fromBelt} -> {conn.toBelt} is out of place: gap {gap * 1000:.0f} mm, "
                                f"angle {turn:.1f} deg. Use Tools > Solve Connections.",
                                [conn.fromBelt, conn.toBelt], (pa.outlet.x, pa.outlet.y)))

    # Overlapping belts (connected neighbours may touch at the joint).
    for i, a_id in enumerate(ids):
        for j in tree.query(footprints[a_id]):
            b_id = ids[j]
            if j <= i or frozenset((a_id, b_id)) in connected_pairs:
                continue
            inter = footprints[a_id].intersection(footprints[b_id])
            if inter.area > OVERLAP_AREA_TOLERANCE and not _is_transfer(inter, belts[a_id], belts[b_id]):
                c = inter.centroid
                issues.append(Issue("error", "belt overlap",
                                    f"{a_id} and {b_id} overlap by {inter.area:.2f} m^2 - parcels there "
                                    f"are driven by both belts.", [a_id, b_id], (c.x, c.y)))

    # Outlets that drop parcels into nothing.
    sinks = [box(s.x - s.width / 2, s.y - s.height / 2, s.x + s.width / 2, s.y + s.height / 2) for s in config.sinks]
    for belt in config.belts:
        ports = belt_ports(belt)
        if ports is None or belt.id in connected_from:
            continue
        rad = math.radians(ports.outlet.heading)
        probe = Point(ports.outlet.x + OUTLET_PROBE * math.cos(rad), ports.outlet.y + OUTLET_PROBE * math.sin(rad))
        on_belt = any(ids[j] != belt.id and footprints[ids[j]].covers(probe) for j in tree.query(probe))
        if not on_belt and not any(s.covers(probe) for s in sinks):
            issues.append(Issue("warning", "open outlet",
                                f"Parcels leaving {belt.id} at ({ports.outlet.x:.2f}, {ports.outlet.y:.2f}) "
                                f"land on no belt and no sink.", [belt.id], (ports.outlet.x, ports.outlet.y)))

    # Sensors and sources must sit over a belt.
    for sensor in config.sensors:
        shape = _rect(sensor.x, sensor.y, sensor.width, sensor.height, sensor.rotation)
        if not any(footprints[ids[j]].intersects(shape) for j in tree.query(shape)):
            issues.append(Issue("warning", "sensor off belt",
                                f"Sensor {sensor.id} isn't over any belt - it will never see a parcel.",
                                [sensor.id], (sensor.x, sensor.y)))
    for source in config.sources:
        spot = Point(source.x, source.y)
        if not any(footprints[ids[j]].covers(spot) for j in tree.query(spot)):
            issues.append(Issue("warning", "source off belt",
                                f"Source {source.id} drops parcels onto the floor, not a belt.",
                                [source.id], (source.x, source.y)))

    issues.sort(key=lambda issue: 0 if issue.severity == "error" else 1)
    return issues
