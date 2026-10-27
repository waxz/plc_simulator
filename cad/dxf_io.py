"""
cad/dxf_io.py - DXF in and out, with ezdxf.

read_background(path)  - flatten a DXF floor plan into polylines (drawing
                         units) for the editor's locked background layer.
                         Block references are expanded; text is skipped.
export_layout(config, path) - write the layout as a DXF in metres, one layer
                         per component kind, for CAD / documentation.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import ezdxf
from ezdxf import disassemble, path as ezpath, units

from config_parser import SimulatorConfig
from cad.checks import belt_footprint
from shapely.geometry import LineString

from cad.path import belt_world_path, path_flow_sign
from cad.ports import arc_angles, belt_ports, centerline_points, curved_ccw
from math_utils import get_rect_vertices

Polyline = List[Tuple[float, float]]

# Unitless drawings ($INSUNITS = 0) are assumed to be millimetres, the usual
# unit for plant / factory layouts.
DEFAULT_SCALE_FOR_UNITLESS = 0.001
_SKIPPED_TYPES = {"TEXT", "MTEXT", "ATTRIB", "ATTDEF", "POINT", "VIEWPORT", "IMAGE", "WIPEOUT"}

LAYERS = {
    # name: (ACI color, description)
    "BELTS": (5, "belt footprints"),
    "BELT_CENTERLINES": (8, "belt centerlines"),
    "FLOW": (3, "flow direction arrows"),
    "SENSORS": (1, "IR sensors and laser banners"),
    "SOURCES": (4, "parcel sources"),
    "SINKS": (6, "parcel sinks"),
    "IO": (2, "buttons and LEDs"),
    "CONNECTIONS": (140, "belt connections"),
    "LABELS": (7, "component ids"),
    "SKETCH": (4, "construction geometry"),
}


@dataclass
class DrawingShape:
    """One drawing entity kept exact for picking and snapping: vertices plus
    DXF-style bulges (0 = straight, tan(sweep/4) = arc), drawing units.
    Lines, arcs, circles and (LW)polylines keep their true arcs; other curves
    (splines, ellipses, hatch boundaries) are flattened to straight pieces."""
    points: List[Tuple[float, float]]
    bulges: List[float]
    closed: bool
    layer: str = "0"


@dataclass
class BackgroundGeometry:
    polylines: List[Polyline]          # drawing units
    scale: float                       # metres per drawing unit
    units_name: str                    # e.g. "mm", or "unitless (assumed mm)"
    bounds: Optional[Tuple[float, float, float, float]]  # min_x, min_y, max_x, max_y, drawing units
    shapes: List[DrawingShape] = None  # exact entities, drawing units


def read_background(path: str, flatten_tolerance: float = 0.0) -> BackgroundGeometry:
    """Polylines of every drawable entity in modelspace. `flatten_tolerance`
    is the max chord error for curves in drawing units (0 = 1/1000 of the
    drawing size)."""
    doc = ezdxf.readfile(path)
    insunits = int(doc.header.get("$INSUNITS", 0))
    if insunits:
        scale = units.conversion_factor(insunits, units.M)
        units_name = units.unit_name(insunits)
    else:
        scale = DEFAULT_SCALE_FOR_UNITLESS
        units_name = "unitless (assumed mm)"

    paths = []
    flatten_layers: List[Optional[str]] = []
    shapes: List[DrawingShape] = []
    for entity in disassemble.recursive_decompose(doc.modelspace()):
        if entity.dxftype() in _SKIPPED_TYPES:
            continue
        exact = _exact_shape(entity)
        if exact is not None:
            shapes.append(exact)
        try:
            paths.append(ezpath.make_path(entity))
        except (TypeError, ValueError):
            continue  # entity type ezdxf can't express as a path
        # Entities with no exact form are kept as flattened shapes below.
        flatten_layers.append(None if exact is not None else entity.dxf.get("layer", "0"))

    bounds = None
    if paths:
        box = ezpath.bbox(paths, fast=True)
        if box.has_data:
            bounds = (box.extmin.x, box.extmin.y, box.extmax.x, box.extmax.y)
    if flatten_tolerance <= 0:
        size = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) if bounds else 1.0
        flatten_tolerance = max(size / 1000.0, 1e-6)

    polylines: List[Polyline] = []
    for p, layer in zip(paths, flatten_layers):
        for sub in p.sub_paths():
            pts = [(v.x, v.y) for v in sub.flattening(flatten_tolerance)]
            if len(pts) >= 2:
                polylines.append(pts)
                if layer is not None:  # no exact form: keep the flattened curve as a shape
                    closed = len(pts) > 2 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < flatten_tolerance
                    shape_pts = pts[:-1] if closed else pts
                    shapes.append(DrawingShape(shape_pts, [0.0] * len(shape_pts), closed, layer))
    return BackgroundGeometry(polylines, scale, units_name, bounds, shapes)


def _exact_shape(entity) -> Optional[DrawingShape]:
    """LINE / ARC / CIRCLE / LWPOLYLINE / 2D POLYLINE as vertices + bulges.
    Entities not in the XY plane (extrusion other than +Z) fall back to
    flattening, so a mirrored arc is never misread."""
    kind = entity.dxftype()
    layer = entity.dxf.get("layer", "0")
    extrusion = entity.dxf.get("extrusion", (0, 0, 1))
    if abs(extrusion[2] - 1.0) > 1e-9:
        return None
    if kind == "LINE":
        s, e = entity.dxf.start, entity.dxf.end
        return DrawingShape([(s.x, s.y), (e.x, e.y)], [0.0, 0.0], False, layer)
    if kind == "ARC":
        c, r = entity.dxf.center, entity.dxf.radius
        a0, a1 = entity.dxf.start_angle, entity.dxf.end_angle
        sweep = (a1 - a0) % 360.0 or 360.0
        p0 = (c.x + r * math.cos(math.radians(a0)), c.y + r * math.sin(math.radians(a0)))
        p1 = (c.x + r * math.cos(math.radians(a1)), c.y + r * math.sin(math.radians(a1)))
        return DrawingShape([p0, p1], [math.tan(math.radians(sweep) / 4.0), 0.0], False, layer)
    if kind == "CIRCLE":
        c, r = entity.dxf.center, entity.dxf.radius
        return DrawingShape([(c.x + r, c.y), (c.x - r, c.y)], [1.0, 1.0], True, layer)
    if kind == "LWPOLYLINE":
        pts = list(entity.get_points("xyb"))
        if len(pts) < 2:
            return None
        return DrawingShape([(float(x), float(y)) for x, y, _ in pts], [float(b) for _, _, b in pts],
                            bool(entity.closed), layer)
    if kind == "POLYLINE" and entity.is_2d_polyline:
        verts = list(entity.vertices)
        if len(verts) < 2:
            return None
        return DrawingShape([(v.dxf.location.x, v.dxf.location.y) for v in verts],
                            [v.dxf.get("bulge", 0.0) for v in verts], bool(entity.is_closed), layer)
    return None


def background_in_metres(geom: BackgroundGeometry, scale: float, offset_x: float, offset_y: float) -> List[Polyline]:
    return [[(x * scale + offset_x, y * scale + offset_y) for x, y in pl] for pl in geom.polylines]


def shapes_in_metres(geom: BackgroundGeometry, scale: float, offset_x: float, offset_y: float) -> List[DrawingShape]:
    """Exact shapes in world metres (uniform scaling keeps every bulge)."""
    return [DrawingShape([(x * scale + offset_x, y * scale + offset_y) for x, y in s.points], list(s.bulges),
                         s.closed, s.layer) for s in (geom.shapes or [])]


def _add_label(msp, text: str, x: float, y: float, height: float = 0.2) -> None:
    msp.add_text(text, height=height, dxfattribs={"layer": "LABELS"}).set_placement((x, y))


def _add_arrow(msp, x: float, y: float, heading: float, size: float) -> None:
    rad = math.radians(heading)
    ux, uy = math.cos(rad), math.sin(rad)
    tip = (x + ux * size / 2, y + uy * size / 2)
    tail = (x - ux * size / 2, y - uy * size / 2)
    left = (tip[0] - ux * size * 0.4 - uy * size * 0.25, tip[1] - uy * size * 0.4 + ux * size * 0.25)
    right = (tip[0] - ux * size * 0.4 + uy * size * 0.25, tip[1] - uy * size * 0.4 - ux * size * 0.25)
    msp.add_lwpolyline([tail, tip], dxfattribs={"layer": "FLOW"})
    msp.add_lwpolyline([left, tip, right], dxfattribs={"layer": "FLOW"})


def export_layout(config: SimulatorConfig, path: str) -> None:
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.M
    for name, (color, description) in LAYERS.items():
        doc.layers.add(name, color=color).description = description
    msp = doc.modelspace()

    for belt in config.belts:
        outline = list(belt_footprint(belt).exterior.coords)
        msp.add_lwpolyline(outline, close=True, dxfattribs={"layer": "BELTS"})
        if belt.type == "path":
            # Native DXF polyline with bulges: exact arcs, editable in any CAD tool.
            pts, bulges = belt_world_path(belt)
            msp.add_lwpolyline([(x, y, b) for (x, y), b in zip(pts, bulges)], format="xyb", close=belt.closed,
                               dxfattribs={"layer": "BELT_CENTERLINES"})
            for hole in belt_footprint(belt).interiors:
                msp.add_lwpolyline(list(hole.coords), close=True, dxfattribs={"layer": "BELTS"})
        elif belt.type == "curved":
            start, end = arc_angles(belt)
            msp.add_arc((belt.x, belt.y), belt.radius, start, end, dxfattribs={"layer": "BELT_CENTERLINES"})
        elif belt_ports(belt) is not None:
            ports = belt_ports(belt)
            msp.add_line((ports.inlet.x, ports.inlet.y), (ports.outlet.x, ports.outlet.y),
                         dxfattribs={"layer": "BELT_CENTERLINES"})
        ports = belt_ports(belt)
        if belt.type == "path" and belt.path:
            line = LineString(centerline_points(belt))
            for frac in (0.25, 0.75) if belt.closed else (0.5,):
                a, b = line.interpolate(frac - 0.005, normalized=True), line.interpolate(frac + 0.005, normalized=True)
                heading = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
                if path_flow_sign(belt) < 0:
                    heading += 180.0
                mid = line.interpolate(frac, normalized=True)
                _add_arrow(msp, mid.x, mid.y, heading, min(belt.beltWidth, 0.6))
        elif ports is not None:
            if belt.type == "curved":
                start, end = arc_angles(belt)
                mid_deg = (start + end) / 2.0
                mx = belt.x + belt.radius * math.cos(math.radians(mid_deg))
                my = belt.y + belt.radius * math.sin(math.radians(mid_deg))
                heading = mid_deg + (90.0 if curved_ccw(belt) else -90.0)
            else:
                mx, my, heading = belt.x, belt.y, ports.outlet.heading
            _add_arrow(msp, mx, my, heading, min(belt.beltWidth, 0.6))
        _add_label(msp, belt.id, belt.x, belt.y)

    for sensor in config.sensors:
        verts = get_rect_vertices(sensor.x, sensor.y, sensor.width, sensor.height, sensor.rotation)
        msp.add_lwpolyline(verts, close=True, dxfattribs={"layer": "SENSORS"})
        _add_label(msp, sensor.id, sensor.x, sensor.y, 0.15)
    for source in config.sources:
        w, h = source.maxWidth, source.maxHeight
        msp.add_lwpolyline(get_rect_vertices(source.x, source.y, w, h, source.rotation), close=True,
                           dxfattribs={"layer": "SOURCES"})
        _add_label(msp, source.id, source.x, source.y, 0.15)
    for sink in config.sinks:
        msp.add_lwpolyline(get_rect_vertices(sink.x, sink.y, sink.width, sink.height, 0), close=True,
                           dxfattribs={"layer": "SINKS"})
        _add_label(msp, sink.id, sink.x, sink.y, 0.15)
    for button in config.buttons:
        msp.add_lwpolyline(get_rect_vertices(button.x, button.y, button.width, button.height, button.rotation),
                           close=True, dxfattribs={"layer": "IO"})
        _add_label(msp, button.id, button.x, button.y, 0.1)
    for led in config.leds:
        msp.add_circle((led.x, led.y), led.radius, dxfattribs={"layer": "IO"})
        _add_label(msp, led.id, led.x, led.y, 0.1)

    for entity in config.sketch:
        msp.add_lwpolyline([(x, y, b) for (x, y), b in zip(entity.points, list(entity.bulges) + [0.0] * len(entity.points))],
                           format="xyb", close=entity.closed, dxfattribs={"layer": "SKETCH"})

    belts = {b.id: b for b in config.belts}
    for conn in config.connections:
        a = belts.get(conn.fromBelt)
        ports = belt_ports(a) if a is not None else None
        if ports is not None:
            msp.add_circle((ports.outlet.x, ports.outlet.y), 0.1, dxfattribs={"layer": "CONNECTIONS"})

    doc.saveas(path)
