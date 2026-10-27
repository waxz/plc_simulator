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
from cad.ports import arc_angles, belt_ports, curved_ccw
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
}


@dataclass
class BackgroundGeometry:
    polylines: List[Polyline]          # drawing units
    scale: float                       # metres per drawing unit
    units_name: str                    # e.g. "mm", or "unitless (assumed mm)"
    bounds: Optional[Tuple[float, float, float, float]]  # min_x, min_y, max_x, max_y, drawing units


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
    for entity in disassemble.recursive_decompose(doc.modelspace()):
        if entity.dxftype() in _SKIPPED_TYPES:
            continue
        try:
            paths.append(ezpath.make_path(entity))
        except (TypeError, ValueError):
            continue  # entity type ezdxf can't express as a path

    bounds = None
    if paths:
        box = ezpath.bbox(paths, fast=True)
        if box.has_data:
            bounds = (box.extmin.x, box.extmin.y, box.extmax.x, box.extmax.y)
    if flatten_tolerance <= 0:
        size = max(bounds[2] - bounds[0], bounds[3] - bounds[1]) if bounds else 1.0
        flatten_tolerance = max(size / 1000.0, 1e-6)

    polylines: List[Polyline] = []
    for p in paths:
        for sub in p.sub_paths():
            pts = [(v.x, v.y) for v in sub.flattening(flatten_tolerance)]
            if len(pts) >= 2:
                polylines.append(pts)
    return BackgroundGeometry(polylines, scale, units_name, bounds)


def background_in_metres(geom: BackgroundGeometry, scale: float, offset_x: float, offset_y: float) -> List[Polyline]:
    return [[(x * scale + offset_x, y * scale + offset_y) for x, y in pl] for pl in geom.polylines]


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
        if belt.type == "curved":
            start, end = arc_angles(belt)
            msp.add_arc((belt.x, belt.y), belt.radius, start, end, dxfattribs={"layer": "BELT_CENTERLINES"})
        elif belt_ports(belt) is not None:
            ports = belt_ports(belt)
            msp.add_line((ports.inlet.x, ports.inlet.y), (ports.outlet.x, ports.outlet.y),
                         dxfattribs={"layer": "BELT_CENTERLINES"})
        ports = belt_ports(belt)
        if ports is not None:
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

    belts = {b.id: b for b in config.belts}
    for conn in config.connections:
        a = belts.get(conn.fromBelt)
        ports = belt_ports(a) if a is not None else None
        if ports is not None:
            msp.add_circle((ports.outlet.x, ports.outlet.y), 0.1, dxfattribs={"layer": "CONNECTIONS"})

    doc.saveas(path)
