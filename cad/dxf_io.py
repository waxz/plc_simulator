"""
cad/dxf_io.py - DXF in and out, with ezdxf.

read_background(path)  - flatten a DXF floor plan into polylines (drawing
                         units) for the editor's locked background layer.
                         Block references are expanded; text is skipped.
                         Optionally only a clip window, without some
                         entity types (dimensions, hatches, leaders) or
                         layers.
inspect_drawing(path)  - what the import dialog needs: header units checked
                         against the dimensions, layers, paper sheets found
                         in modelspace, and a quick preview.
export_layout(config, path) - write the layout as a DXF in metres, one layer
                         per component kind, for CAD / documentation; with
                         the background drawing (as placed) on BACKGROUND.
export_cropped(src, dst, clip, ...) - write one window of a big drawing (with
                         the same filters as the background) as a small DXF:
                         blocks expanded, edges cut at the window, same units,
                         layers and coordinates, texts inside kept.
"""
import math
import os
import re
import statistics
from collections import Counter, OrderedDict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple

import ezdxf
from ezdxf import bbox as ezbbox, path as ezpath, units
from ezdxf.disassemble import virtual_entities
from ezdxf.protocols import SupportsVirtualEntities

from config_parser import SimulatorConfig
from cad.checks import belt_footprint
from shapely.geometry import LineString, box as shapely_box

from cad.path import belt_world_path, path_flow_sign
from cad.ports import arc_angles, belt_ports, centerline_points, curved_ccw
from math_utils import get_rect_vertices

Polyline = List[Tuple[float, float]]
Box = Tuple[float, float, float, float]  # min_x, min_y, max_x, max_y

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
    "BACKGROUND": (8, "the imported floor plan, as placed in the layout"),
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


def read_background(path: str, flatten_tolerance: float = 0.0, *, clip: Optional[Box] = None,
                    skip_types: Iterable[str] = (), hidden_layers: Iterable[str] = ()) -> BackgroundGeometry:
    """Polylines of every drawable entity in modelspace. `flatten_tolerance`
    is the max chord error for curves in drawing units (0 = 1/1000 of the
    drawing size, or of the clip window).

    clip          - (min_x, min_y, max_x, max_y) drawing units: keep only what
                    lies inside; entities crossing the edge are cut at it
                    (as straight pieces). Block references wholly outside
                    are never expanded, which is what makes one sheet of a
                    big drawing set fast.
    skip_types    - option names from SKIP_OPTIONS ("DIMENSION", "HATCH",
                    "LEADER") or raw DXF types to leave out.
    hidden_layers - layers to leave out. Entities on layer "0" inside a block
                    take the layer of the block reference (as in AutoCAD).
    """
    cached = _load(path)
    doc = cached.doc
    scale, units_name = header_scale(doc)
    skip = _expand_skip(skip_types)
    hidden = set(hidden_layers)
    clip = _norm_box(clip) if clip is not None else None
    if flatten_tolerance <= 0 and clip is not None:
        flatten_tolerance = max(max(clip[2] - clip[0], clip[3] - clip[1]) / 1000.0, 1e-6)

    paths = []
    flatten_layers: List[Optional[str]] = []
    shapes: List[DrawingShape] = []
    crossing = []  # (path, layer): cut at the clip edge once the tolerance is known
    for entity, layer, entity_clip in _walk(cached, doc.modelspace(), skip, hidden, None, clip):
        try:
            p = ezpath.make_path(entity)
        except (TypeError, ValueError):
            p = None  # entity type ezdxf can't express as a path
        if entity_clip is not None:
            state = _classify(_path_box(p), entity_clip) if p is not None else _OUTSIDE
            if state == _OUTSIDE:
                continue
            if state == _CROSSING:
                crossing.append((p, layer))
                continue
        exact = _exact_shape(entity)
        if exact is not None:
            exact.layer = layer
            shapes.append(exact)
        if p is None:
            continue
        paths.append(p)
        # Entities with no exact form are kept as flattened shapes below.
        flatten_layers.append(None if exact is not None else layer)

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
    if crossing:
        window = shapely_box(*clip)
        for p, layer in crossing:
            for sub in p.sub_paths():
                pts = [(v.x, v.y) for v in sub.flattening(flatten_tolerance)]
                if len(pts) < 2:
                    continue
                for piece in _line_pieces(LineString(pts).intersection(window)):
                    polylines.append(piece)
                    shapes.append(DrawingShape(piece, [0.0] * len(piece), False, layer))
    if clip is not None:
        kept = [b for b in (bounds,) if b is not None]
        if crossing and polylines:
            xs = [x for pl in polylines for x, _ in pl]
            ys = [y for pl in polylines for _, y in pl]
            kept.append((min(xs), min(ys), max(xs), max(ys)))
        bounds = (min(b[0] for b in kept), min(b[1] for b in kept),
                  max(b[2] for b in kept), max(b[3] for b in kept)) if kept else None
    return BackgroundGeometry(polylines, scale, units_name, bounds, shapes)


def _line_pieces(geom) -> List[Polyline]:
    if geom.is_empty:
        return []
    if geom.geom_type == "LineString":
        return [list(geom.coords)] if len(geom.coords) >= 2 else []
    if hasattr(geom, "geoms"):
        return [pc for g in geom.geoms for pc in _line_pieces(g)]
    return []  # a Point where a line only touches the window


# ---------------------------------------------------------------------------
# Walking modelspace with block expansion, layer filter and clip window
# ---------------------------------------------------------------------------

_OUTSIDE, _CROSSING, _INSIDE = 0, 1, 2

# Import options: name -> DXF types it covers.
SKIP_OPTIONS = {
    "DIMENSION": ("DIMENSION", "ARC_DIMENSION", "LARGE_RADIAL_DIMENSION"),
    "HATCH": ("HATCH", "MPOLYGON", "SOLID", "TRACE"),
    "LEADER": ("LEADER", "MULTILEADER", "MLEADER"),
}


def _expand_skip(skip_types: Iterable[str]) -> Set[str]:
    out: Set[str] = set()
    for name in skip_types:
        out.update(SKIP_OPTIONS.get(name.upper(), (name.upper(),)))
    return out


def _norm_box(box) -> Box:
    x0, y0, x1, y1 = (float(v) for v in box)
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def _classify(box: Optional[Box], clip: Box) -> int:
    if box is None:
        return _OUTSIDE
    if box[2] < clip[0] or box[0] > clip[2] or box[3] < clip[1] or box[1] > clip[3]:
        return _OUTSIDE
    eps = 1e-9 * max(1.0, abs(clip[0]), abs(clip[1]), abs(clip[2]), abs(clip[3]))
    if (box[0] >= clip[0] - eps and box[2] <= clip[2] + eps
            and box[1] >= clip[1] - eps and box[3] <= clip[3] + eps):
        return _INSIDE
    return _CROSSING


def _path_box(p) -> Optional[Box]:
    """Bounding box of a path's control points: never smaller than the curve."""
    pts = p.control_vertices()
    if not pts:
        return None
    xs = [v.x for v in pts]
    ys = [v.y for v in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def _union(a: Optional[Box], b: Optional[Box]) -> Optional[Box]:
    if a is None:
        return b
    if b is None:
        return a
    return (min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3]))


def _transform_box(box: Optional[Box], matrix) -> Optional[Box]:
    if box is None:
        return None
    corners = list(matrix.transform_vertices([(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]))
    xs = [v.x for v in corners]
    ys = [v.y for v in corners]
    return (min(xs), min(ys), max(xs), max(ys))


def _entity_box(entity) -> Optional[Box]:
    """Cheap bounding box of one non-INSERT entity (drawing units)."""
    kind = entity.dxftype()
    try:
        if kind == "LINE":
            s, e = entity.dxf.start, entity.dxf.end
            return (min(s.x, e.x), min(s.y, e.y), max(s.x, e.x), max(s.y, e.y))
        if kind in ("ARC", "CIRCLE") and abs(entity.dxf.get("extrusion", (0, 0, 1))[2] - 1.0) < 1e-9:
            c, r = entity.dxf.center, entity.dxf.radius
            return (c.x - r, c.y - r, c.x + r, c.y + r)
        if kind == "LWPOLYLINE" and not entity.has_arc:
            pts = entity.get_points("xy")
            if not pts:
                return None
            xs = [x for x, _ in pts]
            ys = [y for _, y in pts]
            return (min(xs), min(ys), max(xs), max(ys))
        box = ezbbox.extents([entity], fast=True)
    except Exception:  # noqa: BLE001 - odd entity: unknown extent
        return None
    return (box.extmin.x, box.extmin.y, box.extmax.x, box.extmax.y) if box.has_data else None


def _inserts(entity):
    """An INSERT, or each grid cell of a MINSERT."""
    return list(entity.multi_insert()) if entity.mcount > 1 else [entity]


class _CachedDrawing:
    """A loaded DXF plus per-block extents / layer counts, computed once."""

    def __init__(self, doc):
        self.doc = doc
        self._block_box: Dict[str, Optional[Box]] = {}
        self._block_layers: Dict[str, Counter] = {}

    def block_box(self, name: str) -> Optional[Box]:
        """Extent of a block definition in its own coordinates."""
        if name in self._block_box:
            return self._block_box[name]
        self._block_box[name] = None  # guards against self-referencing blocks
        block = self.doc.blocks.get(name)
        box = None
        if block is not None:
            for e in block:
                kind = e.dxftype()
                if kind in _SKIPPED_TYPES:
                    continue
                if kind == "INSERT":
                    for ins in _inserts(e):
                        box = _union(box, _transform_box(self.block_box(ins.dxf.name), ins.matrix44()))
                else:
                    box = _union(box, _entity_box(e))
        self._block_box[name] = box
        return box

    def insert_box(self, insert) -> Optional[Box]:
        try:
            return _transform_box(self.block_box(insert.dxf.name), insert.matrix44())
        except Exception:  # noqa: BLE001 - degenerate transform: treat as unknown
            return None

    def block_layers(self, name: str) -> Counter:
        """Entity count per layer inside a block, nested blocks included;
        "0" means "the layer of the block reference"."""
        if name in self._block_layers:
            return self._block_layers[name]
        self._block_layers[name] = Counter()
        counts: Counter = Counter()
        block = self.doc.blocks.get(name)
        if block is not None:
            for e in block:
                kind = e.dxftype()
                if kind in _SKIPPED_TYPES:
                    continue
                layer = e.dxf.get("layer", "0")
                if kind == "INSERT":
                    _merge_layers(counts, self.block_layers(e.dxf.name), layer, max(1, e.mcount))
                else:
                    counts[layer] += 1
        self._block_layers[name] = counts
        return counts


def _merge_layers(into: Counter, block_counts: Counter, insert_layer: str, times: int = 1) -> None:
    for layer, n in block_counts.items():
        into[insert_layer if layer == "0" else layer] += n * times


def _walk(cached: _CachedDrawing, entities, skip: Set[str], hidden: Set[str], parent_layer: Optional[str],
          clip: Optional[Box]):
    """Yield (entity, effective layer, clip-or-None) for every primitive,
    expanding block references, dimensions, leaders, ... `clip` is None for
    entities known to be inside the window (or when there is no window)."""
    for entity in entities:
        kind = entity.dxftype()
        if kind in _SKIPPED_TYPES or kind in skip:
            continue
        layer = entity.dxf.get("layer", "0")
        if layer == "0" and parent_layer is not None:
            layer = parent_layer
        if layer in hidden:
            continue
        if kind == "INSERT":
            for ins in _inserts(entity):
                sub_clip = clip
                if clip is not None:
                    box = cached.insert_box(ins)
                    state = _classify(box, clip) if box is not None else _CROSSING  # unknown: look inside
                    if state == _OUTSIDE:
                        continue  # never expanded: the big saving for a clipped sheet
                    if state == _INSIDE:
                        sub_clip = None
                try:
                    children = list(ins.virtual_entities())
                except Exception:  # noqa: BLE001 - broken block reference
                    continue
                yield from _walk(cached, children, skip, hidden, layer, sub_clip)
        elif isinstance(entity, SupportsVirtualEntities):  # DIMENSION, LEADER, MLEADER, MLINE, ...
            try:
                children = list(virtual_entities(entity))
            except Exception:  # noqa: BLE001
                continue
            yield from _walk(cached, children, skip, hidden, layer, clip)
        else:
            yield entity, layer, clip


_CACHE: "OrderedDict[tuple, _CachedDrawing]" = OrderedDict()


def _load(path: str) -> _CachedDrawing:
    """ezdxf.readfile, remembered for the last couple of files (a 20 MB DXF
    takes seconds to parse; the import dialog, the import itself and every
    undo/redo of it read the same file)."""
    full = os.path.abspath(path)
    st = os.stat(full)
    key = (full, st.st_mtime_ns, st.st_size)
    if key in _CACHE:
        _CACHE.move_to_end(key)
        return _CACHE[key]
    cached = _CachedDrawing(ezdxf.readfile(full))
    _CACHE[key] = cached
    while len(_CACHE) > 2:
        _CACHE.popitem(last=False)
    return cached


# ---------------------------------------------------------------------------
# Units
# ---------------------------------------------------------------------------

# Choices of the import dialog: label -> metres per drawing unit
# (None = unitless: the scale is typed in).
UNIT_CHOICES = [("m", 1.0), ("cm", 0.01), ("mm", 0.001), ("inch", 0.0254), ("ft", 0.3048), ("unitless", None)]
_INSUNITS_LABEL = {units.M: "m", units.CM: "cm", units.MM: "mm", units.IN: "inch", units.FT: "ft"}


def header_scale(doc) -> Tuple[float, str]:
    """(metres per unit, units name) from $INSUNITS."""
    insunits = int(doc.header.get("$INSUNITS", 0))
    if insunits:
        try:
            return units.conversion_factor(insunits, units.M), units.unit_name(insunits)
        except (KeyError, ValueError, TypeError):
            pass
    return DEFAULT_SCALE_FOR_UNITLESS, "unitless (assumed mm)"


def unit_label_for_scale(scale: float) -> str:
    for label, value in UNIT_CHOICES:
        if value is not None and abs(value - scale) <= 1e-9 * max(1.0, value):
            return label
    return "unitless"


# DIMLFAC values that mean "drawing unit X, dimension text in unit Y".
_DIMLFAC_UNITS = [
    (100.0, "m", "cm"), (1000.0, "m", "mm"), (10.0, "cm", "mm"), (25.4, "inch", "mm"), (12.0, "ft", "inch"),
    (304.8, "ft", "mm"), (39.3701, "m", "inch"), (2.54, "inch", "cm"), (0.1, "mm", "cm"), (0.001, "mm", "m"),
    (0.01, "cm", "m"), (1 / 12.0, "inch", "ft"),
]
# A plant layout's typical dimension (median) lies in this range, in metres.
_PLAUSIBLE_DIMENSION_M = (0.2, 200.0)


@dataclass
class UnitsHint:
    unit: str              # suggested UNIT_CHOICES label
    scale: float           # its metres per unit
    message: str
    warn: bool             # True when the suggestion differs from the header


def units_hint(doc, header_unit: str, max_dimensions: int = 300) -> Optional[UnitsHint]:
    """Cross-check the header units with the drawing's DIMENSION entities:
    a dimension shows (measured length x DIMLFAC), so DIMLFAC 100 with a
    measured 0.85 shown as "85" means metres drawn, centimetres shown.
    Without a DIMLFAC conversion the median dimension must be a plausible
    plant-layout size. None when the drawing has no usable dimensions."""
    measured, factors = [], []
    for dim in doc.modelspace().query("DIMENSION"):
        if (dim.dxf.get("dimtype", 0) & 7) not in (0, 1, 3, 4):  # linear, aligned, diameter, radius
            continue
        try:
            value = float(dim.get_measurement())
            factor = abs(float(dim.override().get("dimlfac", 1.0) or 1.0))
        except Exception:  # noqa: BLE001 - dimension without its geometry
            continue
        if value > 0 and math.isfinite(value):
            measured.append(value)
            factors.append(factor)
        if len(measured) >= max_dimensions:
            break
    if not measured:
        return None
    m = statistics.median(measured)
    f = statistics.median(factors)
    scales = dict((label, s) for label, s in UNIT_CHOICES if s is not None)
    lo, hi = _PLAUSIBLE_DIMENSION_M

    def plausible(unit):
        return lo <= m * scales[unit] <= hi

    sample = f"{len(measured)} dimension(s), median {m:.4g} units"
    if abs(f - 1.0) > 1e-6:
        matches = [(u, shown) for factor, u, shown in _DIMLFAC_UNITS if abs(f - factor) <= 0.01 * factor]
        if matches:
            unit, shown = next(((u, s) for u, s in matches if plausible(u)), matches[0])
            reason = (f"{sample} shown x{f:g} (DIMLFAC) = {m * f:.4g}: "
                      f"looks like 1 unit = 1 {unit}, dimensions in {shown}")
            return _hint(unit, scales[unit], header_unit, reason)
    if header_unit in scales and plausible(header_unit):
        return UnitsHint(header_unit, scales[header_unit],
                         f"{sample} = {m * scales[header_unit]:.3g} m: plausible.", False)
    # Without DIMLFAC the unit is a guess: the first plausible one, metric first.
    fits = [u for u in ("m", "mm", "cm", "ft", "inch") if plausible(u)]
    best = fits[0] if fits else min(scales, key=lambda u: abs(math.log10(m * scales[u])))
    reason = f"{sample}: as {header_unit} that is {m * scales.get(header_unit, 0):.3g} m" \
        if header_unit in scales else sample
    return _hint(best, scales[best], header_unit, reason + f"; as {best} it is {m * scales[best]:.3g} m")


def _hint(unit: str, scale: float, header_unit: str, reason: str) -> UnitsHint:
    if unit == header_unit:
        return UnitsHint(unit, scale, reason + " - agrees with the header.", False)
    return UnitsHint(unit, scale, f"Header says {header_unit}, but {reason}.", True)


# ---------------------------------------------------------------------------
# Inspecting a drawing for the import dialog
# ---------------------------------------------------------------------------

@dataclass
class Sheet:
    name: str
    box: Box  # drawing units


@dataclass
class DrawingInfo:
    path: str
    header_unit: str                 # UNIT_CHOICES label of $INSUNITS
    header_units_name: str           # e.g. "Inches"
    header_scale: float
    extents: Optional[Box]           # drawing units, measured (not $EXTMIN/$EXTMAX)
    layers: Dict[str, int]           # effective layer -> entity count (blocks expanded)
    off_layers: List[str]            # off / frozen / non-plotting in the DXF
    sheets: List[Sheet]
    hint: Optional[UnitsHint]
    preview: List[Polyline]          # quick, coarse outline for picking a window (drawing units)
    shape_estimate: int              # entities after block expansion


_SHEET_ASPECT = (1.2, 1.8)        # paper: ISO 1.41, ANSI 1.29 / 1.55, with title strips
_SHEET_MIN_FRACTION = 0.03         # of the drawing's larger side
_PREVIEW_BLOCK_LIMIT = 2000        # bigger blocks are previewed as their outline box
_SHEET_TITLE = re.compile(r"PLAN|LAYOUT|ELEVATION|SECTION|PERSPECTIVE|DETAIL|VIEW|SHEET", re.I)


def inspect_drawing(path: str) -> DrawingInfo:
    """Everything the import dialog shows: header units with a dimension
    cross-check, layers with entity counts, sheets (paper frames) found in
    modelspace with their titles, and a coarse preview."""
    cached = _load(path)
    doc = cached.doc
    scale, units_name = header_scale(doc)
    insunits = int(doc.header.get("$INSUNITS", 0))
    header_unit = _INSUNITS_LABEL.get(insunits, unit_label_for_scale(scale) if insunits else "unitless")

    layers: Counter = Counter()
    extents = None
    raw = []          # (entity, box) of top-level primitives, for preview and sheets
    texts = []        # (text, x, y, height)
    for e in doc.modelspace():
        kind = e.dxftype()
        layer = e.dxf.get("layer", "0")
        if kind in ("TEXT", "MTEXT"):
            try:
                text = e.dxf.text if kind == "TEXT" else e.plain_text()
                p = e.dxf.insert
                texts.append((text, p.x, p.y, float(e.dxf.get("height" if kind == "TEXT" else "char_height", 0) or 0)))
            except Exception:  # noqa: BLE001
                pass
            continue
        if kind in _SKIPPED_TYPES:
            continue
        if kind == "INSERT":
            for ins in _inserts(e):
                _merge_layers(layers, cached.block_layers(ins.dxf.name), layer)
                box = cached.insert_box(ins)
                extents = _union(extents, box)
                raw.append((ins, box))
            continue
        layers[layer] += 1
        box = _entity_box(e)
        extents = _union(extents, box)
        raw.append((e, box))

    off = [l.dxf.name for l in doc.layers if l.is_off() or l.is_frozen() or not l.dxf.get("plot", 1)]
    off += [name for name in layers if name.lower() == "defpoints" and name not in off]
    try:
        hint = units_hint(doc, header_unit)
    except Exception:  # noqa: BLE001 - a hint is optional
        hint = None
    return DrawingInfo(
        path=path, header_unit=header_unit, header_units_name=units_name, header_scale=scale, extents=extents,
        layers=dict(sorted(layers.items(), key=lambda kv: kv[0].lower())), off_layers=sorted(off),
        sheets=find_sheets(raw, texts, extents) or find_groups(raw, extents), hint=hint,
        preview=_preview(cached, raw, extents), shape_estimate=sum(layers.values()))


def _rectangle(entity) -> Optional[Box]:
    """Box of an axis-aligned closed rectangle (LWPOLYLINE / 2D POLYLINE)."""
    kind = entity.dxftype()
    if kind == "LWPOLYLINE":
        if entity.has_arc:
            return None
        pts = [(x, y) for x, y in entity.get_points("xy")]
        closed = entity.closed
    elif kind == "POLYLINE" and entity.is_2d_polyline:
        pts = [(v.dxf.location.x, v.dxf.location.y) for v in entity.vertices]
        closed = entity.is_closed
    else:
        return None
    if len(pts) == 5 and math.isclose(pts[0][0], pts[-1][0]) and math.isclose(pts[0][1], pts[-1][1]):
        pts, closed = pts[:4], True
    if len(pts) != 4 or not closed:
        return None
    xs = sorted(x for x, _ in pts)
    ys = sorted(y for _, y in pts)
    w, h = xs[-1] - xs[0], ys[-1] - ys[0]
    tol = 1e-6 * max(w, h, 1e-12)
    if w <= 0 or h <= 0 or xs[1] - xs[0] > tol or xs[3] - xs[2] > tol or ys[1] - ys[0] > tol or ys[3] - ys[2] > tol:
        return None
    return (xs[0], ys[0], xs[-1], ys[-1])


def find_sheets(raw, texts, extents: Optional[Box]) -> List[Sheet]:
    """Paper frames in modelspace: axis-aligned closed rectangles with a
    paper-like aspect, not inside a bigger frame; named after the most
    title-like text inside. Top-to-bottom, left-to-right."""
    if extents is None:
        return []
    min_side = _SHEET_MIN_FRACTION * max(extents[2] - extents[0], extents[3] - extents[1])
    frames = []
    for entity, _ in raw:
        box = _rectangle(entity)
        if box is None:
            continue
        w, h = box[2] - box[0], box[3] - box[1]
        if min(w, h) < min_side or not _SHEET_ASPECT[0] <= max(w, h) / min(w, h) <= _SHEET_ASPECT[1]:
            continue
        frames.append(box)
    frames.sort(key=lambda b: -(b[2] - b[0]) * (b[3] - b[1]))
    sheets: List[Box] = []
    for box in frames:
        margin = 0.02 * max(box[2] - box[0], box[3] - box[1])
        if any(box[0] >= s[0] - margin and box[2] <= s[2] + margin and box[1] >= s[1] - margin
               and box[3] <= s[3] + margin for s in sheets):
            continue  # an inner border of a sheet already found
        sheets.append(box)
    if len(sheets) == 1 and sheets[0] == extents:
        return []  # just the drawing's own outline
    sheets.sort(key=lambda b: (-round(b[3] / max(min_side, 1e-12)), b[0]))
    out = []
    for i, box in enumerate(sheets, 1):
        inside = [t for t in texts if box[0] <= t[1] <= box[2] and box[1] <= t[2] <= box[3] and t[0].strip()]
        titled = [t for t in inside if _SHEET_TITLE.search(t[0])] or inside
        if titled:
            best = max(titled, key=lambda t: (t[3], -len(t[0])))
            name = best[0].strip().lstrip("#").strip().splitlines()[0].strip()
        else:
            name = ""
        out.append(Sheet(f"{i}: {name}" if name else f"Sheet {i}", box))
    return out


_GROUP_GAP_FRACTION = 0.05  # empty band, of the drawing's larger side, that separates two groups


def find_groups(raw, extents: Optional[Box]) -> List[Sheet]:
    """Without paper frames: clusters of geometry separated by wide empty
    bands (recursive XY cut of the top-level entity boxes), biggest first.
    One cluster = nothing to choose, so []."""
    boxes = [b for _, b in raw if b is not None]
    if extents is None or not boxes:
        return []
    min_gap = _GROUP_GAP_FRACTION * max(extents[2] - extents[0], extents[3] - extents[1])

    def split(items, depth=0):
        if depth > 12 or len(items) < 2:
            return [items]
        best = None
        for axis in (0, 1):
            ordered = sorted(items, key=lambda b: b[axis])
            reach = ordered[0][axis + 2]
            for i in range(1, len(ordered)):
                gap = ordered[i][axis] - reach
                if gap > min_gap and (best is None or gap > best[0]):
                    best = (gap, ordered[:i], ordered[i:])
                reach = max(reach, ordered[i][axis + 2])
        if best is None:
            return [items]
        return split(best[1], depth + 1) + split(best[2], depth + 1)

    groups = split(boxes)
    if len(groups) < 2:
        return []
    groups.sort(key=len, reverse=True)
    out = []
    for i, items in enumerate(groups, 1):
        box = (min(b[0] for b in items), min(b[1] for b in items), max(b[2] for b in items), max(b[3] for b in items))
        out.append(Sheet(f"Group {i}: {len(items)} entities, {box[2] - box[0]:.4g} x {box[3] - box[1]:.4g} units", box))
    return out


def _preview(cached: _CachedDrawing, raw, extents: Optional[Box]) -> List[Polyline]:
    if extents is None:
        return []
    tol = max(max(extents[2] - extents[0], extents[3] - extents[1]) / 3000.0, 1e-9)
    skip = _expand_skip(SKIP_OPTIONS)
    out: List[Polyline] = []

    def add(entity):
        try:
            p = ezpath.make_path(entity)
        except (TypeError, ValueError):
            return
        for sub in p.sub_paths():
            pts = [(v.x, v.y) for v in sub.flattening(tol)]
            if len(pts) >= 2:
                out.append(pts)

    for entity, box in raw:
        kind = entity.dxftype()
        if kind in skip:
            continue
        if kind == "INSERT":
            if sum(cached.block_layers(entity.dxf.name).values()) > _PREVIEW_BLOCK_LIMIT:
                if box is not None:
                    out.append([(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3]), (box[0], box[1])])
                continue
            for e, _, _ in _walk(cached, [entity], skip, set(), None, None):
                add(e)
        elif isinstance(entity, SupportsVirtualEntities):
            continue
        else:
            add(entity)
    return out


def default_origin(bounds: Optional[Box], scale: float) -> Tuple[float, float]:
    """Offsets (metres) that put the lower-left of `bounds` at (0, 0)."""
    if bounds is None:
        return 0.0, 0.0
    return -bounds[0] * scale, -bounds[1] * scale


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


def _curved_outline(msp, belt) -> None:
    """A curved belt's outline with true arcs (outer arc, radial edge, inner
    arc back, radial edge), so CAD tools - and a reimport - see arcs, not a
    flattened polygon. A full ring is two circles; a disc / sector with no
    inner radius closes at the center."""
    start, end = arc_angles(belt)
    sweep = end - start
    r_out = belt.radius + belt.beltWidth / 2.0
    r_in = max(0.0, belt.radius - belt.beltWidth / 2.0)
    cx, cy = belt.x, belt.y
    attribs = {"layer": "BELTS"}

    def at(r, deg):
        return cx + r * math.cos(math.radians(deg)), cy + r * math.sin(math.radians(deg))
    if sweep >= 360.0 - 1e-9:
        for r in (r_out, r_in):
            if r > 1e-9:
                msp.add_circle((cx, cy), r, dxfattribs=attribs)
        return
    b = math.tan(math.radians(sweep) / 4.0)
    if r_in <= 1e-9:
        pts = [(*at(r_out, start), b), (*at(r_out, end), 0.0), (cx, cy, 0.0)]
    else:
        pts = [(*at(r_out, start), b), (*at(r_out, end), 0.0), (*at(r_in, end), -b), (*at(r_in, start), 0.0)]
    msp.add_lwpolyline(pts, format="xyb", close=True, dxfattribs=attribs)


def export_layout(config: SimulatorConfig, path: str, background: Iterable[DrawingShape] = ()) -> None:
    """`background`: the background drawing's shapes in layout metres (as
    the editor shows them: window, filters, scale and offset applied)."""
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.M
    for name, (color, description) in LAYERS.items():
        doc.layers.add(name, color=color).description = description
    msp = doc.modelspace()

    for shape in background:
        bulges = list(shape.bulges) + [0.0] * (len(shape.points) - len(shape.bulges))
        msp.add_lwpolyline([(x, y, b) for (x, y), b in zip(shape.points, bulges)], format="xyb",
                           close=shape.closed, dxfattribs={"layer": "BACKGROUND"})

    for belt in config.belts:
        if belt.type == "curved":
            _curved_outline(msp, belt)
        else:
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


@dataclass
class CropResult:
    path: str
    shapes: int            # geometry entities written
    texts: int             # TEXT / MTEXT kept
    bounds: Optional[Box]  # of what was written, drawing units


def _text_point(entity) -> Optional[Tuple[float, float]]:
    try:
        p = entity.dxf.insert
    except AttributeError:
        return None
    return (p.x, p.y)


def export_cropped(src: str, dst: str, clip: Optional[Box] = None, *, skip_types: Iterable[str] = (),
                   hidden_layers: Iterable[str] = (), keep_text: bool = True,
                   to_origin: bool = False) -> CropResult:
    """Write the part of `src` inside `clip` (drawing units; None = whole
    drawing) to `dst` - what the import dialog shows for that window and
    filters, as a DXF a CAD tool or this editor opens in a fraction of the
    time. Block references are expanded, entities crossing the window are
    cut at its edge, lines / arcs / circles / polylines keep their true arcs.
    Units ($INSUNITS) and layer names / colours carry over; coordinates stay
    those of the original unless `to_origin` moves the window's lower-left to
    (0, 0). Top-level TEXT / MTEXT whose insert point is inside are copied
    when `keep_text`."""
    geom = read_background(src, clip=clip, skip_types=skip_types, hidden_layers=hidden_layers)
    source = _load(src).doc
    window = _norm_box(clip) if clip is not None else None
    dx, dy = 0.0, 0.0
    if to_origin:
        anchor = window or geom.bounds
        if anchor is not None:
            dx, dy = -anchor[0], -anchor[1]

    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = int(source.header.get("$INSUNITS", 0))
    hidden = set(hidden_layers)

    def layer(name: str) -> str:
        if name not in doc.layers:
            attribs = {}
            if name in source.layers:
                color = abs(int(source.layers.get(name).dxf.get("color", 7)))  # negative = layer off
                attribs = {"color": color if 1 <= color <= 255 else 7}
            doc.layers.add(name, **attribs)
        return name

    msp = doc.modelspace()
    for shape in geom.shapes or []:
        points = [(x + dx, y + dy, b) for (x, y), b in zip(shape.points, shape.bulges)]
        msp.add_lwpolyline(points, format="xyb", close=shape.closed, dxfattribs={"layer": layer(shape.layer)})

    texts = 0
    if keep_text:
        for entity in source.modelspace().query("TEXT MTEXT"):
            name = entity.dxf.get("layer", "0")
            point = _text_point(entity)
            if name in hidden or point is None:
                continue
            if window is not None and not (window[0] <= point[0] <= window[2] and window[1] <= point[1] <= window[3]):
                continue
            # Rebuilt from its attributes, not copied: a copy drags along
            # AutoCAD-only extension objects ezdxf can't carry over.
            attribs = {"layer": layer(name), "insert": (point[0] + dx, point[1] + dy),
                       "rotation": entity.dxf.get("rotation", 0.0)}
            if entity.dxftype() == "TEXT":
                attribs["height"] = entity.dxf.get("height", 1.0)
                msp.add_text(entity.dxf.get("text", ""), dxfattribs=attribs)
            else:
                attribs.update(char_height=entity.dxf.get("char_height", 1.0),
                               attachment_point=entity.dxf.get("attachment_point", 1))
                if entity.dxf.hasattr("width"):
                    attribs["width"] = entity.dxf.width
                msp.add_mtext(entity.text, dxfattribs=attribs)
            texts += 1

    os.makedirs(os.path.dirname(os.path.abspath(dst)), exist_ok=True)
    doc.saveas(dst)
    bounds = geom.bounds
    if bounds is not None:
        bounds = (bounds[0] + dx, bounds[1] + dy, bounds[2] + dx, bounds[3] + dy)
    return CropResult(dst, len(geom.shapes or []), texts, bounds)
