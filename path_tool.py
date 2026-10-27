"""
path_tool.py - draw belts on the canvas with CAD-style point input, and
edit their shape afterwards.

Three tools (Tools menu), sharing one input model:

  Draw Belt Path    (P)  a centerline through anchors (lines / tangent arcs),
                         offset by the belt width -> path belt (cad/path.py)
  Draw Belt Outline (O)  the belt's outline itself, corner by corner - any
                         polygon, e.g. a skewed quad feeding a merge ->
                         polygon belt (shape "quadrilateral")
  Belt from Drawing (B)  on an imported DXF layout: click inside an area
                         bounded by drawing lines -> polygon belt with that
                         exact outline; click on a drawing line/polyline/arc
                         -> path belt along it (arcs kept exact)

Point input (Path and Outline tools):
  click            place a point. Object snap (OSNAP) picks the point you
                   mean on the background drawing and on components:
                   endpoint, center, midpoint, nearest-on-edge, else grid.
                   A marker shows which snap applies before you click.
  Shift            polar tracking: the segment angle snaps to 15 deg steps
  typed input      "x,y"  "@dx,dy"  "len<angle"  "len" (along the cursor),
                   Enter - typing a digit while drawing jumps to the
                   toolbar's input box
  click 1st point  close (path: into a loop; outline: finish). Also: C
  Enter / right-click / double-click   finish
  Backspace / Esc  remove the last point / cancel
  L / A            (path only) straight lines / tangent arcs

Insert Belt Shape (Shift+P): triangle / rectangle / polygon / circle loops,
U-turn, L-bend, S-bend presets.

Editing: select a path or polygon belt to show handles on its points.
  drag handle          move the point (with object snap)
  Ctrl+click handle    (path) toggle the segment after it: straight <-> arc
  Shift+click handle   insert a point halfway along the segment after it
  Alt+click handle     delete the point
Every action is undoable. The geometry lives in cad/ (path.py, snap.py,
belts.py); this module is only the Qt interaction around it.
"""
import math
from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainterPath, QPen, QPolygonF, QUndoCommand
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QGraphicsEllipseItem,
    QGraphicsItem, QGraphicsPathItem, QGraphicsRectItem, QGraphicsSimpleTextItem, QLabel, QLineEdit,
    QSpinBox, QToolBar, QToolButton,
)
from shapely.geometry import Polygon

from cad import path as cpath
from cad.belts import outline_belt, path_belt
from cad.snap import DrawingPicker, Snap, SnapIndex, best_snap, chain_from, polygon_vertices
from config_parser import Belt, PathNode, SketchEntity
from id_utils import next_indexed_id

PREVIEW_Z = 1900
GRIP_PX = 9
CLOSE_TOLERANCE_PX = 12
SNAP_LABELS = {"endpoint": "Endpoint", "center": "Center", "midpoint": "Midpoint", "nearest": "Nearest",
               "grid": "Grid"}


def _world_painter_path(polygon, to_scene) -> QPainterPath:
    """Shapely polygon (world metres) -> QPainterPath in scene units, holes kept."""
    path = QPainterPath()
    path.setFillRule(Qt.OddEvenFill)
    if polygon.is_empty:
        return path
    for ring in [polygon.exterior, *polygon.interiors]:
        path.addPolygon(QPolygonF([to_scene(x, y) for x, y in ring.coords]))
        path.closeSubpath()
    return path


def _inert(item):
    item.setAcceptedMouseButtons(Qt.NoButton)
    item.setZValue(PREVIEW_Z)
    return item


def _fixed_size(item):
    item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
    return item


# ---------------------------------------------------------------------------
# Object snap
# ---------------------------------------------------------------------------

class SnapEngine:
    """OSNAP for the drawing tools and handles: background drawing shapes
    (set_background_shapes) plus every component's key points and edges,
    falling back to the grid."""

    def __init__(self, window, *, to_scene_pos, from_scene_pos, snap_free_point, scale):
        self.window = window
        self.to_scene = to_scene_pos
        self.from_scene = from_scene_pos
        self.snap_free_point = snap_free_point
        self.scale = scale
        self.background = SnapIndex()
        self.picker: Optional[DrawingPicker] = None
        self.enabled = True

    def set_background_shapes(self, shapes):
        self.background = SnapIndex.from_shapes(shapes)
        self.picker = DrawingPicker(shapes) if shapes else None

    def tolerance_m(self) -> float:
        return self.window.view._snap_tolerance_scene_units() / self.scale

    def _component_index(self) -> SnapIndex:
        points, edges = self.window.view._all_points_and_edges()
        pts = [(*self.from_scene(p), "endpoint") for p in points]
        segs = []
        for a, b in edges:
            (ax, ay), (bx, by) = self.from_scene(a), self.from_scene(b)
            segs.append((ax, ay, bx, by))
            pts.append(((ax + bx) / 2, (ay + by) / 2, "midpoint"))
        return SnapIndex(pts, segs)

    def snap(self, scene_pos: QPointF) -> Snap:
        x, y = self.from_scene(scene_pos)
        tol = self.tolerance_m()
        if self.enabled:
            hit = best_snap(self.background.query(x, y, tol), self._component_index().query(x, y, tol))
            if hit is not None:
                return hit
        tol_scene = self.window.view._snap_tolerance_scene_units()
        grid = self.snap_free_point(scene_pos, [], [], tol_scene)
        gx, gy = self.from_scene(grid)
        kind = "grid" if (abs(gx - x) > 1e-12 or abs(gy - y) > 1e-12) else ""
        return Snap(gx, gy, kind, math.hypot(gx - x, gy - y))


# ---------------------------------------------------------------------------
# Toolbar
# ---------------------------------------------------------------------------

class PathToolbar(QToolBar):
    """Options shared by the drawing tools: segment mode, belt width, corner
    radius, typed-coordinate input. Each tool enables what applies to it."""

    def __init__(self, parent, on_input: Callable[[str], None]):
        super().__init__("Draw", parent)
        self.mode_label = QLabel("  ")
        self.addWidget(self.mode_label)
        self.line_button = QToolButton(text="Line (L)", checkable=True, checked=True)
        self.arc_button = QToolButton(text="Arc (A)", checkable=True)
        group = QButtonGroup(self)
        group.setExclusive(True)
        group.addButton(self.line_button)
        group.addButton(self.arc_button)
        self.addWidget(self.line_button)
        self.addWidget(self.arc_button)
        self.addSeparator()
        self.addWidget(QLabel(" Width "))
        self.width_spin = QDoubleSpinBox(minimum=0.05, maximum=20.0, singleStep=0.1, value=1.0, decimals=3, suffix=" m")
        self.addWidget(self.width_spin)
        self.addWidget(QLabel(" Corner R "))
        self.corner_spin = QDoubleSpinBox(minimum=0.0, maximum=50.0, singleStep=0.25, value=1.0, decimals=3, suffix=" m")
        self.corner_spin.setToolTip("Path: sharp corners are rounded with this radius when finished (0 = sharp)")
        self.addWidget(self.corner_spin)
        self.addSeparator()
        self.osnap_button = QToolButton(text="OSNAP", checkable=True, checked=True)
        self.osnap_button.setToolTip("Snap to drawing and component points (endpoint, center, midpoint, nearest)")
        self.addWidget(self.osnap_button)
        self.input = QLineEdit()
        self.input.setPlaceholderText("x,y   @dx,dy   len<angle   len   (Enter)")
        self.input.setMinimumWidth(240)
        self.input.returnPressed.connect(lambda: on_input(self.input.text()))
        self.addWidget(self.input)
        self.setEnabled(False)

    @property
    def arc_mode(self) -> bool:
        return self.arc_button.isChecked() and self.arc_button.isEnabled()

    def set_arc_mode(self, arc: bool):
        (self.arc_button if arc else self.line_button).setChecked(True)

    def configure(self, title: str, *, segments: bool, width: bool, corner: bool, points: bool):
        self.setEnabled(True)
        self.mode_label.setText(f"  {title}  ")
        for w in (self.line_button, self.arc_button):
            w.setEnabled(segments)
        self.width_spin.setEnabled(width)
        self.corner_spin.setEnabled(corner)
        self.input.setEnabled(points)


# ---------------------------------------------------------------------------
# Point-input tools
# ---------------------------------------------------------------------------

class _PointTool:
    """Clicks/typed points with OSNAP, polar tracking and a live preview.
    Installed on the CanvasView as `view.tool` while active; the view
    forwards mouse and key events to it first."""

    title = ""
    min_points = 2

    def __init__(self, window, toolbar: PathToolbar, snap: SnapEngine, *, belt_item_cls):
        self.window = window
        self.view = window.view
        self.toolbar = toolbar
        self.snapper = snap
        self.belt_item_cls = belt_item_cls
        self.to_scene = snap.to_scene
        self.from_scene = snap.from_scene
        self.active = False
        self.anchors: List[Tuple[float, float]] = []
        self.bulges: List[float] = []
        self.cursor: Optional[Tuple[float, float]] = None
        self.cursor_snap: Optional[Snap] = None
        self._preview_items = []
        self.on_finished: Optional[Callable[[Belt], None]] = None

    # -- lifecycle -------------------------------------------------------------

    def start(self):
        current = getattr(self.view, "tool", None)
        if current is not None and current is not self:
            current.stop()
        if self.active:
            return
        self.active = True
        self.anchors, self.bulges, self.cursor, self.cursor_snap = [], [], None, None
        self.view.tool = self
        self.view.setDragMode(self.view.DragMode.NoDrag)
        self.view.viewport().setCursor(Qt.CrossCursor)
        self.window.scene.clearSelection()
        self._configure_toolbar()
        self._status(self.hint)

    def stop(self):
        self.active = False
        self._clear_preview()
        if getattr(self.view, "tool", None) is self:
            self.view.tool = None
        self.view.setDragMode(self.view.DragMode.RubberBandDrag)
        self.view.viewport().unsetCursor()
        self.toolbar.setEnabled(False)
        self.toolbar.mode_label.setText("  ")
        self.toolbar.input.clear()

    def _configure_toolbar(self):
        raise NotImplementedError

    def _status(self, text: str, ms: int = 0):
        self.window.statusBar().showMessage(text, ms)

    def _new_id(self) -> str:
        return next_indexed_id([i.model.id for i in self.window.items_list], "belt")

    def _add_belt(self, belt: Belt, text: str):
        item = self.view._add_item(self.belt_item_cls(belt), text=text)
        self.window.scene.clearSelection()
        item.setSelected(True)
        if self.on_finished:
            self.on_finished(belt)
        return item

    # -- points ----------------------------------------------------------------

    def _pick(self, view_pos, shift: bool) -> Tuple[float, float]:
        scene_pos = self.view.mapToScene(view_pos)
        self.snapper.enabled = self.toolbar.osnap_button.isChecked()
        if shift and self.anchors:
            self.cursor_snap = None
            return cpath.snap_angle(self.anchors[-1], self.from_scene(scene_pos), 15.0)
        hit = self.snapper.snap(scene_pos)
        self.cursor_snap = hit if hit.kind else None
        return hit.x, hit.y

    def add_anchor(self, p: Tuple[float, float]):
        if self.anchors and math.hypot(p[0] - self.anchors[-1][0], p[1] - self.anchors[-1][1]) < 1e-6:
            return  # same point twice (e.g. the second click of a double-click)
        if self.anchors:
            self.bulges[-1] = self._bulge_to(p)
        self.anchors.append(p)
        self.bulges.append(0.0)
        self._update_preview()

    def _bulge_to(self, p) -> float:
        return 0.0

    def _near_first(self, view_pos) -> bool:
        if len(self.anchors) < 2:
            return False
        first = self.view.mapFromScene(self.to_scene(*self.anchors[0]))
        return (first - view_pos).manhattanLength() <= CLOSE_TOLERANCE_PX

    def typed_input(self, text: str):
        if not self.active:
            return
        heading = None
        if self.anchors and self.cursor:
            heading = math.atan2(self.cursor[1] - self.anchors[-1][1], self.cursor[0] - self.anchors[-1][0])
        p = cpath.parse_point_input(text, self.anchors[-1] if self.anchors else None, heading)
        if p is None:
            self._status(f"Can't read '{text}'. Use x,y  @dx,dy  len<angle  or len.", 5000)
            return
        self.toolbar.input.clear()
        self.add_anchor(p)
        self.view.setFocus()

    def close_path(self):
        raise NotImplementedError

    def finish(self, closed: bool = False):
        raise NotImplementedError

    # -- events (forwarded by CanvasView) ----------------------------------------

    def mouse_press(self, event) -> bool:
        pos = event.position().toPoint()
        if event.button() == Qt.LeftButton:
            if self._near_first(pos):
                self.close_path()
            else:
                self.add_anchor(self._pick(pos, bool(event.modifiers() & Qt.ShiftModifier)))
            return True
        if event.button() == Qt.RightButton:
            self.finish(closed=False)
            return True
        return False

    def mouse_move(self, event) -> bool:
        self.cursor = self._pick(event.position().toPoint(), bool(event.modifiers() & Qt.ShiftModifier))
        self._update_preview()
        return True

    def mouse_release(self, event) -> bool:
        return event.button() in (Qt.LeftButton, Qt.RightButton)

    def mouse_double_click(self, event) -> bool:
        if event.button() == Qt.LeftButton:
            self.finish(closed=False)
            return True
        return False

    def key_press(self, event) -> bool:
        key = event.key()
        if key == Qt.Key_Escape:
            self.stop()
            self._status(f"{self.title} cancelled.", 4000)
        elif key in (Qt.Key_Return, Qt.Key_Enter):
            self.finish(closed=False)
        elif key == Qt.Key_Backspace:
            if self.anchors:
                self.anchors.pop()
                self.bulges.pop()
                if self.bulges:
                    self.bulges[-1] = 0.0
            self._update_preview()
        elif key == Qt.Key_C:
            self.close_path()
        elif event.text() and event.text() in "0123456789.-@,<":
            self.toolbar.input.setFocus()
            self.toolbar.input.setText(self.toolbar.input.text() + event.text())
        else:
            return False
        return True

    # -- preview -----------------------------------------------------------------

    def _clear_preview(self):
        for item in self._preview_items:
            try:
                if item.scene() is not None:
                    item.scene().removeItem(item)
            except RuntimeError:
                pass
        self._preview_items = []

    def _add_preview(self, item):
        self.window.scene.addItem(_inert(item))
        self._preview_items.append(item)
        return item

    def _preview_points(self):
        pts, bulges = list(self.anchors), list(self.bulges)
        if self.cursor is not None and pts:
            bulges[-1] = self._bulge_to(self.cursor)
            pts.append(self.cursor)
            bulges.append(0.0)
        return pts, bulges

    def _draw_anchor_markers(self):
        for i, (x, y) in enumerate(self.anchors):
            marker = self._add_preview(_fixed_size(QGraphicsRectItem(QRectF(-4, -4, 8, 8))))
            marker.setPos(self.to_scene(x, y))
            marker.setPen(QPen(QColor("#22d3ee" if i == 0 else "#ffffff"), 1.5))

    def _draw_snap_marker(self):
        snap = self.cursor_snap
        if snap is None or self.cursor is None:
            return
        p = self.to_scene(snap.x, snap.y)
        pen = QPen(QColor("#facc15"), 2)
        if snap.kind == "endpoint":
            glyph = QGraphicsRectItem(QRectF(-6, -6, 12, 12))
        elif snap.kind == "center":
            glyph = QGraphicsEllipseItem(QRectF(-6, -6, 12, 12))
        elif snap.kind == "midpoint":
            path = QPainterPath()
            path.addPolygon(QPolygonF([QPointF(0, -7), QPointF(7, 5), QPointF(-7, 5), QPointF(0, -7)]))
            glyph = QGraphicsPathItem(path)
        else:  # nearest / grid: an hourglass-ish X
            path = QPainterPath()
            path.moveTo(-6, -6)
            path.lineTo(6, 6)
            path.moveTo(6, -6)
            path.lineTo(-6, 6)
            glyph = QGraphicsPathItem(path)
            if snap.kind == "grid":
                pen = QPen(QColor(160, 160, 160), 1)
        glyph.setPen(pen)
        self._add_preview(_fixed_size(glyph)).setPos(p)
        label = self._add_preview(_fixed_size(QGraphicsSimpleTextItem(SNAP_LABELS.get(snap.kind, snap.kind))))
        label.setBrush(QBrush(QColor("#facc15")))
        label.setFont(QFont("Segoe UI", 8))
        label.setPos(p + QPointF(9, -18))

    def _draw_readout(self, text: str):
        if self.cursor is None:
            return
        label = self._add_preview(_fixed_size(QGraphicsSimpleTextItem(text)))
        label.setBrush(QBrush(QColor(255, 255, 255)))
        label.setFont(QFont("Segoe UI", 9))
        label.setPos(self.to_scene(*self.cursor) + QPointF(12, 12))

    def _segment_readout(self) -> str:
        last = self.anchors[-1]
        dx, dy = self.cursor[0] - last[0], self.cursor[1] - last[1]
        return f"{math.hypot(dx, dy):.3f} m < {math.degrees(math.atan2(dy, dx)):.1f} deg"

    def _update_preview(self):
        raise NotImplementedError


class PathDrawTool(_PointTool):
    """Centerline through anchors, offset by the belt width -> path belt."""

    title = "Draw Belt Path"
    hint = ("Draw Belt Path: click anchors (OSNAP). L/A line/arc, Shift = 15 deg, type coordinates, "
            "click the first anchor to close, Enter/right-click to finish, Esc to cancel.")

    def _configure_toolbar(self):
        self.toolbar.configure("Path", segments=True, width=True, corner=True, points=True)

    def _incoming_heading(self) -> Optional[float]:
        if len(self.anchors) < 2:
            return None
        return cpath.end_heading(cpath.segments(self.anchors, self.bulges, False))

    def _bulge_to(self, p) -> float:
        heading = self._incoming_heading()
        if self.toolbar.arc_mode and heading is not None:
            return cpath.tangent_bulge(self.anchors[-1], heading, p)
        return 0.0

    def key_press(self, event) -> bool:
        if event.key() == Qt.Key_A:
            self.toolbar.set_arc_mode(True)
            self._update_preview()
            return True
        if event.key() == Qt.Key_L:
            self.toolbar.set_arc_mode(False)
            self._update_preview()
            return True
        return super().key_press(event)

    def close_path(self):
        if len(self.anchors) < 2 or (len(self.anchors) == 2 and not self.toolbar.arc_mode):
            self._status("A loop needs at least 3 anchors (or 2 with arcs).", 5000)
            return
        self.bulges[-1] = self._bulge_to(self.anchors[0])
        self.finish(closed=True)

    def finish(self, closed: bool = False):
        if len(self.anchors) < 2:
            self.stop()
            return
        pts, bulges = list(self.anchors), list(self.bulges)
        corner = self.toolbar.corner_spin.value()
        if corner > 0:
            pts, bulges = cpath.fillet_corners(pts, bulges, closed, corner)
        belt = path_belt(self._new_id(), pts, bulges, closed, self.toolbar.width_spin.value())
        self.stop()
        self._add_belt(belt, "Draw Belt Path")
        length = cpath.path_length(cpath.segments(pts, bulges, closed))
        self._status(f"Drew {belt.id}: {'loop' if closed else 'open path'}, {len(pts)} anchors, "
                     f"{length:.2f} m centerline.", 8000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        pts, bulges = self._preview_points()
        if len(pts) >= 2:
            surface = cpath.footprint(pts, bulges, False, self.toolbar.width_spin.value(), tolerance=0.005)
            body = self._add_preview(QGraphicsPathItem(_world_painter_path(surface, self.to_scene)))
            body.setBrush(QBrush(QColor(59, 130, 246, 90)))
            body.setPen(QPen(QColor(147, 197, 253), 0))
            flat, _ = cpath.flatten(cpath.segments(pts, bulges, False), tolerance=0.01)
            center = QPainterPath()
            center.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in flat]))
            self._add_preview(QGraphicsPathItem(center)).setPen(QPen(QColor(255, 140, 0), 0, Qt.DashLine))
        self._draw_anchor_markers()
        self._draw_snap_marker()
        if self.cursor is not None and self.anchors:
            total = cpath.path_length(cpath.segments(pts, bulges, False))
            mode = "arc" if self.toolbar.arc_mode and self._incoming_heading() is not None else "line"
            self._draw_readout(f"{mode} {self._segment_readout()}   total {total:.3f} m")


class SketchLineTool(PathDrawTool):
    """Construction lines and tangent arcs (no belt yet) - the CAD "line"
    command. Connect them into a belt with Belt from Sketch/Drawing."""

    title = "Sketch Line"
    hint = ("Sketch Line: click points (OSNAP). L/A line/tangent arc, Shift = 15 deg, type coordinates, "
            "click the first point to close, Enter/right-click to finish, Esc to cancel.")
    on_sketch: Optional[Callable[[SketchEntity], None]] = None

    def _configure_toolbar(self):
        self.toolbar.configure("Sketch line", segments=True, width=False, corner=False, points=True)

    def finish(self, closed: bool = False):
        if len(self.anchors) < 2:
            self.stop()
            return
        entity = SketchEntity(points=list(self.anchors), bulges=list(self.bulges), closed=closed)
        self.stop()
        if self.on_sketch:
            self.on_sketch(entity)
        length = cpath.path_length(cpath.segments(entity.points, entity.bulges, closed))
        self._status(f"Sketched {len(entity.points) - (0 if closed else 1)} segment(s), {length:.3f} m. "
                     f"Press B to make a belt from it.", 8000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        pts, bulges = self._preview_points()
        if len(pts) >= 2:
            flat, _ = cpath.flatten(cpath.segments(pts, bulges, False), tolerance=0.01)
            line = QPainterPath()
            line.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in flat]))
            self._add_preview(QGraphicsPathItem(line)).setPen(QPen(QColor("#22d3ee"), 2))
        self._draw_anchor_markers()
        self._draw_snap_marker()
        if self.cursor is not None and self.anchors:
            mode = "arc" if self.toolbar.arc_mode and self._incoming_heading() is not None else "line"
            self._draw_readout(f"{mode} {self._segment_readout()}")


class SketchArcTool(_PointTool):
    """3-point arc: start, end, then a point the arc passes through."""

    title = "Sketch Arc"
    hint = "Sketch Arc: click the start point, the end point, then a point on the arc (OSNAP). Esc to cancel."
    on_sketch: Optional[Callable[[SketchEntity], None]] = None

    def _configure_toolbar(self):
        self.toolbar.configure("Sketch arc", segments=False, width=False, corner=False, points=True)

    def add_anchor(self, p):
        if len(self.anchors) < 2:
            super().add_anchor(p)
            return
        self.anchors.append(p)
        self.finish()

    def close_path(self):
        pass

    def finish(self, closed: bool = False):
        if len(self.anchors) < 3:
            if not self.anchors:
                self.stop()
            return
        p0, p1, through = self.anchors[:3]
        bulge = cpath.three_point_bulge(p0, p1, through)
        entity = SketchEntity(points=[p0, p1], bulges=[bulge, 0.0], closed=False)
        self.stop()
        if self.on_sketch:
            self.on_sketch(entity)
        seg = cpath.bulge_to_segment(p0, p1, bulge)
        self._status(f"Sketched arc R {seg.radius:.3f} m, {math.degrees(abs(seg.sweep)):.1f} deg."
                     if seg.kind == "arc" else "Points were in a line - sketched a straight line.", 8000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        pts = list(self.anchors) + ([self.cursor] if self.cursor is not None else [])
        if len(pts) >= 2:
            bulge = cpath.three_point_bulge(pts[0], pts[1], pts[2]) if len(pts) >= 3 else 0.0
            seg = cpath.bulge_to_segment(pts[0], pts[1], bulge)
            flat, _ = cpath.flatten([seg], tolerance=0.01)
            line = QPainterPath()
            line.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in flat]))
            self._add_preview(QGraphicsPathItem(line)).setPen(QPen(QColor("#22d3ee"), 2))
            if len(pts) >= 3 and seg.kind == "arc":
                self._draw_readout(f"R {seg.radius:.3f} m   {math.degrees(abs(seg.sweep)):.1f} deg")
        self._draw_anchor_markers()
        self._draw_snap_marker()


class OutlineDrawTool(_PointTool):
    """The belt outline itself, corner by corner -> polygon belt. The first
    edge drawn sets the flow direction."""

    title = "Draw Belt Outline"
    hint = ("Draw Belt Outline: click the corners (OSNAP), Shift = 15 deg, type coordinates. "
            "Parcels flow along the first edge. Click the first corner or press Enter to finish, Esc to cancel.")

    def _configure_toolbar(self):
        self.toolbar.configure("Outline", segments=False, width=False, corner=False, points=True)

    def close_path(self):
        self.finish(closed=True)

    def finish(self, closed: bool = True):
        if len(self.anchors) < 3:
            if len(self.anchors) == 0:
                self.stop()
            else:
                self._status("An outline needs at least 3 corners.", 5000)
            return
        (x0, y0), (x1, y1) = self.anchors[0], self.anchors[1]
        heading = math.degrees(math.atan2(y1 - y0, x1 - x0))
        try:
            belt = outline_belt(self._new_id(), self.anchors, flow_heading=heading)
        except ValueError as exc:
            self._status(str(exc), 6000)
            return
        self.stop()
        self._add_belt(belt, "Draw Belt Outline")
        area = Polygon(self.anchors).area if len(self.anchors) >= 3 else 0.0
        self._status(f"Drew {belt.id}: {len(self.anchors)} corners, {area:.2f} m^2, "
                     f"flow {heading % 360:.1f} deg (change with Direction).", 8000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        pts, _ = self._preview_points()
        if len(pts) >= 2:
            outline = QPainterPath()
            outline.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in pts]))
            if len(pts) >= 3:
                outline.closeSubpath()
            body = self._add_preview(QGraphicsPathItem(outline))
            body.setBrush(QBrush(QColor(59, 130, 246, 90)) if len(pts) >= 3 else Qt.NoBrush)
            body.setPen(QPen(QColor(147, 197, 253), 0))
            # Flow arrow along the first edge.
            (x0, y0), (x1, y1) = pts[0], pts[1]
            arrow = QPainterPath()
            arrow.moveTo(self.to_scene((x0 * 2 + x1) / 3, (y0 * 2 + y1) / 3))
            arrow.lineTo(self.to_scene((x0 + x1 * 2) / 3, (y0 + y1 * 2) / 3))
            self._add_preview(QGraphicsPathItem(arrow)).setPen(QPen(QColor(255, 140, 0), 2))
        self._draw_anchor_markers()
        self._draw_snap_marker()
        if self.cursor is not None and self.anchors:
            area = Polygon(pts).area if len(pts) >= 3 else 0.0
            self._draw_readout(f"edge {self._segment_readout()}   area {area:.2f} m^2")


class PickFromDrawingTool:
    """Belt from Drawing: hover the imported layout; inside a closed area the
    area is highlighted, on a line the line is. Click to create the belt."""

    title = "Belt from Sketch/Drawing"
    hint = ("Belt from Sketch/Drawing: click on a line or arc for a belt along it and every line/arc "
            "connected to it (toolbar Width), or inside a closed area for a belt with that outline. "
            "Esc to finish.")

    def __init__(self, window, toolbar: PathToolbar, snap: SnapEngine, *, belt_item_cls):
        self.window = window
        self.view = window.view
        self.toolbar = toolbar
        self.snapper = snap
        self.belt_item_cls = belt_item_cls
        self.to_scene = snap.to_scene
        self.from_scene = snap.from_scene
        self.active = False
        self._preview_items = []
        self.target = None  # ("region", Polygon) | ("line", DrawingShape)

    def start(self):
        if self.snapper.picker is None:
            self.window.statusBar().showMessage(
                "Nothing to pick: sketch lines/arcs first (Tools > Sketch Line / Sketch Arc) "
                "or import a DXF background.", 6000)
            return
        current = getattr(self.view, "tool", None)
        if current is not None and current is not self:
            current.stop()
        self.active = True
        self.view.tool = self
        self.view.setDragMode(self.view.DragMode.NoDrag)
        self.view.viewport().setCursor(Qt.PointingHandCursor)
        self.toolbar.configure("From drawing", segments=False, width=True, corner=False, points=False)
        self.window.statusBar().showMessage(self.hint)

    def stop(self):
        self.active = False
        _PointTool._clear_preview(self)
        if getattr(self.view, "tool", None) is self:
            self.view.tool = None
        self.view.setDragMode(self.view.DragMode.RubberBandDrag)
        self.view.viewport().unsetCursor()
        self.toolbar.setEnabled(False)
        self.toolbar.mode_label.setText("  ")

    def target_at(self, x: float, y: float):
        """("line", (points, bulges, closed)) for the chain of connected
        lines/arcs through the entity under the cursor, ("region", polygon)
        for the closed area around it, or None."""
        picker = self.snapper.picker
        line = picker.pick_line(x, y, self.snapper.tolerance_m())
        if line is not None:
            return ("line", chain_from(picker.shapes, line))
        region = picker.pick_region(x, y)
        return ("region", region) if region is not None else None

    def create(self, target) -> Optional[Belt]:
        new_id = next_indexed_id([i.model.id for i in self.window.items_list], "belt")
        kind, geom = target
        try:
            if kind == "region":
                belt = outline_belt(new_id, polygon_vertices(geom))
            else:
                points, bulges, closed = geom
                belt = path_belt(new_id, points, bulges, closed, self.toolbar.width_spin.value())
        except ValueError as exc:
            self.window.statusBar().showMessage(str(exc), 6000)
            return None
        item = self.view._add_item(self.belt_item_cls(belt), text="Belt from Drawing")
        self.window.scene.clearSelection()
        item.setSelected(True)
        what = "outline" if kind == "region" else "centerline"
        self.window.statusBar().showMessage(f"Created {belt.id} from the drawing {what}. "
                                            f"Click another, or Esc to finish.", 8000)
        return belt

    # -- events --------------------------------------------------------------------

    def mouse_press(self, event) -> bool:
        if event.button() == Qt.LeftButton:
            x, y = self.from_scene(self.view.mapToScene(event.position().toPoint()))
            target = self.target_at(x, y)
            if target is not None:
                self.create(target)
            return True
        if event.button() == Qt.RightButton:
            self.stop()
            return True
        return False

    def mouse_move(self, event) -> bool:
        x, y = self.from_scene(self.view.mapToScene(event.position().toPoint()))
        self.target = self.target_at(x, y)
        _PointTool._clear_preview(self)
        if self.target is not None:
            kind, geom = self.target
            if kind == "region":
                item = _PointTool._add_preview(self, QGraphicsPathItem(_world_painter_path(geom, self.to_scene)))
                item.setBrush(QBrush(QColor(250, 204, 21, 70)))
                item.setPen(QPen(QColor("#facc15"), 2))
            else:
                points, bulges, closed = geom
                flat, _ = cpath.flatten(cpath.segments(points, bulges, closed), 0.01)
                if closed:
                    flat.append(flat[0])
                path = QPainterPath()
                path.addPolygon(QPolygonF([self.to_scene(px, py) for px, py in flat]))
                item = _PointTool._add_preview(self, QGraphicsPathItem(path))
                item.setPen(QPen(QColor("#facc15"), 4))
        return True

    def mouse_release(self, event) -> bool:
        return True

    def mouse_double_click(self, event) -> bool:
        return True

    def key_press(self, event) -> bool:
        if event.key() in (Qt.Key_Escape, Qt.Key_Return, Qt.Key_Enter):
            self.stop()
            return True
        return False


# ---------------------------------------------------------------------------
# Shape presets
# ---------------------------------------------------------------------------

class InsertShapeDialog(QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Insert Belt Shape")
        form = QFormLayout(self)
        self.kind = QComboBox()
        for key, (label, _closed, _params) in cpath.PRESETS.items():
            self.kind.addItem(label, key)
        form.addRow("Shape:", self.kind)
        self.size = QDoubleSpinBox(minimum=0.5, maximum=500.0, value=8.0, decimals=3, suffix=" m")
        self.size_b = QDoubleSpinBox(minimum=0.0, maximum=500.0, value=4.0, decimals=3, suffix=" m")
        self.sides = QSpinBox(minimum=3, maximum=24, value=6)
        self.corner = QDoubleSpinBox(minimum=0.0, maximum=100.0, value=1.0, decimals=3, suffix=" m")
        self.width = QDoubleSpinBox(minimum=0.05, maximum=20.0, value=1.0, decimals=3, suffix=" m")
        self.size_label, self.size_b_label = QLabel("Size:"), QLabel("Size B:")
        form.addRow(self.size_label, self.size)
        form.addRow(self.size_b_label, self.size_b)
        form.addRow("Sides:", self.sides)
        form.addRow("Corner radius:", self.corner)
        form.addRow("Belt width:", self.width)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)
        self.kind.currentIndexChanged.connect(self._sync_fields)
        self._sync_fields()

    _HINTS = {
        "triangle": ("Side:", None), "rectangle": ("Length:", "Depth:"), "polygon": ("Circumradius:", None),
        "circle": ("Diameter:", None), "u_turn": ("Leg length:", "Leg spacing:"),
        "l_bend": ("Leg 1:", "Leg 2:"), "s_bend": ("Length:", "Offset:"),
    }

    def _sync_fields(self):
        kind = self.kind.currentData()
        params = cpath.PRESETS[kind][2]
        size_hint, b_hint = self._HINTS[kind]
        self.size_label.setText(size_hint)
        self.size_b.setEnabled("size_b" in params)
        self.size_b_label.setText(b_hint or "Size B:")
        self.sides.setEnabled("sides" in params)
        self.corner.setEnabled(kind in ("triangle", "rectangle", "polygon", "l_bend"))

    def values(self) -> dict:
        return {"kind": self.kind.currentData(), "size": self.size.value(), "size_b": self.size_b.value(),
                "sides": self.sides.value(), "corner_radius": self.corner.value(), "width": self.width.value()}


def make_preset_belt(values: dict, center: Tuple[float, float], existing_ids) -> Belt:
    pts, bulges, closed = cpath.preset(values["kind"], values["size"], values["size_b"], values["sides"],
                                       values["corner_radius"])
    return Belt(
        id=next_indexed_id(existing_ids, "belt"), type="path", shape="path", x=center[0], y=center[1],
        rotation=0.0, beltWidth=values["width"], speed=1.0, directionAngle=0.0, color="#3b82f6",
        path=[PathNode(x=x, y=y, bulge=b) for (x, y), b in zip(pts, bulges)], closed=closed,
    )


# ---------------------------------------------------------------------------
# Point handles on path and polygon belts
# ---------------------------------------------------------------------------

def _is_polygon_belt(model) -> bool:
    return model.type == "linear" and getattr(model, "shape", "") == "quadrilateral" and bool(model.vertexs)


def _shape_field(model) -> Optional[str]:
    if model.type == "path":
        return "path"
    if _is_polygon_belt(model):
        return "vertexs"
    return None


def _copy_field(value):
    return [dict(v) if isinstance(v, dict) else v.model_copy() for v in value]


class ShapeEditCommand(QUndoCommand):
    """Before/after of a belt's point list (`path` nodes or polygon `vertexs`)."""

    def __init__(self, grips, item, field: str, before, after, text: str):
        super().__init__(text)
        self.grips = grips
        self.item = item
        self.field = field
        self.before = _copy_field(before)
        self.after = _copy_field(after)

    def _apply(self, value):
        setattr(self.item.model, self.field, _copy_field(value))
        self.item.sync_from_model()
        self.grips.refresh()
        for view in self.item.scene().views() if self.item.scene() else []:
            if hasattr(view, "invalidate_points_edges_cache"):
                view.invalidate_points_edges_cache()

    def redo(self):
        self._apply(self.after)

    def undo(self):
        self._apply(self.before)


# Backwards-compatible name (path-only edits).
PathEditCommand = ShapeEditCommand


class AnchorGrip(QGraphicsRectItem):
    def __init__(self, manager, belt_item, index: int):
        super().__init__(QRectF(-GRIP_PX / 2, -GRIP_PX / 2, GRIP_PX, GRIP_PX), belt_item)
        self.manager = manager
        self.belt_item = belt_item
        self.index = index
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setZValue(PREVIEW_Z)
        self.setBrush(QBrush(QColor("#ffffff")))
        self.setPen(QPen(QColor("#0f172a"), 1))
        self.setCursor(Qt.SizeAllCursor)
        tip = "Drag: move | Shift+click: insert after | Alt+click: delete"
        if belt_item.model.type == "path":
            tip += " | Ctrl+click: straight/arc"
        self.setToolTip(tip)
        self._before = None

    def mousePressEvent(self, event):
        mods = event.modifiers()
        field = _shape_field(self.belt_item.model)
        if mods & Qt.ControlModifier and field == "path":
            self.manager.toggle_arc(self.belt_item, self.index)
        elif mods & Qt.ShiftModifier:
            self.manager.insert_after(self.belt_item, self.index)
        elif mods & Qt.AltModifier:
            self.manager.delete_point(self.belt_item, self.index)
        else:
            self._before = _copy_field(getattr(self.belt_item.model, field))
        event.accept()

    def mouseMoveEvent(self, event):
        if self._before is None:
            return
        hit = self.manager.snap.snap(event.scenePos()) if self.manager.snap else None
        scene_pos = self.manager.snap.to_scene(hit.x, hit.y) if hit else event.scenePos()
        local = self.belt_item.mapFromScene(scene_pos)
        x, y = local.x() / self.manager.scale, -local.y() / self.manager.scale
        point = getattr(self.belt_item.model, _shape_field(self.belt_item.model))[self.index]
        if isinstance(point, dict):
            point["x"], point["y"] = x, y
        else:
            point.x, point.y = x, y
        self.belt_item.sync_from_model()
        self.setPos(local)
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._before is None:
            return
        before, self._before = self._before, None
        field = _shape_field(self.belt_item.model)
        after = _copy_field(getattr(self.belt_item.model, field))
        if before != after:
            self.manager.push(self.belt_item, field, before, after, "Move point")
        event.accept()


class AnchorGripManager:
    """Shows point handles on the single selected path or polygon belt."""

    def __init__(self, window, *, belt_item_cls, scale, snap: Optional[SnapEngine] = None):
        self.window = window
        self.belt_item_cls = belt_item_cls
        self.scale = scale
        self.snap = snap
        self.grips: List[AnchorGrip] = []
        self.item = None
        window.scene.selectionChanged.connect(self.refresh)

    def clear(self):
        for grip in self.grips:
            try:
                if grip.scene() is not None:
                    grip.scene().removeItem(grip)
            except RuntimeError:
                pass
        self.grips = []

    def refresh(self):
        self.clear()
        try:
            selected = [i for i in self.window.scene.selectedItems() if isinstance(i, self.belt_item_cls)]
        except RuntimeError:
            return  # scene already destroyed (window closing)
        item = selected[0] if len(selected) == 1 else None
        self.item = item if item is not None and _shape_field(item.model) else None
        if self.item is None:
            return
        for i, p in enumerate(self._local_points(self.item)):
            grip = AnchorGrip(self, self.item, i)
            grip.setPos(p)
            self.grips.append(grip)

    def _local_points(self, item) -> List[QPointF]:
        field = _shape_field(item.model)
        if field == "path":
            return item.path_anchor_scene_points()
        return [QPointF(v["x"] * self.scale, -v["y"] * self.scale) for v in item.model.vertexs]

    def push(self, item, field, before, after, text):
        self.window.undo_stack.push(ShapeEditCommand(self, item, field, before, after, text))

    def toggle_arc(self, item, index: int):
        nodes = item.model.path
        n = len(nodes)
        if index == n - 1 and not item.model.closed:
            self.window.statusBar().showMessage("The last anchor of an open path has no segment after it.", 5000)
            return
        after = _copy_field(nodes)
        if abs(after[index].bulge) > 1e-12:
            after[index].bulge = 0.0
        else:
            pts = [(x.x, x.y) for x in nodes]
            bulges = [x.bulge for x in nodes]
            nxt = pts[(index + 1) % n]
            if index > 0 or item.model.closed:
                # Tangent to the incoming segment, so the path stays smooth.
                incoming = cpath.bulge_to_segment(pts[index - 1], pts[index], bulges[index - 1])
                after[index].bulge = cpath.tangent_bulge(pts[index], incoming.heading_at(1.0), nxt)
            else:
                after[index].bulge = math.tan(math.radians(90.0) / 4.0)  # a quarter arc to start with
        self.push(item, "path", nodes, after, "Toggle arc")

    def insert_after(self, item, index: int):
        field = _shape_field(item.model)
        if field == "vertexs":
            verts = item.model.vertexs
            a, b = verts[index], verts[(index + 1) % len(verts)]
            after = _copy_field(verts)
            after.insert(index + 1, {"x": (a["x"] + b["x"]) / 2, "y": (a["y"] + b["y"]) / 2})
            self.push(item, field, verts, after, "Insert corner")
            return
        nodes = item.model.path
        n = len(nodes)
        if index == n - 1 and not item.model.closed:
            return
        seg = cpath.bulge_to_segment((nodes[index].x, nodes[index].y),
                                     (nodes[(index + 1) % n].x, nodes[(index + 1) % n].y), nodes[index].bulge)
        mx, my = seg.point_at(0.5)
        half = math.tan(math.atan(nodes[index].bulge) / 2.0)  # each half of an arc keeps the curve
        after = _copy_field(nodes)
        after[index].bulge = half
        after.insert(index + 1, PathNode(x=mx, y=my, bulge=half))
        self.push(item, field, nodes, after, "Insert anchor")

    def delete_point(self, item, index: int):
        field = _shape_field(item.model)
        points = getattr(item.model, field)
        minimum = 3 if (field == "vertexs" or item.model.closed) else 2
        if len(points) <= minimum:
            self.window.statusBar().showMessage(f"This belt needs at least {minimum} points.", 5000)
            return
        after = _copy_field(points)
        del after[index]
        if field == "path" and index > 0:
            after[index - 1].bulge = 0.0  # the merged segment becomes straight
        self.push(item, field, points, after, "Delete point")

    # Old name, kept for callers that only knew path belts.
    delete_anchor = delete_point
