"""
layout_editor_cad.py - CAD tools for the layout editor: Qt glue around the
Qt-free cad/ package.

  Tools > Connect Belts      (Ctrl+J)        planegcs: B's inlet onto A's outlet
  Tools > Fillet Belts...    (Ctrl+Shift+F)  planegcs: tangent curve between two straights
  Tools > Disconnect Belts   (Ctrl+Shift+J)
  Tools > Solve Connections  (F6)            planegcs: re-attach every connection
  Tools > Check Layout       (F7)            Shapely: overlaps, open outlets, ...
  File  > Import DXF Background... / Background Settings... / Remove Background /
          Export DXF...                       ezdxf (dialog: background_dialog.py)
  View  > Show Ports                         inlet/outlet/connection markers

Connections are kept while editing: when a belt is dragged, every belt
connected to it is re-solved in the same undo step. Every change goes
through the editor's undo stack.
"""
import contextlib
import math
import os
from typing import Dict, List, Optional

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import (
    QAction, QBrush, QColor, QKeySequence, QPainterPath, QPainterPathStroker, QPen, QPolygonF, QUndoCommand,
)
from PySide6.QtWidgets import (
    QApplication, QDialog, QDoubleSpinBox, QFileDialog, QGraphicsEllipseItem, QGraphicsPathItem, QGraphicsPolygonItem, QInputDialog,
    QLabel, QListWidget, QListWidgetItem, QMessageBox, QPushButton, QStyle, QToolBar, QVBoxLayout, QWidget,
)

from background_dialog import BackgroundImportDialog
import ezdxf

from cad import dxf_io
from cad.checks import check_layout
from cad.constraints import (
    GEOMETRY_FIELDS, ConstraintError, connect_belts, fillet_belts, solve_layout,
)
from cad.belts import belt_chain_connections
from cad.ports import belt_ports
from cad.snap import join_shapes
from config_parser import BackgroundDrawing, BeltConnection, SketchEntity
from id_utils import next_indexed_id
from sketch_grips import SketchGripManager
from modify_tool import (
    PlaceBackgroundTool,
    BreakTool, CopyTool, DivideTool, ExtendTool, FilletTool, MirrorTool, MoveTool, OffsetTool, RotateTool, ScaleTool,
    TrimTool,
)
from path_tool import (
    AnchorGripManager, CurvedBeltDrawTool, OutlineDrawTool, PathDrawTool, PathToolbar, PickFromDrawingTool,
    SketchArcTool,
    SketchCircleTool, SketchLineTool, SketchPolygonTool, SketchRectangleTool, SnapEngine,
)
from cad.path import flatten as path_flatten, segments as path_segments

PORT_Z = 1500
SKETCH_Z = 900
BACKGROUND_Z = -1000


# ---------------------------------------------------------------------------
# Undo commands
# ---------------------------------------------------------------------------

class BeltGeometryCommand(QUndoCommand):
    """Set geometry fields on one or more belts (before/after per belt id)."""

    def __init__(self, controller, before: Dict[str, dict], after: Dict[str, dict], text: str):
        super().__init__(text)
        self.controller = controller
        self.before = before
        self.after = after

    def redo(self):
        self.controller.apply_geometry(self.after)

    def undo(self):
        self.controller.apply_geometry(self.before)


class ConnectionsCommand(QUndoCommand):
    def __init__(self, controller, before: List[BeltConnection], after: List[BeltConnection], text: str):
        super().__init__(text)
        self.controller = controller
        self.before = [c.model_copy() for c in before]
        self.after = [c.model_copy() for c in after]

    def redo(self):
        self.controller.set_connections(self.after)

    def undo(self):
        self.controller.set_connections(self.before)


class BackgroundCommand(QUndoCommand):
    def __init__(self, controller, before: Optional[BackgroundDrawing], after: Optional[BackgroundDrawing], text: str):
        super().__init__(text)
        self.controller = controller
        self.before = before
        self.after = after

    def redo(self):
        self.controller.set_background(self.after)

    def undo(self):
        self.controller.set_background(self.before)


# ---------------------------------------------------------------------------
# Scene overlays
# ---------------------------------------------------------------------------

def _inert(item):
    """Overlay item: never selected, moved, or hit by the mouse."""
    item.setAcceptedMouseButtons(Qt.NoButton)
    item.setFlag(item.GraphicsItemFlag.ItemIsSelectable, False)
    item.setFlag(item.GraphicsItemFlag.ItemIsMovable, False)
    return item


class PortOverlay:
    """Markers on the canvas: inlet (green ring), outlet (orange arrow head),
    saved connection (cyan ring) or broken connection (red ring). Inlets and
    outlets only with View > Show Ports (off by default - every belt end
    marked is a lot of noise); connections always."""

    def __init__(self, controller):
        self.controller = controller
        self.items = []
        self.visible = False

    def clear(self):
        for item in self.items:
            try:
                if item.scene() is not None:
                    item.scene().removeItem(item)
            except RuntimeError:
                pass  # already destroyed with the scene (window closing)
        self.items = []

    def refresh(self):
        self.clear()
        scene = self.controller.window.scene
        to_scene = self.controller.to_scene_pos
        scale = self.controller.scale
        belts = {b.id: b for b in self.controller.belt_models()}
        for belt in (belts.values() if self.visible else ()):
            ports = belt_ports(belt)
            if ports is None:
                continue
            r = 0.1 * scale
            p = to_scene(ports.inlet.x, ports.inlet.y)
            ring = QGraphicsEllipseItem(p.x() - r, p.y() - r, 2 * r, 2 * r)
            ring.setPen(QPen(QColor("#22c55e"), 0))
            ring.setBrush(Qt.NoBrush)
            self._add(scene, ring)
            o = to_scene(ports.outlet.x, ports.outlet.y)
            rad = math.radians(ports.outlet.heading)
            ux, uy = math.cos(rad), -math.sin(rad)  # scene Y is down
            size = 0.22 * scale
            tip = QPointF(o.x() + ux * size * 0.5, o.y() + uy * size * 0.5)
            base = QPointF(o.x() - ux * size * 0.5, o.y() - uy * size * 0.5)
            left = QPointF(base.x() - uy * size * 0.45, base.y() + ux * size * 0.45)
            right = QPointF(base.x() + uy * size * 0.45, base.y() - ux * size * 0.45)
            arrow = QGraphicsPolygonItem(QPolygonF([tip, left, right]))
            arrow.setPen(Qt.NoPen)
            arrow.setBrush(QBrush(QColor("#f97316")))
            self._add(scene, arrow)
        broken = self.controller.broken_connection_ids()
        for conn in self.controller.config.connections:
            a = belts.get(conn.fromBelt)
            ports = belt_ports(a) if a is not None else None
            if ports is None:
                continue
            r = 0.18 * scale
            p = to_scene(ports.outlet.x, ports.outlet.y)
            ring = QGraphicsEllipseItem(p.x() - r, p.y() - r, 2 * r, 2 * r)
            bad = (conn.fromBelt, conn.toBelt) in broken
            ring.setPen(QPen(QColor("#ef4444" if bad else "#22d3ee"), 2.5))
            ring.setBrush(Qt.NoBrush)
            self._add(scene, ring)

    def _add(self, scene, item):
        item.setZValue(PORT_Z)
        scene.addItem(_inert(item))
        self.items.append(item)


class BackgroundItem(QGraphicsPathItem):
    def __init__(self, polylines_scene):
        path = QPainterPath()
        for pts in polylines_scene:
            path.moveTo(pts[0])
            for p in pts[1:]:
                path.lineTo(p)
        super().__init__(path)
        pen = QPen(QColor(150, 150, 150), 0)  # cosmetic: 1 px at any zoom
        self.setPen(pen)
        self.setZValue(BACKGROUND_Z)
        _inert(self)


class BackgroundAnchor(QGraphicsPathItem):
    """The background drawing's placement anchor (its lower-left corner):
    drag it to move the whole drawing - it snaps to the layout and the
    sketch - one undo step on the drop. A child of the BackgroundItem, so
    it rides along with the Place Background preview too."""

    SIZE = 9.0

    def __init__(self, controller, home):
        path = QPainterPath()
        r = self.SIZE
        path.addEllipse(QPointF(0, 0), r * 0.55, r * 0.55)
        path.moveTo(-r, 0)
        path.lineTo(r, 0)
        path.moveTo(0, -r)
        path.lineTo(0, r)
        super().__init__(path, controller.background_item)
        self.controller = controller
        self.home = home
        self.setPos(controller.to_scene_pos(*home))
        self.setFlag(QGraphicsPathItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setZValue(1)
        pen = QPen(QColor("#38bdf8"), 2)
        self.setPen(pen)
        self.setCursor(Qt.SizeAllCursor)
        self.setToolTip("Background anchor: drag to move the drawing (snaps to the layout and sketch). "
                        "Shift+B places it by two points.")
        self._dragging = False
        self._delta = (0.0, 0.0)

    def shape(self) -> QPainterPath:
        hit = QPainterPath()
        hit.addEllipse(QPointF(0, 0), self.SIZE + 3, self.SIZE + 3)
        return hit

    def mousePressEvent(self, event):
        c = self.controller
        self._dragging, self._delta = True, (0.0, 0.0)
        c.snap.set_background_shapes(list(c.config.sketch))     # not the drawing being moved
        event.accept()

    def mouseMoveEvent(self, event):
        if not self._dragging:
            return
        c = self.controller
        hit = c.snap.snap(event.scenePos())
        self._delta = (hit.x - self.home[0], hit.y - self.home[1])
        c.background_item.setPos(c.to_scene_pos(*self._delta) - c.to_scene_pos(0, 0))
        c._status(f"Background anchor to ({hit.x:.3f}, {hit.y:.3f})" + (f"  [{hit.kind}]" if hit.kind else ""), 2000)
        event.accept()

    def mouseReleaseEvent(self, event):
        if not self._dragging:
            return
        self._dragging = False
        c = self.controller
        dx, dy = self._delta
        c.background_item.setPos(0, 0)
        c._refresh_snap_shapes()
        event.accept()
        # After the event: the step rebuilds the background (and this anchor).
        QTimer.singleShot(0, lambda: c.move_background(dx, dy))


@contextlib.contextmanager
def _busy():
    """Wait cursor while reading a big drawing."""
    QApplication.setOverrideCursor(Qt.WaitCursor)
    try:
        yield
    finally:
        QApplication.restoreOverrideCursor()


# ---------------------------------------------------------------------------
# Layout check panel
# ---------------------------------------------------------------------------

class LayoutCheckPanel(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        run = QPushButton("Check Layout (F7)")
        run.clicked.connect(controller.run_check)
        layout.addWidget(run)
        self.list = QListWidget()
        self.list.itemDoubleClicked.connect(self._focus_issue)
        layout.addWidget(self.list)

    def show_issues(self, issues):
        self.list.clear()
        if not issues:
            self.list.addItem(QListWidgetItem("No issues found."))
            return
        for issue in issues:
            row = QListWidgetItem(f"[{issue.severity.upper()}] {issue.kind}: {issue.message}")
            row.setForeground(QColor("#ef4444" if issue.severity == "error" else "#eab308"))
            row.setData(Qt.UserRole, issue)
            self.list.addItem(row)

    def _focus_issue(self, row):
        issue = row.data(Qt.UserRole)
        if issue is not None:
            self.controller.select_ids(issue.item_ids, issue.location)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class CadController:
    def __init__(self, window, *, belt_item_cls, to_scene_pos, from_scene_pos, snap_free_point, scale, file_menu):
        self.window = window
        self.belt_item_cls = belt_item_cls
        self.to_scene_pos = to_scene_pos
        self.from_scene_pos = from_scene_pos
        self.scale = scale
        # Drawing tools share one toolbar and one object-snap engine (which
        # also snaps to the imported DXF background, see set_background).
        self.snap = SnapEngine(window, to_scene_pos=to_scene_pos, from_scene_pos=from_scene_pos,
                               snap_free_point=snap_free_point, scale=scale)
        self.draw_toolbar = PathToolbar(window, self._typed_input)
        # The active tool's options and the coordinate input along the bottom,
        # full width (like a CAD command line): the top rows hold the commands.
        self.draw_toolbar.setObjectName("drawToolbar")
        window.addToolBar(Qt.BottomToolBarArea, self.draw_toolbar)
        self.path_tool = PathDrawTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.outline_tool = OutlineDrawTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.curved_tool = CurvedBeltDrawTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.path_tool.on_belts = self.add_belt_chain
        self.pick_tool = PickFromDrawingTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.sketch_line_tool = SketchLineTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.sketch_arc_tool = SketchArcTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.sketch_line_tool.on_sketch = self.add_sketch
        self.sketch_arc_tool.on_sketch = self.add_sketch
        self.sketch_rect_tool = SketchRectangleTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.sketch_circle_tool = SketchCircleTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.sketch_polygon_tool = SketchPolygonTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        for tool in (self.sketch_rect_tool, self.sketch_circle_tool, self.sketch_polygon_tool):
            tool.on_sketch = self.add_sketch
        # Modify tools (modify_tool.py): trim, extend, break, offset, fillet, transforms.
        self.modify_tools = {}
        for name, cls in (("trim", TrimTool), ("extend", ExtendTool), ("break", BreakTool), ("offset", OffsetTool),
                          ("fillet", FilletTool), ("move", MoveTool), ("copy", CopyTool), ("rotate", RotateTool),
                          ("mirror", MirrorTool), ("scale", ScaleTool)):
            tool = cls(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
            tool.cad = self
            self.modify_tools[name] = tool
        self.divide_tool = DivideTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.divide_tool.cad = self
        self.place_tool = PlaceBackgroundTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.place_tool.cad = self
        self.sketch_items: List[SketchItem] = []
        self._background_shapes = []
        self._background_cache = None       # (file/filter key, geometry): re-placing doesn't re-read the DXF
        window.view.on_delete_extra = self._delete_selected_sketch
        window.view.on_copy_extra = self.selected_sketch_entities
        window.view.on_paste_extra = self.paste_sketch
        self.grips = AnchorGripManager(window, belt_item_cls=belt_item_cls, scale=scale, snap=self.snap)
        self.sketch_grips = SketchGripManager(self)
        self.ports = PortOverlay(self)
        self.closing = False  # set by MainWindow.closeEvent: stop touching the scene
        self.background_item: Optional[BackgroundItem] = None
        self.background_anchor: Optional[BackgroundAnchor] = None
        self.check_panel = LayoutCheckPanel(self)
        self._build_actions(file_menu)
        panel = getattr(window, "properties_panel", None)
        if panel is not None:
            panel.sketch_fields = self.fill_sketch_properties
        window.scene.on_items_moved = self.on_items_moved
        window.undo_stack.indexChanged.connect(self._on_undo_index_changed)
        if self.config.background is not None:
            self.set_background(self.config.background, quiet_errors=True)
        self.set_sketch(self.config.sketch)
        self.ports.refresh()

    def _on_undo_index_changed(self, _index):
        if self.closing:
            return
        try:
            self.ports.refresh()
            self.grips.refresh()
        except RuntimeError:
            # Qt already destroyed the scene (process exiting without a
            # closeEvent) - nothing left to draw on.
            self.closing = True

    # -- model access --------------------------------------------------------

    @property
    def config(self):
        return self.window.config

    def belt_items(self) -> Dict[str, object]:
        return {i.model.id: i for i in self.window.items_list if isinstance(i, self.belt_item_cls)}

    def belt_models(self):
        return [i.model for i in self.belt_items().values()]

    def selected_belt_items(self):
        return [i for i in self.window.scene.selectedItems() if isinstance(i, self.belt_item_cls)]

    def broken_connection_ids(self):
        belts = {b.id: b for b in self.belt_models()}
        broken = set()
        for c in self.config.connections:
            a, b = belts.get(c.fromBelt), belts.get(c.toBelt)
            pa = belt_ports(a) if a else None
            pb = belt_ports(b) if b else None
            if pa is None or pb is None:
                broken.add((c.fromBelt, c.toBelt))
                continue
            gap = math.hypot(pa.outlet.x - pb.inlet.x, pa.outlet.y - pb.inlet.y)
            diff = (pa.outlet.heading - pb.inlet.heading) % 360.0
            if gap > 0.005 or min(diff, 360 - diff) > 1.0:
                broken.add((c.fromBelt, c.toBelt))
        return broken

    # -- mutations used by the undo commands ----------------------------------

    def apply_geometry(self, geometry: Dict[str, dict]):
        items = self.belt_items()
        for belt_id, geom in geometry.items():
            item = items.get(belt_id)
            if item is None:
                continue
            for field, value in geom.items():
                setattr(item.model, field, value)
            item.sync_from_model()
        self.window.view.invalidate_points_edges_cache()
        self._refresh_panels()

    def set_connections(self, connections: List[BeltConnection]):
        self.config.connections = [c.model_copy() for c in connections]
        self.ports.refresh()

    def set_background(self, background: Optional[BackgroundDrawing], quiet_errors: bool = False):
        if self.background_item is not None:
            self.window.scene.removeItem(self.background_item)
            self.background_item = None
        self.background_anchor = None
        self.config.background = background
        self._background_shapes = []
        self._refresh_snap_shapes()
        if background is None or not background.visible:
            return
        path = self._resolve(background.path)
        try:
            key = (os.path.abspath(path), os.path.getmtime(path), background.clip, tuple(background.skipTypes),
                   tuple(background.hiddenLayers))
            if self._background_cache is not None and self._background_cache[0] == key:
                geom = self._background_cache[1]
            else:
                with _busy():
                    geom = dxf_io.read_background(path, clip=background.clip, skip_types=background.skipTypes,
                                                  hidden_layers=background.hiddenLayers)
                self._background_cache = (key, geom)
        except (OSError, IOError, ValueError, ezdxf.DXFError) as exc:
            message = f"Could not load background {path}: {exc}"
            if quiet_errors:
                self.window.statusBar().showMessage(message, 10000)
                return
            raise
        polylines = dxf_io.background_in_metres(geom, background.scale, background.offsetX, background.offsetY)
        self._background_shapes = dxf_io.shapes_in_metres(geom, background.scale, background.offsetX,
                                                          background.offsetY)
        self._refresh_snap_shapes()
        scene_lines = [[self.to_scene_pos(x, y) for x, y in pl] for pl in polylines]
        self.background_item = BackgroundItem(scene_lines)
        self.window.scene.addItem(self.background_item)
        points = [p for pl in polylines for p in pl]
        if points:
            home = (min(x for x, _ in points), min(y for _, y in points))      # the drawing's lower-left corner
            self.background_anchor = BackgroundAnchor(self, home)
        rect = self.background_item.boundingRect()
        margin = max(rect.width(), rect.height()) * 0.1
        self.window.scene.setSceneRect(self.window.scene.sceneRect().united(rect.adjusted(-margin, -margin, margin, margin)))

    # -- sketch ----------------------------------------------------------------

    def _refresh_snap_shapes(self):
        """Snap/pick targets = background drawing + sketch entities."""
        self.snap.set_background_shapes(list(self._background_shapes) + list(self.config.sketch))

    def set_sketch(self, entities: List[SketchEntity]):
        for item in self.sketch_items:
            try:
                if item.scene() is not None:
                    item.scene().removeItem(item)
            except RuntimeError:
                pass
        self.config.sketch = [e.model_copy() for e in entities]
        self.sketch_items = []
        for i, entity in enumerate(self.config.sketch):
            item = SketchItem(entity, i, self.to_scene_pos)
            self.window.scene.addItem(item)
            self.sketch_items.append(item)
        self._refresh_snap_shapes()

    def preview_sketch(self, index: int, shape):
        """Show a grip drag live: sketch entity `index` becomes `shape`
        without an undo step (edit_sketch makes the step on the drop)."""
        pts, bul, closed = shape
        entity = self.config.sketch[index]
        entity.points, entity.bulges, entity.closed = [tuple(p) for p in pts], list(bul), closed
        self.sketch_items[index].set_entity(entity, self.to_scene_pos)

    def edit_sketch(self, index: int, shape, text: str = "Edit sketch", before_shape=None):
        """Sketch entity `index` becomes `shape` - one undo step, the entity
        stays selected. `before_shape`: its shape before a live preview."""
        before = [e.model_copy(deep=True) for e in self.config.sketch]
        if before_shape is not None:
            pts, bul, closed = before_shape
            before[index] = SketchEntity(points=pts, bulges=bul, closed=closed)
        pts, bul, closed = shape
        after = list(before)
        after[index] = SketchEntity(points=[tuple(p) for p in pts], bulges=list(bul), closed=closed)
        self.window.undo_stack.push(SketchCommand(self, before, after, text))
        self.select_sketch([index])

    def select_sketch(self, indices):
        self.window.scene.clearSelection()
        for item in self.sketch_items:
            if item.index in indices:
                item.setSelected(True)

    def add_sketch(self, entity: SketchEntity):
        before = list(self.config.sketch)
        self.window.undo_stack.push(SketchCommand(self, before, before + [entity], "Sketch"))

    def _delete_selected_sketch(self) -> bool:
        try:
            selected = {i.index for i in self.window.scene.selectedItems() if isinstance(i, SketchItem)}
        except RuntimeError:
            return False
        if not selected:
            return False
        before = list(self.config.sketch)
        after = [e for i, e in enumerate(before) if i not in selected]
        self.window.undo_stack.push(SketchCommand(self, before, after, "Delete sketch"))
        return True

    def fill_sketch_properties(self, panel) -> bool:
        """The Properties grid for one selected sketch entity: a line's end
        points, length and angle; an arc's center, radius and angles; a
        circle's center and radius; a polyline's vertices. Each edit is one
        undo step (edit_sketch)."""
        from cad import modify as cmod
        indices = self.selected_sketch_indices()
        if len(indices) != 1:
            if indices:
                panel.empty.setText(f"{len(indices)} sketch entities selected.\nSelect one to edit its numbers.")
            return False
        index = indices[0]
        shape = cmod.as_shape(self.config.sketch[index])
        info = cmod.describe(shape)
        kind = info["kind"]
        spins = {}

        def spin(group, label, key, value, suffix=" m", lo=-1e6, hi=1e6):
            box = QDoubleSpinBox()
            box.setRange(lo, hi)
            box.setDecimals(3 if suffix == " m" else 2)
            box.setSuffix(suffix)
            box.setKeyboardTracking(False)          # one edit per Enter / focus change, not per key
            box.setValue(value)
            box.valueChanged.connect(lambda _v: QTimer.singleShot(0, apply))
            box.setToolTip(key)
            spins[key] = box
            panel._add_row(group, label, box)

        def apply():
            v = {k: b.value() for k, b in spins.items()}
            try:
                if kind == "line":
                    (x0, y0), (x1, y1) = (v["x0"], v["y0"]), (v["x1"], v["y1"])
                    if abs(v["length"] - info["length"]) > 1e-9 or abs(v["angle"] - info["angle"]) > 1e-9:
                        a = math.radians(v["angle"])        # length / angle: the start stays put
                        x1, y1 = x0 + v["length"] * math.cos(a), y0 + v["length"] * math.sin(a)
                    new = ([(x0, y0), (x1, y1)], [0.0, 0.0], False)
                elif kind == "arc":
                    new = cmod.make_arc((v["cx"], v["cy"]), v["radius"], v["start"], v["sweep"])
                elif kind == "circle":
                    new = cmod.make_circle((v["cx"], v["cy"]), v["radius"])
                else:
                    new = cmod.move_vertices(shape, {i: (v[f"x{i}"], v[f"y{i}"]) for i in range(len(shape[0]))
                                                     if f"x{i}" in v})
            except cmod.ModifyError as e:
                self._status(str(e))
                return
            if new != shape:
                self.edit_sketch(index, new, f"Edit {kind}")

        if kind == "line":
            spin("Start", "X", "x0", info["start"][0])
            spin("Start", "Y", "y0", info["start"][1])
            spin("End", "X", "x1", info["end"][0])
            spin("End", "Y", "y1", info["end"][1])
            spin("Geometry", "Length", "length", info["length"], lo=1e-6)
            spin("Geometry", "Angle", "angle", info["angle"], suffix=" °", lo=-360.0, hi=360.0)
        elif kind in ("arc", "circle"):
            spin("Center", "X", "cx", info["center"][0])
            spin("Center", "Y", "cy", info["center"][1])
            spin("Geometry", "Radius", "radius", info["radius"], lo=1e-6)
            if kind == "arc":
                spin("Geometry", "Start angle", "start", info["start"], suffix=" °", lo=-360.0, hi=360.0)
                spin("Geometry", "Sweep", "sweep", info["sweep"], suffix=" °", lo=-359.99, hi=359.99)
                panel._add_row("Geometry", "Length", QLabel(f"{info['length']:.3f} m"))
        else:
            panel._add_row("Geometry", "Length", QLabel(f"{info['length']:.3f} m"))
            panel._add_row("Geometry", "Closed", QLabel("yes" if info["closed"] else "no"))
            for i, (x, y) in enumerate(info["vertices"][:100]):     # a huge import: the first 100
                spin("Vertices", f"{i + 1} X", f"x{i}", x)
                spin("Vertices", f"{i + 1} Y", f"y{i}", y)
        for tree_item in panel._group_items.values():
            tree_item.setExpanded(True)
        panel.tree.resizeColumnToContents(0)
        panel.header.setText(f"<span style='font-size:8pt; letter-spacing:1px'>SKETCH</span>"
                             f"<br><b style='font-size:11pt'>{kind.capitalize()} {index + 1}</b>"
                             f"<br><span>Drag its grips on the canvas, or type exact numbers here.</span>")
        return True

    def selected_sketch_indices(self) -> List[int]:
        try:
            return sorted(i.index for i in self.window.scene.selectedItems() if isinstance(i, SketchItem))
        except RuntimeError:
            return []

    def selected_sketch_entities(self) -> List[SketchEntity]:
        """Copies of the selected sketch lines (the clipboard's Ctrl+C / Ctrl+X)."""
        return [self.config.sketch[i].model_copy(deep=True) for i in self.selected_sketch_indices()]

    def paste_sketch(self, entities: List[SketchEntity], dx: float, dy: float):
        """Ctrl+V: the copied sketch lines, moved by (dx, dy) - one undo step."""
        from cad import modify as cmod
        moved = [SketchEntity(points=pts, bulges=bul, closed=closed)
                 for pts, bul, closed in (cmod.move(e, dx, dy) for e in entities)]
        before = list(self.config.sketch)
        self.window.undo_stack.push(SketchCommand(self, before, before + moved, "Paste sketch"))

    def join_selected_sketch(self):
        """Join the selected sketch lines end to end into one (Modify > Join)."""
        indices = self.selected_sketch_indices()
        if len(indices) < 2:
            self._status("Join: select two or more sketch lines that meet end to end.")
            return
        try:
            pts, bul, closed = join_shapes([self.config.sketch[i] for i in indices])
        except ValueError as exc:
            self._status(f"Join: {exc}")
            return
        self.modify_tools["trim"].replace(indices, [(pts, bul, closed)], "Join")
        self._status(f"Joined {len(indices)} lines into one{' closed shape' if closed else ''}.")

    def explode_selected_sketch(self):
        """Split each selected sketch line into its single segments (Modify > Explode)."""
        from cad import modify as cmod
        indices = self.selected_sketch_indices()
        if not indices:
            self._status("Explode: select sketch lines first.")
            return
        parts = [part for i in indices for part in cmod.explode(self.config.sketch[i])]
        self.modify_tools["trim"].replace(indices, parts, "Explode")
        self._status(f"Exploded into {len(parts)} segments.")

    def clear_sketch(self):
        if self.config.sketch:
            self.window.undo_stack.push(SketchCommand(self, list(self.config.sketch), [], "Clear sketch"))

    # -- helpers ---------------------------------------------------------------

    def _layout_dir(self) -> str:
        path = self.window.config_path
        return os.path.dirname(os.path.abspath(path)) if path else os.getcwd()

    def _resolve(self, path: str) -> str:
        return path if os.path.isabs(path) else os.path.join(self._layout_dir(), path)

    def _status(self, message: str, ms: int = 8000):
        self.window.statusBar().showMessage(message, ms)

    def _error(self, title: str, message: str):
        QMessageBox.warning(self.window, title, message)

    def _refresh_panels(self):
        # Re-read the selected item's values into the Properties panel.
        panel = getattr(self.window, "properties_panel", None)
        if panel is not None:
            panel._on_selection_changed()

    def _push(self, text: str, commands: List[QUndoCommand]):
        stack = self.window.undo_stack
        stack.beginMacro(text)
        for cmd in commands:
            stack.push(cmd)
        stack.endMacro()

    def _geometry_command(self, new_geometry: Dict[str, dict], text: str) -> Optional[BeltGeometryCommand]:
        if not new_geometry:
            return None
        items = self.belt_items()
        before = {i: {f: getattr(items[i].model, f) for f in GEOMETRY_FIELDS} for i in new_geometry}
        return BeltGeometryCommand(self, before, new_geometry, text)

    def select_ids(self, ids, location=None):
        scene = self.window.scene
        scene.clearSelection()
        for item in self.window.items_list:
            if item.model.id in ids:
                item.setSelected(True)
        if location is not None:
            self.window.view.centerOn(self.to_scene_pos(*location))

    # -- actions ---------------------------------------------------------------

    def _build_actions(self, file_menu):
        """Commands like a CAD tool's: every one is a tool bar button (icon +
        tooltip with its shortcut; MainWindow gives the icons from
        `self.icon_names`) and a menu entry - File > Background, a flat Tools
        menu in sections (no sub-menus), View > Show Ports (+ Panels, added
        by MainWindow)."""
        w = self.window
        self.icon_names = []        # (action, toolbar_icons name, short label)

        def add(menu, text, slot, shortcut="", icon="", label=""):
            action = QAction(text, w)
            if shortcut:
                action.setShortcut(QKeySequence(shortcut))
            action.triggered.connect(slot)
            menu.addAction(action)
            if icon:
                self.icon_names.append((action, icon, label or text))
            return action

        file_menu.addSeparator()
        background = file_menu.addMenu("Background (DXF)")
        self.import_bg_action = add(background, "Import DXF Background...", self.import_background)
        self.place_bg_action = add(background, "Place Background", self.place_tool.start, "Shift+B", "bg_place",
                                   "Place")
        self.bg_settings_action = add(background, "Background Settings...", self.background_settings, "",
                                      "bg_settings", "Settings")
        self.crop_bg_action = add(background, "Export Cropped DXF...", self.export_cropped_background, "",
                                  "bg_crop", "Crop")
        background.addSeparator()
        self.remove_bg_action = add(background, "Remove Background", self.remove_background, "", "bg_remove",
                                    "Remove")
        self.export_dxf_action = add(file_menu, "Export DXF...", self.export_dxf)

        tools = w.menuBar().addMenu("Tools")
        tools.addSection("Sketch")
        sketch_actions = [
            add(tools, "Line", self.sketch_line_tool.start, "S", "line"),
            add(tools, "Arc", self.sketch_arc_tool.start, "Shift+S", "arc"),
            add(tools, "Arc (Center, Start, End)", lambda: self.sketch_arc_tool.start("center"), "Shift+A",
                "arc_center", "Arc Center"),
            add(tools, "Rectangle", self.sketch_rect_tool.start, "Shift+R", "rectangle"),
            add(tools, "Circle", self.sketch_circle_tool.start, "Shift+C", "circle"),
            add(tools, "Polygon", self.sketch_polygon_tool.start, "Shift+G", "polygon"),
            add(tools, "Clear Sketch", self.clear_sketch, "", "clear", "Clear"),
        ]
        tools.addSection("Modify")
        mt = self.modify_tools
        modify_actions = [
            add(tools, "Trim", mt["trim"].start, "T", "trim"),
            add(tools, "Extend", mt["extend"].start, "E", "extend"),
            add(tools, "Break", mt["break"].start, "K", "break"),
            add(tools, "Offset", mt["offset"].start, "Shift+O", "offset"),
            add(tools, "Fillet", mt["fillet"].start, "F", "fillet"),
            add(tools, "Join", self.join_selected_sketch, "J", "join"),
            add(tools, "Explode", self.explode_selected_sketch, "X", "explode"),
            None,
            add(tools, "Move", mt["move"].start, "M", "move"),
            add(tools, "Copy", mt["copy"].start, "Shift+M", "copy"),
            add(tools, "Rotate", mt["rotate"].start, "R", "rotate"),
            add(tools, "Mirror", mt["mirror"].start, "I", "mirror"),
            add(tools, "Scale", mt["scale"].start, "", "scale"),
        ]
        tools.addSection("Belts")
        belt_actions = [
            add(tools, "Belt from Sketch/Drawing", lambda: self.pick_tool.start("belt"), "B", "belt_pick",
                "Belt from Sketch"),
            add(tools, "Roller from Sketch/Drawing", lambda: self.pick_tool.start("roller"), "Ctrl+Shift+B",
                "roller_pick", "Roller from Sketch"),
            add(tools, "Divide into Belts", self.divide_tool.start, "N", "divide", "Divide"),
            add(tools, "Draw Belt Path", self.path_tool.start, "P", "belt_path", "Belt Path"),
            add(tools, "Draw Curved Belt", self.curved_tool.start, "U", "curved_belt", "Curved Belt"),
            add(tools, "Draw Belt Outline", self.outline_tool.start, "O", "outline_belt", "Outline Belt"),
        ]
        tools.addSection("Connections")
        self.connect_action = add(tools, "Connect Belts", self.connect_selected, "Ctrl+J", "connect", "Connect")
        self.fillet_belts_action = add(tools, "Fillet Belts...", self.fillet_selected, "Ctrl+Shift+F",
                                       "fillet_belts", "Fillet Belts")
        self.disconnect_action = add(tools, "Disconnect Belts", self.disconnect_selected, "Ctrl+Shift+J",
                                     "disconnect", "Disconnect")
        self.solve_action = add(tools, "Solve Connections", self.solve_all, "F6", "solve", "Solve")
        tools.addSeparator()
        self.check_action = add(tools, "Check Layout", self.run_check, "F7", "check", "Check")
        layout_actions = [self.connect_action, self.fillet_belts_action, self.disconnect_action, self.solve_action,
                          self.check_action]
        for action, _icon, label in self.icon_names:
            action.setIconText(label)
            key = action.shortcut().toString()
            action.setToolTip(f"{action.text()} ({key})" if key else action.text())

        # Sketch, belts and connections: on the same top row as File / View
        # (compact icons, one row for the whole tool set, as in a CAD tool).
        bar = QToolBar("Sketch", w)
        bar.setObjectName("sketchToolbar")
        w.addToolBar(Qt.TopToolBarArea, bar)
        for i, group in enumerate((sketch_actions, belt_actions, layout_actions)):
            if i:
                bar.addSeparator()
            for action in group:
                bar.addAction(action)
        self.sketch_toolbar = bar
        # Modify: a vertical tool bar beside the canvas, as in a CAD tool.
        mbar = QToolBar("Modify", w)
        mbar.setObjectName("modifyToolbar")
        mbar.setOrientation(Qt.Vertical)
        w.addToolBar(Qt.LeftToolBarArea, mbar)
        for action in modify_actions:
            if action is None:
                mbar.addSeparator()
                continue
            key = action.shortcut().toString()
            action.setToolTip(f"{action.text()} ({key}) - sketch lines" if key else f"{action.text()} - sketch lines")
            mbar.addAction(action)
        self.modify_toolbar = mbar
        # Background: placement and the drawing's settings (row 1, after File).
        bgbar = QToolBar("Background", w)
        bgbar.setObjectName("backgroundToolbar")
        for action in (self.place_bg_action, self.bg_settings_action, self.crop_bg_action, self.remove_bg_action):
            bgbar.addAction(action)
        self.background_toolbar = bgbar

        view = w.menuBar().addMenu("View")
        self.view_menu = view
        show_ports = QAction("Show Ports", w)
        show_ports.setCheckable(True)
        show_ports.setChecked(False)
        show_ports.toggled.connect(self._set_ports_visible)
        view.addAction(show_ports)
        self.show_ports_action = show_ports

    def _set_ports_visible(self, visible: bool):
        self.ports.visible = visible
        self.ports.refresh()

    def on_items_moved(self, items):
        """Called by PlacedItem.mouseReleaseEvent inside the drag's undo macro:
        keep every connection of the dragged belts attached."""
        moved = [i.model.id for i in items if isinstance(i, self.belt_item_cls)]
        if not moved or not self.config.connections:
            return
        involved = {c.fromBelt for c in self.config.connections} | {c.toBelt for c in self.config.connections}
        if not involved & set(moved):
            return
        try:
            geometry = solve_layout(self.belt_models(), self.config.connections, fixed_ids=moved)
        except ConstraintError as exc:
            self._status(f"Connections not kept: {exc}")
            return
        cmd = self._geometry_command(geometry, "Keep Connections")
        if cmd is not None:
            self.window.undo_stack.push(cmd)

    def add_belt_chain(self, belts, closed: bool, text: str):
        """Add belts drawn as one conveyor line, each joined to the next
        (cad.belts.belt_chain_connections) - one undo step."""
        stack = self.window.undo_stack
        stack.beginMacro(text)
        items = [self.window.view._add_item(self.belt_item_cls(b), text=text) for b in belts]
        before = list(self.config.connections)
        stack.push(ConnectionsCommand(self, before, before + belt_chain_connections(belts, closed),
                                      "Add connections"))
        stack.endMacro()
        self.window.scene.clearSelection()
        for item in items:
            item.setSelected(True)

    def connect_selected(self):
        selected = self.selected_belt_items()
        if len(selected) != 2:
            self._error("Connect Belts", "Select exactly two belts.")
            return
        a, b = self._order_pair(selected[0].model, selected[1].model)
        try:
            geometry = connect_belts(a, b, self.belt_models(), self.config.connections)
        except ConstraintError as exc:
            self._error("Connect Belts", str(exc))
            return
        before = list(self.config.connections)
        after = before + [BeltConnection(fromBelt=a.id, toBelt=b.id)]
        cmds = [c for c in (self._geometry_command(geometry, "Move belts"),) if c is not None]
        cmds.append(ConnectionsCommand(self, before, after, "Add connection"))
        self._push(f"Connect {a.id} -> {b.id}", cmds)
        self._status(f"Connected {a.id} -> {b.id}.")

    def _order_pair(self, x, y):
        """(upstream, downstream): whichever outlet is closer to the other's inlet."""
        px, py = belt_ports(x), belt_ports(y)
        if px is None or py is None:
            return x, y
        d_xy = math.hypot(px.outlet.x - py.inlet.x, px.outlet.y - py.inlet.y)
        d_yx = math.hypot(py.outlet.x - px.inlet.x, py.outlet.y - px.inlet.y)
        return (x, y) if d_xy <= d_yx else (y, x)

    def fillet_selected(self):
        selected = self.selected_belt_items()
        if len(selected) != 2:
            self._error("Fillet Belts", "Select exactly two straight belts.")
            return
        m1, m2 = selected[0].model, selected[1].model
        radius, ok = QInputDialog.getDouble(self.window, "Fillet Belts", "Curve radius (m):",
                                            max(m1.beltWidth, m2.beltWidth) * 1.5, 0.05, 1000.0, 3)
        if not ok:
            return
        curve_id = next_indexed_id([i.model.id for i in self.window.items_list], "belt")
        result, first_error = None, None
        for a, b in ((m1, m2), (m2, m1)):  # whichever order flows into the corner
            try:
                result = fillet_belts(a, b, radius, curve_id)
                break
            except ConstraintError as exc:
                first_error = first_error or exc
        if result is None:
            self._error("Fillet Belts", str(first_error))
            return
        a_id = result.connections[0].fromBelt
        b_id = result.connections[1].toBelt
        geometry_cmd = self._geometry_command({a_id: result.upstream, b_id: result.downstream}, "Trim belts")
        curve_item = self.belt_item_cls(result.curve)
        add_cmd = _AddItem(self.window.view, curve_item)
        before = list(self.config.connections)
        after = [c for c in before if not (c.fromBelt == a_id and c.toBelt == b_id)] + result.connections
        cmds = [c for c in (geometry_cmd, add_cmd) if c is not None]
        cmds.append(ConnectionsCommand(self, before, after, "Connect curve"))
        self._push(f"Fillet {a_id} -> {b_id} (R {radius:g} m)", cmds)
        self._status(f"Added {curve_id} (radius {radius:g} m) between {a_id} and {b_id}.")

    def disconnect_selected(self):
        ids = {i.model.id for i in self.selected_belt_items()}
        if not ids:
            self._error("Disconnect Belts", "Select one or more belts.")
            return
        before = list(self.config.connections)
        if len(ids) == 1:
            after = [c for c in before if c.fromBelt not in ids and c.toBelt not in ids]
        else:
            after = [c for c in before if not (c.fromBelt in ids and c.toBelt in ids)]
        if len(after) == len(before):
            self._status("No connections between the selected belts.")
            return
        self._push("Disconnect belts", [ConnectionsCommand(self, before, after, "Remove connections")])
        self._status(f"Removed {len(before) - len(after)} connection(s).")

    def solve_all(self):
        fixed = [i.model.id for i in self.selected_belt_items()]
        try:
            geometry = solve_layout(self.belt_models(), self.config.connections, fixed_ids=fixed)
        except ConstraintError as exc:
            self._error("Solve Connections", str(exc))
            return
        cmd = self._geometry_command(geometry, "Solve connections")
        if cmd is None:
            self._status("All connections already hold.")
            return
        self._push("Solve connections", [cmd])
        self._status(f"Re-attached {len(geometry)} belt(s).")

    def _typed_input(self, text: str):
        tool = getattr(self.window.view, "tool", None)
        if tool is not None and hasattr(tool, "typed_input"):
            tool.typed_input(text)

    def run_check(self):
        issues = check_layout(self.window._build_export_config())
        self.check_panel.show_issues(issues)
        errors = sum(1 for i in issues if i.severity == "error")
        self._status(f"Layout check: {errors} error(s), {len(issues) - errors} warning(s).")
        return issues

    def import_background(self):
        path, _ = QFileDialog.getOpenFileName(self.window, "Import DXF Background", self._layout_dir(),
                                              "DXF Files (*.dxf)")
        if not path:
            return
        try:
            stored = os.path.relpath(path, self._layout_dir())
        except ValueError:  # different drive on Windows
            stored = path
        self._background_dialog(path, stored, None, "Import background")

    def background_settings(self):
        """Units, window, placement and layers of the current background."""
        current = self.config.background
        if current is None:
            self._status("No background: File > Import DXF Background... first.")
            return
        self._background_dialog(self._resolve(current.path), current.path, current, "Background settings")

    def _background_dialog(self, path: str, stored: str, current: Optional[BackgroundDrawing], text: str):
        self._status(f"Reading {os.path.basename(path)}...", 0)
        try:
            with _busy():
                info = dxf_io.inspect_drawing(path)
        except (OSError, IOError, ValueError, ezdxf.DXFError) as exc:
            self._status("")
            self._error("Import DXF Background", f"Could not read {path}:\n{exc}")
            return
        self._status("")
        dialog = BackgroundImportDialog(self.window, info, stored, current, source_path=path,
                                        layout_dir=self._layout_dir())
        if dialog.exec() != QDialog.Accepted:
            return
        background = dialog.background()
        before = self.config.background.model_copy() if self.config.background is not None else None
        self._push(text, [BackgroundCommand(self, before, background, "Background")])
        count = len(self._background_shapes)
        where = "one window" if background.clip else "whole drawing"
        self._status(f"Background: {count:,} entities ({where}), scale {background.scale:g} m/unit, offset "
                     f"({background.offsetX:.4g}, {background.offsetY:.4g}) m. File > Background Settings... "
                     f"to change.")

    def export_cropped_background(self):
        """The current background's window and filters, written as a small
        DXF (same units and coordinates); offers to switch the background to it."""
        bg = self.config.background
        if bg is None:
            self._status("No background: File > Import DXF Background... first.")
            return
        src = self._resolve(bg.path)
        base = os.path.splitext(os.path.basename(src))[0]
        dst, _ = QFileDialog.getSaveFileName(self.window, "Export Cropped DXF",
                                             os.path.join(self._layout_dir(), f"{base}_crop.dxf"), "DXF Files (*.dxf)")
        if not dst:
            return
        try:
            with _busy():
                result = dxf_io.export_cropped(src, dst, bg.clip, skip_types=bg.skipTypes,
                                               hidden_layers=bg.hiddenLayers)
        except (OSError, ValueError, ezdxf.DXFError) as exc:
            self._error("Export Cropped DXF", f"Could not write {dst}:\n{exc}")
            return
        answer = QMessageBox.question(
            self.window, "Export Cropped DXF",
            f"Wrote {result.shapes:,} shapes and {result.texts:,} texts to\n{dst}\n\n"
            f"Use the cropped file as the background?")
        if answer == QMessageBox.Yes:
            self.use_cropped_background(dst)
        else:
            self._status(f"Exported {dst}")

    def use_cropped_background(self, path: str):
        """Switch the background file to a cropped copy (same coordinates, so
        placement, window and filters carry over unchanged)."""
        bg = self.config.background
        try:
            stored = os.path.relpath(path, self._layout_dir())
        except ValueError:
            stored = path
        after = bg.model_copy(update={"path": stored})
        self._push("Use cropped background", [BackgroundCommand(self, bg.model_copy(), after, "Background")])
        self._status(f"Background is now {stored} ({len(self._background_shapes):,} entities).")

    def move_background(self, dx: float, dy: float, text: str = "Place background"):
        """Shift the background drawing by (dx, dy) metres - one undo step."""
        bg = self.config.background
        if bg is None or (abs(dx) < 1e-12 and abs(dy) < 1e-12):
            return
        after = bg.model_copy(update={"offsetX": bg.offsetX + dx, "offsetY": bg.offsetY + dy})
        self._push(text, [BackgroundCommand(self, bg.model_copy(), after, "Background")])
        self._status(f"Background moved by ({dx:.4g}, {dy:.4g}) m; offset now ({after.offsetX:.4g}, "
                     f"{after.offsetY:.4g}) m.")

    def remove_background(self):
        if self.config.background is None:
            return
        self._push("Remove background", [BackgroundCommand(self, self.config.background, None, "Background")])

    def export_dxf(self):
        start = os.path.splitext(self.window.config_path or "layout.json")[0] + ".dxf"
        path, _ = QFileDialog.getSaveFileName(self.window, "Export DXF", start, "DXF Files (*.dxf)")
        if not path:
            return
        try:
            dxf_io.export_layout(self.window._build_export_config(), path, background=self._background_shapes)
        except OSError as exc:
            self._error("Export DXF", f"Could not write {path}:\n{exc}")
            return
        self._status(f"Exported {path}")


class SketchCommand(QUndoCommand):
    def __init__(self, controller, before: List[SketchEntity], after: List[SketchEntity], text: str):
        super().__init__(text)
        self.controller = controller
        self.before = [e.model_copy() for e in before]
        self.after = [e.model_copy() for e in after]

    def redo(self):
        self.controller.set_sketch(self.after)

    def undo(self):
        self.controller.set_sketch(self.before)


class SketchItem(QGraphicsPathItem):
    """One sketch entity on the canvas: selectable (Delete removes it), not
    movable, drawn as thin cyan construction geometry."""

    def __init__(self, entity: SketchEntity, index: int, to_scene_pos):
        super().__init__(self._path(entity, to_scene_pos))
        self.entity = entity
        self.index = index
        self.setPen(QPen(QColor("#22d3ee"), 0))
        self.setZValue(SKETCH_Z)
        self.setFlag(QGraphicsPathItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setToolTip("Sketch geometry - Tools > Belt from Sketch/Drawing (B) turns it into a belt; Delete removes it")

    @staticmethod
    def _path(entity: SketchEntity, to_scene_pos) -> QPainterPath:
        flat, _ = path_flatten(path_segments(entity.points, entity.bulges, entity.closed), 0.01)
        if entity.closed and flat:
            flat.append(flat[0])
        path = QPainterPath()
        path.addPolygon(QPolygonF([to_scene_pos(x, y) for x, y in flat]))
        return path

    def set_entity(self, entity: SketchEntity, to_scene_pos):
        self.entity = entity
        self.setPath(self._path(entity, to_scene_pos))

    def shape(self) -> QPainterPath:
        stroker = QPainterPathStroker()
        stroker.setWidth(10.0)  # easy to click a thin line
        return stroker.createStroke(self.path())

    def paint(self, painter, option, widget=None):
        selected = bool(option.state & QStyle.State_Selected)
        painter.setPen(QPen(QColor("#facc15" if selected else "#22d3ee"), 2 if selected else 1.5))
        painter.drawPath(self.path())


class _AddItem(QUndoCommand):
    """Add an item through the view's own add/remove bookkeeping."""

    def __init__(self, view, item):
        super().__init__("Add curve")
        self.view = view
        self.item = item

    def redo(self):
        self.view._do_add_item(self.item)

    def undo(self):
        self.view._do_remove_item(self.item)
