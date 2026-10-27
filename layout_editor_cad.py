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

from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import (
    QAction, QBrush, QColor, QKeySequence, QPainterPath, QPainterPathStroker, QPen, QPolygonF, QUndoCommand,
)
from PySide6.QtWidgets import (
    QApplication, QDialog, QFileDialog, QGraphicsEllipseItem, QGraphicsPathItem, QGraphicsPolygonItem, QInputDialog,
    QListWidget, QListWidgetItem, QMessageBox, QPushButton, QStyle, QVBoxLayout, QWidget,
)

from background_dialog import BackgroundImportDialog
import ezdxf

from cad import dxf_io
from cad.checks import check_layout
from cad.constraints import (
    GEOMETRY_FIELDS, ConstraintError, connect_belts, fillet_belts, solve_layout,
)
from cad.ports import belt_ports
from config_parser import BackgroundDrawing, BeltConnection, SketchEntity
from id_utils import next_indexed_id
from path_tool import (
    AnchorGripManager, OutlineDrawTool, PathDrawTool, PathToolbar, PickFromDrawingTool, SketchArcTool,
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
        window.addToolBar(Qt.TopToolBarArea, self.draw_toolbar)
        self.path_tool = PathDrawTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
        self.outline_tool = OutlineDrawTool(window, self.draw_toolbar, self.snap, belt_item_cls=belt_item_cls)
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
        self.sketch_items: List[SketchItem] = []
        self._background_shapes = []
        window.view.on_delete_extra = self._delete_selected_sketch
        self.grips = AnchorGripManager(window, belt_item_cls=belt_item_cls, scale=scale, snap=self.snap)
        self.ports = PortOverlay(self)
        self.closing = False  # set by MainWindow.closeEvent: stop touching the scene
        self.background_item: Optional[BackgroundItem] = None
        self.check_panel = LayoutCheckPanel(self)
        self._build_actions(file_menu)
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
        self.config.background = background
        self._background_shapes = []
        self._refresh_snap_shapes()
        if background is None or not background.visible:
            return
        path = self._resolve(background.path)
        try:
            with _busy():
                geom = dxf_io.read_background(path, clip=background.clip, skip_types=background.skipTypes,
                                              hidden_layers=background.hiddenLayers)
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
        w = self.window
        file_menu.addSeparator()
        for text, slot in (("Import DXF Background...", self.import_background),
                           ("Background Settings...", self.background_settings),
                           ("Remove Background", self.remove_background),
                           ("Export DXF...", self.export_dxf)):
            action = QAction(text, w)
            action.triggered.connect(slot)
            file_menu.addAction(action)

        tools = w.menuBar().addMenu("Tools")
        for text, shortcut, slot in (
            ("Sketch Line", "S", self.sketch_line_tool.start),
            ("Sketch Arc (3 points)", "Shift+S", self.sketch_arc_tool.start),
            ("Sketch Rectangle", "Shift+R", self.sketch_rect_tool.start),
            ("Sketch Circle", "Shift+C", self.sketch_circle_tool.start),
            ("Sketch Polygon", "Shift+G", self.sketch_polygon_tool.start),
            ("Clear Sketch", "", self.clear_sketch),
            (None, None, None),
            ("Belt from Sketch/Drawing", "B", self.pick_tool.start),
            ("Draw Belt Path", "P", self.path_tool.start),
            ("Draw Belt Outline", "O", self.outline_tool.start),
        ):
            if text is None:
                tools.addSeparator()
                continue
            action = QAction(text, w)
            action.setShortcut(QKeySequence(shortcut))
            action.triggered.connect(slot)
            tools.addAction(action)
        tools.addSeparator()
        for text, shortcut, slot in (
            ("Connect Belts", "Ctrl+J", self.connect_selected),
            ("Fillet Belts...", "Ctrl+Shift+F", self.fillet_selected),
            ("Disconnect Belts", "Ctrl+Shift+J", self.disconnect_selected),
            ("Solve Connections", "F6", self.solve_all),
            ("Check Layout", "F7", self.run_check),
        ):
            action = QAction(text, w)
            action.setShortcut(QKeySequence(shortcut))
            action.triggered.connect(slot)
            tools.addAction(action)

        view = w.menuBar().addMenu("View")
        show_ports = QAction("Show Ports", w)
        show_ports.setCheckable(True)
        show_ports.setChecked(False)
        show_ports.toggled.connect(self._set_ports_visible)
        view.addAction(show_ports)

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
        dialog = BackgroundImportDialog(self.window, info, stored, current)
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
            dxf_io.export_layout(self.window._build_export_config(), path)
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
        flat, _ = path_flatten(path_segments(entity.points, entity.bulges, entity.closed), 0.01)
        if entity.closed and flat:
            flat.append(flat[0])
        path = QPainterPath()
        path.addPolygon(QPolygonF([to_scene_pos(x, y) for x, y in flat]))
        super().__init__(path)
        self.entity = entity
        self.index = index
        self.setPen(QPen(QColor("#22d3ee"), 0))
        self.setZValue(SKETCH_Z)
        self.setFlag(QGraphicsPathItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setToolTip("Sketch geometry - Tools > Belt from Sketch/Drawing (B) turns it into a belt; Delete removes it")

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
