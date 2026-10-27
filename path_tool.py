"""
path_tool.py - draw and edit path belts in the layout editor.

Draw Belt Path (Tools menu, P):
  click            add an anchor (snaps to component key points / grid)
  Shift            polar tracking: segment angle snaps to 15 deg steps
  L / A            next segments are straight lines / tangent arcs
  typed input      "x,y"  "@dx,dy"  "len<angle"  "len" (along the cursor),
                   Enter - in the Path toolbar's input box (typing a digit
                   while drawing jumps there)
  click 1st anchor close the path into a loop (also: C)
  Enter / right-click / double-click   finish an open path
  Backspace        remove the last anchor
  Esc              cancel
The preview shows the belt surface (centerline offset by the belt width),
the current segment's length and angle, and the total path length. Sharp
corners are rounded with the toolbar's corner radius when the path is
finished.

Insert Belt Shape (Shift+P): triangle / rectangle / polygon / circle loops,
U-turn, L-bend, S-bend from cad/path.py presets.

Editing a drawn belt: select it to show its anchor handles.
  drag handle          move the anchor
  Ctrl+click handle    toggle the segment after it: straight <-> tangent arc
  Shift+click handle   insert an anchor halfway along the segment after it
  Alt+click handle     delete the anchor
All edits are undoable.

The geometry (bulges, tangent arcs, fillets, offset, typed input) lives in
cad/path.py; this module is only the Qt interaction around it.
"""
import math
from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QPainterPath, QPen, QPolygonF, QUndoCommand
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QGraphicsItem,
    QGraphicsPathItem, QGraphicsRectItem, QGraphicsSimpleTextItem, QLabel, QLineEdit, QSpinBox,
    QToolBar, QToolButton,
)

from cad import path as cpath
from config_parser import Belt, PathNode
from id_utils import next_indexed_id

PREVIEW_Z = 1900
GRIP_PX = 9
CLOSE_TOLERANCE_PX = 12


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


# ---------------------------------------------------------------------------
# Toolbar
# ---------------------------------------------------------------------------

class PathToolbar(QToolBar):
    """Options for the draw tool: segment mode, belt width, corner radius,
    and the typed-coordinate input."""

    def __init__(self, parent, on_input: Callable[[str], None]):
        super().__init__("Draw Path", parent)
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
        self.corner_spin.setToolTip("Sharp corners are rounded with this radius when the path is finished (0 = sharp)")
        self.addWidget(self.corner_spin)
        self.addSeparator()
        self.input = QLineEdit()
        self.input.setPlaceholderText("x,y   @dx,dy   len<angle   len   (Enter)")
        self.input.setMinimumWidth(240)
        self.input.returnPressed.connect(lambda: on_input(self.input.text()))
        self.addWidget(self.input)
        self.setEnabled(False)

    @property
    def arc_mode(self) -> bool:
        return self.arc_button.isChecked()

    def set_arc_mode(self, arc: bool):
        (self.arc_button if arc else self.line_button).setChecked(True)


# ---------------------------------------------------------------------------
# Draw tool
# ---------------------------------------------------------------------------

class PathDrawTool:
    """Installed on the CanvasView as `view.tool` while drawing; the view
    forwards mouse and key events to it first."""

    def __init__(self, window, *, belt_item_cls, to_scene_pos, from_scene_pos, snap_free_point, scale):
        self.window = window
        self.view = window.view
        self.belt_item_cls = belt_item_cls
        self.to_scene = to_scene_pos
        self.from_scene = from_scene_pos
        self.snap_free_point = snap_free_point
        self.scale = scale
        self.toolbar = PathToolbar(window, self.typed_input)
        window.addToolBar(Qt.TopToolBarArea, self.toolbar)
        self.active = False
        self.anchors: List[Tuple[float, float]] = []
        self.bulges: List[float] = []
        self.cursor: Optional[Tuple[float, float]] = None
        self._preview_items = []
        self.on_finished: Optional[Callable[[Belt], None]] = None

    # -- lifecycle -------------------------------------------------------------

    def start(self):
        if self.active:
            return
        self.active = True
        self.anchors, self.bulges, self.cursor = [], [], None
        self.view.tool = self
        self.view.setDragMode(self.view.DragMode.NoDrag)
        self.view.viewport().setCursor(Qt.CrossCursor)
        self.window.scene.clearSelection()
        self.toolbar.setEnabled(True)
        self._status("Draw Belt Path: click anchors. L/A line/arc, Shift = 15 deg, type coordinates, "
                     "click the first anchor to close, Enter/right-click to finish, Esc to cancel.")

    def stop(self):
        self.active = False
        self._clear_preview()
        if getattr(self.view, "tool", None) is self:
            self.view.tool = None
        self.view.setDragMode(self.view.DragMode.RubberBandDrag)
        self.view.viewport().unsetCursor()
        self.toolbar.setEnabled(False)
        self.toolbar.input.clear()

    def _status(self, text: str, ms: int = 0):
        self.window.statusBar().showMessage(text, ms)

    # -- geometry --------------------------------------------------------------

    def _snapped_world(self, view_pos, shift: bool) -> Tuple[float, float]:
        scene_pos = self.view.mapToScene(view_pos)
        points, edges = self.view._all_points_and_edges()
        snapped = self.snap_free_point(scene_pos, points, edges, self.view._snap_tolerance_scene_units())
        world = self.from_scene(snapped)
        if shift and self.anchors:
            world = cpath.snap_angle(self.anchors[-1], self.from_scene(scene_pos), 15.0)
        return world

    def _incoming_heading(self) -> Optional[float]:
        if len(self.anchors) < 2:
            return None
        segs = cpath.segments(self.anchors, self.bulges, False)
        return cpath.end_heading(segs)

    def _bulge_to(self, p) -> float:
        heading = self._incoming_heading()
        if self.toolbar.arc_mode and heading is not None:
            return cpath.tangent_bulge(self.anchors[-1], heading, p)
        return 0.0

    def add_anchor(self, p: Tuple[float, float]):
        if self.anchors and math.hypot(p[0] - self.anchors[-1][0], p[1] - self.anchors[-1][1]) < 1e-6:
            return  # same point twice (e.g. the second click of a double-click)
        if self.anchors:
            self.bulges[-1] = self._bulge_to(p)
        self.anchors.append(p)
        self.bulges.append(0.0)
        self._update_preview()

    def _near_first(self, view_pos) -> bool:
        if len(self.anchors) < 2:
            return False
        first = self.view.mapFromScene(self.to_scene(*self.anchors[0]))
        return (first - view_pos).manhattanLength() <= CLOSE_TOLERANCE_PX

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
        (cx, cy), local = cpath.recenter(pts)
        existing = [i.model.id for i in self.window.items_list]
        belt = Belt(
            id=next_indexed_id(existing, "belt"), type="path", shape="path", x=cx, y=cy, rotation=0.0,
            beltWidth=self.toolbar.width_spin.value(), speed=1.0, directionAngle=0.0, color="#3b82f6",
            path=[PathNode(x=x, y=y, bulge=b) for (x, y), b in zip(local, bulges)], closed=closed,
        )
        self.stop()
        item = self.view._add_item(self.belt_item_cls(belt), text="Draw Belt Path")
        item.setSelected(True)
        length = cpath.path_length(cpath.segments(pts, bulges, closed))
        self._status(f"Drew {belt.id}: {'loop' if closed else 'open path'}, {len(pts)} anchors, "
                     f"{length:.2f} m centerline.", 8000)
        if self.on_finished:
            self.on_finished(belt)

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

    # -- events (forwarded by CanvasView) ----------------------------------------

    def mouse_press(self, event) -> bool:
        if event.button() == Qt.LeftButton:
            if self._near_first(event.position().toPoint()):
                self.close_path()
            else:
                self.add_anchor(self._snapped_world(event.position().toPoint(), bool(event.modifiers() & Qt.ShiftModifier)))
            return True
        if event.button() == Qt.RightButton:
            self.finish(closed=False)
            return True
        return False

    def mouse_move(self, event) -> bool:
        self.cursor = self._snapped_world(event.position().toPoint(), bool(event.modifiers() & Qt.ShiftModifier))
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
            self._status("Draw Belt Path cancelled.", 4000)
        elif key in (Qt.Key_Return, Qt.Key_Enter):
            self.finish(closed=False)
        elif key == Qt.Key_Backspace:
            if self.anchors:
                self.anchors.pop()
                self.bulges.pop()
                if self.bulges:
                    self.bulges[-1] = 0.0
            self._update_preview()
        elif key == Qt.Key_A:
            self.toolbar.set_arc_mode(True)
            self._update_preview()
        elif key == Qt.Key_L:
            self.toolbar.set_arc_mode(False)
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
            if item.scene() is not None:
                item.scene().removeItem(item)
        self._preview_items = []

    def _add_preview(self, item):
        self.window.scene.addItem(_inert(item))
        self._preview_items.append(item)
        return item

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        pts, bulges = list(self.anchors), list(self.bulges)
        if self.cursor is not None and pts:
            bulges[-1] = self._bulge_to(self.cursor)
            pts.append(self.cursor)
            bulges.append(0.0)
        width = self.toolbar.width_spin.value()
        if len(pts) >= 2:
            surface = cpath.footprint(pts, bulges, False, width, tolerance=0.005)
            body = self._add_preview(QGraphicsPathItem(_world_painter_path(surface, self.to_scene)))
            body.setBrush(QBrush(QColor(59, 130, 246, 90)))
            body.setPen(QPen(QColor(147, 197, 253), 0))
            flat, _ = cpath.flatten(cpath.segments(pts, bulges, False), tolerance=0.01)
            center = QPainterPath()
            center.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in flat]))
            line = self._add_preview(QGraphicsPathItem(center))
            line.setPen(QPen(QColor(255, 140, 0), 0, Qt.DashLine))
        for i, (x, y) in enumerate(self.anchors):
            p = self.to_scene(x, y)
            marker = self._add_preview(QGraphicsRectItem(QRectF(-4, -4, 8, 8)))
            marker.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            marker.setPos(p)
            marker.setPen(QPen(QColor("#22d3ee" if i == 0 else "#ffffff"), 1.5))
        if self.cursor is not None and self.anchors:
            last = self.anchors[-1]
            dx, dy = self.cursor[0] - last[0], self.cursor[1] - last[1]
            seg_len, seg_ang = math.hypot(dx, dy), math.degrees(math.atan2(dy, dx))
            total = cpath.path_length(cpath.segments(pts, bulges, False))
            mode = "arc" if self.toolbar.arc_mode and self._incoming_heading() is not None else "line"
            label = self._add_preview(QGraphicsSimpleTextItem(
                f"{mode} {seg_len:.3f} m < {seg_ang:.1f} deg   total {total:.3f} m"))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            label.setBrush(QBrush(QColor(255, 255, 255)))
            label.setFont(QFont("Segoe UI", 9))
            label.setPos(self.to_scene(*self.cursor) + QPointF(12, 12))


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
# Anchor editing
# ---------------------------------------------------------------------------

class PathEditCommand(QUndoCommand):
    def __init__(self, grips, item, before: List[PathNode], after: List[PathNode], text: str):
        super().__init__(text)
        self.grips = grips
        self.item = item
        self.before = [n.model_copy() for n in before]
        self.after = [n.model_copy() for n in after]

    def _apply(self, nodes):
        self.item.model.path = [n.model_copy() for n in nodes]
        self.item.sync_from_model()
        self.grips.refresh()
        for view in self.item.scene().views() if self.item.scene() else []:
            if hasattr(view, "invalidate_points_edges_cache"):
                view.invalidate_points_edges_cache()

    def redo(self):
        self._apply(self.after)

    def undo(self):
        self._apply(self.before)


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
        self.setToolTip("Drag: move anchor | Ctrl+click: straight/arc | Shift+click: insert | Alt+click: delete")
        self._before = None

    def mousePressEvent(self, event):
        mods = event.modifiers()
        nodes = self.belt_item.model.path
        if mods & Qt.ControlModifier:
            self.manager.toggle_arc(self.belt_item, self.index)
        elif mods & Qt.ShiftModifier:
            self.manager.insert_after(self.belt_item, self.index)
        elif mods & Qt.AltModifier:
            self.manager.delete_anchor(self.belt_item, self.index)
        else:
            self._before = [n.model_copy() for n in nodes]
        event.accept()

    def mouseMoveEvent(self, event):
        if self._before is None:
            return
        local = self.belt_item.mapFromScene(self.manager.snap_scene(event.scenePos()))
        node = self.belt_item.model.path[self.index]
        node.x, node.y = local.x() / self.manager.scale, -local.y() / self.manager.scale
        self.belt_item.sync_from_model()
        self.setPos(local)
        event.accept()

    def mouseReleaseEvent(self, event):
        if self._before is None:
            return
        before, self._before = self._before, None
        after = [n.model_copy() for n in self.belt_item.model.path]
        if any((a.x, a.y) != (b.x, b.y) for a, b in zip(after, before)):
            self.manager.push(self.belt_item, before, after, "Move anchor")
        event.accept()


class AnchorGripManager:
    """Shows anchor handles on the single selected path belt."""

    def __init__(self, window, *, belt_item_cls, scale, snap_free_point):
        self.window = window
        self.belt_item_cls = belt_item_cls
        self.scale = scale
        self.snap_free_point = snap_free_point
        self.grips: List[AnchorGrip] = []
        self.item = None
        window.scene.selectionChanged.connect(self.refresh)

    def snap_scene(self, scene_pos: QPointF) -> QPointF:
        view = self.window.view
        points, edges = view._all_points_and_edges()
        return self.snap_free_point(scene_pos, points, edges, view._snap_tolerance_scene_units())

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
        self.item = selected[0] if len(selected) == 1 and selected[0].is_path() else None
        if self.item is None:
            return
        for i, p in enumerate(self.item.path_anchor_scene_points()):
            grip = AnchorGrip(self, self.item, i)
            grip.setPos(p)
            self.grips.append(grip)

    def push(self, item, before, after, text):
        self.window.undo_stack.push(PathEditCommand(self, item, before, after, text))

    def toggle_arc(self, item, index: int):
        nodes = item.model.path
        n = len(nodes)
        if index == n - 1 and not item.model.closed:
            self.window.statusBar().showMessage("The last anchor of an open path has no segment after it.", 5000)
            return
        after = [x.model_copy() for x in nodes]
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
        self.push(item, nodes, after, "Toggle arc")

    def insert_after(self, item, index: int):
        nodes = item.model.path
        n = len(nodes)
        if index == n - 1 and not item.model.closed:
            return
        seg = cpath.bulge_to_segment((nodes[index].x, nodes[index].y),
                                     (nodes[(index + 1) % n].x, nodes[(index + 1) % n].y), nodes[index].bulge)
        mx, my = seg.point_at(0.5)
        half = math.tan(math.atan(nodes[index].bulge) / 2.0)  # each half of an arc keeps the curve
        after = [x.model_copy() for x in nodes]
        after[index].bulge = half
        after.insert(index + 1, PathNode(x=mx, y=my, bulge=half))
        self.push(item, nodes, after, "Insert anchor")

    def delete_anchor(self, item, index: int):
        nodes = item.model.path
        minimum = 3 if item.model.closed else 2
        if len(nodes) <= minimum:
            self.window.statusBar().showMessage(f"A {'loop' if item.model.closed else 'path'} needs at least "
                                                f"{minimum} anchors.", 5000)
            return
        after = [x.model_copy() for x in nodes]
        del after[index]
        if index > 0:
            after[index - 1].bulge = 0.0  # the merged segment becomes straight
        self.push(item, nodes, after, "Delete anchor")
