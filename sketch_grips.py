"""
sketch_grips.py - CAD-style grip editing of selected sketch geometry.

Select sketch lines/arcs/circles (click, or a window) and handles appear:

  Line / polyline   squares on the vertices: drag to move that point
                    (an arc next to it bends to follow); a grip at the
                    middle of each straight segment moves the segment
                    parallel, one on each arc segment bends it through
                    the dragged point
  Arc               its two end points, its center (moves the arc) and
                    the middle of the arc (drag to change the radius,
                    center and angles kept)
  Circle            its center (moves it) and four quadrant points (drag
                    to change the radius)

Grips snap like the drawing tools (OSNAP, then the grid) to everything
except the entity being edited. The sketch updates live while dragging;
the drop is one undo step. The Properties panel edits the same numbers
exactly (layout_editor_cad.sketch_property_rows).
"""
import math
from typing import Callable, List, Optional, Tuple

from PySide6.QtCore import QRectF, Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QPen
from PySide6.QtWidgets import QGraphicsItem, QGraphicsRectItem

from cad import modify as cmod
from cad.path import segments as path_segments
from path_tool import GRIP_PX, PREVIEW_Z

MAX_GRIP_ITEMS = 10          # more selected than this: no grips (too busy)

Point = Tuple[float, float]
Edit = Callable[[object, Point], object]     # (shape before the drag, point) -> new shape

VERTEX, MIDDLE, CENTER, RADIUS = "#ffffff", "#7dd3fc", "#fb923c", "#fde047"


def grip_specs(shape) -> List[Tuple[Point, str, Edit, str]]:
    """(position, color, edit, tooltip) of every grip on a shape."""
    shape = cmod.as_shape(shape)
    pts, _bul, closed = shape
    kind = cmod.kind_of(shape)
    specs = []
    if kind == "circle":
        c, r = cmod.circle_info(shape)
        specs.append((c, CENTER, lambda s, p, c=c: cmod.move(s, p[0] - c[0], p[1] - c[1]), "Drag: move the circle"))
        for a in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
            specs.append(((c[0] + r * math.cos(a), c[1] + r * math.sin(a)), RADIUS,
                          lambda s, p, c=c: cmod.make_circle(c, max(math.dist(p, c), 1e-6)),
                          "Drag: change the radius"))
        return specs
    if kind == "arc":
        seg = cmod.single_arc(shape)
        c = seg.center
        start, sweep = math.degrees(seg.start_angle), math.degrees(seg.sweep)
        specs.append((c, CENTER, lambda s, p, c=c: cmod.move(s, p[0] - c[0], p[1] - c[1]), "Drag: move the arc"))
        specs.append((seg.point_at(0.5), RADIUS,
                      lambda s, p, c=c: cmod.make_arc(c, max(math.dist(p, c), 1e-6), start, sweep),
                      "Drag: change the radius (center and angles kept)"))
    segs = path_segments(*shape)
    n = len(pts)
    for i, p in enumerate(pts):
        specs.append((p, VERTEX, lambda s, q, i=i: cmod.move_vertices(s, {i: q}), "Drag: move this point"))
    if kind == "arc":
        return specs
    for k, seg in enumerate(segs):
        mid = seg.point_at(0.5)
        if seg.kind == "arc":
            specs.append((mid, MIDDLE, lambda s, q, k=k: cmod.bend_segment(s, k, q), "Drag: bend this arc"))
        else:
            a, b = k, (k + 1) % n
            specs.append((mid, MIDDLE, lambda s, q, a=a, b=b, mid=mid: _move_segment(s, a, b, q[0] - mid[0], q[1] - mid[1]),
                          "Drag: move this segment"))
    return specs


def _move_segment(shape, a: int, b: int, dx: float, dy: float):
    pts = shape[0]
    return cmod.move_vertices(shape, {a: (pts[a][0] + dx, pts[a][1] + dy), b: (pts[b][0] + dx, pts[b][1] + dy)})


class SketchGrip(QGraphicsRectItem):
    def __init__(self, manager, index: int, number: int, color: str, tip: str):
        super().__init__(QRectF(-GRIP_PX / 2, -GRIP_PX / 2, GRIP_PX, GRIP_PX))
        self.manager = manager
        self.index = index            # the sketch entity
        self.number = number          # which of its grips
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        self.setZValue(PREVIEW_Z)
        self.setBrush(QBrush(QColor(color)))
        self.setPen(QPen(QColor("#0f172a"), 1))
        self.setCursor(Qt.SizeAllCursor)
        self.setToolTip(tip)

    def mousePressEvent(self, event):
        if event.button() != Qt.LeftButton:
            event.ignore()
            return
        self.manager.begin(self)
        event.accept()

    def mouseMoveEvent(self, event):
        self.manager.drag(self, event.scenePos())
        event.accept()

    def mouseReleaseEvent(self, event):
        self.manager.finish(self)
        event.accept()


class SketchGripManager:
    """Grips on the selected sketch items (CadController.sketch_items)."""

    def __init__(self, cad):
        self.cad = cad
        self.window = cad.window
        self.grips: List[SketchGrip] = []
        self._drag: Optional[dict] = None
        self.window.scene.selectionChanged.connect(self.refresh)

    def clear(self):
        for grip in self.grips:
            try:
                if grip.scene() is not None:
                    grip.scene().removeItem(grip)
            except RuntimeError:
                pass
        self.grips = []

    def refresh(self):
        if self._drag is not None or getattr(self.cad, "closing", False):
            return
        self.clear()
        indices = self.cad.selected_sketch_indices()
        if not indices or len(indices) > MAX_GRIP_ITEMS:
            return
        for index in indices:
            for number, (p, color, _edit, tip) in enumerate(grip_specs(self.cad.config.sketch[index])):
                grip = SketchGrip(self, index, number, color, tip)
                grip.setPos(self.cad.to_scene_pos(*p))
                self.window.scene.addItem(grip)
                self.grips.append(grip)

    # -- dragging ----------------------------------------------------------------------

    def begin(self, grip: SketchGrip):
        entity = self.cad.config.sketch[grip.index]
        before = cmod.as_shape(entity)
        self._drag = {"grip": grip, "before": before, "edit": grip_specs(before)[grip.number][2], "after": None}
        # Snap to everything but the entity being edited.
        others = [e for i, e in enumerate(self.cad.config.sketch) if i != grip.index]
        self.cad.snap.set_background_shapes(list(self.cad._background_shapes) + others)

    def drag(self, grip: SketchGrip, scene_pos):
        d = self._drag
        if d is None or d["grip"] is not grip:
            return
        hit = self.cad.snap.snap(scene_pos)
        try:
            shape = d["edit"](d["before"], (hit.x, hit.y))
        except (cmod.ModifyError, ValueError, ZeroDivisionError):
            return
        d["after"] = shape
        self.cad.preview_sketch(grip.index, shape)
        specs = grip_specs(shape)
        mine = [g for g in self.grips if g.index == grip.index]
        if len(specs) == len(mine):
            for g in mine:
                g.setPos(self.cad.to_scene_pos(*specs[g.number][0]))
        if hit.kind:
            self.window.statusBar().showMessage(f"Snap: {hit.kind}  ({hit.x:.3f}, {hit.y:.3f})", 2000)

    def finish(self, grip: SketchGrip):
        d, self._drag = self._drag, None
        if d is None or d["grip"] is not grip:
            return
        if d["after"] is None:
            self.cad._refresh_snap_shapes()
            return
        # After the event: the push rebuilds the sketch items (and these grips).
        QTimer.singleShot(0, lambda: self.cad.edit_sketch(grip.index, d["after"], "Edit sketch",
                                                          before_shape=d["before"]))
