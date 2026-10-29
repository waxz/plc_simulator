"""
modify_tool.py - CAD modify tools for sketch geometry in the layout editor
(Tools > Modify and the Modify tool bar). The geometry is cad/modify.py;
this is the Qt interaction around it, sharing the drawing tools' object
snap, typed point input and tool bar (path_tool.py). Every change is one
undo step (SketchCommand).

  Trim (T)        click the piece to cut away - between the lines crossing
                  it (other sketch lines and the background drawing); the
                  piece under the cursor shows red first. Repeats until Esc.
  Extend (E)      click near a line's end: it grows to the next line it meets
  Break (K)       click a point on a line: split in two there
  Divide into Belts (N)   click a line or arc (sketch or background drawing):
                  it becomes Parts belts / roller conveyors of equal length,
                  joined end to end (tool bar Parts, Surface, Direction, Width)
  Offset (Shift+O) tool bar Distance; click a line, then the side
  Fillet (F)      tool bar Corner R; click two lines: joined with a tangent arc
  Move (M) / Copy (Shift+M) / Rotate (R) / Mirror (I) / Scale
                  act on the selected sketch lines: base point, then target
                  (Rotate: the angle's direction or "1<90"; Scale: a point at
                  the factor's distance, or type the factor; Mirror: two
                  points of the mirror line, tool bar Keep originals)
  Join (J) / Explode (X)   the selected sketch lines, at once
  Place Background (Shift+B)   move the imported DXF drawing: its anchor
                  point, then where it goes
"""
import math
from typing import List, Optional, Sequence, Tuple

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainterPath, QPen, QPolygonF
from PySide6.QtWidgets import QGraphicsPathItem

from cad import modify
from cad import path as cpath
from cad.belts import divide_segment, divided_belts, reversed_segment
from cad.modify import ModifyError
from config_parser import SketchEntity
from id_utils import next_indexed_id
from path_tool import _PointTool

PICK_PX = 10.0       # a click this close to a line picks it


def _entity(shape) -> SketchEntity:
    pts, bul, closed = shape
    return SketchEntity(points=[tuple(p) for p in pts], bulges=list(bul), closed=closed)


def _bounds(shape, pad: float = 0.0) -> Tuple[float, float, float, float]:
    """Bounding box of a shape, arcs included (their bulge sag)."""
    pts, bul, closed = shape
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    bulge_pad = 0.0
    for i, b in enumerate(bul[:len(pts) - (0 if closed else 1)]):
        b = abs(b)
        if b > 1e-12:
            a, c = pts[i], pts[(i + 1) % len(pts)]
            chord = math.hypot(c[0] - a[0], c[1] - a[1])
            # up to a semicircle the arc stays within the sagitta of its chord;
            # beyond, within its circle: 2 r from an end
            bulge_pad = max(bulge_pad, b * chord / 2.0 if b <= 1.0 else chord * (1 + b * b) / (2 * b))
    pad += bulge_pad
    return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad


def _boxes_touch(a, b) -> bool:
    return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]


class _ModifyTool(_PointTool):
    """A modify tool: talks to the CAD controller for the sketch and its undo."""

    title = "Modify"
    hint = ""
    cad = None                      # set by CadController

    def _configure_toolbar(self):
        self.toolbar.configure(self.title, segments=False, width=False, corner=False, points=True)

    # -- sketch access -------------------------------------------------------------

    def shapes(self) -> List[tuple]:
        return [modify.as_shape(e) for e in self.cad.config.sketch]

    def background(self) -> list:
        """Background drawing shapes (metres) with their bounding boxes, cached."""
        shapes = self.cad._background_shapes
        cache = getattr(self.cad, "_modify_bg_cache", None)
        if cache is None or cache[0] is not shapes:
            boxed = []
            for s in shapes:
                try:
                    shape = modify.as_shape(s)
                    boxed.append((_bounds(shape), shape))
                except (ValueError, IndexError):
                    pass
            cache = self.cad._modify_bg_cache = (shapes, boxed)
        return cache[1]

    def others(self, index: Optional[int], near) -> List[tuple]:
        """Every other sketch line plus the background drawing - those whose
        box reaches `near` (a box), so a big drawing stays quick."""
        out = [s for i, s in enumerate(self.shapes()) if i != index and _boxes_touch(_bounds(s), near)]
        out += [s for box, s in self.background() if _boxes_touch(box, near)]
        return out

    def pick_tolerance(self) -> float:
        return PICK_PX / max(1e-9, self.view.transform().m11() * self.cad.scale)

    def entity_at(self, p) -> Optional[int]:
        best, best_d = None, self.pick_tolerance()
        for i, shape in enumerate(self.shapes()):
            _s, d = modify.nearest_param(shape, p)
            if d <= best_d:
                best, best_d = i, d
        return best

    def replace(self, indices: Sequence[int], new_shapes, text: str):
        """Swap sketch entities `indices` for `new_shapes` - one undo step."""
        from layout_editor_cad import SketchCommand
        before = list(self.cad.config.sketch)
        drop = set(indices)
        insert_at = min(drop) if drop else len(before)
        kept = [e for i, e in enumerate(before) if i not in drop]
        at = sum(1 for i in range(insert_at) if i not in drop)
        after = kept[:at] + [_entity(s) for s in new_shapes] + kept[at:]
        self.window.undo_stack.push(SketchCommand(self.cad, before, after, text))

    # -- preview helpers -----------------------------------------------------------

    def draw_shape(self, shape, color: str, width: float = 2.0, dashed: bool = False):
        pts, bul, closed = shape
        flat, _ = cpath.flatten(cpath.segments(pts, bul, closed), tolerance=0.01)
        if closed and flat:
            flat.append(flat[0])
        path = QPainterPath()
        path.addPolygon(QPolygonF([self.to_scene(x, y) for x, y in flat]))
        pen = QPen(QColor(color), width)
        pen.setCosmetic(True)
        if dashed:
            pen.setStyle(Qt.DashLine)
        self._add_preview(QGraphicsPathItem(path)).setPen(pen)

    def close_path(self):
        pass

    def finish(self, closed: bool = False):
        self.stop()
        self._status(f"{self.title} done.", 3000)

    def raw_point(self, event):
        return self.from_scene(self.view.mapToScene(event.position().toPoint()))


class _PickTool(_ModifyTool):
    """Click a sketch line (no snap: the click picks the line and the spot
    on it); act(index, shape, point) does the work. Repeats until Esc."""

    def mouse_press(self, event) -> bool:
        if event.button() == Qt.RightButton:
            self.finish()
            return True
        if event.button() != Qt.LeftButton:
            return False
        p = self.raw_point(event)
        index = self.entity_at(p)
        if index is None:
            self._status(f"{self.title}: click on a sketch line.", 4000)
            return True
        try:
            self.act(index, self.shapes()[index], p)
        except ModifyError as exc:
            self._status(f"{self.title}: {exc}", 6000)
        self._update_preview()
        return True

    def mouse_move(self, event) -> bool:
        self.cursor = self.raw_point(event)
        self._update_preview()
        return True

    def act(self, index: int, shape, point):
        raise NotImplementedError

    def _update_preview(self):
        self._clear_preview()


class TrimTool(_PickTool):
    title = "Trim"
    hint = ("Trim: click the piece to cut away - between the lines crossing it (sketch and drawing). "
            "The piece shows red first. Esc to finish.")

    def result(self, index, shape, point):
        return modify.trim(shape, point, self.others(index, _bounds(shape)))

    def act(self, index, shape, point):
        left = self.result(index, shape, point)
        self.replace([index], left, "Trim")
        self._status(f"Trimmed - {len(left)} piece{'s' if len(left) != 1 else ''} left.", 4000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active or self.cursor is None:
            return
        index = self.entity_at(self.cursor)
        if index is None:
            return
        shape = self.shapes()[index]
        try:
            left = self.result(index, shape, self.cursor)
        except ModifyError:
            return
        self.draw_shape(shape, "#ef4444", 3.0)                 # all of it red ...
        for piece in left:
            self.draw_shape(piece, "#22d3ee", 2.0)             # ... except what stays


class ExtendTool(_PickTool):
    title = "Extend"
    hint = "Extend: click near the end of a sketch line - it grows to the next line it meets. Esc to finish."

    def act(self, index, shape, point):
        boundaries = [s for i, s in enumerate(self.shapes()) if i != index] + [s for _b, s in self.background()]
        self.replace([index], [modify.extend(shape, point, boundaries)], "Extend")
        self._status("Extended.", 3000)


class BreakTool(_ModifyTool):
    title = "Break"
    hint = "Break: click a point on a sketch line (OSNAP) - it splits in two there. Esc to finish."

    def add_anchor(self, p):
        index = self.entity_at(p)
        if index is None:
            self._status("Break: click on a sketch line.", 4000)
            return
        try:
            parts = modify.break_at(self.shapes()[index], p)
        except ModifyError as exc:
            self._status(f"Break: {exc}", 6000)
            return
        self.replace([index], parts, "Break")
        self._status(f"Split into {len(parts)} line{'s' if len(parts) != 1 else ''}.", 3000)


class DivideTool(_ModifyTool):
    """Divide into Belts: click a line or an arc - of the sketch or of the
    imported drawing - and that piece becomes tool bar Parts conveyors of
    equal length, each with its own drive, joined end to end: linear along
    a line, curved along an arc, belts or roller conveyors (Surface),
    running the way it was drawn or the other way (Direction). The piece
    under the cursor shows where it will be cut first. Repeats until Esc."""

    title = "Divide into Belts"
    hint = ("Divide into Belts: click a line or arc (sketch or drawing) - it becomes Parts conveyors of equal "
            "length, joined end to end. Tool bar: Parts, Surface, Direction, Width. Esc to finish.")
    ROLLER_SPEED = 0.3

    def _configure_toolbar(self):
        self.toolbar.configure(self.title, segments=False, width=True, corner=False, points=False, divide=True)
        self.toolbar.on_options_changed = self._update_preview

    def stop(self):
        self.toolbar.on_options_changed = None
        super().stop()

    def segment_at(self, p):
        """The line / arc segment nearest p within the pick distance - of a
        sketch line first, else of the background drawing - or None."""
        tol = self.pick_tolerance()
        near = (p[0] - tol, p[1] - tol, p[0] + tol, p[1] + tol)
        best, best_d = None, tol
        candidates = self.shapes() + [s for box, s in self.background() if _boxes_touch(box, near)]
        for shape in candidates:
            s, d = modify.nearest_param(shape, p)
            if d <= best_d:
                segs = cpath.segments(*shape)
                if segs:
                    best, best_d = segs[min(int(s), len(segs) - 1)], d
        return best

    def build(self, seg):
        bar = self.toolbar
        roller = bar.divide_surface_combo.currentData() == "roller"
        taken = [i.model.id for i in self.window.items_list]
        prefix = "roller" if roller else "belt"

        def new_id():
            taken.append(next_indexed_id(taken, prefix))
            return taken[-1]

        return divided_belts(new_id, seg, bar.parts_spin.value(), bar.width_spin.value(),
                             reverse=bar.divide_direction_combo.currentData() == "reverse",
                             speed=self.ROLLER_SPEED if roller else 1.0,
                             color="#4b5563" if roller else "#3b82f6", surface="roller" if roller else "belt")

    def mouse_press(self, event) -> bool:
        if event.button() == Qt.RightButton:
            self.finish()
            return True
        if event.button() != Qt.LeftButton:
            return False
        seg = self.segment_at(self.raw_point(event))
        if seg is None:
            self._status("Divide into Belts: click on a line or an arc.", 4000)
            return True
        try:
            belts = self.build(seg)
        except ValueError as exc:
            self._status(f"Divide into Belts: {exc}", 6000)
            return True
        self.cad.add_belt_chain(belts, False, "Divide into Belts")
        what = "roller conveyors" if belts[0].surface == "roller" else "belts"
        self._status(f"Divided into {len(belts)} {what} ({belts[0].id} .. {belts[-1].id}), joined end to end. "
                     f"Click another, or Esc to finish.", 8000)
        self._update_preview()
        return True

    def mouse_move(self, event) -> bool:
        self.cursor = self.raw_point(event)
        self._update_preview()
        return True

    def _update_preview(self):
        self._clear_preview()
        if not self.active or self.cursor is None:
            return
        seg = self.segment_at(self.cursor)
        if seg is None:
            return
        if self.toolbar.divide_direction_combo.currentData() == "reverse":
            seg = reversed_segment(seg)
        pieces = divide_segment(seg, self.toolbar.parts_spin.value())
        for k, piece in enumerate(pieces):
            shape = ([piece.p0, piece.p1], [0.0 if piece.kind == "line" else
                                           math.tan(piece.sweep / 4.0), 0.0], False)
            self.draw_shape(shape, "#22d3ee" if k % 2 == 0 else "#f59e0b", 4.0)
        # Where it is cut, and the flow at the end.
        for piece in pieces[1:]:
            c = self.to_scene(*piece.p0)
            mark = QPainterPath()
            mark.addEllipse(c, 4.0, 4.0)
            pen = QPen(QColor("#ffffff"), 1.5)
            pen.setCosmetic(True)
            self._add_preview(QGraphicsPathItem(mark)).setPen(pen)
        end, heading = pieces[-1].p1, pieces[-1].heading_at(1.0)
        tip = self.to_scene(*end)
        arrow = QPainterPath()
        size = 10.0
        for side in (-0.5, 0.5):
            a = heading + math.pi + side
            arrow.moveTo(tip)
            arrow.lineTo(tip.x() + size * math.cos(a), tip.y() - size * math.sin(a))
        pen = QPen(QColor("#f59e0b"), 2.0)
        pen.setCosmetic(True)
        self._add_preview(QGraphicsPathItem(arrow)).setPen(pen)
        self._draw_readout(f"{len(pieces)} x {pieces[0].length:.3f} m")


class OffsetTool(_ModifyTool):
    title = "Offset"
    hint = "Offset: set the Distance, click a sketch line, then click the side for the copy. Esc to finish."

    def _configure_toolbar(self):
        self.toolbar.configure(self.title, segments=False, width=False, corner=False, points=True, distance=True)

    def start(self):
        self.picked = None
        super().start()

    def mouse_press(self, event) -> bool:
        if event.button() == Qt.LeftButton and self.picked is None:
            index = self.entity_at(self.raw_point(event))
            if index is None:
                self._status("Offset: click a sketch line first.", 4000)
            else:
                self.picked = index
                self._status("Offset: now click the side for the copy.")
            return True
        return super().mouse_press(event)

    def add_anchor(self, side):
        if self.picked is None:
            return
        try:
            copy = modify.offset(self.shapes()[self.picked], self.toolbar.distance_spin.value(), side)
        except ModifyError as exc:
            self._status(f"Offset: {exc}", 6000)
            return
        self.replace([], [copy], "Offset")
        self.picked = None
        self._status(f"Offset {self.toolbar.distance_spin.value():g} m. Click the next line, or Esc.", 5000)
        self._update_preview()

    def _update_preview(self):
        self._clear_preview()
        if not self.active or self.picked is None or self.cursor is None:
            return
        shapes = self.shapes()
        if self.picked >= len(shapes):
            self.picked = None
            return
        self.draw_shape(shapes[self.picked], "#facc15", 2.0)
        try:
            self.draw_shape(modify.offset(shapes[self.picked], self.toolbar.distance_spin.value(), self.cursor),
                            "#22d3ee", 1.5, dashed=True)
        except ModifyError:
            pass


class FilletTool(_ModifyTool):
    title = "Fillet"
    hint = "Fillet: set Corner R, click a line, then the line to join it to (on the sides to keep). Esc to finish."

    def _configure_toolbar(self):
        self.toolbar.configure(self.title, segments=False, width=False, corner=True, points=False)

    def start(self):
        self.first = None
        super().start()

    def mouse_press(self, event) -> bool:
        if event.button() != Qt.LeftButton:
            return super().mouse_press(event)
        p = self.raw_point(event)
        index = self.entity_at(p)
        if index is None:
            self._status("Fillet: click on a sketch line.", 4000)
            return True
        if self.first is None:
            self.first = (index, p)
            self._status("Fillet: now click the second line.")
            return True
        (i, pi), self.first = self.first, None
        if index == i:
            self._status("Fillet: pick a different line for the second one.", 5000)
            return True
        shapes = self.shapes()
        try:
            joined = modify.fillet(shapes[i], pi, shapes[index], p, self.toolbar.corner_spin.value())
        except ModifyError as exc:
            self._status(f"Fillet: {exc}", 6000)
            return True
        self.replace([i, index], [joined], "Fillet")
        self._status(f"Filleted (R {self.toolbar.corner_spin.value():g} m).", 4000)
        return True

    def _update_preview(self):
        self._clear_preview()
        if self.active and self.first is not None and self.first[0] < len(self.shapes()):
            self.draw_shape(self.shapes()[self.first[0]], "#facc15", 2.0)


class _TransformTool(_ModifyTool):
    """Acts on the sketch lines selected when it starts: clicks give the
    points, transform(shapes, points) the result; preview follows the cursor."""

    points_needed = 2
    keeps_originals = False

    def start(self):
        from layout_editor_cad import SketchItem
        try:
            items = [i for i in self.window.scene.selectedItems() if isinstance(i, SketchItem)]
        except RuntimeError:
            items = []
        self.targets = sorted(i.index for i in items)
        if not self.targets:
            self._status(f"{self.title}: select sketch lines first (click them, or drag a box).", 6000)
            return
        super().start()

    def transform(self, shape, points):
        raise NotImplementedError

    def keep_originals(self) -> bool:
        return self.keeps_originals

    def add_anchor(self, p):
        super().add_anchor(p)
        if len(self.anchors) < self.points_needed:
            return
        shapes = self.shapes()
        try:
            moved = [self.transform(shapes[i], self.anchors) for i in self.targets if i < len(shapes)]
        except ModifyError as exc:
            self._status(f"{self.title}: {exc}", 6000)
            del self.anchors[1:], self.bulges[1:]
            return
        if self.keep_originals():
            self.replace([], moved, self.title)
            done = f"{self.title}: {len(moved)} copied."
        else:
            self.replace(self.targets, moved, self.title)
            done = f"{self.title}: {len(moved)} done."
        self.after_apply(done)

    def after_apply(self, message: str):
        self.stop()
        self._status(message, 4000)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        shapes = self.shapes()
        for i in self.targets:
            if i < len(shapes):
                self.draw_shape(shapes[i], "#facc15", 2.0)
        pts = list(self.anchors) + ([self.cursor] if self.cursor is not None else [])
        if len(pts) >= self.points_needed:
            for i in self.targets:
                if i < len(shapes):
                    try:
                        self.draw_shape(self.transform(shapes[i], pts[:self.points_needed]), "#22d3ee", 1.5, True)
                    except ModifyError:
                        pass
        if len(pts) >= 2:
            line = QPainterPath()
            line.moveTo(self.to_scene(*pts[0]))
            line.lineTo(self.to_scene(*pts[1]))
            self._add_preview(QGraphicsPathItem(line)).setPen(QPen(QColor(148, 163, 184), 0, Qt.DotLine))
        self._draw_anchor_markers()
        self._draw_snap_marker()


class _NumberAfterBaseTool(_TransformTool):
    """After the base point, a typed plain number is the value itself (the
    angle, the factor) - not a distance along the cursor."""

    def number_point(self, base, value: float):
        raise NotImplementedError

    def typed_input(self, text: str):
        if self.active and len(self.anchors) == 1:
            try:
                value = float(text.strip())
            except ValueError:
                value = None
            if value is not None:
                self.toolbar.input.clear()
                self.add_anchor(self.number_point(self.anchors[0], value))
                self.view.setFocus()
                return
        super().typed_input(text)


class MoveTool(_TransformTool):
    title = "Move"
    hint = "Move: click the base point, then where it goes (OSNAP, x,y or @dx,dy). Esc to cancel."

    def transform(self, shape, points):
        (x0, y0), (x1, y1) = points[:2]
        return modify.move(shape, x1 - x0, y1 - y0)


class CopyTool(MoveTool):
    title = "Copy"
    hint = "Copy: click the base point, then each place for a copy (OSNAP, x,y or @dx,dy). Esc to finish."
    keeps_originals = True

    def after_apply(self, message: str):
        del self.anchors[1:], self.bulges[1:]           # same base point: place another copy
        self._status(message + " Click another place, or Esc.", 4000)
        self._update_preview()


class RotateTool(_NumberAfterBaseTool):
    title = "Rotate"
    hint = "Rotate: click the centre, then the direction of the new angle - or type the angle (90). Esc to cancel."

    def number_point(self, base, value):
        a = math.radians(value)
        return base[0] + math.cos(a), base[1] + math.sin(a)

    def transform(self, shape, points):
        (cx, cy), (x, y) = points[:2]
        if math.hypot(x - cx, y - cy) < 1e-9:
            raise ModifyError("Pick a point away from the centre.")
        return modify.rotate(shape, (cx, cy), math.atan2(y - cy, x - cx))


class MirrorTool(_TransformTool):
    title = "Mirror"
    hint = "Mirror: click two points of the mirror line (tool bar: Keep originals). Esc to cancel."

    def _configure_toolbar(self):
        self.toolbar.configure(self.title, segments=False, width=False, corner=False, points=True, keep=True)

    def keep_originals(self) -> bool:
        return self.toolbar.keep_check.isChecked()

    def transform(self, shape, points):
        return modify.mirror(shape, points[0], points[1])


class ScaleTool(_NumberAfterBaseTool):
    title = "Scale"
    hint = ("Scale: click the base point, then a point at the factor's distance from it - or type the factor "
            "(e.g. 2). Esc to cancel.")

    def number_point(self, base, value):
        return base[0] + value, base[1]

    def transform(self, shape, points):
        (cx, cy), (x, y) = points[:2]
        return modify.scale(shape, (cx, cy), math.hypot(x - cx, y - cy))


class PlaceBackgroundTool(_ModifyTool):
    """Place Background (Shift+B): click the anchor point on the imported
    drawing (snaps to its corners, ends, centres), then where that point
    goes (snaps to the layout and sketch, or type x,y / @dx,dy). The drawing
    follows the cursor; the drop moves BackgroundDrawing.offsetX/Y - one
    undo step."""

    title = "Place Background"
    hint = ("Place Background: click the anchor point on the drawing (OSNAP), then where it goes "
            "(x,y or @dx,dy). Esc to cancel.")

    def start(self):
        if self.cad.background_item is None:
            self._status("Place Background: import a DXF background first (File > Background).", 6000)
            return
        super().start()

    def stop(self):
        super().stop()
        if self.cad.background_item is not None:
            self.cad.background_item.setPos(0, 0)
        self.cad._refresh_snap_shapes()

    def add_anchor(self, p):
        super().add_anchor(p)
        if len(self.anchors) == 1:
            # Where it goes: the layout and sketch, not the drawing being moved.
            self.snapper.set_background_shapes(list(self.cad.config.sketch))
            self._status("Place Background: click (or type) where the anchor point goes. Esc to cancel.")
            return
        (x0, y0), (x1, y1) = self.anchors[:2]
        self.stop()
        self.cad.move_background(x1 - x0, y1 - y0)

    def _update_preview(self):
        self._clear_preview()
        if not self.active:
            return
        if self.anchors and self.cursor is not None:
            (x0, y0), (x1, y1) = self.anchors[0], self.cursor
            self.cad.background_item.setPos(self.to_scene(x1 - x0, y1 - y0) - self.to_scene(0, 0))
            line = QPainterPath()
            line.moveTo(self.to_scene(x0, y0))
            line.lineTo(self.to_scene(x1, y1))
            self._add_preview(QGraphicsPathItem(line)).setPen(QPen(QColor(148, 163, 184), 0, Qt.DotLine))
        self._draw_anchor_markers()
        self._draw_snap_marker()
