"""
background_dialog.py - File > Import DXF Background / Background Settings.

Real drawings rarely import as-is: the $INSUNITS header can be wrong, the
geometry can sit kilometres from the origin, and one DXF often holds a
whole drawing set. The dialog shows a quick preview of the file and lets
the user fix all of that before anything is loaded into the editor:

  Units       m / cm / mm / inch / ft / unitless (typed scale), prefilled
              from $INSUNITS, with a cross-check against the DIMENSION
              entities (warns when the header looks wrong).
  Window      drag a rectangle on the preview, or pick a sheet (paper
              frames found in modelspace, or groups of geometry far apart);
              only geometry inside is loaded (BackgroundDrawing.clip).
  Placement   put the window's lower-left at (0, 0), or a picked point, or
              type the offsets (BackgroundDrawing.offsetX/offsetY, metres).
  Skip        dimensions, hatches, leaders; hide layers. What is skipped or
              hidden is not drawn, snapped to or picked.
  Crop        "Export Cropped DXF..." writes the window (with the filters)
              as a small DXF (dxf_io.export_cropped) and can switch the
              background to it - a big drawing set then loads in no time.

The result is a BackgroundDrawing; the dialog never touches the editor.
"""
import math
import os
from typing import List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QGraphicsPathItem,
    QGraphicsRectItem, QGraphicsScene, QGraphicsView, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget,
    QFileDialog, QListWidgetItem, QMessageBox, QPushButton, QSplitter, QVBoxLayout, QWidget,
)

import hmi_style as hmi
from cad import dxf_io
from config_parser import BackgroundDrawing

Box = Tuple[float, float, float, float]


def _to_scene(x: float, y: float) -> QPointF:
    return QPointF(x, -y)  # drawing y points up, scene y down


class PreviewView(QGraphicsView):
    """Coarse preview of the whole drawing. Left-drag: window. Wheel: zoom.
    Middle/right-drag: pan. In pick mode a click picks a point (snapped to
    the nearest preview vertex within a few pixels)."""

    window_dragged = Signal(tuple)   # Box, drawing units
    point_picked = Signal(tuple)     # (x, y), drawing units

    def __init__(self, info: dxf_io.DrawingInfo, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(self))
        self.setRenderHint(QPainter.Antialiasing, False)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(hmi.qcolor(hmi.BG)))
        self.setMinimumSize(420, 320)
        path = QPainterPath()
        vertices = []
        for pl in info.preview:
            path.moveTo(_to_scene(*pl[0]))
            for p in pl[1:]:
                path.lineTo(_to_scene(*p))
            vertices.extend(pl)
        self._vertices = vertices
        item = QGraphicsPathItem(path)
        item.setPen(QPen(hmi.qcolor(hmi.TEXT_DIM), 0))
        self.scene().addItem(item)
        self.sheet_items: List[QGraphicsRectItem] = []
        for sheet in info.sheets:
            r = self.scene().addRect(self._rect(sheet.box), QPen(QColor(59, 130, 246, 160), 0, Qt.DashLine))
            self.sheet_items.append(r)
        self.window_item = self.scene().addRect(QRectF(), QPen(QColor("#f97316"), 0),
                                                QBrush(QColor(249, 115, 22, 40)))
        self.window_item.setVisible(False)
        self.origin_item = self.scene().addEllipse(QRectF(-1, -1, 2, 2), QPen(QColor("#dc2626"), 0),
                                                   QBrush(QColor("#dc2626")))
        self.origin_item.setVisible(False)
        self.origin_item.setFlag(self.origin_item.GraphicsItemFlag.ItemIgnoresTransformations)
        self.pick_mode = False
        self._drag_start: Optional[QPointF] = None
        self._pan_start = None
        self.extents = info.extents
        if info.extents is not None:
            self.scene().setSceneRect(self._rect(info.extents).adjusted(-1, -1, 1, 1))

    @staticmethod
    def _rect(box: Box) -> QRectF:
        return QRectF(QPointF(box[0], -box[3]), QPointF(box[2], -box[1]))

    def fit(self):
        if self.extents is not None:
            self.fitInView(self._rect(self.extents), Qt.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        self.fit()

    def show_window(self, box: Optional[Box]):
        self.window_item.setVisible(box is not None)
        if box is not None:
            self.window_item.setRect(self._rect(box))

    def show_origin(self, point: Optional[Tuple[float, float]]):
        self.origin_item.setVisible(point is not None)
        if point is not None:
            self.origin_item.setRect(QRectF(-4, -4, 8, 8))
            self.origin_item.setPos(_to_scene(*point))

    def _drawing_pos(self, event) -> Tuple[float, float]:
        p = self.mapToScene(event.position().toPoint())
        return p.x(), -p.y()

    def snap_point(self, x: float, y: float, pixels: float = 8.0) -> Tuple[float, float]:
        tol = pixels / max(self.transform().m11(), 1e-12)
        best, best_d = (x, y), tol
        for vx, vy in self._vertices:
            d = math.hypot(vx - x, vy - y)
            if d <= best_d:
                best, best_d = (vx, vy), d
        return best

    def wheelEvent(self, event):
        factor = 1.25 if event.angleDelta().y() > 0 else 0.8
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._pan_start = event.position()
            return
        if event.button() == Qt.LeftButton:
            if self.pick_mode:
                self.point_picked.emit(self.snap_point(*self._drawing_pos(event)))
                return
            self._drag_start = self.mapToScene(event.position().toPoint())
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_start is not None:
            delta = event.position() - self._pan_start
            self._pan_start = event.position()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - int(delta.x()))
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - int(delta.y()))
            return
        if self._drag_start is not None:
            end = self.mapToScene(event.position().toPoint())
            self.window_item.setVisible(True)
            self.window_item.setRect(QRectF(self._drag_start, end).normalized())
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._pan_start is not None and event.button() in (Qt.MiddleButton, Qt.RightButton):
            self._pan_start = None
            return
        if self._drag_start is not None and event.button() == Qt.LeftButton:
            end = self.mapToScene(event.position().toPoint())
            start, self._drag_start = self._drag_start, None
            r = QRectF(start, end).normalized()
            if r.width() * self.transform().m11() > 4 and r.height() * self.transform().m22() > 4:
                self.window_dragged.emit((r.left(), -r.bottom(), r.right(), -r.top()))
            return
        super().mouseReleaseEvent(event)


class BackgroundImportDialog(QDialog):
    """Choose units, window, placement and filters for a DXF background.
    `current` prefills it (Background Settings); otherwise defaults suit a
    fresh import."""

    WHOLE = "Whole drawing"
    CUSTOM = "Custom window (dragged on the preview)"

    def __init__(self, parent, info: dxf_io.DrawingInfo, stored_path: str,
                 current: Optional[BackgroundDrawing] = None, source_path: Optional[str] = None,
                 layout_dir: Optional[str] = None):
        super().__init__(parent)
        self.info = info
        self.stored_path = stored_path
        self.source_path = source_path or stored_path   # the file the preview shows
        self.layout_dir = layout_dir or os.getcwd()      # stored paths are relative to it
        self.current = current
        self.setWindowTitle("Background Settings" if current is not None else "Import DXF Background")
        self.window_box: Optional[Box] = None
        self._scale = info.header_scale
        self._updating = False

        self.preview = PreviewView(info, self)
        self.preview.window_dragged.connect(self._on_window_dragged)
        self.preview.point_picked.connect(self._on_point_picked)

        side = QWidget()
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(0, 0, 0, 0)
        self.file_label = QLabel(
            f"<b>{stored_path}</b><br>{info.shape_estimate:,} entities (blocks expanded), "
            f"{len(info.layers)} layers. Header units: {info.header_units_name}.")
        self.file_label.setWordWrap(True)
        side_layout.addWidget(self.file_label)

        # -- units -------------------------------------------------------------
        units_box = QGroupBox("Units")
        form = QFormLayout(units_box)
        self.unit_combo = QComboBox()
        for label, _ in dxf_io.UNIT_CHOICES:
            self.unit_combo.addItem(label)
        self.scale_spin = QDoubleSpinBox()
        self.scale_spin.setDecimals(8)
        self.scale_spin.setRange(1e-8, 1e6)
        self.scale_spin.setSuffix(" m / unit")
        form.addRow("Drawing unit", self.unit_combo)
        form.addRow("Scale", self.scale_spin)
        self.hint_label = QLabel()
        self.hint_label.setWordWrap(True)
        self.use_hint_button = QPushButton()
        self.use_hint_button.clicked.connect(self._apply_hint)
        form.addRow(self.hint_label)
        form.addRow(self.use_hint_button)
        side_layout.addWidget(units_box)

        # -- window ------------------------------------------------------------
        window_box = QGroupBox("Import only a window")
        wl = QVBoxLayout(window_box)
        self.sheet_combo = QComboBox()
        self.sheet_combo.addItem(self.WHOLE)
        for sheet in info.sheets:
            self.sheet_combo.addItem(sheet.name)
        self.sheet_combo.addItem(self.CUSTOM)
        wl.addWidget(self.sheet_combo)
        self.window_label = QLabel()
        self.window_label.setWordWrap(True)
        wl.addWidget(self.window_label)
        wl.addWidget(QLabel("<i>Drag a rectangle on the preview to choose any window.</i>"))
        crop_row = QHBoxLayout()
        self.crop_button = QPushButton("Export Cropped DXF...")
        self.crop_button.setToolTip("Write this window, without what is left out below, as a new DXF "
                                    "(same units and coordinates) - and optionally use it as the background")
        self.crop_button.clicked.connect(self._export_cropped_clicked)
        self.crop_origin_check = QCheckBox("Move to (0, 0)")
        self.crop_origin_check.setToolTip("Put the window's lower-left corner at the drawing's origin")
        crop_row.addWidget(self.crop_button)
        crop_row.addWidget(self.crop_origin_check)
        crop_row.addStretch(1)
        wl.addLayout(crop_row)
        side_layout.addWidget(window_box)

        # -- placement ---------------------------------------------------------
        place_box = QGroupBox("Placement")
        pf = QFormLayout(place_box)
        self.origin_check = QCheckBox("Window's lower-left at (0, 0)")
        self.pick_button = QPushButton("Pick origin point on preview")
        self.pick_button.setCheckable(True)
        self.pick_button.toggled.connect(self._set_pick_mode)
        self.offset_x = QDoubleSpinBox()
        self.offset_y = QDoubleSpinBox()
        for spin in (self.offset_x, self.offset_y):
            spin.setDecimals(6)
            spin.setRange(-1e9, 1e9)
            spin.setSuffix(" m")
            spin.valueChanged.connect(self._on_offset_edited)
        pf.addRow(self.origin_check)
        pf.addRow(self.pick_button)
        pf.addRow("Offset X", self.offset_x)
        pf.addRow("Offset Y", self.offset_y)
        self.size_label = QLabel()
        self.size_label.setWordWrap(True)
        pf.addRow(self.size_label)
        side_layout.addWidget(place_box)

        # -- filters -----------------------------------------------------------
        filter_box = QGroupBox("Leave out")
        fl = QVBoxLayout(filter_box)
        skip_row = QHBoxLayout()
        self.skip_checks = {}
        for key, text in (("DIMENSION", "Dimensions"), ("HATCH", "Hatches / fills"), ("LEADER", "Leaders")):
            check = QCheckBox(text)
            self.skip_checks[key] = check
            skip_row.addWidget(check)
        fl.addLayout(skip_row)
        self.layer_filter = QLineEdit()
        self.layer_filter.setPlaceholderText(f"Filter {len(info.layers)} layers...")
        self.layer_filter.textChanged.connect(self._filter_layers)
        fl.addWidget(self.layer_filter)
        self.layer_list = QListWidget()
        for name, count in info.layers.items():
            item = QListWidgetItem(f"{name}  ({count:,})")
            item.setData(Qt.UserRole, name)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            self.layer_list.addItem(item)
        fl.addWidget(self.layer_list, 1)
        buttons_row = QHBoxLayout()
        for text, state in (("Show all", Qt.Checked), ("Hide all", Qt.Unchecked)):
            b = QPushButton(text)
            b.clicked.connect(lambda _=False, s=state: self._check_visible_layers(s))
            buttons_row.addWidget(b)
        fl.addLayout(buttons_row)
        side_layout.addWidget(filter_box, 1)

        splitter = QSplitter()
        splitter.addWidget(self.preview)
        splitter.addWidget(side)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 0)
        side.setMaximumWidth(460)
        splitter.setSizes([740, 440])
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout = QVBoxLayout(self)
        layout.addWidget(splitter, 1)
        layout.addWidget(buttons)
        self.resize(1200, 760)

        self._prefill()
        self.unit_combo.currentTextChanged.connect(self._on_unit_changed)
        self.scale_spin.valueChanged.connect(self._on_scale_changed)
        self.sheet_combo.currentIndexChanged.connect(self._on_sheet_changed)
        self.origin_check.toggled.connect(self._on_origin_toggled)
        self._refresh()

    # -- prefill -----------------------------------------------------------------

    def _prefill(self):
        info, cur = self.info, self.current
        self._updating = True
        if cur is not None:
            scale = cur.scale
            self.window_box = dxf_io._norm_box(cur.clip) if cur.clip else None
            skip = {t.upper() for t in cur.skipTypes}
            hidden = set(cur.hiddenLayers)
        else:
            scale = info.header_scale
            skip = set(dxf_io.SKIP_OPTIONS)
            hidden = set(info.off_layers)
        self._scale = scale
        self.scale_spin.setValue(scale)
        label = dxf_io.unit_label_for_scale(scale) if cur is not None or info.header_unit == "unitless" \
            else info.header_unit
        self.unit_combo.setCurrentText(label)
        self.scale_spin.setEnabled(label == "unitless")
        for key, check in self.skip_checks.items():
            check.setChecked(key in skip)
        for i in range(self.layer_list.count()):
            item = self.layer_list.item(i)
            item.setCheckState(Qt.Unchecked if item.data(Qt.UserRole) in hidden else Qt.Checked)
        self._select_sheet_for_window()
        if cur is not None:
            self.offset_x.setValue(cur.offsetX)
            self.offset_y.setValue(cur.offsetY)
            ox, oy = dxf_io.default_origin(self._placed_box(), scale)
            self.origin_check.setChecked(abs(ox - cur.offsetX) < 1e-6 and abs(oy - cur.offsetY) < 1e-6)
        else:
            self.origin_check.setChecked(True)
        self._updating = False
        self._show_hint()

    def _select_sheet_for_window(self):
        if self.window_box is None:
            self.sheet_combo.setCurrentIndex(0)
            return
        for i, sheet in enumerate(self.info.sheets, 1):
            if all(abs(a - b) < 1e-9 * max(1.0, abs(a)) for a, b in zip(sheet.box, self.window_box)):
                self.sheet_combo.setCurrentIndex(i)
                return
        self.sheet_combo.setCurrentIndex(self.sheet_combo.count() - 1)

    def _show_hint(self):
        hint = self.info.hint
        if hint is None:
            self.hint_label.setText("<i>No dimensions to cross-check the units with.</i>")
            self.use_hint_button.setVisible(False)
            return
        color = "#b45309" if hint.warn and hint.unit != self.unit_combo.currentText() else "#4b5563"
        prefix = "⚠ " if hint.warn else ""
        self.hint_label.setText(f"<span style='color:{color}'>{prefix}{hint.message}</span>")
        self.use_hint_button.setText(f"Use {hint.unit}")
        self.use_hint_button.setVisible(hint.warn and hint.unit != self.unit_combo.currentText())

    # -- state -------------------------------------------------------------------

    def scale(self) -> float:
        return self.scale_spin.value()

    def _placed_box(self) -> Optional[Box]:
        """The window, or the whole drawing when there is none (drawing units)."""
        return self.window_box if self.window_box is not None else self.info.extents

    def _refresh(self):
        box = self._placed_box()
        scale = self.scale()
        if self.origin_check.isChecked():
            ox, oy = dxf_io.default_origin(box, scale)
            self._updating = True
            self.offset_x.setValue(ox)
            self.offset_y.setValue(oy)
            self._updating = False
        if self.window_box is not None:
            b = self.window_box
            self.window_label.setText(f"Window: ({b[0]:.6g}, {b[1]:.6g}) - ({b[2]:.6g}, {b[3]:.6g}) units")
        else:
            self.window_label.setText("Whole drawing (no window).")
        if box is not None:
            ox, oy = self.offset_x.value(), self.offset_y.value()
            w, h = (box[2] - box[0]) * scale, (box[3] - box[1]) * scale
            x0, y0 = box[0] * scale + ox, box[1] * scale + oy
            self.size_label.setText(f"<b>Result: {w:.4g} x {h:.4g} m</b>, from ({x0:.4g}, {y0:.4g}) "
                                    f"to ({x0 + w:.4g}, {y0 + h:.4g}) m in the layout.")
        else:
            self.size_label.setText("The drawing has no geometry.")
        self.preview.show_window(self.window_box)
        ox, oy = self.offset_x.value(), self.offset_y.value()
        self.preview.show_origin((-ox / scale, -oy / scale) if scale else None)

    # -- slots -------------------------------------------------------------------

    def _on_unit_changed(self, label: str):
        value = dict(dxf_io.UNIT_CHOICES).get(label)
        self.scale_spin.setEnabled(value is None)
        if value is not None:
            self.scale_spin.setValue(value)
        self._show_hint()

    def _on_scale_changed(self, value: float):
        old, self._scale = self._scale, value
        if not self.origin_check.isChecked() and old:
            # Keep the same drawing point at the layout origin.
            self._updating = True
            self.offset_x.setValue(self.offset_x.value() / old * value)
            self.offset_y.setValue(self.offset_y.value() / old * value)
            self._updating = False
        if not self._updating and self.unit_combo.currentText() != "unitless":
            label = dxf_io.unit_label_for_scale(value)
            if label != self.unit_combo.currentText():
                self.unit_combo.blockSignals(True)
                self.unit_combo.setCurrentText(label)
                self.unit_combo.blockSignals(False)
                self.scale_spin.setEnabled(label == "unitless")
        self._refresh()

    def _apply_hint(self):
        if self.info.hint is not None:
            self.set_unit(self.info.hint.unit)

    def _on_sheet_changed(self, index: int):
        if self._updating:
            return
        if index == 0:
            self.window_box = None
        elif 1 <= index <= len(self.info.sheets):
            self.window_box = self.info.sheets[index - 1].box
        self._refresh()

    def _on_window_dragged(self, box: Box):
        self.window_box = dxf_io._norm_box(box)
        self._updating = True
        self._select_sheet_for_window()
        self._updating = False
        self._refresh()

    def _set_pick_mode(self, on: bool):
        self.preview.pick_mode = on
        self.preview.viewport().setCursor(Qt.CrossCursor if on else Qt.ArrowCursor)

    def _on_point_picked(self, point):
        self.pick_button.setChecked(False)
        self.pick_origin(*point)

    def _on_origin_toggled(self, _on: bool):
        self._refresh()

    def _on_offset_edited(self, _value):
        if self._updating:
            return
        self.origin_check.blockSignals(True)
        self.origin_check.setChecked(False)
        self.origin_check.blockSignals(False)
        self._refresh()

    def _filter_layers(self, text: str):
        text = text.lower()
        for i in range(self.layer_list.count()):
            item = self.layer_list.item(i)
            item.setHidden(bool(text) and text not in item.data(Qt.UserRole).lower())

    def _check_visible_layers(self, state):
        for i in range(self.layer_list.count()):
            item = self.layer_list.item(i)
            if not item.isHidden():
                item.setCheckState(state)

    def _export_cropped_clicked(self):
        base = os.path.splitext(os.path.basename(self.source_path))[0]
        start = os.path.join(self.layout_dir, f"{base}_crop.dxf")
        dst, _ = QFileDialog.getSaveFileName(self, "Export Cropped DXF", start, "DXF Files (*.dxf)")
        if not dst:
            return
        result = self.export_cropped(dst, use_as_background=False)
        if result is None:
            return
        answer = QMessageBox.question(
            self, "Export Cropped DXF",
            f"Wrote {result.shapes:,} shapes and {result.texts:,} texts to\n{dst}\n\n"
            f"Use the cropped file as the background? It loads much faster than the full drawing.")
        if answer == QMessageBox.Yes:
            self.use_background_file(dst, moved=self.crop_origin_check.isChecked())

    # -- API (also used by tests) -----------------------------------------------

    def export_cropped(self, dst: str, use_as_background: bool = False):
        """Write the window with the current filters to `dst`; returns the
        dxf_io.CropResult (None when it failed, after telling the user)."""
        to_origin = self.crop_origin_check.isChecked()
        try:
            # Moving to (0, 0) needs a known corner: the window, else the drawing's extents.
            clip = self.window_box or (self.info.extents if to_origin else None)
            result = dxf_io.export_cropped(
                self.source_path, dst, clip,
                skip_types=[k for k, c in self.skip_checks.items() if c.isChecked()],
                hidden_layers=self.hidden_layers(), to_origin=to_origin)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, "Export Cropped DXF", f"Could not write {dst}:\n{exc}")
            return None
        if use_as_background:
            self.use_background_file(dst, moved=to_origin)
        return result

    def use_background_file(self, path: str, moved: bool = False):
        """Make `path` (a cropped copy of the previewed drawing) the
        background file. Its coordinates are the original's unless it was
        moved to (0, 0); either way the layout placement stays the same."""
        try:
            self.stored_path = os.path.relpath(path, self.layout_dir)
        except ValueError:  # different drive on Windows
            self.stored_path = path
        anchor = self.window_box or self.info.extents
        if moved and anchor is not None:
            x0, y0 = anchor[0], anchor[1]
            scale = self.scale()
            self.origin_check.setChecked(False)
            self._updating = True
            self.offset_x.setValue(self.offset_x.value() + x0 * scale)
            self.offset_y.setValue(self.offset_y.value() + y0 * scale)
            self._updating = False
            self.window_box = None   # the cropped file is the window now
        self.file_label.setText(f"<b>{self.stored_path}</b><br>Cropped from {os.path.basename(self.source_path)}"
                                f"; the cropped file will be the background.")
        self._refresh()

    def set_unit(self, label: str):
        self.unit_combo.setCurrentText(label)

    def set_window(self, box: Optional[Box]):
        if box is None:
            self.sheet_combo.setCurrentIndex(0)
        else:
            self._on_window_dragged(box)

    def select_sheet(self, name_part: str) -> dxf_io.Sheet:
        for i, sheet in enumerate(self.info.sheets, 1):
            if name_part.lower() in sheet.name.lower():
                self.sheet_combo.setCurrentIndex(i)
                return sheet
        raise KeyError(name_part)

    def pick_origin(self, x: float, y: float):
        """Put drawing point (x, y) at the layout's (0, 0)."""
        self.origin_check.setChecked(False)
        self._updating = True
        self.offset_x.setValue(-x * self.scale())
        self.offset_y.setValue(-y * self.scale())
        self._updating = False
        self._refresh()

    def set_layer_visible(self, name: str, visible: bool):
        for i in range(self.layer_list.count()):
            item = self.layer_list.item(i)
            if item.data(Qt.UserRole) == name:
                item.setCheckState(Qt.Checked if visible else Qt.Unchecked)
                return
        raise KeyError(name)

    def hidden_layers(self) -> List[str]:
        return [self.layer_list.item(i).data(Qt.UserRole) for i in range(self.layer_list.count())
                if self.layer_list.item(i).checkState() != Qt.Checked]

    def background(self) -> BackgroundDrawing:
        visible = self.current.visible if self.current is not None else True
        return BackgroundDrawing(
            path=self.stored_path, scale=self.scale(), offsetX=self.offset_x.value(), offsetY=self.offset_y.value(),
            visible=visible, clip=self.window_box,
            skipTypes=[k for k, c in self.skip_checks.items() if c.isChecked()],
            hiddenLayers=self.hidden_layers())
