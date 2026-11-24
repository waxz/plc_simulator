"""
test_dxf_import.py - importing real-world DXF drawings as the editor's
background: wrong $INSUNITS caught by the dimensions, importing one window
(sheet) of a drawing set, placing it at the origin, leaving out dimensions /
hatches / leaders / layers (also for snapping and picking), the closed-area
index built off the UI thread, and - when the user's own drawings are
present - that a sheet of a 20 MB drawing set opens in a few seconds.
"""
import os
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog  # noqa: E402

from background_dialog import BackgroundImportDialog  # noqa: E402
from cad import dxf_io  # noqa: E402
from cad.snap import DrawingPicker  # noqa: E402
from config_parser import BackgroundDrawing, SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402

_app = QApplication.instance() or QApplication([])
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REAL_SET = os.path.join(HERE, "local", "layoyt.dxf")      # the user's drawing set (local/: not in git)
REAL_PLAN = os.path.join(HERE, "local", "layout2.dxf")     # the user's second layout (local/: not in git)

SHEET_W, SHEET_H = 42.0, 29.7  # an "A3 at 1:100" frame, drawn in metres


def drawing_set(path, insunits=ezdxf.units.IN, dimlfac=100.0):
    """Two sheets side by side far from the origin, drawn in metres but with
    a wrong $INSUNITS. Sheet 1: a 10 x 5 m room (walls on WALL), a column
    block, dimensions shown in cm, a hatch, a leader, furniture. Sheet 2:
    a big block only reachable by expanding it."""
    doc = ezdxf.new("R2010", setup=True)
    doc.header["$INSUNITS"] = insunits
    msp = doc.modelspace()
    x0, y0 = 5000.0, -9000.0
    for i, title in enumerate(("GROUND FLOOR PLAN", "SECTION AA")):
        fx = x0 + i * SHEET_W
        msp.add_lwpolyline([(fx, y0), (fx + SHEET_W, y0), (fx + SHEET_W, y0 + SHEET_H), (fx, y0 + SHEET_H)],
                           close=True)
        msp.add_lwpolyline([(fx + 1, y0 + 1), (fx + SHEET_W - 1, y0 + 1), (fx + SHEET_W - 1, y0 + SHEET_H - 1),
                            (fx + 1, y0 + SHEET_H - 1)], close=True)  # inner border: not a sheet
        msp.add_text(title, height=0.8).set_placement((fx + 30, y0 + 2))
        msp.add_text("notes", height=0.3).set_placement((fx + 30, y0 + 4))
    rx, ry = x0 + 5, y0 + 5
    doc.layers.add("WALL")
    doc.layers.add("FURNITURE")
    msp.add_lwpolyline([(rx, ry), (rx + 10, ry), (rx + 10, ry + 5), (rx, ry + 5)], close=True,
                       dxfattribs={"layer": "WALL"})
    msp.add_line((rx + 2, ry + 1), (rx + 3, ry + 1), dxfattribs={"layer": "FURNITURE"})
    column = doc.blocks.new("COLUMN")
    column.add_lwpolyline([(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)], close=True)  # layer 0
    msp.add_blockref("COLUMN", (rx + 5, ry + 2.5), dxfattribs={"layer": "WALL"})
    for a, b in (((rx, ry), (rx + 10, ry)), ((rx, ry), (rx, ry + 5)), ((rx + 2, ry), (rx + 2.85, ry))):
        msp.add_linear_dim(base=(a[0], a[1] - 1), p1=a, p2=b, angle=0 if a[1] == b[1] else 90,
                           override={"dimlfac": dimlfac}).render()
    hatch = msp.add_hatch()
    hatch.paths.add_polyline_path([(rx + 6, ry + 1), (rx + 7, ry + 1), (rx + 7, ry + 2)], is_closed=True)
    msp.add_leader([(rx + 8, ry + 4), (rx + 9, ry + 4.5)])
    big = doc.blocks.new("BIG")
    for k in range(300):
        big.add_line((0, k * 0.05), (10, k * 0.05))
    msp.add_blockref("BIG", (x0 + SHEET_W + 5, y0 + 5))
    msp.add_line((rx - 10, ry + 3), (rx + 1, ry + 3))  # crosses the left edge of sheet 1
    doc.saveas(path)
    return (x0, y0, x0 + SHEET_W, y0 + SHEET_H), (rx, ry)


class DrawingSetTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, self.room = drawing_set(self.path)

    def test_wrong_insunits_is_caught_by_the_dimensions(self):
        info = dxf_io.inspect_drawing(self.path)
        self.assertEqual(info.header_unit, "inch")
        self.assertIsNotNone(info.hint)
        self.assertTrue(info.hint.warn)
        self.assertEqual(info.hint.unit, "m")
        self.assertIn("DIMLFAC", info.hint.message)
        self.assertIn("cm", info.hint.message)

    def test_correct_header_gives_no_warning(self):
        path = os.path.join(self.tmp.name, "ok.dxf")
        drawing_set(path, insunits=ezdxf.units.M)
        hint = dxf_io.inspect_drawing(path).hint
        self.assertEqual(hint.unit, "m")
        self.assertFalse(hint.warn)

    def test_implausible_dimensions_without_dimlfac_warn_too(self):
        path = os.path.join(self.tmp.name, "nofac.dxf")
        drawing_set(path, insunits=ezdxf.units.IN, dimlfac=1.0)
        hint = dxf_io.inspect_drawing(path).hint
        self.assertTrue(hint.warn)       # median 5 m dimension as inches = 13 cm: implausible
        self.assertEqual(hint.unit, "m")

    def test_sheets_are_found_and_named(self):
        sheets = dxf_io.inspect_drawing(self.path).sheets
        self.assertEqual(len(sheets), 2)  # inner borders are not separate sheets
        self.assertIn("GROUND FLOOR PLAN", sheets[0].name)
        self.assertIn("SECTION AA", sheets[1].name)
        for got, want in zip(sheets[0].box, self.sheet1):
            self.assertAlmostEqual(got, want)

    def test_groups_without_frames(self):
        path = os.path.join(self.tmp.name, "groups.dxf")
        doc = ezdxf.new()
        msp = doc.modelspace()
        for k in range(20):
            msp.add_line((k, 0), (k, 10))           # group near 0..20
            msp.add_line((900 + k, 0), (900 + k, 3))  # group far away
        doc.saveas(path)
        sheets = dxf_io.inspect_drawing(path).sheets
        self.assertEqual(len(sheets), 2)
        self.assertEqual(sheets[0].box[0], 0)
        self.assertEqual(sheets[1].box[0], 900)

    def test_clip_keeps_one_sheet_and_cuts_crossing_lines(self):
        whole = dxf_io.read_background(self.path)
        sheet = dxf_io.read_background(self.path, clip=self.sheet1)
        self.assertGreater(len(whole.shapes), len(sheet.shapes) + 290)  # the BIG block on sheet 2
        for s in sheet.shapes:
            for x, y in s.points:
                self.assertTrue(self.sheet1[0] - 1e-6 <= x <= self.sheet1[2] + 1e-6, (x, y))
        rx, ry = self.room
        cut = [s for s in sheet.shapes if any(abs(y - (ry + 3)) < 1e-9 for _, y in s.points)
               and len(s.points) == 2 and abs(s.points[0][1] - s.points[1][1]) < 1e-9]
        self.assertEqual(len(cut), 1)
        self.assertAlmostEqual(min(x for x, _ in cut[0].points), self.sheet1[0])  # cut at the window edge

    def test_skip_types_and_hidden_layers(self):
        full = dxf_io.read_background(self.path, clip=self.sheet1)
        lean = dxf_io.read_background(self.path, clip=self.sheet1, skip_types=("DIMENSION", "HATCH", "LEADER"))
        self.assertLess(len(lean.shapes), len(full.shapes) - 5)
        layers = {s.layer for s in full.shapes}
        self.assertIn("WALL", layers)
        no_walls = dxf_io.read_background(self.path, clip=self.sheet1, hidden_layers=("WALL",))
        self.assertNotIn("WALL", {s.layer for s in no_walls.shapes})
        # The column block (its outline on layer 0) is inserted on WALL, so it goes too.
        self.assertEqual(sum(1 for s in full.shapes if s.layer == "WALL") - 2,
                         sum(1 for s in no_walls.shapes if s.layer == "WALL") + 0)

    def test_layer_counts_resolve_block_layer_zero(self):
        info = dxf_io.inspect_drawing(self.path)
        self.assertEqual(info.layers["WALL"], 2)       # room + the column block's outline
        self.assertEqual(info.layers["FURNITURE"], 1)


class ImportDialogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, self.room = drawing_set(self.path)
        self.info = dxf_io.inspect_drawing(self.path)

    def dialog(self, current=None):
        d = BackgroundImportDialog(None, self.info, "set.dxf", current)
        self.addCleanup(d.deleteLater)
        return d

    def test_header_units_are_prefilled_and_the_warning_is_shown(self):
        d = self.dialog()
        self.assertEqual(d.unit_combo.currentText(), "inch")
        self.assertAlmostEqual(d.scale(), 0.0254)
        self.assertFalse(d.use_hint_button.isHidden())
        self.assertIn("Header says inch", d.hint_label.text())
        d.use_hint_button.click()
        self.assertEqual(d.unit_combo.currentText(), "m")
        self.assertAlmostEqual(d.scale(), 1.0)
        self.assertTrue(d.use_hint_button.isHidden())

    def test_window_place_at_origin_and_filters(self):
        d = self.dialog()
        d.set_unit("m")
        d.select_sheet("GROUND FLOOR")
        bg = d.background()
        self.assertEqual(bg.clip, self.sheet1)
        self.assertAlmostEqual(bg.offsetX, -self.sheet1[0])
        self.assertAlmostEqual(bg.offsetY, -self.sheet1[1])
        self.assertIn("42 x 29.7 m", d.size_label.text())
        self.assertEqual(sorted(bg.skipTypes), ["DIMENSION", "HATCH", "LEADER"])
        d.set_layer_visible("FURNITURE", False)
        self.assertIn("FURNITURE", d.background().hiddenLayers)

    def test_dragged_window_and_picked_origin_follow_the_scale(self):
        d = self.dialog()
        rx, ry = self.room
        d.set_unit("m")
        d.set_window((rx + 11, ry + 6, rx - 1, ry - 1))  # any corner order
        self.assertEqual(d.sheet_combo.currentText(), d.CUSTOM)
        self.assertEqual(d.background().clip, (rx - 1, ry - 1, rx + 11, ry + 6))
        self.assertAlmostEqual(d.background().offsetX, -(rx - 1))
        d.pick_origin(rx, ry)                       # the room's corner at (0, 0)
        self.assertFalse(d.origin_check.isChecked())
        self.assertAlmostEqual(d.background().offsetX, -rx)
        d.set_unit("mm")                            # the same drawing point stays at the origin
        self.assertAlmostEqual(d.background().offsetX, -rx * 0.001)
        self.assertAlmostEqual(d.background().offsetY, -ry * 0.001)

    def test_settings_dialog_starts_from_the_current_background(self):
        current = BackgroundDrawing(path="set.dxf", scale=1.0, offsetX=-self.sheet1[0], offsetY=-self.sheet1[1],
                                    clip=self.sheet1, skipTypes=["HATCH"], hiddenLayers=["FURNITURE"])
        d = self.dialog(current)
        self.assertEqual(d.unit_combo.currentText(), "m")
        self.assertIn("GROUND FLOOR", d.sheet_combo.currentText())
        self.assertTrue(d.origin_check.isChecked())
        self.assertEqual(d.background().model_dump(), current.model_dump())

    def test_preview_drag_emits_a_window(self):
        d = self.dialog()
        d.show()
        _app.processEvents()
        view = d.preview
        got = []
        view.window_dragged.connect(got.append)
        vp = view.viewport()
        a = QPointF(view.mapFromScene(QPointF(self.sheet1[0] + 2, -(self.sheet1[1] + 2))))
        b = QPointF(view.mapFromScene(QPointF(self.sheet1[0] + 20, -(self.sheet1[1] + 15))))
        _app.sendEvent(vp, QMouseEvent(QEvent.MouseButtonPress, a, vp.mapToGlobal(a), Qt.LeftButton,
                                       Qt.LeftButton, Qt.NoModifier))
        _app.sendEvent(vp, QMouseEvent(QEvent.MouseMove, b, vp.mapToGlobal(b), Qt.NoButton, Qt.LeftButton,
                                       Qt.NoModifier))
        _app.sendEvent(vp, QMouseEvent(QEvent.MouseButtonRelease, b, vp.mapToGlobal(b), Qt.LeftButton,
                                       Qt.NoButton, Qt.NoModifier))
        self.assertEqual(len(got), 1)
        x0, y0, x1, y1 = got[0]
        self.assertAlmostEqual(x0, self.sheet1[0] + 2, delta=0.5)
        self.assertAlmostEqual(y1, self.sheet1[1] + 15, delta=0.5)
        d.close()


class EditorImportTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dxf = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, self.room = drawing_set(self.dxf)
        self.path = os.path.join(self.tmp.name, "layout.json")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.resize(1200, 800)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)
        self.cad = self.window.cad

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def import_with(self, configure):
        def exec_(dialog):
            configure(dialog)
            return QDialog.Accepted
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(self.dxf, "")), \
                mock.patch.object(BackgroundImportDialog, "exec", new=exec_):
            self.cad.import_background()

    def test_one_sheet_in_metres_at_the_origin_snaps_and_survives_reopen(self):
        def configure(d):
            d.set_unit("m")
            d.select_sheet("GROUND FLOOR")
            d.set_layer_visible("FURNITURE", False)
        self.import_with(configure)
        bg = self.window.config.background
        self.assertEqual(bg.clip, self.sheet1)
        rx, ry = self.room
        corner = (rx - self.sheet1[0], ry - self.sheet1[1])  # (5, 5) m from the sheet's lower-left
        snap = self.cad.snap.background.query(corner[0] + 0.03, corner[1] - 0.02, 0.1)
        self.assertEqual(snap.kind, "endpoint")
        self.assertAlmostEqual(snap.x, corner[0])
        self.assertAlmostEqual(snap.y, corner[1])
        # Hidden layer: no snap to the furniture line's end.
        fx, fy = corner[0] + 3, corner[1] + 1
        hit = self.cad.snap.background.query(fx, fy, 0.05)
        self.assertTrue(hit is None or hit.kind != "endpoint" or abs(hit.x - fx) > 1e-9)
        # Picking a closed area works on the clipped drawing.
        region = self.cad.snap.picker.pick_region(corner[0] + 1, corner[1] + 4)
        self.assertIsNotNone(region)
        self.assertAlmostEqual(region.area, 10 * 5 - 0.4 * 0.4, places=6)

        self.assertTrue(self.window._save_config())
        reopened = MainWindow(config=load_config(self.path), config_path=self.path)
        self.addCleanup(reopened.deleteLater)
        self.assertEqual(reopened.config.background.model_dump(), bg.model_dump())
        self.assertIsNotNone(reopened.cad.snap.background.query(corner[0], corner[1], 0.01))
        reopened.undo_stack.setClean()

    def test_background_settings_changes_scale_and_undo_restores(self):
        self.import_with(lambda d: (d.set_unit("m"), d.select_sheet("GROUND FLOOR")))
        before = self.window.config.background.model_copy()

        def to_mm(d):
            self.assertEqual(d.unit_combo.currentText(), "m")  # starts from the current settings
            d.set_unit("mm")
            return QDialog.Accepted
        with mock.patch.object(BackgroundImportDialog, "exec", new=to_mm):
            self.cad.background_settings()
        self.assertAlmostEqual(self.window.config.background.scale, 0.001)
        self.assertAlmostEqual(self.window.config.background.offsetX, -self.sheet1[0] * 0.001)
        self.window.undo_stack.undo()
        self.assertEqual(self.window.config.background.model_dump(), before.model_dump())

    def test_cancel_changes_nothing(self):
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(self.dxf, "")), \
                mock.patch("layout_editor_cad.BackgroundImportDialog.exec", return_value=QDialog.Rejected):
            self.cad.import_background()
        self.assertIsNone(self.window.config.background)


class RegionIndexTest(unittest.TestCase):
    def test_regions_are_found_on_a_worker_thread(self):
        from cad.dxf_io import DrawingShape
        shapes = [DrawingShape([(0, 0), (4, 0), (4, 3), (0, 3)], [0.0] * 4, True)]
        shapes += [DrawingShape([(10 + k, 0), (10 + k, 1)], [0.0, 0.0], False) for k in range(200)]
        picker = DrawingPicker(shapes)
        self.assertFalse(picker.regions_ready())
        picker.start_regions()
        region = picker.pick_region(1, 1)  # waits for the worker
        self.assertTrue(picker.regions_ready())
        self.assertAlmostEqual(region.area, 12.0)
        self.assertIsNone(picker.pick_region(50, 50, wait=False))


@unittest.skipUnless(os.path.exists(REAL_SET), "the user's drawing set layoyt.dxf is not present")
class RealDrawingSetTest(unittest.TestCase):
    """layoyt.dxf: 22.8 MB, $INSUNITS inches but drawn in metres, ~9 km from
    the origin, 14 A3 sheets, 88k entities when expanded."""

    def test_one_sheet_opens_in_a_few_seconds(self):
        dxf_io._CACHE.clear()
        start = time.perf_counter()
        info = dxf_io.inspect_drawing(REAL_SET)
        self.assertTrue(info.hint.warn)
        self.assertEqual(info.hint.unit, "m")
        sheet = next(s for s in info.sheets if "IMPLANTATION" in s.name.upper())
        offset = dxf_io.default_origin(sheet.box, 1.0)
        bg = BackgroundDrawing(path=REAL_SET, scale=1.0, offsetX=offset[0], offsetY=offset[1], clip=sheet.box,
                               skipTypes=list(dxf_io.SKIP_OPTIONS))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "layout.json")
            with open(path, "w", encoding="utf-8") as f:
                f.write(SimulatorConfig(background=bg).model_dump_json())
            window = MainWindow(config=load_config(path), config_path=path)
            elapsed = time.perf_counter() - start
            try:
                self.assertIsNotNone(window.cad.background_item)
                rect = window.cad.background_item.boundingRect()
                lo = to_scene_pos(0, 0)
                hi = to_scene_pos(sheet.box[2] - sheet.box[0], sheet.box[3] - sheet.box[1])
                self.assertGreaterEqual(rect.left(), lo.x() - 1)    # the sheet sits at the origin, in metres
                self.assertLessEqual(rect.right(), hi.x() + 1)
                self.assertGreater(len(window.cad._background_shapes), 50)
                t = time.perf_counter()
                window.cad.snap.picker.regions()
                self.assertLess(time.perf_counter() - t, 2.0)
            finally:
                window.undo_stack.setClean()
                window.close()
                window.deleteLater()
        self.assertLess(elapsed, 8.0, f"opening one sheet took {elapsed:.1f} s")


@unittest.skipUnless(os.path.exists(REAL_PLAN), "the user's layout2.dxf is not present")
class RealPlanTest(unittest.TestCase):
    """layout2.dxf: correct metres, no paper frames, a plan plus a small
    group 1.4 km away."""

    def test_units_agree_and_the_plan_group_is_offered(self):
        info = dxf_io.inspect_drawing(REAL_PLAN)
        self.assertEqual(info.header_unit, "m")
        self.assertFalse(info.hint.warn)
        self.assertGreaterEqual(len(info.sheets), 2)
        plan = info.sheets[0]
        self.assertLess(plan.box[2] - plan.box[0], 300)  # the plan, not the 2 km extents
        geom = dxf_io.read_background(REAL_PLAN, clip=plan.box, skip_types=list(dxf_io.SKIP_OPTIONS))
        self.assertGreater(len(geom.shapes), 1000)


if __name__ == "__main__":
    unittest.main()
