"""
test_dxf_crop.py - cropping a big DXF to one window and exporting it
(dxf_io.export_cropped), from the import dialog's "Export Cropped DXF..."
button (clicked) and from File > Background (DXF) > Export Cropped DXF...,
and switching the background to the cropped file without moving it.
"""
import os
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog, QMessageBox  # noqa: E402

from background_dialog import BackgroundImportDialog  # noqa: E402
from cad import dxf_io  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow  # noqa: E402
from test_dxf_import import drawing_set  # noqa: E402

_app = QApplication.instance() or QApplication([])


class ExportCroppedTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, self.room = drawing_set(self.path)
        self.out = os.path.join(self.tmp.name, "sheet1.dxf")

    def test_window_is_written_cut_at_its_edge_with_units_layers_and_texts(self):
        result = dxf_io.export_cropped(self.path, self.out, self.sheet1, skip_types=["DIMENSION"])
        doc = ezdxf.readfile(self.out)
        self.assertEqual(doc.header["$INSUNITS"], ezdxf.units.IN)
        self.assertIn("WALL", doc.layers)
        x0, y0, x1, y1 = self.sheet1
        for e in doc.modelspace().query("LWPOLYLINE"):
            for x, y, *_ in e.get_points():
                self.assertTrue(x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6, (x, y))
        texts = sorted(e.dxf.text for e in doc.modelspace().query("TEXT"))
        self.assertEqual(texts, ["GROUND FLOOR PLAN", "notes"])     # sheet 2's texts are left out
        self.assertEqual(result.texts, 2)
        # Same geometry as the clipped background, far fewer entities than the whole set.
        clipped = dxf_io.read_background(self.path, clip=self.sheet1, skip_types=["DIMENSION"])
        again = dxf_io.read_background(self.out)
        self.assertEqual(len(again.shapes), len(clipped.shapes))
        self.assertLess(len(again.shapes), len(dxf_io.read_background(self.path).shapes) - 290)
        for a, b in zip(again.bounds, clipped.bounds):
            self.assertAlmostEqual(a, b, places=6)

    def test_hidden_layers_are_left_out_and_to_origin_moves_the_window(self):
        dxf_io.export_cropped(self.path, self.out, self.sheet1, hidden_layers=["FURNITURE"], to_origin=True)
        again = dxf_io.read_background(self.out)
        self.assertNotIn("FURNITURE", {s.layer for s in again.shapes})
        self.assertGreaterEqual(min(x for s in again.shapes for x, _ in s.points), -1e-6)
        self.assertLess(max(x for s in again.shapes for x, _ in s.points), 42.0 + 1e-6)


class CropDialogTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, self.room = drawing_set(self.path)
        self.info = dxf_io.inspect_drawing(self.path)
        self.d = BackgroundImportDialog(None, self.info, "set.dxf", None, source_path=self.path,
                                        layout_dir=self.tmp.name)
        self.addCleanup(self.d.deleteLater)
        self.d.set_unit("m")
        self.d.select_sheet("GROUND FLOOR")

    def click_crop(self, answer):
        out = os.path.join(self.tmp.name, "set_crop.dxf")
        with mock.patch("background_dialog.QFileDialog.getSaveFileName", return_value=(out, "")), \
                mock.patch("background_dialog.QMessageBox.question", return_value=answer):
            QTest.mouseClick(self.d.crop_button, Qt.LeftButton)
        return out

    def test_clicked_export_then_use_it_keeps_the_placement(self):
        before = self.d.background()
        out = self.click_crop(QMessageBox.Yes)
        self.assertTrue(os.path.exists(out))
        after = self.d.background()
        self.assertEqual(after.path, "set_crop.dxf")
        self.assertEqual(after.model_dump(exclude={"path"}), before.model_dump(exclude={"path"}))
        self.assertIn("set_crop.dxf", self.d.file_label.text())

    def test_clicked_export_without_switching(self):
        out = self.click_crop(QMessageBox.No)
        self.assertTrue(os.path.exists(out))
        self.assertEqual(self.d.background().path, "set.dxf")

    def test_moved_to_origin_still_lands_in_the_same_place(self):
        before = self.d.background()
        self.d.crop_origin_check.setChecked(True)
        out = os.path.join(self.tmp.name, "moved.dxf")
        self.d.export_cropped(out, use_as_background=True)
        after = self.d.background()
        self.assertIsNone(after.clip)

        def world_box(bg, path):
            g = dxf_io.read_background(path, clip=bg.clip, skip_types=bg.skipTypes, hidden_layers=bg.hiddenLayers)
            pts = [p for pl in dxf_io.background_in_metres(g, bg.scale, bg.offsetX, bg.offsetY) for p in pl]
            return (min(x for x, _ in pts), min(y for _, y in pts), max(x for x, _ in pts), max(y for _, y in pts))
        for a, b in zip(world_box(after, out), world_box(before, self.path)):
            self.assertAlmostEqual(a, b, places=6)


class EditorCropMenuTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dxf = os.path.join(self.tmp.name, "set.dxf")
        self.sheet1, _ = drawing_set(self.dxf)
        self.layout = os.path.join(self.tmp.name, "layout.json")
        with open(self.layout, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.layout), config_path=self.layout)
        self.addCleanup(self._close)
        cad = self.window.cad

        def exec_(dialog):
            dialog.set_unit("m")
            dialog.select_sheet("GROUND FLOOR")
            return QDialog.Accepted
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(self.dxf, "")), \
                mock.patch.object(BackgroundImportDialog, "exec", new=exec_):
            cad.import_background()
        self.cad = cad

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def test_menu_exports_and_switches_with_undo(self):
        before = self.window.config.background.model_copy()
        shapes = len(self.cad._background_shapes)
        out = os.path.join(self.tmp.name, "crop.dxf")
        action = next(a for a in self.window.findChildren(type(self.window.run_sim_action))
                      if a.text() == "Export Cropped DXF...")
        with mock.patch("layout_editor_cad.QFileDialog.getSaveFileName", return_value=(out, "")), \
                mock.patch("layout_editor_cad.QMessageBox.question", return_value=QMessageBox.Yes):
            action.trigger()
        bg = self.window.config.background
        self.assertEqual(bg.path, "crop.dxf")
        self.assertEqual(bg.clip, before.clip)
        self.assertEqual(len(self.cad._background_shapes), shapes)
        self.window.undo_stack.undo()
        self.assertEqual(self.window.config.background.path, "set.dxf")


if __name__ == "__main__":
    unittest.main()
