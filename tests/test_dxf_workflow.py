"""
test_dxf_workflow.py - the whole DXF round trip, driven through the layout
editor like a user would:

  1. File > Import DXF Background: a drawing set in the wrong units, pick
     one sheet, put it at the origin, Export Cropped DXF... and use it
  2. draw belts on it: a conveyor line (Belt Path, separate belts, rounded
     corner) and a curved belt (center, start, end)
  3. save the layout and File > Export DXF
  4. reimport the exported DXF into an empty layout and check that it
     holds the same thing: the floor plan where it was, and each belt's
     outline - Belt from Sketch/Drawing inside each outline rebuilds a belt
     of the same type, shape and position.

File dialogs are mocked; everything else goes through the editor's tools.
"""
import os
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QKeyEvent, QMouseEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog, QMessageBox  # noqa: E402

from background_dialog import BackgroundImportDialog  # noqa: E402
from cad import dxf_io  # noqa: E402
from cad.checks import belt_footprint  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from test_dxf_import import SHEET_H, SHEET_W, drawing_set  # noqa: E402

_app = QApplication.instance() or QApplication([])


class Editor:
    """One editor window on a layout file, with user-style input."""

    def __init__(self, test, layout):
        with open(layout, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.layout = layout
        self.window = MainWindow(config=load_config(layout), config_path=layout)
        self.window.resize(1400, 900)
        self.window.show()
        _app.processEvents()
        self.view = self.window.view
        self.cad = self.window.cad
        test.addCleanup(self.close)

    def close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def look_at(self, x, y):
        self.view.resetTransform()
        self.view.centerOn(to_scene_pos(x, y))

    def click(self, x, y):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(to_scene_pos(x, y)))
        for kind, button, buttons in ((QEvent.MouseMove, Qt.NoButton, Qt.NoButton),
                                      (QEvent.MouseButtonPress, Qt.LeftButton, Qt.LeftButton),
                                      (QEvent.MouseButtonRelease, Qt.LeftButton, Qt.NoButton)):
            _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos), button, buttons, Qt.NoModifier))

    def key(self, key):
        _app.sendEvent(self.view, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier, ""))

    def belts(self):
        return [i.model for i in self.window.items_list]

    def import_background(self, dxf, configure):
        def exec_(dialog):
            configure(dialog)
            return QDialog.Accepted
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(dxf, "")), \
                mock.patch.object(BackgroundImportDialog, "exec", new=exec_):
            self.cad.import_background()


def footprints_match(test, a, b, what):
    pa, pb = belt_footprint(a), belt_footprint(b)
    test.assertLess(pa.symmetric_difference(pb).area, 0.002 * pa.area, what)


class DxfRoundTripTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(__import__("shutil").rmtree, self.tmp, True)
        self.drawing = os.path.join(self.tmp, "plant_set.dxf")
        self.sheet1, self.room = drawing_set(self.drawing)   # $INSUNITS says inch, drawn in metres

    def test_import_crop_draw_export_and_reimport(self):
        # -- 1. import one sheet, crop it, use the cropped file -------------------------
        ed = Editor(self, os.path.join(self.tmp, "line.json"))
        cropped = os.path.join(self.tmp, "plant_sheet1.dxf")

        def configure(d):
            d.use_hint_button.click()                    # the dimensions say metres, not inch
            self.assertEqual(d.unit_combo.currentText(), "m")
            d.select_sheet("GROUND FLOOR")               # origin check: sheet's lower-left at (0, 0)
            with mock.patch("background_dialog.QFileDialog.getSaveFileName", return_value=(cropped, "")), \
                    mock.patch("background_dialog.QMessageBox.question", return_value=QMessageBox.Yes):
                QTest.mouseClick(d.crop_button, Qt.LeftButton)
        ed.import_background(self.drawing, configure)
        bg = ed.window.config.background
        self.assertEqual(bg.path, "plant_sheet1.dxf")                 # the cropped file is the background
        self.assertLess(os.path.getsize(cropped), os.path.getsize(self.drawing))
        x0, y0 = self.room[0] - self.sheet1[0], self.room[1] - self.sheet1[1]   # room corner: (5, 5) m
        hit = ed.cad.snap.background.query(x0 + 0.02, y0 - 0.02, 0.1)
        self.assertEqual((hit.kind, round(hit.x, 9), round(hit.y, 9)), ("endpoint", x0, y0))
        plan = list(ed.cad._background_shapes)
        self.assertGreater(len(plan), 5)

        # -- 2. draw belts on it ----------------------------------------------------------
        ed.look_at(10, 7.5)
        toolbar = ed.cad.draw_toolbar
        toolbar.width_spin.setValue(0.8)
        toolbar.corner_spin.setValue(1.0)
        ed.cad.path_tool.start()                                       # Belt Path, separate belts
        self.assertTrue(toolbar.split_check.isChecked())
        for point in ("6,6", "14,6", "14,9.5"):
            ed.cad.path_tool.typed_input(point)
        ed.key(Qt.Key_Return)
        ed.cad.curved_tool.start()                                     # Curved Belt: center, start, end
        toolbar.arc_method_combo.setCurrentIndex(toolbar.arc_method_combo.findData("center"))
        for point in ("10,7.5", "11.5,7.5", "10,12"):
            ed.cad.curved_tool.typed_input(point)
        drawn = ed.belts()
        self.assertEqual([b.type for b in drawn], ["linear", "curved", "linear", "curved"])
        self.assertEqual(len(ed.window.config.connections), 2)
        self.assertEqual(ed.cad.broken_connection_ids(), set())

        # -- 3. save, export ------------------------------------------------------------
        self.assertTrue(ed.window._save_config())
        saved = load_config(ed.layout)
        self.assertEqual(saved.background.path, "plant_sheet1.dxf")
        self.assertEqual(len(saved.belts), 4)
        exported = os.path.join(self.tmp, "line_export.dxf")
        with mock.patch("layout_editor_cad.QFileDialog.getSaveFileName", return_value=(exported, "")):
            ed.cad.export_dxf()
        doc = ezdxf.readfile(exported)
        self.assertEqual(doc.units, ezdxf.units.M)
        msp = doc.modelspace()
        self.assertEqual(len(msp.query('LWPOLYLINE[layer=="BELTS"]')), 4)
        self.assertEqual(len(msp.query('*[layer=="BACKGROUND"]')), len(plan))
        self.assertEqual(sorted(e.dxf.text for e in msp.query('TEXT[layer=="LABELS"]')),
                         sorted(b.id for b in drawn))

        # -- 4. reimport the export into an empty layout -----------------------------------
        others = [name for name in dxf_io.LAYERS if name != "BACKGROUND"]
        plan_again = dxf_io.read_background(exported, hidden_layers=others)
        self.assertEqual(len(plan_again.shapes), len(plan))           # the floor plan, where it was
        for a, b in zip(sorted(s.points[0] for s in plan_again.shapes), sorted(s.points[0] for s in plan)):
            self.assertAlmostEqual(a[0], b[0], places=6)
            self.assertAlmostEqual(a[1], b[1], places=6)

        again = Editor(self, os.path.join(self.tmp, "check.json"))

        def configure_reimport(d):
            self.assertEqual(d.unit_combo.currentText(), "m")         # $INSUNITS = metres
            self.assertFalse(d.use_hint_button.isVisible())
            d.pick_origin(0.0, 0.0)                                  # keep the exported coordinates
            for name in [n for n in dxf_io.LAYERS if n != "BELTS"] + ["0"]:
                try:
                    d.set_layer_visible(name, False)
                except KeyError:
                    pass
        again.import_background(exported, configure_reimport)
        self.assertEqual(len(again.cad._background_shapes), 4)        # only the 4 belt outlines
        again.look_at(10, 7.5)
        toolbar = again.cad.draw_toolbar
        for original in drawn:
            again.cad.pick_tool.start()
            toolbar.type_combo.setCurrentIndex(toolbar.type_combo.findData("auto"))
            inside = belt_footprint(original).representative_point()
            again.click(inside.x, inside.y)
            again.key(Qt.Key_Escape)
            rebuilt = again.belts()[-1]
            self.assertEqual(rebuilt.type, original.type, original.id)
            footprints_match(self, rebuilt, original, original.id)
            if original.type == "curved":
                self.assertAlmostEqual(rebuilt.x, original.x, places=6)
                self.assertAlmostEqual(rebuilt.y, original.y, places=6)
                self.assertAlmostEqual(rebuilt.radius, original.radius, places=6)
                self.assertAlmostEqual(rebuilt.beltWidth, original.beltWidth, places=6)
        self.assertEqual(len(again.belts()), 4)

    def test_sheet_size_is_kept_through_the_crop(self):
        info = dxf_io.inspect_drawing(self.drawing)
        sheet = next(s for s in info.sheets if "GROUND FLOOR" in s.name)
        out = os.path.join(self.tmp, "sheet.dxf")
        dxf_io.export_cropped(self.drawing, out, sheet.box, to_origin=True)
        g = dxf_io.read_background(out)
        w, h = g.bounds[2] - g.bounds[0], g.bounds[3] - g.bounds[1]
        self.assertAlmostEqual(w, SHEET_W, places=6)
        self.assertAlmostEqual(h, SHEET_H, places=6)


if __name__ == "__main__":
    unittest.main()
