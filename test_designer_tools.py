"""
test_designer_tools.py - the designer's measure window, a belt's AX start / end
(highlighted, set by input, drag or a pick on the canvas), its number fields
(6 decimals, typed anywhere, applied on Enter) and a designer closing with the
simulator that started it.
"""
import os
import subprocess
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QPointF, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import ax_geometry  # noqa: E402
import layout_editor as le  # noqa: E402
from config_parser import load_config  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
APP = QApplication.instance() or QApplication([])


class Base(unittest.TestCase):
    def setUp(self):
        self.window = le.MainWindow(config=load_config(os.path.join(HERE, "merge_demo.json")),
                                    config_path=os.path.join(tempfile.mkdtemp(), "layout.json"))
        self.addCleanup(self.window.deleteLater)
        self.addCleanup(self.window.close)
        self.addCleanup(self.window.undo_stack.setClean)               # a test's edits: never a save question
        self.window.show()
        self.items = {i.model.id: i for i in self.window.items_list}

    def select(self, ident):
        self.window.scene.clearSelection()
        self.items[ident].setSelected(True)
        APP.processEvents()
        return self.window.properties_panel


class MeasureTest(Base):
    def test_point_path_and_angle_and_nothing_left_behind(self):
        w = self.window
        w.measure_action.setChecked(True)
        d = w.measure_dialog
        self.assertTrue(d.isVisible())
        d.pick(("point", le.to_scene_pos(0, 0)), None, le.to_scene_pos(0, 0))      # where the clicks were
        d.pick(("point", le.to_scene_pos(3, 4)), None, le.to_scene_pos(3, 4))
        self.assertIn("distance: 5.000 m", d.result.toPlainText())
        d.buttons["path"].setChecked(True)
        for ident in ("belt_2_b", "belt_2_c", "curve"):                 # lines and a curve
            d.pick(("free", QPointF()), self.items[ident], QPointF())
        self.assertIn("total: 2.171 m (3)", d.result.toPlainText())       # 0.3 + 0.3 + pi / 2
        d.pick(("free", QPointF()), self.items["curve"], QPointF())       # again: taken off
        self.assertIn("total: 0.600 m (2)", d.result.toPlainText())
        d.buttons["angle"].setChecked(True)
        d.pick(("free", QPointF()), self.items["infeed"], QPointF())
        d.pick(("free", QPointF()), self.items["belt_2"], QPointF())
        self.assertIn("angle between: 30.00 deg", d.result.toPlainText())
        self.assertTrue(w.scene._measure_items)                            # highlighted while open
        d.close()
        self.assertEqual(w.scene._measure_items, [])                       # nothing left on the canvas
        self.assertFalse(w.measure_action.isChecked())


class AxPortsTest(Base):
    def test_a_belts_start_and_end_set_by_input_drag_or_pick(self):
        panel = self.select("belt_2_a")
        belt = self.items["belt_2_a"]
        path = lambda: ax_geometry.belt_path(self.window.config, belt.model)   # noqa: E731
        self.assertEqual(len(panel._port_handles), 2)                      # highlighted: start and end
        self.assertTrue(all(h.flags() & le.QGraphicsItem.ItemIsMovable for h in panel._port_handles))
        self.assertIn("port.inX", panel.spins)                             # by input
        before = path().start[:2]
        panel._set_port(belt, "in", (-0.7, 1.8))                           # a drag ends here
        APP.processEvents()
        self.assertAlmostEqual(path().start[0], -0.7, places=5)
        self.assertAlmostEqual(le.from_scene_pos(panel._port_handles[0].pos())[0], -0.7, places=5)
        self.window.undo_stack.undo()
        self.assertEqual((belt.model.inletPoint, belt.model.outletPoint), (None, None))   # exactly as it was
        self.assertEqual(path().start[:2], before)
        panel._pick_port("out")                                            # a pick: the next click
        self.assertIsNotNone(self.window.view.pick_point)
        self.window.view.pick_point((0.6, 1.1))
        self.assertAlmostEqual(path().end[1], 1.1, places=5)

    def test_a_curves_start_and_end_are_shown_not_moved(self):
        panel = self.select("curve")
        self.assertEqual(len(panel._port_handles), 2)
        self.assertFalse(any(h.flags() & le.QGraphicsItem.ItemIsMovable for h in panel._port_handles))
        self.assertNotIn("port.inX", panel.spins)


class NumberFieldTest(Base):
    def test_six_decimals_typed_anywhere_applied_on_enter(self):
        panel = self.select("belt_2_b")
        spin, belt = panel.spins["speed"], self.items["belt_2_b"].model
        self.assertEqual(spin.text(), "2.3094 m/s")                         # not rounded to 2.309
        edit = spin.lineEdit()
        edit.setFocus()
        edit.selectAll()
        QTest.keyClicks(edit, "1.23456")
        self.assertEqual(belt.speed, 2.3094)                                # not applied while typing
        QTest.keyClick(edit, Qt.Key_Return)
        self.assertAlmostEqual(belt.speed, 1.23456)
        edit.setCursorPosition(1)
        QTest.keyClicks(edit, "0")                                          # a digit in the middle
        QTest.keyClick(edit, Qt.Key_Return)
        self.assertAlmostEqual(belt.speed, 10.23456)
        spin.setValue(2.0)
        self.assertEqual(spin.text(), "2.000 m/s")


class FieldsTest(Base):
    def test_undo_from_a_field_and_a_number_typed_over_its_point(self):
        panel = self.select("belt_2_b")
        spin, belt = panel.spins["speed"], self.items["belt_2_b"].model
        edit = spin.lineEdit()
        edit.setFocus()
        edit.selectAll()
        QTest.keyClicks(edit, "1.5")
        QTest.keyClick(edit, Qt.Key_Return)
        self.assertAlmostEqual(belt.speed, 1.5)
        QTest.keyClick(edit, Qt.Key_Z, Qt.ControlModifier)                 # the focus still in the field
        self.assertAlmostEqual(belt.speed, 2.3094)                         # the window's undo ran
        edit = panel.spins["speed"].lineEdit()
        edit.setFocus()
        edit.setCursorPosition(2)                                          # after "2."
        QTest.keyClick(edit, Qt.Key_Backspace)                             # the point ...
        QTest.keyClick(edit, Qt.Key_Backspace)                             # ... and the number before it
        self.assertTrue(edit.text().startswith("3094"), edit.text())      # allowed while typing
        QTest.keyClicks(edit, "1.")
        QTest.keyClick(edit, Qt.Key_Return)
        self.assertAlmostEqual(belt.speed, 1.3094)

    def test_the_wheel_changes_no_drop_down(self):
        from PySide6.QtCore import QPoint
        from PySide6.QtGui import QWheelEvent
        panel = self.select("div_1")
        combo = panel.combos["diverterType"]
        before = combo.currentText()
        wheel = QWheelEvent(QPointF(5, 5), QPointF(5, 5), QPoint(0, 0), QPoint(0, -120), Qt.NoButton,
                            Qt.NoModifier, Qt.NoScrollPhase, False)
        QApplication.sendEvent(combo, wheel)
        self.assertEqual(combo.currentText(), before)

    def test_the_panels_device_table_and_its_buttons_fit_its_row(self):
        panel = self.select(self.window.config.controlPanels[0].id)
        editor = panel.elements_editor
        APP.processEvents()
        self.assertGreaterEqual(editor.height(), editor.table.height() + 20)        # room for the buttons
        rows = []
        for i in range(panel.tree.topLevelItemCount()):
            group = panel.tree.topLevelItem(i)
            rows += [group.child(k) for k in range(group.childCount())]
        row = next(r for r in rows if panel.tree.itemWidget(r, 0) is editor)
        self.assertEqual(row.sizeHint(0).height(), editor.height())


class SnapAndCurvesTest(Base):
    def test_a_pick_snaps_to_a_belts_end_and_a_diverters_zone_is_set(self):
        snap = self.window.view.snap_point
        self.assertIsNotNone(snap)
        self.assertEqual(snap(le.to_scene_pos(0.03, 0.02)), (0.0, 0.0))     # the infeed's start
        panel = self.select("div_1")
        div = self.items["div_1"]
        self.assertIn("zone.inX", panel.spins)
        self.assertEqual(len(panel._port_handles), 2)
        self.assertTrue(all(h.flags() & le.QGraphicsItem.ItemIsMovable for h in panel._port_handles))
        panel._set_port(div, "in", (8.1, 0.0))
        APP.processEvents()
        self.assertEqual(div.model.axStart, [8.1, 0.0])
        self.assertIn("set by hand: begin", ax_geometry.describe_point(self.window.config, div.model))
        self.window.undo_stack.undo()
        self.assertEqual((div.model.axStart, div.model.axEnd), (None, None))

    def test_path_length_takes_an_arc(self):
        from types import SimpleNamespace
        self.window.measure_action.setChecked(True)
        d = self.window.measure_dialog
        d.buttons["path"].setChecked(True)
        quarter = SimpleNamespace(points=[(0.0, 0.0), (1.0, 1.0)], bulges=[0.41421356, 0.0], closed=False)
        d._curve_at = lambda pos: ("sketch 1", "sketch 1", quarter)        # a sketch arc under the click
        d.pick(("free", QPointF()), None, QPointF())
        self.assertIn("sketch 1 (1 segment, 1 arc): 1.571 m", d.result.toPlainText())   # r = 1, 90 deg
        d.close()


class ExitWithTest(Base):
    def test_the_designer_closes_with_its_simulator(self):
        import sim_link
        child = subprocess.Popen([sys.executable, "-c", "pass"])
        child.wait()
        self.assertFalse(sim_link.process_alive(child.pid))
        self.assertTrue(sim_link.process_alive(os.getpid()))
        self.window.undo_stack.setClean()                                 # nothing unsaved: no question
        timer = le.exit_with_process(APP, child.pid, poll_ms=10)
        for _ in range(300):                                              # up to 3 s: a new window's setup
            if not self.window.isVisible():                               # events go first
                break
            QTest.qWait(10)
        self.assertFalse(self.window.isVisible())                         # closed (nothing unsaved)
        self.assertFalse(timer.isActive())


if __name__ == "__main__":
    unittest.main()
