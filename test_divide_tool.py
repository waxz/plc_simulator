"""
test_divide_tool.py - the designer's Divide into Belts (a line or arc of the
sketch or the drawing as several belts / roller conveyors of equal length,
joined end to end), the Arc (Center, Start, End) command, and wheel zoom
about the cursor. Core geometry without Qt; the editor driven with real
events.
"""
import math
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPoint, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QKeyEvent, QMouseEvent, QWheelEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from cad import path as cpath  # noqa: E402
from cad.belts import divide_segment, divided_belts  # noqa: E402
from cad.ports import belt_ports  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402

_app = QApplication.instance() or QApplication([])


def counter(prefix="belt"):
    n = [0]

    def new_id():
        n[0] += 1
        return f"{prefix}_{n[0]}"
    return new_id


class GeometryTest(unittest.TestCase):
    def test_a_line_in_equal_parts(self):
        seg = cpath.Segment("line", (0.0, 0.0), (6.0, 0.0))
        pieces = divide_segment(seg, 3)
        self.assertEqual([(p.p0, p.p1) for p in pieces], [((0, 0), (2, 0)), ((2, 0), (4, 0)), ((4, 0), (6, 0))])
        belts = divided_belts(counter(), seg, 3, 1.0)
        self.assertEqual([(b.type, round(b.x, 6), b.length) for b in belts],
                         [("linear", 1.0, 2.0), ("linear", 3.0, 2.0), ("linear", 5.0, 2.0)])

    def test_an_arc_in_equal_parts_joined_end_to_end(self):
        seg = cpath.bulge_to_segment((2.0, 0.0), (0.0, 2.0), math.tan(math.pi / 8))    # quarter circle, CCW
        belts = divided_belts(counter("roller"), seg, 3, 0.8, surface="roller")
        self.assertEqual({b.type for b in belts}, {"curved"})
        self.assertEqual({b.surface for b in belts}, {"roller"})
        self.assertTrue(all(abs(b.endAngle - b.startAngle - 30.0) < 1e-6 for b in belts))
        for a, b in zip(belts, belts[1:]):                       # each outlet meets the next inlet
            out, inlet = belt_ports(a).outlet, belt_ports(b).inlet
            self.assertAlmostEqual(math.hypot(out.x - inlet.x, out.y - inlet.y), 0.0, places=6)
        first, last = belt_ports(belts[0]).inlet, belt_ports(belts[-1]).outlet
        self.assertAlmostEqual(first.x, 2.0, places=6)
        self.assertAlmostEqual(last.y, 2.0, places=6)
        back = divided_belts(counter(), seg, 3, 0.8, reverse=True)
        self.assertAlmostEqual(belt_ports(back[0]).inlet.y, 2.0, places=6)          # from the arc's end

    def test_an_arc_whole_or_in_parts_belts_or_rollers(self):
        seg = cpath.bulge_to_segment((3.0, 0.0), (0.0, 3.0), math.tan(math.pi / 8))    # quarter circle
        for surface in ("belt", "roller"):
            (whole,) = divided_belts(counter(), seg, 1, 1.0, surface=surface)
            self.assertEqual((whole.type, whole.surface), ("curved", surface))
            self.assertAlmostEqual(whole.radius, 3.0, places=9)
            self.assertAlmostEqual(whole.endAngle - whole.startAngle, 90.0, places=6)
            parts = divided_belts(counter(), seg, 5, 1.0, surface=surface)
            self.assertEqual([(b.type, b.surface) for b in parts], [("curved", surface)] * 5)
            self.assertAlmostEqual(sum(b.endAngle - b.startAngle for b in parts), 90.0, places=6)

    def test_too_tight_for_the_width(self):
        seg = cpath.bulge_to_segment((0.4, 0.0), (0.0, 0.4), math.tan(math.pi / 8))
        with self.assertRaises(ValueError):
            divided_belts(counter(), seg, 2, 1.0)


class EditorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "layout.json")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.resize(1200, 800)
        self.window.show()
        _app.processEvents()
        self.view = self.window.view
        self.view.resetTransform()
        self.view.centerOn(to_scene_pos(3, 2))
        self.cad = self.window.cad
        self.bar = self.cad.draw_toolbar
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def _mouse(self, kind, x, y, button=Qt.LeftButton, buttons=None):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(to_scene_pos(x, y)))
        if buttons is None:
            buttons = Qt.NoButton if kind == QEvent.MouseButtonRelease else button
        _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos), button, buttons, Qt.NoModifier))

    def click(self, x, y):
        self._mouse(QEvent.MouseMove, x, y, Qt.NoButton, Qt.NoButton)
        self._mouse(QEvent.MouseButtonPress, x, y)
        self._mouse(QEvent.MouseButtonRelease, x, y)

    def action(self, text):
        return next(a for m in self.window.menuBar().findChildren(type(self.window.menuBar().addMenu("x")))
                    for a in m.actions() if a.text() == text)

    def belts(self):
        return [i.model for i in self.window.items_list]

    def test_divide_a_sketch_line_into_rollers(self):
        self.cad.sketch_line_tool.start()
        for text in ("0,0", "6,0"):
            self.cad.sketch_line_tool.typed_input(text)
        self.cad.sketch_line_tool.finish()
        divide = self.action("Divide into Belts")
        self.assertEqual(divide.shortcut().toString(), "N")
        divide.trigger()
        self.bar.parts_spin.setValue(3)
        self.cad.surface_combo.setCurrentIndex(self.cad.surface_combo.findData("roller"))   # the tool bar's
        self._mouse(QEvent.MouseMove, 2.5, 0.02, Qt.NoButton, Qt.NoButton)
        self.assertGreaterEqual(len(self.cad.divide_tool._preview_items), 4)       # 3 pieces, cuts, arrow
        self.click(2.5, 0.02)
        rollers = self.belts()
        self.assertEqual([(b.id, b.surface, b.length) for b in rollers],
                         [("roller_1", "roller", 2.0), ("roller_2", "roller", 2.0), ("roller_3", "roller", 2.0)])
        links = {(c.fromBelt, c.toBelt) for c in self.window.config.connections}
        self.assertEqual(links, {("roller_1", "roller_2"), ("roller_2", "roller_3")})
        self.window.undo_stack.undo()                                               # one undo step
        self.assertEqual(self.belts(), [])
        self.assertEqual(self.window.config.connections, [])

    def test_divide_a_drawn_arc_into_belts_reverse(self):
        self.bar.arc_method_combo.setCurrentIndex(self.bar.arc_method_combo.findData("3pt"))
        self.cad.sketch_arc_tool.start()
        for text in ("4,0", "0,4", "2.8284,2.8284"):                  # a quarter circle about the origin
            self.cad.sketch_arc_tool.typed_input(text)
        self.cad.divide_tool.start()
        self.assertEqual(self.bar.parts_spin.minimum(), 1)                       # 1: the whole arc
        self.bar.parts_spin.setValue(1)
        self.click(4 * math.cos(math.radians(60)), 4 * math.sin(math.radians(60)))
        (whole,) = self.belts()
        self.assertAlmostEqual(whole.endAngle - whole.startAngle, 90.0, delta=0.01)   # typed 2.8284
        self.window.undo_stack.undo()
        self.bar.parts_spin.setValue(2)
        self.bar.divide_direction_combo.setCurrentIndex(self.bar.divide_direction_combo.findData("reverse"))
        self.click(4 * math.cos(math.radians(20)), 4 * math.sin(math.radians(20)))
        belts = self.belts()
        self.assertEqual([b.type for b in belts], ["curved", "curved"])
        self.assertTrue(all(b.surface == "belt" for b in belts))
        self.assertAlmostEqual(belt_ports(belts[0]).inlet.y, 4.0, places=3)        # from the arc's end
        self.cad.divide_tool.stop()

    def test_nothing_there(self):
        self.cad.divide_tool.start()
        self.click(1, 1)
        self.assertEqual(self.belts(), [])
        self.assertIn("click on a line or an arc", self.window.statusBar().currentMessage())

    def test_arc_from_center_start_end(self):
        arc = self.action("Arc (Center, Start, End)")
        self.assertEqual(arc.shortcut().toString(), "Shift+A")
        arc.trigger()
        self.assertEqual(self.bar.arc_method_combo.currentData(), "center")
        tool = self.cad.sketch_arc_tool
        for text in ("0,0", "2,0", "0,2"):                     # center, start (radius 2), end
            tool.typed_input(text)
        (entity,) = self.window.config.sketch
        seg = cpath.segments(entity.points, entity.bulges, entity.closed)[0]
        self.assertEqual(seg.kind, "arc")
        self.assertAlmostEqual(seg.radius, 2.0, places=6)
        self.assertAlmostEqual(seg.center[0], 0.0, places=6)
        self.assertAlmostEqual(seg.center[1], 0.0, places=6)

    def test_wheel_zoom_keeps_the_point_under_the_cursor(self):
        view, vp = self.view, self.view.viewport()
        for pos in (QPoint(15, 15), QPoint(vp.width() - 15, vp.height() - 15), QPoint(vp.width() // 3, 120)):
            for delta in (120,) * 6 + (-120,) * 12:
                before = view.mapToScene(pos)
                _app.sendEvent(vp, QWheelEvent(QPointF(pos), QPointF(vp.mapToGlobal(pos)), QPoint(0, 0),
                                               QPoint(0, delta), Qt.NoButton, Qt.NoModifier, Qt.NoScrollPhase,
                                               False))
                after = view.mapToScene(pos)
                px = view.transform().m11()
                self.assertLess(abs(after.x() - before.x()) * px, 1.01)
                self.assertLess(abs(after.y() - before.y()) * px, 1.01)


if __name__ == "__main__":
    unittest.main()
