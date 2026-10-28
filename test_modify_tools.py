"""
test_modify_tools.py - the layout editor's Modify tools (modify_tool.py)
driven like a user: the Modify tool bar / menu, clicks on sketch lines,
typed points, undo, and the clipboard (Ctrl+C / Ctrl+V / Ctrl+X) for
sketch lines and components together.
"""
import math
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QAction, QKeyEvent, QKeySequence, QMouseEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QToolBar  # noqa: E402

from cad.path import path_length, segments  # noqa: E402
from config_parser import SimulatorConfig, SketchEntity, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from layout_editor_cad import SketchItem  # noqa: E402

_app = QApplication.instance() or QApplication([])


def line(a, b):
    return SketchEntity(points=[a, b], bulges=[0.0, 0.0])


class ModifyToolsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = os.path.join(self.tmp.name, "layout.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(path), config_path=path)
        self.window.resize(1400, 900)
        self.window.show()
        _app.processEvents()
        self.view = self.window.view
        self.view.resetTransform()
        self.view.centerOn(to_scene_pos(5, 0))
        self.cad = self.window.cad
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    # -- input -----------------------------------------------------------------------

    def _mouse(self, kind, x, y, button=Qt.LeftButton):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(to_scene_pos(x, y)))
        buttons = Qt.NoButton if kind in (QEvent.MouseButtonRelease, QEvent.MouseMove) else button
        _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos),
                                       Qt.NoButton if kind == QEvent.MouseMove else button, buttons, Qt.NoModifier))

    def click(self, x, y):
        self._mouse(QEvent.MouseMove, x, y)
        self._mouse(QEvent.MouseButtonPress, x, y)
        self._mouse(QEvent.MouseButtonRelease, x, y)

    def key(self, key, mods=Qt.NoModifier):
        _app.sendEvent(self.view, QKeyEvent(QEvent.KeyPress, key, mods, ""))

    def tool_button(self, text):
        bar = self.window.findChild(QToolBar, "modifyToolbar")
        action = next(a for a in bar.actions() if a.text() == text)
        return bar.widgetForAction(action)

    def start(self, text):
        QTest.mouseClick(self.tool_button(text), Qt.LeftButton)

    def select(self, *indices):
        self.window.scene.clearSelection()
        for item in self.cad.sketch_items:
            if item.index in indices:
                item.setSelected(True)

    def sketch(self):
        return [(tuple(round(v, 6) for v in e.points[0]), tuple(round(v, 6) for v in e.points[-1]))
                for e in self.window.config.sketch]

    # -- the tool bar ----------------------------------------------------------------

    def test_modify_tool_bar_and_menu(self):
        bar = self.window.findChild(QToolBar, "modifyToolbar")
        self.assertEqual(bar.orientation(), Qt.Vertical)
        names = [a.text() for a in bar.actions() if a.text()]
        for name in ("Trim", "Extend", "Break", "Offset", "Fillet", "Join", "Explode",
                     "Move", "Copy", "Rotate", "Mirror", "Scale"):
            self.assertIn(name, names)
        trim = next(a for a in self.window.findChildren(QAction) if a.text() == "Trim")
        self.assertEqual(trim.shortcut(), QKeySequence("T"))

    # -- cut-type tools --------------------------------------------------------------

    def test_trim_between_crossings_repeats_and_undoes(self):
        self.cad.set_sketch([line((0, 0), (10, 0)), line((3, -2), (3, 2)), line((7, -2), (7, 2))])
        self.start("Trim")
        self.assertTrue(self.cad.modify_tools["trim"].active)
        self.click(5, 0.02)                                         # the middle piece
        self.assertEqual(self.sketch()[:2], [((0, 0), (3, 0)), ((7, 0), (10, 0))])
        self.click(3, 1.5)                                          # a crossing line's upper end: still active
        self.assertIn(((3, -2), (3, 0)), self.sketch())
        self.key(Qt.Key_Escape)
        self.assertFalse(self.cad.modify_tools["trim"].active)
        self.window.undo_stack.undo()
        self.window.undo_stack.undo()
        self.assertEqual(self.sketch()[0], ((0, 0), (10, 0)))

    def test_trim_to_the_background_drawing(self):
        from cad.dxf_io import DrawingShape
        self.cad.set_sketch([line((0, 0), (10, 0))])
        self.cad._background_shapes = [DrawingShape([(4, -1), (4, 1)], [0.0, 0.0], False)]
        self.start("Trim")
        self.click(8, 0)
        self.assertEqual(self.sketch(), [((0, 0), (4, 0))])

    def test_extend_and_break(self):
        self.cad.set_sketch([line((0, 0), (6, 0)), line((10, -3), (10, 3))])
        self.start("Extend")
        self.click(5.5, 0)
        self.assertEqual(self.sketch()[0], ((0, 0), (10, 0)))
        self.key(Qt.Key_Escape)
        self.start("Break")
        self.cad.modify_tools["break"].typed_input("4,0")
        self.assertEqual(self.sketch()[:2], [((0, 0), (4, 0)), ((4, 0), (10, 0))])

    def test_offset_with_the_tool_bar_distance(self):
        self.cad.set_sketch([line((0, 0), (10, 0))])
        self.start("Offset")
        self.cad.draw_toolbar.distance_spin.setValue(1.5)
        self.assertTrue(all(a.isVisible() for a in self.cad.draw_toolbar.distance_actions))
        self.click(5, 0)                                            # the line
        self.click(5, 3)                                            # the side
        self.assertEqual(self.sketch()[1], ((0, 1.5), (10, 1.5)))

    def test_fillet_two_lines(self):
        self.cad.set_sketch([line((0, 0), (5, 0)), line((7, 2), (7, 8))])
        self.start("Fillet")
        self.cad.draw_toolbar.corner_spin.setValue(1.0)
        self.click(1, 0)
        self.click(7, 6)
        (joined,) = self.window.config.sketch
        segs = segments(joined.points, joined.bulges, joined.closed)
        self.assertEqual([s.kind for s in segs], ["line", "arc", "line"])
        self.assertAlmostEqual(segs[1].radius, 1.0)

    # -- join / explode ----------------------------------------------------------------

    def test_join_and_explode_the_selection(self):
        self.cad.set_sketch([line((0, 0), (4, 0)), line((4, 0), (4, 3)), line((9, 9), (9, 10))])
        self.select(0, 1)
        self.start("Join")
        self.assertEqual(len(self.window.config.sketch), 2)
        joined = next(e for e in self.window.config.sketch if len(e.points) == 3)
        self.assertAlmostEqual(path_length(segments(joined.points, joined.bulges, False)), 7.0)
        self.select(next(i for i, e in enumerate(self.window.config.sketch) if len(e.points) == 3))
        self.start("Explode")
        self.assertEqual(len(self.window.config.sketch), 3)

    # -- transforms ------------------------------------------------------------------

    def test_move_copy_rotate_mirror_scale(self):
        self.cad.set_sketch([line((0, 0), (2, 0))])
        self.select(0)
        self.start("Move")
        self.cad.modify_tools["move"].typed_input("0,0")
        self.cad.modify_tools["move"].typed_input("@1,1")
        self.assertEqual(self.sketch(), [((1, 1), (3, 1))])

        self.select(0)
        self.start("Copy")
        tool = self.cad.modify_tools["copy"]
        tool.typed_input("1,1")
        tool.typed_input("1,5")
        tool.typed_input("1,9")                                     # copies until Esc
        self.key(Qt.Key_Escape)
        self.assertEqual(self.sketch(), [((1, 1), (3, 1)), ((1, 5), (3, 5)), ((1, 9), (3, 9))])

        self.select(0)
        self.start("Rotate")
        self.cad.modify_tools["rotate"].typed_input("1,1")
        self.cad.modify_tools["rotate"].typed_input("90")           # 90 deg about (1, 1)
        self.assertEqual(self.sketch()[0], ((1, 1), (1, 3)))

        self.select(0)
        self.start("Mirror")
        self.assertTrue(self.cad.draw_toolbar.keep_check.isChecked())
        self.cad.modify_tools["mirror"].typed_input("0,0")
        self.cad.modify_tools["mirror"].typed_input("0,1")          # about the y axis, originals kept
        self.assertIn(((-1, 1), (-1, 3)), self.sketch())
        self.assertIn(((1, 1), (1, 3)), self.sketch())

        self.select(0)
        self.start("Scale")
        self.cad.modify_tools["scale"].typed_input("1,1")
        self.cad.modify_tools["scale"].typed_input("2")             # factor 2 (a point 2 m away)
        pts = self.window.config.sketch[0].points
        self.assertAlmostEqual(math.dist(pts[0], pts[1]), 4.0)

    def test_transforms_need_a_selection(self):
        self.cad.set_sketch([line((0, 0), (2, 0))])
        self.window.scene.clearSelection()
        self.start("Move")
        self.assertFalse(self.cad.modify_tools["move"].active)
        self.assertIn("select sketch lines first", self.window.statusBar().currentMessage())

    # -- clipboard ---------------------------------------------------------------------

    def test_cut_copy_paste_sketch_lines(self):
        self.cad.set_sketch([line((0, 0), (2, 0))])
        self.select(0)
        self.key(Qt.Key_C, Qt.ControlModifier)
        self._mouse(QEvent.MouseMove, 5, 5)
        self.key(Qt.Key_V, Qt.ControlModifier)
        self.assertEqual(len(self.window.config.sketch), 2)
        pasted = self.window.config.sketch[1].points
        self.assertAlmostEqual((pasted[0][0] + pasted[1][0]) / 2, 5.0, places=1)    # centred on the mouse
        self.select(0)
        self.key(Qt.Key_X, Qt.ControlModifier)                      # cut: gone, still on the clipboard
        self.assertEqual(len(self.window.config.sketch), 1)
        self.key(Qt.Key_V, Qt.ControlModifier)
        self.assertEqual(len(self.window.config.sketch), 2)

    def test_cut_components_and_sketch_together(self):
        from layout_editor import BeltItem
        from config_parser import Belt
        self.cad.set_sketch([line((0, 0), (2, 0))])
        belt = Belt(id="belt_1", type="linear", x=1.0, y=3.0, length=2.0, beltWidth=1.0, rotation=0.0, speed=1.0,
                    directionAngle=0.0, color="#888888")
        item = self.view._add_item(BeltItem(belt))
        self.window.scene.clearSelection()
        item.setSelected(True)
        self.cad.sketch_items[0].setSelected(True)
        self.key(Qt.Key_X, Qt.ControlModifier)
        self.assertEqual(self.window.config.sketch, [])
        self.assertEqual(self.window.items_list, [])
        self.window.undo_stack.undo()                               # one step brings both back
        self.assertEqual(len(self.window.config.sketch), 1)
        self.assertEqual(len(self.window.items_list), 1)
        self.key(Qt.Key_V, Qt.ControlModifier)                      # the cut pair pastes as a pair
        self.assertEqual(len(self.window.config.sketch), 2)
        self.assertEqual(len(self.window.items_list), 2)


if __name__ == "__main__":
    unittest.main()
