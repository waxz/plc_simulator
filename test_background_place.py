"""
test_background_place.py - moving the imported DXF drawing in the layout
editor: dragging its placement anchor (snapping to the sketch), the Place
Background tool (anchor point, then target; typed points), one undo step
each, without reading the DXF file again; plus the icon tool bars.
"""
import os
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QToolBar  # noqa: E402

from config_parser import BackgroundDrawing, SimulatorConfig, SketchEntity, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402

_app = QApplication.instance() or QApplication([])


class BackgroundPlaceTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        doc = ezdxf.new()
        msp = doc.modelspace()
        msp.add_lwpolyline([(0, 0), (4000, 0), (4000, 3000), (0, 3000)], close=True)     # a 4 x 3 m room, mm
        msp.add_line((1000, 1000), (2000, 1000))
        doc.saveas(os.path.join(self.tmp.name, "hall.dxf"))
        path = os.path.join(self.tmp.name, "layout.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(path), config_path=path)
        self.window.resize(1400, 900)
        self.window.show()
        _app.processEvents()
        self.cad = self.window.cad
        self.view = self.window.view
        self.view.resetTransform()
        self.view.centerOn(to_scene_pos(5, 2))
        self.cad.set_background(BackgroundDrawing(path="hall.dxf", scale=0.001))
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def offset(self):
        bg = self.window.config.background
        return round(bg.offsetX, 6), round(bg.offsetY, 6)

    def _send(self, kind, scene_pos, button=Qt.LeftButton):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(scene_pos))
        buttons = button if kind in (QEvent.MouseButtonPress, QEvent.MouseMove) else Qt.NoButton
        _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos),
                                       Qt.NoButton if kind == QEvent.MouseMove else button, buttons, Qt.NoModifier))

    def test_drag_the_anchor_snaps_to_the_sketch_and_undoes(self):
        self.cad.set_sketch([SketchEntity(points=[(6, 2), (9, 2)], bulges=[0.0, 0.0])])
        anchor = self.cad.background_anchor
        self.assertEqual(anchor.home, (0.0, 0.0))                   # the drawing's lower-left corner
        with mock.patch("cad.dxf_io.read_background", side_effect=AssertionError("re-read")):
            self._send(QEvent.MouseButtonPress, anchor.scenePos())
            self._send(QEvent.MouseMove, to_scene_pos(6.02, 2.01))    # near the sketch line's end
            self._send(QEvent.MouseButtonRelease, to_scene_pos(6.02, 2.01))
            _app.processEvents()
        self.assertEqual(self.offset(), (6.0, 2.0))
        self.assertEqual(self.cad.background_anchor.home, (6.0, 2.0))
        xs = [p[0] for s in self.cad._background_shapes for p in s.points]
        self.assertAlmostEqual(min(xs), 6.0)
        self.window.undo_stack.undo()
        self.assertEqual(self.offset(), (0.0, 0.0))

    def test_place_tool_by_two_points(self):
        self.cad.place_bg_action.trigger()
        tool = self.cad.place_tool
        self.assertTrue(tool.active)
        tool.typed_input("1,1")                                     # the line's start in the drawing
        tool.typed_input("@10,-1")
        self.assertFalse(tool.active)
        self.assertEqual(self.offset(), (10.0, -1.0))
        self.assertEqual(self.cad.background_item.pos(), QPointF(0, 0))
        self.window.undo_stack.undo()
        self.assertEqual(self.offset(), (0.0, 0.0))

    def test_place_tool_without_a_background(self):
        self.cad.set_background(None)
        self.cad.place_tool.start()
        self.assertFalse(self.cad.place_tool.active)
        self.assertIn("import a DXF background first", self.window.statusBar().currentMessage())

    def test_icon_tool_bars(self):
        names = {b.objectName() for b in self.window.findChildren(QToolBar)}
        for name in ("fileToolbar", "backgroundToolbar", "viewToolbar", "sketchToolbar", "modifyToolbar"):
            self.assertIn(name, names)
        bg_bar = self.window.findChild(QToolBar, "backgroundToolbar")
        place = next(a for a in bg_bar.actions() if a.text() == "Place Background")
        self.assertEqual(place.toolTip(), "Place Background (Shift+B)")
        for bar in self.window.findChildren(QToolBar):
            if bar is self.cad.draw_toolbar:
                continue
            self.assertEqual(bar.toolButtonStyle(), Qt.ToolButtonIconOnly, bar.objectName())
            for action in bar.actions():
                if action.text() and not action.isSeparator():
                    self.assertFalse(action.icon().isNull(), action.text())
                    self.assertTrue(action.toolTip(), action.text())
        QTest.mouseClick(bg_bar.widgetForAction(place), Qt.LeftButton)
        self.assertTrue(self.cad.place_tool.active)
        self.window.set_theme("light")                              # icons redrawn, not lost
        self.assertFalse(place.icon().isNull())
        self.window.set_theme("dark")


if __name__ == "__main__":
    unittest.main()
