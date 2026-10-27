"""
test_path_tool.py - drawing and editing path belts in the layout editor
(path_tool.py), driven by real mouse/key events on the canvas: clicking
anchors, line/arc modes, typed coordinates, Shift angle snap, closing a
loop, shape presets, anchor handles, undo, and running a drawn loop in the
simulator. Qt runs offscreen, so no window appears.
"""
import math
import os
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QKeyEvent, QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from cad import path as cpath  # noqa: E402
from cad.checks import belt_footprint  # noqa: E402
from cad.ports import belt_ports  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from path_tool import AnchorGrip, InsertShapeDialog  # noqa: E402
from physics import PhysicsEngine  # noqa: E402

_app = QApplication.instance() or QApplication([])


class PathToolTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "paths.json")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.resize(1200, 800)
        self.window.show()
        _app.processEvents()
        self.view = self.window.view
        self.view.resetTransform()
        self.view.centerOn(to_scene_pos(3, 2))
        self.tool = self.window.cad.path_tool
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    # -- input helpers -----------------------------------------------------------

    def _mouse(self, kind, x, y, button=Qt.LeftButton, buttons=None, mods=Qt.NoModifier):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(to_scene_pos(x, y)))
        if buttons is None:
            buttons = Qt.NoButton if kind == QEvent.MouseButtonRelease else button
        _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos), button, buttons, mods))

    def click(self, x, y, button=Qt.LeftButton, mods=Qt.NoModifier):
        self._mouse(QEvent.MouseMove, x, y, Qt.NoButton, Qt.NoButton, mods)
        self._mouse(QEvent.MouseButtonPress, x, y, button, mods=mods)
        self._mouse(QEvent.MouseButtonRelease, x, y, button, mods=mods)

    def key(self, key, text=""):
        _app.sendEvent(self.view, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier, text))

    def drawn(self):
        return [i for i in self.window.items_list if i.model.type == "path"]

    @staticmethod
    def world_anchors(belt):
        return cpath.belt_world_path(belt)[0]

    # -- drawing -----------------------------------------------------------------

    def test_clicks_draw_an_open_path_with_rounded_corner(self):
        self.tool.toolbar.corner_spin.setValue(1.0)
        self.tool.toolbar.width_spin.setValue(0.8)
        self.tool.start()
        for x, y in ((0, 0), (6, 0), (6, 4)):
            self.click(x, y)
        self.key(Qt.Key_Return)
        self.assertFalse(self.tool.active)
        (item,) = self.drawn()
        belt = item.model
        self.assertFalse(belt.closed)
        self.assertEqual(belt.beltWidth, 0.8)
        pts = self.world_anchors(belt)
        self.assertEqual(len(pts), 4)  # the corner became two tangent points + an arc
        self.assertAlmostEqual(pts[1][0], 5.0, places=6)
        self.assertAlmostEqual(pts[2][1], 1.0, places=6)
        ports = belt_ports(belt)
        self.assertAlmostEqual(ports.inlet.heading % 360, 0.0, places=6)
        self.assertAlmostEqual(ports.outlet.heading % 360, 90.0, places=6)
        self.window.undo_stack.undo()
        self.assertEqual(self.drawn(), [])

    def test_arc_mode_adds_a_tangent_arc(self):
        self.tool.toolbar.corner_spin.setValue(0.0)
        self.tool.start()
        self.click(0, 0)
        self.click(4, 0)
        self.key(Qt.Key_A, "a")
        self.click(6, 2)
        self.key(Qt.Key_Return)
        belt = self.drawn()[0].model
        segs = cpath.segments(*cpath.belt_world_path(belt), False)
        self.assertEqual([s.kind for s in segs], ["line", "arc"])
        self.assertAlmostEqual(segs[1].heading_at(0.0), segs[0].heading_at(1.0), places=9)  # smooth
        self.assertAlmostEqual(segs[1].radius, 2.0, places=6)

    def test_typed_coordinates_place_exact_anchors(self):
        self.tool.toolbar.corner_spin.setValue(0.0)
        self.tool.start()
        for text in ("1.25,0.5", "@5,0", "3<90"):
            self.tool.typed_input(text)
        self.key(Qt.Key_Return)
        pts = self.world_anchors(self.drawn()[0].model)
        for (x, y), (ex, ey) in zip(pts, [(1.25, 0.5), (6.25, 0.5), (6.25, 3.5)]):
            self.assertAlmostEqual(x, ex, places=9)
            self.assertAlmostEqual(y, ey, places=9)

    def test_typing_a_digit_while_drawing_goes_to_the_input_box(self):
        self.tool.start()
        self.key(Qt.Key_2, "2")
        self.assertEqual(self.tool.toolbar.input.text(), "2")

    def test_clicking_the_first_anchor_closes_a_loop(self):
        self.tool.toolbar.corner_spin.setValue(0.5)
        self.tool.start()
        for x, y in ((0, 0), (6, 0), (3, 4), (0, 0)):
            self.click(x, y)
        belt = self.drawn()[0].model
        self.assertTrue(belt.closed)
        self.assertEqual(len(belt.path), 6)  # 3 corners, each filleted into 2 anchors
        self.assertEqual(len(belt_footprint(belt).interiors), 1)

    def test_shift_snaps_the_segment_angle(self):
        self.tool.toolbar.corner_spin.setValue(0.0)
        self.tool.start()
        self.click(0, 0)
        self.click(3.3, 0.33, mods=Qt.ShiftModifier)  # ~5.7 deg -> 0 deg
        self.key(Qt.Key_Return)
        pts = self.world_anchors(self.drawn()[0].model)
        self.assertAlmostEqual(pts[1][1], 0.0, places=9)

    def test_backspace_and_escape(self):
        self.tool.start()
        self.click(0, 0)
        self.click(4, 0)
        self.key(Qt.Key_Backspace)
        self.assertEqual(len(self.tool.anchors), 1)
        self.key(Qt.Key_Escape)
        self.assertFalse(self.tool.active)
        self.assertEqual(self.drawn(), [])
        self.assertIsNone(self.view.tool)

    # -- presets -----------------------------------------------------------------

    def test_insert_shape_preset(self):
        values = {"kind": "rectangle", "size": 10.0, "size_b": 4.0, "sides": 6, "corner_radius": 1.0, "width": 1.2}
        with mock.patch.object(InsertShapeDialog, "exec", return_value=InsertShapeDialog.Accepted), \
                mock.patch.object(InsertShapeDialog, "values", return_value=values):
            self.window.cad.insert_shape()
        belt = self.drawn()[0].model
        self.assertTrue(belt.closed)
        self.assertEqual(belt.beltWidth, 1.2)
        length = cpath.path_length(cpath.segments(*cpath.belt_local_path(belt), True))
        self.assertAlmostEqual(length, 28 - 8 + 2 * math.pi, places=6)

    # -- anchor handles ------------------------------------------------------------

    def _draw_straight(self):
        self.tool.toolbar.corner_spin.setValue(0.0)
        self.tool.start()
        for x, y in ((0, 0), (6, 0), (6, 4)):
            self.click(x, y)
        self.key(Qt.Key_Return)
        item = self.drawn()[0]
        self.window.scene.clearSelection()
        item.setSelected(True)
        _app.processEvents()
        return item

    def test_selecting_a_path_belt_shows_one_handle_per_anchor(self):
        item = self._draw_straight()
        grips = self.window.cad.grips.grips
        self.assertEqual(len(grips), 3)
        self.assertTrue(all(isinstance(g, AnchorGrip) and g.parentItem() is item for g in grips))

    def test_dragging_a_handle_moves_the_anchor_and_undo_restores_it(self):
        item = self._draw_straight()
        before = self.world_anchors(item.model)
        self._mouse(QEvent.MouseButtonPress, 6, 4)
        self._mouse(QEvent.MouseMove, 7, 5, Qt.NoButton, Qt.LeftButton)
        self._mouse(QEvent.MouseMove, 8, 6, Qt.NoButton, Qt.LeftButton)
        self._mouse(QEvent.MouseButtonRelease, 8, 6)
        after = self.world_anchors(item.model)
        self.assertAlmostEqual(after[2][0], 8.0, places=6)
        self.assertAlmostEqual(after[2][1], 6.0, places=6)
        for (x, y), (bx, by) in zip(after[:2], before[:2]):
            self.assertAlmostEqual(x, bx, places=9)
            self.assertAlmostEqual(y, by, places=9)
        self.window.undo_stack.undo()
        for (x, y), (bx, by) in zip(self.world_anchors(item.model), before):
            self.assertAlmostEqual(x, bx, places=9)
            self.assertAlmostEqual(y, by, places=9)

    def test_ctrl_click_toggles_a_tangent_arc_and_alt_click_deletes(self):
        item = self._draw_straight()
        self._mouse(QEvent.MouseButtonPress, 6, 0, mods=Qt.ControlModifier)
        self._mouse(QEvent.MouseButtonRelease, 6, 0, mods=Qt.ControlModifier)
        segs = cpath.segments(*cpath.belt_local_path(item.model), False)
        self.assertEqual(segs[1].kind, "arc")
        self.assertAlmostEqual(segs[1].heading_at(0.0), segs[0].heading_at(1.0), places=9)
        self._mouse(QEvent.MouseButtonPress, 6, 0, mods=Qt.AltModifier)
        self._mouse(QEvent.MouseButtonRelease, 6, 0, mods=Qt.AltModifier)
        self.assertEqual(len(item.model.path), 2)

    # -- end to end ------------------------------------------------------------------

    def test_drawn_loop_saves_and_runs_in_the_simulator(self):
        self.tool.toolbar.corner_spin.setValue(1.0)
        self.tool.start()
        for x, y in ((0, 0), (8, 0), (8, 5), (0, 5), (0, 0)):
            self.click(x, y)
        self.assertTrue(self.window._save_config())
        config = load_config(self.path)
        config.tractionModel = "friction"
        surface = belt_footprint(config.belts[0])
        engine = PhysicsEngine(config)
        parcel = engine.add_parcel(4.0, 0.0, 0.4, 0.4)
        for _ in range(30):  # ~1.3 laps of the ~23 m loop at 1 m/s, checked every second
            engine.run_for(1.0)
            p = parcel.body.position
            self.assertLess(surface.distance(cpath.Polygon([(p.x, p.y), (p.x + 1e-6, p.y), (p.x, p.y + 1e-6)])),
                            1e-3, f"parcel left the loop at ({p.x:.2f}, {p.y:.2f})")
        self.assertAlmostEqual(parcel.body.velocity.length, 1.0, delta=0.02)


if __name__ == "__main__":
    unittest.main()
