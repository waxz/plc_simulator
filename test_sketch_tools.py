"""
test_sketch_tools.py - the CAD workflow for building belts:
sketch lines/arcs (or import a DXF layout) -> object-snap to exact points ->
connect lines/curves into a belt (chain -> path belt, closed area ->
polygon belt), plus drawing a polygon outline directly and editing its
corners. Core geometry is tested without Qt; the editor is driven with real
mouse/key events (Qt offscreen).
"""
import math
import os
import tempfile
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QKeyEvent, QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog  # noqa: E402

from cad import dxf_io, path as cpath  # noqa: E402
from cad.belts import default_flow_heading, outline_belt  # noqa: E402
from cad.checks import belt_footprint  # noqa: E402
from cad.dxf_io import DrawingShape  # noqa: E402
from cad.snap import DrawingPicker, SnapIndex, chain_from, polygon_vertices  # noqa: E402
from config_parser import SimulatorConfig, SketchEntity, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from layout_editor_cad import SketchItem  # noqa: E402
from physics import PhysicsEngine  # noqa: E402

_app = QApplication.instance() or QApplication([])

SKEWED_QUAD = [(0.0, 0.0), (4.0, 1.0), (3.5, 4.0), (0.5, 3.0)]  # like belt_1_2_5


def lines_around(points):
    return [DrawingShape([points[i], points[(i + 1) % len(points)]], [0.0, 0.0], False) for i in range(len(points))]


class CoreGeometryTest(unittest.TestCase):
    def test_three_point_arc(self):
        self.assertAlmostEqual(cpath.three_point_bulge((0, 0), (2, 0), (1, -1)), 1.0)    # CCW semicircle
        self.assertAlmostEqual(cpath.three_point_bulge((0, 0), (2, 0), (1, 1)), -1.0)    # CW semicircle
        self.assertEqual(cpath.three_point_bulge((0, 0), (2, 0), (1, 0)), 0.0)          # collinear

    def test_area_corners_survive_rounding_noise(self):
        # Where a neighbouring outline starts, the picked area can carry a
        # near-copy of a corner (40.6 vs 40.599999999999994): still 4 corners.
        from shapely.geometry import Polygon
        area = Polygon([(127.0, 39.4), (70.0, 39.4), (70.0, 40.6), (127.0, 40.6),
                        (127.0, 40.599999999999994), (126.99999999999999, 40.599999999999994)])
        self.assertEqual(polygon_vertices(area), [(127.0, 39.4), (70.0, 39.4), (70.0, 40.6), (127.0, 40.6)])
        belt = outline_belt("b", polygon_vertices(area))
        self.assertAlmostEqual(belt_footprint(belt).area, 57 * 1.2, places=6)

    def test_reversing_a_polyline_keeps_its_shape(self):
        pts, bul = [(0, 0), (4, 0), (6, 2)], [0.0, math.tan(math.pi / 8), 0.0]
        r_pts, r_bul = cpath.reverse_polyline(pts, bul)
        fwd = cpath.path_length(cpath.segments(pts, bul, False))
        back = cpath.path_length(cpath.segments(r_pts, r_bul, False))
        self.assertAlmostEqual(fwd, back)
        self.assertAlmostEqual(r_bul[0], -bul[1])

    def test_chain_joins_connected_lines_and_arcs_in_any_drawing_direction(self):
        arc = math.tan(math.pi / 8)
        shapes = [DrawingShape([(0, 0), (4, 0)], [0, 0], False),
                  DrawingShape([(6, 2), (4, 0)], [-arc, 0], False),     # drawn backwards
                  DrawingShape([(6, 6), (6, 2)], [0, 0], False),
                  DrawingShape([(20, 20), (21, 21)], [0, 0], False)]    # unrelated
        pts, bul, closed = chain_from(shapes, shapes[0])
        self.assertFalse(closed)
        self.assertEqual(len(pts), 4)
        self.assertAlmostEqual(cpath.path_length(cpath.segments(pts, bul, False)), 4 + math.pi + 4, places=9)

    def test_chain_stops_at_a_branch_and_closes_a_loop(self):
        branch = [DrawingShape([(0, 0), (1, 0)], [0, 0], False), DrawingShape([(1, 0), (2, 0)], [0, 0], False),
                  DrawingShape([(1, 0), (1, 1)], [0, 0], False)]
        pts, _, _ = chain_from(branch, branch[0])
        self.assertEqual(pts, [(0, 0), (1, 0)])
        loop = lines_around(SKEWED_QUAD)
        pts, _, closed = chain_from(loop, loop[2])
        self.assertTrue(closed)
        self.assertEqual(len(pts), 4)

    def test_snap_priority_endpoint_center_midpoint_nearest(self):
        index = SnapIndex.from_shapes(lines_around(SKEWED_QUAD) + [DrawingShape([(10, 0), (12, 2)], [math.tan(math.pi / 8), 0], False)])
        self.assertEqual(index.query(3.9, 1.1, 0.2).kind, "endpoint")
        self.assertEqual((index.query(3.9, 1.1, 0.2).x, index.query(3.9, 1.1, 0.2).y), (4.0, 1.0))
        self.assertEqual(index.query(10.05, 2.0, 0.2).kind, "center")
        self.assertEqual(index.query(2.0, 0.6, 0.2).kind, "midpoint")
        self.assertEqual(index.query(1.0, 0.3, 0.2).kind, "nearest")
        self.assertIsNone(index.query(30, 30, 0.2))

    def test_region_from_separate_lines(self):
        region = DrawingPicker(lines_around(SKEWED_QUAD)).pick_region(2, 2)
        self.assertAlmostEqual(region.area, 10.5)
        self.assertEqual(sorted(polygon_vertices(region)), sorted(SKEWED_QUAD))

    def test_outline_belt(self):
        belt = outline_belt("b", SKEWED_QUAD, flow_heading=30)
        self.assertEqual(belt.shape, "quadrilateral")
        self.assertEqual(belt.directionAngle, 30)
        world = sorted((belt.x + v["x"], belt.y + v["y"]) for v in belt.vertexs)
        self.assertEqual(world, sorted(SKEWED_QUAD))
        self.assertAlmostEqual(default_flow_heading([(0, 0), (10, 0), (10, 2), (0, 2)]), 0.0)
        with self.assertRaises(ValueError):
            outline_belt("b", [(0, 0), (1, 1), (2, 2)])          # no area
        with self.assertRaises(ValueError):
            outline_belt("b", [(0, 0), (2, 2), (2, 0), (0, 2)])  # crosses itself

    def test_parcel_on_an_outline_belt_moves_along_its_flow_direction(self):
        belt = outline_belt("b", [(-5, -1), (5, -1), (5, 1), (-5, 1)], flow_heading=0)
        e = PhysicsEngine(SimulatorConfig(belts=[belt], tractionModel="friction"))
        p = e.add_parcel(-3, 0, 0.4, 0.4)
        e.run_for(1.0)
        self.assertAlmostEqual(p.body.velocity.x, 1.0, delta=0.01)
        self.assertAlmostEqual(p.body.velocity.y, 0.0, delta=0.01)

    def test_dxf_keeps_exact_lines_arcs_circles_and_bulges(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "s.dxf")
            doc = ezdxf.new()
            doc.units = ezdxf.units.MM
            m = doc.modelspace()
            m.add_line((0, 0), (4000, 0))
            m.add_arc((0, 0), 1000, 0, 90)
            m.add_circle((0, 0), 500)
            m.add_lwpolyline([(0, 0, 0), (2000, 0, 1), (2000, 2000, 0)], format="xyb")
            doc.saveas(path)
            shapes = dxf_io.read_background(path).shapes
            self.assertEqual([len(s.points) for s in shapes], [2, 2, 2, 3])
            self.assertAlmostEqual(shapes[1].bulges[0], math.tan(math.pi / 8))
            self.assertEqual(shapes[2].bulges, [1.0, 1.0])
            self.assertTrue(shapes[2].closed)
            self.assertEqual(shapes[3].bulges, [0.0, 1.0, 0.0])

    def test_sketch_is_exported_to_its_own_dxf_layer(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "o.dxf")
            config = SimulatorConfig(sketch=[SketchEntity(points=[(0, 0), (2, 0)], bulges=[1.0, 0.0])])
            dxf_io.export_layout(config, out)
            line = ezdxf.readfile(out).modelspace().query('LWPOLYLINE[layer=="SKETCH"]').first
            self.assertEqual([round(b, 6) for *_xy, b in line.get_points("xyb")], [1.0, 0.0])


class CenterArcTest(unittest.TestCase):
    def test_center_start_end(self):
        start, end, bulge = cpath.center_arc((0, 0), (2, 0), (0, 5))          # CCW by default
        self.assertAlmostEqual(end[0], 0.0)
        self.assertAlmostEqual(end[1], 2.0)                                   # on the radius, toward (0, 5)
        self.assertAlmostEqual(bulge, math.tan(math.pi / 8))                  # a quarter turn
        _, end, bulge = cpath.center_arc((0, 0), (2, 0), (0, 5), sweep=-3 * math.pi / 2)
        self.assertAlmostEqual(end[1], 2.0)
        self.assertLess(bulge, 0)                                             # the long way, clockwise
        self.assertIsNone(cpath.center_arc((0, 0), (0, 0), (1, 1)))           # no radius
        self.assertIsNone(cpath.center_arc((0, 0), (2, 0), (5, 0), sweep=0.0))


class CurvedBeltThroughTest(unittest.TestCase):
    def test_ccw_and_cw_quarter_arcs(self):
        from path_tool import curved_belt_through
        h = math.sqrt(0.5)
        ccw = curved_belt_through("b", (2, 0), (0, 2), (2 * h, 2 * h), 0.8)   # about (0, 0), R 2
        self.assertEqual((ccw.type, ccw.shape), ("curved", "arc"))
        self.assertAlmostEqual(ccw.x, 0.0)
        self.assertAlmostEqual(ccw.y, 0.0)
        self.assertAlmostEqual(ccw.radius, 2.0)
        self.assertAlmostEqual(ccw.beltWidth, 0.8)
        self.assertAlmostEqual(ccw.startAngle, 0.0)
        self.assertAlmostEqual(ccw.endAngle, 90.0)
        self.assertGreater(ccw.directionAngle, 0)                           # parcels run CCW
        cw = curved_belt_through("b", (0, 2), (2, 0), (2 * h, 2 * h), 0.8)    # same arc, other way
        self.assertAlmostEqual(cw.startAngle, 0.0)
        self.assertAlmostEqual(cw.endAngle, 90.0)
        self.assertLess(cw.directionAngle, 0)                               # parcels run CW

    def test_more_than_half_a_turn_and_bad_input(self):
        from path_tool import curved_belt_through
        big = curved_belt_through("b", (1, 0), (0, -1), (-1, 0), 0.5)       # 270 deg CCW, R 1
        self.assertAlmostEqual((big.endAngle - big.startAngle) % 360.0, 270.0)
        with self.assertRaises(ValueError):
            curved_belt_through("b", (0, 0), (4, 0), (2, 0), 0.5)           # collinear
        with self.assertRaises(ValueError):
            curved_belt_through("b", (1, 0), (-1, 0), (0, 1), 3.0)          # wider than the arc


class EditorWorkflowTest(unittest.TestCase):
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
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

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

    def belts(self):
        return [i.model for i in self.window.items_list]

    def sketch_line(self, *points):
        self.cad.sketch_line_tool.start()
        for x, y in points:
            self.click(x, y)
        self.key(Qt.Key_Return)

    # -- top-level Sketch tool bar ---------------------------------------------------

    def test_sketch_tool_bar_buttons_start_the_tools(self):
        from PySide6.QtTest import QTest
        from PySide6.QtWidgets import QToolBar
        bar = self.window.findChild(QToolBar, "sketchToolbar")
        self.assertTrue(bar.isVisible())
        labels = [a.iconText() for a in bar.actions() if a.text()]
        for name in ("Line", "Arc", "Rectangle", "Circle", "Polygon", "Clear",
                     "Belt from Sketch", "Belt Path", "Curved Belt", "Outline Belt"):
            self.assertIn(name, labels)
        arc = next(a for a in bar.actions() if a.iconText() == "Arc")
        self.assertEqual(arc.toolTip(), "Arc (Shift+S)")
        QTest.mouseClick(bar.widgetForAction(arc), Qt.LeftButton)
        self.assertTrue(self.cad.sketch_arc_tool.active)
        self.assertTrue(all(a.isVisible() for a in self.cad.draw_toolbar.arc_method_actions))   # Arc method shown
        line = next(a for a in bar.actions() if a.iconText() == "Line")
        QTest.mouseClick(bar.widgetForAction(line), Qt.LeftButton)
        self.assertTrue(self.cad.sketch_line_tool.active)
        self.assertFalse(self.cad.sketch_arc_tool.active)
        self.assertFalse(any(a.isVisible() for a in self.cad.draw_toolbar.arc_method_actions))

    def test_quick_access_tool_bars(self):
        from PySide6.QtTest import QTest
        from PySide6.QtWidgets import QToolBar
        file_bar = self.window.findChild(QToolBar, "fileToolbar")
        labels = [a.iconText() for a in file_bar.actions() if a.text()]
        self.assertEqual(labels, ["Open", "Save", "Undo", "Redo", "Import DXF", "Export DXF"])
        button = lambda bar, text: bar.widgetForAction(next(a for a in bar.actions() if a.iconText() == text))
        # Undo / Redo buttons: fixed labels, the tooltip names the step.
        self.sketch_line((0, 0), (4, 0))
        undo = button(file_bar, "Undo")
        self.assertEqual(undo.text(), "Undo")
        QTest.mouseClick(undo, Qt.LeftButton)
        self.assertEqual(self.window.config.sketch, [])
        QTest.mouseClick(button(file_bar, "Redo"), Qt.LeftButton)
        self.assertEqual(len(self.window.config.sketch), 1)
        QTest.mouseClick(button(file_bar, "Save"), Qt.LeftButton)
        self.assertTrue(self.window.undo_stack.isClean())
        self.assertEqual(len(load_config(self.path).sketch), 1)
        # Connections on the sketch row, Fit / Ports on the View bar.
        bar = self.window.findChild(QToolBar, "sketchToolbar")
        for name in ("Connect", "Fillet Belts", "Disconnect", "Solve", "Check"):
            self.assertIn(name, [a.iconText() for a in bar.actions()])
        bar = self.window.findChild(QToolBar, "viewToolbar")
        ports = next(a for a in bar.actions() if a.iconText() == "Ports")
        QTest.mouseClick(bar.widgetForAction(ports), Qt.LeftButton)
        self.assertTrue(ports.isChecked() and self.cad.ports.visible)
        before = self.view.transform().m11()
        fit = next(a for a in bar.actions() if a.iconText() == "Fit")
        self.assertEqual(fit.shortcut().toString(), "Z")
        fit.trigger()          # (at this test window's width + fallback font the last button sits in the overflow)
        self.assertNotAlmostEqual(self.view.transform().m11(), before)
        # Tool options + coordinate input along the bottom; nothing shown idle.
        draw = self.cad.draw_toolbar
        self.assertEqual(self.window.toolBarArea(draw), Qt.BottomToolBarArea)
        self.assertFalse(any(a.isVisible() for a in draw.width_actions + draw.pick_actions))
        self.cad.path_tool.start()
        self.assertTrue(all(a.isVisible() for a in draw.width_actions))
        self.key(Qt.Key_Escape)
        self.assertFalse(any(a.isVisible() for a in draw.width_actions))

    # -- center, start, end arcs ----------------------------------------------------

    def use_center_method(self):
        combo = self.cad.draw_toolbar.arc_method_combo
        combo.setCurrentIndex(combo.findData("center"))

    def sweep(self, *points):
        for x, y in points:
            self._mouse(QEvent.MouseMove, x, y, Qt.NoButton, Qt.NoButton)

    def test_sketch_arc_center_start_end_follows_the_sweep(self):
        for way, through, expect_ccw in (("ccw", (0.3, 2.0), True), ("cw", (0.3, -2.0), False)):
            self.cad.sketch_arc_tool.start()
            self.use_center_method()
            self.assertIn("center", self.window.statusBar().currentMessage())
            self.click(0, 0)                                  # center
            self.click(2, 0)                                  # start: radius 2
            self.sweep((1.5, 1.3 if expect_ccw else -1.3), through, (-1.5, 1.3 if expect_ccw else -1.3))
            self.click(-2, 0)                                 # end: half a turn, the way swept
            entity = self.window.config.sketch[-1]
            seg = cpath.bulge_to_segment(entity.points[0], entity.points[1], entity.bulges[0])
            self.assertEqual(seg.kind, "arc", way)
            self.assertAlmostEqual(seg.radius, 2.0, places=2)
            self.assertAlmostEqual(abs(math.degrees(seg.sweep)), 180.0, places=0)
            self.assertEqual(seg.sweep > 0, expect_ccw, way)
            self.assertAlmostEqual(seg.center[0], 0.0, places=2)

    def test_more_than_half_a_turn_and_a_typed_end(self):
        self.cad.sketch_arc_tool.start()
        self.use_center_method()
        self.click(0, 0)
        self.click(2, 0)
        self.sweep((1.4, 1.4), (0, 2), (-1.4, 1.4), (-2, 0), (-1.4, -1.4))
        self.click(0, -2)                                     # three quarters, CCW
        entity = self.window.config.sketch[-1]
        self.assertAlmostEqual(math.degrees(4 * math.atan(entity.bulges[0])), 270.0, places=0)
        # A typed end point (no sweep) goes counter-clockwise.
        self.cad.sketch_arc_tool.start()
        self.cad.sketch_arc_tool.typed_input("0,0")
        self.cad.sketch_arc_tool.typed_input("2,0")
        self.cad.sketch_arc_tool.typed_input("0,-5")          # direction only: ends at (0, -2)
        entity = self.window.config.sketch[-1]
        self.assertAlmostEqual(entity.points[1][1], -2.0)
        self.assertAlmostEqual(math.degrees(4 * math.atan(entity.bulges[0])), 270.0)

    def test_switching_method_mid_arc_starts_again(self):
        self.cad.sketch_arc_tool.start()
        self.click(0, 0)
        self.use_center_method()
        self.assertEqual(self.cad.sketch_arc_tool.anchors, [])
        self.assertTrue(self.cad.sketch_arc_tool.active)

    def test_curved_belt_by_center_start_end(self):
        self.cad.draw_toolbar.width_spin.setValue(0.8)
        self.cad.curved_tool.start()
        self.use_center_method()
        self.click(3, 2)                                      # center
        self.click(3, 4)                                      # start, R 2
        self.sweep((4.4, 3.4), (5, 2))
        self.click(5, 2)                                      # a quarter turn clockwise
        belt = self.belts()[0]
        self.assertEqual(belt.type, "curved")
        self.assertAlmostEqual(belt.x, 3.0, places=2)
        self.assertAlmostEqual(belt.y, 2.0, places=2)
        self.assertAlmostEqual(belt.radius, 2.0, places=2)
        self.assertAlmostEqual((belt.endAngle - belt.startAngle) % 360.0, 90.0, places=0)
        self.assertLess(belt.directionAngle, 0)               # parcels run the way it was swept

    # -- draw a curved belt directly ------------------------------------------------

    def test_draw_curved_belt_from_the_menu_with_clicks(self):
        from PySide6.QtGui import QAction
        action = next(a for a in self.window.findChildren(QAction) if a.text() == "Draw Curved Belt")
        self.assertEqual(action.shortcut().toString(), "U")
        self.cad.draw_toolbar.width_spin.setValue(0.6)
        action.trigger()
        self.click(5, 2)                               # start
        self.click(3, 4)                               # end
        self.cad.curved_tool.typed_input(f"{3 + 2 * math.cos(math.pi / 4)!r},{2 + 2 * math.sin(math.pi / 4)!r}")
        belts = self.belts()
        self.assertEqual(len(belts), 1)
        belt = belts[0]
        self.assertEqual(belt.type, "curved")
        self.assertAlmostEqual(belt.x, 3.0, places=2)    # free clicks are pixel-accurate
        self.assertAlmostEqual(belt.y, 2.0, places=2)
        self.assertAlmostEqual(belt.radius, 2.0, places=2)
        self.assertAlmostEqual(belt.beltWidth, 0.6)
        self.assertAlmostEqual((belt.endAngle - belt.startAngle) % 360.0, 90.0, places=1)
        self.assertGreater(belt.directionAngle, 0)
        self.assertTrue(self.window.items_list[0].is_curved())
        # Parcels on it are carried round the curve by physics.
        engine = PhysicsEngine(self.window._build_export_config())
        self.assertEqual(len(engine.config.belts), 1)
        self.window.undo_stack.undo()
        self.assertEqual(self.belts(), [])

    def test_collinear_third_point_keeps_drawing(self):
        self.cad.curved_tool.start()
        self.click(0, 0)
        self.click(4, 0)
        self.cad.curved_tool.typed_input("2,0")       # on the chord: no arc
        self.assertEqual(self.belts(), [])
        self.assertTrue(self.cad.curved_tool.active)
        self.assertIn("in a line", self.window.statusBar().currentMessage())
        self.cad.curved_tool.typed_input("2,-2")      # now a CW semicircle through (2, -2)
        belt = self.belts()[0]
        self.assertEqual(belt.type, "curved")
        self.assertAlmostEqual(belt.radius, 2.0, places=2)
        self.assertAlmostEqual((belt.endAngle - belt.startAngle) % 360.0, 180.0, places=1)

    # -- sketch, then connect into a belt --------------------------------------------

    def test_sketch_line_and_arc_then_connect_them_into_one_path_belt(self):
        self.sketch_line((0, 0), (4, 0))
        self.cad.sketch_arc_tool.start()
        self.click(4, 0)                                   # start: snaps to the line's end
        self.click(6, 2)                                   # end
        # The point the arc passes through is typed, not clicked: a free click is
        # only pixel-accurate, while endpoints snap exactly. Middle of the
        # tangent quarter arc (center (4, 2), radius 2):
        self.cad.sketch_arc_tool.typed_input(f"{4 + 2 * math.sin(math.pi / 4)!r},{2 - 2 * math.cos(math.pi / 4)!r}")
        self.sketch_line((6, 2), (6, 6))
        self.assertEqual(len(self.window.config.sketch), 3)
        self.assertEqual(self.belts(), [], "sketching must not create belts")

        self.cad.draw_toolbar.width_spin.setValue(0.8)
        self.cad.pick_tool.start()
        self.click(6, 4)                                   # on the last line
        self.key(Qt.Key_Escape)
        (belt,) = self.belts()
        self.assertEqual(belt.type, "path")
        self.assertEqual(belt.beltWidth, 0.8)
        pts, bul = cpath.belt_world_path(belt)
        self.assertAlmostEqual(cpath.path_length(cpath.segments(pts, bul, False)), 4 + math.pi + 4, places=6)

    def test_closed_area_of_separately_sketched_lines_becomes_a_polygon_belt(self):
        for a, b in zip(SKEWED_QUAD, SKEWED_QUAD[1:] + SKEWED_QUAD[:1]):
            self.sketch_line(a, b)                          # four separate lines, endpoints snapped
        self.cad.pick_tool.start()
        self.click(2.0, 2.0)                               # inside
        (belt,) = self.belts()
        self.assertEqual(belt.shape, "quadrilateral")
        world = sorted((round(belt.x + v["x"], 9), round(belt.y + v["y"], 9)) for v in belt.vertexs)
        self.assertEqual(world, sorted(SKEWED_QUAD))

    def test_deleting_a_sketch_entity_and_undo(self):
        self.sketch_line((0, 0), (4, 0))
        item = next(i for i in self.window.scene.items() if isinstance(i, SketchItem))
        item.setSelected(True)
        self.key(Qt.Key_Delete)
        self.assertEqual(self.window.config.sketch, [])
        self.window.undo_stack.undo()
        self.assertEqual(len(self.window.config.sketch), 1)

    def test_sketch_survives_save_and_reopen(self):
        self.sketch_line((0, 0), (4, 0), (4, 3))
        self.assertTrue(self.window._save_config())
        reopened = MainWindow(config=load_config(self.path), config_path=self.path)
        self.addCleanup(reopened.deleteLater)
        self.assertEqual(reopened.config.sketch[0].points, [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0)])
        self.assertEqual(len(reopened.cad.sketch_items), 1)
        reopened.undo_stack.setClean()

    # -- outline directly ---------------------------------------------------------------

    def test_draw_outline_with_typed_length_and_angle(self):
        self.cad.outline_tool.start()
        self.cad.outline_tool.typed_input("0,0")
        self.cad.outline_tool.typed_input("4<14.036243467926479")   # first edge -> flow direction
        self.cad.outline_tool.typed_input("3.5,4")
        self.cad.outline_tool.typed_input("0.5,3")
        self.key(Qt.Key_Return)
        (belt,) = self.belts()
        self.assertEqual(belt.shape, "quadrilateral")
        self.assertAlmostEqual(belt.directionAngle, 14.036243467926479)
        self.assertEqual(len(belt.vertexs), 4)

    def test_outline_corners_snap_to_an_imported_dxf_layout(self):
        plan = os.path.join(self.tmp.name, "plan.dxf")
        doc = ezdxf.new()
        doc.units = ezdxf.units.MM
        for a, b in zip(SKEWED_QUAD, SKEWED_QUAD[1:] + SKEWED_QUAD[:1]):
            doc.modelspace().add_line((a[0] * 1000, a[1] * 1000), (b[0] * 1000, b[1] * 1000))
        doc.modelspace().add_lwpolyline([(6000, 0, 0), (10000, 0, math.tan(math.pi / 8)), (12000, 2000, 0)],
                                        format="xyb")
        doc.saveas(plan)
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(plan, "")), \
                mock.patch("layout_editor_cad.BackgroundImportDialog.exec", return_value=QDialog.Accepted):
            self.cad.import_background()

        # Click a little off each corner: OSNAP puts the corner exactly on the drawing.
        self.cad.outline_tool.start()
        for x, y in SKEWED_QUAD:
            self.click(x + 0.07, y - 0.06)
        self.click(0.05, 0.05)  # first corner again -> finish
        belt = self.belts()[0]
        world = sorted((round(belt.x + v["x"], 9), round(belt.y + v["y"], 9)) for v in belt.vertexs)
        self.assertEqual(world, sorted(SKEWED_QUAD))

        # And the drawing's curved polyline becomes a path belt with the exact arc.
        self.cad.pick_tool.start()
        self.click(8.0, 0.0)
        path_belt = self.belts()[1]
        self.assertEqual(path_belt.type, "path")
        self.assertAlmostEqual(path_belt.path[1].bulge, math.tan(math.pi / 8))

    # -- corner handles on polygon belts ---------------------------------------------------

    def test_drag_a_polygon_corner_and_undo(self):
        self.cad.outline_tool.start()
        for x, y in [(0, 0), (4, 0), (4, 2), (0, 2), (0, 0)]:
            self.click(x, y)
        belt = self.belts()[0]
        item = self.window.items_list[0]
        self.window.scene.clearSelection()
        item.setSelected(True)
        _app.processEvents()
        self.assertEqual(len(self.cad.grips.grips), 4)
        before = [dict(v) for v in belt.vertexs]
        self._mouse(QEvent.MouseButtonPress, 4, 2)
        self._mouse(QEvent.MouseMove, 5, 3, Qt.NoButton, Qt.LeftButton)
        self._mouse(QEvent.MouseButtonRelease, 5, 3)
        corners = sorted((belt.x + v["x"], belt.y + v["y"]) for v in belt.vertexs)
        self.assertIn((5.0, 3.0), corners)
        self.window.undo_stack.undo()
        self.assertEqual(belt.vertexs, before)


if __name__ == "__main__":
    unittest.main()
