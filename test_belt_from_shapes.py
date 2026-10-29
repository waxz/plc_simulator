"""
test_belt_from_shapes.py - Belt from Sketch/Drawing with a chosen belt type
(auto / linear / curved / path) and direction, basic sketch shapes
(rectangle, circle, polygon) as the way to build loops, and picking shapes
that are split into several parts: line ends that miss each other by a few
millimetres, lines drawn twice, an area split by a line, a chain through a
branch. Core geometry without Qt; the editor driven with real events.
"""
import math
import os
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QKeyEvent, QMouseEvent  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

from cad import path as cpath  # noqa: E402
from cad.belts import arc_sector, belt_from_target, flow_arrows  # noqa: E402
from cad.dxf_io import DrawingShape  # noqa: E402
from cad.snap import DrawingPicker, chain_from, heal_lines, join_shapes  # noqa: E402
from config_parser import BackgroundDrawing, SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from physics import PhysicsEngine  # noqa: E402
from surface import belt_contains, surface_velocity  # noqa: E402

_app = QApplication.instance() or QApplication([])
Q = math.tan(math.pi / 8)  # bulge of a quarter arc


def line(a, b):
    return DrawingShape([a, b], [0.0, 0.0], False)


def arc(a, b, bulge):
    return DrawingShape([a, b], [bulge, 0.0], False)


def ring_sector():
    """Quarter ring, center (0, 0), radii 1..3, drawn as 2 arcs + 2 lines."""
    return [arc((3, 0), (0, 3), Q), arc((0, 1), (1, 0), -Q), line((1, 0), (3, 0)), line((0, 3), (0, 1))]


def rectangle_with_gaps(x0, y0, x1, y1, gap):
    """Four separate lines whose ends miss each other by `gap`."""
    return [line((x0 + gap, y0), (x1 - gap, y0)), line((x1, y0 + gap), (x1, y1 - gap)),
            line((x1 - gap, y1), (x0 + gap, y1)), line((x0, y1 - gap), (x0, y0 + gap))]


class BeltTypeTest(unittest.TestCase):
    def test_straight_line_is_linear_with_any_direction(self):
        target = ("line", ([(0, 0), (2, 0), (5, 0)], [0, 0, 0], False))
        belt = belt_from_target("b", target, width=0.6)
        self.assertEqual((belt.type, belt.shape), ("linear", "rectangle"))
        self.assertEqual((belt.x, belt.y, belt.length, belt.beltWidth, belt.rotation), (2.5, 0, 5, 0.6, 0))
        self.assertEqual(belt.directionAngle, 0)
        self.assertEqual(belt_from_target("b", target, direction="reverse").directionAngle, 180)
        self.assertEqual(belt_from_target("b", target, direction="across+").directionAngle, 90)
        self.assertEqual(belt_from_target("b", target, direction="across-").directionAngle, -90)
        vx, vy, _ = surface_velocity(belt_from_target("b", target, direction="reverse"), 2.5, 0)
        self.assertAlmostEqual(vx, -1.0)

    def test_arc_is_curved_following_its_drawn_direction(self):
        ccw = belt_from_target("b", ("line", ([(2, 0), (0, 2)], [Q, 0], False)), width=0.5)
        self.assertEqual((ccw.type, ccw.shape), ("curved", "arc"))
        self.assertAlmostEqual(ccw.radius, 2)
        self.assertAlmostEqual(ccw.startAngle, 0)
        self.assertAlmostEqual(ccw.endAngle, 90)
        self.assertGreater(ccw.directionAngle, 0)  # CCW, as drawn
        cw = belt_from_target("b", ("line", ([(0, 2), (2, 0)], [-Q, 0], False)), width=0.5)
        self.assertAlmostEqual(cw.startAngle, 0)    # same arc...
        self.assertLess(cw.directionAngle, 0)       # ...drawn the other way: CW
        flipped = belt_from_target("b", ("line", ([(2, 0), (0, 2)], [Q, 0], False)), direction="reverse")
        self.assertLess(flipped.directionAngle, 0)
        with self.assertRaises(ValueError):
            belt_from_target("b", ("line", ([(2, 0), (0, 2)], [Q, 0], False)), direction="across+")

    def test_circle_line_is_a_full_curved_ring(self):
        belt = belt_from_target("b", ("line", ([(1, 0), (-1, 0)], [1.0, 1.0], True)), width=0.4)
        self.assertEqual(belt.type, "curved")
        self.assertEqual((belt.startAngle, belt.endAngle), (0, 360))
        self.assertTrue(belt_contains(belt, 0, -1))

    def test_mixed_chain_is_a_path_and_forced_types_explain_why_not(self):
        target = ("line", ([(0, 0), (4, 0), (6, 2)], [0, Q, 0], False))
        belt = belt_from_target("b", target)
        self.assertEqual(belt.type, "path")
        self.assertEqual(belt_from_target("b", target, direction="reverse").directionAngle, 180)
        with self.assertRaisesRegex(ValueError, "straight"):
            belt_from_target("b", target, kind="linear")
        with self.assertRaisesRegex(ValueError, "arc"):
            belt_from_target("b", target, kind="curved")
        straight = ("line", ([(0, 0), (4, 0)], [0, 0], False))
        self.assertEqual(belt_from_target("b", straight, kind="path").type, "path")

    def test_ring_sector_area_is_a_curved_belt_filling_it(self):
        shapes = ring_sector()
        region = DrawingPicker(shapes).pick_region(1.5, 1.5)
        self.assertIsNotNone(region, "arc ends differ from line ends in the last digit; still one area")
        center, r_in, r_out, start, sweep = arc_sector(region, shapes)
        self.assertAlmostEqual(r_in, 1)
        self.assertAlmostEqual(r_out, 3)
        self.assertAlmostEqual(sweep, 90, places=3)
        belt = belt_from_target("b", ("region", region), shapes=shapes)
        self.assertEqual(belt.type, "curved")
        self.assertAlmostEqual(belt.radius, 2)
        self.assertAlmostEqual(belt.beltWidth, 2)
        self.assertAlmostEqual(belt.startAngle, 0, places=3)
        self.assertAlmostEqual(belt.endAngle, 90, places=3)
        self.assertGreater(belt.directionAngle, 0)                          # forward = CCW
        cw = belt_from_target("b", ("region", region), shapes=shapes, direction="reverse")
        self.assertLess(cw.directionAngle, 0)
        # Parcels on it turn about the center.
        e = PhysicsEngine(SimulatorConfig(belts=[belt]))
        p = e.add_parcel(2.0, 0.6, 0.3, 0.3)
        e.run_for(0.5)
        self.assertGreater(p.body.position.y, 0.6)
        # Same area as Linear: a polygon belt; as Path: the middle arc.
        self.assertEqual(belt_from_target("b", ("region", region), shapes=shapes, kind="linear").shape,
                         "quadrilateral")
        path = belt_from_target("b", ("region", region), shapes=shapes, kind="path")
        self.assertEqual(path.type, "path")
        self.assertAlmostEqual(path.beltWidth, 2)
        pts, bul = cpath.belt_world_path(path)
        self.assertAlmostEqual(cpath.path_length(cpath.segments(pts, bul, False)), math.pi, places=6)

    def test_disc_and_ring_from_circles(self):
        outer = DrawingShape([(2, 0), (-2, 0)], [1.0, 1.0], True)
        disc = DrawingPicker([outer]).pick_region(0.2, 0.2)
        belt = belt_from_target("b", ("region", disc), shapes=[outer])
        self.assertEqual((belt.type, belt.radius, belt.beltWidth, belt.startAngle, belt.endAngle),
                         ("curved", 1.0, 2.0, 0.0, 360.0))
        inner = DrawingShape([(1, 0), (-1, 0)], [1.0, 1.0], True)
        ring = DrawingPicker([outer, inner]).pick_region(1.5, 0.0)
        belt = belt_from_target("b", ("region", ring), shapes=[outer, inner])
        self.assertEqual((belt.type, belt.radius, belt.beltWidth), ("curved", 1.5, 1.0))
        self.assertTrue(belt_contains(belt, 0, -1.5))
        self.assertFalse(belt_contains(belt, 0, 0.5))

    def test_polygon_area_directions(self):
        shapes = rectangle_with_gaps(0, 0, 10, 2, 0.0)
        region = DrawingPicker(shapes).pick_region(5, 1)
        heading = {d: belt_from_target("b", ("region", region), shapes=shapes, direction=d).directionAngle
                   for d in ("forward", "reverse", "across+", "across-")}
        self.assertEqual(heading, {"forward": 0, "reverse": 180, "across+": 90, "across-": 270})
        with self.assertRaisesRegex(ValueError, "arc-shaped"):
            belt_from_target("b", ("region", region), shapes=shapes, kind="curved")

    def test_flow_arrows_point_the_way_parcels_go(self):
        (x, y, h), = flow_arrows(belt_from_target("b", ("line", ([(0, 0), (4, 0)], [0, 0], False)),
                                                  direction="reverse"))
        self.assertEqual((x, y, h % 360), (2, 0, 180))
        (x, y, h), = flow_arrows(belt_from_target("b", ("line", ([(2, 0), (0, 2)], [Q, 0], False))))
        self.assertAlmostEqual(h % 360, 135)  # CCW at 45 deg: heading 135


class SplitShapesTest(unittest.TestCase):
    """Bug: belts from DXF shapes failed when the shape was split into parts."""

    def test_area_with_gapped_corners_is_found_with_a_join_gap(self):
        shapes = rectangle_with_gaps(0, 0, 4, 2, 0.005)  # 5 mm short at every corner
        self.assertIsNone(DrawingPicker(shapes).pick_region(2, 1))
        region = DrawingPicker(shapes, join_gap=0.02).pick_region(2, 1)
        self.assertIsNotNone(region)
        self.assertAlmostEqual(region.area, 8.0, delta=0.05)

    def test_line_stopping_short_of_another_closes_the_area(self):
        # The right side stops 1 cm below the top; the top overshoots the left.
        shapes = [line((0, 0), (4, 0)), line((4, 0), (4, 1.99)), line((-0.3, 2), (4, 2)), line((0, 2), (0, 0))]
        self.assertIsNone(DrawingPicker(shapes).pick_region(2, 1))
        self.assertAlmostEqual(DrawingPicker(shapes, join_gap=0.02).pick_region(2, 1).area, 8.0, delta=0.05)

    def test_changing_the_join_gap_finds_the_areas_again(self):
        picker = DrawingPicker(rectangle_with_gaps(0, 0, 4, 2, 0.005))
        self.assertIsNone(picker.pick_region(2, 1))
        picker.set_join_gap(0.02)
        self.assertIsNotNone(picker.pick_region(2, 1))

    def test_chain_joins_across_a_gap_and_past_a_duplicate(self):
        shapes = [line((0, 0), (2, 0)), line((2.004, 0), (4, 0)), line((4, 0), (4, 3)), line((4, 3), (4, 0))]
        pts, _, closed = chain_from(shapes, shapes[0], tolerance=0.01)
        self.assertFalse(closed)
        self.assertEqual(pts, [(0, 0), (2, 0), (4, 0), (4, 3)])  # gap closed, the twice-drawn line once
        pts, _, _ = chain_from(shapes, shapes[0])                 # default 1 mm: stops at the gap
        self.assertEqual(pts, [(0, 0), (2, 0)])
        picker = DrawingPicker(shapes, join_gap=0.01)
        self.assertEqual(picker.chain(shapes[0])[0], [(0, 0), (2, 0), (4, 0), (4, 3)])

    def test_join_selected_pieces_through_a_branch(self):
        a, b, c = line((0, 0), (1, 0)), line((1, 0), (2, 0)), line((1, 1), (1, 0))
        self.assertEqual(chain_from([a, b, c], a)[0], [(0, 0), (1, 0)])  # a branch stops the automatic chain
        pts, _, closed = join_shapes([c, a])                            # the user says which way
        self.assertEqual(sorted([pts[0], pts[-1]]), [(0, 0), (1, 1)])
        with self.assertRaisesRegex(ValueError, "don't join"):
            join_shapes([a, line((5, 5), (6, 5))])

    def test_heal_lines_merges_ends(self):
        healed = heal_lines([[(0, 0), (1, 0)], [(1.003, 0.001), (1, 1)]], 0.01)
        self.assertEqual(healed[0][-1], healed[1][0])


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

    def _mouse(self, kind, x, y, button=Qt.LeftButton, buttons=None, mods=Qt.NoModifier):
        vp = self.view.viewport()
        pos = QPointF(self.view.mapFromScene(to_scene_pos(x, y)))
        if buttons is None:
            buttons = Qt.NoButton if kind == QEvent.MouseButtonRelease else button
        _app.sendEvent(vp, QMouseEvent(kind, pos, vp.mapToGlobal(pos), button, buttons, mods))

    def click(self, x, y, mods=Qt.NoModifier):
        self._mouse(QEvent.MouseMove, x, y, Qt.NoButton, Qt.NoButton, mods)
        self._mouse(QEvent.MouseButtonPress, x, y, mods=mods)
        self._mouse(QEvent.MouseButtonRelease, x, y, mods=mods)

    def key(self, key):
        _app.sendEvent(self.view, QKeyEvent(QEvent.KeyPress, key, Qt.NoModifier, ""))

    def belts(self):
        return [i.model for i in self.window.items_list]

    def set_type(self, kind, direction="forward"):
        self.bar.type_combo.setCurrentIndex(self.bar.type_combo.findData(kind))
        self.bar.direction_combo.setCurrentIndex(self.bar.direction_combo.findData(direction))

    def sketch(self, tool, *typed):
        tool.start()
        for text in typed:
            tool.typed_input(text)

    def test_insert_belt_shape_menu_is_gone_and_basic_shapes_are_there(self):
        texts = [a.text() for m in self.window.menuBar().findChildren(type(self.window.menuBar().addMenu("x")))
                 for a in m.actions()]
        self.assertNotIn("Insert Belt Shape...", texts)
        for name in ("Rectangle", "Circle", "Polygon"):            # Tools > Sketch
            self.assertIn(name, texts)

    def test_circle_sketch_becomes_a_curved_disc_or_a_linear_polygon(self):
        self.sketch(self.cad.sketch_circle_tool, "2,2", "4,2")
        (entity,) = self.window.config.sketch
        self.assertTrue(entity.closed)
        self.assertEqual(self.belts(), [])
        self.cad.pick_tool.start()
        self.click(2.3, 2.4)                                  # inside: Auto -> curved disc
        belt = self.belts()[-1]
        self.assertEqual((belt.type, belt.radius, belt.beltWidth), ("curved", 1.0, 2.0))
        self.set_type("linear", "across+")
        self.click(2.3, 2.4)
        belt = self.belts()[-1]
        self.assertEqual((belt.type, belt.shape), ("linear", "quadrilateral"))

    def test_rectangle_and_polygon_sketches(self):
        self.bar.corner_spin.setValue(0.0)
        self.sketch(self.cad.sketch_rect_tool, "0,0", "@6,2")
        self.bar.sides_spin.setValue(5)
        self.sketch(self.cad.sketch_polygon_tool, "10,0", "12,0")
        rect, pentagon = self.window.config.sketch
        self.assertEqual(len(rect.points), 4)
        self.assertEqual(len(pentagon.points), 5)
        self.cad.pick_tool.start()
        self.set_type("linear", "reverse")
        self.click(3, 1)
        belt = self.belts()[-1]
        self.assertEqual(belt.shape, "quadrilateral")
        self.assertEqual(belt.directionAngle, 180)

    def test_reverse_on_a_straight_sketch_line(self):
        self.cad.sketch_line_tool.start()
        self.click(0, 0)
        self.click(5, 0)
        self.key(Qt.Key_Return)
        self.cad.pick_tool.start()
        self.set_type("auto", "reverse")
        self.click(2.5, 0)
        belt = self.belts()[-1]
        self.assertEqual((belt.type, belt.length, belt.directionAngle), ("linear", 5.0, 180))

    def roller_action(self):
        return next(a for m in self.window.menuBar().findChildren(type(self.window.menuBar().addMenu("x")))
                    for a in m.actions() if a.text() == "Roller from Sketch/Drawing")

    def test_roller_from_a_line_and_from_a_circle(self):
        action = self.roller_action()
        self.assertEqual(action.shortcut().toString(), "Ctrl+Shift+B")
        self.cad.sketch_line_tool.start()
        self.click(0, 0)
        self.click(5, 0)
        self.key(Qt.Key_Return)
        self.sketch(self.cad.sketch_circle_tool, "10,2", "12,2")
        action.trigger()                                      # the tool, set to rollers
        self.assertEqual(self.bar.surface, "roller")
        self.set_type("auto")
        self.click(2.5, 0)
        roller = self.belts()[-1]
        self.assertEqual((roller.type, roller.surface, roller.length), ("linear", "roller", 5.0))
        self.assertTrue(roller.id.startswith("roller"))
        self.assertEqual((roller.rollerPitch, roller.speed), (0.125, 0.3))
        self.assertIn("roller conveyor", self.window.statusBar().currentMessage())
        self.click(10.3, 2.4)                                 # inside the circle: a curved roller conveyor
        curved = self.belts()[-1]
        self.assertEqual((curved.type, curved.surface), ("curved", "roller"))
        self.cad.pick_tool.start("belt")                      # B: belts again
        self.click(2.5, 0)
        self.assertEqual(self.belts()[-1].surface, "belt")

    def test_a_bent_chain_is_no_roller_conveyor(self):
        self.cad.sketch_line_tool.start()
        for x, y in ((0, 0), (4, 0), (4, 3)):
            self.click(x, y)
        self.key(Qt.Key_Return)
        self.roller_action().trigger()
        self.set_type("auto")
        self.click(2, 0)                                      # Auto would make a path belt
        self.assertEqual(self.belts(), [])
        self.assertIn("straight or curved", self.window.statusBar().currentMessage())

    def test_wrong_type_shows_why_and_creates_nothing(self):
        self.sketch(self.cad.sketch_rect_tool, "0,0", "4,2")
        self.cad.pick_tool.start()
        self.cad.pick_tool.picker.regions()  # areas are found on a worker thread; wait for them
        self.set_type("curved")
        self._mouse(QEvent.MouseMove, 2, 1, Qt.NoButton, Qt.NoButton)
        self.assertIn("arc-shaped", self.window.statusBar().currentMessage())
        self.click(2, 1)
        self.assertEqual(self.belts(), [])

    def test_preview_shows_the_belt_and_its_flow_arrow(self):
        self.sketch(self.cad.sketch_rect_tool, "0,0", "4,2")
        self.cad.pick_tool.start()
        self.cad.pick_tool.picker.regions()
        self._mouse(QEvent.MouseMove, 2, 1, Qt.NoButton, Qt.NoButton)
        self.assertGreaterEqual(len(self.cad.pick_tool._preview_items), 3)  # area, belt, arrow, label

    def test_shift_click_merges_an_area_split_in_two(self):
        # A 6 x 2 rectangle split by a line at x = 2: two areas.
        self.cad.sketch_line_tool.start()
        for p in ((0, 0), (6, 0), (6, 2), (0, 2)):
            self.click(*p)
        self.click(0, 0)                                      # close
        self.cad.sketch_line_tool.start()
        self.click(2, 0)
        self.click(2, 2)
        self.key(Qt.Key_Return)
        self.cad.pick_tool.start()
        self.click(1, 1, Qt.ShiftModifier)
        self.click(4, 1, Qt.ShiftModifier)
        self.assertEqual(self.belts(), [], "Shift+click only collects")
        self.key(Qt.Key_Return)
        (belt,) = self.belts()
        self.assertEqual(belt.shape, "quadrilateral")
        xs = [belt.x + v["x"] for v in belt.vertexs]
        self.assertAlmostEqual(min(xs), 0)
        self.assertAlmostEqual(max(xs), 6)

    def test_shift_click_joins_lines_through_a_branch(self):
        for a, b in (((0, 0), (3, 0)), ((3, 0), (6, 0)), ((3, 0), (3, 3))):
            self.cad.sketch_line_tool.start()
            self.click(*a)
            self.click(*b)
            self.key(Qt.Key_Return)
        self.cad.pick_tool.start()
        self.set_type("path")
        self.click(1.5, 0, Qt.ShiftModifier)
        self.click(3, 1.5, Qt.ShiftModifier)
        self.key(Qt.Key_Return)
        (belt,) = self.belts()
        pts, bul = cpath.belt_world_path(belt)
        self.assertAlmostEqual(cpath.path_length(cpath.segments(pts, bul, False)), 6.0)

    def test_preview_appears_once_the_areas_are_found(self):
        self.sketch(self.cad.sketch_rect_tool, "0,0", "4,2")
        tool = self.cad.pick_tool
        tool.start()
        self._mouse(QEvent.MouseMove, 2, 1, Qt.NoButton, Qt.NoButton)  # maybe before the worker is done
        tool.picker.regions()
        deadline = time.monotonic() + 3.0
        while tool.target is None and time.monotonic() < deadline:
            _app.processEvents()
            time.sleep(0.02)
        self.assertEqual(tool.target[0], "region")

    def test_esc_clears_the_selection_first(self):
        self.sketch(self.cad.sketch_rect_tool, "0,0", "4,2")
        self.cad.pick_tool.start()
        self.click(2, 1, Qt.ShiftModifier)
        self.assertEqual(len(self.cad.pick_tool.selection), 1)
        self.key(Qt.Key_Escape)
        self.assertEqual(self.cad.pick_tool.selection, [])
        self.assertTrue(self.cad.pick_tool.active)
        self.key(Qt.Key_Escape)
        self.assertFalse(self.cad.pick_tool.active)

    def test_dxf_rectangle_with_gapped_lines_becomes_a_belt(self):
        plan = os.path.join(self.tmp.name, "plan.dxf")
        doc = ezdxf.new()
        doc.units = ezdxf.units.MM
        for s in rectangle_with_gaps(0, 0, 4000, 2000, 6):   # 6 mm gaps at every corner
            doc.modelspace().add_line(s.points[0], s.points[1])
        doc.saveas(plan)
        self.cad.set_background(BackgroundDrawing(path=plan, scale=0.001))
        self.cad.pick_tool.start()
        self.bar.gap_spin.setValue(0.0)
        self.click(2, 1)
        self.assertEqual(self.belts(), [], "with no join gap the corners don't meet")
        self.bar.gap_spin.setValue(0.02)
        self.click(2, 1)
        (belt,) = self.belts()
        self.assertEqual(belt.shape, "quadrilateral")
        self.cad.set_background(None)


if __name__ == "__main__":
    unittest.main()
