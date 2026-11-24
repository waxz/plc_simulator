"""
test_designer_ax.py - the designer's merge belt, its AX geometry, moving a
selection to a position, and one name per component:

  * the Merge Belt library template: a belt whose end is cut at an angle,
    edited by length / width / end angle (its start edge stays put);
  * ax_geometry: belts as AX states them (start / end x, y, z, heading,
    arcs), eyes as a distance along the belt under them, the ST snippet;
  * the property panel's "AX / PLC geometry" rows and Copy ST;
  * Move Selection To: a group moved as one, one undo step;
  * a label that only repeats the id is not shown twice (id_utils).
"""
import math
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

import ax_geometry  # noqa: E402
from cad.belts import cut_of, make_cut_belt, mitre_corners, mitre_of, mitred_belt, set_mitre  # noqa: E402
from config_parser import Belt, Sensor, SimulatorConfig  # noqa: E402
from id_utils import display_name, label_repeats_id, own_label  # noqa: E402
from math_utils import get_belt_polygon  # noqa: E402

_app = QApplication.instance() or QApplication([])


class MergeBeltTest(unittest.TestCase):
    def test_its_end_lies_along_the_line(self):
        # travelling at -45 degrees, end cut at 45: the end edge is horizontal (the line's edge)
        b = mitred_belt("m", (1.5, 0.75), -45.0, math.hypot(0.5, 0.5), 0.5, 45.0)
        corners = get_belt_polygon(b)
        end = [c for c in corners if abs(c[1] - 0.25) < 1e-9]
        self.assertEqual(len(end), 2, corners)
        self.assertAlmostEqual(abs(end[0][0] - end[1][0]), 0.5 / math.sin(math.radians(45)))
        self.assertAlmostEqual(mitre_of(b), 45.0)

    def test_edit_keeps_the_start_edge(self):
        b = mitred_belt("m", (0.0, 0.0), 30.0, 1.0, 0.5, 60.0)
        start = ax_geometry.belt_path(SimulatorConfig(), b).start
        set_mitre(b, length=2.0, width=0.6, end_deg=120.0)
        self.assertAlmostEqual(mitre_of(b), 120.0)
        self.assertEqual(ax_geometry.belt_path(SimulatorConfig(), b).start, start)
        self.assertAlmostEqual(b.length, 2.0)

    def test_too_short_is_refused(self):
        with self.assertRaises(ValueError):
            mitre_corners(0.1, 0.5, 30.0)      # needs 0.25 * cot 30 = 0.433 m
        self.assertIsNone(mitre_of(Belt(id="r", type="linear", x=0, y=0, length=1, beltWidth=0.5, rotation=0,
                                        speed=1, directionAngle=0, color="#fff")))


class AxGeometryTest(unittest.TestCase):
    def config(self):
        line = Belt(id="line", type="linear", x=2.0, y=0.0, length=4.0, beltWidth=0.6, rotation=0.0, speed=1.0,
                    directionAngle=0.0, color="#fff")
        arc = Belt(id="arc", type="curved", shape="arc", x=4.0, y=1.0, radius=1.0, startAngle=270.0, endAngle=360.0,
                   beltWidth=0.6, rotation=0.0, speed=1.0, directionAngle=90.0, color="#fff")
        eye = Sensor(id="eye_1", x=3.0, y=0.0, width=0.05, height=0.8, rotation=0.0, label="", color="#f00")
        return SimulatorConfig(belts=[line, arc], sensors=[eye])

    def test_belts_as_ax_states_them(self):
        c = self.config()
        line = ax_geometry.belt_path(c, c.belts[0])
        self.assertEqual((line.shape, line.start, line.end, line.length), ("line", (0.0, 0.0, 0.75),
                                                                           (4.0, 0.0, 0.75), 4.0))
        arc = ax_geometry.belt_path(c, c.belts[1])
        self.assertEqual((arc.shape, arc.start, arc.end), ("arc", (4.0, 0.0, 0.75), (5.0, 1.0, 0.75)))
        self.assertEqual(arc.segments[0].centre, (4.0, 1.0, 0.75))
        self.assertTrue(arc.segments[0].ccw)
        self.assertEqual((arc.heading_start, arc.heading_end), (0.0, 90.0))

    def test_an_eye_is_a_distance_along_its_belt(self):
        c = self.config()
        self.assertEqual(ax_geometry.along(c, 3.0, 0.0), ("line", 3.0, 0.0))
        st = ax_geometry.st_snippet(c)
        self.assertIn("line.Path.AddLine(startPoint := Point3Of(LREAL#0.000000, LREAL#0.000000, LREAL#0.750000)", st)
        self.assertIn("arc.Path.AddArc(", st)
        self.assertIn("ccw := TRUE", st)
        self.assertIn("line.AddEye(s_um := LINT#3000000, use := EyeUse#Occupied);   // eye_1", st)

    def test_the_merge_demo(self):
        """Its merge and branch points on the grid, every eye on a belt and clear of
        its joints, no check failing."""
        here = os.path.dirname(os.path.abspath(__file__))
        layout = os.path.join(here, "merge_demo.json")
        if not os.path.exists(layout):
            self.skipTest("merge_demo.json not here")
        from config_parser import load_config
        config = load_config(layout)
        geo = ax_geometry.layout_geometry(config)
        cuts = {b["id"]: b for b in geo["belts"] if b["shape"] == "cut"}
        self.assertEqual(sorted(cuts), ["belt_2", "belt_3", "belt_4", "belt_5", "belt_7"])   # belt_5: the fourth side feed
        for p in [cuts[i]["end"] for i in ("belt_2", "belt_3", "belt_4")] + [cuts["belt_7"]["start"]]:
            for v in p[:2]:
                self.assertAlmostEqual(v / 0.125, round(v / 0.125), places=5, msg=p)
        self.assertEqual(ax_geometry.check(config), [])


# the branch as drawn by hand in the designer: its start along the wheels' edge, its left
# side 0.47 degrees off its travel (merge_demo.json, 2026-11-13)
HAND_DRAWN_BRANCH = dict(
    id="belt_7", type="linear", shape="quadrilateral", x=9.025888286914979, y=-1.0883882869149801,
    length=2.030329964423233, beltWidth=1.6767765738299603, rotation=0.0, directionAngle=315.0, speed=1.0,
    color="#3b82f6", vertexs=[{"x": -1.0151649822116156, "y": 0.8383882869149801},
                              {"x": 0.6616115916183443, "y": -0.8383882869149801},
                              {"x": 1.0151649822116173, "y": -0.4848348963217062},
                              {"x": -0.2866115916183425, "y": 0.8383882869149801}])


class PortsTest(unittest.TestCase):
    def test_an_outline_belt_runs_from_edge_middle_to_edge_middle(self):
        b = Belt(**HAND_DRAWN_BRANCH)
        path = ax_geometry.belt_path(SimulatorConfig(), b)
        self.assertEqual(path.start[:2], (8.375, -0.25))            # the middle of its cut start, on the wheels' edge
        self.assertEqual(path.end[:2], (9.864277, -1.75))           # the middle of its square end
        self.assertIsNone(cut_of(b))                                 # not exactly a cut belt ...
        loose = cut_of(b, loose=True)                                # ... but nearly
        self.assertAlmostEqual(loose["start_deg"], 45.0, places=1)
        self.assertTrue(make_cut_belt(b, loose=True))
        cut = cut_of(b)
        self.assertTrue(cut["canonical"])
        self.assertLess(math.dist(cut["start"], (8.375, -0.25)), 0.01)

    def test_a_cut_start(self):
        b = mitred_belt("br", (8.375, -0.25), -45.0, 1.5 * math.sqrt(2), 0.5, 90.0, start_deg=45.0)
        corners = get_belt_polygon(b)
        on_edge = [c for c in corners if abs(c[1] + 0.25) < 1e-9]
        self.assertEqual(len(on_edge), 2, corners)                   # its start lies along the edge y = -0.25
        cut = cut_of(b)
        self.assertEqual((round(cut["start_deg"], 6), round(cut["end_deg"], 6)), (45.0, 90.0))
        set_mitre(b, start_deg=60.0)
        self.assertAlmostEqual(cut_of(b)["start_deg"], 60.0)
        self.assertLess(math.dist(cut_of(b)["start"], (8.375, -0.25)), 1e-9)

    def test_ports_set_by_hand_move_with_the_belt(self):
        b = Belt(**HAND_DRAWN_BRANCH)
        b.inletPoint = ax_geometry.to_local(b, 8.4, -0.25)
        path = ax_geometry.belt_path(SimulatorConfig(), b)
        self.assertEqual(path.start[:2], (8.4, -0.25))
        self.assertIn("ports set by hand (inlet)", path.note)
        b.x += 1.0
        self.assertEqual(ax_geometry.belt_path(SimulatorConfig(), b).start[:2], (9.4, -0.25))

    def test_checks(self):
        feed = Belt(id="feed", type="linear", x=1.0, y=0.0, length=2.0, beltWidth=0.5, rotation=0.0, speed=1.0,
                    directionAngle=0.0, color="#fff")
        lonely = Belt(**HAND_DRAWN_BRANCH)
        on_joint = Sensor(id="gate", x=1.98, y=0.0, width=0.05, height=0.7, rotation=0.0, label="", color="#f00")
        induct = Sensor(id="induct", x=0.3, y=0.0, width=0.05, height=0.7, rotation=0.0, label="", color="#f00")
        from config_parser import Source
        src = Source(id="src", x=0.8, y=0.0, rotation=0.0, interval=5, minWidth=0.3, maxWidth=0.3, minHeight=0.3,
                     maxHeight=0.3, label="")
        problems = ax_geometry.check(SimulatorConfig(belts=[feed, lonely], sensors=[on_joint, induct], sources=[src]))
        text = "\n".join(problems)
        self.assertIn("gate sits on a joint (1.980 m along feed", text)
        self.assertIn("src drops loads after induct", text)
        self.assertIn("belt_7: its inlet (8.375, -0.250) meets no belt's outlet", text)


class OneNameTest(unittest.TestCase):
    def test_a_label_repeating_the_id_is_no_label(self):
        self.assertTrue(label_repeats_id("belt_5_induct", "BELT_5 INDUCT"))
        self.assertTrue(label_repeats_id("s1", "SENSOR"))
        self.assertFalse(label_repeats_id("s1", "Infeed eye"))
        s = Sensor(id="belt_5_gate", x=0, y=0, width=0.1, height=1, rotation=0, label="BELT_5-GATE", color="#f00")
        self.assertEqual((own_label(s), display_name(s)), ("", "belt_5_gate"))


class DesignerTest(unittest.TestCase):
    def setUp(self):
        from layout_editor import MainWindow
        self.tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(self.tmp.cleanup)
        path = os.path.join(self.tmp.name, "layout.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        from config_parser import load_config
        self.window = MainWindow(config=load_config(path), config_path=path,
                                 library_path=os.path.join(self.tmp.name, "library.json"))
        self.addCleanup(self.window.deleteLater)
        self.window.show()
        _app.processEvents()

    def tearDown(self):
        self.window.undo_stack.setClean()
        self.window.close()

    def test_merge_belt_template_and_its_properties(self):
        w = self.window
        item = w.view.place_template("Merge Belt", 2.0, 1.0)
        self.assertIsNotNone(item)
        self.assertAlmostEqual(mitre_of(item.model), 45.0)
        w.scene.clearSelection()
        item.setSelected(True)
        _app.processEvents()
        panel = w.properties_panel
        self.assertIn("mitre.end", panel.spins)
        start = ax_geometry.belt_path(w.config, item.model).start
        panel.spins["mitre.end"].setValue(60.0)
        panel.spins["length"].setValue(1.5)
        self.assertAlmostEqual(mitre_of(item.model), 60.0)
        self.assertAlmostEqual(item.model.length, 1.5)
        self.assertEqual(ax_geometry.belt_path(w.config, item.model).start, start)
        # the AX rows follow, and Copy ST gives the path
        self.assertIn("start (", panel.ax_label.text())
        self.assertIn("end edge 60", panel.ax_label.text())
        panel._copy_ax_st()
        self.assertIn(f"{item.model.id}.Path.AddLine(", QApplication.clipboard().text())
        w.undo_stack.undo()
        w.undo_stack.undo()
        self.assertAlmostEqual(mitre_of(item.model), 45.0)

    def test_ports_and_make_cut_belt_in_the_panel(self):
        from config_parser import Belt as B
        from layout_editor import BeltItem
        w = self.window
        item = w.view._add_item(BeltItem(B(**HAND_DRAWN_BRANCH)), text="Add belt")
        w.scene.clearSelection()
        item.setSelected(True)
        _app.processEvents()
        panel = w.properties_panel
        self.assertAlmostEqual(panel.spins["port.inX"].value(), 8.375)       # from its shape
        panel.spins["port.inX"].setValue(8.4)                                  # set by hand
        self.assertIsNotNone(item.model.inletPoint)
        self.assertEqual(ax_geometry.belt_path(w.config, item.model).start[:2], (8.4, -0.25))
        panel._reset_ports()
        self.assertIsNone(item.model.inletPoint)
        before = [tuple(round(v, 3) for v in c) for c in get_belt_polygon(item.model)]
        panel._make_cut_belt()                                                 # the hand-drawn branch, editable
        self.assertTrue(cut_of(item.model)["canonical"])
        _app.processEvents()
        item.setSelected(False)
        item.setSelected(True)
        _app.processEvents()
        self.assertIn("mitre.start", panel.spins)
        w.undo_stack.undo()                                                    # one step: the shape as drawn
        self.assertEqual([tuple(round(v, 3) for v in c) for c in get_belt_polygon(item.model)], before)

    def test_move_a_selection_to_a_position(self):
        w = self.window
        a = w.view.place_template("Linear Belt", 2.0, 0.0)
        b = w.view.place_template("IR Sensor", 3.0, 0.0)
        before = [(a.model.x, a.model.y), (b.model.x, b.model.y)]
        start = ax_geometry.belt_path(w.config, a.model).start
        w.move_items_by([a, b], 10.0 - start[0], 5.0 - start[1])     # the belt's start onto (10, 5)
        self.assertEqual(ax_geometry.belt_path(w.config, a.model).start[:2], (10.0, 5.0))
        self.assertAlmostEqual(b.model.x - a.model.x, before[1][0] - before[0][0])     # the group kept its shape
        refs = dict(w._selection_references([a, b]))
        self.assertIn(f"start of {a.model.id} (its AX path)", refs)
        w.undo_stack.undo()                                          # one undo step
        self.assertEqual([(a.model.x, a.model.y), (b.model.x, b.model.y)], before)


if __name__ == "__main__":
    unittest.main()
