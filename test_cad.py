"""
test_cad.py - the Qt-free CAD core (cad/): belt ports, planegcs constraints
(connect, fillet, solve), Shapely layout checks, ezdxf import/export.
Geometry is checked against closed-form values.
"""
import math
import os
import tempfile
import unittest

import ezdxf

from cad import dxf_io
from cad.checks import check_layout
from cad.constraints import ConstraintError, connect_belts, fillet_belts, solve_layout
from cad.ports import belt_ports, heading_difference
from config_parser import Belt, BeltConnection, Sensor, Sink, SimulatorConfig, Source, load_config


def straight(id, x, y, length, rotation, speed=1.0, direction=0.0, width=1.0):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=width, rotation=rotation,
                speed=speed, directionAngle=direction, color="#fff")


def curve(id, x, y, radius, start, end, ccw=True, rotation=0.0):
    return Belt(id=id, type="curved", shape="arc", x=x, y=y, radius=radius, startAngle=start, endAngle=end,
                rotation=rotation, beltWidth=1.0, speed=1.0, directionAngle=90.0 if ccw else -90.0, color="#fff")


def apply(belts, geometry):
    by_id = {b.id: b for b in belts}
    for belt_id, geom in geometry.items():
        by_id[belt_id] = by_id[belt_id].model_copy(update=geom)
    return by_id


class PortsTest(unittest.TestCase):
    def assertPort(self, port, x, y, heading):
        self.assertAlmostEqual(port.x, x, places=9)
        self.assertAlmostEqual(port.y, y, places=9)
        self.assertAlmostEqual(heading_difference(port.heading, heading), 0.0, places=9)

    def test_linear_belt_flows_along_its_rotation(self):
        p = belt_ports(straight("a", 0, 0, 4, 90))
        self.assertPort(p.inlet, 0, -2, 90)
        self.assertPort(p.outlet, 0, 2, 90)

    def test_reversed_direction_or_negative_speed_swaps_the_ends(self):
        for belt in (straight("a", 0, 0, 4, 0, direction=180), straight("a", 0, 0, 4, 0, speed=-1)):
            p = belt_ports(belt)
            self.assertPort(p.inlet, 2, 0, 180)
            self.assertPort(p.outlet, -2, 0, 180)

    def test_cross_flow_and_custom_shapes_have_no_ports(self):
        self.assertIsNone(belt_ports(straight("a", 0, 0, 4, 0, direction=90)))
        quad = straight("q", 0, 0, 4, 0).model_copy(update={"shape": "quadrilateral",
                                                             "vertexs": [{"x": 0, "y": 0}] * 4})
        self.assertIsNone(belt_ports(quad))

    def test_curved_belt_ports_follow_flow_direction(self):
        p = belt_ports(curve("c", 0, 0, 2, 270, 360, ccw=True))
        self.assertPort(p.inlet, 0, -2, 0)
        self.assertPort(p.outlet, 2, 0, 90)
        p = belt_ports(curve("c", 0, 0, 2, 270, 360, ccw=False))
        self.assertPort(p.inlet, 2, 0, 270)
        self.assertPort(p.outlet, 0, -2, 180)

    def test_curved_rotation_turns_the_arc(self):
        p = belt_ports(curve("c", 0, 0, 2, 270, 360, rotation=90))
        self.assertPort(p.inlet, 2, 0, 90)
        self.assertPort(p.outlet, 0, 2, 180)


class ConstraintTest(unittest.TestCase):
    def assertConnected(self, a, b):
        pa, pb = belt_ports(a).outlet, belt_ports(b).inlet
        self.assertAlmostEqual(math.hypot(pa.x - pb.x, pa.y - pb.y), 0.0, places=6)
        self.assertAlmostEqual(heading_difference(pa.heading, pb.heading), 0.0, places=4)

    def test_connect_moves_b_onto_a_outlet_and_keeps_its_length(self):
        a, b = straight("a", 2.5, 0, 5, 0), straight("b", 10, 10, 3, 37)
        moved = apply([a, b], connect_belts(a, b, [a, b], []))
        self.assertConnected(moved["a"], moved["b"])
        self.assertEqual(moved["a"], a, "upstream belt must not move")
        self.assertAlmostEqual(moved["b"].length, 3.0)

    def test_connect_a_straight_to_a_curve_makes_them_tangent(self):
        a = straight("a", 2.5, 0, 5, 0)
        c = curve("c", 20, 20, 2.0, 0, 90)
        moved = apply([a, c], connect_belts(a, c, [a, c], []))
        self.assertConnected(moved["a"], moved["c"])
        self.assertAlmostEqual(moved["c"].radius, 2.0, places=6)
        sweep = (moved["c"].endAngle - moved["c"].startAngle) % 360
        self.assertAlmostEqual(sweep, 90.0, places=4)

    def test_left_fillet_matches_the_closed_form(self):
        a, b = straight("a", 2.5, 0, 5, 0), straight("b", 6, 3, 6, 90)  # corner at (6, 0)
        r = fillet_belts(a, b, 2.0, "c")
        belts = apply([a, b], {"a": r.upstream, "b": r.downstream})
        pa, pc, pb = belt_ports(belts["a"]), belt_ports(r.curve), belt_ports(belts["b"])
        self.assertAlmostEqual(pa.outlet.x, 4.0, places=6)          # corner - R*tan(45)
        self.assertAlmostEqual(pb.inlet.y, 2.0, places=6)
        self.assertAlmostEqual(r.curve.x, 4.0, places=6)
        self.assertAlmostEqual(r.curve.y, 2.0, places=6)
        self.assertAlmostEqual(belts["a"].length, 4.0, places=6)    # inlet (0,0) kept
        self.assertAlmostEqual(belts["b"].length, 4.0, places=6)    # outlet (6,6) kept
        self.assertConnected(belts["a"], r.curve)
        self.assertConnected(r.curve, belts["b"])
        self.assertGreater(r.curve.directionAngle, 0)               # left turn = CCW
        self.assertEqual([(c.fromBelt, c.toBelt) for c in r.connections], [("a", "c"), ("c", "b")])

    def test_right_fillet_and_non_right_angle(self):
        a = straight("a", 2.5, 0, 5, 0)
        b = straight("b", 8, -2, 4, -45)  # heads south-east, 45 deg right turn
        r = fillet_belts(a, b, 1.5, "c")
        belts = apply([a, b], {"a": r.upstream, "b": r.downstream})
        self.assertConnected(belts["a"], r.curve)
        self.assertConnected(r.curve, belts["b"])
        self.assertLess(r.curve.directionAngle, 0)
        self.assertAlmostEqual((r.curve.endAngle - r.curve.startAngle) % 360, 45.0, places=4)

    def test_fillet_rejects_parallel_belts_and_oversized_radius(self):
        a = straight("a", 2.5, 0, 5, 0)
        with self.assertRaises(ConstraintError):
            fillet_belts(a, straight("p", 2.5, 5, 5, 0), 1.0, "c")
        with self.assertRaises(ConstraintError):
            fillet_belts(a, straight("b", 6, 3, 6, 90), 20.0, "c")

    def test_moving_one_belt_drags_its_connected_chain(self):
        a, b = straight("a", 2.5, 0, 5, 0), straight("b", 6, 3, 6, 90)
        r = fillet_belts(a, b, 2.0, "c")
        belts = apply([a, b, r.curve], {"a": r.upstream, "b": r.downstream})
        belts["a"] = belts["a"].model_copy(update={"x": belts["a"].x + 1, "y": belts["a"].y - 2, "rotation": 15})
        solved = apply(list(belts.values()), solve_layout(list(belts.values()), r.connections, ["a"]))
        self.assertConnected(solved["a"], solved["c"])
        self.assertConnected(solved["c"], solved["b"])
        self.assertAlmostEqual(solved["b"].length, belts["b"].length, places=6)

    def test_a_group_with_no_fixed_belt_is_anchored_not_left_floating(self):
        a, b = straight("a", 0, 0, 2, 0), straight("b", 5, 5, 2, 30)
        conns = [BeltConnection(fromBelt="a", toBelt="b")]
        result = solve_layout([a, b], conns, fixed_ids=[])
        self.assertNotIn("a", result)  # first belt holds still
        self.assertConnected(a, apply([a, b], result)["b"])

    def test_impossible_constraints_are_reported(self):
        a, b, c = straight("a", 0, 0, 2, 0), straight("b", 2, 0, 2, 0), straight("c", 10, 7, 2, 90)
        conns = [BeltConnection(fromBelt="a", toBelt="b"), BeltConnection(fromBelt="b", toBelt="c")]
        with self.assertRaises(ConstraintError):
            solve_layout([a, b, c], conns, fixed_ids=["a", "c"])


class ChecksTest(unittest.TestCase):
    def kinds(self, config):
        return sorted(i.kind for i in check_layout(config))

    def test_clean_layout_has_no_issues(self):
        a, b = straight("a", 2.5, 0, 5, 0), straight("b", 7.5, 0, 5, 0)
        config = SimulatorConfig(belts=[a, b], connections=[BeltConnection(fromBelt="a", toBelt="b")],
                                 sinks=[Sink(id="k", x=10.5, y=0, width=1, height=1, label="K")],
                                 sources=[Source(id="s", x=0.5, y=0, rotation=0, interval=1, minWidth=0.3,
                                                 maxWidth=0.3, minHeight=0.3, maxHeight=0.3, label="S")],
                                 sensors=[Sensor(id="ir", x=4, y=0, width=0.1, height=1, rotation=0,
                                                 label="IR", color="#f00")])
        self.assertEqual(check_layout(config), [])

    def test_stacked_belts_are_an_overlap_error(self):
        config = SimulatorConfig(belts=[straight("a", 0, 0, 5, 0), straight("b", 0.5, 0.2, 5, 0)],
                                 sinks=[Sink(id="k", x=3.5, y=0, width=1, height=2, label="K")])
        issues = check_layout(config)
        self.assertEqual(issues[0].kind, "belt overlap")
        self.assertEqual(issues[0].severity, "error")

    def test_end_overlap_at_a_transfer_is_not_an_error(self):
        config = SimulatorConfig(belts=[straight("a", 2.5, 0, 5, 0), straight("b", 7.4, 0, 5, 0)])
        self.assertNotIn("belt overlap", self.kinds(config))

    def test_open_outlet_broken_connection_and_off_belt_components(self):
        a, b = straight("a", 2.5, 0, 5, 0), straight("b", 20, 0, 5, 0)
        config = SimulatorConfig(
            belts=[a, b], connections=[BeltConnection(fromBelt="a", toBelt="b"),
                                       BeltConnection(fromBelt="b", toBelt="gone")],
            sensors=[Sensor(id="ir", x=10, y=10, width=0.1, height=1, rotation=0, label="IR", color="#f00")],
            sources=[Source(id="s", x=10, y=-10, rotation=0, interval=1, minWidth=0.3, maxWidth=0.3,
                            minHeight=0.3, maxHeight=0.3, label="S")])
        kinds = self.kinds(config)
        self.assertEqual(kinds.count("broken connection"), 2)
        self.assertIn("open outlet", kinds)       # b leads nowhere
        self.assertIn("sensor off belt", kinds)
        self.assertIn("source off belt", kinds)

    def test_existing_project_layouts_only_report_real_problems(self):
        issues = check_layout(load_config("test_config.json"))
        self.assertFalse([i for i in issues if i.severity == "error"], [i.message for i in issues])


class DxfTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_export_writes_a_valid_metre_drawing_with_layers(self):
        path = os.path.join(self.tmp.name, "out.dxf")
        config = load_config("test_config.json")
        dxf_io.export_layout(config, path)
        doc = ezdxf.readfile(path)
        self.assertEqual(doc.units, ezdxf.units.M)
        self.assertEqual(len(doc.audit().errors), 0)
        msp = doc.modelspace()
        self.assertEqual(len(msp.query('LWPOLYLINE[layer=="BELTS"]')), len(config.belts))
        curved = sum(1 for b in config.belts if b.type == "curved")
        self.assertEqual(len(msp.query('ARC[layer=="BELT_CENTERLINES"]')), curved)
        self.assertEqual(len(msp.query('LWPOLYLINE[layer=="SENSORS"]')), len(config.sensors))

    def test_exported_belt_outline_matches_the_layout(self):
        path = os.path.join(self.tmp.name, "one.dxf")
        dxf_io.export_layout(SimulatorConfig(belts=[straight("a", 1, 2, 4, 90, width=0.5)]), path)
        outline = ezdxf.readfile(path).modelspace().query('LWPOLYLINE[layer=="BELTS"]').first
        xs = [p[0] for p in outline.get_points()]
        ys = [p[1] for p in outline.get_points()]
        self.assertAlmostEqual(min(xs), 0.75, places=6)
        self.assertAlmostEqual(max(ys), 4.0, places=6)

    def test_background_import_expands_blocks_skips_text_and_reads_units(self):
        path = os.path.join(self.tmp.name, "plan.dxf")
        doc = ezdxf.new()
        doc.units = ezdxf.units.MM
        msp = doc.modelspace()
        msp.add_lwpolyline([(0, 0), (40000, 0), (40000, 20000), (0, 20000)], close=True)
        msp.add_arc((10000, 10000), 2000, 0, 90)
        msp.add_text("HALL A").set_placement((100, 100))
        block = doc.blocks.new("COLUMN")
        block.add_lwpolyline([(-200, -200), (200, -200), (200, 200), (-200, 200)], close=True)
        for x in range(0, 40001, 10000):
            msp.add_blockref("COLUMN", (x, 10000))
        doc.saveas(path)

        geom = dxf_io.read_background(path)
        self.assertEqual(len(geom.polylines), 1 + 1 + 5)  # hall, arc, 5 columns; text skipped
        self.assertAlmostEqual(geom.scale, 0.001)
        metres = dxf_io.background_in_metres(geom, geom.scale, 1.0, 2.0)
        hall = max(metres, key=len)
        self.assertIn((41.0, 22.0), [(round(x, 6), round(y, 6)) for x, y in hall])


if __name__ == "__main__":
    unittest.main()
