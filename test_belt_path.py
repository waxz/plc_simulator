"""
test_belt_path.py - drawn ("path") belts without Qt: the vector path math
(bulges, tangent arcs, fillets, offset footprint, typed input, presets),
physics on a path belt in both traction models, ports, layout checks, DXF.
"""
import math
import os
import tempfile
import unittest

import ezdxf

from cad import dxf_io, path as cpath
from cad.checks import belt_footprint, check_layout
from cad.ports import belt_ports
from config_parser import Belt, PathNode, SimulatorConfig, load_config
from physics import PhysicsEngine


def path_belt(points, bulges, closed=False, width=1.0, speed=1.0, direction=0.0, x=0.0, y=0.0, rotation=0.0):
    return Belt(id="p", type="path", shape="path", x=x, y=y, rotation=rotation, beltWidth=width, speed=speed,
                directionAngle=direction, color="#fff", closed=closed,
                path=[PathNode(x=px, y=py, bulge=b) for (px, py), b in zip(points, bulges)])


def preset_belt(kind, size, size_b=0.0, corner=0.0, **kw):
    pts, bulges, closed = cpath.preset(kind, size, size_b, corner_radius=corner)
    return path_belt(pts, bulges, closed, **kw)


class BulgeMathTest(unittest.TestCase):
    def test_bulge_one_is_a_semicircle(self):
        seg = cpath.bulge_to_segment((0, 0), (2, 0), 1.0)
        self.assertAlmostEqual(seg.radius, 1.0)
        self.assertAlmostEqual(seg.center[0], 1.0)
        self.assertAlmostEqual(seg.center[1], 0.0)
        self.assertAlmostEqual(seg.length, math.pi)
        self.assertAlmostEqual(math.degrees(seg.heading_at(0.0)) % 360, 270.0)  # CCW over the bottom

    def test_negative_bulge_turns_right(self):
        seg = cpath.bulge_to_segment((0, 0), (1, -1), -math.tan(math.pi / 8))
        self.assertLess(seg.sweep, 0)
        self.assertAlmostEqual(seg.center[1], -1.0)

    def test_tangent_arc_continues_the_heading(self):
        b = cpath.tangent_bulge((0, 0), 0.0, (1, 1))
        seg = cpath.bulge_to_segment((0, 0), (1, 1), b)
        self.assertAlmostEqual(seg.heading_at(0.0), 0.0)
        self.assertAlmostEqual(math.degrees(seg.heading_at(1.0)), 90.0)
        self.assertAlmostEqual(seg.radius, 1.0)

    def test_fillet_shortens_a_square_perimeter_exactly(self):
        pts, bulges, closed = cpath.preset("rectangle", 10, 10, corner_radius=2)
        self.assertEqual(len(pts), 8)
        length = cpath.path_length(cpath.segments(pts, bulges, closed))
        self.assertAlmostEqual(length, 40 - 4 * 2 * 2 + 2 * math.pi * 2, places=9)

    def test_fillet_radius_is_limited_by_short_segments(self):
        pts, bulges = cpath.fillet_corners([(0, 0), (1, 0), (1, 5)], [0, 0, 0], False, 10.0)
        self.assertAlmostEqual(pts[1][0], 0.5)  # gave up half of the 1 m segment, no more


class OffsetTest(unittest.TestCase):
    def test_footprint_area_is_length_times_width_for_every_preset(self):
        for kind in cpath.PRESETS:
            with self.subTest(kind=kind):
                pts, bulges, closed = cpath.preset(kind, 6.0, 3.0, corner_radius=1.0)
                surface = cpath.footprint(pts, bulges, closed, 0.8)
                length = cpath.path_length(cpath.segments(pts, bulges, closed))
                self.assertTrue(surface.is_valid)
                self.assertAlmostEqual(surface.area, length * 0.8, delta=0.002 * length)
                self.assertEqual(len(surface.interiors), 1 if closed else 0)

    def test_sharp_corner_is_mitred(self):
        surface = cpath.footprint([(0, 0), (4, 0), (4, 4)], [0, 0, 0], False, 1.0)
        self.assertTrue(surface.covers(cpath.Polygon([(4.45, -0.45), (4.49, -0.45), (4.49, -0.49)])))

    def test_world_path_applies_offset_and_rotation(self):
        belt = path_belt([(0, 0), (2, 0)], [0, 0], x=1, y=1, rotation=90)
        pts, _ = cpath.belt_world_path(belt)
        self.assertAlmostEqual(pts[1][0], 1.0)
        self.assertAlmostEqual(pts[1][1], 3.0)


class InputTest(unittest.TestCase):
    def test_coordinate_formats(self):
        self.assertEqual(cpath.parse_point_input("3,4", None), (3.0, 4.0))
        self.assertEqual(cpath.parse_point_input(" @1, -2 ", (1, 1)), (2.0, -1.0))
        x, y = cpath.parse_point_input("2<90", (1, 1))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 3.0)
        self.assertEqual(cpath.parse_point_input("5", (0, 0), 0.0), (5.0, 0.0))
        self.assertIsNone(cpath.parse_point_input("@1,1", None))
        self.assertIsNone(cpath.parse_point_input("nonsense", (0, 0)))

    def test_angle_snap_keeps_distance(self):
        x, y = cpath.snap_angle((0, 0), (1.0, 0.25), 15.0)
        self.assertAlmostEqual(math.hypot(x, y), math.hypot(1.0, 0.25))
        self.assertAlmostEqual(math.degrees(math.atan2(y, x)), 15.0)


class PathPhysicsTest(unittest.TestCase):
    def run_u_turn(self, mode):
        e = PhysicsEngine(SimulatorConfig(belts=[preset_belt("u_turn", 8, 3)], tractionModel=mode,
                                          useSurfaceVelocity=True, physicsDt=0.004))
        p = e.add_parcel(-3.5, -1.5, 0.4, 0.4)
        e.run_for(8 + math.pi * 1.5 + 2)
        return p

    def test_parcel_rides_a_u_turn_and_comes_back_reversed(self):
        for mode in ("kinematic", "friction"):
            with self.subTest(mode=mode):
                p = self.run_u_turn(mode)
                self.assertAlmostEqual(p.body.position.y, 1.5, delta=0.05)    # on the return leg
                self.assertAlmostEqual(p.body.velocity.x, -1.0, delta=0.01)
                self.assertAlmostEqual(abs(p.body.velocity.y), 0.0, delta=0.01)
                self.assertAlmostEqual(math.degrees(p.body.angle) % 360, 180.0, delta=2.0)

    def test_parcel_stays_on_a_circle_loop(self):
        for mode in ("kinematic", "friction"):
            with self.subTest(mode=mode):
                e = PhysicsEngine(SimulatorConfig(belts=[preset_belt("circle", 6)], tractionModel=mode,
                                                  useSurfaceVelocity=True, physicsDt=0.004))
                p = e.add_parcel(0.0, -3.0, 0.4, 0.4)
                for _ in range(10):  # one full lap, checking the radius on the way
                    e.run_for(math.pi * 6 / 10)
                    self.assertAlmostEqual(p.body.position.length, 3.0, delta=0.05)

    def test_reversed_belt_runs_against_anchor_order(self):
        e = PhysicsEngine(SimulatorConfig(belts=[path_belt([(0, 0), (10, 0)], [0, 0], direction=180)],
                                          tractionModel="kinematic", useSurfaceVelocity=True))
        p = e.add_parcel(5, 0, 0.4, 0.4)
        e.run_for(0.5)
        self.assertAlmostEqual(p.body.velocity.x, -1.0, places=6)


class PathIntegrationTest(unittest.TestCase):
    def test_open_path_ports_follow_the_path_ends(self):
        ports = belt_ports(preset_belt("l_bend", 6, 4, corner=1.0))
        self.assertAlmostEqual(ports.inlet.heading % 360, 0.0, places=6)
        self.assertAlmostEqual(ports.outlet.heading % 360, 90.0, places=6)
        self.assertIsNone(belt_ports(preset_belt("circle", 6)))  # a loop has no ends

    def test_checks_use_the_offset_footprint_and_flag_tight_curves(self):
        loop = preset_belt("circle", 6)
        self.assertAlmostEqual(belt_footprint(loop).area, math.pi * 6 * 1.0, delta=0.05)
        tight = path_belt([(0, 0), (0.6, 0)], [1.0, 0], width=1.0)  # radius 0.3 < 0.5
        kinds = [i.kind for i in check_layout(SimulatorConfig(belts=[tight]))]
        self.assertIn("curve too tight", kinds)

    def test_dxf_export_writes_the_centerline_as_a_bulge_polyline(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "p.dxf")
            dxf_io.export_layout(SimulatorConfig(belts=[preset_belt("u_turn", 8, 3)]), out)
            msp = ezdxf.readfile(out).modelspace()
            center = msp.query('LWPOLYLINE[layer=="BELT_CENTERLINES"]').first
            bulges = [round(b, 6) for *_xy, b in center.get_points("xyb")]
            self.assertEqual(bulges[:3], [0.0, 1.0, 0.0])

    def test_saved_path_belt_round_trips(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "layout.json")
            belt = preset_belt("triangle", 6, corner=1.0)
            with open(path, "w", encoding="utf-8") as f:
                f.write(SimulatorConfig(belts=[belt]).model_dump_json())
            loaded = load_config(path).belts[0]
            self.assertEqual(loaded.type, "path")
            self.assertTrue(loaded.closed)
            self.assertEqual([(n.x, n.y, n.bulge) for n in loaded.path], [(n.x, n.y, n.bulge) for n in belt.path])


if __name__ == "__main__":
    unittest.main()
