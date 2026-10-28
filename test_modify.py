"""
test_modify.py - the CAD modify operations on sketch geometry
(cad/modify.py): trim, extend, break, offset, fillet, explode and the
transforms, on lines, arcs and closed shapes - exact arcs throughout.
"""
import math
import unittest

from cad import modify
from cad.path import bulge_to_segment, path_length, segments
from cad.modify import ModifyError

Q = math.tan(math.pi / 8)          # bulge of a quarter arc, CCW


def line(a, b):
    return [a, b], [0.0, 0.0], False


def length(shape):
    return path_length(segments(*shape))


def close(test, p, q, places=7):
    test.assertAlmostEqual(p[0], q[0], places=places)
    test.assertAlmostEqual(p[1], q[1], places=places)


class TrimTest(unittest.TestCase):
    def test_line_between_two_cutters_and_at_an_open_end(self):
        shape = line((0, 0), (10, 0))
        cutters = [line((3, -1), (3, 1)), line((7, -1), (7, 1))]
        left = modify.trim(shape, (5, 0.1), cutters)
        self.assertEqual(len(left), 2)
        close(self, left[0][0][-1], (3, 0))
        close(self, left[1][0][0], (7, 0))
        self.assertAlmostEqual(length(left[0]) + length(left[1]), 6.0)
        (only,) = modify.trim(shape, (1, 0), cutters)                  # the end piece goes
        close(self, only[0][0], (3, 0))
        close(self, only[0][-1], (10, 0))

    def test_an_arc_keeps_its_circle(self):
        semicircle = ([(2, 0), (-2, 0)], [1.0, 0.0], False)            # CCW over the top, R 2
        left = modify.trim(semicircle, (0, 2), [line((0, 0), (3, 3)), line((0, 0), (-3, 3))])
        self.assertEqual(len(left), 2)
        for pts, bul, _closed in left:
            seg = bulge_to_segment(pts[0], pts[1], bul[0])
            self.assertEqual(seg.kind, "arc")
            self.assertAlmostEqual(seg.radius, 2.0)
            close(self, seg.center, (0, 0))
            self.assertAlmostEqual(math.degrees(abs(seg.sweep)), 45.0)

    def test_closed_shape_opens_between_crossings(self):
        square = ([(0, 0), (4, 0), (4, 4), (0, 4)], [0.0] * 4, True)
        (left,) = modify.trim(square, (2, 0), [line((1, -1), (1, 1)), line((3, -1), (3, 1))])
        self.assertFalse(left[2])
        self.assertAlmostEqual(length(left), 16.0 - 2.0)
        close(self, left[0][0], (3, 0))
        close(self, left[0][-1], (1, 0))

    def test_nothing_to_trim_to(self):
        with self.assertRaises(ModifyError):
            modify.trim(line((0, 0), (10, 0)), (5, 0), [line((0, 5), (10, 5))])


class ExtendBreakTest(unittest.TestCase):
    def test_extend_a_line_end_and_its_start(self):
        wall = line((10, -5), (10, 5))
        pts, _b, _c = modify.extend(line((0, 0), (6, 0)), (5, 0), [wall])
        close(self, pts[-1], (10, 0))
        pts, _b, _c = modify.extend(line((6, 0), (8, 0)), (6.2, 0), [line((2, -1), (2, 1))])
        close(self, pts[0], (2, 0))                                    # the start end, nearest the pick
        close(self, pts[-1], (8, 0))
        with self.assertRaises(ModifyError):
            modify.extend(line((0, 0), (6, 0)), (5, 0), [line((-3, -1), (-3, 1))])   # behind it

    def test_extend_an_arc_round_its_circle(self):
        quarter = ([(2, 0), (0, 2)], [Q, 0.0], False)                 # CCW, centre (0, 0), R 2
        pts, bul, _c = modify.extend(quarter, (0.1, 2), [line((0, 0), (-5, 0))])
        close(self, pts[-1], (-2, 0))
        seg = bulge_to_segment(pts[0], pts[1], bul[0])
        self.assertAlmostEqual(math.degrees(seg.sweep), 180.0)
        close(self, seg.center, (0, 0))

    def test_break_open_and_closed(self):
        a, b = modify.break_at(line((0, 0), (10, 0)), (4, 0.3))
        close(self, a[0][-1], (4, 0))
        close(self, b[0][0], (4, 0))
        circle = ([(1, 0), (-1, 0)], [1.0, 1.0], True)
        (opened,) = modify.break_at(circle, (0, 1.5))
        self.assertFalse(opened[2])
        close(self, opened[0][0], (0, 1))
        close(self, opened[0][-1], (0, 1))
        self.assertAlmostEqual(length(opened), 2 * math.pi)


class OffsetFilletTest(unittest.TestCase):
    def test_offset_a_line_either_side(self):
        pts, _b, _c = modify.offset(line((0, 0), (10, 0)), 0.5, (5, 3))
        close(self, pts[0], (0, 0.5))
        pts, _b, _c = modify.offset(line((0, 0), (10, 0)), 0.5, (5, -3))
        close(self, pts[-1], (10, -0.5))

    def test_offset_a_closed_square_outwards_has_mitred_corners(self):
        square = ([(0, 0), (4, 0), (4, 4), (0, 4)], [0.0] * 4, True)
        pts, _b, closed = modify.offset(square, 1.0, (5, 2))
        self.assertTrue(closed)
        self.assertEqual(sorted((round(x, 9), round(y, 9)) for x, y in pts),
                         sorted([(-1, -1), (5, -1), (5, 5), (-1, 5)]))

    def test_offset_a_rounded_corner_changes_the_radius(self):
        corner = ([(0, 0), (4, 0), (5, 1), (5, 5)], [0.0, Q, 0.0, 0.0], False)   # R 1 CCW corner
        pts, bul, _c = modify.offset(corner, 0.25, (2, 1))                       # inside (left)
        segs = segments(pts, bul, False)
        arc = next(s for s in segs if s.kind == "arc")
        self.assertAlmostEqual(arc.radius, 0.75)
        with self.assertRaises(ModifyError):
            modify.offset(corner, 1.5, (2, 1))                                    # past the centre

    def test_fillet_two_lines_with_a_tangent_arc_or_a_sharp_corner(self):
        a, b = line((0, 0), (5, 0)), line((7, 2), (7, 8))                        # don't touch yet
        pts, bul, _c = modify.fillet(a, (1, 0), b, (7, 6), 1.0)
        segs = segments(pts, bul, False)
        self.assertEqual([s.kind for s in segs], ["line", "arc", "line"])
        self.assertAlmostEqual(segs[1].radius, 1.0)
        close(self, pts[0], (0, 0))
        close(self, pts[-1], (7, 8))
        self.assertAlmostEqual(segs[1].heading_at(0.0), segs[0].heading_at(1.0))
        pts, _b, _c = modify.fillet(a, (1, 0), b, (7, 6), 0.0)
        close(self, pts[1], (7, 0))
        with self.assertRaises(ModifyError):
            modify.fillet(a, (1, 0), line((0, 1), (5, 1)), (1, 1), 1.0)          # parallel

    def test_fillet_keeps_the_picked_sides_of_crossing_lines(self):
        a, b = line((0, 0), (10, 0)), line((6, -3), (6, 5))                      # crossing at (6, 0)
        pts, _b, _c = modify.fillet(a, (2, 0), b, (6, 4), 0.0)
        self.assertEqual([tuple(round(v, 9) for v in p) for p in pts], [(0, 0), (6, 0), (6, 5)])


class ExplodeTransformTest(unittest.TestCase):
    def test_explode(self):
        parts = modify.explode(([(0, 0), (4, 0), (5, 1), (5, 5)], [0.0, Q, 0.0, 0.0], False))
        self.assertEqual(len(parts), 3)
        self.assertAlmostEqual(parts[1][1][0], Q)

    def test_transforms(self):
        quarter = ([(2, 0), (0, 2)], [Q, 0.0], False)
        pts, bul, _c = modify.mirror(quarter, (0, 0), (0, 1))
        close(self, pts[0], (-2, 0))
        self.assertAlmostEqual(bul[0], -Q)                                       # turns the other way
        pts, _b, _c = modify.rotate(quarter, (0, 0), math.pi / 2)
        close(self, pts[0], (0, 2))
        pts, _b, _c = modify.scale(quarter, (0, 0), 2.0)
        close(self, pts[1], (0, 4))
        pts, _b, _c = modify.move(quarter, 1.0, -1.0)
        close(self, pts[0], (3, -1))


class EditGeometryTest(unittest.TestCase):
    def test_kinds_and_describe(self):
        self.assertEqual(modify.kind_of(line((0, 0), (3, 4))), "line")
        self.assertEqual(modify.kind_of(([(2, 0), (0, 2)], [Q, 0.0], False)), "arc")
        self.assertEqual(modify.kind_of(modify.make_circle((1, 1), 2)), "circle")
        self.assertEqual(modify.kind_of(([(0, 0), (1, 0), (1, 1)], [0.0] * 3, False)), "polyline")
        info = modify.describe(line((0, 0), (3, 4)))
        self.assertAlmostEqual(info["length"], 5.0)
        self.assertAlmostEqual(info["angle"], math.degrees(math.atan2(4, 3)))
        info = modify.describe(([(2, 0), (0, 2)], [Q, 0.0], False))
        close(self, info["center"], (0, 0))
        self.assertAlmostEqual(info["radius"], 2.0)
        self.assertAlmostEqual(info["start"], 0.0)
        self.assertAlmostEqual(info["sweep"], 90.0)
        c, r = modify.circle_info(modify.make_circle((1, 1), 2))
        close(self, c, (1, 1))
        self.assertAlmostEqual(r, 2.0)

    def test_make_arc_round_trips(self):
        arc = modify.make_arc((1, -1), 3.0, 30.0, -120.0)
        info = modify.describe(arc)
        close(self, info["center"], (1, -1))
        self.assertAlmostEqual(info["radius"], 3.0)
        self.assertAlmostEqual(info["start"], 30.0)
        self.assertAlmostEqual(info["sweep"], -120.0)
        with self.assertRaises(ModifyError):
            modify.make_arc((0, 0), 0.0, 0.0, 90.0)
        with self.assertRaises(ModifyError):
            modify.make_circle((0, 0), -1.0)

    def test_moving_a_vertex_bends_the_arc_beside_it_through_its_midpoint(self):
        corner = ([(0, 0), (4, 0), (5, 1), (5, 5)], [0.0, Q, 0.0, 0.0], False)
        mid = segments(*corner)[1].point_at(0.5)
        pts, bul, _c = modify.move_vertices(corner, {2: (5, 2)})
        close(self, pts[2], (5, 2))
        arc = segments(pts, bul, False)[1]
        self.assertEqual(arc.kind, "arc")
        close(self, arc.p0, (4, 0))
        close(self, arc.p1, (5, 2))
        self.assertAlmostEqual(math.dist(arc.center, mid), arc.radius)       # still through the old middle
        pts, bul, _c = modify.move_vertices(corner, {1: (4, 1), 2: (5, 2)})  # both ends by (0, 1): same arc
        arc = segments(pts, bul, False)[1]
        self.assertAlmostEqual(arc.radius, 1.0)

    def test_bend_a_segment(self):
        pts, bul, _c = modify.bend_segment(line((0, 0), (2, 0)), 0, (1, 1))
        arc = segments(pts, bul, False)[0]
        self.assertAlmostEqual(arc.radius, 1.0)
        close(self, arc.center, (1, 0))
        _p, bul, _c = modify.bend_segment(line((0, 0), (2, 0)), 0, (1, 0))      # in line: straight
        self.assertEqual(bul[0], 0.0)


if __name__ == "__main__":
    unittest.main()
