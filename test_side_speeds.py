"""
test_side_speeds.py - a belt or wheel diverter meeting a line at an angle
runs the line's speed / cos(angle) (side_speeds.py): the side inductions'
chains, the belt after the wheel diverter and the wheels themselves;
Tools > Check Layout reports a mismatch, and a speed its drive cannot run.
"""
import math
import unittest

import side_speeds
from cad.checks import check_layout
from config_parser import load_config

FACTOR = 1.0 / math.cos(math.radians(30.0))         # 1.1547


class SideSpeedTest(unittest.TestCase):
    def test_the_demo_runs_its_side_belts_and_wheels_at_the_angle(self):
        config = load_config("merge_demo.json")               # the line at 2 m/s: the side at 2.309
        expected = side_speeds.expected_speeds(config)
        chain = [f"belt_2_{x}" for x in "abcde"] + ["belt_2"]
        for ident in chain + ["belt_4_a", "belt_3", "belt_7", "div_1"]:
            self.assertAlmostEqual(expected[ident][0], 2.0 * FACTOR, places=3, msg=ident)
        for ident in ("infeed", "belt_out", "belt_1", "belt_6"):
            self.assertNotIn(ident, expected)                   # the line itself and its straight belts
        self.assertEqual(side_speeds.problems(config), [])
        self.assertFalse([i for i in check_layout(config) if i.kind == "side speed"])

    def test_a_faster_line_asks_its_side_belts_for_more(self):
        config = load_config("merge_demo.json")
        for belt in config.belts:
            if belt.id == "infeed":
                belt.speed = 2.2
        expected = side_speeds.expected_speeds(config)
        self.assertAlmostEqual(expected["belt_3_b"][0], 2.2 * FACTOR, places=3)     # 2.540 m/s
        self.assertAlmostEqual(expected["div_1"][0], 2.2 * FACTOR, places=3)
        issues = [i for i in check_layout(config) if i.kind == "side speed"]
        self.assertIn("belt_7", {i.item_ids[0] for i in issues if i.severity == "warning"})
        too_fast = {i.item_ids[0] for i in issues if i.severity == "error"}
        self.assertIn("belt_2_a", too_fast)                                        # drives at 2.5 m/s
        self.assertIn("belt_3", too_fast)


if __name__ == "__main__":
    unittest.main()
