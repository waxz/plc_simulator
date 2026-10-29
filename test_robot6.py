"""
test_robot6.py - the 6-axis industrial robot: its kinematics (robot6.py:
inverse against forward, limits, reach), the pick-and-place program on real
joints (machines.py), the FANUC-style PLC interface (enable, hold, program
select, speed override, move to target, gripper, fault and reset) and its
I/O frame fields, and the script API (RobotHandle).
"""
import math
import random
import unittest

import heights
import robot6
from config_parser import Pallet, RobotArm, SimulatorConfig
from mqtt_message_binding import custom_component_mapping
from physics import PhysicsEngine


def layout(**robot):
    args = dict(id="r1", x=0.0, y=0.0, pickX=1.2, pickY=0.0, placeX=0.0, placeY=1.2, cycleTime=1.5)
    args.update(robot)
    return SimulatorConfig(robots=[RobotArm(**args)])


def started(**robot):
    """A robot the PLC has enabled and started on program 1 (pick and place)."""
    return layout(enable=True, program=1, start=True, **robot)


class KinematicsTest(unittest.TestCase):
    def test_inverse_meets_forward_tool_down_with_the_asked_yaw(self):
        geo = robot6.Geometry(2.5)
        rng = random.Random(7)
        solved = 0
        for _ in range(1500):
            x, y, z, yaw = rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.3, 1.8), rng.uniform(-180, 180)
            try:
                q = robot6.inverse(geo, x, y, z, yaw)
            except robot6.Unreachable:
                continue
            solved += 1
            f = robot6.forward(geo, q)
            for a, b in zip(f["tcp"], (x, y, z)):
                self.assertAlmostEqual(a, b, places=9)
            self.assertAlmostEqual(f["axis"][2], -1.0, places=9)             # pointing straight down
            jaw = math.degrees(math.atan2(f["jaw"][1], f["jaw"][0]))
            self.assertAlmostEqual(math.cos(math.radians(jaw - yaw - 90)), 1.0, places=9)
            self.assertTrue(robot6.within_limits(q))
        self.assertGreater(solved, 500)

    def test_out_of_reach_and_scaling(self):
        geo = robot6.Geometry(1.0)
        with self.assertRaises(robot6.Unreachable):
            robot6.inverse(geo, 3.0, 0.0, 0.8)
        big = robot6.Geometry(2.0)
        self.assertAlmostEqual(big.a2, 2 * geo.a2)
        q = robot6.inverse(big, 1.2, 0.5, 1.0, 30.0)
        self.assertEqual(q[3], 0.0)                                          # J4 stays at 0 tool-down
        self.assertAlmostEqual(q[4], 90.0 - q[2])                            # J5 = 90 - J3


class CycleTest(unittest.TestCase):
    def test_the_cycle_moves_real_joints_and_grips_at_the_part_top(self):
        c = started()
        e = PhysicsEngine(c)
        part = e.add_parcel(1.2, 0.0, 0.3, 0.3)
        r = c.robots[0]
        self.assertTrue(r.atHome)
        top = heights.surface_z(c, 0) + heights.part_height(part)
        lowest, gripped_at = 9.0, None
        for _ in range(int(1.6 / c.physicsDt)):
            e.step(c.physicsDt)
            lowest = min(lowest, r.tcpZ)
            if r.holding and gripped_at is None:
                gripped_at = (r.tcpX, r.tcpY, r.tcpZ)
        self.assertAlmostEqual(gripped_at[0], 1.2, places=3)
        self.assertAlmostEqual(gripped_at[2], top, places=3)                 # down onto the part's top
        self.assertAlmostEqual(lowest, top, places=3)
        self.assertEqual(r.cycles, 1)
        self.assertTrue(r.atHome and r.inPosition and not r.busy)
        self.assertAlmostEqual(part.body.position.x, 0.0, places=6)          # set down at the place point
        self.assertAlmostEqual(part.body.position.y, 1.2, places=6)

    def test_onto_a_pallet_the_part_lands_on_the_deck(self):
        c = started(placeX=0.0, placeY=1.2)
        c.pallets.append(Pallet(id="p", x=0.0, y=1.2))
        e = PhysicsEngine(c)
        part = e.add_parcel(1.2, 0.0, 0.3, 0.3)
        r = c.robots[0]
        released_z = None
        for _ in range(int(1.6 / c.physicsDt)):
            holding = r.holding
            e.step(c.physicsDt)
            if holding and not r.holding:
                released_z = r.tcpZ
        self.assertEqual(e.machines.pallets[0].loads, [part])
        deck = heights.surface_z(c, 0) + heights.PALLET_H
        self.assertAlmostEqual(released_z - heights.part_height(part), deck, places=3)

    def test_speed_override_stretches_the_cycle(self):
        c = started(speedOverride=50)
        e = PhysicsEngine(c)
        e.add_parcel(1.2, 0.0, 0.3, 0.3)
        e.run_for(1.6)
        self.assertEqual(c.robots[0].cycles, 0)                              # half speed: not yet
        e.run_for(1.5)
        self.assertEqual(c.robots[0].cycles, 1)


class PlcInterfaceTest(unittest.TestCase):
    def setUp(self):
        self.c = layout(control="plc")
        self.e = PhysicsEngine(self.c)
        self.r = self.c.robots[0]

    def pulse(self, field):
        setattr(self.r, field, True)
        self.e.run_for(0.02)
        setattr(self.r, field, False)

    def test_enable_start_and_hold(self):
        self.e.add_parcel(1.2, 0.0, 0.3, 0.3)
        self.r.enable = False
        self.e.run_for(0.05)
        self.assertFalse(self.r.ready)
        self.pulse("start")
        self.assertFalse(self.r.busy)                                        # not enabled: ignored
        self.r.enable = True
        self.e.run_for(0.05)
        self.assertTrue(self.r.ready)
        self.pulse("start")
        self.assertTrue(self.r.busy and not self.r.ready)
        self.e.run_for(0.2)
        self.r.hold = True
        q = (self.r.j1, self.r.j2, self.r.j3)
        self.e.run_for(0.5)
        self.assertEqual(q, (self.r.j1, self.r.j2, self.r.j3))              # held: stands still
        self.r.hold = False
        self.e.run_for(1.5)
        self.assertEqual(self.r.cycles, 1)

    def test_move_to_target_grip_and_release(self):
        part = self.e.add_parcel(1.2, 0.0, 0.3, 0.3)
        top = heights.surface_z(self.c, 0) + heights.part_height(part)
        self.r.program = 3
        self.r.targetX, self.r.targetY, self.r.targetZ, self.r.targetYaw = 1.2, 0.0, top, 0.0
        self.pulse("start")
        self.e.run_for(3.0)
        self.assertFalse(self.r.busy)
        self.assertAlmostEqual(self.r.tcpX, 1.2, places=4)
        self.assertAlmostEqual(self.r.tcpZ, top, places=4)
        self.r.gripperClose = True
        self.e.run_for(0.02)
        self.assertTrue(self.r.holding)
        self.assertNotIn(part, self.e.parcels)
        self.r.targetX, self.r.targetY = 0.0, -1.2                          # carry it elsewhere
        self.pulse("start")
        self.e.run_for(3.0)
        self.r.gripperClose = False
        self.e.run_for(0.02)
        self.assertFalse(self.r.holding)
        self.assertIn(part, self.e.parcels)
        self.assertAlmostEqual(part.body.position.y, -1.2, places=4)
        self.r.program = 2                                                   # back to the perch
        self.pulse("start")
        self.e.run_for(3.0)
        self.assertTrue(self.r.atHome)

    def test_an_unreachable_target_faults_until_reset(self):
        self.r.program = 3
        self.r.targetX, self.r.targetY, self.r.targetZ = 9.0, 0.0, 1.0
        self.pulse("start")
        self.assertTrue(self.r.fault)
        self.assertEqual(self.r.faultCode, 1)
        self.assertFalse(self.r.ready)
        self.r.targetX = 1.0
        self.pulse("start")
        self.assertTrue(self.r.fault)                                        # still faulted
        self.pulse("faultReset")
        self.e.run_for(0.02)
        self.assertFalse(self.r.fault)
        self.assertTrue(self.r.ready)

    def test_the_io_frame_carries_commands_and_joint_feedback(self):
        inputs = [path for _bt, path in custom_component_mapping(self.c, ["r1"], direction="in")]
        outputs = [path for _bt, path in custom_component_mapping(self.c, ["r1"], direction="out")]
        for field in ("start", "enable", "hold", "faultReset", "gripperClose", "program", "speedOverride",
                      "targetX", "targetY", "targetZ", "targetYaw"):
            self.assertIn(f"r1/{field}", inputs)
        for field in ("ready", "atHome", "inPosition", "fault", "faultCode", "cycles", "tcpX", "tcpY", "tcpZ",
                      *[f"j{i}" for i in range(1, 7)]):
            self.assertIn(f"r1/{field}", outputs)


class ApiTest(unittest.TestCase):
    def test_a_script_picks_and_places_with_linear_moves(self):
        c = layout(control="plc")                       # no PLC start: only the script moves it
        e = PhysicsEngine(c)
        part = e.add_parcel(1.2, 0.0, 0.3, 0.3)
        top = heights.surface_z(c, 0) + heights.part_height(part)
        arm = e.machines.robot("r1")
        (arm.move_to(1.2, 0.0, top + 0.3, yaw=0)
            .move_to(1.2, 0.0, top, linear=True)
            .grip()
            .move_to(1.2, 0.0, top + 0.3, linear=True)
            .move_to(0.0, -1.2, top + 0.3, yaw=90)
            .move_to(0.0, -1.2, top, linear=True)
            .release()
            .home())
        self.assertTrue(arm.busy)
        e.run_for(arm.remaining() + 0.1)
        self.assertFalse(arm.busy)
        self.assertFalse(arm.holding)
        self.assertAlmostEqual(part.body.position.y, -1.2, places=4)
        self.assertAlmostEqual(math.degrees(part.body.angle) % 180, 90.0, places=3)     # turned with the tool
        self.assertTrue(c.robots[0].atHome)
        with self.assertRaises(robot6.Unreachable):
            arm.move_to(10.0, 0.0, 1.0)


if __name__ == "__main__":
    unittest.main()
