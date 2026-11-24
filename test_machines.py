"""
test_machines.py - pallets, stoppers, robot arms and elevators in the
physics engine (machines.py): a pallet riding the belts and carrying a
load to a sink, a stopper holding it (auto and PLC), a robot moving a part
onto a pallet, and an elevator carrying a parcel up a level.
"""
import math
import unittest

from config_parser import Belt, Elevator, Pallet, RobotArm, Sensor, SimulatorConfig, Sink, Source, Stopper
from physics import PhysicsEngine


def belt(id, x, y, length, width=1.0, rotation=0.0, speed=1.0, level=0):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=width, rotation=rotation, speed=speed,
                directionAngle=0.0, color="#888888", level=level)


def config(**lists) -> SimulatorConfig:
    return SimulatorConfig(useSurfaceVelocity=True, **lists)


def run_under_plc(engine, plc, seconds, scan=0.02):
    """The simulation with the PLC program (plc_logic, what plc_mock.py runs)
    scanning every `scan` s: machines follow the PLC."""
    end = engine.sim_time + seconds
    while engine.sim_time < end - 1e-9:
        plc.step(now=engine.sim_time)
        engine.run_for(scan)


def plc_for(config):
    from plc_logic import make_controller
    return make_controller(config, autostart=True)


def pallet_of(engine, pallet_id="pal_1"):
    return next(p for p in engine.parcels if p.id == pallet_id)


class PalletTest(unittest.TestCase):
    def test_a_pallet_rides_the_belt_and_a_sink_takes_only_its_load(self):
        c = config(belts=[belt("b1", 5, 0, 10)],
                   pallets=[Pallet(id="pal_1", x=1.0, y=0.0)],
                   sources=[Source(id="src", x=1.0, y=0.0, rotation=0.0, interval=0.1, minWidth=0.3,
                                   maxWidth=0.3, minHeight=0.3, maxHeight=0.3, label="P")],
                   sinks=[Sink(id="sink", x=9.0, y=0.0, width=1.0, height=2.0, label="out")])
        e = PhysicsEngine(c)
        e.config.sources[0].interval = 0.05
        e.run_for(0.1)                                          # the source fires over the pallet: a load
        pallet = pallet_of(e)
        self.assertEqual(len(pallet.loads), 1)
        self.assertEqual(c.pallets[0].loadCount, 1)
        e.config.sources.clear()
        e.run_for(9.0)
        pallet = pallet_of(e)                                   # still there, past the sink
        self.assertGreater(pallet.body.position.x, 8.0)
        self.assertEqual(pallet.loads, [])
        self.assertEqual(e.delivered.get("sink"), 1)
        views = e.parcel_views()
        self.assertEqual([v.kind for v in views], ["pallet"])

    def test_the_load_follows_and_reset_brings_the_pallet_back(self):
        c = config(belts=[belt("b1", 5, 0, 10)], pallets=[Pallet(id="pal_1", x=1.0, y=0.0, capacity=2)])
        e = PhysicsEngine(c)
        part = e.add_parcel(1.0, 0.0, 0.2, 0.2)
        e.lift_out(part)
        e.machines.put_on_pallet(part, pallet_of(e))
        e.run_for(2.0)
        pallet = pallet_of(e)
        self.assertAlmostEqual(part.body.position.y, pallet.body.position.y, places=6)
        self.assertGreater(part.body.position.x, 2.0)
        self.assertEqual([v.kind for v in e.parcel_views()], ["pallet", "parcel"])
        e.clear_parcels()                                       # Clear Parcels: the pallet stays, empty
        self.assertEqual(pallet_of(e).loads, [])
        e.reset()
        self.assertAlmostEqual(pallet_of(e).body.position.x, 1.0)


class StopperTest(unittest.TestCase):
    def layout(self, **stopper):
        return config(belts=[belt("b1", 5, 0, 10)], pallets=[Pallet(id="pal_1", x=1.0, y=0.0)],
                      stoppers=[Stopper(id="st_1", x=5.0, y=0.0, **stopper)])

    def test_plc_stopper_holds_until_released(self):
        e = PhysicsEngine(self.layout(control="plc"))
        stopper = e.config.stoppers[0]
        e.run_for(6.0)
        pallet = pallet_of(e)
        front = pallet.body.position.x + 0.4
        self.assertLess(front, 5.0)                             # held at the bar
        self.assertGreater(front, 4.8)
        self.assertTrue(stopper.occupied and stopper.isUp)
        stopper.release = True
        e.run_for(0.2)
        self.assertFalse(stopper.isUp)                          # after actuationTime
        e.run_for(2.0)
        self.assertGreater(pallet_of(e).body.position.x, 5.5)
        stopper.release = False
        e.run_for(3.0)
        self.assertTrue(stopper.isUp)                           # back up once the bar is clear

    def test_the_plc_program_holds_then_lets_go(self):
        c = self.layout(holdTime=1.0, releaseTime=1.0)
        e, plc = PhysicsEngine(c), plc_for(c)
        run_under_plc(e, plc, 5.0)
        self.assertLess(pallet_of(e).body.position.x, 5.0)
        run_under_plc(e, plc, 3.0)
        self.assertGreater(pallet_of(e).body.position.x, 6.0)


class RobotTest(unittest.TestCase):
    def test_the_plc_program_has_the_robot_move_a_part_onto_a_pallet(self):
        c = config(pallets=[Pallet(id="pal_1", x=3.0, y=1.0)],
                   robots=[RobotArm(id="r1", x=2.0, y=0.0, pickX=1.0, pickY=1.0, placeX=3.0, placeY=1.0,
                                    cycleTime=1.5)])
        e, plc = PhysicsEngine(c), plc_for(c)
        part = e.add_parcel(1.0, 1.0, 0.3, 0.3)
        robot = c.robots[0]
        run_under_plc(e, plc, 0.1)                              # enabled, then started
        self.assertTrue(robot.busy)
        run_under_plc(e, plc, 0.6)                              # past the pick: in the gripper
        self.assertTrue(robot.holding)
        self.assertNotIn(part, e.parcels)
        run_under_plc(e, plc, 1.2)
        self.assertFalse(robot.busy)
        self.assertEqual(robot.cycles, 1)
        self.assertEqual(pallet_of(e).loads, [part])

    def test_plc_robot_waits_for_start_and_drops_on_the_floor(self):
        c = config(robots=[RobotArm(id="r1", x=0.0, y=0.0, pickX=1.0, pickY=0.0, placeX=0.0, placeY=1.0,
                                    cycleTime=0.9, control="plc")])
        e = PhysicsEngine(c)
        part = e.add_parcel(1.0, 0.0, 0.3, 0.3)
        robot = c.robots[0]
        e.run_for(1.0)
        self.assertTrue(robot.partAtPick)
        self.assertFalse(robot.busy)
        robot.start = True
        e.run_for(1.0)
        self.assertEqual(robot.cycles, 1)
        self.assertIn(part, e.parcels)
        self.assertAlmostEqual(part.body.position.x, 0.0, places=3)
        self.assertAlmostEqual(part.body.position.y, 1.0, places=3)


class ElevatorTest(unittest.TestCase):
    def test_the_plc_program_lifts_a_parcel_to_the_upper_belt(self):
        c = config(belts=[belt("in", 2, 0, 4), belt("lift", 4.7, 0, 1.4), belt("up", 7.4, 0, 4, level=1),
                          belt("under", 7.4, 0, 4, speed=0.0)],
                   elevators=[Elevator(id="el_1", x=4.7, y=0.0, width=1.4, height=1.0, beltId="lift",
                                       levelHeight=1.0, liftSpeed=1.0)],
                   sensors=[Sensor(id="s_low", x=7.0, y=0.0, width=0.1, height=1.0, rotation=0.0, label="low",
                                   color="#f00"),
                            Sensor(id="s_up", x=7.0, y=0.0, width=0.1, height=1.0, rotation=0.0, label="up",
                                   color="#f00", level=1)])
        e, plc = PhysicsEngine(c), plc_for(c)
        self.assertTrue(e.multi_level)
        part = e.add_parcel(1.0, 0.0, 0.3, 0.3)
        elevator = c.elevators[0]
        seen_moving = False
        for _ in range(400):
            run_under_plc(e, plc, 0.02)
            seen_moving |= elevator.moving
        self.assertTrue(seen_moving)
        self.assertEqual(part.level, 1)
        self.assertGreater(part.body.position.x, 7.5)           # carried on along the upper belt
        self.assertGreater(c.sensors[1].risingEdges, 0)
        self.assertEqual(c.sensors[0].risingEdges, 0)           # the floor below never saw it
        run_under_plc(e, plc, 4.0)
        self.assertEqual(elevator.level, 0)                     # back down for the next one


if __name__ == "__main__":
    unittest.main()


class PlcControlTest(unittest.TestCase):
    def test_stoppers_stay_up_while_the_line_is_stopped(self):
        from config_parser import load_config
        from plc_logic import make_controller
        c = load_config("machines_demo.json")
        for stopper in c.stoppers:
            stopper.control = "plc"
        e = PhysicsEngine(c)
        plc = make_controller(c, autostart=False)
        while e.sim_time < 20.0:
            plc.step(now=e.sim_time)
            e.run_for(0.05)
        self.assertTrue(all(s.isUp and not s.release for s in c.stoppers))


class WindowTest(unittest.TestCase):
    def test_the_simulator_window_draws_and_inspects_machines(self):
        import os
        import tempfile
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ["CONVEYOR_SIM_SETTINGS"] = os.path.join(tempfile.mkdtemp(), "settings.json")
        from config_parser import load_config
        from visualizer import Renderer
        c = load_config("machines_demo.json")
        e = PhysicsEngine(c)
        r = Renderer(c, headless=False)
        self.addCleanup(r.quit)
        r.physics = e
        e.run_for(17.6)                                         # the lift on its way, the robot working
        r.render(e)
        robot = c.robots[0]
        self.assertIs(r._hit_test(robot.x, robot.y), robot)
        self.assertIs(r._hit_test(c.elevators[0].x, c.elevators[0].y + 0.4), c.elevators[0])
        r.editor_ui.show_props_for(robot)
        self.assertIn("Cycles", r.editor_ui._live_values(robot))
        r.editor_ui.open_manager_panel()                        # the Components tree: a folder per kind
        folders = set(r.editor_ui.manager_folder_rows.values())
        self.assertLessEqual({"pallet", "stopper", "robot", "elevator"}, folders)


class PickSensorTest(unittest.TestCase):
    """A robot's pick trigger is an external IR sensor at its pick point
    (pickSensorId), wired to the PLC."""

    def layout(self, control="plc"):
        return config(belts=[belt("feed", 1.0, 0.0, 2.0)],
                      sensors=[Sensor(id="s_pick", x=1.8, y=0.0, width=0.1, height=1.0, rotation=0.0,
                                      label="pick", color="#f00")],
                      robots=[RobotArm(id="r1", x=1.8, y=1.2, pickX=1.8, pickY=0.0, placeX=2.6, placeY=1.2,
                                       cycleTime=1.5, control=control, pickSensorId="s_pick")])

    def test_the_plc_starts_the_robot_on_its_pick_sensor_and_it_is_no_jam(self):
        from plc_logic import make_controller, part_at_pick
        c = self.layout()
        plc = make_controller(c, autostart=True, jam_time=1.0)
        robot = c.robots[0]
        robot.partAtPick = robot.ready = robot.placeReady = True   # the robot's own view does not count
        plc.step(now=0.0)
        self.assertFalse(part_at_pick(c, robot))
        self.assertFalse(robot.start)
        c.sensors[0].sensor_states = True           # the IR sensor sees the part
        for i in range(1, 60):
            plc.step(now=0.1 * i)
        self.assertTrue(part_at_pick(c, robot))
        self.assertTrue(robot.start)
        self.assertIsNone(plc.jammed)               # waiting for the robot is not a jam
