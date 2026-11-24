"""
test_friction_traction.py - friction-based belt traction (traction.py), the
fixed-step physics loop, and per-step sensor evaluation (plan acceptance
criteria B1-B6, C1, D1). Every number is checked against the closed-form
physics it should reproduce, not against recorded output.
"""
import math
import unittest

from config_parser import Belt, DriveConfig, Sensor, SimulatorConfig
from physics import PhysicsEngine

G = 9.81


def linear_belt(id, x, length, speed, mu=0.3, y=0.0, width=1.0, drive=None):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=width, rotation=0,
                speed=speed, directionAngle=0, color="#fff", friction=mu, drive=drive)


def engine(belts, sensors=(), dt=0.002, traction="friction"):
    config = SimulatorConfig(belts=list(belts), sensors=list(sensors), tractionModel=traction,
                             physicsDt=dt, floorFriction=0.6, useSurfaceVelocity=True)
    return PhysicsEngine(config)


class AccelerationAndStoppingTest(unittest.TestCase):
    def test_parcel_at_rest_accelerates_at_mu_g(self):  # B1
        e = engine([linear_belt("a", 0, 40, 1.0, mu=0.3)])
        p = e.add_parcel(-15, 0, 0.4, 0.4)
        e.run_for(0.1)
        self.assertAlmostEqual(p.body.velocity.x / 0.1, 0.3 * G, delta=0.05 * 0.3 * G)

    def test_parcel_reaches_belt_speed_after_v_over_mu_g(self):  # B1
        e = engine([linear_belt("a", 0, 40, 1.0, mu=0.3)])
        p = e.add_parcel(-15, 0, 0.4, 0.4)
        e.run_for(1.0 / (0.3 * G) - 0.02)
        self.assertLess(p.body.velocity.x, 0.99)
        e.run_for(0.04)
        self.assertAlmostEqual(p.body.velocity.x, 1.0, delta=0.01)

    def test_acceleration_does_not_depend_on_parcel_mass(self):
        for mass in (0.5, 5.0):
            with self.subTest(mass=mass):
                e = engine([linear_belt("a", 0, 40, 1.0, mu=0.3)])
                p = e.add_parcel(-15, 0, 0.4, 0.4, mass=mass)
                e.run_for(0.1)
                self.assertAlmostEqual(p.body.velocity.x / 0.1, 0.3 * G, delta=0.05 * 0.3 * G)

    def test_stopping_distance_on_a_stopped_belt(self):  # B2
        e = engine([linear_belt("a", 0, 40, 0.0, mu=0.3)])
        p = e.add_parcel(-15, 0, 0.4, 0.4)
        p.body.velocity = (1.0, 0.0)
        e.run_for(1.0)
        expected = 1.0 / (2 * 0.3 * G)
        self.assertAlmostEqual(p.body.position.x + 15, expected, delta=0.05 * expected)
        self.assertAlmostEqual(p.body.velocity.x, 0.0, delta=1e-3)

    def test_parcel_tracks_a_ramping_belt_drive(self):  # B6
        drive = DriveConfig(accel=1.0, decel=1.0, tau=0.05)
        e = engine([linear_belt("a", 0, 40, 2.0, mu=0.8, drive=drive)])
        p = e.add_parcel(-15, 0, 0.4, 0.4)
        e.run_for(3.0)
        belt = e.config.belts[0]
        self.assertAlmostEqual(belt.actualSpeed, 2.0, delta=1e-3)
        self.assertAlmostEqual(p.body.velocity.x, belt.actualSpeed, delta=0.01 * 2.0)


class TransferTest(unittest.TestCase):
    def test_carrier_velocity_is_the_friction_weighted_mix_across_a_belt_joint(self):  # B3
        e = engine([linear_belt("a", -5, 10, 1.0, mu=0.5), linear_belt("b", 5, 10, 0.0, mu=0.5)])
        # Center just on belt b: 3 of 9 samples on a (1 m/s), 6 on b (0 m/s).
        p = e.add_parcel(0.001, 0, 0.6, 0.6)
        e.step(0.002)
        carrier = p.traction["carrier"]
        self.assertAlmostEqual(carrier.velocity.x, 1.0 / 3.0, delta=1e-6)
        self.assertEqual(p.belt_id, "b")

    def test_side_by_side_belts_at_different_speeds_yaw_the_parcel(self):  # B3
        # Left half (y > 0) on a fast belt, right half (y < 0) on a stopped one:
        # the fast side drags ahead, so the parcel turns clockwise (omega < 0).
        e = engine([linear_belt("fast", 0, 20, 1.0, mu=0.5, y=0.5),
                    linear_belt("slow", 0, 20, 0.0, mu=0.5, y=-0.5)])
        p = e.add_parcel(-5, 0.0, 0.4, 0.4)
        e.step(0.002)
        self.assertLess(p.traction["carrier"].angular_velocity, 0.0)
        e.run_for(0.5)
        self.assertLess(p.body.angle, 0.0)

    def test_parcel_follows_a_curved_belt(self):  # B4
        curve = Belt(id="c", type="curved", shape="arc", x=0, y=0, radius=2.0, beltWidth=1.0, rotation=0,
                     startAngle=270, endAngle=360, speed=1.0, directionAngle=90, color="#fff", friction=0.5)
        e = engine([curve])
        p = e.add_parcel(0.0, -2.0, 0.4, 0.4)
        p.body.velocity = (1.0, 0.0)
        e.run_for(1.5)
        self.assertAlmostEqual(p.body.angular_velocity, 1.0 / 2.0, delta=0.01)  # mid-arc: omega = v / r
        e.run_for(math.pi - 0.1 - 1.5)  # quarter arc of r=2 is pi metres at 1 m/s
        pos = p.body.position
        self.assertAlmostEqual(pos.length, 2.0, delta=0.01 * 2.0)
        self.assertGreater(math.degrees(math.atan2(pos.y, pos.x)), -10.0, "parcel didn't travel the arc")
        # Heading stays tangent to the arc at the parcel's actual position.
        tangent = math.degrees(math.atan2(pos.y, pos.x)) + 90.0
        self.assertAlmostEqual(math.degrees(p.body.angle), tangent, delta=2.0)


class AccumulationTest(unittest.TestCase):
    def test_queue_against_a_stopped_belt_settles_where_forces_balance(self):  # B5
        # Equal mu: parcels on the stopped belt can hold back up to mu*m*g each,
        # parcels on the moving belt push with mu*m*g each. The queue keeps
        # sliding while push exceeds what the stopped belt can hold, so it
        # comes to rest with at least half its footprint on the stopped belt -
        # a bit more, since momentum carries it past the balance point.
        e = engine([linear_belt("up", -5, 10, 1.0, mu=0.5), linear_belt("dn", 5, 10, 0.0, mu=0.5)])
        parcels = [e.add_parcel(0.5 - 0.41 * i, 0, 0.4, 0.4) for i in range(5)]
        e.run_for(5.0)
        xs = [p.body.position.x for p in parcels]
        for a, b in zip(xs, xs[1:]):
            self.assertGreater(a - b, 0.399, "parcels overlap")
        self.assertTrue(all(abs(p.body.velocity.x) < 0.01 for p in parcels), "queue still moving")
        footprint_on_stopped = sum(min(max(x + 0.2, 0.0), 0.4) / 0.4 for x in xs)
        self.assertGreaterEqual(footprint_on_stopped, 2.5 - 0.15)
        self.assertLess(footprint_on_stopped, 3.5)


class FixedStepTest(unittest.TestCase):
    def test_update_runs_whole_steps_and_carries_the_remainder(self):
        e = engine([linear_belt("a", 0, 10, 1.0)], dt=0.004)
        e.update(0.010)  # 2 steps, 0.002 carried
        self.assertEqual(e.step_count, 2)
        e.update(0.002)  # 0.004 accumulated -> 1 more step
        self.assertEqual(e.step_count, 3)
        self.assertAlmostEqual(e.sim_time, 0.012)
        self.assertEqual(e.config.simTimeNs, 12_000_000)

    def test_backlog_beyond_the_step_cap_is_dropped_and_reported(self):
        e = engine([linear_belt("a", 0, 10, 1.0)], dt=0.004)
        e.config.maxStepsPerAdvance = 10
        e.update(1.0)
        self.assertEqual(e.step_count, 10)
        self.assertAlmostEqual(e.dropped_time, 1.0 - 0.04, places=6)

    def test_result_does_not_depend_on_how_real_time_is_sliced(self):
        positions = []
        for frame_dt in (1 / 60, 1 / 25, 0.002):
            e = engine([linear_belt("a", 0, 40, 1.0, mu=0.3)])
            p = e.add_parcel(-15, 0, 0.4, 0.4)
            while e.sim_time < 1.0 - 1e-9:
                e.update(min(frame_dt, 1.0 - e.sim_time))
            positions.append(p.body.position.x)
        self.assertAlmostEqual(max(positions) - min(positions), 0.0, delta=1e-6)


class SensorPerStepTest(unittest.TestCase):
    def test_short_pulse_between_frames_is_counted(self):  # C1
        # 3 m/s, 0.1 m parcel, 0.02 m sensor: blocked for 40 ms, which fits
        # inside a single 80 ms frame. Sampling once per frame would usually
        # miss it; per-step evaluation must still count exactly one edge.
        sensor = Sensor(id="s", x=0, y=0, width=0.02, height=1.0, rotation=0, label="S", color="#f00")
        e = engine([linear_belt("a", 0, 40, 3.0, mu=0.8)], sensors=[sensor])
        p = e.add_parcel(-1.0, 0, 0.1, 0.1)
        p.body.velocity = (3.0, 0.0)
        for _ in range(8):
            e.update(0.08)  # 40 steps per frame, under maxStepsPerAdvance
        self.assertGreater(p.body.position.x, 0.5, "parcel didn't pass the sensor")
        self.assertEqual(sensor.risingEdges, 1)
        self.assertEqual(sensor.sensor_states, "0")

    def test_edge_time_error_is_at_most_one_step(self):  # C1
        sensor = Sensor(id="s", x=0, y=0, width=0.02, height=1.0, rotation=0, label="S", color="#f00")
        e = engine([linear_belt("a", 0, 40, 1.0, mu=0.8)], sensors=[sensor])
        p = e.add_parcel(-1.0, 0, 0.2, 0.2)
        p.body.velocity = (1.0, 0.0)
        # Leading edge (x + 0.1) reaches the sensor's near face (-0.01) at t = 0.89 s.
        while sensor.risingEdges == 0:
            e.step(0.002)
        self.assertAlmostEqual(e.sim_time, 0.89, delta=0.002 + 1e-9)


class KinematicModeUnchangedTest(unittest.TestCase):
    def test_kinematic_parcels_get_no_carrier_and_follow_the_belt(self):  # D1
        e = engine([linear_belt("a", 0, 40, 1.0)], traction="kinematic")
        p = e.add_parcel(-15, 0, 0.4, 0.4)
        self.assertIsNone(p.traction)
        e.run_for(0.5)
        self.assertAlmostEqual(p.body.velocity.x, 1.0, places=6)

    def test_removed_friction_parcels_take_their_carrier_and_joints_with_them(self):
        e = engine([linear_belt("a", 0, 40, 1.0)])
        e.add_parcel(-15, 0, 0.4, 0.4)
        e.add_parcel(-14, 0, 0.4, 0.4)
        e.clear_all_parcels()
        self.assertEqual(len(e.space.bodies), 0)
        self.assertEqual(len(e.space.constraints), 0)


if __name__ == "__main__":
    unittest.main()
