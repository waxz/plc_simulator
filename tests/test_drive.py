"""
test_drive.py - belt drive model (drive.py): ramp, lag, clamp, encoder
(plan acceptance criteria A1-A4), plus the runtime fields' save behavior.
"""
import json
import math
import unittest

from config_parser import Belt, DriveConfig, SimulatorConfig
from drive import ENCODER_MODULUS, step_drive

H = 0.002


def make_belt(speed=0.0, drive=None):
    return Belt(id="b", type="linear", x=0, y=0, length=4, beltWidth=1, rotation=0,
                speed=speed, directionAngle=0, color="#fff", drive=drive)


def run(belt, seconds, h=H):
    for _ in range(int(round(seconds / h))):
        step_drive(belt, h)


class IdealDriveTest(unittest.TestCase):
    def test_actual_follows_setpoint_every_step_without_drive_config(self):  # A3
        belt = make_belt(speed=1.5)
        step_drive(belt, H)
        self.assertEqual(belt.actualSpeed, 1.5)
        belt.speed = -0.7
        step_drive(belt, H)
        self.assertEqual(belt.actualSpeed, -0.7)


class RampTest(unittest.TestCase):
    def test_reaches_setpoint_after_dv_over_accel(self):  # A1
        belt = make_belt(speed=2.0, drive=DriveConfig(accel=2.0, decel=2.0, tau=0.0))
        run(belt, 0.998)
        self.assertLess(belt.actualSpeed, 2.0)
        run(belt, 0.004)
        self.assertAlmostEqual(belt.actualSpeed, 2.0, places=9)

    def test_ramp_slope_equals_accel(self):  # A1
        belt = make_belt(speed=2.0, drive=DriveConfig(accel=2.0, tau=0.0))
        run(belt, 0.5)
        self.assertAlmostEqual(belt.actualSpeed, 1.0, delta=0.01 * 1.0)

    def test_uses_decel_when_slowing_down(self):
        belt = make_belt(speed=2.0, drive=DriveConfig(accel=10.0, decel=1.0, tau=0.0))
        run(belt, 1.0)
        self.assertAlmostEqual(belt.actualSpeed, 2.0)
        belt.speed = 0.0
        run(belt, 1.0)
        self.assertAlmostEqual(belt.actualSpeed, 1.0, delta=0.01)

    def test_reversing_decelerates_through_zero_then_accelerates(self):
        belt = make_belt(speed=1.0, drive=DriveConfig(accel=1.0, decel=4.0, tau=0.0))
        run(belt, 1.0)
        belt.speed = -1.0
        run(belt, 0.25)  # 1.0 -> 0 at decel 4
        self.assertAlmostEqual(belt.actualSpeed, 0.0, delta=0.01)
        run(belt, 0.5)   # then 0 -> -0.5 at accel 1
        self.assertAlmostEqual(belt.actualSpeed, -0.5, delta=0.02)

    def test_speed_clamp(self):
        belt = make_belt(speed=9.0, drive=DriveConfig(accel=100.0, tau=0.0, speedMax=3.0))
        run(belt, 1.0)
        self.assertEqual(belt.actualSpeed, 3.0)


def limited(**kw):
    """A drive with a train where 1 motor rpm is 1 mm/s on the surface (gear 1,
    drum 60/pi mm; inverter up to 200 Hz, so the inverter never limits it)."""
    return DriveConfig(gearRatio=1.0, drumDiameter=0.06 / math.pi, maxFrequency=200.0, tau=0.0, **kw)


class MotorLimitTest(unittest.TestCase):
    """motorMaxSpeed (rpm) and motorMaxAccel (rpm/s): the motor's limits, through
    the train to the surface. A stop brakes at motorMaxAccel; no ramp is steeper."""

    def test_a_stop_brakes_at_the_motor_max_accel_not_decel(self):
        belt = make_belt(speed=2.0, drive=limited(accel=10.0, decel=1.0, motorMaxAccel=4000.0))
        run(belt, 1.0)
        belt.speed = 0.0
        run(belt, 0.25)                       # 2.0 - 4 m/s^2 * 0.25
        self.assertAlmostEqual(belt.actualSpeed, 1.0, delta=0.01)
        run(belt, 0.252)
        self.assertEqual(belt.actualSpeed, 0.0)

    def test_slowing_to_a_lower_speed_keeps_the_normal_ramp(self):
        belt = make_belt(speed=2.0, drive=limited(accel=10.0, decel=1.0, motorMaxAccel=4000.0))
        run(belt, 1.0)
        belt.speed = 0.5
        run(belt, 0.5)                        # 2.0 - 1 * 0.5
        self.assertAlmostEqual(belt.actualSpeed, 1.5, delta=0.01)

    def test_no_ramp_steeper_than_the_motor_max_accel(self):
        belt = make_belt(speed=2.0, drive=limited(accel=10.0, motorMaxAccel=4000.0))
        run(belt, 0.25)
        self.assertAlmostEqual(belt.actualSpeed, 1.0, delta=0.01)

    def test_no_faster_than_the_motor_max_speed(self):
        belt = make_belt(speed=2.0, drive=limited(accel=10.0, motorMaxSpeed=1200.0))
        run(belt, 1.0)
        self.assertAlmostEqual(belt.actualSpeed, 1.2, places=9)
        self.assertAlmostEqual(belt.motorSpeed, 1200.0, places=6)

    def test_a_hold_is_a_stop(self):
        belt = make_belt(speed=1.0, drive=limited(decel=0.5, motorMaxAccel=5000.0))
        run(belt, 1.0)
        belt._zone_hold = True
        run(belt, 0.202)
        self.assertEqual(belt.actualSpeed, 0.0)

    def test_without_limits_a_stop_uses_decel(self):
        belt = make_belt(speed=2.0, drive=limited(accel=10.0, decel=1.0))
        run(belt, 1.0)
        belt.speed = 0.0
        run(belt, 1.0)
        self.assertAlmostEqual(belt.actualSpeed, 1.0, delta=0.01)

    def test_without_a_train_rpm_limits_do_nothing(self):
        belt = make_belt(speed=2.0, drive=DriveConfig(accel=10.0, decel=1.0, tau=0.0,
                                                      motorMaxSpeed=10.0, motorMaxAccel=10.0))
        run(belt, 1.0)
        self.assertAlmostEqual(belt.actualSpeed, 2.0)


class LagTest(unittest.TestCase):
    def test_first_order_lag_reaches_63_percent_at_tau(self):  # A2
        belt = make_belt(speed=1.0, drive=DriveConfig(accel=1e9, decel=1e9, tau=0.1))
        run(belt, 0.1)
        self.assertAlmostEqual(belt.actualSpeed, 1.0 - math.exp(-1.0), delta=0.02 * 0.632)


class EncoderTest(unittest.TestCase):
    def test_counts_match_distance_travelled(self):  # A4
        belt = make_belt(speed=1.25, drive=DriveConfig(accel=1e9, tau=0.0, countsPerM=1000.0))
        run(belt, 2.0)
        self.assertAlmostEqual(belt.encoderCount, 2500, delta=1)

    def test_fractional_counts_are_not_lost_at_low_speed(self):
        # 0.01 m/s * 0.002 s * 1000 counts/m = 0.02 counts per step.
        belt = make_belt(speed=0.01, drive=DriveConfig(accel=1e9, tau=0.0, countsPerM=1000.0))
        run(belt, 10.0)
        self.assertAlmostEqual(belt.encoderCount, 100, delta=1)

    def test_wraps_like_a_uint32_counter(self):  # A4
        belt = make_belt(speed=1.0, drive=DriveConfig(accel=1e9, tau=0.0, countsPerM=1000.0))
        belt.encoderCount = ENCODER_MODULUS - 5
        run(belt, 0.01)  # +10 counts
        self.assertEqual(belt.encoderCount, 5)

    def test_reverse_running_counts_down_and_wraps_below_zero(self):
        belt = make_belt(speed=-1.0, drive=DriveConfig(accel=1e9, tau=0.0, countsPerM=1000.0))
        run(belt, 0.01)
        self.assertEqual(belt.encoderCount, ENCODER_MODULUS - 10)


class RuntimeFieldsTest(unittest.TestCase):
    def test_runtime_outputs_are_not_saved_into_layout_files(self):
        belt = make_belt(speed=1.0, drive=DriveConfig())
        belt.actualSpeed = 0.5
        belt.encoderCount = 42
        config = SimulatorConfig(belts=[belt])
        config.simTimeNs = 123
        saved = json.loads(config.model_dump_json())
        self.assertNotIn("actualSpeed", saved["belts"][0])
        self.assertNotIn("encoderCount", saved["belts"][0])
        self.assertNotIn("simTimeNs", saved)
        self.assertEqual(saved["belts"][0]["drive"]["accel"], 2.0)

    def test_old_layout_files_without_drive_fields_still_load(self):
        data = {"belts": [{"id": "b", "type": "linear", "x": 0, "y": 0, "length": 4, "beltWidth": 1,
                           "rotation": 0, "speed": 1, "directionAngle": 0, "color": "#fff"}]}
        config = SimulatorConfig(**data)
        self.assertIsNone(config.belts[0].drive)
        self.assertEqual(config.tractionModel, "kinematic")


if __name__ == "__main__":
    unittest.main()
