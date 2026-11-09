"""
test_drivetrain.py - belts and roller conveyors driven like real ones
(drivetrain.py): a frequency inverter commanded in Hz, a motor, a gearbox, a
chain, a drum or rollers. The surface speed follows from the motor speed,
the reduction and the drum / roller diameter; the PLC commands Hz, gets the
inverter's output frequency and the motor speed back, and the mock PLC
converts its design speeds to Hz with the same mechanical data.
"""
import copy
import math
import unittest

import drivetrain
import st_export
from config_parser import Belt, DriveConfig, load_config
from drive import step_drive
from mqtt_message_binding import ByteType, ComponentMessage, belt_fields, custom_component_mapping
from plc_logic import LineController

H = 0.002


def make_belt(speed=1.0, command="frequency", drive=None, surface="belt"):
    return Belt(id="b", type="linear", x=0, y=0, length=4, beltWidth=1, rotation=0, speed=speed,
                directionAngle=0, color="#fff", commandFormat=command, drive=drive, surface=surface)


def run(belt, seconds, h=H):
    for _ in range(int(round(seconds / h))):
        step_drive(belt, h)


# A 4-pole motor (1450 rpm at 50 Hz), gearbox i = 20, a 100 mm drum:
# 1450 / 20 = 72.5 rpm at the drum, * pi * 0.1 / 60 = 0.37961 m/s at 50 Hz.
V50 = 1450.0 / 20.0 * math.pi * 0.1 / 60.0


class TrainMathTest(unittest.TestCase):
    def test_surface_speed_from_motor_gearbox_and_drum(self):
        t = drivetrain.train_of(make_belt(drive=DriveConfig(gearRatio=20.0, drumDiameter=0.1)))
        self.assertAlmostEqual(t.motor_speed(50.0), 1450.0)
        self.assertAlmostEqual(t.output_speed(50.0), 72.5)
        self.assertAlmostEqual(t.speed_for_frequency(50.0), V50, places=6)
        self.assertAlmostEqual(t.speed_for_frequency(25.0), V50 / 2, places=6)        # proportional
        self.assertAlmostEqual(t.frequency_for_speed(t.speed_for_frequency(37.3)), 37.3, places=9)
        self.assertAlmostEqual(t.speed_for_frequency(-50.0), -V50, places=6)          # sign follows

    def test_chain_and_roller_diameter(self):
        # a roller conveyor: gearbox i = 10, chain 2:1, 50 mm rollers
        t = drivetrain.train_of(make_belt(surface="roller", drive=DriveConfig(
            gearRatio=10.0, transmissionRatio=2.0, drumDiameter=0.05)))
        self.assertAlmostEqual(t.speed_for_frequency(50.0), 1450.0 / 20.0 * math.pi * 0.05 / 60.0, places=6)

    def test_another_motor(self):
        # a 2-pole 60 Hz motor: 3500 rpm at 60 Hz
        t = drivetrain.train_of(make_belt(drive=DriveConfig(motorRatedSpeed=3500.0, motorRatedFrequency=60.0,
                                                            gearRatio=35.0, drumDiameter=0.2)))
        self.assertAlmostEqual(t.speed_for_frequency(60.0), 100.0 * math.pi * 0.2 / 60.0, places=6)

    def test_max_speed_is_at_the_max_frequency(self):
        t = drivetrain.train_of(make_belt(drive=DriveConfig(gearRatio=20.0, drumDiameter=0.1, maxFrequency=87.0)))
        self.assertAlmostEqual(t.max_speed, V50 * 87.0 / 50.0, places=6)

    def test_a_frequency_belt_without_train_data_runs_its_design_speed_at_rated_frequency(self):
        t = drivetrain.train_of(make_belt(speed=1.2))
        self.assertAlmostEqual(t.drum_diameter, drivetrain.DEFAULT_DRUM_DIAMETER)
        self.assertAlmostEqual(t.speed_for_frequency(50.0), 1.2, places=9)
        # only the gear ratio given: the drum diameter is what gives 1.2 m/s at 50 Hz
        t = drivetrain.train_of(make_belt(speed=1.2, drive=DriveConfig(gearRatio=15.0)))
        self.assertAlmostEqual(t.gear_ratio, 15.0)
        self.assertAlmostEqual(t.speed_for_frequency(50.0), 1.2, places=9)
        # no design speed: 1 m/s
        self.assertAlmostEqual(drivetrain.train_of(make_belt(speed=0.0)).speed_for_frequency(50.0), 1.0, places=9)

    def test_belts_commanded_in_m_per_s_have_no_train_unless_configured(self):
        self.assertIsNone(drivetrain.train_of(make_belt(command="speed")))
        self.assertIsNone(drivetrain.train_of(make_belt(command="vfd", drive=DriveConfig())))
        self.assertIsNotNone(drivetrain.train_of(make_belt(command="vfd", drive=DriveConfig(gearRatio=20.0,
                                                                                           drumDiameter=0.1))))


class FrequencyDriveTest(unittest.TestCase):
    def belt(self, **drive):
        b = make_belt(drive=DriveConfig(gearRatio=20.0, drumDiameter=0.1, tau=0.0, accel=1e9, decel=1e9, **drive))
        b.run = True
        return b

    def test_the_commanded_frequency_sets_the_surface_speed(self):
        b = self.belt()
        b.frequency = 40.0
        run(b, 0.1)
        self.assertAlmostEqual(b.actualSpeed, V50 * 0.8, places=6)
        self.assertAlmostEqual(b.outputFrequency, 40.0, places=6)
        self.assertAlmostEqual(b.motorSpeed, 1160.0, places=3)
        self.assertTrue(b.running and b.atSpeed)

    def test_reverse_and_run_off(self):
        b = self.belt()
        b.frequency, b.reverse = 50.0, True
        run(b, 0.1)
        self.assertAlmostEqual(b.actualSpeed, -V50, places=6)
        self.assertAlmostEqual(b.motorSpeed, -1450.0, places=3)
        self.assertTrue(b.reversing)
        b.run = False
        run(b, 0.1)
        self.assertEqual((b.actualSpeed, b.outputFrequency, b.motorSpeed), (0.0, 0.0, 0.0))

    def test_the_inverter_does_not_exceed_its_max_frequency(self):
        b = self.belt(maxFrequency=60.0)
        b.frequency = 120.0
        run(b, 0.1)
        self.assertAlmostEqual(b.outputFrequency, 60.0, places=6)
        self.assertAlmostEqual(b.actualSpeed, V50 * 1.2, places=6)

    def test_ramp_times_are_seconds_from_zero_to_rated_frequency(self):
        b = make_belt(drive=DriveConfig(gearRatio=20.0, drumDiameter=0.1, tau=0.0, rampUpTime=2.0, rampDownTime=4.0))
        b.run, b.frequency = True, 50.0
        run(b, 1.0)
        self.assertAlmostEqual(b.outputFrequency, 25.0, delta=0.05)        # halfway after half the ramp
        run(b, 1.1)
        self.assertAlmostEqual(b.outputFrequency, 50.0, places=6)
        b.frequency = 0.0
        run(b, 2.0)
        self.assertAlmostEqual(b.outputFrequency, 25.0, delta=0.05)        # the longer ramp down

    def test_motor_encoder_counts_motor_revolutions(self):
        b = self.belt(encoderCountsPerRev=1024)
        b.frequency = 50.0
        run(b, 0.002)                                    # reach speed in the first step
        start = b.encoderCount
        run(b, 1.0)
        self.assertAlmostEqual(b.encoderCount - start, 1450.0 / 60.0 * 1024, delta=2)

    def test_without_a_drive_config_the_speed_is_immediate(self):
        b = make_belt(speed=1.2)                         # derived train, ideal drive
        b.run, b.frequency = True, 25.0
        step_drive(b, H)
        self.assertAlmostEqual(b.actualSpeed, 0.6, places=9)
        self.assertAlmostEqual(b.motorSpeed, 725.0, places=6)

    def test_a_fault_stops_it(self):
        b = self.belt()
        b.frequency = 50.0
        run(b, 0.1)
        b.fault = True
        run(b, 0.002)
        self.assertEqual(b.outputFrequency, 0.0)        # the inverter output is off at once
        run(b, 4.0)
        self.assertLess(abs(b.actualSpeed), 0.01)        # the motor coasts down


class ExchangeTest(unittest.TestCase):
    def test_the_command_and_the_feedback_on_the_wire(self):
        b = make_belt()
        self.assertEqual(belt_fields(b, "in"), ([(ByteType.BIT, "run"), (ByteType.BIT, "reverse")],
                                                [(ByteType.FLOAT, "frequency")]))
        bits, values = belt_fields(b, "out")
        self.assertEqual([n for _t, n in bits], ["running", "atSpeed", "reversing", "fault"])
        self.assertEqual([n for _t, n in values], ["actualSpeed", "encoderCount", "outputFrequency", "motorSpeed"])
        # the other formats are unchanged
        self.assertEqual([n for _t, n in belt_fields(make_belt(command="vfd"), "in")[1]], ["speed"])
        self.assertEqual([n for _t, n in belt_fields(make_belt(command="speed"), "out")[1]],
                         ["actualSpeed", "encoderCount"])

    def test_the_plc_side_types(self):
        b = make_belt()
        self.assertEqual(st_export.device_type(b, "in"), "BeltFreq_Command")
        self.assertEqual(st_export.device_type(b, "out"), "BeltFreq_Feedback")
        self.assertEqual(st_export.device_type(make_belt(command="vfd"), "in"), "BeltVfd_Command")
        self.assertEqual(st_export.device_type(make_belt(command="speed"), "out"), "BeltDrive_Feedback")

    def test_hz_from_the_plc_drive_the_simulated_belt(self):
        """A PLC packs run + Hz; the simulator unpacks them, its drive train
        turns them into surface speed, and the feedback packs back."""
        plant = load_config("test_config.json")
        belt = plant.belts[0]
        belt.commandFormat = "frequency"
        belt.drive = DriveConfig(gearRatio=20.0, drumDiameter=0.1, tau=0.0, accel=1e9, decel=1e9)
        plc = copy.deepcopy(plant)
        command = custom_component_mapping(plant, [belt.id], direction="in")
        plc.belts[0].run, plc.belts[0].frequency = True, 30.0
        payload = ComponentMessage(command).serialize(plc)
        ComponentMessage(command).deserialize(payload, plant)
        self.assertEqual((belt.run, belt.frequency), (True, 30.0))
        run(belt, 0.1)
        self.assertAlmostEqual(belt.actualSpeed, V50 * 0.6, places=5)
        feedback = custom_component_mapping(plant, [belt.id], direction="out")
        ComponentMessage(feedback).deserialize(ComponentMessage(feedback).serialize(plant), plc)
        self.assertAlmostEqual(plc.belts[0].outputFrequency, 30.0, places=4)
        self.assertAlmostEqual(plc.belts[0].motorSpeed, 870.0, places=2)


class MockPlcTest(unittest.TestCase):
    def test_the_mock_plc_commands_hz_that_give_the_design_speed(self):
        config = load_config("test_config.json")
        line_belts = [b for b in config.belts if b.id.startswith("belt_1_1_")]
        for i, b in enumerate(line_belts):
            b.commandFormat = "frequency"
            b.speed = 0.5 + 0.1 * i                     # each its own design speed ...
            b.drive = DriveConfig(gearRatio=8.0 - 0.5 * i, drumDiameter=0.1, tau=0.0, accel=1e9, decel=1e9)
            self.assertLess(b.speed, drivetrain.train_of(b).max_speed)    # ... below 50 Hz
        controller = LineController(config)
        start = next(btn for btn in config.buttons if btn.label == "L1 START")
        import devices
        devices.press(start)
        controller.step(0.05)
        devices.release(start)
        controller.step(0.10)
        for b in line_belts:
            run(b, 0.05)
        for b in line_belts:
            self.assertTrue(b.run, b.id)
            self.assertAlmostEqual(b.frequency, drivetrain.train_of(b).frequency_for_speed(b.speed), places=9)
            self.assertAlmostEqual(b.actualSpeed, b.speed, places=6, msg=b.id)


    def test_a_design_speed_beyond_the_max_frequency_is_capped_by_the_inverter(self):
        config = load_config("test_config.json")
        belt = next(b for b in config.belts if b.id.startswith("belt_1_1_"))
        belt.commandFormat, belt.speed = "frequency", 0.6
        belt.drive = DriveConfig(gearRatio=13.0, drumDiameter=0.08, tau=0.0, accel=1e9, decel=1e9)
        v_max = drivetrain.train_of(belt).max_speed                  # 0.467 m/s at 50 Hz
        controller = LineController(config)
        start = next(btn for btn in config.buttons if btn.label == "L1 START")
        import devices
        devices.press(start)
        controller.step(0.05)
        devices.release(start)
        controller.step(0.10)
        run(belt, 0.05)
        self.assertAlmostEqual(belt.frequency, 0.6 / v_max * 50.0, places=6)   # the PLC asks for 64 Hz
        self.assertAlmostEqual(belt.outputFrequency, 50.0, places=6)          # the inverter gives 50
        self.assertAlmostEqual(belt.actualSpeed, v_max, places=6)


if __name__ == "__main__":
    unittest.main()
