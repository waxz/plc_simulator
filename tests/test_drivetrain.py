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
        plant = load_config("station_1.json")
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
        config = load_config("station_1.json")
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
        config = load_config("station_1.json")
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


class ElevatorHoistTest(unittest.TestCase):
    """An elevator driven by a hoist: motor, gearbox, drum / sprocket."""

    def lift(self, drive=None):
        from config_parser import Elevator
        return Elevator(id="lift", x=0, y=0, levels=2, levelHeight=1.2, liftSpeed=0.6, drive=drive)

    def test_lift_speed_from_the_hoist(self):
        self.assertIsNone(drivetrain.train_of(self.lift()))                      # no drive: liftSpeed
        self.assertEqual(drivetrain.lift_speed(self.lift()), 0.6)
        hoist = self.lift(DriveConfig(gearRatio=40.0, drumDiameter=0.2))
        self.assertAlmostEqual(drivetrain.lift_speed(hoist), 1450.0 / 40.0 * math.pi * 0.2 / 60.0, places=9)
        # only the gear ratio: the drum is the one that gives the design speed
        self.assertAlmostEqual(drivetrain.lift_speed(self.lift(DriveConfig(gearRatio=40.0))), 0.6, places=9)
        # an inverter capped below the rated frequency: slower
        capped = self.lift(DriveConfig(gearRatio=40.0, drumDiameter=0.2, maxFrequency=25.0))
        self.assertAlmostEqual(drivetrain.lift_speed(capped), drivetrain.lift_speed(hoist) / 2, places=9)
        self.assertTrue(drivetrain.summary(capped)[1])                          # and the designer says so

    def test_travel_time_with_ramps(self):
        hoist = self.lift(DriveConfig(gearRatio=40.0, drumDiameter=0.2, rampUpTime=1.0, rampDownTime=1.0))
        v = drivetrain.lift_speed(hoist)
        # 1 s up, 1 s down (0.38 m together), the rest at full speed
        self.assertAlmostEqual(drivetrain.lift_time(hoist, 1.2), 2.0 + (1.2 - v) / v, places=9)
        self.assertAlmostEqual(drivetrain.lift_time(self.lift(), 1.2), 2.0, places=9)   # no ramps: 1.2 / 0.6

    def test_the_simulated_lift_follows_the_profile(self):
        from physics import PhysicsEngine
        config = load_config("logistics_full.json")
        lift = config.elevators[0]
        lift.control = "manual"
        lift.drive = DriveConfig(gearRatio=40.0, drumDiameter=0.2, rampUpTime=1.0, rampDownTime=1.0)
        engine = PhysicsEngine(config)
        lift.targetLevel = 1
        t, peak, peak_rpm, h = 0.0, 0.0, 0.0, 0.02
        while t < 20.0 and not (lift.atLevel and lift.level == 1):
            engine.update(h)
            t += h
            peak, peak_rpm = max(peak, abs(lift.liftVelocity)), max(peak_rpm, abs(lift.motorSpeed))
        expected = drivetrain.lift_time(lift, lift.levelHeight)
        self.assertAlmostEqual(t, expected, delta=0.05 * expected)              # ramps and cruise
        self.assertAlmostEqual(peak, drivetrain.lift_speed(lift), places=3)
        self.assertAlmostEqual(peak_rpm, 1450.0, delta=1.0)
        self.assertEqual((lift.position, lift.liftVelocity), (1.0, 0.0))        # on the floor, stopped

    def test_the_plc_interface_is_unchanged(self):
        self.assertEqual(st_export.device_type(self.lift(DriveConfig(gearRatio=40.0)), "in"), "Elevator_Command")
        frames = {f["name"]: f["fingerprint"] for f in st_export.contract(load_config("logistics_full.json"))["frames"]}
        self.assertEqual((frames["Sim_Input"], frames["Sim_Output"]), (0x083EBF55, 0xA188665C))


class TurntableAndTransferDriveTest(unittest.TestCase):
    """A turntable's slewing drive (rotary) and a transfer's chain drive."""

    def test_turn_speed_from_the_slewing_drive(self):
        from config_parser import Turntable
        table = Turntable(id="tt", x=0, y=0, turnSpeed=90.0)
        self.assertIsNone(drivetrain.train_of(table))
        self.assertEqual(drivetrain.machine_speed(table), 90.0)               # no drive: turnSpeed
        table.drive = DriveConfig(gearRatio=50.0, transmissionRatio=6.0)       # no drum needed
        self.assertAlmostEqual(drivetrain.machine_speed(table), 1450.0 / 50.0 / 6.0 * 6.0, places=9)   # deg/s
        self.assertAlmostEqual(drivetrain.motor_speed_of(table, 29.0), 1450.0, places=6)
        table.drive = DriveConfig(transmissionRatio=6.0)                        # gear derived: the design speed
        self.assertAlmostEqual(drivetrain.machine_speed(table), 90.0, places=9)

    def test_the_simulated_table_turns_on_its_ramps(self):
        from physics import PhysicsEngine
        config = load_config("logistics_full.json")
        table = config.turntables[0]
        table.control = "manual"
        table.drive = DriveConfig(gearRatio=50.0, transmissionRatio=6.0, rampUpTime=0.5, rampDownTime=0.5)
        engine = PhysicsEngine(config)
        table.targetIndex = 1
        t, peak = 0.0, 0.0
        while t < 30.0 and not (table.inPosition and table.index == 1):
            engine.update(0.02)
            t += 0.02
            peak = max(peak, abs(table.turnVelocity))
        turn = abs(table.angles[1] - table.angles[0])
        expected = drivetrain.travel_time(table, turn)
        self.assertAlmostEqual(t, expected, delta=0.05 * expected)
        self.assertAlmostEqual(peak, 29.0, places=3)
        self.assertEqual((table.angle, table.turnVelocity), (table.angles[1], 0.0))

    def test_the_motor_limits_cap_every_machine(self):
        from config_parser import Elevator, Transfer, Turntable
        table = Turntable(id="tt", x=0, y=0, turnSpeed=90.0,
                          drive=DriveConfig(gearRatio=50.0, transmissionRatio=6.0, rampUpTime=0.5,
                                            motorMaxSpeed=725.0, motorMaxAccel=1450.0))
        self.assertAlmostEqual(drivetrain.machine_speed(table), 14.5, places=9)       # 725 rpm: half of 29 deg/s
        self.assertAlmostEqual(drivetrain.machine_ramps(table)[0], 29.0, places=9)    # 1450 rpm/s, not 58 deg/s^2
        self.assertAlmostEqual(drivetrain.machine_ramps(table)[1], 29.0, places=9)    # no ramp time: the motor's
        lift = Elevator(id="lift", x=0, y=0, levels=2, levelHeight=1.2, liftSpeed=0.6,
                        drive=DriveConfig(gearRatio=40.0, drumDiameter=0.2, motorMaxSpeed=1000.0))
        self.assertAlmostEqual(drivetrain.lift_speed(lift), 1000.0 / 40.0 * math.pi * 0.2 / 60.0, places=9)
        transfer = Transfer(id="tr", x=0, y=0, speed=0.5,
                            drive=DriveConfig(gearRatio=30.0, drumDiameter=0.08, motorMaxAccel=3000.0))
        self.assertAlmostEqual(drivetrain.machine_ramps(transfer)[0],
                               3000.0 / 30.0 * math.pi * 0.08 / 60.0, places=9)
        self.assertEqual(drivetrain.motor_limits(Transfer(id="t2", x=0, y=0, speed=0.5)), (math.inf, math.inf))

    def test_the_designer_pins_a_belt_train_so_rpm_limits_act(self):
        belt = Belt(id="b", type="linear", x=0, y=0, length=4, beltWidth=1, rotation=0, speed=0.8,
                    directionAngle=0, color="#fff", commandFormat="vfd")
        self.assertIsNone(drivetrain.train_of(belt))                              # m/s: no train
        shown = drivetrain.train_of(belt, assume_drive=True)                     # what the designer shows
        self.assertAlmostEqual(shown.speed_for_frequency(50.0), 0.8, places=9)
        belt.drive = DriveConfig(motorMaxAccel=2900.0)
        self.assertEqual(drivetrain.motor_limits(belt), (math.inf, math.inf))    # no train: no effect
        drivetrain.pin_belt_train(belt)
        self.assertEqual((belt.drive.gearRatio, belt.drive.drumDiameter), (shown.gear_ratio, shown.drum_diameter))
        belt.speed = 0.3                                                          # the PLC's command now
        self.assertAlmostEqual(drivetrain.motor_limits(belt)[1], 2900.0 * 0.8 / 1450.0, places=9)
        self.assertIn("rpm/s", drivetrain.summary(belt)[0])

    def test_chain_speed_from_the_sprocket(self):
        from config_parser import Transfer
        transfer = Transfer(id="tr", x=0, y=0, speed=0.5)
        self.assertEqual(drivetrain.machine_speed(transfer), 0.5)
        transfer.drive = DriveConfig(gearRatio=30.0, drumDiameter=0.08)
        self.assertAlmostEqual(drivetrain.machine_speed(transfer), 1450.0 / 30.0 * math.pi * 0.08 / 60.0, places=9)
        self.assertIn("cross 0.202 m/s", drivetrain.summary(transfer)[0])


if __name__ == "__main__":
    unittest.main()
