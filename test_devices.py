"""
test_devices.py - the simulated field devices (devices.py, drive.py,
physics.py) and their I/O: IR sensors and laser banners with on/off delay
and light-on/dark-on output, push buttons (momentary/latching, NO/NC), LEDs
with blink modes, belt motors commanded like a VFD (run / reverse / speed)
with feedback (running, atSpeed, reversing, fault, actualSpeed, encoder)
and an overload fault - plus the cleaned-up test_config.json and the motor
feedback reaching plc_mock.py over TCP.
"""
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import devices
import io_layout
from config_parser import (
    Belt, ControlButton, DriveConfig, LedIndicator, Sensor, SimulatorConfig, Sink, Source, load_config,
)
from drive import inject_fault, step_drive
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config, io_topics
from mqtt_message_binding import (
    ByteType, ComponentMessage, belt_feedback_mapping, custom_component_mapping,
)
from physics import PhysicsEngine
from plc_logic import DeliveryController
from sim_runner import SimulationRunner

HERE = os.path.dirname(os.path.abspath(__file__))


def ir(**kw):
    fields = dict(id="eye", x=0, y=0, width=0.1, height=1.2, rotation=0, label="IR", color="#f00")
    fields.update(kw)
    return Sensor(**fields)


def run_filter(sensor, pattern, h=0.01):
    """Feed raw beam states step by step; return the outputs."""
    return [devices.condition(sensor, raw, h) for raw in pattern]


class SensorSignalTest(unittest.TestCase):
    def test_default_follows_the_beam_and_counts_detections(self):
        s = ir()
        out = run_filter(s, [[False], [True], [True], [False], [True], [False]])
        self.assertEqual(out, ["0", "1", "1", "0", "1", "0"])
        self.assertEqual(s.risingEdges, 2)

    def test_on_delay_ignores_short_flicker(self):
        s = ir(onDelay=0.05)
        out = run_filter(s, [[True]] * 3 + [[False]] + [[True]] * 6)
        self.assertEqual(out[:4], ["0", "0", "0", "0"], "30 ms flicker < 50 ms on-delay")
        self.assertEqual(out[4:], ["0", "0", "0", "0", "1", "1"])
        self.assertEqual(s.risingEdges, 1)

    def test_off_delay_stretches_a_short_pulse(self):
        s = ir(offDelay=0.05)
        out = run_filter(s, [[True]] + [[False]] * 6)
        self.assertEqual(out, ["1", "1", "1", "1", "1", "0", "0"])
        self.assertEqual(s.risingEdges, 1)

    def test_invert_is_light_on_nc_output(self):
        s = ir(invert=True)
        self.assertEqual(run_filter(s, [[False], [True], [False]]), ["1", "0", "1"])
        self.assertEqual(s.risingEdges, 1, "a detection is a detection, whatever the output polarity")

    def test_laser_banner_filters_each_beam(self):
        s = Sensor(id="lb", x=0, y=0, width=0.1, height=1, rotation=0, label="LB", color="#f00",
                   sensorType="laser_banner", laserCount=4, offDelay=0.02)
        out = run_filter(s, [[True, True, False, False], [False, True, False, False], [False, False, False, False],
                             [False, False, False, False]])
        self.assertEqual(out, ["1100", "1100", "0100", "0000"])
        self.assertEqual(s.risingEdges, 1)
        self.assertEqual(s.blockedBeams, 0)

    def test_parcel_passing_an_ir_and_a_banner_in_physics(self):
        config = SimulatorConfig(
            belts=[Belt(id="b", type="linear", x=3, y=0, length=8, beltWidth=1, rotation=0, speed=2,
                        directionAngle=0, color="#00f")],
            sensors=[ir(id="eye", x=3, offDelay=0.1),
                     Sensor(id="lb", x=4, y=0, width=0.1, height=1, rotation=0, label="LB", color="#f00",
                            sensorType="laser_banner", laserCount=20, beamDistance=0.05)])
        e = PhysicsEngine(config)
        e.add_parcel(0.5, 0, 0.4, 0.4)
        max_blocked = 0
        for _ in range(300):
            e.update(0.01)
            max_blocked = max(max_blocked, config.sensors[1].blockedBeams)
        self.assertEqual(config.sensors[0].risingEdges, 1)
        self.assertEqual(config.sensors[1].risingEdges, 1)
        self.assertGreaterEqual(max_blocked, 7)        # a 0.4 m parcel under 5 cm beams
        self.assertLessEqual(max_blocked, 9)


class ButtonTest(unittest.TestCase):
    def test_momentary_no(self):
        b = ControlButton(id="b", x=0, y=0, label="START")
        devices.press(b)
        self.assertIs(b.sensor_states, True)
        devices.release(b)
        self.assertIs(b.sensor_states, False)

    def test_latching_toggles_on_each_press(self):
        b = ControlButton(id="b", x=0, y=0, label="ESTOP", buttonType="latching")
        devices.press(b)
        devices.release(b)
        self.assertTrue(b.sensor_states)
        devices.press(b)
        devices.release(b)
        self.assertFalse(b.sensor_states)

    def test_normally_closed_reads_one_until_pressed(self):
        config = SimulatorConfig(buttons=[ControlButton(id="stop", x=0, y=0, label="STOP", normallyClosed=True)])
        PhysicsEngine(config)                         # sets the contacts at start
        stop = config.buttons[0]
        self.assertIs(stop.sensor_states, True)
        devices.press(stop)
        self.assertIs(stop.sensor_states, False)
        self.assertTrue(devices.is_pressed_signal(stop, False))
        self.assertFalse(devices.is_pressed_signal(stop, True))

    def test_plc_program_understands_an_nc_stop(self):
        config = SimulatorConfig(
            belts=[Belt(id="b", type="linear", x=0, y=0, length=2, beltWidth=1, rotation=0, speed=1,
                        directionAngle=0, color="#00f")],
            buttons=[ControlButton(id="start", x=0, y=0, label="START"),
                     ControlButton(id="stop", x=0, y=0, label="STOP", normallyClosed=True)])
        PhysicsEngine(config)
        plc = DeliveryController(config)
        plc.step(0.0)
        self.assertFalse(plc.running, "an NC STOP reading 1 is not a press")
        devices.press(config.buttons[0])
        plc.step(0.1)
        devices.release(config.buttons[0])
        self.assertTrue(plc.running)
        devices.press(config.buttons[1])               # signal goes 0
        plc.step(0.2)
        self.assertFalse(plc.running)


class LedTest(unittest.TestCase):
    def test_blink_modes(self):
        led = LedIndicator(id="l", x=0, y=0, label="ERR", packFormat="byte")
        led.mode = 0
        self.assertFalse(devices.lamp_on(led, 0.1))
        led.mode = 1
        self.assertTrue(devices.lamp_on(led, 0.7))
        led.mode = 2                                   # 1 Hz
        self.assertEqual([devices.lamp_on(led, t) for t in (0.1, 0.6, 1.1)], [True, False, True])
        led.mode = 3                                   # 4 Hz
        self.assertEqual([devices.lamp_on(led, t) for t in (0.05, 0.2, 0.3)], [True, False, True])

    def test_mode_led_on_the_wire_is_a_byte_and_drives_the_lamp(self):
        config = SimulatorConfig(leds=[LedIndicator(id="ok", x=0, y=0, label="OK"),
                                       LedIndicator(id="err", x=0, y=0, label="ERR", packFormat="byte")])
        mapping = custom_component_mapping(config, ["ok", "err"], direction="in")
        self.assertEqual([p for _, p in mapping if not p.startswith("padding")], ["ok/isOn", "err/mode"])
        ComponentMessage(mapping, "<").deserialize(bytes([0b1, 2]), config)
        self.assertEqual(config.leds[1].mode, 2)
        e = PhysicsEngine(config)
        lamp = []
        for _ in range(100):
            e.update(0.02)
            lamp.append(config.leds[1].isOn)
        self.assertTrue(any(lamp) and not all(lamp), "it blinks")
        self.assertTrue(config.leds[0].isOn)


def vfd_belt(**kw):
    fields = dict(id="belt_1", type="linear", x=0, y=0, length=4, beltWidth=1, rotation=0, speed=1.5,
                  directionAngle=0, color="#00f", commandFormat="vfd")
    fields.update(kw)
    return Belt(**fields)


class MotorCommandTest(unittest.TestCase):
    def test_vfd_run_reverse_speed(self):
        belt = vfd_belt()
        self.assertEqual(devices.commanded_speed(belt), 0.0, "not running until `run`")
        belt.run = True
        self.assertEqual(devices.commanded_speed(belt), 1.5)
        belt.reverse = True
        self.assertEqual(devices.commanded_speed(belt), -1.5)
        belt.speed = -2.0                              # the magnitude counts, the bit sets the direction
        self.assertEqual(devices.commanded_speed(belt), -2.0)
        belt.fault = True
        self.assertEqual(devices.commanded_speed(belt), 0.0)

    def test_vfd_command_on_the_wire(self):
        config = SimulatorConfig(belts=[vfd_belt(), vfd_belt(id="belt_2", speed=0.8)],
                                 leds=[LedIndicator(id="l", x=0, y=0, label="RUN")])
        mapping = custom_component_mapping(config, ["l", "belt_1", "belt_2"], direction="in")
        fields = [(f.address, f.byte_type, f.path) for f in io_layout.layout_fields(mapping) if not f.is_padding]
        self.assertEqual(fields, [("0.0", ByteType.BIT, "l/isOn"), ("0.1", ByteType.BIT, "belt_1/run"),
                                  ("0.2", ByteType.BIT, "belt_1/reverse"), ("0.3", ByteType.BIT, "belt_2/run"),
                                  ("0.4", ByteType.BIT, "belt_2/reverse"), ("1", ByteType.FLOAT, "belt_1/speed"),
                                  ("5", ByteType.FLOAT, "belt_2/speed")])
        self.assertEqual(io_layout.st_fields(mapping), [
            ("l", "BOOL"), ("belt_1_run", "BOOL"), ("belt_1_reverse", "BOOL"), ("belt_2_run", "BOOL"),
            ("belt_2_reverse", "BOOL"), ("belt_1_speed", "REAL"), ("belt_2_speed", "REAL")])
        payload = bytes([0b00110]) + struct.pack("<ff", 1.25, 0.5)  # belt_1 run+reverse, belt_2 off
        ComponentMessage(mapping, "<").deserialize(payload, config)
        b1, b2 = config.belts
        self.assertEqual((b1.run, b1.reverse, b1.speed), (True, True, 1.25))
        self.assertEqual((b2.run, b2.reverse, b2.speed), (False, False, 0.5))
        self.assertEqual(devices.commanded_speed(b1), -1.25)

    def test_drive_ramps_and_reports_status(self):
        belt = vfd_belt(drive=DriveConfig(accel=2.0, decel=2.0, tau=0.0), run=True)
        step_drive(belt, 0.1)
        self.assertTrue(belt.running)
        self.assertFalse(belt.atSpeed, "still ramping")
        for _ in range(10):
            step_drive(belt, 0.1)
        self.assertAlmostEqual(belt.actualSpeed, 1.5)
        self.assertTrue(belt.atSpeed)
        self.assertFalse(belt.reversing)
        belt.reverse = True
        for _ in range(20):
            step_drive(belt, 0.1)
        self.assertAlmostEqual(belt.actualSpeed, -1.5)
        self.assertTrue(belt.reversing and belt.atSpeed)
        self.assertLess(belt.encoderCount, 1 << 32)

    def test_feedback_frame_layout(self):
        config = SimulatorConfig(belts=[vfd_belt(), vfd_belt(id="belt_2")])
        mapping = belt_feedback_mapping(config)
        fields = [(f.address, f.path) for f in io_layout.layout_fields(mapping) if not f.is_padding]
        self.assertEqual(fields, [
            ("0.0", "belt_1/running"), ("0.1", "belt_1/atSpeed"), ("0.2", "belt_1/reversing"),
            ("0.3", "belt_1/fault"), ("0.4", "belt_2/running"), ("0.5", "belt_2/atSpeed"),
            ("0.6", "belt_2/reversing"), ("0.7", "belt_2/fault"), ("1", "belt_1/actualSpeed"),
            ("5", "belt_1/encoderCount"), ("9", "belt_2/actualSpeed"), ("13", "belt_2/encoderCount")])
        b1 = config.belts[0]
        b1.running, b1.fault, b1.actualSpeed, b1.encoderCount = True, True, 0.75, 123456
        payload = ComponentMessage(mapping, "<").serialize(config)
        self.assertEqual(payload[0], 0b1001)
        self.assertEqual(struct.unpack_from("<fI", payload, 1), (0.75, 123456))


class OverloadFaultTest(unittest.TestCase):
    def test_overload_trips_coasts_and_resets_when_run_is_removed(self):
        belt = vfd_belt(drive=DriveConfig(accel=10, decel=10, tau=0.0, maxLoad=5.0, overloadTime=0.5), run=True)
        for _ in range(10):
            step_drive(belt, 0.05)
        self.assertTrue(belt.atSpeed)
        belt.load = 8.0
        for _ in range(8):                            # 0.4 s: not yet
            step_drive(belt, 0.05)
        self.assertFalse(belt.fault)
        for _ in range(4):
            step_drive(belt, 0.05)
        self.assertTrue(belt.fault)
        speeds = []
        for _ in range(10):
            step_drive(belt, 0.05)
            speeds.append(belt.actualSpeed)
        self.assertTrue(speeds[0] > speeds[-1] > 0.0, "coasting down, not a ramp stop")
        belt.load = 2.0
        step_drive(belt, 0.05)
        self.assertTrue(belt.fault, "latched while `run` is still on")
        belt.run = False
        step_drive(belt, 0.05)
        self.assertFalse(belt.fault)

    def test_parcels_piling_up_on_a_belt_trip_it(self):
        config = SimulatorConfig(
            belts=[vfd_belt(length=4, x=2, run=True, speed=1.0,
                            drive=DriveConfig(maxLoad=2.5, overloadTime=0.3))],
            sinks=[])
        e = PhysicsEngine(config)
        for x in (0.5, 1.5, 2.5):
            e.add_parcel(x, 0, 0.4, 0.4)
        for _ in range(60):
            e.update(0.01)
        belt = config.belts[0]
        self.assertGreaterEqual(belt.load, 2.5)
        self.assertTrue(belt.fault)

    def test_plc_stops_the_line_on_a_drive_fault_and_restarts_after_reset(self):
        config = SimulatorConfig(belts=[vfd_belt(), vfd_belt(id="belt_2", reverse=True)],
                                 buttons=[ControlButton(id="go", x=0, y=0, label="START")],
                                 leds=[LedIndicator(id="run", x=0, y=0, label="RUN"),
                                       LedIndicator(id="err", x=0, y=0, label="ERROR")])
        plc = DeliveryController(config, autostart=True)
        plc.step(0.0)
        b1, b2 = config.belts
        self.assertEqual((b1.run, b1.reverse, b2.run, b2.reverse), (True, False, True, True))
        b2.fault = True                                # as received in the feedback frame
        events = plc.step(0.1)
        self.assertIn("FAULT on belt_2: drive tripped - line stopped", events)
        self.assertFalse(b1.run or b2.run)
        self.assertTrue(config.leds[1].isOn)
        devices.press(config.buttons[0])
        self.assertIn("go: can't start, drive fault on belt_2", plc.step(0.2))
        devices.release(config.buttons[0])
        b2.fault = False                               # the drive reset (run removed)
        self.assertIn("drive belt_2 fault cleared - press Start", plc.step(0.3))
        devices.press(config.buttons[0])
        plc.step(0.4)
        self.assertTrue(b1.run and b2.run)


class TestConfigTest(unittest.TestCase):
    """test_config.json: Station_1 only (the AX project's Station_1_Input /
    Station_1_Output), with drives, sensor timing, button roles and keys."""

    def setUp(self):
        self.config = load_config("test_config.json")

    def test_only_station_1(self):
        ids = {c.id for kind in ("belts", "sensors", "buttons", "leds") for c in getattr(self.config, kind)}
        for component_id in ids:
            self.assertTrue(component_id.split("_", 1)[1].startswith("1_"), component_id)
        self.assertEqual(len(self.config.belts), 16)
        self.assertEqual([s.id for s in self.config.sinks], ["sink_4"])

    def test_wire_frames_are_unchanged(self):
        fixture = load_config("test_fixture_config.json")   # the file before the cleanup
        for config_a, config_b in ((self.config, fixture),):
            pub_a, sub_a = io_topics(config_a)
            pub_b, sub_b = io_topics(config_b)
            for (topics_a, topics_b, direction) in ((pub_a, pub_b, "out"), (sub_a, sub_b, "in")):
                for name in topics_a:
                    a = custom_component_mapping(config_a, topics_a[name]["custom_ids"], direction)
                    b = custom_component_mapping(config_b, topics_b[name]["custom_ids"], direction)
                    self.assertEqual(io_layout.layout_fingerprint(a), io_layout.layout_fingerprint(b))

    def test_devices_are_configured(self):
        labels = {b.id: b.label for b in self.config.buttons}
        self.assertEqual(labels["button_1_1_1"], "L1 START")
        self.assertEqual(labels["button_1_3_2"], "L3 STOP")
        self.assertEqual(len({b.key for b in self.config.buttons}), 9, "a key per button")
        self.assertEqual([l.label for l in self.config.leds[:3]], ["L1 AUTO", "L1 MANUAL", "L1 ERROR"])
        commanded = [b for b in self.config.belts if b.id != "belt_1_1_6"]
        self.assertTrue(all(b.drive is not None and b.drive.maxLoad for b in commanded))
        self.assertTrue(all(b.speed == 0 for b in commanded), "the PLC sets the speed")
        self.assertEqual(self.config.belts[-1].id, "belt_1_1_6")
        self.assertGreater(self.config.belts[-1].speed, 0, "the outfeed runs without the PLC")
        self.assertFalse(self.config.mqttSettings.beltFeedback)

    def test_station_delivers_and_every_sensor_counts(self):
        e = PhysicsEngine(self.config)
        plc = DeliveryController(self.config, autostart=True, jam_time=60)
        for belt_id in plc.speeds:
            plc.speeds[belt_id] = 1.5                   # ~35 m from lane 1's infeed to the sink
        t = 0.0
        while t < 40.0:
            e.update(0.05)
            t += 0.05
            plc.step(t)
        self.assertGreater(e.delivered.get("sink_4", 0), 5)
        for sensor in self.config.sensors:
            self.assertGreater(sensor.risingEdges, 3, sensor.id)
        self.assertFalse(any(b.fault for b in self.config.belts))


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class MotorFeedbackOverTcpTest(unittest.TestCase):
    """A vfd line with motor feedback on: the simulator and the real
    plc_mock.py process."""

    def test_fault_reaches_the_plc_which_stops_and_the_drive_resets(self):
        tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(tmp.cleanup)
        config = SimulatorConfig(
            belts=[vfd_belt(drive=DriveConfig(accel=5, decel=5, tau=0.02)),
                   vfd_belt(id="belt_2", reverse=True, drive=DriveConfig(accel=5, decel=5, tau=0.02))],
            buttons=[ControlButton(id="go", x=0, y=0, label="START"),
                     ControlButton(id="halt", x=0, y=0, label="STOP", normallyClosed=True)],
            leds=[LedIndicator(id="run", x=0, y=0, label="RUN"),
                  LedIndicator(id="err", x=0, y=0, label="ERROR", packFormat="byte")])
        layout = os.path.join(tmp.name, "line.json")
        with open(layout, "w", encoding="utf-8") as f:
            f.write(config.model_dump_json())
        exchange = os.path.join(tmp.name, "exchange.json")
        with open(exchange, "w", encoding="utf-8") as f:
            json.dump({"transport": "tcp", "enabled": True, "tcpBindHost": "127.0.0.1",
                       "tcpServerHost": "127.0.0.1", "tcpPort": free_port(), "beltFeedback": True,
                       "beltFeedbackIntervalMs": 10}, f)

        sim = load_config(layout)
        apply_exchange_config(sim, exchange)
        lock = threading.Lock()
        physics = PhysicsEngine(sim, lock=lock)
        engine = MQTTSyncEngine(sim, lock=lock)
        runner = SimulationRunner(physics, engine, tick_s=0.002)
        self.addCleanup(engine.stop)
        self.addCleanup(runner.stop)
        runner.start()

        proc = subprocess.Popen(
            [sys.executable, "-u", os.path.join(HERE, "plc_mock.py"), layout, "--exchange-config", exchange,
             "--scan", "0.02", "--send-interval", "60"],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8")
        lines = []
        threading.Thread(target=lambda: [lines.append(l.rstrip()) for l in proc.stdout], daemon=True).start()

        def stop():
            proc.terminate()
            proc.wait(timeout=10)
            proc.stdout.close()
        self.addCleanup(stop)
        log = lambda: "\n".join(lines)  # noqa: E731
        self.assertTrue(wait_for(lambda: any("I/O layout check: OK" in l for l in lines), 15), log())
        b1, b2 = sim.belts

        # START (NO) runs both drives - belt_2 in reverse - and they report at speed.
        with lock:
            devices.press(sim.buttons[0])
        self.assertTrue(wait_for(lambda: b1.atSpeed and b2.atSpeed, 10), log())
        with lock:
            devices.release(sim.buttons[0])
        self.assertTrue(b1.run and b2.run and b2.reverse and not b1.reverse)
        self.assertTrue(wait_for(lambda: abs(b2.actualSpeed + 1.5) < 0.005, 5), b2.actualSpeed)
        self.assertTrue(b2.reversing)

        # A drive fault reaches the PLC through BELT_FEEDBACK; it stops the line,
        # the run command goes away, so the drive resets; the PLC says so.
        with lock:
            inject_fault(b1)
        self.assertTrue(wait_for(lambda: any("FAULT on belt_1" in l for l in lines), 10), log())
        self.assertTrue(wait_for(lambda: not b1.run and not b2.run, 5))
        self.assertTrue(wait_for(lambda: any("drive belt_1 fault cleared" in l for l in lines), 10), log())
        self.assertFalse(b1.fault)

        # START again; then the NC STOP: pressing it opens the contact.
        with lock:
            devices.press(sim.buttons[0])
        self.assertTrue(wait_for(lambda: b1.run, 5), log())
        with lock:
            devices.release(sim.buttons[0])
            devices.press(sim.buttons[1])
        self.assertTrue(wait_for(lambda: not b1.run, 5), log())
        self.assertTrue(any("STOP (halt)" in l for l in lines), log())
        self.assertTrue(wait_for(lambda: not b1.running, 5))
        self.assertIsNone(proc.poll())


if __name__ == "__main__":
    unittest.main()
