"""The PLC link's heartbeat and watchdog (config_parser.LinkWatchdog,
logistics.py), through the PLC's I/O frames (device_check.Rig): the
simulator's heartbeat counts every step; the PLC's, once it moved, standing
still for timeoutMs puts every PLC command at rest until it moves again."""
import time
import unittest

import plc_logic
from config_parser import Belt, LinkWatchdog, SimulatorConfig
from device_check import SCAN_S, Rig

TIMEOUT_MS = 150


def layout():
    belt = Belt(id="belt", type="linear", x=0.0, y=0.0, length=6.0, beltWidth=0.6, rotation=0.0, speed=1.0,
                directionAngle=0.0, color="#64748b", commandFormat="vfd")
    return SimulatorConfig(belts=[belt], watchdogs=[LinkWatchdog(id="link", x=0.0, y=-2.0, timeoutMs=TIMEOUT_MS)])


class WatchdogTest(unittest.TestCase):
    def setUp(self):
        self.rig = Rig(layout())
        self.plc_belt = self.rig.plc_of("belts", "belt")
        self.plc_w = self.rig.plc_of("watchdogs", "link")
        self.sim_w = self.rig.sim_of("watchdogs", "link")
        self.sim_belt = self.rig.sim_of("belts", "belt")
        self.plc_belt.run, self.plc_belt.speed = True, 1.0

    def beat(self, rig):
        self.plc_w.plcHeartbeat = (self.plc_w.plcHeartbeat + 1) & 0xFFFF

    def test_the_simulators_heartbeat_reaches_the_plc(self):
        self.rig.run(0.2)
        first = self.plc_w.simHeartbeat
        self.rig.run(0.2)
        self.assertGreater(self.plc_w.simHeartbeat, first, "it counts every step")
        self.assertEqual(self.sim_w.simHeartbeat, self.plc_w.simHeartbeat)

    def test_not_armed_before_the_plcs_first_beat(self):
        self.rig.run(0.3)
        time.sleep(TIMEOUT_MS / 1000.0 * 2)
        self.rig.run(0.3)
        self.assertFalse(self.sim_w.tripped, "a PLC that never counts is not watched")
        self.assertFalse(self.plc_w.plcAlive)
        self.assertTrue(self.sim_belt.run, "its commands apply")

    def test_trips_when_the_plcs_heartbeat_stops_and_recovers(self):
        self.rig.run(0.5, self.beat)
        self.assertTrue(self.plc_w.plcAlive, "the simulator sees the PLC's heartbeat")
        self.assertGreater(abs(self.sim_belt.actualSpeed), 0.9)
        self.rig.run(0.05)                                   # the PLC's heartbeat stands still...
        time.sleep(TIMEOUT_MS / 1000.0 * 1.5)                # ... longer than the timeout (wall time)
        self.rig.run(0.5)
        self.assertTrue(self.sim_w.tripped)
        self.assertEqual(self.sim_w.trips, 1)
        self.assertFalse(self.plc_w.plcAlive, "the PLC reads it")
        self.assertFalse(self.sim_belt.run, "the PLC's commands at rest")
        self.assertLess(abs(self.sim_belt.actualSpeed), 0.05, "the belt stopped")
        self.assertTrue(self.plc_belt.run, "though the PLC still commands it")
        self.rig.run(0.5, self.beat)                         # it beats again
        self.assertFalse(self.sim_w.tripped)
        self.assertTrue(self.plc_w.plcAlive)
        self.assertGreater(abs(self.sim_belt.actualSpeed), 0.9, "its commands apply again")
        self.assertEqual(self.sim_w.trips, 1)

    def test_the_latency_from_the_plcs_echo(self):
        """The PLC echoes the simHeartbeat its outputs answer; when they arrive
        the simulator measures how many of its steps old that is (simulated ms).
        In the rig an output answers the previous scan's input: one scan old
        at the end of a scan; echoing an older input is seen as older."""
        self.rig.run(0.2)
        self.assertEqual(self.sim_w.latencyMs, 0.0, "no echo: not measured")

        def answer(rig, scans_old=[]):
            scans_old.append(self.plc_w.simHeartbeat)
            self.plc_w.simHeartbeatEcho = scans_old[-1]
        self.rig.run(0.5, answer)
        one_scan = SCAN_S * 1000.0
        step = self.rig.sim.physicsDt * 1000.0
        steps = (self.sim_w.simHeartbeat - self.sim_w.simHeartbeatEcho) & 0xFFFF
        self.assertAlmostEqual(self.sim_w.latencyMs, steps * step, places=6)      # the definition
        self.assertTrue(0.0 < self.sim_w.latencyMs <= one_scan + step, self.sim_w.latencyMs)
        self.assertEqual(self.plc_w.latencyMs, self.sim_w.latencyMs, "the PLC reads it as a tag")

        late = []

        def answer_late(rig):
            late.append(self.plc_w.simHeartbeat)
            self.plc_w.simHeartbeatEcho = late[-3] if len(late) >= 3 else late[0]   # two scans behind
        self.rig.run(0.5, answer_late)
        self.assertTrue(2 * one_scan < self.sim_w.latencyMs <= 3 * one_scan + step, self.sim_w.latencyMs)
        self.assertGreaterEqual(self.sim_w.latencyMaxMs, self.sim_w.latencyMs)

    def test_the_mock_plcs_program_beats(self):
        w = self.rig.plc_of("watchdogs", "link")
        before = w.plcHeartbeat
        w.simHeartbeat = 1234
        plc_logic.DEVICE_PROGRAMS["watchdogs"](None, w, None)
        self.assertEqual(w.plcHeartbeat, (before + 1) & 0xFFFF)
        self.assertEqual(w.simHeartbeatEcho, 1234, "and it echoes the simulator's")


if __name__ == "__main__":
    unittest.main()
