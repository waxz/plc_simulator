"""The simulator's health: stall_watch.py (a stuck loop's stacks to
sim_stall.log), SimulationRunner.health() (tick timing, stalls), and the
window's HEALTH field and debug panel (F3) in visualizer.py."""
import os
import tempfile
import threading
import time
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame  # noqa: E402

import hmi_style as hmi  # noqa: E402
import stall_watch  # noqa: E402
from config_parser import Belt, LedIndicator, Sensor, SimulatorConfig  # noqa: E402
from sim_runner import STALL_S, SimulationRunner  # noqa: E402
from visualizer import Renderer  # noqa: E402


class StallWatchTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.log = os.path.join(tmp.name, "stall.log")
        self._log, stall_watch.LOG = stall_watch.LOG, self.log
        self.addCleanup(setattr, stall_watch, "LOG", self._log)

    def test_a_stuck_loop_gets_its_stacks_written_once(self):
        ticks = [0]
        stuck = threading.Event()

        def loop():
            while not stuck.is_set():
                ticks[0] += 1
                time.sleep(0.01)
            time.sleep(1.2)                     # stuck here: this line must be in the log
            ticks[0] += 1

        watch = stall_watch.StallWatch("test", lambda: ticks[0], after_s=0.5)
        watch.start()
        self.addCleanup(watch.stop)
        worker = threading.Thread(target=loop, name="TheLoop")
        worker.start()
        time.sleep(0.2)
        stuck.set()
        worker.join()
        time.sleep(0.4)
        self.assertEqual(watch.dumps, 1)
        text = open(self.log, encoding="utf-8").read()
        self.assertIn("the test loop has not run for", text)
        self.assertIn("thread TheLoop", text)
        self.assertIn("time.sleep(1.2)", text)
        self.assertIn("ran again after", text)

    def test_a_running_loop_writes_nothing(self):
        ticks = [0]
        watch = stall_watch.StallWatch("busy", lambda: ticks[0], after_s=0.3)
        watch.start()
        self.addCleanup(watch.stop)
        end = time.monotonic() + 0.8
        while time.monotonic() < end:
            ticks[0] += 1
            time.sleep(0.01)
        self.assertEqual(watch.dumps, 0)
        self.assertFalse(os.path.exists(self.log))


class SlowPhysics:
    """A physics engine whose update() takes as long as told."""
    def __init__(self):
        self.sim_time = 0.0
        self.dropped_time = 0.0
        self.delay = 0.0

    def update(self, dt):
        if self.delay:
            time.sleep(self.delay)
            self.delay = 0.0
        self.sim_time += dt


class RunnerHealthTest(unittest.TestCase):
    def test_timing_and_a_slow_tick(self):
        physics = SlowPhysics()
        runner = SimulationRunner(physics, None, tick_s=0.002, speed=1.0)
        runner.start()
        self.addCleanup(runner.stop)
        time.sleep(0.5)
        health = runner.health()
        self.assertGreater(health["tick_hz"], 50)
        self.assertAlmostEqual(health["ratio"], 1.0, delta=0.2)
        self.assertEqual(health["stall_count"], 0)
        self.assertIsNone(health["last_stall"])
        physics.delay = STALL_S + 0.15         # one tick takes 0.4 s
        time.sleep(0.8)
        health = runner.health()
        self.assertEqual(health["stall_count"], 1)
        self.assertGreater(health["last_stall"]["length_s"], STALL_S)
        self.assertLess(health["last_stall"]["ago_s"], 1.0)
        self.assertGreater(health["physics_ms_max"], STALL_S * 1000)


class FakeProxy:
    """What PhysicsProxy gives the renderer: the last snapshot's time and health."""
    sim_time = 10.0
    delivered = {}
    dropped_time = 0.0

    def __init__(self, age_s, health):
        self.last_snapshot_at = time.monotonic() - age_s
        self.health = health

    def parcel_views(self):
        return []


class WindowHealthTest(unittest.TestCase):
    def setUp(self):
        self.config = SimulatorConfig(
            belts=[Belt(id="infeed", type="linear", x=0, y=0, length=4, beltWidth=0.5, rotation=0, speed=1,
                        directionAngle=0, color="#9ca3af")],
            sensors=[Sensor(id="infeed_jam", x=1, y=0, width=0.05, height=0.7, rotation=0, label="", color="#f00")],
            leds=[LedIndicator(id="panel_fault", x=-2, y=-2, label="FAULT")])
        self.r = Renderer(self.config)
        self.addCleanup(self.r.quit)

    def health(self, **kw):
        h = {"ticks": 100, "tick_hz": 480.0, "tick_ms_avg": 0.4, "tick_ms_max": 3.0, "physics_ms_max": 2.0,
             "ratio": 1.0, "stall_count": 0, "longest_stall_s": 0.0, "last_stall": None, "dropped_s": 0.0}
        h.update(kw)
        return h

    def test_status(self):
        self.assertEqual(self.r.health_status(FakeProxy(0.05, self.health()))[0], "OK")
        stalled = self.health(stall_count=1, longest_stall_s=1.8, last_stall={"ago_s": 3.0, "length_s": 1.8})
        self.assertEqual(self.r.health_status(FakeProxy(0.05, stalled))[0], "STALL 1.8 s")
        old = self.health(stall_count=1, longest_stall_s=1.8, last_stall={"ago_s": 30.0, "length_s": 1.8})
        self.assertEqual(self.r.health_status(FakeProxy(0.05, old))[0], "OK")
        self.assertEqual(self.r.health_status(FakeProxy(2.5, self.health()))[0], "SIM NOT RESPONDING 2.5 s")

    def test_the_panel_names_a_long_blocked_eye_and_the_fault_lamp(self):
        proxy = FakeProxy(0.05, self.health())
        self.config.sensors[0].sensor_states = True
        self.config.leds[0].isOn = True
        self.r.render(proxy)                   # the eye becomes blocked now
        self.r._blocked_since["infeed_jam"] -= 3.2          # ... 3.2 s ago
        lines = dict(self.r.debug_lines(proxy))
        jam = next(text for text in lines if text.startswith("infeed_jam blocked"))
        self.assertEqual(lines[jam], hmi.ALARM)              # as long as the PLC's jam time: red
        self.assertIn("panel_fault: ON", lines)
        self.config.sensors[0].sensor_states = False
        self.r.render(proxy)
        self.assertFalse(any(t.startswith("infeed_jam blocked") for t, _c in self.r.debug_lines(proxy)))

    def test_f3_draws_the_panel(self):
        proxy = FakeProxy(0.05, self.health(stall_count=2, longest_stall_s=0.6,
                                            last_stall={"ago_s": 1.0, "length_s": 0.6}))
        self.r.render(proxy)
        before = pygame.image.tobytes(self.r.screen, "RGB")
        self.r.show_debug = True
        self.r.render(proxy)
        self.assertNotEqual(before, pygame.image.tobytes(self.r.screen, "RGB"))
        texts = [t for t, _c in self.r.debug_lines(proxy)]
        self.assertTrue(any("stalls 2, longest 0.60 s" in t for t in texts), texts)


if __name__ == "__main__":
    unittest.main()
