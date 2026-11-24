"""
test_tracking.py - tracking.py: the PLC's tracked loads (trackedLoads slots)
checked against the parcels at the simulation step the PLC's values come
from (the watchdog's simHeartbeat).

A parcel rides a belt at 1 m/s. The test plays the PLC: it fills a slot
with the parcel's pose as recorded some steps ago and that step's
heartbeat, as a PLC answering a frame late does.
"""
import math
import unittest

from config_parser import Belt, LinkWatchdog, SimulatorConfig, TrackedLoadView
from physics import PhysicsEngine
import tracking


def make_engine():
    belt = Belt(id="belt", type="linear", x=5.0, y=0.0, length=10.0, beltWidth=0.6, rotation=0.0, speed=1.0,
                directionAngle=0.0, color="#64748b")
    config = SimulatorConfig(belts=[belt], physicsDt=0.004,
                             watchdogs=[LinkWatchdog(id="watchdog", x=0.0, y=-2.0)],
                             trackedLoads=[TrackedLoadView(id="track_1", x=0.0, y=-3.0),
                                           TrackedLoadView(id="track_2", x=0.0, y=-3.5)])
    engine = PhysicsEngine(config)
    parcel = engine.add_parcel(1.0, 0.0, 0.4, 0.25, 0.0, 1.0)
    return engine, config, parcel


def pose(parcel):
    x, y = parcel.body.position
    return float(x), float(y), math.degrees(parcel.body.angle)


class TrackingCheckTest(unittest.TestCase):
    def setUp(self):
        self.engine, self.config, self.parcel = make_engine()
        self.slot = self.config.trackedLoads[0]
        self.history = {}                                   # heartbeat -> the parcel's pose then

    def step(self, n=1):
        for _ in range(n):
            self.engine.step(self.config.physicsDt) if hasattr(self.engine, "step") else \
                self.engine.run_for(self.config.physicsDt)
            self.history[self.config.watchdogs[0].simHeartbeat] = pose(self.parcel)

    def show(self, frame, offset_along=0.0, label_frame=None):
        x, y, yaw = self.history[frame]
        s = self.slot
        s.active, s.loadId, s.loadYaw = True, 7, yaw
        s.loadX, s.loadY = x + offset_along * math.cos(math.radians(yaw)), y
        s.loadLength, s.loadWidth = 0.4, 0.25
        s.frame = frame if label_frame is None else label_frame

    def check(self):
        self.step()                                         # the simulator checks after its next step

    def test_a_late_answer_with_its_frame_is_exact(self):
        self.step(200)                                      # the parcel is moving at the belt's speed
        self.assertGreater(self.parcel.body.velocity.x, 0.9)
        frame = self.config.watchdogs[0].simHeartbeat - 5   # the PLC answers 5 steps (20 mm) late
        self.show(frame)
        self.check()
        self.assertTrue(self.slot.matched)
        self.assertLess(abs(self.slot.alongError), 1e-6)
        self.assertLess(abs(self.slot.lateralError), 1e-6)
        report = self.engine.logistics.tracking.report()
        self.assertEqual(report["loads"], 1)
        self.assertLess(abs(report["recentLoads"][0]["length"]), 1e-3, "0.4 m along the travel")
        self.assertLess(abs(report["recentLoads"][0]["width"]), 1e-3, "0.25 m across")

    def test_the_size_errors(self):
        self.step(200)
        self.show(self.config.watchdogs[0].simHeartbeat - 3)
        self.slot.loadLength, self.slot.loadWidth = 0.43, 0.24
        self.check()
        self.assertAlmostEqual(self.slot.lengthError, 0.03, places=3)    # the 0.4 x 0.25 parcel
        self.assertAlmostEqual(self.slot.widthError, -0.01, places=3)

    def test_an_offset_is_reported_along_the_travel(self):
        self.step(200)
        frame = self.config.watchdogs[0].simHeartbeat - 3
        self.show(frame, offset_along=0.010)
        self.check()
        self.assertAlmostEqual(self.slot.alongError, 0.010, places=6)
        self.assertLess(abs(self.slot.lateralError), 1e-6)

    def test_without_the_frame_the_delay_is_an_error(self):
        """The same late pose labelled as the newest step: the parcel moved
        on since, so the load is behind it by the delay times the speed."""
        self.step(200)
        newest = self.config.watchdogs[0].simHeartbeat
        self.show(newest - 5, label_frame=newest)
        self.check()
        self.assertAlmostEqual(self.slot.alongError, -5 * 0.004 * 1.0, delta=0.002)

    def test_an_empty_slot_and_a_load_far_from_every_parcel(self):
        self.step(10)
        self.check()
        self.assertFalse(self.slot.matched)
        frame = self.config.watchdogs[0].simHeartbeat
        self.show(frame, offset_along=2.0)                  # 2 m off: no parcel near it
        self.check()
        self.assertFalse(self.slot.matched)
        self.assertEqual(self.engine.logistics.tracking.unmatched, 1)

    def test_an_old_frame_is_stale(self):
        self.step(10)
        self.show(self.config.watchdogs[0].simHeartbeat)
        self.slot.frame = (self.slot.frame - tracking.SNAPSHOTS - 10) & 0xFFFF
        self.check()
        self.assertEqual(self.engine.logistics.tracking.stale, 1)


if __name__ == "__main__":
    unittest.main()
