"""
test_plc_merge.py - a merge line's inductions by a slot train (plc_merge.py),
the program the AX library carries next (ax_dev reviews 35-36): the train,
the sync tracker, the program's choice, and closed loop on merge_demo.json's
physics - every induction served, no miss, each load in its car, the loads
on the line apart.
"""
import math
import random
import unittest

from config_parser import load_config
from physics import PhysicsEngine
from plc_merge import FRONT_IN_CAR_M, PITCH_M, MergeTrainController, SlotTrain, SyncTracker, required_gap


class TrainTest(unittest.TestCase):
    def test_cars_a_pitch_apart_cover_the_start_and_leave_past_the_end(self):
        train = SlotTrain(8.0, before=2.0)
        for _ in range(2000):                                   # 20 m at 10 mm a scan
            train.advance(0.01, 1.0)
            s = sorted(c.s for c in train.cars)
            self.assertTrue(all(abs((b - a) - PITCH_M) < 1e-9 for a, b in zip(s, s[1:])))
            self.assertLessEqual(s[0], -2.0 + PITCH_M / 2.0 + 1e-9)   # reaches back before the start
            self.assertLessEqual(s[-1], 8.0 + PITCH_M / 2.0 + 1e-9)


class TrackerTest(unittest.TestCase):
    def test_a_load_at_rest_meets_its_moving_car_at_its_speed(self):
        tracker, dt, a_max = SyncTracker(), 0.01, 1.6
        x, v = -1.7, 0.0                                        # the load 1.7 m before the port, at rest
        target, vt = -4.0, 1.1547                               # its car 4 m before, coming at 1.15 m/s
        for i in range(600):
            t = i * dt
            command = tracker.update(t, target, vt, x, v, a_max, 2.5)
            v += max(-2.0 * dt, min(2.0 * dt, command - v))     # a drive at 2 m/s^2
            x += v * dt
            target += vt * dt
            if x >= 0.0:
                break
        self.assertTrue(tracker.active)
        self.assertLess(abs(x - target), 0.02)                  # on its car at the port
        self.assertLess(abs(v - vt), 0.05)                      # at its speed
        self.assertAlmostEqual(required_gap(0.0, 1.0, 2.0), 0.375)


class ProgramTest(unittest.TestCase):
    def test_the_layout_that_shows_slots_gets_the_merge_train(self):
        from plc_mock import make_program
        config = load_config("merge_demo.json")
        self.assertIsInstance(make_program(config), MergeTrainController)
        roles = MergeTrainController(config).roles()
        self.assertEqual(roles["merge_line"], "infeed")
        self.assertIn("port 0.00 m at 0 deg", roles["belt_1"])            # the straight one
        self.assertIn("port 2.00 m at 30 deg", roles["belt_2"])
        self.assertIn("1.70 m from the gate", roles["belt_3"])
        # the fourth side feed: found from the geometry, whatever its sensors are called
        self.assertIn("gate belt_4_gate_2, section belt_5_b+belt_5_c+belt_5_d+belt_5_e+belt_5", roles["belt_5"])
        self.assertIn("port 5.50 m at 30 deg", roles["belt_5"])
        for t in config.trackedLoads:
            t.role = "load"
        self.assertNotIsInstance(make_program(config), MergeTrainController)


class ClosedLoopTest(unittest.TestCase):
    """The program against the plant's physics (no network), 60 s, with the sync sections'
    drives at 2, 3 and 4 m/s^2 (the user: it must work at each)."""

    def run_line(self, accel):
        random.seed(1)
        config = load_config("merge_demo.json")
        # (found on a copy: a controller writes its outputs into the layout - a stopped belt's speed)
        sections = {b.id for cl in MergeTrainController(load_config("merge_demo.json")).clients for b in cl.section}
        for belt in config.belts:
            if belt.id in sections:
                belt.drive.accel = belt.drive.decel = accel
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        t, dt, min_gap, landed, cars = 0.0, 0.01, math.inf, {}, {}
        for i in range(6000):
            engine.update(dt)
            t += dt
            c.step(t)
            on = sorted((v for v in engine.parcel_views() if 0.3 < v.x < 7.8 and abs(v.y) < 0.3), key=lambda v: v.x)
            for a, b in zip(on, on[1:]):
                extent = [max(math.dist(v.points[0], v.points[1]), math.dist(v.points[1], v.points[2]))
                          for v in (a, b)]
                min_gap = min(min_gap, (b.x - a.x) - sum(extent) / 2.0)
            for car in c.train.cars:                            # each car past 7 m: its parcels
                if 6.9 <= car.s < 7.05 and id(car) not in cars:
                    cars[id(car)] = (car.state, sum(1 for v in on if abs(v.x - car.s) < c.train.pitch / 2.0))
            if i % 20 == 0:                                     # a landed load where its car is
                for car in c.train.cars:
                    if car.state == "occupied" and 4.6 <= car.s <= 7.4 and car.load not in landed:
                        near = min(on, key=lambda v: abs(v.x - car.s), default=None)
                        if near is not None and abs(near.x - car.s) < 0.6:          # head to head:
                            front = near.x + max(math.dist(near.points[0], near.points[1]),
                                                 math.dist(near.points[1], near.points[2])) / 2.0
                            landed[car.load] = (car.client, front - (car.s + FRONT_IN_CAR_M))
        return c, engine, min_gap, landed, cars

    def test_every_induction_lands_its_loads_in_their_cars(self):
        for accel in (2.0, 3.0, 4.0):
            with self.subTest(accel=accel):
                c, engine, min_gap, landed, cars = self.run_line(accel)
                for client in c.clients:
                    self.assertGreaterEqual(client.landed, 3, c.summary())     # every induction served
                    self.assertEqual(client.missed, 0, c.summary())
                self.assertGreater(len(landed), 20)
                for state, parcels in cars.values():                         # one load to a car
                    self.assertEqual(parcels, 1 if state == "occupied" else 0, sorted(cars.values()))
                # head to head, on its car (1.2 m): at 2 m/s a load waits too close to its port for the
                # whole ramp at 2 or 3 m/s^2 - it ramps at constant acceleration and catches up after the
                # port; within 0.2 m of its target (the left side lags 0.05 - 0.12 m: a side offset)
                worst = max(abs(e) for _, e in landed.values())
                self.assertLess(worst, 0.2, sorted(landed.values()))
                self.assertGreater(min_gap, 0.4)                               # the loads on the line apart
                self.assertGreater(sum(engine.delivered.values()), 10)

    def test_a_plc_clock_not_the_plants_moves_no_car_off_its_load(self):
        """The PLC's clock 1.5 x the plant's (a simulator at 0.67 of real time): distances from
        the encoders and time from the line's travel keep every car on its parcel - with the
        CPU's clock the cars drifted and loads landed in occupied cars (two or three to a car)."""
        random.seed(1)
        config = load_config("merge_demo.json")
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        t, dt, seen = 0.0, 0.01, {}
        for _ in range(4000):                                               # 40 s
            engine.update(dt)
            t += dt
            c.step(t * 1.5)
            on = [v for v in engine.parcel_views() if abs(v.y) < 0.35 and 0.0 < v.x < 8.0]
            for car in c.train.cars:
                if 6.9 <= car.s < 7.05 and id(car) not in seen:
                    seen[id(car)] = (car.state, sum(1 for v in on if abs(v.x - car.s) < c.train.pitch / 2.0))
        self.assertGreater(len(seen), 30)
        for state, parcels in seen.values():
            self.assertEqual(parcels, 1 if state == "occupied" else 0, sorted(seen.values()))

    def test_a_50_ms_scan_gives_no_car_back(self):
        """plc_mock's default scan (50 ms): a car moves 0.1 m a scan, past the short ramp's exact
        start - it was given back and granted again, over and over (the slots seen appear and
        vanish); now it starts at the first scan in time."""
        random.seed(1)
        config = load_config("merge_demo.json")
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        t, cars = 0.0, {}
        for i in range(6000):                                               # 60 s
            engine.update(0.01)
            t += 0.01
            if i % 5 == 0:
                c.step(t)
            on = [v for v in engine.parcel_views() if abs(v.y) < 0.35 and 0.0 < v.x < 8.0]
            for car in c.train.cars:
                if 6.9 <= car.s < 7.05 and id(car) not in cars:
                    cars[id(car)] = (car.state, sum(1 for v in on if abs(v.x - car.s) < c.train.pitch / 2.0))
        self.assertEqual(sum(cl.released for cl in c.clients), 0, c.summary())
        self.assertEqual(sum(cl.missed for cl in c.clients), 0, c.summary())
        self.assertGreater(sum(cl.landed for cl in c.clients), 60, c.summary())
        for state, parcels in cars.values():
            self.assertEqual(parcels, 1 if state == "occupied" else 0, sorted(cars.values()))

    def test_a_load_longer_than_the_longest_gets_no_car_and_is_held(self):
        """A 1.5 m load (longer than max_length, 1.2 m): it runs through the gate, the gate still
        blocked past 1.2 m - its request withdrawn, its feed held; taken away, the feed goes on."""
        config = load_config("merge_demo.json")
        for s in config.sources:
            if s.id == "src_belt_2":
                s.interval = 1e9                               # this feed: only the long load
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        feed = next(cl for cl in c.clients if cl.name == "belt_2")
        g = c.geo["belt_2_a"]                                  # on its metering belt, before the gate
        (x0, y0, _), (x1, y1, _) = g["start"], g["end"]
        long_one = engine.add_parcel(x0 + 0.4 * (x1 - x0), y0 + 0.4 * (y1 - y0), 1.5, 0.3, g["heading_end"])   # front short of the gate
        t = 0.0

        def run(seconds):
            nonlocal t
            for _ in range(int(seconds / 0.01)):
                engine.update(0.01)
                t += 0.01
                c.step(t)

        run(6.0)
        self.assertEqual(feed.oversize, 1, c.summary())
        self.assertEqual(feed.state, "oversize")
        self.assertIsNone(feed.car)                            # no car for it
        self.assertEqual(feed.landed, 0)
        self.assertLess(abs(float(feed.section[0].actualSpeed or 0.0)), 0.02)     # held
        engine.parcels.remove(long_one)                        # the operator takes it away
        engine._remove_parcel(long_one)
        run(1.0)
        self.assertNotEqual(feed.state, "oversize")
        self.assertEqual(feed.queue, [])

    def test_the_gate_measures_each_load_along_its_travel(self):
        """Each load's length from its gate (the travel from its front to its tail passing) against
        the parcel's extent along the travel: an IR eye's 50 mm spot taken off, half a scan's travel
        at each edge, both belts at the induction speed while it crosses."""
        random.seed(1)
        config = load_config("merge_demo.json")
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        errors, crossing, t = {}, {cl.name: None for cl in c.clients}, 0.0
        for _ in range(4000):                                   # 40 s
            engine.update(0.01)
            t += 0.01
            c.step(t)
            for cl in c.clients:
                load = crossing[cl.name]
                if load is not None and cl.crossing is None:      # its tail just passed: measured
                    gx, gy, h = cl.origin
                    ux, uy = math.cos(h), math.sin(h)
                    for v in engine.parcel_views():
                        s = (v.x - gx) * ux + (v.y - gy) * uy
                        if abs(-(v.x - gx) * uy + (v.y - gy) * ux) > 0.3:
                            continue
                        true = 2.0 * max(abs((px - v.x) * ux + (py - v.y) * uy) for px, py in v.points)
                        if -0.02 <= s - true / 2.0 <= 0.1:          # the one whose tail just left the gate
                            errors.setdefault(cl.name, []).append(load.length - true)
                            break
                crossing[cl.name] = cl.crossing
        self.assertEqual(len(errors), 5, sorted(errors))
        for name, e in errors.items():
            self.assertLess(abs(sum(e) / len(e)), 0.02, f"{name}: {[round(x, 3) for x in e]}")
            self.assertLess(max(abs(x) for x in e), 0.05, f"{name}: {[round(x, 3) for x in e]}")

    def test_a_landed_loads_box_keeps_its_turn_and_sits_on_its_parcel(self):
        """The track box of a landed load: its chain's heading (a load keeps its turn on the line,
        like the parcel), its measured size (a banner's width too), its centre where it came in."""
        random.seed(1)
        config = load_config("merge_demo.json")
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        t, dist, turn = 0.0, [], []
        for i in range(4000):                                   # 40 s
            engine.update(0.01)
            t += 0.01
            c.step(t)
            if i % 25:
                continue
            views = [v for v in engine.parcel_views() if 0.5 < v.x < 7.5 and abs(v.y) < 0.4]
            for row in config.trackedLoads:
                if not row.active or row.role != "load" or not 0.5 < row.loadX < 7.5:
                    continue
                near = min(views, key=lambda v: math.hypot(v.x - row.loadX, v.y - row.loadY), default=None)
                if near is None or math.hypot(near.x - row.loadX, near.y - row.loadY) > 0.6:
                    continue
                (x0, y0), (x1, y1) = near.points[0], near.points[1]
                d = (math.degrees(math.atan2(y1 - y0, x1 - x0)) - row.loadYaw) % 90.0
                turn.append(min(d, 90.0 - d))
                dist.append(math.hypot(near.x - row.loadX, near.y - row.loadY))
        self.assertGreater(len(dist), 100)
        self.assertLess(max(turn), 2.0)                         # turned as its parcel (was 30 degrees off)
        dist.sort()
        self.assertLess(dist[len(dist) // 2], 0.05)            # its centre on its parcel's (was 92 mm)
        self.assertLess(dist[-1], 0.12)


class CommissioningTest(unittest.TestCase):
    """Each feed's landing offset, measured in the physics (plc_merge.commission), takes its loads
    onto their car's point - head aligned (the left side lagged 0.1 m) and center aligned."""

    def test_the_offsets_take_every_feed_onto_its_point(self):
        from plc_merge import commission, measure_landings
        for align in ("head", "center"):
            with self.subTest(align=align):
                offsets = commission(load_config("merge_demo.json"), 60.0, align)
                self.assertEqual(len(offsets), 5, offsets)                 # every feed measured
                errors = measure_landings(load_config("merge_demo.json"), 60.0, align, offsets)
                for name, e in errors.items():
                    self.assertTrue(e, name)
                    mean = sum(e) / len(e)
                    self.assertLess(abs(mean), 0.04, f"{name}: {mean * 1000:+.0f} mm with {offsets}")
                    self.assertLess(max(abs(x) for x in e), 0.2, name)


if __name__ == "__main__":
    unittest.main()
