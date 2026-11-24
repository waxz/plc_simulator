"""
test_source_area.py - a parcel source makes a parcel only while its
detection area is clear (config_parser.Source.clear_area): an interval
shorter than the line can take no longer piles parcels at the start of the
belt, on a stopped belt or a moving one, straight or at an angle.
"""
import math
import random
import unittest

from config_parser import Source, load_config
from physics import PhysicsEngine
from plc_merge import MergeTrainController


def near(engine, source, radius):
    return [p for p in engine.parcels if math.dist(p.body.position, (source.x, source.y)) < radius]


class SourceAreaTest(unittest.TestCase):
    def test_the_automatic_area_is_the_largest_parcel_and_a_gap(self):
        s = Source(id="s", x=0, y=0, rotation=0, interval=1, minWidth=0.3, maxWidth=0.4, minHeight=0.2,
                   maxHeight=0.35, label="")
        self.assertEqual(s.clear_area(), (0.8, 0.4))
        s.clearLength, s.clearWidth = 1.5, 0.6
        self.assertEqual(s.clear_area(), (1.5, 0.6))

    def test_a_short_interval_on_a_stopped_belt_makes_one_parcel(self):
        random.seed(2)
        config = load_config("merge_demo.json")             # no PLC: every belt stands
        for source in config.sources:
            source.interval = 0.5
        engine = PhysicsEngine(config)
        for _ in range(500):                                # 5 s: ten intervals
            engine.update(0.01)
        for source in config.sources:                       # the straight one and the 30-degree ones
            self.assertEqual(len(near(engine, source, 1.5)), 1, source.id)

    def test_on_a_moving_belt_the_parcels_keep_the_areas_gap(self):
        random.seed(3)
        config = load_config("merge_demo.json")
        source = next(s for s in config.sources if s.id == "src_belt_5")
        source.interval = 0.5
        engine = PhysicsEngine(config)
        c = MergeTrainController(config, autostart=True)
        t, smallest = 0.0, math.inf
        for _ in range(3000):                               # 30 s
            engine.update(0.01)
            t += 0.01
            c.step(t)
            on = sorted((v for v in engine.parcel_views() if -2.5 < v.x < 0.0 and abs(v.y) < 0.3),
                        key=lambda v: v.x)
            for a, b in zip(on, on[1:]):
                extent = [max(math.dist(v.points[0], v.points[1]), math.dist(v.points[1], v.points[2]))
                          for v in (a, b)]
                smallest = min(smallest, (b.x - a.x) - sum(extent) / 2.0)
        self.assertGreater(smallest, 0.1)                   # never touching (the area's gap: 0.2 m)
        self.assertGreater(c.clients[0].landed, 3)


if __name__ == "__main__":
    unittest.main()
