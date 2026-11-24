"""A chute (a sink with a capacity) counts the loads it takes before they
are deleted, so `full` comes on at its capacity although every load that
reached it is gone; a full chute takes no more (they wait in front of it)
until it is emptied - after emptyTime by itself, or on the PLC's `empty`."""
import unittest

from config_parser import Belt, SimulatorConfig, Sink
from physics import PhysicsEngine


def chute_layout(empty_time=0.0):
    belt = Belt(id="feed", type="linear", x=0.0, y=0.0, length=4.0, beltWidth=0.6, rotation=0.0, speed=1.0,
                directionAngle=0.0, color="#64748b")
    # the box starts where the belt ends (x 2.0): a load leaving the belt is in it
    chute = Sink(id="chute", x=2.4, y=0.0, width=0.8, height=0.8, label="", capacity=3, emptyTime=empty_time)
    return SimulatorConfig(belts=[belt], sinks=[chute])


class ChuteFullTest(unittest.TestCase):
    def test_full_at_capacity_though_every_load_is_deleted(self):
        c = chute_layout()
        e = PhysicsEngine(c)
        chute = c.sinks[0]
        for k in range(5):                                   # five loads, 0.7 m apart, onto the running belt
            e.add_parcel(-1.6 + 0.7 * k, 0.0, 0.3, 0.3)
        e.run_for(8.0)
        self.assertEqual(e.delivered.get("chute", 0), 3, "it takes its capacity")
        self.assertEqual(chute.count, 3)
        self.assertTrue(chute.full, "full on although the 3 loads it took were deleted")
        self.assertEqual(len(e.parcels), 2, "the other 2 wait in front of the full chute")

    def test_emptied_by_the_plc_then_takes_the_waiting_loads(self):
        c = chute_layout()
        e = PhysicsEngine(c)
        chute = c.sinks[0]
        for k in range(5):
            e.add_parcel(-1.6 + 0.7 * k, 0.0, 0.3, 0.3)
        e.run_for(8.0)
        chute.empty = True                                   # the PLC's empty (a rising edge)
        e.run_for(0.1)
        chute.empty = False
        e.run_for(3.0)
        self.assertEqual(e.delivered.get("chute", 0), 5)
        self.assertEqual(chute.count, 2)
        self.assertFalse(chute.full)

    def test_emptied_by_itself_after_empty_time(self):
        c = chute_layout(empty_time=2.0)
        e = PhysicsEngine(c)
        chute = c.sinks[0]
        for k in range(5):
            e.add_parcel(-1.6 + 0.7 * k, 0.0, 0.3, 0.3)
        t_full = None
        while e.sim_time < 6.0 and t_full is None:
            e.run_for(0.02)
            if chute.full:
                t_full = e.sim_time
        self.assertIsNotNone(t_full, "full at its capacity")
        e.run_for(1.8)
        self.assertTrue(chute.full, "still full before emptyTime")
        e.run_for(0.4)
        self.assertEqual(e.delivered.get("chute", 0), 5, "emptied after emptyTime, it took the waiting loads")
        self.assertFalse(chute.full)


def run_five(mode, empty_time=0.0, seconds=8.0):
    c = chute_layout(empty_time)
    c.sinks[0].fillMode = mode
    e = PhysicsEngine(c)
    for k in range(5):
        e.add_parcel(-1.6 + 0.7 * k, 0.0, 0.3, 0.3)
    e.run_for(seconds)
    return c, e, c.sinks[0]


class FillModeTest(unittest.TestCase):
    """The chute's fill mode (Sink.fillMode): count, keep or delete."""

    def test_keep_the_loads_stay_in_the_bin_and_it_is_full_at_capacity(self):
        c, e, chute = run_five("keep")
        self.assertEqual(chute.count, 3)
        self.assertTrue(chute.full)
        self.assertEqual(e.delivered.get("chute", 0), 3)
        kept = [p for p in e.parcels if getattr(p, "binned_in", None)]
        self.assertEqual(len(kept), 3, "the 3 taken stay, in the bin")
        x0, x1 = chute.x - chute.width / 2, chute.x + chute.width / 2
        y0, y1 = chute.y - chute.height / 2, chute.y + chute.height / 2
        for p in kept:
            x, y = p.body.position
            self.assertTrue(x0 <= x <= x1 and y0 <= y <= y1, "in its slot inside the box")
        self.assertEqual(len({(round(p.body.position.x, 3), round(p.body.position.y, 3)) for p in kept}), 3,
                         "each in a slot of its own")
        self.assertEqual(len(e.parcels), 5, "the other 2 wait in front of it")
        before = [tuple(p.body.position) for p in kept]
        e.run_for(2.0)
        self.assertEqual(before, [tuple(p.body.position) for p in kept], "kept loads do not move")

    def test_keep_emptied_removes_the_bins_loads_and_takes_the_waiting(self):
        c, e, chute = run_five("keep")
        chute.empty = True
        e.run_for(0.1)
        chute.empty = False
        e.run_for(3.0)
        self.assertEqual(e.delivered.get("chute", 0), 5)
        self.assertEqual(chute.count, 2)
        self.assertFalse(chute.full)
        self.assertEqual(len([p for p in e.parcels if getattr(p, "binned_in", None)]), 2,
                         "the first 3 removed with the emptying, the 2 that waited now in it")

    def test_delete_every_load_goes_and_it_is_never_full(self):
        c, e, chute = run_five("delete")
        self.assertEqual(e.delivered.get("chute", 0), 5)
        self.assertEqual(len(e.parcels), 0)
        self.assertFalse(chute.full)
        self.assertEqual(chute.count, 5, "still counted")

    def test_the_switch_in_the_simulator_window(self):
        import io_signals
        c = chute_layout()
        self.assertIsNone(io_signals.apply(c, "chute", "fillMode", "keep"))
        self.assertEqual(c.sinks[0].fillMode, "keep")
        self.assertIn("one of", io_signals.apply(c, "chute", "fillMode", "pile"))
        self.assertIn("not a chute", io_signals.apply(c, "feed", "fillMode", "keep"))


if __name__ == "__main__":
    unittest.main()
