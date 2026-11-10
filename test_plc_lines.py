"""
test_plc_lines.py - the line-based PLC program (plc_logic.LineController):
lines found from names, per-line Start/Stop/Reset and lamps, the start
interlock (a feeder runs only while the line it feeds runs), feeders holding
parcels at the merge, per-line jams and drive faults - first on signals,
then closed loop with the physics engine on station_1.json (L1 main line,
L2 / L3 feeders).
"""
import unittest

import devices
from config_parser import SimulatorConfig, load_config
from physics import PhysicsEngine
from plc_logic import DeliveryController, LineController, find_lines, line_of, make_controller


class Signals:
    """Drive the controller like the I/O image would: buttons and sensors by label."""

    def __init__(self, test, jam_time=5.0):
        self.config = load_config("station_1.json")
        self.c = LineController(self.config, jam_time=jam_time)
        self.t = 0.0
        self.test = test

    def by_label(self, kind, label):
        return next(x for x in getattr(self.config, kind) if x.label == label)

    def step(self, dt=0.05):
        self.t += dt
        return self.c.step(self.t)

    def press(self, label):
        button = self.by_label("buttons", label)
        devices.press(button)
        events = self.step()
        devices.release(button)
        return events + self.step()

    def block(self, label, blocked=True):
        sensor = self.by_label("sensors", label)
        bits = len(sensor.sensor_states) if isinstance(sensor.sensor_states, str) else 1
        sensor.sensor_states = ("1" if blocked else "0") * bits if isinstance(sensor.sensor_states, str) else blocked

    def belts_moving(self, line):
        return [abs(b.speed) > 0 for b in self.c.lines[line].belts]

    def lamps(self, line):
        return {led.label.split()[1]: led.isOn for led in self.c.lines[line].leds}


class StructureTest(unittest.TestCase):
    def test_lines_main_line_and_merges_from_the_layout(self):
        config = load_config("station_1.json")
        self.assertEqual(line_of(next(b for b in config.buttons if b.label == "L2 START")), "L2")
        self.assertEqual(line_of(config.belts[0]), "L1")                      # belt_1_1_1
        lines = find_lines(config)
        self.assertEqual(list(lines), ["L1", "L2", "L3"])
        c = LineController(config)
        self.assertEqual(c.main.name, "L1")                                  # reaches the outfeed
        self.assertEqual((c.lines["L2"].feeds, c.lines["L2"].hold_sensor, c.lines["L2"].merge_sensor),
                         ("L1", "sensor_1_2_1", "sensor_1_1_1"))
        self.assertEqual(c.lines["L3"].hold_sensor, "sensor_1_3_1")

    def test_make_controller_picks_by_layout(self):
        self.assertIsInstance(make_controller(load_config("station_1.json")), LineController)
        self.assertIsInstance(make_controller(SimulatorConfig()), DeliveryController)


class SignalTest(unittest.TestCase):
    def setUp(self):
        self.s = Signals(self)

    def test_feeders_need_the_main_line_and_lines_start_and_stop_on_their_own(self):
        s = self.s
        events = s.press("L2 START")
        self.assertIn("start L1, the line it feeds, first", " ".join(events))
        self.assertFalse(any(s.belts_moving("L2")))
        self.assertEqual(s.lamps("L2"), {"AUTO": False, "MANUAL": True, "ERROR": False})
        s.press("L1 START")
        self.assertTrue(all(s.belts_moving("L1")))
        self.assertFalse(any(s.belts_moving("L2")))                          # only L1
        s.press("L2 START")
        self.assertTrue(all(s.belts_moving("L2")))
        self.assertEqual(s.lamps("L2"), {"AUTO": True, "MANUAL": False, "ERROR": False})
        s.press("L2 STOP")                                                    # an NC contact
        self.assertFalse(any(s.belts_moving("L2")))
        self.assertTrue(all(s.belts_moving("L1")))
        self.assertIn("L1 RUN | L2 STOP | L3 STOP", s.c.summary())

    def test_a_feeder_holds_its_parcel_while_the_main_line_is_stopped(self):
        s = self.s
        for label in ("L1 START", "L2 START"):
            s.press(label)
        s.press("L1 STOP")
        self.assertTrue(all(s.belts_moving("L2")))                            # brings parcels up...
        s.block("L2 BANNER")
        events = s.step()
        self.assertIn("L2 HOLD at sensor_1_2_1 (L1 stop)", events)
        self.assertFalse(any(s.belts_moving("L2")))                           # ...and holds them there
        self.assertEqual(s.c.lines["L2"].state, "HOLD")
        self.assertTrue(s.lamps("L2")["AUTO"])                                # still in auto
        for _ in range(200):                                                  # 10 s > jam_time: not a jam
            s.step()
        self.assertIsNone(s.c.lines["L2"].jammed)
        events = s.press("L1 START")
        self.assertIn("L2 released into L1", events)
        self.assertTrue(all(s.belts_moving("L2")))

    def test_a_feeder_gives_way_to_a_parcel_at_the_merge(self):
        s = self.s
        for label in ("L1 START", "L2 START"):
            s.press(label)
        s.block("L2 BANNER")
        s.block("L1 IR")
        events = s.step()
        self.assertFalse(any(s.belts_moving("L2")))                           # main line has priority
        self.assertTrue(all(s.belts_moving("L1")))
        self.assertEqual(s.c.lines["L2"].yields, 1)
        self.assertFalse(any("HOLD" in e for e in events))                    # routine: counted, not logged
        s.block("L1 IR", False)
        s.step()
        self.assertTrue(all(s.belts_moving("L2")))
        self.assertIn("L2 RUN (gave way 1x)", s.c.summary())

    def test_a_jam_stops_only_its_line_until_reset(self):
        s = self.s
        for label in ("L1 START", "L2 START", "L3 START"):
            s.press(label)
        s.block("L3 BANNER")                                                  # L1 running: no hold, a jam
        for _ in range(110):
            s.step()
        self.assertEqual(s.c.lines["L3"].jammed, "sensor_1_3_1")
        self.assertFalse(any(s.belts_moving("L3")))
        self.assertTrue(all(s.belts_moving("L1")) and all(s.belts_moving("L2")))
        self.assertEqual(s.lamps("L3"), {"AUTO": False, "MANUAL": False, "ERROR": True})
        self.assertIn("remove the parcel first", " ".join(s.press("L3 RESET")))
        s.block("L3 BANNER", False)
        self.assertIn("L3: jam at sensor_1_3_1 cleared (button_1_3_3)", s.press("L3 RESET"))
        s.press("L3 START")
        self.assertTrue(all(s.belts_moving("L3")))

    def test_a_drive_fault_on_the_main_line_makes_the_feeders_hold(self):
        s = self.s
        for label in ("L1 START", "L2 START"):
            s.press(label)
        s.config.belts[0].fault = True                                        # belt_1_1_1 trips
        self.assertIn("L1 FAULT on belt_1_1_1: drive tripped - line stopped", s.step())
        self.assertEqual(s.lamps("L1")["ERROR"], True)
        s.block("L2 BANNER")
        self.assertIn("L2 HOLD at sensor_1_2_1 (L1 fault)", s.step())
        self.assertIn("can't start, drive fault", " ".join(s.press("L1 START")))
        s.config.belts[0].fault = False
        s.step()
        s.press("L1 START")
        self.assertTrue(all(s.belts_moving("L1")))
        self.assertTrue(all(s.belts_moving("L2")))                            # released


class ClosedLoopTest(unittest.TestCase):
    """The program against the simulated plant (no network)."""

    def test_parcels_are_delivered_and_feeders_hold_when_the_main_line_stops(self):
        config = load_config("station_1.json")
        # Quicker than the plant: belts at 2 m/s (the program's design speed),
        # a third of the parcels - same behaviour, a fraction of the run time.
        for belt in config.belts:
            belt.speed = 2.0
        for source in config.sources:
            source.interval *= 3
        engine = PhysicsEngine(config)
        c = LineController(config, jam_time=5.0)
        buttons = {b.label: b for b in config.buttons}
        t = 0.0

        def run(seconds):
            nonlocal t
            for _ in range(int(seconds / 0.05)):
                engine.update(0.05)
                t += 0.05
                c.step(t)

        def press(label):
            devices.press(buttons[label])
            run(0.2)
            devices.release(buttons[label])
            run(0.2)

        for label in ("L1 START", "L2 START", "L3 START"):
            press(label)
        run(20)
        self.assertGreater(sum(engine.delivered.values()), 0)          # ~30 m of line at 2 m/s
        self.assertEqual({line.state for line in c.lines.values()} - {"RUN", "HOLD"}, set())   # no jams
        self.assertGreater(c.lines["L2"].yields + c.lines["L3"].yields, 0)
        press("L1 STOP")
        for _ in range(40):                     # feeders bring their next parcel up, then hold it
            run(0.5)
            if c.lines["L2"].state == c.lines["L3"].state == "HOLD":
                break
        self.assertEqual([c.lines[n].state for n in ("L1", "L2", "L3")], ["STOP", "HOLD", "HOLD"])
        run(6)                                  # held, not jammed
        self.assertEqual([c.lines[n].state for n in ("L1", "L2", "L3")], ["STOP", "HOLD", "HOLD"])
        delivered = sum(engine.delivered.values())
        press("L1 START")
        run(15)
        self.assertGreater(sum(engine.delivered.values()), delivered)
        self.assertNotIn("JAM", c.summary())


if __name__ == "__main__":
    unittest.main()
