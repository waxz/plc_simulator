"""
test_wheel_sort.py - the mock PLC's wheel diverter control (wheel_sort.py): a
plan of directions from the scanner, each parcel tracked from the IR eye by
the line's travel and the wheels' speed and angle, the wheels turned only
when the parcel before is off the deck - and on merge_demo.json, the parcels
reach the chute of their destination.
"""
import math
import os
import random
import unittest

import wheel_sort
from config_parser import Diverter, Scanner, Sensor, load_config

HERE = os.path.dirname(os.path.abspath(__file__))
V = 2.0                # the line, m/s
DT = 0.01              # the scan


class Rig:
    """A deck 1 m long (u -0.5 .. 0.5), its eye at u -0.75 (5 cm beam), the scanner before it;
    parcels as (front, length) moving along u at the pace the wheels keep."""

    def __init__(self):
        self.d = Diverter(id="d", x=0.0, y=0.0, width=1.0, height=0.5, diverterType="wheels",
                          divertAngle=-30.0, speed=2.3094, destination=1)
        self.eye = Sensor(id="eye", x=-0.75, y=0.0, width=0.05, height=0.6, rotation=0.0, label="eye", color="#ffffff",
                          sensorType="ir")
        self.scanner = Scanner(id="scan", x=-2.0, y=0.0)
        self.line = type("Belt", (), {"id": "line", "actualSpeed": V})()
        self.s = wheel_sort.WheelSorter(self.d, self.eye, self.scanner, self.line, -0.75, spot=0.05)
        self.parcels = []                    # [front, length, destination, read]
        self.events = []
        self.turns = []                      # (time, angle, fronts on the deck then)
        self.t = 0.0

    def add(self, front, length, destination):
        self.parcels.append([front, length, destination, False])

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.t += DT
            for p in self.parcels:
                p[0] += V * DT
                if not p[3] and p[0] >= -2.0:            # the scanner reads it
                    p[3] = True
                    self.scanner.readCount += 1
                    self.scanner.destination = p[2]
            front_blocked = any(p[0] >= -0.775 and p[0] - p[1] <= -0.725 for p in self.parcels)
            self.eye.sensor_states = front_blocked
            before = self.s.angle_cmd
            self.s.step(DT, V * DT, V, True, self.events)
            if self.s.angle_cmd != before:
                on = [(round(p[0], 3), p[1]) for p in self.parcels if p[0] > -0.5 and p[0] - p[1] < 0.5]
                self.turns.append((round(self.t, 2), self.s.angle_cmd, on))


class WheelSortTest(unittest.TestCase):
    def test_the_plan_follows_the_reads(self):
        r = Rig()
        r.scanner.readCount = 0
        r.s.last_read = 0
        for i, dest in enumerate((2, 1, 2, 2, 1, 1)):
            r.add(-2.5 - 1.6 * i, 0.4, dest)
        r.run(0.3)
        self.assertEqual(list(r.s.plan), [0.0])                         # the first read
        planned = [0.0, -30.0, 0.0, 0.0, -30.0, -30.0]
        for _ in range(12):
            r.run(0.5)                                    # what is coming: tracked, then still planned
            self.assertEqual([i.angle for i in r.s.items] + list(r.s.plan),
                             planned[r.s.off_deck:r.scanner.readCount])
        self.assertEqual(r.s.off_deck, 6)

    def test_it_tracks_the_front_and_measures_the_length(self):
        r = Rig()
        r.s.last_read = 0
        r.add(-1.0, 0.45, 2)
        r.run(0.4)
        item = r.s.items[0]
        self.assertAlmostEqual(item.length, 0.45, delta=0.02)
        self.assertAlmostEqual(item.head, r.parcels[0][0], delta=0.02)

    def test_the_wheels_turn_only_when_the_parcel_before_is_off_the_deck(self):
        r = Rig()
        r.s.last_read = 0
        # straight, divert, straight: 1.6 m apart, 0.5 m long
        for i, dest in enumerate((2, 1, 2)):
            r.add(-2.2 - 1.6 * i, 0.5, dest)
        r.run(4.0)
        self.assertEqual([a for _t, a, _on in r.turns], [-30.0, 0.0])
        # to -30 once the first (straight) is past the deck's end: its tail at 0.5 (one scan's play)
        _t, _a, on = r.turns[0]
        first = r.parcels[0]
        self.assertNotIn(True, [abs(f - first[0]) < 1e-6 for f, _l in on])   # the first is not on it
        self.assertEqual(r.s.items[-1].angle if r.s.items else 0.0, 0.0)

    def test_no_plan_goes_straight_on(self):
        r = Rig()
        r.s.scanner = None
        r.add(-1.0, 0.4, 1)
        r.run(0.3)
        self.assertEqual(r.s.items[0].angle, 0.0)
        self.assertEqual(r.s.no_plan, 1)
        self.assertTrue(any("without a plan" in e for e in r.events))

    def test_commands(self):
        r = Rig()
        r.s.last_read = 0
        r.add(-2.1, 0.4, 1)
        r.run(0.75)                                       # read, then at the eye: its direction, -30
        self.assertTrue(r.d.enable)
        self.assertEqual(r.d.commandAngle, -30.0)
        self.assertAlmostEqual(r.d.commandSpeed, V / math.cos(math.radians(30.0)), places=6)   # the line's pace


class RowsTest(unittest.TestCase):
    """A deck steered row by row (rowControl: a swivel motor per wheel row - 6 on a 1 m deck): each
    row turns for the next parcel the moment the one before is clear of it."""

    def test_a_row_turns_once_the_parcel_before_is_clear_of_it(self):
        r = Rig()
        r.d.rowControl = True
        r.s = wheel_sort.WheelSorter(r.d, r.eye, r.scanner, r.line, -0.75, spot=0.05)
        r.s.last_read = 0
        r.add(-2.2, 0.4, 1)                       # diverted
        r.add(-3.4, 0.4, 2)                       # straight, 1.2 m behind
        seen = []
        for _ in range(150):
            r.run(0.01)
            seen.append(list(r.s.row_cmd))
            # no row ever turns while a parcel that is not clear of it wants the other way
            for i, angle in enumerate(r.s.row_cmd):
                owner = next((it for it in r.s.items if not r.s.clear_of(it, i)), None)
                if owner is not None:
                    self.assertEqual(angle, owner.angle)
        # at some moment the first rows already point straight for the second parcel while the
        # last still divert the first
        self.assertTrue(any(rows[0] == 0.0 and rows[-1] == -30.0 for rows in seen), seen[::10])
        self.assertEqual(r.d.rowAngles, r.s.row_cmd)
        self.assertEqual(len(r.d.rowSpeeds), 6)

    def test_the_simulation_moves_a_parcel_the_average_way_of_its_rows(self):
        import logistics
        from config_parser import SimulatorConfig
        from physics import PhysicsEngine
        d = Diverter(id="d", x=0.0, y=0.0, width=1.2, height=0.6, diverterType="wheels", rowControl=True,
                     divertAngle=-30.0, speed=2.0, control="manual")      # 1.2 m: 7 rows of wheels
        e = PhysicsEngine(SimulatorConfig(diverters=[d], useSurfaceVelocity=True))
        d.enable = True
        self.assertEqual(logistics.wheel_rows(d), 7)
        d.rowSpeeds, d.rowAngles = [1.0] * 7, [0.0, 0.0, 0.0, 0.0, -30.0, -30.0, -30.0]
        e.run_for(0.3)                            # the rows swivel
        self.assertEqual(d.rowWheelAngles, [0.0, 0.0, 0.0, 0.0, -30.0, -30.0, -30.0])
        p = e.add_parcel(0.0857, 0.0, 0.5, 0.3)   # half on rows 0-3, half on rows 4-6
        v = e.logistics._deck_velocity(d, p)
        self.assertAlmostEqual(math.degrees(math.atan2(v[1], v[0])), -15.0, delta=4.0)
        d.rowAngles = [-30.0] * 7
        e.run_for(0.3)
        v = e.logistics._deck_velocity(d, p)
        self.assertAlmostEqual(math.degrees(math.atan2(v[1], v[0])), -30.0, delta=1.0)

    def test_on_the_exchange_a_speed_and_an_angle_per_row(self):
        import io_signals
        import st_export
        config = load_config(os.path.join(HERE, "merge_demo.json"))
        d = config.diverters[0]
        self.assertTrue(d.rowControl)                   # 1 m: 6 rows of wheels, a swivel motor each
        self.assertEqual([(s.name, s.count) for s in io_signals.signals(d) if s.direction == "command"],
                         [("enable", 1), ("rowSpeeds", 6), ("rowAngles", 6)])
        structs = st_export.device_structs(config)
        self.assertEqual(structs["WheelDeck6_Command"],
                         [("enable", "BOOL"), ("rowSpeeds", "ARRAY[0..5] OF REAL"), ("rowAngles", "ARRAY[0..5] OF REAL")])
        self.assertNotIn("wheels", [b.id for b in config.belts])      # the deck carries: no belt under it

    def test_a_layout_with_rows_loads_as_row_control(self):
        d = Diverter.model_validate({"id": "d", "x": 0, "y": 0, "diverterType": "wheels", "rows": 4})
        self.assertTrue(d.rowControl)
        self.assertNotIn("rows", d.model_dump())


class WheelGridTest(unittest.TestCase):
    """The wheels of a deck: rows along the travel, wheels in a row, the pitch each way - chosen,
    or fitted to the deck (0)."""

    def test_fitted_and_chosen(self):
        import equipment_geometry as geo
        d = Diverter(id="d", x=0.0, y=0.0, width=1.0, height=0.5, diverterType="wheels", rowControl=True)
        us, vs = geo.grid_of(d)
        self.assertEqual((len(us), len(vs)), (6, 4))                          # fitted: as before
        self.assertAlmostEqual(us[1] - us[0], 0.15)
        d.wheelRows, d.wheelColumns, d.wheelPitchAlong, d.wheelPitchAcross = 8, 3, 0.12, 0.14
        us, vs = geo.grid_of(d)
        self.assertEqual((len(us), len(vs)), (8, 3))
        self.assertAlmostEqual(us[0], -0.42)                                  # centred: (8 - 1) / 2 x 0.12
        self.assertAlmostEqual(vs[2] - vs[1], 0.14)
        from device_kinds import wheel_rows
        self.assertEqual(wheel_rows(d), 8)                                    # a swivel motor per row
        bounds = geo.row_bounds(d.width, us)
        self.assertEqual((bounds[0][0], bounds[-1][1]), (-0.5, 0.5))          # the strips cover the deck
        self.assertAlmostEqual(bounds[0][1], -0.36)                           # midway between rows 0 and 1
        self.assertEqual(geo.row_at(bounds, 0.0), 4)

    def test_the_sorter_and_the_simulation_use_the_chosen_rows(self):
        d = Diverter(id="d", x=0.0, y=0.0, width=1.0, height=0.5, diverterType="wheels", rowControl=True,
                     wheelRows=4, wheelPitchAlong=0.2, divertAngle=-30.0, destination=1)
        r = Rig()
        r.d = d
        r.s = wheel_sort.WheelSorter(d, r.eye, r.scanner, r.line, -0.75, spot=0.05)
        self.assertEqual(r.s.rows, 4)
        self.assertAlmostEqual(r.s.row_end(0), -0.2)                          # rows at -0.3, -0.1, 0.1, 0.3
        self.assertEqual(r.s.row_at(0.45), 3)

    def test_a_grid_larger_than_the_deck_is_flagged(self):
        from cad.checks import check_layout
        from config_parser import SimulatorConfig
        d = Diverter(id="d", x=0.0, y=0.0, width=1.0, height=0.5, diverterType="wheels", wheelRows=10,
                     wheelPitchAlong=0.15)
        issues = [i for i in check_layout(SimulatorConfig(diverters=[d])) if i.kind == "wheels off the deck"]
        self.assertEqual([i.item_ids for i in issues], [["d"]])
        d.wheelRows = 6
        self.assertFalse([i for i in check_layout(SimulatorConfig(diverters=[d])) if i.kind == "wheels off the deck"])


class MergeDemoTest(unittest.TestCase):
    def test_each_parcel_reaches_its_destinations_chute(self):
        from physics import PhysicsEngine
        from plc_merge import MergeTrainController
        random.seed(1)
        config = load_config(os.path.join(HERE, "merge_demo.json"))
        engine = PhysicsEngine(config)
        controller = MergeTrainController(config, autostart=True)
        self.assertEqual([(s.diverter.id, s.eye.id, s.scanner.id) for s in controller.sorters],
                         [("div_1", "infeed_jam", "scan_1")])
        routes, take = [], engine.handle_sinks

        def tracked():
            before = [(p, getattr(p, "destination", 0), tuple(p.body.position)) for p in engine.parcels]
            take()
            alive = {id(p) for p in engine.parcels}
            for p, dest, (x, y) in before:
                if id(p) not in alive:
                    routes.append((dest, min(config.sinks, key=lambda s: math.hypot(x - s.x, y - s.y)).id))
        engine.handle_sinks = tracked
        t = 0.0
        for i in range(9000):
            engine.update(0.01)
            t += 0.01
            controller.step(t)
        expected = {1: "chute_1", 2: "chute_2"}
        wrong = [r for r in routes if expected.get(r[0]) != r[1]]
        self.assertGreater(len(routes), 140)
        self.assertLessEqual(len(wrong), 2, wrong)      # a short parcel right behind a diverted one may not make it
        self.assertIn("div_1: plan [", controller.summary())


if __name__ == "__main__":
    unittest.main()
