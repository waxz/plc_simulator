"""
test_complete_plant.py - complete_plant.json (tools/build_complete_plant.py):
every kind of equipment with the designer's newer layers (control panel,
drive trains, PLC tag tables), and the whole plant running under the PLC
program (plc_logic, what plc_mock.py runs): parcels sorted to their
destinations, pallets moved, robot, elevator and transfers working.
"""
import os
import random
import sys
import unittest

from config_parser import load_config
from device_kinds import DEVICE_KINDS
from physics import PhysicsEngine
from plc_logic import make_controller

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLANT = os.path.join(HERE, "complete_plant.json")


class CompletePlantTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(PLANT)

    def test_the_build_script_reproduces_it(self):
        sys.path.insert(0, os.path.join(HERE, "tools"))
        import build_complete_plant
        self.assertEqual(build_complete_plant.build().model_dump(), self.config.model_dump())

    def test_has_every_kind_and_the_newer_layers(self):
        c = self.config
        for name in DEVICE_KINDS:
            if name in ("plcValues", "watchdogs", "trackedLoads"):      # shown only, optional: a layout has them when its PLC shows values
                continue
            self.assertTrue(getattr(c, name), name)
        self.assertEqual({s.scannerType for s in c.scanners}, {"barcode", "rfid", "dws"})
        self.assertEqual({d.diverterType for d in c.diverters}, {"pusher", "wheels"})
        self.assertEqual({(b.type, b.surface) for b in c.belts},
                         {("linear", "belt"), ("linear", "roller"), ("curved", "belt"), ("curved", "roller")})
        self.assertEqual({s.sensorType for s in c.sensors}, {"ir", "laser_banner"})
        (panel,) = c.controlPanels
        self.assertEqual([e.kind for e in panel.elements], ["button", "button", "button", "lamp", "lamp", "buzzer"])
        self.assertLessEqual({"START", "STOP"}, {b.label for b in c.buttons})          # the panel's, for the PLC
        self.assertEqual({b.id for b in c.belts if b.commandFormat == "frequency"}, {"infeed", "sorter"})
        self.assertTrue(all(m.drive is not None for m in (*c.elevators, *c.turntables, *c.transfers)))
        self.assertEqual({t.table for t in c.plcTags},
                         {"Sorting", "Pallet loop", "Tote line", "Operator panel", "System"})

    def test_the_plant_runs_under_the_plc(self):
        random.seed(7)
        c = load_config(PLANT)
        e = PhysicsEngine(c)
        routes, take = [], e.handle_sinks

        def tracked():
            before = [(p, getattr(p, "destination", 0), getattr(p, "readable", True), tuple(p.body.position))
                      for p in e.parcels]
            take()
            alive = {id(p) for p in e.parcels}
            for p, destination, readable, (x, y) in before:
                if id(p) not in alive:
                    sink = next((s.id for s in c.sinks if abs(x - s.x) <= s.width / 2 and abs(y - s.y) <= s.height / 2),
                                None)
                    routes.append((destination, readable, sink))
        e.handle_sinks = tracked
        plc = make_controller(c, autostart=True)
        while e.sim_time < 120.0:
            plc.step(now=e.sim_time)
            e.run_for(0.02)
        chutes = {1: "chute_1", 2: "chute_2", 3: "chute_3"}
        sorted_ = [(d, r, s) for d, r, s in routes if s in chutes.values()]
        self.assertGreater(len(sorted_), 30)
        self.assertEqual([x for x in sorted_ if chutes.get(x[0]) != x[2]], [])          # each to its own chute
        rejects = [readable for _d, readable, s in routes if s == "reject_bin"]
        self.assertTrue(rejects)
        self.assertFalse(any(rejects))                                                    # only no-reads rejected
        for sink in ("upstairs", "tote_store", "unload"):
            self.assertGreater(e.delivered.get(sink, 0), 0, sink)                         # elevator, totes, robot
        self.assertGreater(c.robots[0].cycles, 3)
        self.assertTrue(all(t.transferred >= 2 for t in c.transfers))
        sorter = next(b for b in c.belts if b.id == "sorter")
        self.assertAlmostEqual(sorter.frequency, 50.0)                                     # in Hz, for 1 m/s
        self.assertAlmostEqual(sorter.actualSpeed, 1.0, places=2)
        lamps = {lamp.id: lamp.isOn for lamp in c.leds}
        self.assertEqual((lamps["op_panel_run"], lamps["op_panel_fault"]), (True, False))


if __name__ == "__main__":
    unittest.main()
