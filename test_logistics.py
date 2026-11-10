"""
test_logistics.py - parcel identity and the logistics equipment
(logistics.py): scanners (barcode / RFID / DWS), diverters (auto by
destination, PLC command), chutes with capacity, accumulation zones,
turntables, guard doors and safety circuits, their PLC frame fields, and
the logistics_demo.json sorting line end to end.
"""
import math
import random
import unittest

from config_parser import (Accumulator, Belt, ControlButton, Diverter, GuardDoor, SafetyCircuit, Scanner,
                           SimulatorConfig, Sink, Source, Turntable, load_config)
from mqtt_data_exchange import io_topics
from mqtt_message_binding import custom_component_mapping
from physics import PhysicsEngine


def belt(id, x, y, length, rotation=0.0, speed=1.0):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=1.0, rotation=rotation, speed=speed,
                directionAngle=0.0, color="#888888")


def source(**kw):
    args = dict(id="src", x=0.5, y=0.0, rotation=0.0, interval=1.0, minWidth=0.3, maxWidth=0.3, minHeight=0.3,
                maxHeight=0.3, label="S")
    args.update(kw)
    return Source(**args)


def plc_for(config):
    from plc_logic import make_controller
    return make_controller(config, autostart=True)


def run_under_plc(engine, plc, seconds, scan=0.02):
    """The simulation with the PLC program (plc_logic, what plc_mock.py runs)
    scanning every `scan` s: the equipment follows the PLC."""
    end = engine.sim_time + seconds
    while engine.sim_time < end - 1e-9:
        plc.step(now=engine.sim_time)
        engine.run_for(scan)


def line(**kw):
    return SimulatorConfig(belts=[belt("main", 5.0, 0.0, 10.0)], useSurfaceVelocity=True, **kw)


class IdentityTest(unittest.TestCase):
    def test_sources_number_and_route_their_parcels(self):
        random.seed(1)
        c = line(sources=[source(destinations=[1, 2], destinationWeights=[1.0, 0.0], noReadRate=0.5,
                                 barcodeStart=500, minMass=2.0, maxMass=4.0)])
        e = PhysicsEngine(c)
        e.run_for(5.5)
        parcels = sorted(e.parcels, key=lambda p: p.barcode)
        self.assertEqual([p.barcode for p in parcels], list(range(500, 500 + len(parcels))))
        self.assertTrue(all(p.destination == 1 for p in parcels))         # weight 0 for destination 2
        self.assertTrue(any(not p.readable for p in parcels) and any(p.readable for p in parcels))
        self.assertTrue(all(2.0 <= p.weight <= 4.0 for p in parcels))

    def test_a_plain_source_numbers_its_parcels_only(self):
        c = line(sources=[source()])
        e = PhysicsEngine(c)
        e.run_for(3.5)
        self.assertTrue(e.parcels)
        for p in e.parcels:
            self.assertEqual((p.destination, p.readable), (0, True))     # no destination draws
        self.assertEqual(sorted(p.barcode for p in e.parcels)[0], 1000001)


class ScannerTest(unittest.TestCase):
    def test_reads_once_per_parcel_no_reads_and_dws_measures(self):
        c = line(scanners=[Scanner(id="s", x=4.0, y=0.0, scannerType="dws"),
                           Scanner(id="rf", x=6.0, y=0.0, scannerType="rfid")])
        e = PhysicsEngine(c)
        p1 = e.add_parcel(2.0, 0.0, 0.5, 0.3)
        p1.barcode, p1.destination, p1.weight = 4711, 3, 6.5
        e.run_for(2.5)
        s, rf = c.scanners
        self.assertEqual((s.readCount, s.code, s.destination, s.noRead), (1, 4711, 3, False))
        self.assertEqual(s.weight, 6.5)
        self.assertEqual((s.measuredLength, s.measuredWidth), (0.5, 0.3))
        self.assertGreater(s.measuredHeight, 0.1)
        p2 = e.add_parcel(2.0, 0.0, 0.3, 0.3)
        p2.barcode, p2.readable = 4712, False
        e.run_for(4.5)
        self.assertEqual((s.readCount, s.code, s.noRead), (2, 0, True))     # unreadable label
        self.assertEqual((rf.readCount, rf.code, rf.noRead), (2, 4712, False))   # RFID reads it anyway


class DiverterTest(unittest.TestCase):
    def layout(self, **diverter):
        args = dict(id="d", x=5.0, y=0.0, divertAngle=-90.0, destination=2)
        args.update(diverter)
        c = line(diverters=[Diverter(**args)], sinks=[Sink(id="side", x=5.0, y=-3.5, width=1.5, height=1.0,
                                                          label="side")])
        c.belts.append(belt("branch", 5.0, -2.0, 3.0, rotation=-90.0))
        return c

    def test_plc_divert_command_and_present_code(self):
        c = self.layout(control="plc")
        e = PhysicsEngine(c)
        p = e.add_parcel(4.6, 0.0, 0.3, 0.3)
        p.barcode = 99
        e.run_for(0.05)
        d = c.diverters[0]
        self.assertTrue(d.present)
        self.assertEqual(d.presentCode, 99)
        self.assertFalse(d.active)
        d.divert = True
        e.run_for(5.0)
        self.assertEqual(e.delivered.get("side"), 1)


class WheelDiverterTest(unittest.TestCase):
    """Pop-up wheel diverters: the angle command is relative to the layout's
    divert angle, the wheels swivel to it, and the parcels go the way the
    wheels point - under the PLC program (plc_logic, what plc_mock.py runs)
    each destination to its own branch belt."""

    def layout(self, angle=45.0):
        c = SimulatorConfig(
            useSurfaceVelocity=True,
            belts=[belt("main", 8.0, 0.0, 16.0), belt("left", 6.0, 2.0, 3.0, rotation=90.0, speed=0.8),
                   belt("right", 11.0, -2.0, 3.0, rotation=-90.0, speed=0.8)],
            sources=[source(interval=1.6, maxWidth=0.4, maxHeight=0.4, destinations=[1, 2, 3])],
            scanners=[Scanner(id="scan", x=2.5, y=0.0)],
            diverters=[Diverter(id="d1", x=6.0, y=0.0, width=1.5, diverterType="wheels", divertAngle=angle,
                                speed=1.2, destination=1, control="plc"),
                       Diverter(id="d2", x=11.0, y=0.0, width=1.5, diverterType="wheels", divertAngle=-angle,
                                speed=1.2, destination=2, control="plc")],
            sinks=[Sink(id="left_end", x=6.0, y=4.1, width=1.4, height=1.0, label="L"),
                   Sink(id="right_end", x=11.0, y=-4.1, width=1.4, height=1.0, label="R"),
                   Sink(id="line_end", x=16.5, y=0.0, width=1.0, height=1.4, label="E")])
        return c

    def routes(self, engine, config):
        """(destination, sink) of every parcel a sink takes."""
        routes, take = [], engine.handle_sinks

        def tracked():
            before = [(p, p.destination, tuple(p.body.position)) for p in engine.parcels]
            take()
            alive = {id(p) for p in engine.parcels}
            for p, destination, (x, y) in before:
                if id(p) not in alive:
                    routes.append((destination, next((s.id for s in config.sinks if abs(x - s.x) <= s.width / 2
                                                      and abs(y - s.y) <= s.height / 2), "lost")))
        engine.handle_sinks = tracked
        return routes

    def test_the_plc_mock_sends_each_destination_to_its_branch(self):
        random.seed(5)
        c = self.layout()
        e = PhysicsEngine(c)
        routes = self.routes(e, c)
        plc = plc_for(c)
        steered = 0.0
        while e.sim_time < 60.0:
            plc.step(now=e.sim_time)
            e.run_for(0.02)
            steered = max(steered, c.diverters[0].wheelAngle)
        self.assertEqual(c.diverters[0].commandAngle, 0.0)                # the mock: as designed
        self.assertAlmostEqual(steered, 45.0)                               # the wheels turned to it
        expected = {1: "left_end", 2: "right_end", 3: "line_end"}
        self.assertGreater(len(routes), 25)
        self.assertEqual([r for r in routes if expected[r[0]] != r[1]], [])
        self.assertEqual({d for d, _s in routes}, {1, 2, 3})

    def test_the_wheels_swivel_to_divert_angle_plus_command_and_back(self):
        import logistics
        c = self.layout(angle=30.0)
        e = PhysicsEngine(c)
        d = c.diverters[0]
        d.control = "manual"
        d.divert, d.commandAngle = True, 20.0
        self.assertEqual(logistics.divert_angle(d), 50.0)
        e.run_for(0.05)
        self.assertGreater(d.wheelAngle, 0.0)
        self.assertLess(d.wheelAngle, 50.0)                                 # swivelling, not there yet
        e.run_for(0.3)
        self.assertAlmostEqual(d.wheelAngle, 50.0)
        d.commandAngle = 60.0                                               # 90: more than wheels steer
        e.run_for(0.3)
        self.assertAlmostEqual(d.wheelAngle, 60.0)
        self.assertEqual(logistics.diverter_motion(d)[1], 60.0)            # the parcels go where they point
        d.divert = False
        e.run_for(0.3)
        self.assertAlmostEqual(d.wheelAngle, 0.0)                           # down and straight again
        p = e.add_parcel(5.0, 0.0, 0.3, 0.3)
        d.divert, d.commandAngle = True, -60.0                              # 30 - 60: to the right
        e.run_for(0.8)
        self.assertLess(p.body.position.y, -0.1)


class ChuteTest(unittest.TestCase):
    def test_full_chute_backs_up_until_emptied(self):
        c = line(sinks=[Sink(id="chute", x=10.3, y=0.0, width=0.6, height=1.0, label="c", capacity=2)])
        e = PhysicsEngine(c)
        for x in (1.0, 2.0, 3.0):
            e.add_parcel(x, 0.0, 0.3, 0.3)
        e.run_for(12.0)
        chute = c.sinks[0]
        self.assertEqual((chute.count, chute.full), (2, True))
        self.assertEqual(len(e.parcels), 1)                              # the third waits in front
        chute.empty = True
        e.run_for(0.5)
        self.assertEqual(e.delivered["chute"], 3)
        self.assertEqual(chute.count, 1)

    def test_empty_time_and_only_chutes_on_the_exchange(self):
        c = line(sinks=[Sink(id="chute", x=10.3, y=0.0, width=0.6, height=1.0, label="c", capacity=1,
                             emptyTime=1.0),
                        Sink(id="plain", x=-3.0, y=0.0, width=1.0, height=1.0, label="p")])
        e = PhysicsEngine(c)
        e.add_parcel(9.0, 0.0, 0.3, 0.3)
        e.run_for(2.0)
        self.assertTrue(c.sinks[0].full)
        e.run_for(1.2)
        self.assertEqual((c.sinks[0].count, c.sinks[0].full), (0, False))
        pub, sub = io_topics(c)
        ids = pub["input"]["custom_ids"] + sub["output"]["custom_ids"]
        self.assertIn("chute", ids)
        self.assertNotIn("plain", ids)


class AccumulatorTest(unittest.TestCase):
    def layout(self, mode="singulation", control="plc"):
        c = SimulatorConfig(belts=[belt("z1", 1.0, 0.0, 2.0), belt("z2", 3.0, 0.0, 2.0), belt("z3", 5.0, 0.0, 2.0)],
                            accumulators=[Accumulator(id="a", x=3.0, y=-1.0, beltIds=["z1", "z2", "z3"], mode=mode,
                                                      control=control, release=False)],
                            useSurfaceVelocity=True)
        return c

    def test_zones_hold_one_parcel_each_until_released(self):
        c = self.layout(control="plc")
        e = PhysicsEngine(c)
        acc = c.accumulators[0]
        for _ in range(3):                                               # one parcel at a time, spaced
            e.add_parcel(0.3, 0.0, 0.3, 0.3)
            e.run_for(3.0)
        self.assertEqual(acc.zones, 0b111)
        self.assertTrue(acc.full)
        self.assertEqual([b.actualSpeed for b in c.belts], [0.0, 0.0, 0.0])   # all held
        xs = sorted(round(p.body.position.x, 1) for p in e.parcels)
        self.assertEqual(len(xs), 3)
        self.assertTrue(all(abs(a - b) > 1.5 for a, b in zip(xs, xs[1:])))   # one per zone, gaps kept
        acc.release = True
        e.run_for(10.0)
        self.assertEqual(len([p for p in e.parcels if p.body.position.x < 6.0]), 0)

    def test_runs_rule(self):
        from logistics import accumulator_runs
        self.assertEqual(accumulator_runs([True, True, True], "singulation", False), [False, False, False])
        self.assertEqual(accumulator_runs([True, True, True], "train", True), [True, True, True])
        self.assertEqual(accumulator_runs([True, False, True], "singulation", False), [True, True, False])


class TurntableTest(unittest.TestCase):
    def test_the_plc_program_turns_a_parcel_onto_the_cross_belt(self):
        c = SimulatorConfig(belts=[belt("in", 1.0, 0.0, 2.0), belt("tt", 2.75, 0.0, 1.5),
                                   belt("out", 2.75, 2.25, 3.0, rotation=90.0)],
                            turntables=[Turntable(id="t", x=2.75, y=0.0, radius=0.85, beltId="tt",
                                                  turnSpeed=180.0)],
                            sinks=[Sink(id="end", x=2.75, y=4.2, width=1.2, height=1.0, label="e")],
                            useSurfaceVelocity=True)
        e, plc = PhysicsEngine(c), plc_for(c)
        p = e.add_parcel(0.5, 0.0, 0.3, 0.3)
        run_under_plc(e, plc, 3.0)
        t = c.turntables[0]
        self.assertEqual(t.index, 1)
        self.assertAlmostEqual(c.belts[1].rotation, 90.0)
        run_under_plc(e, plc, 8.0)
        self.assertEqual(e.delivered.get("end"), 1)
        self.assertNotIn(p, e.parcels)
        self.assertEqual(t.index, 0)                                     # back for the next one
        self.assertAlmostEqual(c.belts[1].rotation, 0.0)


class TransferTest(unittest.TestCase):
    """A pallet corner of roller conveyors with a 90-degree transfer."""

    def layout(self, control="plc"):
        from config_parser import Pallet, Transfer
        c = SimulatorConfig(belts=[belt("in", 2.5, 0.0, 4.0), belt("out", 4.0, 2.5, 4.0, rotation=90.0)],
                            transfers=[Transfer(id="t", x=4.0, y=0.0, control=control)],
                            pallets=[Pallet(id="p", x=1.0, y=0.0)], useSurfaceVelocity=True)
        for b in c.belts:
            b.surface = "roller"
        c.belts[0].length, c.belts[0].x = 5.0, 2.0          # the infeed runs into the corner square
        return c

    def test_the_plc_program_stops_the_pallet_centred_and_carries_it_across_unturned(self):
        c = self.layout()
        e, plc = PhysicsEngine(c), plc_for(c)
        pallet = e.machines.pallets[0]
        t = c.transfers[0]
        lifted_at = None
        scan = max(1, round(0.02 / c.physicsDt))
        for i in range(int(12.0 / c.physicsDt)):
            if i % scan == 0:
                plc.step(now=e.sim_time)
            e.step(c.physicsDt)
            if t.lift > 0 and lifted_at is None:
                lifted_at = tuple(pallet.body.position)
        self.assertAlmostEqual(lifted_at[0], 4.0, delta=0.05)          # stopped on the centre
        self.assertGreater(pallet.body.position.y, 1.5)                 # on the cross conveyor
        self.assertAlmostEqual(pallet.body.position.x, 4.0, delta=0.05)
        self.assertAlmostEqual(pallet.body.angle, 0.0, places=6)        # not turned
        self.assertEqual(t.transferred, 1)
        self.assertFalse(t.up)

    def test_plc_holds_it_until_told_to_transfer(self):
        c = self.layout(control="plc")
        e = PhysicsEngine(c)
        t = c.transfers[0]
        e.run_for(8.0)
        self.assertTrue(t.centered)
        self.assertFalse(t.up)
        pallet = e.machines.pallets[0]
        self.assertAlmostEqual(pallet.body.position.x, 4.0, delta=0.05)
        t.transfer = True
        e.run_for(1.0)
        self.assertTrue(t.up)
        e.run_for(5.0)
        self.assertGreater(pallet.body.position.y, 1.5)
        pub, sub = io_topics(c)
        outputs = [p for _bt, p in custom_component_mapping(c, pub["input"]["custom_ids"], direction="out")]
        self.assertIn("t/centered", outputs)

    def test_the_demo_pallet_loop_runs_on_rollers_and_transfers(self):
        """Under the PLC program (plc_logic, what plc_mock.py runs): its
        machines follow the PLC, so nothing moves without one."""
        from plc_logic import make_controller
        random.seed(3)
        c = load_config("machines_demo.json")
        self.assertFalse([b for b in c.belts if b.type != "linear"])     # no curves
        self.assertEqual({b.surface for b in c.belts if b.id.startswith("loop_")}, {"roller"})
        e = PhysicsEngine(c)
        plc = make_controller(c, autostart=True)
        while e.sim_time < 60.0:
            plc.step(now=e.sim_time)
            e.run_for(0.05)
        self.assertTrue(all(t.transferred >= 2 for t in c.transfers))
        # Carried across, not turned (a turn would be 90 degrees; the PLC's scan
        # leaves a fraction of a degree of play).
        for p in e.machines.pallets:
            self.assertAlmostEqual(math.sin(p.body.angle - math.radians(p.model.rotation)), 0.0, delta=0.01)
        self.assertGreater(c.robots[0].cycles, 1)


class SafetyTest(unittest.TestCase):
    def layout(self):
        return line(buttons=[ControlButton(id="es", x=0, y=2, label="E-STOP", buttonType="latching",
                                           normallyClosed=True),
                             ControlButton(id="rst", x=1, y=2, label="RESET")],
                    doors=[GuardDoor(id="door", x=3, y=2)],
                    safety=[SafetyCircuit(id="safe", x=0, y=3, stopButtonIds=["es"], doorIds=["door"],
                                          resetButtonId="rst")])

    def test_estop_and_door_stop_the_belts_until_reset(self):
        import devices
        c = self.layout()
        e = PhysicsEngine(c)
        e.run_for(0.5)
        main, circuit = c.belts[0], c.safety[0]
        self.assertTrue(circuit.ok)
        self.assertAlmostEqual(main.actualSpeed, 1.0)
        devices.press(c.buttons[0])                                      # E-stop: latched
        e.run_for(0.2)
        self.assertFalse(circuit.ok)
        self.assertEqual(main.actualSpeed, 0.0)
        devices.press(c.buttons[0])                                      # pulled out again
        e.run_for(0.2)
        self.assertTrue(circuit.tripped and not circuit.ok)              # still needs a reset
        devices.press(c.buttons[1])
        e.run_for(0.1)
        devices.release(c.buttons[1])
        e.run_for(0.2)
        self.assertTrue(circuit.ok)
        self.assertAlmostEqual(main.actualSpeed, 1.0)
        self.assertFalse(e.logistics.toggle_door("door"))               # opened
        e.run_for(0.1)
        self.assertFalse(circuit.ok)
        e.logistics.toggle_door("door")
        circuit.reset = True                                             # the PLC resets it
        e.run_for(0.1)
        self.assertTrue(circuit.ok)

    def test_a_locked_door_stays_shut(self):
        c = self.layout()
        e = PhysicsEngine(c)
        c.doors[0].lock = True
        e.run_for(0.1)
        self.assertTrue(c.doors[0].locked)
        self.assertTrue(e.logistics.toggle_door("door"))                # still closed
        self.assertTrue(c.safety[0].ok)


class ExchangeTest(unittest.TestCase):
    def test_frames_carry_every_device(self):
        c = load_config("logistics_demo.json")
        pub, sub = io_topics(c)
        inputs = [p for _bt, p in custom_component_mapping(c, pub["input"]["custom_ids"], direction="out")]
        outputs = [p for _bt, p in custom_component_mapping(c, sub["output"]["custom_ids"], direction="in")]
        for path in ("dws_1/code", "dws_1/weight", "div_1/presentCode", "tt_1/index", "acc/zones", "door_1/closed",
                     "safety_1/ok", "chute_1/count"):
            self.assertIn(path, inputs)
        for path in ("div_1/divert", "tt_1/targetIndex", "acc/release", "door_1/lock", "safety_1/reset",
                     "chute_1/empty"):
            self.assertIn(path, outputs)


class DemoTest(unittest.TestCase):
    def test_the_plc_program_sorts_by_tracking_scanner_reads(self):
        """Diverters, turntable and accumulation on control "plc": plc_logic
        tracks each scanner read (code -> destination) and fires the
        diverter that sees a code for its destination."""
        from plc_logic import make_controller
        random.seed(3)
        c = load_config("logistics_demo.json")
        for device in (*c.diverters, *c.turntables, *c.accumulators):
            device.control = "plc"
        e = PhysicsEngine(c)
        plc = make_controller(c, autostart=True)
        events = []
        while e.sim_time < 120.0:
            events += plc.step(now=e.sim_time)
            e.run_for(0.05)
        for chute in ("chute_1", "chute_2", "chute_3"):
            self.assertGreater(e.delivered.get(chute, 0), 5)
        self.assertGreater(e.delivered.get("reject_bin", 0), 0)
        self.assertTrue(any("-> destination 2" in ev for ev in events))
        self.assertTrue(c.doors[0].locked)                              # locked while the line runs


if __name__ == "__main__":
    unittest.main()
