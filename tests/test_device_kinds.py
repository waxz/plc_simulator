"""
test_device_kinds.py - the device kinds on the PLC exchange, declared once
(device_kinds.py), and everything derived from them: the frames, the
component lookup, the PLC programs (plc_logic.DEVICE_PROGRAMS), the typed
PLC STRUCTs (st_export.py); a new kind added in one place; and
logistics_full.json - every kind of machine, sensor and device - whose
frames carry every field across exactly, both ways, and which its build
script reproduces.
"""
import os
import random
import unittest

import config_parser as cp
import device_kinds
import io_layout
import plc_logic
import st_export
from config_parser import load_config
from device_kinds import DEVICE_KINDS, ByteType, DeviceKind, exchanged_devices, kind_of
from mqtt_data_exchange import io_topics
from mqtt_message_binding import (ComponentMessage, _read_bit, _write_bit, custom_component_mapping,
                                  find_component_by_id, known_component_ids, parse_path)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FULL = os.path.join(HERE, "logistics_full.json")


class RegistryTest(unittest.TestCase):
    def test_every_kind_is_a_config_list_with_its_model(self):
        for name, kind in DEVICE_KINDS.items():
            self.assertIn(name, cp.SimulatorConfig.model_fields)
            fields = kind.model.model_fields
            for side in (kind.commands, kind.feedback):
                for group in side:
                    for _bt, field in group:
                        self.assertIn(field, fields, f"{name}.{field}")
            if kind.commands != device_kinds.NONE:
                self.assertIn("control", fields, name)                # PLC or by hand
            self.assertTrue(kind.doc)

    def test_the_wire_order_is_the_registry_order(self):
        config = load_config(FULL)
        pub, sub = io_topics(config)
        ids = [m.id for m in exchanged_devices(config)]
        self.assertEqual(pub["input"]["custom_ids"][-len(ids) - 1:-1], ids)          # then the clock
        self.assertEqual(sub["output"]["custom_ids"][-len(ids):], ids)
        kinds = [kind_of(find_component_by_id(i, config)).list_name for i in ids]
        self.assertEqual(kinds, sorted(kinds, key=list(DEVICE_KINDS).index))
        self.assertNotIn("tote_store", known_component_ids(config))              # a plain sink: no signals
        self.assertIn("chute_1", known_component_ids(config))

    def test_a_new_kind_is_one_registration(self):
        """A kind registered once is on the exchange, in the lookup and in
        the PLC STRUCTs - no other module changes."""
        name = "doors"
        saved = DEVICE_KINDS.pop(name), device_kinds._BY_MODEL.pop(cp.GuardDoor)
        try:
            config = load_config(FULL)
            self.assertNotIn("door_1", io_topics(config)[1]["output"]["custom_ids"])
            device_kinds.register(DeviceKind(name, cp.GuardDoor, "Gate", commands=(device_kinds.bits("lock"), []),
                                             feedback=(device_kinds.bits("closed"), []), doc="test"))
            self.assertIn("door_1", io_topics(config)[1]["output"]["custom_ids"])
            self.assertEqual([n for n, _t in st_export.device_structs(config)["Gate_Feedback"]], ["closed"])
            with self.assertRaises(ValueError):
                device_kinds.register(DEVICE_KINDS[name])                        # once only
        finally:
            DEVICE_KINDS.pop(name, None)
            DEVICE_KINDS[name], device_kinds._BY_MODEL[cp.GuardDoor] = saved
            # back in its place in the wire order
            order = ["stoppers", "robots", "elevators", "scanners", "diverters", "turntables", "transfers",
                     "accumulators", "doors", "safety", "sinks"]
            for key in order:
                DEVICE_KINDS[key] = DEVICE_KINDS.pop(key)

    def test_every_kind_has_a_plc_program(self):
        self.assertEqual(set(plc_logic.DEVICE_PROGRAMS), set(DEVICE_KINDS))


class FullSystemTest(unittest.TestCase):
    """logistics_full.json: every kind of machine, sensor and device."""

    @classmethod
    def setUpClass(cls):
        cls.config = load_config(FULL)

    def test_has_every_kind(self):
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
        self.assertTrue(c.safety[0].curtainIds and c.safety[0].stopButtonIds and c.safety[0].doorIds)
        self.assertTrue(c.robots[0].pickSensorId)
        self.assertTrue(c.pallets and c.sources and c.leds)
        self.assertLessEqual({"START", "STOP"}, {b.label for b in c.buttons})
        self.assertTrue(any(s.capacity for s in c.sinks) and any(not s.capacity for s in c.sinks))
        self.assertEqual(c.mqttSettings.transport, "tcp")
        self.assertTrue(c.mqttSettings.enabled)

    def test_the_build_script_reproduces_it(self):
        import sys
        sys.path.insert(0, os.path.join(HERE, "tools"))
        import build_logistics_full
        self.assertEqual(build_logistics_full.build().model_dump(), self.config.model_dump())

    def _loopback(self, ids, direction):
        """Values set on one side, a frame across, the same values on the
        other side - field by field."""
        rng = random.Random(7)
        sender, receiver = load_config(FULL), load_config(FULL)
        mapping = custom_component_mapping(sender, ids, direction=direction)
        expected = {}
        for byte_type, path in mapping:
            component_id, field, index = parse_path(path)
            if component_id.startswith("padding") or component_id == "clock":
                continue
            component = find_component_by_id(component_id, sender)
            if byte_type == ByteType.BIT:
                value = rng.random() < 0.5
                setattr(component, field, _write_bit(getattr(component, field), index, value))
            elif byte_type == ByteType.FLOAT:
                value = round(rng.uniform(-50, 50), 2)
                setattr(component, field, value)
            else:
                bits = {ByteType.BYTE: 8, ByteType.UINT16: 16, ByteType.UINT32: 32}.get(byte_type, 8)
                value = rng.randrange(1 << bits)
                setattr(component, field, value)
            expected[path] = value
        payload = ComponentMessage(mapping).serialize(sender)
        self.assertEqual(len(payload), io_layout.layout_size(mapping))
        ComponentMessage(mapping).deserialize(payload, receiver, skip_manual=(direction == "in"))
        for path, value in expected.items():
            component_id, field, index = parse_path(path)
            got = getattr(find_component_by_id(component_id, receiver), field)
            if isinstance(value, bool):
                got = _read_bit(got, index)
            if isinstance(value, float):
                self.assertAlmostEqual(got, value, places=3, msg=path)
            else:
                self.assertEqual(got, value, path)
        return expected

    def test_every_field_crosses_the_input_frame(self):
        """Simulator -> PLC: every sensor, button and device feedback."""
        pub, _sub = io_topics(self.config)
        sent = self._loopback(pub["input"]["custom_ids"], "out")
        self.assertGreater(len(sent), 100)
        devices = {parse_path(p)[0] for p in sent}
        self.assertLessEqual({m.id for m in exchanged_devices(self.config)
                              if kind_of(m).feedback != device_kinds.NONE}, devices)   # a PLC value has none

    def test_every_field_crosses_the_output_frame(self):
        """PLC -> simulator: every belt, lamp and device command."""
        _pub, sub = io_topics(self.config)
        sent = self._loopback(sub["output"]["custom_ids"], "in")
        devices = {parse_path(p)[0] for p in sent}
        self.assertLessEqual({b.id for b in self.config.belts}, devices)
        self.assertLessEqual({m.id for m in exchanged_devices(self.config)
                              if kind_of(m).commands != device_kinds.NONE}, devices)

    def test_the_frame_map_checks_and_draws_the_frames(self):
        import sys
        import tempfile
        sys.path.insert(0, os.path.join(HERE, "tools"))
        import frame_map
        results = frame_map.check(load_config(FULL), "LE", lambda: load_config(FULL))
        self.assertTrue(all(ok for _n, ok, _d in results), results)
        out = os.path.join(tempfile.mkdtemp(), "map.html")
        self.assertEqual(frame_map.main([FULL, "--out", out]), 0)
        with open(out, encoding="utf-8") as f:
            page = f.read()
        self.assertIn("165 bytes, layout 083EBF55", page)
        self.assertIn("div_1/presentCode", page)

    def test_the_plc_structs_cover_every_kind(self):
        structs = st_export.device_structs(self.config)
        for kind in DEVICE_KINDS.values():
            if kind.list_name in ("plcValues", "watchdogs", "trackedLoads"):                  # optional (not in this layout): shown only
                continue
            self.assertIn(f"{kind.st_name}_Feedback", structs)
            if kind.commands != device_kinds.NONE:
                self.assertIn(f"{kind.st_name}_Command", structs)
        self.assertIn("BeltDrive_Command", structs)


if __name__ == "__main__":
    unittest.main()
