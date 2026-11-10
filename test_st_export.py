"""
test_st_export.py - the SIMATIC AX package generated from the simulator's
exchange definition (st_export.py): its frames match the wire exactly (the
golden-vector packer reproduces ComponentMessage's bytes for live values,
for every frame of several layouts), its offsets / sizes / fingerprints
are the ones io_layout and LAYOUT_CHECK use, the ST it writes is complete
and deterministic, and --check catches a stale package.
"""
import json
import os
import random
import re
import tempfile
import unittest

import io_layout
import st_export
from config_parser import load_config
from mqtt_message_binding import ByteType, ComponentMessage
from physics import PhysicsEngine

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUTS = ("station_1.json", "machines_demo.json", "logistics_demo.json")


def running(name: str, seconds: float = 20.0):
    random.seed(5)
    config = load_config(os.path.join(HERE, name))
    PhysicsEngine(config).run_for(seconds)
    return config


class WireTest(unittest.TestCase):
    def test_the_packer_reproduces_the_simulators_bytes_for_live_values(self):
        for name in LAYOUTS:
            config = running(name)
            for frame, _frame_type, mapping in st_export.frames(config):
                if frame.startswith("Metric"):
                    continue                                            # no live source in the config
                with self.subTest(layout=name, frame=frame):
                    endian = config.mqttSettings.endianness
                    wire = ComponentMessage(mapping, endianness="<" if endian == "LE" else ">").serialize(config)
                    _members, access = st_export.members(mapping)
                    values = st_export.unpack(wire, access, endian)
                    self.assertEqual(st_export.pack(len(wire), access, values, endian), wire)
                    self.assertEqual(len(wire), io_layout.layout_size(mapping))

    def test_big_endian_layouts_too(self):
        config = running("logistics_demo.json", 5.0)
        config.mqttSettings.endianness = "BE"
        for frame, _t, mapping in st_export.frames(config)[:2]:
            wire = ComponentMessage(mapping, endianness=">").serialize(config)
            _m, access = st_export.members(mapping)
            self.assertEqual(st_export.pack(len(wire), access, st_export.unpack(wire, access, "BE"), "BE"), wire)
        self.assertIn("Endianness#Big", st_export.generate(config, "x", "Ns")["SimIoSerde.st"])


class ContractTest(unittest.TestCase):
    def test_sizes_and_fingerprints_are_the_layout_checks(self):
        config = load_config(os.path.join(HERE, "station_1.json"))
        c = st_export.contract(config, "station_1.json")
        frames = {f["name"]: f for f in c["frames"]}
        self.assertEqual(frames["Sim_Input"]["size"], 18)                   # the AX project's Station_1 frames
        self.assertEqual(frames["Sim_Output"]["size"], 122)
        self.assertEqual(frames["Metric_Layout_1"]["size"], 26)
        mapping = st_export.frames(config)[0][2]
        self.assertEqual(frames["Sim_Input"]["fingerprint"], io_layout.layout_fingerprint(mapping))
        self.assertEqual(c["frameTypes"]["LAYOUT_CHECK"], 7)
        json.dumps(c)

    def test_every_signal_has_a_member_and_offsets_are_the_packers(self):
        config = load_config(os.path.join(HERE, "logistics_demo.json"))
        for frame, _t, mapping in st_export.frames(config):
            members, access = st_export.members(mapping)
            real = [f for f in io_layout.layout_fields(mapping) if not f.is_padding]
            self.assertEqual([f for f, _ in access], real)
            names = {m.name for m in members}
            for _f, expr in access:
                self.assertIn(expr.split("[")[0], names)
            self.assertEqual(len(names), len(members))                        # no duplicate members


class EmitTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(os.path.join(HERE, "logistics_demo.json"))
        self.files = st_export.generate(self.config, "logistics_demo.json", "Simulator.Io")

    def test_complete_and_deterministic_st(self):
        self.assertEqual(st_export.generate(self.config, "logistics_demo.json", "Simulator.Io"), self.files)
        types, serde = self.files["SimIoTypes.st"], self.files["SimIoSerde.st"]
        for frame, _t, mapping in st_export.frames(self.config):
            self.assertIn(f"{frame} : STRUCT", types)
            self.assertIn(f"value : {frame};", serde)
            self.assertIn(f"startOffset + UDINT#{io_layout.layout_size(mapping)};", serde)
        self.assertEqual(serde.count("FUNCTION DeserializeFromBytes"), serde.count("FUNCTION SerializeToBytes"))
        self.assertEqual(serde.count("FUNCTION "), serde.count("END_FUNCTION"))
        # A device is one member of its kind's STRUCT (device_kinds.py), and
        # the serializers reach its fields through it.
        self.assertIn("dws_1 : Scanner_Feedback;", types)
        self.assertIn("Scanner_Feedback : STRUCT", types)
        self.assertIn("            code : UDINT;", types)
        self.assertIn("div_1 : Diverter_Command;", types)
        self.assertIn("infeed : BeltDrive_Command;", types)
        self.assertIn("value.dws_1.code", serde)
        structs = st_export.device_structs(self.config)
        self.assertEqual([n for n, _t in structs["Diverter_Command"]], ["divert", "commandSpeed", "commandAngle"])
        for text in self.files.values():
            self.assertIn("Do not edit by hand", text)
            self.assertNotRegex(text, r"\bNone\b")

    def test_golden_vectors_hold_the_python_bytes(self):
        vectors = self.files["SimIoVectorsTest.st"]
        frame, _t, mapping = st_export.frames(self.config)[0]
        _m, access = st_export.members(mapping, self.config, st_export.FRAME_DIRECTIONS.get(frame))
        size = io_layout.layout_size(mapping)
        expected = st_export.pack(size, access, st_export.sample_values(access), "LE")
        literal = ", ".join(f"BYTE#16#{b:02X}" for b in expected)
        self.assertIn(f"expected : ARRAY[0..{size - 1}] OF BYTE := [{literal}];", vectors)
        self.assertEqual(len(re.findall(r"\{Test\}", vectors)), len(st_export.frames(self.config)))

    def test_check_mode_reports_stale_files(self):
        out = tempfile.mkdtemp()
        layout = os.path.join(HERE, "logistics_demo.json")
        self.assertEqual(st_export.main([layout, "--out", out, "--contract", os.path.join(out, "c.json")]), 0)
        self.assertEqual(st_export.main([layout, "--out", out, "--contract", os.path.join(out, "c.json"),
                                         "--check"]), 0)
        with open(os.path.join(out, "SimIoTypes.st"), "a", encoding="utf-8") as f:
            f.write("// edited\n")
        self.assertEqual(st_export.main([layout, "--out", out, "--check"]), 1)


if __name__ == "__main__":
    unittest.main()
