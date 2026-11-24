"""
test_st_export.py - the exchange contract (st_export.py) and the SIMATIC AX
ST written from it by the one ST generator (ax_export.py): the frames match
the wire exactly (the golden-vector packers reproduce ComponentMessage's
bytes for live values, for every frame of several layouts), the offsets /
sizes / fingerprints are the ones io_layout and LAYOUT_CHECK use, the ST is
complete and deterministic, the export settings are kept in the layout, and
--check catches a stale file.
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

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def export_text(config, namespace="Simulator.Io", **settings) -> str:
    """The layout's ST from ax_export, as one export would write it."""
    import ax_export
    import plc_tags
    from config_parser import AxExport
    result = ax_export.export(st_export.contract(config, "x"), AxExport(output="out.st", namespace=namespace,
                                                                        **settings), plc_tags.tags_json(config))
    return result.files["out.st"]
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
        self.assertIn("Endianness#Big", export_text(config, "Ns"))


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
    """The ST ax_export writes - the designer, st_export.py and the AX
    project's generate_st_serde.py all write it through ax_export."""

    def setUp(self):
        self.config = load_config(os.path.join(HERE, "logistics_demo.json"))
        self.text = export_text(self.config)

    def test_complete_and_deterministic_st(self):
        self.assertEqual(export_text(self.config), self.text)
        for frame in st_export.contract(self.config)["frames"]:
            self.assertIn(f"{frame['name']} : STRUCT", self.text)
            self.assertIn(f"value : {frame['name']};", self.text)
            self.assertIn(f"startOffset + UDINT#{frame['size']};", self.text)
            self.assertIn(f"TO_DINT(startOffset) + DINT#{frame['size']} THEN", self.text)    # the length guard
        self.assertEqual(self.text.count("FUNCTION DeserializeFromBytes"), self.text.count("FUNCTION SerializeToBytes"))
        self.assertEqual(self.text.count("FUNCTION "), self.text.count("END_FUNCTION"))
        # A device is one member of its kind's STRUCT (device_kinds.py), and
        # the serializers reach its fields through it.
        self.assertIn("dws_1 : Scanner_Feedback;", self.text)
        self.assertIn("Scanner_Feedback : STRUCT", self.text)
        self.assertIn("div_1 : Diverter_Command;", self.text)
        self.assertIn("infeed : BeltDrive_Command;", self.text)
        self.assertIn("value.dws_1.code", self.text)
        self.assertIn("Sim_Tags : STRUCT", self.text)
        self.assertIn("Do not edit by hand", self.text)
        self.assertNotRegex(self.text, r"\bNone\b")
        structs = st_export.device_structs(self.config)
        self.assertEqual([n for n, _t in structs["Diverter_Command"]], ["enable", "commandSpeed", "commandAngle"])

    def test_golden_vectors_hold_the_simulators_bytes(self):
        """The vector test's expected bytes (ax_export.pack_frame, from the
        contract) are the simulator's own bytes for the same values."""
        import ax_export
        config = running("logistics_demo.json", 10.0)
        contract = st_export.contract(config)
        for frame, _t, mapping in st_export.frames(config):
            if frame.startswith("Metric"):
                continue
            with self.subTest(frame=frame):
                wire = ComponentMessage(mapping, endianness="<").serialize(config)
                _m, access = st_export.members(mapping, config, st_export.FRAME_DIRECTIONS.get(frame))
                values = {expr: value for (f, expr), value in zip(access, st_export.unpack(wire, access, "LE").values())}
                described = next(f for f in contract["frames"] if f["name"] == frame)
                self.assertEqual(ax_export.pack_frame(described, values, "LE"), wire)
        vectors = export_text(self.config, vectors="vectors.st")
        self.assertIn("Sim_Tags", vectors)                                 # (the ST) and the test file:
        import ax_export as ax
        from config_parser import AxExport
        files = ax.export(st_export.contract(self.config), AxExport(output="o.st", vectors="t.st")).files
        self.assertIn("METHOD PUBLIC Sim_Input_matches_the_simulator", files["t.st"])
        self.assertEqual(files["t.st"].count("{Test}"), len(st_export.contract(self.config)["frames"]))

    def test_settings_are_kept_in_the_layout_and_check_catches_a_stale_file(self):
        import json
        import shutil
        folder = tempfile.mkdtemp()
        layout = os.path.join(folder, "plant.json")
        shutil.copy(os.path.join(HERE, "logistics_demo.json"), layout)
        out = os.path.join(folder, "ax", "sim_io.st")
        self.assertEqual(st_export.main([layout, "-o", out, "--namespace", "Plant.Io", "--frames",
                                         "Sim_Input,Sim_Output", "--save"]), 0)
        with open(layout, encoding="utf-8") as f:
            saved = json.load(f)["axExport"]
        self.assertEqual((saved["output"], saved["namespace"], saved["frames"]),
                         ("ax/sim_io.st", "Plant.Io", ["Sim_Input", "Sim_Output"]))      # relative to the layout
        self.assertEqual(load_config(layout).axExport.namespace, "Plant.Io")
        self.assertEqual(st_export.main([layout, "--check"]), 0)                         # as saved: up to date
        with open(out, "a", encoding="utf-8") as f:
            f.write("// edited\n")
        self.assertEqual(st_export.main([layout, "--check"]), 1)
        self.assertEqual(st_export.main([layout]), 0)                                    # regenerated as saved
        self.assertEqual(st_export.main([layout, "--check"]), 0)
        contract_file = os.path.join(folder, "c.json")
        st_export.main([layout, "--contract", contract_file])
        again = os.path.join(folder, "again.st")
        self.assertEqual(st_export.main(["--from-contract", contract_file, "-o", again, "--namespace", "Plant.Io",
                                         "--frames", "Sim_Input,Sim_Output"]), 0)
        with open(out, encoding="utf-8") as a, open(again, encoding="utf-8") as b:
            self.assertEqual(a.read(), b.read())                                         # the same ST from the file

    def test_laser_banners_of_any_size_are_byte_arrays(self):
        """A banner's beams are one ARRAY OF BYTE, beam i = bit (i MOD 8) of
        byte (i DIV 8), whatever its beam count - the simulator's own bytes."""
        import ax_export
        from config_parser import Sensor, SimulatorConfig

        def banner(sensor_id, beams):
            return Sensor(id=sensor_id, x=0, y=0, width=0.1, height=1, rotation=0, label="", color="#f00",
                          sensorType="laser_banner", laserCount=beams, beamDistance=0.02)
        config = SimulatorConfig(sensors=[banner("b48", 48), banner("b20", 20),
                                          Sensor(id="eye", x=0, y=0, width=0.1, height=1, rotation=0, label="",
                                                 color="#f00")])
        text = export_text(config)
        self.assertIn("b48 : ARRAY[0..5] OF BYTE;", text)
        self.assertIn("b20 : ARRAY[0..2] OF BYTE;", text)        # 20 beams: 3 bytes, the last one padded
        self.assertIn("eye : BOOL;", text)
        frame = next(f for f in st_export.contract(config)["frames"] if f["name"] == "Sim_Input")
        packed = ax_export.pack_bit_arrays(frame)
        b48 = [f for f in packed["fields"] if f["member"].startswith("b48[")]
        self.assertEqual([f["type"] for f in b48], ["byte"] * 6)
        # The bytes the simulator sends for some blocked beams hold them as the PLC reads them.
        config.sensors[0].sensor_states = "".join("1" if i in (0, 9, 47) else "0" for i in range(48))
        config.sensors[1].sensor_states = "".join("1" if i == 19 else "0" for i in range(20))
        _name, _t, mapping = next(f for f in st_export.frames(config) if f[0] == "Sim_Input")
        wire = ComponentMessage(mapping, endianness="<").serialize(config)
        first = b48[0]["byte"]
        self.assertEqual(list(wire[first:first + 6]), [0x01, 0x02, 0, 0, 0, 0x80])        # beams 0, 9, 47
        b20 = next(f for f in packed["fields"] if f["member"] == "b20[2]")
        self.assertEqual(wire[b20["byte"]], 0x08)                                          # beam 19: byte 2, bit 3

    def test_a_frame_too_big_for_the_plc_is_refused(self):
        import ax_export
        from config_parser import AxExport
        contract = st_export.contract(load_config(os.path.join(HERE, "logistics_full.json")))
        with self.assertRaisesRegex(ax_export.ExportError, "Sim_Output \\(247 bytes\\)"):
            ax_export.export(contract, AxExport(output="x.st", maxPayload=242))     # an older 256-byte client
        self.assertEqual(ax_export.PLC_MAX_PAYLOAD, 4096)                          # the TcpClient's 4110 - 14
        self.assertIn("x.st", ax_export.export(contract, AxExport(output="x.st")).files)
        self.assertIn("x.st", ax_export.export(contract, AxExport(output="x.st", maxPayload=0)).files)


class TagArraysTest(unittest.TestCase):
    """axExport.tagArrays: a kind's tags as arrays of its wire STRUCTs, the
    device's index its place in its kind, named in <prefix>_TagIndex; the
    wire stays the same."""

    def export(self, arrays):
        import ax_export
        import control_panel
        import plc_tags
        config = load_config("merge_demo.json")
        control_panel.refresh(config)
        config.axExport.tagArrays = arrays
        contract = st_export.contract(config, "merge_demo.json")
        tags = plc_tags.tags_json(config)
        text = next(iter(ax_export.export(contract, config.axExport, tags,
                                          os.path.abspath("merge_demo.json")).files.values()))
        return config, contract, tags, text

    def test_arrays_index_and_copies(self):
        config, contract, tags, text = self.export(["belts", "trackedLoads"])
        ids = [b.id for b in config.belts]
        path = {t["key"]: t["stPath"] for t in tags}
        self.assertEqual(path["belt_5_a/speed"], f"beltVfdCommands[{ids.index('belt_5_a')}].speed")
        self.assertEqual(path["belt_2/frequency"], f"beltFreqCommands[{ids.index('belt_2')}].frequency")
        self.assertEqual(path["track_03/matched"], "trackedLoadFeedbacks[2].matched")
        self.assertEqual(path["panel_start/sensor_states"], "panel_start")          # not a listed kind
        self.assertIn(f"beltVfdCommands : ARRAY[0..{len(ids) - 1}] OF BeltVfd_Command;", text)
        self.assertIn("trackedLoadCommands : ARRAY[0..15] OF TrackedLoad_Command;", text)
        self.assertIn("CLASS PUBLIC Plant_TagIndex", text)
        self.assertIn(f"belt_7 : INT := INT#{ids.index('belt_7')};", text)
        self.assertIn(f"dest.belt_5_a := tags.beltVfdCommands[{ids.index('belt_5_a')}];", text)
        self.assertIn("tags.trackedLoadFeedbacks[0] := source.track_01;", text)
        self.assertNotIn("belt_5_a_speed :", text)                                   # no flat tag for it
        _c, plain_contract, _t, plain = self.export([])
        self.assertEqual([f["fingerprint"] for f in contract["frames"]],
                         [f["fingerprint"] for f in plain_contract["frames"]], "the wire is the same")
        self.assertIn("belt_5_a_speed :", plain)
        self.assertNotIn("_TagIndex", plain)


class MemberNameTest(unittest.TestCase):
    """Component ids become ST member names the AX compiler accepts: ST
    identifiers, never a keyword, unique ignoring case - and two ids that
    clean to the same name are two members, never merged into one."""

    def test_names_are_valid_unique_st_identifiers(self):
        from config_parser import Belt, SimulatorConfig

        def belt(belt_id, x):
            return Belt(id=belt_id, type="linear", x=x, y=0, length=2, beltWidth=1, rotation=0, speed=1,
                        directionAngle=0, color="#888")
        config = SimulatorConfig(belts=[belt("belt-1", 0), belt("belt_1", 2), belt("2nd_belt", 4), belt("Belt_A", 6),
                                        belt("belt_a", 8), belt("type", 10)])
        frame = next(f for f in st_export.contract(config)["frames"] if f["name"] == "Sim_Output")
        names = [m["name"] for m in frame["members"]]
        self.assertEqual(names, ["belt_1", "belt_1_2", "_2nd_belt", "Belt_A", "belt_a_2", "type_"])
        self.assertEqual(len({n.lower() for n in names}), len(names))
        self.assertTrue(all(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", n) for n in names))
        members = {f["path"]: f["member"] for f in frame["fields"]}
        self.assertEqual(members["belt-1/speed"], "belt_1.speed")
        self.assertEqual(members["belt_1/speed"], "belt_1_2.speed")              # its own member

    def test_station_1_keeps_its_names(self):
        frame = next(f for f in st_export.contract(load_config("station_1.json"))["frames"]
                     if f["name"] == "Sim_Input")
        self.assertEqual([m["name"] for m in frame["members"]][:3], ["button_1_1_1", "button_1_1_2", "button_1_1_3"])


if __name__ == "__main__":
    unittest.main()
