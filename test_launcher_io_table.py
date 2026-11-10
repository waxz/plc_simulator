"""
test_launcher_io_table.py - the launcher (launcher.py: what each button
starts, the recent layouts, the window with stand-in processes) and the
designer's PLC tag table (io_table.py: every PLC signal from the exchange
contract as a tag - one per signal or bit array - with TIA addresses, your
names / tag tables / comments, CSV and TIA Portal export).
"""
import csv
import json
import os
import shutil
import sys
import tempfile
import unittest
import zipfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_SETTINGS_DIR = tempfile.mkdtemp()
os.environ["CONVEYOR_SIM_SETTINGS"] = os.path.join(_SETTINGS_DIR, "settings.json")      # never the user's

from PySide6.QtGui import QUndoStack  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import control_panel  # noqa: E402
import io_table  # noqa: E402
import launcher  # noqa: E402
from config_parser import ControlPanel, PlcTag, load_config  # noqa: E402
from mqtt_data_exchange import io_topics  # noqa: E402

_app = QApplication.instance() or QApplication([])
HERE = os.path.dirname(os.path.abspath(__file__))


def tearDownModule():
    shutil.rmtree(_SETTINGS_DIR, ignore_errors=True)


def script(launch):
    return os.path.basename(launch.args[1])


class LauncherCommandsTest(unittest.TestCase):
    def test_designer(self):
        self.assertEqual(launcher.designer_launch("x.json").args[1:],
                         [os.path.join(launcher.HERE, "layout_editor.py"), os.path.abspath("x.json")])
        self.assertEqual(launcher.designer_launch(None).args[1:], [os.path.join(launcher.HERE, "layout_editor.py")])
        self.assertEqual(launcher.designer_launch("x.json").args[0], sys.executable)

    def test_simulator_alone_with_the_layouts_exchange(self):
        (sim,) = launcher.simulation_launches(launcher.SimulationOptions("a.json"))
        self.assertEqual(script(sim), "main.py")
        self.assertEqual(sim.args[2:], [os.path.abspath("a.json")])

    def test_simulator_mock_plc_and_bridge_wired_together(self):
        options = launcher.SimulationOptions("a.json", exchange_config="ex.json", headless=True, view3d=True,
                                             mock_plc=True, bridge=True)
        bridge, sim, mock = launcher.simulation_launches(options)
        self.assertEqual([script(l) for l in (bridge, sim, mock)],
                         ["plc_foxglove_bridge.py", "main.py", "plc_mock.py"])       # listeners first
        exchange = ["--exchange-config", os.path.abspath("ex.json")]
        for l in (bridge, sim, mock):
            self.assertEqual(l.args[3:5], exchange)
        self.assertIn("--headless", sim.args)
        self.assertEqual(sim.args[sim.args.index("--view3d") + 1], "open")
        self.assertEqual(sim.env, {"SDL_VIDEODRIVER": "dummy"})
        self.assertEqual(bridge.args[bridge.args.index("--listen") + 1], launcher.BRIDGE_ADDRESS)
        self.assertIn("--autostart", mock.args)
        self.assertEqual(mock.args[mock.args.index("--bridge") + 1], launcher.BRIDGE_ADDRESS)

    def test_recent_layouts(self):
        first, second = os.path.join(HERE, "station_1.json"), os.path.join(HERE, "logistics_full.json")
        launcher.remember_layout(first)
        launcher.remember_layout(second)
        launcher.remember_layout(first)                                   # again: back to the top, once
        self.assertEqual(launcher.recent_layouts()[:2], [first, second])


class _FakeProcess:
    def __init__(self):
        self.code = None

    def poll(self):
        return self.code

    def terminate(self):
        self.code = 1


class LauncherWindowTest(unittest.TestCase):
    def test_run_watch_and_stop(self):
        started = []
        window = launcher._window()(starter=lambda launch: started.append(launch) or _FakeProcess())
        self.addCleanup(window.close)
        window.layout_combo.setEditText(os.path.join(HERE, "logistics_full.json"))
        window.mock_plc.setChecked(True)
        window.run_simulation()
        self.assertEqual([script(l) for l in started], ["main.py", "plc_mock.py"])
        self.assertEqual(window.running_list.count(), 2)
        window.open_designer()
        self.assertEqual(script(started[-1]), "layout_editor.py")
        window.running_list.item(0).setSelected(True)
        window.stop_selected()
        self.assertIn("ended", window.running_list.item(0).text())
        self.assertIn("running", window.running_list.item(1).text())
        window.stop_all()
        self.assertTrue(all("ended" in window.running_list.item(i).text() for i in range(3)))
        window.running.clear()                                           # (no question on close)


class PlcTagsTest(unittest.TestCase):
    """The designer's PLC tag table (io_table.py): one tag per signal, TIA
    addresses, names / tables / comments of your own kept in the layout."""

    def test_every_member_of_the_frames_once(self):
        config = load_config("logistics_full.json")
        tags, summary = io_table.tags_of(config)
        self.assertIn("Sim_Input 165 B (083EBF55) at %I0", summary)
        self.assertIn("Sim_Output 247 B (C5E971E0) at %Q0", summary)
        pub, sub = io_topics(config)
        self.assertLessEqual(set(pub["input"]["custom_ids"]) | set(sub["output"]["custom_ids"]),
                             {t.component for t in tags})
        div = {t.signal: t for t in tags if t.component == "div_1"}
        self.assertEqual((div["divert"].name, div["divert"].data_type, div["divert"].address),
                         ("div_1_divert", "Bool", "%Q1.1"))
        self.assertEqual((div["presentCode"].data_type, div["presentCode"].address), ("UDInt", "%ID122"))
        self.assertEqual((div["diverted"].data_type, div["diverted"].address), ("UInt", "%IW126"))
        self.assertEqual(div["commandSpeed"].address, "%QD222")
        self.assertEqual(div["divert"].comment, "Divert 1")                    # the component's label
        self.assertTrue(all(t.member.startswith(("Sim_Input.div_1.", "Sim_Output.div_1.")) for t in div.values()))
        self.assertEqual(len({t.name for t in tags}), len(tags))               # names are unique

    def test_a_banner_is_one_array_tag_a_button_a_bool(self):
        tags, _ = io_table.tags_of(load_config("station_1.json"))
        rows = {t.component: t.cells() for t in tags}
        self.assertEqual(rows["sensor_1_2_1"], ["sensor_1_2_1", "Default tag table", "Array[0..31] of Bool",
                                                "%I2.0..%I5.7", "L2 BANNER", "sensor_1_2_1", "sensor_states[0..31]",
                                                "Sim_Input.sensor_1_2_1"])
        self.assertEqual(rows["button_1_1_1"][:5], ["button_1_1_1", "Default tag table", "Bool", "%I0.0",
                                                    "L1 START"])
        self.assertEqual(rows["clock"][2:4], ["LInt", "%IB10..%IB17"])
        self.assertEqual(len(tags), 13 + 39)                    # the 13 input and 39 output members

    def test_a_control_panels_devices_are_listed(self):
        config = load_config("logistics_full.json")
        config.controlPanels.append(ControlPanel(id="op1", x=0, y=0, elements=control_panel.default_elements()))
        rows, _ = io_table.rows_of(config)
        self.assertEqual([r[io_table.COMPONENT] for r in rows if r[io_table.COMPONENT].startswith("op1")],
                         ["op1_start", "op1_stop", "op1_reset", "op1_run", "op1_fault", "op1_horn"])

    def test_your_names_tables_and_comments_are_kept_in_the_layout(self):
        config = load_config("station_1.json")
        config.plcTags = [PlcTag(key="button_1_1_1/sensor_states", name="L1_Start_PB", table="Line 1",
                                 comment="start push button")]
        tags, _ = io_table.tags_of(config)
        start = next(t for t in tags if t.component == "button_1_1_1")
        self.assertEqual((start.name, start.table, start.comment), ("L1_Start_PB", "Line 1", "start push button"))
        saved = json.loads(config.model_dump_json())
        self.assertEqual(saved["plcTags"][0]["name"], "L1_Start_PB")
        self.assertNotIn("plcTags", json.loads(load_config("station_1.json").model_dump_json()))
        self.assertIn("already", io_table.check_name("led_1_1_1", tags, start.key))
        self.assertIn("quote", io_table.check_name('a"b', tags, start.key))
        self.assertIn("name", io_table.check_name("  ", tags, start.key))
        self.assertEqual(io_table.check_name("L1_Start_PB", tags, start.key), "")

    def test_st_names_for_the_generated_ax_code(self):
        import plc_tags
        self.assertEqual(plc_tags.st_name("L1 Start PB"), "L1_Start_PB")
        self.assertEqual(plc_tags.st_name("1st lamp"), "_1st_lamp")
        self.assertEqual(plc_tags.st_name("Type"), "Type_")                     # a keyword, any case
        self.assertEqual(plc_tags.st_name("ok_name"), "ok_name")
        config = load_config("station_1.json")
        config.plcTags = [PlcTag(key="button_1_1_1/sensor_states", name="L1 Start PB"),
                          PlcTag(key="led_1_1_2/isOn", name="l1_start_pb")]       # the same in ST, ignoring case
        data = {t["key"]: t for t in plc_tags.tags_json(config)}
        self.assertEqual(data["button_1_1_1/sensor_states"]["stName"], "L1_Start_PB")
        self.assertEqual(data["led_1_1_2/isOn"]["stName"], "l1_start_pb_2")
        belt = data["belt_1_1_1/speed"]
        self.assertEqual((belt["member"], belt["frame"], belt["input"], belt["address"]),
                         ("belt_1_1_1.speed", "Sim_Output", False, "%QD2"))
        self.assertEqual(data["sensor_1_2_1/sensor_states"]["member"], "sensor_1_2_1")

    def test_st_export_writes_the_tags_for_the_ax_generator(self):
        import st_export
        out = os.path.join(_SETTINGS_DIR, "st")
        tags_file = os.path.join(out, "tags.json")
        st_export.main(["station_1.json", "--out", out, "--tags", tags_file])
        with open(tags_file, encoding="utf-8") as f:
            tags = json.load(f)
        self.assertEqual(len(tags), 13 + 39)
        self.assertEqual(tags[0]["stName"], "button_1_1_1")

    def test_tia_portal_export(self):
        tags, _ = io_table.tags_of(load_config("station_1.json"))
        rows, skipped = io_table.tia_rows(tags)
        self.assertEqual(rows[0][:5], ["Name", "Path", "Data Type", "Logical Address", "Comment"])
        self.assertEqual(skipped, 1)                                             # the clock: 64 bits
        self.assertEqual(len(rows) - 1, 13 + 39 - 2 - 1 + 2 * 32)               # banners as one Bool per beam
        self.assertIn(["sensor_1_2_1_31", "Default tag table", "Bool", "%I5.7", "L2 BANNER [31]", "True", "True",
                       "False", "", ""], rows)
        led = next(r for r in rows if r[0] == "led_1_1_1")
        self.assertEqual(led[2:4] + led[7:8], ["Bool", "%Q0.0", "True"])        # an output: writeable
        path = os.path.join(_SETTINGS_DIR, "tags.xlsx")
        io_table.write_xlsx(path, "PLC Tags", rows)
        with zipfile.ZipFile(path) as z:
            self.assertIn('name="PLC Tags"', z.read("xl/workbook.xml").decode())
            sheet = z.read("xl/worksheets/sheet1.xml").decode()
        self.assertIn('<t xml:space="preserve">%I5.7</t>', sheet)
        self.assertEqual(sheet.count("<row "), len(rows))


class PlcTagsPanelTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config("station_1.json")
        self.stack = QUndoStack()
        self.selected = []
        self.message = ""
        self.panel = io_table.IoTablePanel(lambda: self.config, on_select=self.selected.append,
                                           undo_stack=self.stack, status=lambda m: setattr(self, "message", m))
        self.panel.refresh()

    def row_of(self, component):
        return next(r for r in range(self.panel.table.rowCount())
                    if self.panel.table.item(r, io_table.COMPONENT).text() == component)

    def test_the_panel_filters_selects_and_exports(self):
        panel = self.panel
        self.assertEqual(panel.table.rowCount(), 52)
        self.assertEqual(panel.table.horizontalHeaderItem(0).text(), "Name")
        panel.filter.setText("belt_1_1_1")
        self.assertEqual([r[io_table.SIGNAL] for r in panel.visible_rows()], ["speed", "directionAngle"])
        panel._double_clicked(next(r for r in range(52) if not panel.table.isRowHidden(r)), io_table.ADDRESS)
        self.assertEqual(self.selected, ["belt_1_1_1"])
        path = os.path.join(_SETTINGS_DIR, "io.csv")
        panel.export_csv(path)
        with open(path, newline="", encoding="utf-8") as f:
            exported = list(csv.reader(f))
        self.assertEqual(exported[0], list(io_table.COLUMNS))
        self.assertEqual(len(exported), 3)                      # the filtered rows

    def test_addresses_sort_by_area_byte_and_bit(self):
        panel = self.panel
        panel.table.sortItems(io_table.ADDRESS)
        addresses = [panel.table.item(r, io_table.ADDRESS).text() for r in range(panel.table.rowCount())]
        self.assertEqual(addresses[:3], ["%I0.0", "%I0.1", "%I0.2"])
        self.assertLess(addresses.index("%I1.0"), addresses.index("%I2.0..%I5.7"))
        self.assertTrue(addresses[-1].startswith("%Q"))

    def test_rename_in_the_table_is_undoable_and_checked(self):
        panel = self.panel
        cell = panel.table.item(self.row_of("button_1_1_1"), io_table.NAME)
        cell.setText("L1_Start")                                # what typing in the cell does
        _app.processEvents()
        self.assertEqual([(t.key, t.name, t.table) for t in self.config.plcTags],
                         [("button_1_1_1/sensor_states", "L1_Start", "")])
        self.assertEqual(panel.table.item(self.row_of("button_1_1_1"), io_table.NAME).text(), "L1_Start")
        self.assertFalse(panel.edit_tag("led_1_1_1/isOn", name="L1_Start"))   # taken
        self.assertIn("already", self.message)
        self.stack.undo()
        _app.processEvents()
        self.assertEqual(self.config.plcTags, [])
        self.assertEqual(panel.table.item(self.row_of("button_1_1_1"), io_table.NAME).text(), "button_1_1_1")

    def test_tag_tables_on_the_left(self):
        panel = self.panel
        keys = ["button_1_1_1/sensor_states", "button_1_1_2/sensor_states"]
        panel.move_to_table(keys, "Line 1")
        self.assertEqual(self.stack.count(), 1)                 # one undo step
        _app.processEvents()
        self.assertEqual(panel.table_names(), ["Default tag table", "Line 1"])
        self.assertEqual(panel.tables.topLevelItem(2).text(0), "Line 1 [2]")
        panel.show_table("Line 1")
        self.assertEqual([r[io_table.NAME] for r in panel.visible_rows()], ["button_1_1_1", "button_1_1_2"])
        self.assertEqual([t.name for t in self.config.plcTags], ["", ""])     # names stay generated
        panel.edit_tag(keys[0], comment="Start, line 1")
        self.assertEqual(self.config.plcTags[-1].comment, "Start, line 1")
        _app.processEvents()
        panel.show_table("Show all tags")
        self.assertEqual(len(panel.visible_rows()), 52)
        panel.show_table("Line 1")
        self.assertEqual(panel.export_tia(os.path.join(_SETTINGS_DIR, "line1.xlsx")), 2)   # only the table shown
        self.assertTrue(panel.reset_name(keys[0]) is False)     # never renamed: nothing to reset


if __name__ == "__main__":
    unittest.main()
