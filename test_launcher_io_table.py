"""
test_launcher_io_table.py - the launcher (launcher.py: what each button
starts, the recent layouts, the window with stand-in processes) and the
designer's I/O table (io_table.py: every PLC signal from the exchange
contract, one row per signal or bit array).
"""
import csv
import os
import shutil
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_SETTINGS_DIR = tempfile.mkdtemp()
os.environ["CONVEYOR_SIM_SETTINGS"] = os.path.join(_SETTINGS_DIR, "settings.json")      # never the user's

from PySide6.QtWidgets import QApplication  # noqa: E402

import control_panel  # noqa: E402
import io_table  # noqa: E402
import launcher  # noqa: E402
from config_parser import ControlPanel, load_config  # noqa: E402
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
        first, second = os.path.join(HERE, "test_config.json"), os.path.join(HERE, "logistics_full.json")
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


class IoTableTest(unittest.TestCase):
    def test_every_member_of_the_frames_once(self):
        config = load_config("logistics_full.json")
        rows, summary = io_table.rows_of(config)
        self.assertIn("Sim_Input 165 B (083EBF55)", summary)
        self.assertIn("Sim_Output 247 B (C5E971E0)", summary)
        components = {r[0] for r in rows}
        pub, sub = io_topics(config)
        self.assertLessEqual(set(pub["input"]["custom_ids"]) | set(sub["output"]["custom_ids"]), components)
        div = [r for r in rows if r[0] == "div_1"]
        self.assertIn(["div_1", "divert", "PLC output", "BOOL"], [r[:4] for r in div])
        self.assertTrue(all(r[7].startswith(("Sim_Input.div_1.", "Sim_Output.div_1.")) for r in div))

    def test_a_banner_is_one_array_row_a_button_a_bool(self):
        rows, _ = io_table.rows_of(load_config("test_config.json"))
        banner = [r for r in rows if r[0] == "sensor_1_2_1"]
        self.assertEqual(banner, [["sensor_1_2_1", "sensor_states[0..31]", "PLC input", "ARRAY[0..31] OF BOOL",
                                   "Sim_Input", "2..5", "", "Sim_Input.sensor_1_2_1"]])
        button = next(r for r in rows if r[0] == "button_1_1_1")
        self.assertEqual(button, ["button_1_1_1", "sensor_states", "PLC input", "BOOL", "Sim_Input", "0", "0",
                                  "Sim_Input.button_1_1_1"])
        self.assertEqual(len(rows), 13 + 39)                    # the 13 input and 39 output members

    def test_a_control_panels_devices_are_listed(self):
        config = load_config("logistics_full.json")
        config.controlPanels.append(ControlPanel(id="op1", x=0, y=0, elements=control_panel.default_elements()))
        rows, _ = io_table.rows_of(config)
        self.assertEqual([r[0] for r in rows if r[0].startswith("op1")],
                         ["op1_start", "op1_stop", "op1_reset", "op1_run", "op1_fault", "op1_horn"])

    def test_the_panel_filters_and_exports(self):
        selected = []
        panel = io_table.IoTablePanel(lambda: load_config("test_config.json"), on_select=selected.append)
        panel.refresh()
        self.assertEqual(panel.table.rowCount(), 52)
        panel.filter.setText("belt_1_1_1")
        self.assertEqual([r[1] for r in panel.visible_rows()], ["speed", "directionAngle"])
        panel._double_clicked(next(r for r in range(52) if not panel.table.isRowHidden(r)), 3)
        self.assertEqual(selected, ["belt_1_1_1"])
        path = os.path.join(_SETTINGS_DIR, "io.csv")
        panel.export_csv(path)
        with open(path, newline="", encoding="utf-8") as f:
            exported = list(csv.reader(f))
        self.assertEqual(exported[0], list(io_table.COLUMNS))
        self.assertEqual(len(exported), 3)                      # the filtered rows


if __name__ == "__main__":
    unittest.main()
