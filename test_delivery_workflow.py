"""
test_delivery_workflow.py - the whole design-to-control workflow for a
parcel delivery line, as a user does it:

  1. Layout editor: import the hall plan (DXF), put it at the origin.
  2. Place components from the library: linear belt -> curved belt ->
     linear belt, two IR sensors, a parcel source, a sink, Start/Stop push
     buttons and Run/Jam LEDs. Save the layout.
  3. Simulator: load the saved layout, exchange I/O over TCP.
  4. plc_mock.py (a separate process, the real CLI): receives every button
     and sensor, runs the PLC program (plc_logic.py) and sends belt speeds
     and LEDs back.
  5. Press Start: belts run, parcels pass the sensors and are delivered.
     Press Stop: belts stop.

No channel configuration anywhere: the layout's own ids (button_1,
sensor_1, belt_1, ...) define the TCP I/O image (mqtt_data_exchange.io_topics).
"""
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf  # noqa: E402
from PySide6.QtWidgets import QApplication, QDialog  # noqa: E402

from background_dialog import BackgroundImportDialog  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow  # noqa: E402
from metrics_store import MetricsStore  # noqa: E402
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config, io_topics  # noqa: E402
from physics import PhysicsEngine  # noqa: E402
from sim_runner import SimulationRunner  # noqa: E402

_app = QApplication.instance() or QApplication([])
HERE = os.path.dirname(os.path.abspath(__file__))

# The hall in the DXF (mm), far from the origin as real drawings are. Its
# point HALL_ORIGIN becomes the layout's (0, 0).
HALL = (48000.0, 77000.0, 64000.0, 89000.0)
HALL_ORIGIN = (50000.0, 80000.0)
SPEED = 2.5


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(predicate, timeout: float, poll: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return predicate()


class DeliveryWorkflowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(self.tmp.cleanup)
        self.layout_path = os.path.join(self.tmp.name, "delivery.json")
        self.dxf_path = os.path.join(self.tmp.name, "hall.dxf")
        doc = ezdxf.new()
        doc.units = ezdxf.units.MM
        x0, y0, x1, y1 = HALL
        doc.modelspace().add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True)
        doc.saveas(self.dxf_path)
        with open(self.layout_path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())

    # -- 1 + 2: design it in the editor ----------------------------------------------

    def design(self):
        window = MainWindow(config=load_config(self.layout_path), config_path=self.layout_path,
                            library_path=os.path.join(self.tmp.name, "library.json"))
        self.addCleanup(window.deleteLater)
        window.show()
        _app.processEvents()

        def import_dialog(dialog):
            dialog.set_unit("mm")
            dialog.pick_origin(*HALL_ORIGIN)
            return QDialog.Accepted
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(self.dxf_path, "")), \
                mock.patch.object(BackgroundImportDialog, "exec", new=import_dialog):
            window.cad.import_background()
        self.assertIsNotNone(window.cad.background_item)
        bg = window.config.background
        self.assertAlmostEqual(bg.offsetX, -50.0)

        place = window.view.place_template
        place("Linear Belt", 3, 0, length=6, speed=SPEED)
        place("Curved Belt", 6, 2, radius=2, startAngle=270, endAngle=360, directionAngle=90, speed=SPEED)
        place("Linear Belt", 8, 3.5, length=3, rotation=90, speed=SPEED)
        place("IR Sensor", 4.5, 0, height=1.2, label="INFEED")
        place("IR Sensor", 8, 4, height=1.2, rotation=90, label="OUTFEED")
        place("Parcel Source", 0.6, 0, interval=1.0, minWidth=0.3, maxWidth=0.4, minHeight=0.3, maxHeight=0.4)
        place("Sink", 8, 5.4, width=1.2, height=0.8)
        place("Push Button", 1, -2, label="START")
        place("Push Button", 2.5, -2, label="STOP")
        place("LED Indicator", 4, -2, label="RUN")
        place("LED Indicator", 5, -2, label="JAM")
        kinds = sorted(i.kind for i in window.items_list)
        self.assertEqual(kinds, ["belt"] * 3 + ["button"] * 2 + ["led"] * 2 + ["sensor"] * 2 + ["sink", "source"])
        self.assertTrue(window._save_config())
        window.undo_stack.setClean()
        window.close()

        saved = load_config(self.layout_path)
        self.assertEqual([b.type for b in saved.belts], ["linear", "curved", "linear"])
        self.assertEqual([b.id for b in saved.buttons], ["button_1", "button_2"])
        self.assertEqual(saved.background.path, "hall.dxf")
        return saved

    # -- 3 + 4: simulator and PLC over TCP ----------------------------------------------

    def start_simulator(self, exchange_path):
        config = load_config(self.layout_path)
        apply_exchange_config(config, exchange_path)
        self.lock = threading.Lock()
        self.physics = PhysicsEngine(config, lock=self.lock)
        self.metrics = MetricsStore()
        self.engine = MQTTSyncEngine(config, lock=self.lock, metrics_store=self.metrics)
        self.runner = SimulationRunner(self.physics, self.engine, tick_s=0.002)
        self.addCleanup(self.engine.stop)
        self.addCleanup(self.runner.stop)
        self.runner.start()
        return config

    def start_plc(self, exchange_path):
        proc = subprocess.Popen(
            [sys.executable, "-u", os.path.join(HERE, "plc_mock.py"), self.layout_path,
             "--exchange-config", exchange_path, "--scan", "0.02", "--send-interval", "1.0", "--jam-time", "30"],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8")
        self.plc_lines = []

        def read():
            for line in proc.stdout:
                self.plc_lines.append(line.rstrip())
        threading.Thread(target=read, daemon=True).start()

        def stop():
            proc.terminate()
            proc.wait(timeout=10)
            proc.stdout.close()
        self.addCleanup(stop)
        return proc

    def plc_said(self, text: str) -> bool:
        return any(text in line for line in list(self.plc_lines))

    def press(self, config, button_id: str, hold: float = 0.3):
        """What clicking a button in the simulator window does."""
        button = next(b for b in config.buttons if b.id == button_id)
        with self.lock:
            button.sensor_states = True
        time.sleep(hold)
        with self.lock:
            button.sensor_states = False

    def test_design_import_place_save_simulate_and_control_over_tcp(self):
        saved = self.design()
        publish, subscribe = io_topics(saved)
        self.assertEqual(publish["input"]["custom_ids"], ["button_1", "button_2", "sensor_1", "sensor_2", "clock"])
        self.assertEqual(subscribe["output"]["custom_ids"], ["led_1", "led_2", "belt_1", "belt_2", "belt_3"])

        exchange = os.path.join(self.tmp.name, "exchange.json")
        with open(exchange, "w", encoding="utf-8") as f:
            json.dump({"transport": "tcp", "enabled": True, "tcpBindHost": "127.0.0.1",
                       "tcpServerHost": "127.0.0.1", "tcpPort": free_port(), "endianness": "LE"}, f)
        sim = self.start_simulator(exchange)
        proc = self.start_plc(exchange)
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == 1, 15),
                        "plc_mock did not connect:\n" + "\n".join(self.plc_lines))
        self.assertTrue(self.plc_said("Roles: start=button_1, stop=button_2"), "\n".join(self.plc_lines))
        self.assertTrue(wait_for(lambda: self.plc_said("I/O layout check: OK"), 5), "\n".join(self.plc_lines))
        self.assertTrue(self.engine.layout_match)

        # Before Start the PLC holds every belt at 0: nothing is delivered.
        self.assertTrue(wait_for(lambda: all(b.speed == 0 for b in sim.belts), 5))
        time.sleep(1.5)
        self.assertEqual(self.physics.delivered.get("sink_1", 0), 0)

        # Start: the PLC sees the button, runs the belts, tracks parcels.
        self.press(sim, "button_1")
        self.assertTrue(wait_for(lambda: self.plc_said("START (button_1)"), 5), "\n".join(self.plc_lines))
        self.assertTrue(wait_for(lambda: all(abs(b.speed - SPEED) < 1e-6 for b in sim.belts), 5))
        self.assertTrue(next(l for l in sim.leds if l.id == "led_1").isOn, "RUN LED on")
        self.assertTrue(wait_for(lambda: self.physics.delivered.get("sink_1", 0) >= 2, 25),
                        "no parcels delivered:\n" + "\n".join(self.plc_lines[-20:]))
        self.assertTrue(wait_for(lambda: self.plc_said("parcel at sensor_1"), 5))
        self.assertTrue(wait_for(lambda: self.plc_said("delivered 2"), 5), "\n".join(self.plc_lines[-20:]))

        # Stop: belts go to 0 and parcels stay where they are.
        self.press(sim, "button_2")
        self.assertTrue(wait_for(lambda: self.plc_said("STOP (button_2)"), 5))
        self.assertTrue(wait_for(lambda: all(b.speed == 0 for b in sim.belts), 5))
        self.assertFalse(next(l for l in sim.leds if l.id == "led_1").isOn)
        time.sleep(1.0)
        delivered = self.physics.delivered.get("sink_1", 0)
        time.sleep(2.0)
        self.assertEqual(self.physics.delivered.get("sink_1", 0), delivered, "nothing moves after Stop")
        self.assertIsNone(proc.poll(), "plc_mock is still running")


if __name__ == "__main__":
    unittest.main()
