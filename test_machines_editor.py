"""
test_machines_editor.py - pallets, stoppers, robot arms and elevators in
the layout editor: placed from the library's Machines folder, edited in
Properties (outline follows), saved and loaded back, and wired into the
PLC I/O (mqtt_message_binding / io_topics).
"""
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import ComponentLibrary, MainWindow  # noqa: E402
from mqtt_data_exchange import io_topics  # noqa: E402
from mqtt_message_binding import ComponentMessage, custom_component_mapping, known_component_ids  # noqa: E402

_app = QApplication.instance() or QApplication([])


class MachinesInTheEditorTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "layout.json")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def test_the_library_has_a_machines_folder(self):
        lib = self.window.findChild(ComponentLibrary)
        folder = lib._folder("Machines")
        names = [folder.child(i).text(0) for i in range(folder.childCount())]
        self.assertEqual(names, ["Pallet", "Stopper", "Robot Arm", "Elevator"])

    def test_place_edit_save_and_load(self):
        place = self.window.view.place_template
        pallet = place("Pallet", 1.0, 0.0)
        stopper = place("Stopper", 3.0, 0.0)
        robot = place("Robot Arm", 5.0, 2.0)
        elevator = place("Elevator", 8.0, 0.0)
        self.assertEqual([i.kind for i in (pallet, stopper, robot, elevator)],
                         ["pallet", "stopper", "robot", "elevator"])
        # A dropped robot brings its pick / place points along.
        self.assertEqual((robot.model.pickX, robot.model.placeX), (6.2, 3.8))
        # Properties: the outline follows a size edit (one undo step).
        self.window.scene.clearSelection()
        pallet.setSelected(True)
        panel = self.window.properties_panel
        before = pallet.polygon().boundingRect().width()
        panel.spins["width"].setValue(1.6)
        self.assertAlmostEqual(pallet.polygon().boundingRect().width(), before * 2)
        self.window.undo_stack.undo()
        self.assertAlmostEqual(pallet.model.width, 0.8)
        self.window.scene.clearSelection()
        robot.setSelected(True)
        self.assertIn("pickX", panel.spins)
        self.assertIn("control", panel.combos)
        self.assertTrue(self.window._save_config())
        saved = load_config(self.path)
        self.assertEqual([len(saved.pallets), len(saved.stoppers), len(saved.robots), len(saved.elevators)],
                         [1, 1, 1, 1])
        self.assertAlmostEqual(saved.robots[0].pickX, 6.2)

    def test_machines_get_plc_signals(self):
        c = load_config("machines_demo.json")
        self.assertLessEqual({"st_feed", "robot_1", "lift_1"}, known_component_ids(c))
        self.assertNotIn("pal_1", known_component_ids(c))           # pallets carry no signals
        pub, sub = io_topics(c)
        feedback = custom_component_mapping(c, pub["input"]["custom_ids"], direction="out")
        commands = custom_component_mapping(c, sub["output"]["custom_ids"], direction="in")
        paths = [p for _t, p in feedback]
        for path in ("st_load/isUp", "st_load/occupied", "robot_1/busy", "robot_1/placeReady", "robot_1/cycles",
                     "lift_1/atLevel", "lift_1/level"):
            self.assertIn(path, paths)
        self.assertIn("lift_1/targetLevel", [p for _t, p in commands])
        # The command frame round-trips.
        message = ComponentMessage(commands, "<")
        c.stoppers[1].release, c.robots[0].start, c.elevators[0].targetLevel = True, True, 1
        data = message.serialize(c)
        c.stoppers[1].release, c.robots[0].start, c.elevators[0].targetLevel = False, False, 0
        message.deserialize(data, c)
        self.assertEqual((c.stoppers[1].release, c.robots[0].start, c.elevators[0].targetLevel), (True, True, 1))


if __name__ == "__main__":
    unittest.main()
