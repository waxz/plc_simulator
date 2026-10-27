"""
test_layout_editor.py - the Qt layout editor (layout_editor.py) as a
design-time tool: open a layout, edit it, save it, and run the saved file
in the simulator without losing anything the editor doesn't itself edit
(drive settings, traction model, exchange settings, ...).

Runs with Qt's offscreen platform, so no window appears.
"""
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from config_parser import Belt, DriveConfig, load_config  # noqa: E402
from layout_editor import BeltItem, MainWindow  # noqa: E402
from physics import PhysicsEngine  # noqa: E402

_app = QApplication.instance() or QApplication([])


class LayoutEditorRoundTripTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "layout.json")
        config = load_config("test_config.json")
        config.tractionModel = "friction"
        config.physicsDt = 0.002
        config.mqttSettings.beltFeedback = True
        config.belts[0].drive = DriveConfig(accel=1.5, tau=0.1)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(config.model_dump_json(indent=2))
        self.original = load_config(self.path)

    def open_window(self) -> MainWindow:
        window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.addCleanup(window.deleteLater)
        return window

    def test_opens_every_component_of_the_layout(self):
        window = self.open_window()
        c = self.original
        expected = len(c.belts) + len(c.sensors) + len(c.sources) + len(c.sinks) + len(c.buttons) + len(c.leds)
        self.assertEqual(len(window.items_list), expected)
        self.assertIn("layout.json", window.windowTitle())

    def test_save_without_edits_preserves_everything_including_simulation_settings(self):
        window = self.open_window()
        self.assertTrue(window._save_config())
        saved = load_config(self.path)
        self.assertEqual(saved.model_dump(), self.original.model_dump())
        self.assertEqual(saved.tractionModel, "friction")
        self.assertEqual(saved.belts[0].drive.accel, 1.5)
        self.assertTrue(saved.mqttSettings.beltFeedback)

    def test_moved_and_added_components_are_saved_and_run_in_the_simulator(self):
        window = self.open_window()
        first = next(item for item in window.items_list if isinstance(item, BeltItem))
        first.model.x += 1.25
        new_belt = Belt(id="belt_new", type="linear", x=50, y=50, length=3, beltWidth=1, rotation=30,
                        speed=1.0, directionAngle=0, color="#ffffff")
        window.view._add_item(BeltItem(new_belt), text="Add Belt")
        self.assertIn("*", window.windowTitle(), "unsaved edits should mark the title")

        self.assertTrue(window._save_config())
        self.assertNotIn("*", window.windowTitle())
        saved = load_config(self.path)
        self.assertEqual(len(saved.belts), len(self.original.belts) + 1)
        moved = next(b for b in saved.belts if b.id == first.model.id)
        self.assertAlmostEqual(moved.x, next(b for b in self.original.belts if b.id == moved.id).x + 1.25)
        added = next(b for b in saved.belts if b.id == "belt_new")
        self.assertEqual(added.rotation, 30)

        engine = PhysicsEngine(saved)  # the saved layout is runnable as-is
        engine.run_for(0.5)
        self.assertAlmostEqual(engine.sim_time, 0.5)

    def test_undo_removes_an_added_component_before_saving(self):
        window = self.open_window()
        new_belt = Belt(id="belt_tmp", type="linear", x=50, y=50, length=3, beltWidth=1, rotation=0,
                        speed=1.0, directionAngle=0, color="#ffffff")
        window.view._add_item(BeltItem(new_belt), text="Add Belt")
        window.undo_stack.undo()
        self.assertTrue(window.undo_stack.isClean())
        self.assertTrue(window._save_config())
        self.assertNotIn("belt_tmp", [b.id for b in load_config(self.path).belts])

    def test_new_layout_without_a_file_saves_to_the_given_path(self):
        path = os.path.join(self.tmp.name, "new_layout.json")
        window = MainWindow(config_path=path)
        self.addCleanup(window.deleteLater)
        self.assertTrue(window._save_config())
        self.assertEqual(len(load_config(path).belts), 2)  # the two seed belts


if __name__ == "__main__":
    unittest.main()
