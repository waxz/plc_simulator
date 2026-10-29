"""
test_catalogue.py - every built-in component of the designer's library,
end to end: it is in its catalogue folder with a description, places on
the canvas (place_template - what dropping it does), shows its property
grid, survives save / load, and the saved layout runs in the simulation
and the 3D view. Plus the library's description panel and search, and
the curved roller conveyor.
"""
import json
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

import scene3d  # noqa: E402
from config_parser import SimulatorConfig, load_config  # noqa: E402
from layout_editor import ComponentLibrary, MainWindow  # noqa: E402
from physics import PhysicsEngine  # noqa: E402

_app = QApplication.instance() or QApplication([])


class CatalogueTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.path = os.path.join(self.tmp, "layout.json")
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig().model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.resize(1400, 900)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)
        self.library = self.window.library_panel.list_widget

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def test_folders_and_descriptions(self):
        top = [self.library.topLevelItem(i).text(0) for i in range(self.library.topLevelItemCount())]
        self.assertEqual(top[:3], ["Conveyors", "Sensors", "Identification"])
        self.assertEqual(top[-1], "My Components")
        self.assertEqual(self.library.folder_of(self.library._row("Roller Conveyor")), "Conveyors/Rollers")
        self.assertEqual(self.library.folder_of(self.library._row("Curved Roller Conveyor")), "Conveyors/Rollers")
        self.assertEqual(self.library.folder_of(self.library._row("Linear Belt")), "Conveyors/Belts")
        self.assertEqual(self.library.folder_of(self.library._row("90° Transfer")), "Conveyors/Transfers")
        for folder, name, _kind, _factory, description in ComponentLibrary.CATALOGUE:
            self.assertTrue(description, name)
            self.assertIn(description, self.library._row(name).toolTip(0))
        # The description panel follows the selection; search finds descriptions too.
        self.library.setCurrentItem(self.library._row("Wheel Diverter"))
        self.assertIn("Pop-up wheels", self.window.library_panel.description.text())
        self.library.set_filter("pallet")
        visible = [name for _f, name, *_ in ComponentLibrary.CATALOGUE if not self.library._row(name).isHidden()]
        self.assertIn("90° Transfer", visible)                         # "...lifts a pallet..."
        self.assertIn("Pallet", visible)
        self.assertNotIn("IR Sensor", visible)
        self.library.set_filter("")

    def test_every_component_places_edits_saves_and_runs(self):
        view, panel = self.window.view, self.window.properties_panel
        placed = []
        for i, (_folder, name, kind, _factory, _d) in enumerate(ComponentLibrary.CATALOGUE):
            with self.subTest(component=name):
                item = view.place_template(name, (i % 6) * 4.0, (i // 6) * 4.0)
                self.assertIsNotNone(item)
                self.assertEqual(item.kind, kind)
                self.window.scene.clearSelection()
                item.setSelected(True)
                _app.processEvents()
                self.assertTrue(panel.tree.isVisible())
                self.assertIsNotNone(panel.current)
                placed.append(item.model.id)
        self.assertTrue(self.window._save_config())
        config = load_config(self.path)
        with open(self.path, encoding="utf-8") as f:
            saved = json.load(f)
        ids = {c["id"] for value in saved.values() if isinstance(value, list) for c in value
               if isinstance(c, dict) and "id" in c}
        self.assertLessEqual(set(placed), ids)
        engine = PhysicsEngine(config)
        engine.run_for(2.0)                                             # every machine steps without error
        scene = scene3d.static_scene(config, self.path)
        json.dumps(scene3d.dynamic_state(config, engine.parcel_views(), engine.sim_time))
        rollers = [b for b in scene["belts"] if b["roller"]]
        self.assertEqual({r["roller"]["kind"] for r in rollers}, {"line", "arc"})

    def test_curved_roller_conveyor(self):
        item = self.window.view.place_template("Curved Roller Conveyor", 0.0, 0.0)
        self.assertEqual((item.model.type, item.model.surface), ("curved", "roller"))
        self.window.scene.clearSelection()
        item.setSelected(True)
        _app.processEvents()
        self.assertEqual(self.window.properties_panel.combos["surface"].currentText(), "roller")
        self.assertIn("curved roller", self.window.manager_panel.describe(item))


if __name__ == "__main__":
    unittest.main()
