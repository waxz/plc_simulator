"""
test_component_library.py - reusable components: snapshot any placed
component (a belt of any type, sensor, source, sink, button, LED) into the
library, update / rename / remove it, place copies, and move libraries
between machines as JSON files (component_library.py + the editor's
Component Library panel).
"""
import json
import math
import os
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from cad.belts import path_belt  # noqa: E402
from component_library import (  # noqa: E402
    FORMAT, LibraryEntry, entry_from_model, load_library, save_library, unique_name,
)
from config_parser import Belt, Sensor, SimulatorConfig, load_config  # noqa: E402
from layout_editor import MainWindow  # noqa: E402

_app = QApplication.instance() or QApplication([])
Q = math.tan(math.pi / 8)


def merge_belt():
    return Belt(id="belt_7", type="linear", shape="quadrilateral", x=12, y=-3, length=4, beltWidth=1, rotation=15,
                speed=1.4, directionAngle=30, color="#ff8800",
                vertexs=[{"x": -2, "y": -0.5}, {"x": 2, "y": -1}, {"x": 2, "y": 1}, {"x": -2, "y": 0.5}])


class LibraryFileTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def test_snapshot_drops_id_position_and_runtime_state(self):
        sensor = Sensor(id="sensor_3", x=5, y=6, width=0.1, height=1.2, rotation=90, label="IN", color="#f00",
                        sensor_states="1", isActive=True)
        entry = entry_from_model("Infeed eye", sensor)
        self.assertEqual(entry.kind, "sensor")
        self.assertNotIn("id", entry.model)
        self.assertNotIn("sensor_states", entry.model)
        self.assertEqual((entry.model["x"], entry.model["y"], entry.model["rotation"]), (0.0, 0.0, 90))
        copy = entry.instantiate("sensor_9", 1, 2)
        self.assertEqual((copy.id, copy.x, copy.y, copy.height, copy.label), ("sensor_9", 1, 2, 1.2, "IN"))
        self.assertIsNone(copy.sensor_states)

    def test_belts_of_every_type_round_trip_through_a_file(self):
        path = path_belt("belt_2", [(0, 0), (4, 0), (6, 2)], [0, Q, 0], False, 0.8)
        curved = Belt(id="belt_3", type="curved", shape="arc", x=3, y=3, radius=2, beltWidth=1, rotation=0,
                      startAngle=0, endAngle=90, speed=1, directionAngle=-90, color="#00f")
        entries = [entry_from_model(n, m) for n, m in (("Merge", merge_belt()), ("Curve 90", curved),
                                                        ("S path", path))]
        file = os.path.join(self.tmp.name, "lib.json")
        save_library(file, entries)
        with open(file, encoding="utf-8") as f:
            self.assertEqual(json.load(f)["format"], FORMAT)
        loaded, problems = load_library(file)
        self.assertEqual(problems, [])
        self.assertEqual([e.name for e in loaded], ["Merge", "Curve 90", "S path"])
        merge = loaded[0].instantiate("belt_1", 1, 1)
        self.assertEqual(merge.vertexs, merge_belt().vertexs)
        self.assertEqual((merge.rotation, merge.directionAngle, merge.speed), (15, 30, 1.4))
        self.assertEqual(loaded[1].instantiate("b", 0, 0).directionAngle, -90)
        self.assertEqual([n.bulge for n in loaded[2].instantiate("b", 0, 0).path], [0, Q, 0])

    def test_invalid_entries_are_skipped_and_reported(self):
        file = os.path.join(self.tmp.name, "lib.json")
        good = entry_from_model("Eye", Sensor(id="s", x=0, y=0, width=0.1, height=1, rotation=0, label="E",
                                              color="#f00")).to_json()
        with open(file, "w", encoding="utf-8") as f:
            json.dump({"format": FORMAT, "version": 1, "components": [
                good, {"name": "Bad kind", "kind": "robot", "model": {}},
                {"name": "No width", "kind": "belt", "model": {"type": "linear"}}]}, f)
        loaded, problems = load_library(file)
        self.assertEqual([e.name for e in loaded], ["Eye"])
        self.assertEqual(len(problems), 2)
        with open(file, "w", encoding="utf-8") as f:
            json.dump({"belts": []}, f)  # a layout, not a library
        with self.assertRaisesRegex(ValueError, "not a component library"):
            load_library(file)

    def test_unique_name(self):
        self.assertEqual(unique_name("Merge", ["Merge", "Merge (2)"]), "Merge (3)")
        self.assertEqual(unique_name("New", ["Merge"]), "New")


class LibraryPanelTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.layout = os.path.join(self.tmp.name, "layout.json")
        with open(self.layout, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig(belts=[merge_belt()]).model_dump_json())
        self.library_file = os.path.join(self.tmp.name, "my_library.json")
        self.window = self.open()

    def open(self):
        window = MainWindow(config=load_config(self.layout), config_path=self.layout, library_path=self.library_file)
        self.addCleanup(self._close, window)
        return window

    @staticmethod
    def _close(window):
        window.undo_stack.setClean()
        window.close()
        window.deleteLater()

    def select(self, item_id, window=None):
        window = window or self.window
        window.scene.clearSelection()
        item = next(i for i in window.items_list if i.model.id == item_id)
        item.setSelected(True)
        return item

    def panel(self, window=None):
        return (window or self.window).library_panel

    def test_curved_belt_is_a_built_in(self):
        template = self.panel().list_widget.get_template("Curved Belt")
        belt = template.factory("belt_9")
        self.assertEqual((belt.type, belt.shape), ("curved", "arc"))

    def test_add_selected_belt_place_it_and_it_is_kept_for_next_time(self):
        self.select("belt_7")
        template = self.panel().add_selected("Merge 15")
        self.assertIsNotNone(template)
        item = self.window.view.place_template("Merge 15", 2, 3)
        self.assertEqual(item.model.id, "belt_8")
        self.assertEqual((item.model.x, item.model.y), (2, 3))
        self.assertEqual(item.model.vertexs, merge_belt().vertexs)
        self.assertTrue(os.path.exists(self.library_file), "saved on change")
        reopened = self.open()
        self.assertIsNotNone(self.panel(reopened).list_widget.get_template("Merge 15"))
        reopened.undo_stack.setClean()
        self.window.undo_stack.setClean()

    def test_add_needs_exactly_one_selection_and_names_stay_unique(self):
        self.window.scene.clearSelection()
        self.assertIsNone(self.panel().add_selected("Nothing"))
        self.select("belt_7")
        self.panel().add_selected("Linear Belt")         # a built-in's name: kept apart
        self.assertIsNotNone(self.panel().list_widget.get_template("Linear Belt (2)"))
        self.assertFalse(self.panel().list_widget.get_template("Linear Belt").removable)

    def test_update_rename_remove(self):
        lib = self.panel().list_widget
        self.select("belt_7")
        self.panel().add_selected("Merge")
        item = self.select("belt_7")
        item.model.speed = 2.5
        lib.setCurrentItem(lib._row("Merge"))
        self.panel().update_from_selection()
        self.assertEqual(lib.get_template("Merge").factory("x").speed, 2.5)
        self.assertTrue(self.panel().rename_current("Merge 2.5"))
        self.assertIsNone(lib.get_template("Merge"))
        self.assertEqual(lib.get_template("Merge 2.5").factory("x").speed, 2.5)
        self.assertTrue(self.panel().remove_current())
        self.assertIsNone(lib.get_template("Merge 2.5"))
        loaded, _ = load_library(self.library_file)
        self.assertEqual(loaded, [])
        # Built-ins can't be updated, renamed or removed.
        lib.setCurrentItem(lib._row("Linear Belt"))
        self.assertFalse(self.panel().remove_current())
        self.assertFalse(self.panel().rename_current("X"))
        self.assertFalse(self.panel().update_btn.isEnabled())

    def test_export_then_import_on_another_machine(self):
        self.select("belt_7")
        self.panel().add_selected("Merge")
        shared = os.path.join(self.tmp.name, "shared.json")
        self.assertEqual(self.panel().export_library(shared), 1)

        other_library = os.path.join(self.tmp.name, "other_user.json")
        other = MainWindow(library_path=other_library)
        self.addCleanup(self._close, other)
        self.assertEqual(self.panel(other).import_library(shared), 1)
        self.assertEqual(self.panel(other).import_library(shared), 1)  # again: kept apart, not overwritten
        names = self.panel(other).list_widget.templates
        self.assertIn("Merge", names)
        self.assertIn("Merge (2)", names)
        self.assertEqual(len(load_library(other_library)[0]), 2)
        other.undo_stack.setClean()

    def test_import_of_a_bad_file_changes_nothing(self):
        bad = os.path.join(self.tmp.name, "bad.json")
        with open(bad, "w", encoding="utf-8") as f:
            f.write("{not json")
        before = dict(self.panel().list_widget.templates)
        self.assertEqual(self.panel().import_library(bad), 0)
        self.assertEqual(self.panel().list_widget.templates.keys(), before.keys())


if __name__ == "__main__":
    unittest.main()
