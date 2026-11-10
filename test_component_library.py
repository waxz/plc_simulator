"""
test_component_library.py - reusable components: snapshot any placed
component (a belt of any type, sensor, machine, ...) into the library,
update / rename / remove it, place copies, and move libraries between
machines as JSON files (component_library.py + the editor's Component
Library panel). Stations: several components saved as one entry and placed
again with fresh ids, their links and belt connections pointing to the new
copies (also what a paste does).
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
    FORMAT, LibraryEntry, entry_from_model, load_library, save_library, station_from_models, unique_name,
)
from config_parser import (  # noqa: E402
    Belt, BeltConnection, ControlButton, RobotArm, SafetyCircuit, Sensor, SimulatorConfig, Turntable, load_config,
)
from layout_editor import MainWindow  # noqa: E402

_app = QApplication.instance() or QApplication([])
Q = math.tan(math.pi / 8)


def straight(belt_id, x, y):
    return Belt(id=belt_id, type="linear", shape="rectangle", x=x, y=y, length=2, beltWidth=1, rotation=0,
                speed=1, directionAngle=0, color="#888")


def turn_cell():
    """A turntable on its platform belt, an infeed belt joined to it, an
    e-stop, and a safety relay stopping both belts and one outside."""
    return [("belt", straight("belt_1", 0, 0)), ("belt", straight("belt_2", 3, 0)),
            ("turntable", Turntable(id="turntable_1", x=3, y=0, beltId="belt_2")),
            ("button", ControlButton(id="estop_1", x=1, y=2, width=0.3, height=0.3, rotation=0, label="E-STOP")),
            ("safety", SafetyCircuit(id="safety_1", x=-1, y=2, stopButtonIds=["estop_1"],
                                     beltIds=["belt_1", "belt_2", "belt_99"]))]


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
                good, {"name": "Bad kind", "kind": "spaceship", "model": {}},
                {"name": "No width", "kind": "belt", "model": {"type": "linear"}}]}, f)
        loaded, problems = load_library(file)
        self.assertEqual([e.name for e in loaded], ["Eye"])
        self.assertEqual(len(problems), 2)
        with open(file, "w", encoding="utf-8") as f:
            json.dump({"belts": []}, f)  # a layout, not a library
        with self.assertRaisesRegex(ValueError, "not a component library"):
            load_library(file)

    def test_a_robot_keeps_its_pick_and_place_points_relative(self):
        robot = RobotArm(id="robot_1", x=10, y=5, pickX=11, pickY=5, placeX=10, placeY=7)
        copy = entry_from_model("Picker", robot).instantiate("robot_2", 0, 0)
        self.assertEqual((copy.pickX, copy.pickY, copy.placeX, copy.placeY), (1, 0, 0, 2))

    def test_a_station_keeps_its_links_and_connections(self):
        connections = [BeltConnection(fromBelt="belt_1", toBelt="belt_2"),
                       BeltConnection(fromBelt="belt_2", toBelt="belt_99")]          # leaves the station
        entry, dropped = station_from_models("Turn cell", turn_cell(), connections)
        self.assertEqual(dropped, ["safety_1.beltIds -> belt_99"])
        self.assertEqual(entry.connections, [{"fromBelt": "belt_1", "toBelt": "belt_2"}])
        file = os.path.join(self.tmp.name, "lib.json")
        save_library(file, [entry])
        (loaded,), problems = load_library(file)
        self.assertEqual(problems, [])
        self.assertTrue(loaded.is_station)

        def new_id(kind, old):
            return f"{old}_copy" if kind == "belt" else old
        placed, links = loaded.place_station(new_id, 100, 50)
        models = {m.id: m for _, m in placed}
        self.assertEqual(set(models), {"belt_1_copy", "belt_2_copy", "turntable_1", "estop_1", "safety_1"})
        self.assertEqual(models["turntable_1"].beltId, "belt_2_copy")
        self.assertEqual(models["safety_1"].beltIds, ["belt_1_copy", "belt_2_copy"])
        self.assertEqual(models["safety_1"].stopButtonIds, ["estop_1"])
        self.assertEqual([(c.fromBelt, c.toBelt) for c in links], [("belt_1_copy", "belt_2_copy")])
        cx = sum(m.x for _, m in turn_cell()) / 5
        self.assertAlmostEqual(models["turntable_1"].x, 100 + 3 - cx)
        with self.assertRaises(ValueError):
            loaded.instantiate("x")
        with self.assertRaisesRegex(ValueError, "two or more"):
            station_from_models("One", turn_cell()[:1])

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

    def _turn_cell_window(self):
        with open(self.layout, "w", encoding="utf-8") as f:
            f.write(SimulatorConfig(belts=[m for k, m in turn_cell() if k == "belt"],
                                    turntables=[m for k, m in turn_cell() if k == "turntable"],
                                    buttons=[m for k, m in turn_cell() if k == "button"],
                                    safety=[m for k, m in turn_cell() if k == "safety"],
                                    connections=[BeltConnection(fromBelt="belt_1", toBelt="belt_2")]
                                    ).model_dump_json())
        window = self.open()
        window.scene.clearSelection()
        for item in window.items_list:
            item.setSelected(True)
        return window

    def test_a_selection_of_several_is_saved_and_placed_as_a_station(self):
        window = self._turn_cell_window()
        template = self.panel(window).add_selected("Turn cell")
        self.assertEqual(template.kind, "station")
        self.assertIn("belt_99", window.statusBar().currentMessage())          # the dropped link, said
        before = (len(window.items_list), len(window.config.connections))
        items = window.view.place_station(template.entry, 20, 0)
        self.assertEqual(len(items), 5)
        self.assertEqual(len(window.items_list), before[0] + 5)
        models = {i.kind: i.model for i in items if i.kind != "belt"}
        belts = sorted(i.model.id for i in items if i.kind == "belt")
        self.assertEqual(belts, ["belt_3", "belt_4"])
        self.assertEqual(models["turntable"].id, "turntable_2")
        self.assertEqual(models["turntable"].beltId, "belt_4")
        self.assertEqual(models["safety"].beltIds, ["belt_3", "belt_4"])
        self.assertEqual(models["safety"].stopButtonIds, [models["button"].id])
        self.assertIn(("belt_3", "belt_4"), [(c.fromBelt, c.toBelt) for c in window.config.connections])
        self.assertEqual(sorted(i.model.id for i in window.scene.selectedItems()), sorted(i.model.id for i in items))
        window.undo_stack.undo()                                                  # one step
        self.assertEqual((len(window.items_list), len(window.config.connections)), before)
        self.assertIsNotNone(window.view.place_template("Turn cell", 0, 30))     # what dropping it does
        reloaded, _ = load_library(self.library_file)
        self.assertTrue(reloaded[0].is_station)

    def test_paste_points_links_to_the_copies(self):
        window = self._turn_cell_window()
        window.scene.clearSelection()
        for item in window.items_list:
            item.setSelected(item.model.id in ("belt_2", "turntable_1"))
        window.view._copy_selected()
        window.view._paste_clipboard()
        pasted = {i.kind: i.model for i in window.scene.selectedItems()}
        self.assertEqual(pasted["turntable"].beltId, pasted["belt"].id)
        self.assertNotEqual(pasted["belt"].id, "belt_2")
        window.scene.clearSelection()
        for item in window.items_list:
            item.setSelected(item.model.id == "turntable_1")
        window.view._copy_selected()
        window.view._paste_clipboard()
        (alone,) = window.scene.selectedItems()
        self.assertEqual(alone.model.beltId, "belt_2")                   # outside the copy: kept

    def test_import_of_a_bad_file_changes_nothing(self):
        bad = os.path.join(self.tmp.name, "bad.json")
        with open(bad, "w", encoding="utf-8") as f:
            f.write("{not json")
        before = dict(self.panel().list_widget.templates)
        self.assertEqual(self.panel().import_library(bad), 0)
        self.assertEqual(self.panel().list_widget.templates.keys(), before.keys())


if __name__ == "__main__":
    unittest.main()
