"""
test_ui_trees.py - the UI organised in folders and trees instead of flat
lists: the component library as a folder tree (with your own sub-folders),
the layout as a kind-grouped tree synced with the canvas, menus grouped into
sub-menus, and the simulator window's folder toolbar and component tree.
"""
import os
import tempfile
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtGui import QKeySequence  # noqa: E402
from PySide6.QtWidgets import QApplication, QMenu  # noqa: E402

from component_library import load_library  # noqa: E402
from config_parser import load_config  # noqa: E402
from layout_editor import MainWindow  # noqa: E402
from editor_ui import TOP_BAR  # noqa: E402
from visualizer import Renderer  # noqa: E402

_app = QApplication.instance() or QApplication([])


class EditorTreesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.library_file = os.path.join(self.tmp.name, "lib.json")
        self.window = self.open()

    def open(self):
        window = MainWindow(config=load_config("test_config.json"), config_path="test_config.json",
                            library_path=self.library_file)
        self.addCleanup(self._close, window)
        return window

    @staticmethod
    def _close(window):
        window.undo_stack.setClean()
        window.close()
        window.deleteLater()

    def item(self, component_id):
        return next(i for i in self.window.items_list if i.model.id == component_id)

    # -- component library ---------------------------------------------------------

    def test_library_is_a_folder_tree(self):
        tree = self.window.library_panel.list_widget
        top = [tree.topLevelItem(i).text(0) for i in range(tree.topLevelItemCount())]
        self.assertEqual(top, ["Belts", "Sensors", "Parcel Flow", "Operator Panel", "My Components"])
        belts = tree.topLevelItem(0)
        self.assertEqual([belts.child(i).text(0) for i in range(belts.childCount())], ["Linear Belt", "Curved Belt"])
        self.assertEqual(tree.folder_of(tree._row("Push Button")), "Operator Panel")
        # Components drag, folders don't.
        self.assertIsNotNone(tree.mimeData([tree._row("Sink")]))
        self.assertIsNone(tree.mimeData([belts]))

    def test_your_components_go_in_sub_folders_and_keep_them(self):
        panel = self.window.library_panel
        tree = panel.list_widget
        self.item("belt_1_2_1").setSelected(True)
        panel.add_selected("Lane infeed", folder="Station 1/Lanes")
        self.assertEqual(tree.folder_of(tree._row("Lane infeed")), "My Components/Station 1/Lanes")
        # Adding while a folder is current puts it there.
        self.window.scene.clearSelection()
        self.item("button_1_1_2").setSelected(True)
        tree.setCurrentItem(tree._folders["My Components/Station 1"])
        panel.add_selected("Stop button")
        self.assertEqual(tree.folder_of(tree._row("Stop button")), "My Components/Station 1")
        # Move, then the emptied folder disappears; saved and loaded with folders.
        tree.setCurrentItem(tree._row("Lane infeed"))
        self.assertTrue(panel.move_current("Station 1"))
        self.assertNotIn("My Components/Station 1/Lanes", tree._folders)
        entries = {e.name: e.folder for e in load_library(self.library_file)[0]}
        self.assertEqual(entries, {"Lane infeed": "Station 1", "Stop button": "Station 1"})
        reopened = self.open()
        rtree = reopened.library_panel.list_widget
        self.assertEqual(rtree.folder_of(rtree._row("Stop button")), "My Components/Station 1")

    def test_library_search(self):
        panel = self.window.library_panel
        panel.search.setText("belt")
        tree = panel.list_widget
        self.assertFalse(tree._row("Linear Belt").isHidden())
        self.assertTrue(tree._row("Sink").isHidden())
        self.assertTrue(tree._folders["Operator Panel"].isHidden())
        panel.search.clear()
        self.assertFalse(tree._row("Sink").isHidden())

    # -- component manager -------------------------------------------------------

    def test_manager_groups_by_kind_with_details(self):
        tree = self.window.manager_panel.tree
        folders = [tree.topLevelItem(i).text(0) for i in range(tree.topLevelItemCount())]
        self.assertEqual(folders, ["Belts  (16)", "Sensors  (3)", "Sources  (3)", "Sinks  (1)", "Buttons  (9)",
                                   "LEDs  (9)"])
        sensors = tree.topLevelItem(1)
        rows = {sensors.child(i).text(0): (sensors.child(i).text(1), sensors.child(i).text(2))
                for i in range(sensors.childCount())}
        self.assertEqual(rows["sensor_1_1_1"], ("L1 IR", "IR"))
        self.assertEqual(rows["sensor_1_2_1"], ("L2 BANNER", "laser banner, 32 beams"))
        buttons = tree.topLevelItem(4)
        self.assertEqual(buttons.child(0).text(2), "momentary NO, key 1")

    def test_manager_and_canvas_selection_follow_each_other(self):
        panel = self.window.manager_panel
        self.assertTrue(panel.select_component("belt_1_3_2"))
        self.assertTrue(self.item("belt_1_3_2").isSelected())
        self.window.scene.clearSelection()
        self.item("led_1_2_3").setSelected(True)
        chosen = [r.text(0) for r in panel.tree.selectedItems()]
        self.assertEqual(chosen, ["led_1_2_3"])

    def test_manager_filter_and_folded_folders_survive_refresh(self):
        panel = self.window.manager_panel
        panel.search.setText("L2 ")
        ids = [r.data(0, 0x0100).model.id for r in panel._rows()]
        self.assertIn("source_2", ids)                       # by its label, L2 INFEED
        self.assertIn("button_1_2_1", ids)
        self.assertFalse([i for i in ids if "_1_1_" in i or "_1_3_" in i], ids)
        panel.search.clear()
        belts = panel.tree.topLevelItem(0)
        belts.setExpanded(False)
        panel.refresh()
        self.assertFalse(panel.tree.topLevelItem(0).isExpanded())

    # -- menus -------------------------------------------------------------------------

    def menu(self, *path) -> QMenu:
        """The (sub-)menu at the end of `path`, by title, checking it hangs
        off the one before."""
        menus = self.window.findChildren(QMenu)
        menu = next(m for m in menus if m.title() == path[0] and m.parent() is self.window.menuBar())
        for name in path[1:]:
            menu = next(m for m in menus if m.title() == name and m.parent() is menu)
        return menu

    def test_menus_are_grouped_and_keep_their_shortcuts(self):
        sketch = {a.text(): a.shortcut().toString() for a in self.menu("Tools", "Sketch").actions() if a.text()}
        self.assertEqual(sketch["Line"], "S")
        self.assertEqual(sketch["Polygon"], "Shift+G")
        belts = {a.text(): a.shortcut().toString() for a in self.menu("Tools", "Belts").actions()}
        self.assertEqual(belts["Belt from Sketch/Drawing"], "B")
        connections = [a.text() for a in self.menu("Tools", "Connections").actions() if a.text()]
        self.assertEqual(connections, ["Connect Belts", "Fillet Belts...", "Disconnect Belts", "Solve Connections"])
        background = [a.text() for a in self.menu("File", "Background (DXF)").actions() if a.text()]
        self.assertIn("Import DXF Background...", background)
        panels = [a.text() for a in self.menu("View", "Panels").actions() if a.text()]
        for dock in ("Component Library", "Component Manager", "Properties", "Layout Check"):
            self.assertIn(dock, panels)
        check = next(a for a in self.menu("Tools").actions() if a.text() == "Check Layout")
        self.assertEqual(check.shortcut(), QKeySequence("F7"))


class SimulatorTreesTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config("test_config.json")
        self.r = Renderer(self.config)
        self.addCleanup(self.r.quit)
        self.ui = self.r.editor_ui

    def test_tool_bar_is_one_row_under_the_status_bar(self):
        # No component tools: layouts are designed in the layout editor.
        titles = [label.text for label, _buttons in self.ui.toolbar_groups]
        self.assertEqual(titles, ["SIMULATION", "LAYOUT", "VIEW"])
        bar = self.ui.toolbar_panel.get_abs_rect()
        self.assertEqual((bar.x, bar.y, bar.width), (0, TOP_BAR, self.r.width))
        buttons = [b for _label, group in self.ui.toolbar_groups for b in group]
        self.assertEqual(len(buttons), 7)
        for left, right in zip(buttons, buttons[1:]):
            self.assertLess(left.get_abs_rect().right, right.get_abs_rect().left)   # left to right
        for b in buttons:
            self.assertTrue(bar.contains(b.get_abs_rect()))
        # The inspector starts below the tool bar.
        self.assertEqual(self.ui.prop_panel.get_abs_rect().y, bar.bottom)
        self.assertFalse(self.ui.drawing_btn.is_enabled)     # this layout has no drawing

    def test_component_window_is_a_tree(self):
        self.ui.open_manager_panel()
        folders = list(self.ui.manager_folder_rows)
        self.assertEqual(len(folders), 6)
        self.assertTrue(folders[0].endswith("Belts (16)"))
        self.assertIn("      sensor_1_1_1  ·  L1 IR", self.ui.manager_items)
        self.ui.manager_collapsed ^= {"belt"}
        self.ui.refresh_manager_panel()
        self.assertFalse(any(k == "belt" for k, _i, _m in self.ui.manager_items.values()))
        self.assertTrue(next(iter(self.ui.manager_folder_rows)).startswith("+"))


if __name__ == "__main__":
    unittest.main()
