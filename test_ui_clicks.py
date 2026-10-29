"""
test_ui_clicks.py - both windows driven with simulated mouse clicks, the way
a user works: the property grid, the light/dark theme switch, the
simulation buttons (Start/Pause, Clear Parcels, Reset), Run in Simulator,
and selecting through the component tree.

Qt: QTest.mouseClick on the real widgets / canvas / menus. pygame: mouse
events posted to the window's own event loop (Renderer.process_events).
"""
import os
import shutil
import tempfile
import time
import unittest
from unittest import mock

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_SETTINGS_DIR = tempfile.mkdtemp()
os.environ["CONVEYOR_SIM_SETTINGS"] = os.path.join(_SETTINGS_DIR, "settings.json")  # never the user's

import ezdxf  # noqa: E402
import pygame  # noqa: E402
import pygame_gui  # noqa: E402
from PySide6.QtCore import QPoint, Qt  # noqa: E402
from PySide6.QtTest import QTest  # noqa: E402
from PySide6.QtWidgets import QApplication, QMenu, QToolBar  # noqa: E402

import hmi_style as hmi  # noqa: E402
from config_parser import BackgroundDrawing, load_config  # noqa: E402
from layout_editor import MainWindow  # noqa: E402
from physics import PhysicsEngine  # noqa: E402
from visualizer import Renderer  # noqa: E402

_app = QApplication.instance() or QApplication([])
HERE = os.path.dirname(os.path.abspath(__file__))


def tearDownModule():
    shutil.rmtree(_SETTINGS_DIR, ignore_errors=True)


class EditorClicksTest(unittest.TestCase):
    def setUp(self):
        hmi.set_theme("dark")
        hmi.save_settings(theme="dark")
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.layout = os.path.join(self.tmp.name, "station.json")
        shutil.copy(os.path.join(HERE, "test_config.json"), self.layout)
        self.window = MainWindow(config=load_config(self.layout), config_path=self.layout,
                                 library_path=os.path.join(self.tmp.name, "lib.json"))
        self.window.resize(1500, 950)
        self.window.show()
        QTest.qWaitForWindowExposed(self.window)
        self.addCleanup(self._close)

    def _close(self):
        hmi.set_theme("dark")
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def item(self, component_id):
        return next(i for i in self.window.items_list if i.model.id == component_id)

    def click_on_canvas(self, component_id):
        view = self.window.view
        item = self.item(component_id)
        view.centerOn(item)
        _app.processEvents()
        pos = view.mapFromScene(item.sceneBoundingRect().center())
        QTest.mouseClick(view.viewport(), Qt.LeftButton, Qt.NoModifier, pos)
        _app.processEvents()

    @staticmethod
    def indicator(checkbox) -> QPoint:
        """Where a user clicks a check box: on the box itself (left end)."""
        return QPoint(8, checkbox.height() // 2)

    def toolbar_button(self, action):
        for toolbar in self.window.findChildren(QToolBar):
            widget = toolbar.widgetForAction(action)
            if widget is not None:
                return widget
        raise AssertionError(f"no toolbar button for {action.text()}")

    # -- property grid --------------------------------------------------------------

    def test_click_a_component_then_edit_it_in_the_property_grid(self):
        panel = self.window.properties_panel
        self.assertFalse(panel.tree.isVisible())
        self.assertTrue(panel.empty.isVisible())
        self.click_on_canvas("button_1_1_2")
        self.assertTrue(self.item("button_1_1_2").isSelected())
        self.assertTrue(panel.tree.isVisible())
        self.assertIn("button_1_1_2", panel.header.text())
        groups = [panel.tree.topLevelItem(i).text(0) for i in range(panel.tree.topLevelItemCount())]
        self.assertEqual(groups, ["Identity", "Position", "Geometry", "Operation", "Display", "PLC I/O",
                                  "Feedback (simulator -> PLC)"])        # a button: its contact, to the PLC

        # Click the NC contact box: the model changes, undo puts it back.
        nc = panel.checkboxes["normallyClosed"]
        QTest.mouseClick(nc, Qt.LeftButton, Qt.NoModifier, self.indicator(nc))
        self.assertTrue(self.item("button_1_1_2").model.normallyClosed)
        self.window.undo_stack.undo()
        self.assertFalse(self.item("button_1_1_2").model.normallyClosed)
        self.assertFalse(panel.checkboxes["normallyClosed"].isChecked())

        # Click into Width, type a value, Enter.
        width = panel.spins["width"]
        QTest.mouseClick(width, Qt.LeftButton)
        width.selectAll()
        QTest.keyClicks(width, "0.9")
        QTest.keyClick(width, Qt.Key_Return)
        self.assertAlmostEqual(self.item("button_1_1_2").model.width, 0.9)

    def test_sensor_signal_settings_are_editable(self):
        self.click_on_canvas("sensor_1_1_1")
        panel = self.window.properties_panel
        box = panel.checkboxes["invert"]
        QTest.mouseClick(box, Qt.LeftButton, Qt.NoModifier, self.indicator(box))
        self.assertTrue(self.item("sensor_1_1_1").model.invert)
        self.assertEqual(panel.spins["offDelay"].suffix(), " s")
        self.assertIn("Signal", [panel.tree.topLevelItem(i).text(0) for i in range(panel.tree.topLevelItemCount())])

    # -- theme ------------------------------------------------------------------------

    def test_theme_switch_button_and_menu(self):
        button = self.toolbar_button(self.window.theme_action)
        self.assertEqual(button.text(), "Light Theme")
        QTest.mouseClick(button, Qt.LeftButton)
        self.assertEqual(hmi.THEME, "light")
        self.assertIn(hmi.hex_(hmi.THEMES["light"]["PANEL"]), self.window.styleSheet())
        self.assertEqual(self.window.scene.backgroundBrush().color().name(), hmi.hex_(hmi.THEMES["light"]["BG"]))
        self.assertEqual(hmi.load_settings()["theme"], "light", "remembered")
        self.assertEqual(button.text(), "Dark Theme")
        self.assertTrue(self.window.theme_actions["light"].isChecked())

        # Back to dark through View > Theme > Dark, clicked.
        bar = self.window.menuBar()
        view_action = next(a for a in bar.actions() if a.text() == "View")
        QTest.mouseClick(bar, Qt.LeftButton, Qt.NoModifier, bar.actionGeometry(view_action).center())
        view_menu = view_action.menu()
        self.assertTrue(view_menu.isVisible())
        theme_action = next(a for a in view_menu.actions() if a.text() == "Theme")
        QTest.mouseClick(view_menu, Qt.LeftButton, Qt.NoModifier, view_menu.actionGeometry(theme_action).center())
        theme_menu = theme_action.menu()
        for _ in range(100):                      # the sub-menu opens on a short delay
            if theme_menu.isVisible():
                break
            QTest.qWait(20)
        self.assertTrue(theme_menu.isVisible())
        dark = self.window.theme_actions["dark"]
        QTest.mouseClick(theme_menu, Qt.LeftButton, Qt.NoModifier, theme_menu.actionGeometry(dark).center())
        self.assertEqual(hmi.THEME, "dark")
        self.assertEqual(button.text(), "Light Theme")

    # -- run in simulator --------------------------------------------------------------

    def test_run_in_simulator_runs_the_canvas_without_saving(self):
        with open(self.layout, encoding="utf-8") as f:
            before = f.read()
        self.click_on_canvas("belt_1_1_2")
        self.window.properties_panel.spins["speed"].setValue(1.25)    # unsaved edit
        with mock.patch("layout_editor.subprocess.Popen") as popen:
            QTest.mouseClick(self.toolbar_button(self.window.run_sim_action), Qt.LeftButton)
        args = popen.call_args[0][0]
        self.assertTrue(args[1].endswith("main.py"))
        run_copy = args[2]
        self.assertNotEqual(os.path.normcase(run_copy), os.path.normcase(os.path.abspath(self.layout)))
        self.assertEqual(args[3:], ["--source", os.path.abspath(self.layout)])
        self.assertEqual(next(b for b in load_config(run_copy).belts if b.id == "belt_1_1_2").speed, 1.25)
        self.assertFalse(self.window.undo_stack.isClean(), "still unsaved")
        with open(self.layout, encoding="utf-8") as f:
            self.assertEqual(f.read(), before)                          # the layout file untouched
        self.assertIn("not saved", self.window.statusBar().currentMessage())

    def test_the_running_designer_shows_the_layout_asked_for(self):
        import layout_editor
        import sim_link
        link = layout_editor.attach_instance_link(_app, poll_ms=10)
        self.addCleanup(link.close)

        def ask(path):
            self.assertIsNotNone(sim_link.request_open_designer(path))
            for _ in range(100):
                _app.processEvents()
                QTest.qWait(10)
                if link._pending is None:              # the designer took it
                    break
            _app.processEvents()

        ask(self.layout)                                            # the layout it has open: just the front
        self.assertTrue(self.window.isVisible())
        other = os.path.join(os.path.dirname(self.layout), "other_layout.json")
        shutil.copy(self.layout, other)
        ask(other)                                                  # another layout: opened in its place
        shown = [w for w in _app.topLevelWidgets() if isinstance(w, MainWindow) and w.isVisible()]
        self.assertEqual([os.path.basename(w.config_path) for w in shown], ["other_layout.json"])
        for w in shown:
            self.addCleanup(w.close)
        timer = _app._instance_link[1]
        timer.stop()

    def test_run_in_simulator_reuses_the_running_simulator(self):
        from sim_link import SimulatorLink
        link = SimulatorLink(self.layout)                            # a simulator window is running
        self.addCleanup(link.close)
        with mock.patch("layout_editor.subprocess.Popen") as popen:
            QTest.mouseClick(self.toolbar_button(self.window.run_sim_action), Qt.LeftButton)
            QTest.mouseClick(self.toolbar_button(self.window.run_sim_action), Qt.LeftButton)
        popen.assert_not_called()                                   # no second simulator
        self.assertEqual(os.path.normcase(link.take_reload()), os.path.normcase(self.window._run_file))
        self.assertEqual(os.path.normcase(link.pending_source), os.path.normcase(os.path.abspath(self.layout)))
        self.assertIn("reloading", self.window.statusBar().currentMessage())
        link.close()
        with mock.patch("layout_editor.subprocess.Popen") as popen:  # it closed: start one
            popen.return_value.poll.return_value = None
            QTest.mouseClick(self.toolbar_button(self.window.run_sim_action), Qt.LeftButton)
            QTest.mouseClick(self.toolbar_button(self.window.run_sim_action), Qt.LeftButton)
        self.assertEqual(popen.call_count, 1, "the second click waits for the one starting")

    # -- component tree ------------------------------------------------------------------

    def test_click_a_row_in_the_component_tree_selects_it_on_the_canvas(self):
        tree = self.window.manager_panel.tree
        row = next(r for r in self.window.manager_panel._rows() if r.text(0) == "led_1_2_3")
        tree.scrollToItem(row)
        _app.processEvents()
        QTest.mouseClick(tree.viewport(), Qt.LeftButton, Qt.NoModifier, tree.visualItemRect(row).center())
        self.assertTrue(self.item("led_1_2_3").isSelected())
        self.assertIn("led_1_2_3", self.window.properties_panel.header.text())


class SimulatorClicksTest(unittest.TestCase):
    def setUp(self):
        hmi.set_theme("dark")
        hmi.save_settings(theme="dark")
        self.config = load_config("test_config.json")
        self.r = Renderer(self.config)
        self.addCleanup(self._close)
        self.physics = PhysicsEngine(self.config)
        self.r.physics = self.physics
        for belt in self.config.belts:
            belt.speed = 1.5
        self.physics.update(6.0)                                       # some parcels about
        self.assertGreater(len(self.physics.parcels), 0)
        self.frame()

    def _close(self):
        hmi.set_theme("dark")
        self.r.quit()

    def frame(self):
        self.r.manager.update(0.02)
        self.r.render(self.physics)

    def click_at(self, pos, button=1):
        for kind in (pygame.MOUSEMOTION, pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP):
            data = {"pos": pos}
            if kind == pygame.MOUSEMOTION:
                data.update(rel=(0, 0), buttons=(0, 0, 0))
            else:
                data["button"] = button
            pygame.event.post(pygame.event.Event(kind, data))
            self.r.process_events()
            self.r.manager.update(0.02)
        self.r.process_events()       # the UI_BUTTON_PRESSED the release produced
        self.frame()

    def click(self, ui_button):
        self.click_at(ui_button.get_abs_rect().center)

    def test_start_pause_clear_and_reset_buttons(self):
        ui = self.r.editor_ui
        self.assertFalse(self.r.playing)
        self.click(ui.sim_play_btn)
        self.assertTrue(self.r.playing)
        self.assertEqual(ui.sim_play_btn.text, "Pause")
        self.click(ui.sim_play_btn)
        self.assertFalse(self.r.playing)
        self.assertEqual(ui.sim_play_btn.text, "Start")

        self.click(ui.sim_clear_btn)
        self.assertEqual(self.physics.parcels, [])
        self.assertGreater(self.physics.sim_time, 0)                   # clear keeps the clock

        self.physics.update(3.0)
        self.click(ui.sim_reset_btn)
        self.assertEqual(self.physics.parcels, [])
        self.assertEqual(self.physics.sim_time, 0.0)
        self.assertEqual(self.physics.delivered, {})

    def test_theme_button_switches_the_whole_window(self):
        ui = self.r.editor_ui
        self.assertEqual(ui.theme_btn.text, "Light Theme")
        self.click(ui.theme_btn)
        self.assertEqual(hmi.THEME, "light")
        self.assertEqual(ui.theme_btn.text, "Dark Theme")
        canvas = self.r.screen.get_at((220, self.r.height - 40))[:3]   # empty canvas, bottom left
        self.assertIn(canvas, (hmi.BG, hmi.GRID, hmi.GRID_MAJOR, hmi.AXIS_X, hmi.AXIS_Y))
        self.assertEqual(self.r.screen.get_at((self.r.width - 3, 3))[:3], hmi.PANEL_DARK)
        self.click(ui.theme_btn)
        self.assertEqual(hmi.THEME, "dark")

    def test_layout_drawing_is_shown_and_the_button_hides_it(self):
        # A layout with a DXF background: a 4 x 2 m frame and a diagonal, in mm.
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, True)
        doc = ezdxf.new("R2010")
        doc.header["$INSUNITS"] = ezdxf.units.MM
        doc.modelspace().add_lwpolyline([(0, 0), (4000, 0), (4000, 2000), (0, 2000)], close=True)
        doc.modelspace().add_line((0, 0), (4000, 2000))
        doc.saveas(os.path.join(tmp, "hall.dxf"))
        self.config.background = BackgroundDrawing(path="hall.dxf", scale=0.001, offsetX=-50.0, offsetY=-50.0)
        self.r.config_path = os.path.join(tmp, "layout.json")
        self.r.load_drawing()
        self.assertEqual(len(self.r.drawing.lines), 2)
        self.assertEqual(self.r.drawing.bounds(), (-50.0, -50.0, -46.0, -48.0))
        ui = self.r.editor_ui
        self.assertTrue(ui.drawing_btn.is_enabled)
        # Centre the view on the frame's left edge: drawing colour there.
        self.r.zoom = 1.0
        self.r.camera_x = self.r.width / 2 + 50.0 * self.r.scale
        self.r.camera_y = self.r.height / 2 - 49.0 * self.r.scale
        self.frame()
        edge = self.r._world_to_screen(-50.0, -49.0)
        self.assertEqual(self.r.screen.get_at(edge)[:3], hmi.DRAWING)
        self.assertEqual(ui.drawing_btn.text, "Hide Drawing")
        self.click(ui.drawing_btn)
        self.assertFalse(self.r.show_drawing)
        self.assertEqual(ui.drawing_btn.text, "Show Drawing")
        self.assertNotEqual(self.r.screen.get_at(edge)[:3], hmi.DRAWING)
        # Panning moves the cached drawing with the view.
        self.click(ui.drawing_btn)
        self.r.camera_x += 30
        self.frame()
        self.assertEqual(self.r.screen.get_at((edge[0] + 30, edge[1]))[:3], hmi.DRAWING)

    def test_speed_button_and_keys(self):
        ui = self.r.editor_ui
        self.assertEqual(ui.sim_speed_btn.text, "Speed 1x")
        seen = []
        for _ in range(5):
            self.click(ui.sim_speed_btn)
            seen.append(ui.sim_speed_btn.text)
        self.assertEqual(seen, ["Speed 2x", "Speed 4x", "Speed 8x", "Speed MAX", "Speed 1x"])
        self.assertEqual(self.r.speed, 1.0)
        for key, expected in ((pygame.K_RIGHTBRACKET, 2.0), (pygame.K_RIGHTBRACKET, 4.0), (pygame.K_LEFTBRACKET, 2.0)):
            pygame.event.post(pygame.event.Event(pygame.KEYDOWN, {"key": key, "unicode": "", "mod": 0}))
            self.r.process_events()
            self.assertEqual(self.r.speed, expected)

    def test_the_scene_is_drawn_under_the_simulation_lock(self):
        import threading
        lock = threading.Lock()
        self.r.sim_lock = lock
        lock.acquire()                                   # the simulation thread mid-step
        threading.Timer(0.3, lock.release).start()
        t = time.monotonic()
        self.frame()
        self.assertGreaterEqual(time.monotonic() - t, 0.25)

    def test_edit_in_designer_opens_the_layout_editor(self):
        self.r.config_path = "test_config.json"
        with mock.patch("visualizer.subprocess.Popen") as popen:
            self.click(self.r.editor_ui.designer_btn)
        args = popen.call_args[0][0]
        self.assertTrue(args[1].endswith("layout_editor.py"))
        self.assertEqual(args[2], "test_config.json")

    def test_edit_in_designer_reuses_the_running_designer(self):
        from sim_link import InstanceLink
        self.r.config_path = "test_config.json"
        link = InstanceLink("designer")                              # a designer is open
        self.addCleanup(link.close)
        with mock.patch("visualizer.subprocess.Popen") as popen:
            self.click(self.r.editor_ui.designer_btn)
            self.click(self.r.editor_ui.designer_btn)
        popen.assert_not_called()                                   # no second designer
        self.assertEqual(os.path.normcase(link.take()), os.path.normcase(os.path.abspath("test_config.json")))
        link.close()
        with mock.patch("visualizer.subprocess.Popen") as popen:    # none running: start one, once
            popen.return_value.poll.return_value = None
            self.click(self.r.editor_ui.designer_btn)
            self.click(self.r.editor_ui.designer_btn)
        self.assertEqual(popen.call_count, 1)

    def centre_on(self, item):
        self.r.zoom = 1.0
        self.r.camera_x = self.r.width / 2 - item.x * self.r.scale
        self.r.camera_y = self.r.height / 2 + item.y * self.r.scale
        return self.r._world_to_screen(item.x, item.y)

    def test_clicked_component_is_inspected_read_only(self):
        ui = self.r.editor_ui
        sensor = next(s for s in self.config.sensors if s.sensorType == "ir")
        pos = self.centre_on(sensor)
        self.click_at(pos)
        self.assertIs(ui.selected_item, sensor)
        self.assertEqual(ui.prop_title.text, "SENSOR")
        self.assertEqual(ui.prop_rows["id"][1].text, sensor.id)
        self.assertTrue(ui.prop_rows["x"][1].text.endswith(" m"))
        self.assertEqual(ui.live_rows["State"][1].text, "clear")
        # read-only: labels, no text entries or Delete / Copy
        self.assertFalse(any(isinstance(w, pygame_gui.elements.UITextEntryLine)
                             for w in ui.prop_panel.get_container().elements))
        # a drag that starts on the component pans the view; the sensor stays put
        x, y = sensor.x, sensor.y
        camera = self.r.camera_x
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, {"pos": pos, "button": 1}))
        pygame.event.post(pygame.event.Event(pygame.MOUSEMOTION, {"pos": (pos[0] + 40, pos[1]), "rel": (40, 0),
                                                                   "buttons": (1, 0, 0)}))
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, {"pos": (pos[0] + 40, pos[1]), "button": 1}))
        self.r.process_events()
        self.assertEqual((sensor.x, sensor.y), (x, y))
        self.assertEqual(self.r.camera_x, camera + 40)

    def test_belt_inspector_shows_the_live_drive(self):
        ui = self.r.editor_ui
        belt = self.config.belts[0]
        belt.running, belt.actualSpeed = False, 0.0
        ui.show_props_for(belt)
        self.assertEqual(ui.live_rows["Drive"][1].text, "stopped")
        belt.running, belt.actualSpeed = True, 1.25
        self.frame()
        self.assertEqual(ui.live_rows["Drive"][1].text, "running")
        self.assertEqual(ui.live_rows["Actual speed"][1].text, "1.25 m/s")

    def test_click_a_push_button_on_the_canvas_while_running(self):
        ui = self.r.editor_ui
        self.click(ui.sim_play_btn)
        button = self.config.buttons[0]
        self.r.camera_x, self.r.camera_y, self.r.zoom = self.r.width / 2, self.r.height / 2, 1.0
        self.r.camera_x -= button.x * self.r.scale
        self.r.camera_y += button.y * self.r.scale
        pos = self.r._world_to_screen(button.x, button.y)
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, {"pos": pos, "button": 1}))
        self.r.process_events()
        self.assertIs(button.sensor_states, True)
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, {"pos": pos, "button": 1}))
        self.r.process_events()
        self.assertIs(button.sensor_states, False)


if __name__ == "__main__":
    unittest.main()
