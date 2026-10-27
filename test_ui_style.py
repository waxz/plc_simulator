"""
test_ui_style.py - the tidy, operator-panel look of both windows: labels a
fixed small size, only where they fit, never on top of each other; no debug
text on the canvas by default; one status bar.
"""
import os
import tempfile
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pygame  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import hmi_style as hmi  # noqa: E402
from config_parser import (  # noqa: E402
    Belt, BeltConnection, ControlButton, LedIndicator, SimulatorConfig, load_config,
)
from layout_editor import MainWindow, to_scene_pos  # noqa: E402
from physics import PhysicsEngine  # noqa: E402
from visualizer import Renderer  # noqa: E402

_app = QApplication.instance() or QApplication([])


def straight(id, x, length, y=0.0):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=1, rotation=0, speed=1,
                directionAngle=0, color="#9ca3af")


class SimulatorWindowTest(unittest.TestCase):
    def setUp(self):
        self.config = SimulatorConfig(
            belts=[straight("long_belt", 0, 12), straight("tiny", 7, 0.3)],
            buttons=[ControlButton(id=f"b{i}", x=-3, y=-2 - i * 0.25, width=0.7, height=0.25,
                                   label=f"L{i} START") for i in range(3)],
            leds=[LedIndicator(id="lamp", x=-5, y=-2, label="AUTO", isOn=True)])
        self.r = Renderer(self.config)
        self.addCleanup(self.r.quit)
        self.physics = PhysicsEngine(self.config)

    def labels_after_render(self, zoom):
        """Texts actually placed on screen by one render."""
        self.r.zoom = zoom
        drawn = []

        def capture():
            placed = Renderer._flush_labels(self.r)
            drawn.extend(placed)
            return placed
        self.r._flush_labels = capture
        self.r.render(self.physics)
        return drawn

    def test_queued_labels_never_overlap_except_forced(self):
        r = self.r
        r._labels = []
        r._queue_label("first", (300, 300), r.PRIORITY_DEVICE)
        r._queue_label("second", (302, 301), r.PRIORITY_DEVICE)        # on top of the first
        r._queue_label("alarm", (304, 300), r.PRIORITY_ALARM, force=True)
        # The forced alarm goes first; "first" overlaps it but... both are
        # checked against what is already placed: only the alarm is forced.
        self.assertEqual(r._flush_labels(), ["alarm"])
        r._queue_label("first", (300, 300), r.PRIORITY_DEVICE)
        r._queue_label("second", (302, 301), r.PRIORITY_DEVICE)
        r._queue_label("apart", (600, 500), r.PRIORITY_DEVICE)
        self.assertEqual(r._flush_labels(), ["first", "apart"])

    def test_small_text_only_where_it_fits(self):
        self.assertEqual(self.r.label_font.get_height(), pygame.font.Font(hmi.bundled_font_path(), hmi.LABEL_PX)
                         .get_height())
        close = self.labels_after_render(zoom=1.0)
        self.assertIn("long_belt", close)
        self.assertNotIn("tiny", close, "a 0.3 m belt has no room for its id")
        far = self.labels_after_render(zoom=0.05)
        self.assertNotIn("long_belt", far)

    def test_no_debug_text_by_default(self):
        self.physics.add_parcel(0, 0, 0.4, 0.4)
        self.physics.update(0.1)
        quiet = self.labels_after_render(zoom=1.0)
        self.r.show_values = True
        verbose = self.labels_after_render(zoom=1.0)
        def is_speed(text):
            try:
                float(text.split("/")[0])
                return True
            except ValueError:
                return False
        self.assertFalse(any(map(is_speed, quiet)), "no parcel speed readouts until V")
        self.assertTrue(any(map(is_speed, verbose)))

    def test_status_bar_with_an_exchange(self):
        class Exchange:
            _enabled = True

            def stats(self):
                return {"clients": 1, "rx_size_errors": 0}
        self.r.exchange = Exchange()
        self.r.render(self.physics)
        # The bar is the panel colour along the top edge.
        self.assertEqual(self.r.screen.get_at((self.r.width - 2, 2))[:3], hmi.PANEL_DARK)


class EditorWindowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.window = MainWindow(config=load_config("test_config.json"), config_path="test_config.json",
                                 library_path=os.path.join(self.tmp.name, "lib.json"))
        self.window.resize(1400, 900)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def view_at(self, scale):
        v = self.window.view
        v.resetTransform()
        v.scale(scale, scale)
        v.centerOn(to_scene_pos(17, -15))
        v.viewport().repaint()
        _app.processEvents()
        return v

    def shown(self):
        return {i.model.id: i.tag for i in self.window.items_list if i.tag.shown}

    def test_tags_fit_and_do_not_overlap(self):
        v = self.view_at(1.0)
        tags = self.shown()
        self.assertIn("belt_1_1_2", tags)                     # a 10 m belt carries its id
        self.assertNotIn("belt_1_2_3", tags, "a 0.4 m belt doesn't")
        rects = []
        for tag in tags.values():
            origin = v.mapFromScene(tag.scenePos())
            rects.append(tag.boundingRect().translated(origin.x(), origin.y()).adjusted(1, 2, -1, -2))
        for i, a in enumerate(rects):
            for b in rects[i + 1:]:
                self.assertFalse(a.intersects(b))

    def test_tags_keep_their_size_and_hide_when_zoomed_out(self):
        self.view_at(1.0)
        tag = next(i.tag for i in self.window.items_list if i.model.id == "belt_1_1_2")
        size = tag.boundingRect().height()
        self.view_at(3.0)
        self.assertEqual(tag.boundingRect().height(), size, "constant screen size")
        self.view_at(0.05)
        self.assertEqual(self.shown(), {})
        item = next(i for i in self.window.items_list if i.model.id == "belt_1_1_2")
        item.setSelected(True)
        self.view_at(0.05)
        self.assertIn("belt_1_1_2", self.shown(), "the selected item's tag always shows")

    def test_no_debug_text_and_quiet_ports(self):
        from layout_editor import BeltItem
        import inspect
        source = inspect.getsource(BeltItem)
        self.assertNotIn("speed=", source)
        cad = self.window.cad
        self.assertFalse(cad.ports.visible)
        self.assertEqual(cad.ports.items, [], "no port markers by default")
        cad.set_connections([BeltConnection(fromBelt="belt_1_1_1", toBelt="belt_1_1_2")])
        self.assertEqual(len(cad.ports.items), 1, "connections are still marked")

    def test_dark_theme(self):
        self.assertIn(hmi.hex_(hmi.PANEL), self.window.styleSheet())
        self.assertEqual(self.window.scene.backgroundBrush().color().name(), hmi.hex_(hmi.BG))


if __name__ == "__main__":
    unittest.main()
