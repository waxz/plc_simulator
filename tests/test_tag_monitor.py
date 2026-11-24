"""tag_monitor.py: the simulator window's PLC tag monitor (F4)."""
import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame  # noqa: E402

import plc_tags  # noqa: E402
from config_parser import load_config  # noqa: E402
from tag_monitor import TagMonitor, format_value  # noqa: E402
from visualizer import Renderer  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def tag(data_type, count=1):
    return plc_tags.Tag("c/s", "c", "s", "I", data_type, 0, 0, count=count)


class FormatTest(unittest.TestCase):
    def test_values(self):
        self.assertEqual(format_value(tag("Bool"), True), "TRUE")
        self.assertEqual(format_value(tag("Bool"), "1"), "TRUE")
        self.assertEqual(format_value(tag("Bool"), "0"), "FALSE")
        self.assertEqual(format_value(tag("Real"), 1.41421), "1.414")
        self.assertEqual(format_value(tag("UInt"), 12), "12")
        self.assertEqual(format_value(tag("Array[0..3] of Bool", 4), "0110"), "0110")
        self.assertEqual(format_value(tag("Bool"), None), "-")


class MonitorTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(os.path.join(HERE, "station_1.json"))
        self.monitor = TagMonitor(self.config)

    def test_views_cover_every_tag(self):
        tags, _ = plc_tags.tags_of(self.config)
        m = self.monitor
        self.assertFalse(m.visible)
        m.cycle()
        self.assertEqual(len(m.rows()), len(tags))
        m.cycle()
        self.assertTrue(all(t.is_input for t, _v, _c in m.rows()))
        m.cycle()
        self.assertTrue(all(not t.is_input for t, _v, _c in m.rows()))
        m.cycle()
        self.assertFalse(m.visible)

    def test_a_change_is_highlighted_then_fades(self):
        m = self.monitor
        m.cycle()
        belt = next(t for t, _v, _c in m.rows() if t.data_type == "Real" and not t.is_input)
        component = next(b for b in self.config.belts if b.id == belt.component)
        m.rows(now=100.0)
        setattr(component, belt.signal, 0.75)
        row = next(r for r in m.rows(now=100.2) if r[0].key == belt.key)
        self.assertEqual(row[1:], ("0.750", True))
        row = next(r for r in m.rows(now=102.0) if r[0].key == belt.key)
        self.assertEqual(row[1:], ("0.750", False))

    def test_f4_and_the_wheel_in_the_window(self):
        r = Renderer(self.config)
        self.addCleanup(r.quit)
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_F4, mod=0, unicode=""))
        r.process_events()
        self.assertEqual(r.tag_monitor.view, "all")
        r.render(None)
        self.assertIsNotNone(r.tag_monitor.rect)
        pos = r.tag_monitor.rect.center
        zoom = r.zoom
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=5, pos=pos))
        r.process_events()
        self.assertEqual(r.zoom, zoom, "the wheel over the panel scrolls it, not the canvas")
        self.assertGreater(r.tag_monitor.scroll, 0)


if __name__ == "__main__":
    unittest.main()
