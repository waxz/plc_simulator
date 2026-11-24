"""
test_equipment_geometry.py - the diverters as machines (equipment_geometry.py)
and the views that draw them from it: the pusher beside the belt on the side
it pushes away from, its plate sweeping across; the wheel module's wheels;
the designer's DiverterItem (its pusher outside its zone) and the simulator
window drawing both, idle and active.
"""
import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pygame  # noqa: E402
from PySide6.QtCore import QRectF  # noqa: E402
from PySide6.QtGui import QImage, QPainter  # noqa: E402
from PySide6.QtWidgets import QApplication, QGraphicsScene  # noqa: E402

import equipment_geometry as geo  # noqa: E402
from config_parser import Diverter, load_config  # noqa: E402

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_app = QApplication.instance() or QApplication([])


class GeometryTest(unittest.TestCase):
    def test_the_pusher_stands_opposite_where_it_pushes(self):
        right = geo.pusher_parts(1.0, 1.0, -90.0)
        self.assertEqual(right.side, 1)                                 # on the left, pushes right
        self.assertGreater(right.body[1] - right.body[3] / 2, 0.5)      # beside the belt, not on it
        self.assertEqual(geo.pusher_parts(1.0, 1.0, 90.0).side, -1)
        home, out = geo.pusher_parts(1.0, 1.0, -90.0, 0.0), geo.pusher_parts(1.0, 1.0, -90.0, 1.0)
        self.assertAlmostEqual(home.plate[1] - out.plate[1], 0.85)      # strokes across the belt
        self.assertGreater(home.plate[1], 0.5)                          # home: at the belt edge
        self.assertEqual(len(home.rods), 2)
        self.assertAlmostEqual(home.plate[2], 0.8)

    def test_wheels(self):
        centres = geo.wheel_centres(1.0, 1.0)
        self.assertEqual(len(centres), 6 * 8)
        self.assertTrue(all(abs(x) < 0.45 and abs(y) < 0.45 for x, y in centres))
        self.assertEqual(geo.wheel_steer(90.0), 60.0)
        self.assertEqual(geo.wheel_steer(-30.0), -30.0)


class DesignerTest(unittest.TestCase):
    def render(self, model):
        from layout_editor import DiverterItem
        scene = self.scene = QGraphicsScene()          # owns the item: keep it
        item = DiverterItem(model)
        scene.addItem(item)
        image = QImage(400, 400, QImage.Format_ARGB32)
        image.fill(0)
        painter = QPainter(image)
        scene.render(painter, QRectF(0, 0, 400, 400), item.sceneBoundingRect())
        painter.end()
        return item, image

    def test_the_pusher_body_is_part_of_the_item(self):
        item, image = self.render(Diverter(id="d", x=0, y=0, diverterType="pusher", divertAngle=-90.0))
        zone = item.polygon().boundingRect()
        self.assertLess(item.boundingRect().top(), zone.top() - 50)     # the body, beyond the zone
        self.assertFalse(image.isNull())
        wheels, _ = self.render(Diverter(id="w", x=0, y=0, diverterType="wheels", divertAngle=45.0))
        self.assertAlmostEqual(wheels.boundingRect().height(), wheels.polygon().boundingRect().height() + 4)


class SimulatorWindowTest(unittest.TestCase):
    def test_draws_both_idle_and_active(self):
        from visualizer import Renderer
        config = load_config(os.path.join(HERE, "logistics_demo.json"))
        renderer = Renderer(config, headless=False)
        renderer.zoom = 2.0
        for active in (False, True):
            for d in config.diverters:
                d.active = active
            renderer.render(None)
        pygame.display.quit()


if __name__ == "__main__":
    unittest.main()
