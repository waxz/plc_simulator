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

from PySide6.QtWidgets import QApplication, QDialog  # noqa: E402

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


# ---------------------------------------------------------------------------
# CAD tools (layout_editor_cad.py) driven through the real window
# ---------------------------------------------------------------------------
import math  # noqa: E402
from unittest import mock  # noqa: E402

import ezdxf  # noqa: E402
from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402

from cad.ports import belt_ports, heading_difference  # noqa: E402
from config_parser import BeltConnection, SimulatorConfig  # noqa: E402
from layout_editor import to_scene_pos  # noqa: E402


def _straight(id, x, y, length, rotation):
    return Belt(id=id, type="linear", x=x, y=y, length=length, beltWidth=1, rotation=rotation,
                speed=1.0, directionAngle=0, color="#3b82f6")


class CadToolsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "cad.json")
        config = SimulatorConfig(belts=[_straight("a", 2.5, 0, 5, 0), _straight("b", 6, 3, 6, 90)])
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(config.model_dump_json())
        self.window = MainWindow(config=load_config(self.path), config_path=self.path)
        self.window.resize(1000, 700)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def item(self, belt_id):
        return next(i for i in self.window.items_list if i.model.id == belt_id)

    def select(self, *ids):
        self.window.scene.clearSelection()
        for belt_id in ids:
            self.item(belt_id).setSelected(True)

    def assertConnected(self, a_id, b_id):
        pa, pb = belt_ports(self.item(a_id).model).outlet, belt_ports(self.item(b_id).model).inlet
        self.assertAlmostEqual(math.hypot(pa.x - pb.x, pa.y - pb.y), 0.0, places=5)
        self.assertAlmostEqual(heading_difference(pa.heading, pb.heading), 0.0, places=3)

    def fillet(self, radius=2.0):
        self.select("a", "b")
        with mock.patch("layout_editor_cad.QInputDialog.getDouble", return_value=(radius, True)):
            self.window.cad.fillet_selected()

    def test_fillet_adds_a_tangent_curve_and_undo_restores_everything(self):
        before = {i: self.item(i).model.model_dump() for i in ("a", "b")}
        self.fillet(2.0)
        curve = next(i for i in self.window.items_list if i.model.type == "curved")
        self.assertConnected("a", curve.model.id)
        self.assertConnected(curve.model.id, "b")
        self.assertAlmostEqual(self.item("a").model.length, 4.0, places=5)
        self.assertEqual(len(self.window.config.connections), 2)
        # The canvas item follows the model (scene position = model position).
        self.assertAlmostEqual(self.item("a").pos().x(), to_scene_pos(2.0, 0).x(), places=3)

        self.window.undo_stack.undo()
        self.assertNotIn(curve, self.window.items_list)
        self.assertEqual(self.window.config.connections, [])
        for belt_id, dumped in before.items():
            self.assertEqual(self.item(belt_id).model.model_dump(), dumped)
        self.window.undo_stack.redo()
        self.assertIn(curve, self.window.items_list)

    def test_connect_attaches_b_to_a(self):
        self.select("a", "b")
        self.window.cad.connect_selected()
        self.assertConnected("a", "b")
        self.assertEqual([(c.fromBelt, c.toBelt) for c in self.window.config.connections], [("a", "b")])
        self.assertAlmostEqual(self.item("b").model.rotation % 360, 0.0, places=4)

    def _drag(self, item, dx_m, dy_m):
        view = self.window.view
        start = view.mapFromScene(item.mapToScene(item.boundingRect().center()))
        end = view.mapFromScene(item.mapToScene(item.boundingRect().center()) + QPointF(dx_m * 50, -dy_m * 50))
        vp = view.viewport()
        for kind, pos, button, buttons in (
            (QEvent.MouseButtonPress, start, Qt.LeftButton, Qt.LeftButton),
            (QEvent.MouseMove, (start + end) / 2, Qt.NoButton, Qt.LeftButton),
            (QEvent.MouseMove, end, Qt.NoButton, Qt.LeftButton),
            (QEvent.MouseButtonRelease, end, Qt.LeftButton, Qt.NoButton),
        ):
            event = QMouseEvent(kind, QPointF(pos), vp.mapToGlobal(QPointF(pos)), button, buttons, Qt.NoModifier)
            _app.sendEvent(vp, event)
        _app.processEvents()

    def test_dragging_a_belt_drags_its_connected_chain_in_one_undo_step(self):
        self.fillet(2.0)
        curve_id = next(i for i in self.window.items_list if i.model.type == "curved").model.id
        b_before = self.item("b").model.model_dump()
        steps_before = self.window.undo_stack.count()
        self._drag(self.item("a"), 3.3, -2.1)
        self.assertNotEqual(self.item("a").model.x, 2.0, "drag didn't move the belt")
        self.assertConnected("a", curve_id)
        self.assertConnected(curve_id, "b")
        self.assertEqual(self.window.undo_stack.count(), steps_before + 1, "move + follow should be one step")
        self.window.undo_stack.undo()
        self.assertEqual(self.item("b").model.model_dump(), b_before)

    def test_check_layout_reports_into_the_panel(self):
        issues = self.window.cad.run_check()
        self.assertTrue(any(i.kind == "open outlet" for i in issues))
        self.assertEqual(self.window.cad.check_panel.list.count(), len(issues))

    def test_connections_and_background_survive_save_and_reopen(self):
        plan = os.path.join(self.tmp.name, "plan.dxf")
        doc = ezdxf.new()
        doc.units = ezdxf.units.MM
        doc.modelspace().add_lwpolyline([(0, 0), (20000, 0), (20000, 10000)])
        doc.saveas(plan)
        with mock.patch("layout_editor_cad.QFileDialog.getOpenFileName", return_value=(plan, "")), \
                mock.patch("layout_editor_cad.BackgroundImportDialog.exec", return_value=QDialog.Accepted):
            self.window.cad.import_background()
        self.assertIsNotNone(self.window.cad.background_item)
        self.assertEqual(self.window.config.background.path, "plan.dxf")  # stored relative
        self.fillet(1.5)
        self.assertTrue(self.window._save_config())

        reopened = MainWindow(config=load_config(self.path), config_path=self.path)
        self.addCleanup(reopened.deleteLater)
        self.assertEqual(len(reopened.config.connections), 2)
        self.assertIsNotNone(reopened.cad.background_item)
        self.assertAlmostEqual(reopened.cad.background_item.boundingRect().width(), 20 * 50, delta=1)

        reopened.undo_stack.setClean()
        self.window.undo_stack.undo()  # fillet
        self.window.undo_stack.undo()  # background
        self.assertIsNone(self.window.cad.background_item)

    def test_export_dxf(self):
        out = os.path.join(self.tmp.name, "out.dxf")
        with mock.patch("layout_editor_cad.QFileDialog.getSaveFileName", return_value=(out, "")):
            self.window.cad.export_dxf()
        self.assertEqual(len(ezdxf.readfile(out).modelspace().query('LWPOLYLINE[layer=="BELTS"]')), 2)
