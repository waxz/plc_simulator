"""
test_io_signals.py - a component's PLC signals by name (io_signals.py) and
the "manual" command source: the signals match the exchange mapping, set by
hand only in manual and with the wire's types, PLC frames leave manual
components alone, a forced sensor keeps its output, manual machines follow
their commands (a diverter: run, speed, direction); the simulator window's
inspector shows the PLC I/O live and sets commands by hand there and in the
simulation process; the designer lists the signals and the command source.
"""
import os
import shutil
import tempfile
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
_SETTINGS_DIR = tempfile.mkdtemp()
os.environ["CONVEYOR_SIM_SETTINGS"] = os.path.join(_SETTINGS_DIR, "settings.json")  # never the user's

import pygame  # noqa: E402
import pygame_gui  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402

import io_signals  # noqa: E402
import logistics  # noqa: E402
from config_parser import (Belt, ControlButton, Diverter, LedIndicator, Sensor, SimulatorConfig,  # noqa: E402
                           Source, Stopper, load_config)
from mqtt_message_binding import MACHINE_FIELDS, ComponentMessage, paddng_component  # noqa: E402
from physics import PhysicsEngine  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "logistics_demo.json")
_app = QApplication.instance() or QApplication([])


def belt(id="b", **kw):
    args = dict(id=id, type="linear", x=5.0, y=0.0, length=10.0, beltWidth=1.0, rotation=0.0, speed=1.0,
                directionAngle=0.0, color="#888888")
    args.update(kw)
    return Belt(**args)


def sensor(id="s", **kw):
    args = dict(id=id, x=5.0, y=0.0, width=0.1, height=1.0, rotation=0.0, label="S", color="#ff0000")
    args.update(kw)
    return Sensor(**args)


def names(signals, direction):
    return [s.name for s in signals if s.direction == direction]


class SignalsTest(unittest.TestCase):
    def test_belts_by_command_format(self):
        signals = io_signals.signals(belt())
        self.assertEqual(names(signals, "command"), ["speed", "directionAngle"])
        self.assertEqual(names(signals, "feedback"),
                         ["running", "atSpeed", "reversing", "fault", "actualSpeed", "encoderCount"])
        self.assertEqual({s.name: s.plc_type for s in signals}["encoderCount"], "DWORD")
        vfd = io_signals.signals(belt(commandFormat="vfd"))
        self.assertEqual([(s.name, s.plc_type) for s in vfd if s.direction == "command"],
                         [("run", "BOOL"), ("reverse", "BOOL"), ("speed", "REAL")])

    def test_sensors_buttons_and_lamps(self):
        [ir] = io_signals.signals(sensor())
        self.assertEqual((ir.name, ir.plc_type, ir.direction, ir.settable), ("sensor_states", "BOOL", "feedback", True))
        [banner] = io_signals.signals(sensor(sensorType="laser_banner", laserCount=12, beamDistance=0.05))
        self.assertEqual((banner.count, banner.plc_type), (12, "ARRAY[0..11] OF BOOL"))
        [packed] = io_signals.signals(sensor(packFormat="byte"))
        self.assertEqual(packed.plc_type, "BYTE")
        [button] = io_signals.signals(ControlButton(id="go", x=0, y=0, label="GO"))
        self.assertEqual((button.direction, button.settable), ("feedback", False))   # pressed by the operator
        [lamp] = io_signals.signals(LedIndicator(id="l", x=0, y=0, label="L"))
        self.assertEqual((lamp.name, lamp.direction, lamp.settable), ("isOn", "command", True))
        [mode] = io_signals.signals(LedIndicator(id="l", x=0, y=0, label="L", packFormat="byte"))
        self.assertEqual((mode.name, mode.plc_type), ("mode", "BYTE"))

    def test_machines_match_the_exchange_fields(self):
        config = load_config(DEMO)
        seen = set()
        for kind in MACHINE_FIELDS:
            for machine in getattr(config, kind):
                if kind == "sinks" and not machine.capacity:
                    self.assertEqual(io_signals.signals(machine), [])          # a plain sink: no signals
                    continue
                seen.add(kind)
                signals = io_signals.signals(machine)
                for direction, side in (("command", "in"), ("feedback", "out")):
                    bits, values = MACHINE_FIELDS[kind][side]
                    self.assertEqual(sorted(names(signals, direction)), sorted(n for _, n in bits + values), kind)
        self.assertGreaterEqual(seen, {"scanners", "diverters", "doors", "safety", "sinks"})
        self.assertEqual(io_signals.signals(Source(id="s", x=0, y=0, rotation=0, interval=1, minWidth=0.3,
                                                   maxWidth=0.3, minHeight=0.3, maxHeight=0.3, label="S")), [])

    def test_the_signals_are_the_wire_mapping(self):
        config = load_config(DEMO)
        for component in [*config.belts, *config.diverters, *config.scanners]:
            for direction, side in (("command", "in"), ("feedback", "out")):
                wire = [p.split("/")[1] for _, p in paddng_component(config, [component.id], side)
                        if not p.startswith("padding")]
                self.assertEqual(names(io_signals.signals(component), direction), wire, component.id)

    def test_control_options(self):
        config = load_config(DEMO)
        self.assertEqual(io_signals.control_options(config.diverters[0]), ["auto", "plc", "manual"])
        self.assertEqual(io_signals.control_options(belt()), ["plc", "manual"])
        self.assertEqual(io_signals.control_options(sensor()), ["sim", "manual"])
        self.assertEqual(io_signals.control_options(config.doors[0]), ["plc", "manual"])
        self.assertEqual(io_signals.control_options(config.scanners[0]), [])        # no commands


class ApplyTest(unittest.TestCase):
    def setUp(self):
        self.config = SimulatorConfig(
            belts=[belt()], sensors=[sensor(), sensor("banner", sensorType="laser_banner", laserCount=4,
                                                      beamDistance=0.05, sensor_states="0000")],
            leds=[LedIndicator(id="lamp", x=0, y=0, label="L", packFormat="byte")],
            diverters=[Diverter(id="d", x=5, y=0)])

    def test_only_in_manual(self):
        c = self.config
        self.assertIn("not in manual", io_signals.apply(c, "b", "speed", 0.5))
        self.assertEqual(c.belts[0].speed, 1.0)
        self.assertIn("one of", io_signals.apply(c, "b", "control", "auto"))         # a belt has no auto
        self.assertIsNone(io_signals.apply(c, "b", "control", "manual"))
        self.assertIsNone(io_signals.apply(c, "b", "speed", "0.5"))
        self.assertEqual(c.belts[0].speed, 0.5)
        self.assertIn("no settable signal", io_signals.apply(c, "b", "actualSpeed", 3.0))   # feedback
        self.assertIn("no component", io_signals.apply(c, "nope", "control", "manual"))
        self.assertIn("is not a REAL", io_signals.apply(c, "b", "speed", "fast"))

    def test_types_as_on_the_wire(self):
        c = self.config
        for cid in ("s", "banner", "lamp", "d"):
            io_signals.apply(c, cid, "control", "manual")
        self.assertIsNone(io_signals.apply(c, "s", "sensor_states", True))
        self.assertTrue(io_signals.read(c, ["s"])["s"]["values"]["sensor_states"])
        self.assertIsNone(io_signals.apply(c, "banner", "sensor_states", "0110"))
        self.assertEqual(c.sensors[1].sensor_states, "0110")
        self.assertIsNone(io_signals.apply(c, "lamp", "mode", 258))                 # BYTE: masked
        self.assertEqual(c.leds[0].mode, 2)
        self.assertIsNone(io_signals.apply(c, "d", "divert", 1))
        self.assertIs(c.diverters[0].divert, True)
        values = io_signals.read(c, ["d", "missing"])
        self.assertEqual(list(values), ["d"])
        self.assertEqual(values["d"]["control"], "manual")


class ManualModeTest(unittest.TestCase):
    def test_plc_frames_leave_manual_components_alone(self):
        config = SimulatorConfig(belts=[belt("a"), belt("m", control="manual")])
        mapping = paddng_component(config, ["a", "m"], "in")
        sender = SimulatorConfig(belts=[belt("a", speed=2.5), belt("m", speed=2.5)])
        payload = ComponentMessage(mapping).serialize(sender)
        ComponentMessage(mapping).deserialize(payload, config, skip_manual=True)
        self.assertEqual([b.speed for b in config.belts], [2.5, 1.0])
        ComponentMessage(mapping).deserialize(payload, config)        # the PLC side: no such rule
        self.assertEqual([b.speed for b in config.belts], [2.5, 2.5])

    def test_the_simulator_skips_manual_on_its_inbound_frames(self):
        import inspect
        import mqtt_data_exchange
        self.assertIn("skip_manual=True", inspect.getsource(mqtt_data_exchange.MQTTSyncEngine._make_recv_handler))

    def test_a_forced_sensor_keeps_its_output(self):
        config = SimulatorConfig(belts=[belt()], sensors=[sensor("free"), sensor("forced", control="manual")],
                                 useSurfaceVelocity=True)
        engine = PhysicsEngine(config)
        config.sensors[1].sensor_states = True                        # nothing in front of it
        engine.run_for(0.5)
        self.assertTrue(io_signals.read(config, ["forced"])["forced"]["values"]["sensor_states"])
        self.assertFalse(io_signals.read(config, ["free"])["free"]["values"]["sensor_states"])
        config.sensors[1].sensor_states = False                       # a parcel passes: still off
        engine.add_parcel(4.0, 0.0, 0.3, 0.3)
        engine.run_for(2.0)
        self.assertEqual(config.sensors[0].risingEdges, 1)
        self.assertFalse(io_signals.read(config, ["forced"])["forced"]["values"]["sensor_states"])

    def test_a_diverter_runs_at_its_commanded_speed_and_direction(self):
        d = Diverter(id="d", x=5.0, y=0.0, divertAngle=-90.0, speed=1.0)
        self.assertEqual(logistics.diverter_motion(d), (1.0, -90.0))
        d.commandSpeed, d.commandAngle = 0.4, 90.0
        self.assertEqual(logistics.diverter_motion(d), (1.0, -90.0))       # auto: the layout's
        d.control = "manual"
        self.assertEqual(logistics.diverter_motion(d), (0.4, 90.0))
        d.commandSpeed = d.commandAngle = 0.0
        self.assertEqual(logistics.diverter_motion(d), (1.0, -90.0))       # 0: the layout's
        commands = [s.name for s in io_signals.signals(d) if s.direction == "command"]
        self.assertEqual(commands, ["divert", "commandSpeed", "commandAngle"])

    def test_manual_machines_follow_their_commands(self):
        config = SimulatorConfig(belts=[belt()], useSurfaceVelocity=True,
                                 diverters=[Diverter(id="d", x=5.0, y=0.0, control="manual")],
                                 stoppers=[Stopper(id="st", x=8.0, y=0.0, control="manual")])
        engine = PhysicsEngine(config)
        engine.run_for(0.2)
        self.assertFalse(config.diverters[0].active)
        self.assertTrue(config.stoppers[0].isUp)
        self.assertIsNone(io_signals.apply(config, "d", "divert", True))
        self.assertIsNone(io_signals.apply(config, "st", "release", True))
        engine.run_for(1.0)
        self.assertTrue(config.diverters[0].active)
        self.assertFalse(config.stoppers[0].isUp)


class SimulatorInspectorTest(unittest.TestCase):
    """The simulator window's inspector (editor_ui.py): PLC I/O live, and
    commands set by hand in manual."""

    @classmethod
    def setUpClass(cls):
        from visualizer import Renderer
        cls.config = load_config(DEMO)
        cls.config.sensors.append(sensor("eye", x=3.0, y=0.0))
        cls.engine = PhysicsEngine(cls.config)
        cls.r = Renderer(cls.config, headless=False)
        cls.r.physics = cls.engine
        cls.ui = cls.r.editor_ui

    @classmethod
    def tearDownClass(cls):
        pygame.display.quit()

    def inspect(self, component_id):
        item = next(c for kind in ("diverters", "belts", "sensors", "scanners") for c in getattr(self.config, kind)
                    if c.id == component_id)
        self.r.selected_item = item
        self.ui.show_props_for(item)
        return item

    def press(self, element):
        pygame.event.post(pygame.event.Event(pygame_gui.UI_BUTTON_PRESSED, {"ui_element": element}))
        self.r.process_events()

    def source_button(self, option):
        return next(b for b, o in self.ui.io_source_buttons.items() if o == option)

    def test_live_readouts_in_auto(self):
        d = self.inspect("div_2")
        self.assertEqual(set(self.ui.io_source_buttons.values()), {"auto", "plc", "manual"})
        self.assertTrue(self.source_button("auto").is_selected)
        self.assertEqual(list(self.ui.io_rows), ["divert", "commandSpeed", "commandAngle", "active", "present",
                                                 "presentCode", "diverted"])
        widget = self.ui.io_rows["divert"][1]
        self.assertIsInstance(widget, pygame_gui.elements.UILabel)       # not set by hand in auto
        d.diverted = 5
        self.ui.update_live()
        self.assertEqual(self.ui.io_rows["diverted"][1].text, "5")
        self.assertEqual(self.ui.io_rows["active"][1].text, "FALSE")

    def test_manual_commands_set_from_the_inspector(self):
        d = self.inspect("div_1")
        self.press(self.source_button("manual"))
        self.assertEqual(d.control, "manual")
        toggle = self.ui.io_rows["divert"][1]
        self.assertIsInstance(toggle, pygame_gui.elements.UIButton)
        self.press(toggle)
        self.assertTrue(d.divert)
        self.ui.update_live()
        self.assertEqual(toggle.text, "ON")
        entry = self.ui.io_rows["commandAngle"][1]
        self.assertIsInstance(entry, pygame_gui.elements.UITextEntryLine)
        entry.set_text("45")
        pygame.event.post(pygame.event.Event(pygame_gui.UI_TEXT_ENTRY_FINISHED, {"ui_element": entry,
                                                                                 "text": "45"}))
        self.r.process_events()
        self.assertEqual(d.commandAngle, 45.0)
        self.engine.run_for(0.1)
        self.assertTrue(d.active)
        self.assertEqual(logistics.diverter_motion(d)[1], 45.0)
        self.press(self.source_button("auto"))                            # back: readouts again
        self.assertEqual(d.control, "auto")
        self.assertIsInstance(self.ui.io_rows["divert"][1], pygame_gui.elements.UILabel)

    def test_typing_a_value_is_not_a_shortcut(self):
        self.inspect("infeed")
        self.press(self.source_button("manual"))
        entry = self.ui.io_rows["speed"][1]
        entry.focus()
        self.assertTrue(self.ui.typing())
        playing = self.r.playing
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, {"key": pygame.K_SPACE, "unicode": " ", "mod": 0,
                                                              "scancode": 44}))
        self.r.process_events()
        self.assertEqual(self.r.playing, playing)                         # Space typed, not Start
        entry.unfocus()
        self.press(self.source_button("plc"))

    def test_a_sensor_is_forced_in_manual(self):
        sensor = self.inspect("eye")
        self.assertEqual(set(self.ui.io_source_buttons.values()), {"sim", "manual"})
        self.press(self.source_button("manual"))
        self.press(self.ui.io_rows["sensor_states"][1])
        self.engine.run_for(0.2)
        self.assertTrue(io_signals.value_of(sensor, io_signals.signals(sensor)[0]))

    def test_a_scanner_only_reports(self):
        self.inspect("dws_1")
        self.assertEqual(self.ui.io_source_buttons, {})
        self.assertIn("weight", self.ui.io_rows)


class WindowApplyTest(unittest.TestCase):
    def test_the_window_applies_them_here_and_in_the_simulation_process(self):
        from visualizer import Renderer
        config = load_config(DEMO)
        renderer = Renderer(config, headless=True)
        self.assertFalse(renderer.set_io("div_1", "divert", True))      # auto: not by hand
        self.assertTrue(renderer.set_io("div_1", "control", "manual"))
        self.assertTrue(renderer.set_io("div_1", "divert", True))
        self.assertTrue(config.diverters[0].divert)
        sent = []
        renderer.physics = type("Proxy", (), {"set_io": lambda self, *a: sent.append(a)})()
        self.assertTrue(renderer.set_io("infeed", "control", "manual"))
        self.assertEqual(sent, [("infeed", "control", "manual")])


class ProxyTest(unittest.TestCase):
    def test_hand_set_values_reach_the_simulation_process(self):
        from sim_process import PhysicsProxy, wait_for
        config = load_config(DEMO)
        proxy = PhysicsProxy(config, {"config": DEMO, "speed": 1.0})
        proxy.start()
        self.addCleanup(proxy.stop)
        belt_ = next(b for b in config.belts if b.id == "infeed")
        proxy.set_io("infeed", "control", "manual")
        proxy.set_io("infeed", "speed", 0.25)
        proxy.set_io("div_1", "control", "manual")
        proxy.set_io("div_1", "divert", True)
        # The mirror shows the child's values: it took them.
        self.assertTrue(wait_for(proxy, lambda: belt_.speed == 0.25 and config.diverters[0].divert))


class DesignerPanelTest(unittest.TestCase):
    def setUp(self):
        from layout_editor import MainWindow
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.layout = os.path.join(self.tmp, "line.json")
        shutil.copy(DEMO, self.layout)
        self.window = MainWindow(config=load_config(self.layout), config_path=self.layout)
        self.window.resize(1400, 1000)
        self.window.show()
        _app.processEvents()
        self.addCleanup(self._close)
        self.panel = self.window.properties_panel

    def _close(self):
        self.window.undo_stack.setClean()
        self.window.close()
        self.window.deleteLater()

    def select(self, component_id):
        self.window.scene.clearSelection()
        next(i for i in self.window.items_list if i.model.id == component_id).setSelected(True)
        _app.processEvents()

    def group_rows(self, title):
        tree = self.panel.tree
        group = next(tree.topLevelItem(i) for i in range(tree.topLevelItemCount())
                     if tree.topLevelItem(i).text(0) == title)
        return [group.child(i).text(0) for i in range(group.childCount())]

    def test_signals_and_command_source(self):
        from layout_editor import IO_COMMANDS, IO_FEEDBACK, IO_GROUP
        self.select("div_1")
        self.assertEqual(self.group_rows(IO_GROUP), ["Command source"])
        self.assertEqual(self.group_rows(IO_COMMANDS), ["divert", "commandSpeed", "commandAngle"])
        self.assertEqual(self.group_rows(IO_FEEDBACK), ["active", "present", "presentCode", "diverted"])
        self.assertEqual(self.panel.tree.itemWidget(self.panel.io_rows["presentCode"], 1).text(), "DWORD")
        combo = self.panel.combos["control"]
        self.assertEqual([combo.itemText(i) for i in range(combo.count())], ["auto", "plc", "manual"])
        combo.setCurrentText("manual")                                  # a layout setting, undoable
        self.assertEqual(self.window.scene.selectedItems()[0].model.control, "manual")
        self.window.undo_stack.undo()
        self.assertEqual(combo.currentText(), "auto")
        self.select("dws_1")                                            # a scanner: feedback only
        self.assertNotIn("control", self.panel.combos)
        self.assertEqual(self.group_rows(IO_FEEDBACK)[:2], ["noRead", "present"])

    def test_a_new_diverter_type_redraws_what_it_covers(self):
        self.select("div_1")
        item = self.window.scene.selectedItems()[0]
        with_body = item.boundingRect()
        self.panel.combos["diverterType"].setCurrentText("wheels")       # no pusher body beside the belt
        self.assertLess(item.boundingRect().height(), with_body.height() - 50)
        self.window.undo_stack.undo()
        self.assertEqual(item.boundingRect(), with_body)


if __name__ == "__main__":
    unittest.main()
