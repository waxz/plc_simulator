"""
test_control_panel.py - an operator control panel whose push buttons, lamps
and buzzer are its properties (control_panel.py): they become ordinary
buttons / lamps on its face, reach the PLC like any other, and a saved
layout keeps only the panel with its elements.
"""
import json
import math
import unittest

import control_panel
import st_export
from config_parser import ControlPanel, PanelElement, SimulatorConfig, load_config
from mqtt_data_exchange import io_topics
from mqtt_message_binding import ComponentMessage, custom_component_mapping


def panel(**kw):
    kw.setdefault("elements", control_panel.default_elements())
    return ControlPanel(id="op1", x=10.0, y=5.0, width=1.6, height=0.8, **kw)


class ChildrenTest(unittest.TestCase):
    def test_elements_become_buttons_and_lamps_on_the_face(self):
        buttons, lamps = control_panel.children(panel())
        self.assertEqual([b.id for b in buttons], ["op1_start", "op1_stop", "op1_reset"])
        self.assertEqual([l.id for l in lamps], ["op1_run", "op1_fault", "op1_horn"])
        self.assertEqual([l.indicator for l in lamps], ["lamp", "lamp", "buzzer"])
        stop = buttons[1]
        self.assertTrue(stop.normallyClosed)
        self.assertEqual((stop.label, stop.panel), ("STOP", "op1"))
        # 6 elements: 4 per row, 2 rows; START top left of a 1.6 x 0.8 panel
        self.assertAlmostEqual(buttons[0].x, 10.0 - 0.8 + 0.2)
        self.assertAlmostEqual(buttons[0].y, 5.0 + 0.4 - 0.2)
        for child in (*buttons, *lamps):             # all on the panel
            self.assertLessEqual(abs(child.x - 10.0), 0.8)
            self.assertLessEqual(abs(child.y - 5.0), 0.4)

    def test_they_turn_with_the_panel(self):
        buttons, _lamps = control_panel.children(panel(rotation=90.0))
        # local (-0.6, 0.2) turned 90 degrees CCW: (-0.2, -0.6)
        self.assertAlmostEqual(buttons[0].x, 10.0 - 0.2)
        self.assertAlmostEqual(buttons[0].y, 5.0 - 0.6)
        self.assertEqual(buttons[0].rotation, 90.0)

    def test_element_names_are_plc_identifiers_and_unique(self):
        with self.assertRaises(ValueError):
            PanelElement(name="start button")
        with self.assertRaises(ValueError):
            panel(elements=[PanelElement(name="a"), PanelElement(kind="lamp", name="a")])


class LayoutFileTest(unittest.TestCase):
    def test_saved_with_the_panel_only_and_rebuilt_on_load(self):
        config = load_config("station_1.json")
        own_buttons, own_lamps = len(config.buttons), len(config.leds)
        config.controlPanels.append(panel())
        control_panel.refresh(config)
        self.assertEqual(len(config.buttons), own_buttons + 3)
        saved = json.loads(config.model_dump_json())
        self.assertEqual(len(saved["buttons"]), own_buttons)        # no derived buttons in the file
        self.assertEqual(len(saved["leds"]), own_lamps)
        self.assertEqual(len(saved["controlPanels"][0]["elements"]), 6)
        loaded = SimulatorConfig.model_validate(saved)
        self.assertEqual([b.id for b in loaded.buttons], [b.id for b in config.buttons])
        self.assertEqual([l.id for l in loaded.leds], [l.id for l in config.leds])

    def test_refresh_follows_edits_and_keeps_runtime_state(self):
        config = load_config("station_1.json")
        config.controlPanels.append(panel())
        control_panel.refresh(config)
        start = next(b for b in config.buttons if b.id == "op1_start")
        start.sensor_states = True                    # held by the operator
        p = config.controlPanels[0]
        p.elements = [e for e in p.elements if e.name != "horn"]
        p.x += 1.0
        control_panel.refresh(config)
        self.assertNotIn("op1_horn", [l.id for l in config.leds])
        start = next(b for b in config.buttons if b.id == "op1_start")
        self.assertTrue(start.sensor_states)
        self.assertAlmostEqual(start.x, 11.0 - 0.8 + 0.2)
        config.controlPanels.clear()                  # a deleted panel takes its devices along
        control_panel.refresh(config)
        self.assertFalse(any(b.panel for b in config.buttons) or any(l.panel for l in config.leds))


class ExchangeTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config("logistics_full.json")
        self.config.controlPanels.append(panel())
        control_panel.refresh(self.config)

    def test_buttons_to_the_plc_lamps_and_buzzer_from_it(self):
        pub, sub = io_topics(self.config)
        self.assertEqual([i for i in pub["input"]["custom_ids"] if i.startswith("op1")],
                         ["op1_start", "op1_stop", "op1_reset"])
        self.assertEqual([i for i in sub["output"]["custom_ids"] if i.startswith("op1")],
                         ["op1_run", "op1_fault", "op1_horn"])
        import ax_export
        from config_parser import AxExport
        types = ax_export.export(st_export.contract(self.config),
                                 AxExport(output="x.st", maxPayload=0)).files["x.st"]     # 248 bytes: no limit here
        for name in ("op1_start", "op1_stop", "op1_run", "op1_horn"):
            self.assertIn(f"{name} : BOOL;", types)

    def test_a_press_and_a_lamp_on_the_wire(self):
        import copy
        plc = copy.deepcopy(self.config)
        inputs = custom_component_mapping(self.config, ["op1_start"], direction="out")
        next(b for b in self.config.buttons if b.id == "op1_start").sensor_states = True
        ComponentMessage(inputs).deserialize(ComponentMessage(inputs).serialize(self.config), plc)
        self.assertTrue(next(b for b in plc.buttons if b.id == "op1_start").sensor_states)
        outputs = custom_component_mapping(self.config, ["op1_horn"], direction="in")
        next(l for l in plc.leds if l.id == "op1_horn").isOn = True
        ComponentMessage(outputs).deserialize(ComponentMessage(outputs).serialize(plc), self.config)
        self.assertTrue(next(l for l in self.config.leds if l.id == "op1_horn").isOn)


if __name__ == "__main__":
    unittest.main()
