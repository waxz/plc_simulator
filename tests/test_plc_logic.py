"""
test_plc_logic.py - the generic PLC program (plc_logic.DeliveryController)
and the layout-derived TCP I/O channels (mqtt_data_exchange.io_topics).
"""
import unittest

from config_parser import Belt, ControlButton, LedIndicator, Sensor, SimulatorConfig, Sink, load_config
from mqtt_data_exchange import PUBLISH_TOPICS, SUBSCRIBE_TOPICS, io_topics
from mqtt_message_binding import ComponentMessage, custom_component_mapping
from plc_logic import DeliveryController, is_on


def layout(buttons=("START", "STOP"), leds=("RUN", "JAM")):
    return SimulatorConfig(
        belts=[Belt(id="belt_1", type="linear", x=0, y=0, length=4, beltWidth=1, rotation=0, speed=2.0,
                    directionAngle=0, color="#00f"),
               Belt(id="belt_2", type="linear", x=4, y=0, length=4, beltWidth=1, rotation=0, speed=0.0,
                    directionAngle=0, color="#00f")],
        sensors=[Sensor(id="sensor_1", x=1, y=0, width=0.1, height=1, rotation=0, label="IN", color="#f00"),
                 Sensor(id="sensor_2", x=5.5, y=0, width=0.1, height=1, rotation=0, label="OUT", color="#f00")],
        sinks=[Sink(id="sink_1", x=6.5, y=0, width=1, height=1, label="SNK")],
        buttons=[ControlButton(id=f"button_{i}", x=i, y=-2, label=label) for i, label in enumerate(buttons, 1)],
        leds=[LedIndicator(id=f"led_{i}", x=i, y=-3, label=label) for i, label in enumerate(leds, 1)],
    )


class Plc:
    """Drive the controller like the exchange does: set inputs, scan."""

    def __init__(self, config, **kw):
        self.config = config
        self.c = DeliveryController(config, **kw)
        self.t = 0.0
        self.events = []

    def scan(self, dt=0.05):
        self.t += dt
        self.events += self.c.step(self.t)

    def press(self, button_id):
        b = next(b for b in self.config.buttons if b.id == button_id)
        b.sensor_states = True
        self.scan()
        b.sensor_states = False
        self.scan()

    def sensor(self, sensor_id, on):
        next(s for s in self.config.sensors if s.id == sensor_id).sensor_states = "1" if on else "0"
        self.scan()

    def speeds(self):
        return [b.speed for b in self.config.belts]

    def led(self, led_id):
        return next(l for l in self.config.leds if l.id == led_id).isOn


class DeliveryControllerTest(unittest.TestCase):
    def test_roles_from_labels(self):
        plc = Plc(layout(buttons=("STOP", "START", "RESET"), leds=("JAM", "RUN")))
        roles = plc.c.roles()
        self.assertEqual((roles["start"], roles["stop"], roles["reset"]), ("button_2", "button_1", "button_3"))
        self.assertEqual((roles["run_led"], roles["jam_led"]), ("led_2", "led_1"))
        self.assertEqual(roles["delivery_sensor"], "sensor_2")  # the one nearest the sink

    def test_start_stop_drive_belts_at_design_speed_and_leds(self):
        plc = Plc(layout())
        plc.scan()
        self.assertEqual(plc.speeds(), [0.0, 0.0], "waits for Start")
        plc.press("button_1")
        self.assertEqual(plc.speeds(), [2.0, 1.0], "layout speed; 1 m/s where the layout says 0")
        self.assertTrue(plc.led("led_1"))
        self.assertFalse(plc.led("led_2"))
        plc.press("button_2")
        self.assertEqual(plc.speeds(), [0.0, 0.0])
        self.assertFalse(plc.led("led_1"))
        self.assertIn("START (button_1)", plc.events)
        self.assertIn("STOP (button_2)", plc.events)

    def test_holding_a_button_is_one_press(self):
        plc = Plc(layout(buttons=("GO",)))           # one unlabelled button toggles
        plc.config.buttons[0].sensor_states = True
        for _ in range(5):
            plc.scan()
        self.assertTrue(plc.c.running)
        plc.config.buttons[0].sensor_states = False
        plc.scan()
        plc.press("button_1")
        self.assertFalse(plc.c.running)

    def test_parcels_are_counted_and_deliveries_tracked(self):
        plc = Plc(layout(), autostart=True)
        for _ in range(3):
            plc.sensor("sensor_1", True)
            plc.sensor("sensor_1", False)
        plc.sensor("sensor_2", True)
        plc.sensor("sensor_2", False)
        self.assertEqual(plc.c.sensors["sensor_1"].count, 3)
        self.assertEqual(plc.c.delivered, 1)
        self.assertIn("parcel at sensor_2 (#1) - delivered 1", plc.events)
        self.assertIn("RUN | parcels: sensor_1 3, sensor_2 1 | delivered 1", plc.c.summary())

    def test_jam_stops_belts_until_cleared_and_started(self):
        plc = Plc(layout(), jam_time=1.0, autostart=True)
        plc.sensor("sensor_1", True)
        for _ in range(25):
            plc.scan()                                  # 1.25 s blocked
        self.assertEqual(plc.c.jammed, "sensor_1")
        self.assertEqual(plc.speeds(), [0.0, 0.0])
        self.assertTrue(plc.led("led_2"))
        plc.press("button_1")                           # still blocked: stays jammed
        self.assertEqual(plc.c.jammed, "sensor_1")
        plc.sensor("sensor_1", False)
        plc.press("button_1")
        self.assertIsNone(plc.c.jammed)
        self.assertEqual(plc.speeds(), [2.0, 1.0])
        self.assertFalse(plc.led("led_2"))

    def test_is_on(self):
        self.assertTrue(is_on("1") and is_on(True) and is_on(3))
        self.assertFalse(is_on("0") or is_on(None) or is_on(False) or is_on(""))


class IoTopicsTest(unittest.TestCase):
    def test_test_config_keeps_its_wire_layout(self):
        publish, subscribe = io_topics(load_config("station_1.json"))
        self.assertIs(publish, PUBLISH_TOPICS)
        self.assertIs(subscribe, SUBSCRIBE_TOPICS)

    def test_a_designed_layout_gets_channels_from_its_own_components(self):
        config = layout()
        publish, subscribe = io_topics(config)
        self.assertEqual(publish["input"]["custom_ids"], ["button_1", "button_2", "sensor_1", "sensor_2", "clock"])
        self.assertEqual(subscribe["output"]["custom_ids"], ["led_1", "led_2", "belt_1", "belt_2"])
        self.assertEqual(publish["input"]["interval"], PUBLISH_TOPICS["input"]["interval"])
        self.assertNotIn("button_1", PUBLISH_TOPICS["input"]["custom_ids"], "defaults untouched")

    def test_simulator_and_plc_images_round_trip(self):
        sim, plc = layout(), layout()
        _, subscribe = io_topics(sim)
        mapping = custom_component_mapping(plc, subscribe["output"]["custom_ids"], direction="in")
        plc.belts[1].speed = 1.75
        plc.leds[1].isOn = True
        ComponentMessage(mapping).deserialize(ComponentMessage(mapping).serialize(plc), sim)
        self.assertEqual(sim.belts[1].speed, 1.75)
        self.assertTrue(sim.leds[1].isOn)


if __name__ == "__main__":
    unittest.main()


class LaserBannerTest(unittest.TestCase):
    """A laser banner shines its lasers straight down, one every pitch across
    the belt: a beam is blocked while a parcel is under its point. The PLC
    measures each parcel's shape with it (plc_logic.BannerProfile): width
    from the blocked beams, length from the belt's travel while blocked."""

    def plant(self):
        from physics import PhysicsEngine
        cfg = SimulatorConfig(
            useSurfaceVelocity=True,
            belts=[Belt(id="belt_1", type="linear", x=5.0, y=0.0, length=10.0, beltWidth=1.0, rotation=0.0,
                        speed=0.0, directionAngle=0.0, color="#888")],
            sensors=[Sensor(id="banner", x=4.0, y=0.0, width=0.1, height=1.0, rotation=0.0, label="BANNER",
                            color="#f00", sensorType="laser_banner", laserCount=41, beamDistance=0.02),
                     Sensor(id="eye", x=9.0, y=0.0, width=0.1, height=1.0, rotation=0.0, label="END", color="#f00")],
            buttons=[ControlButton(id="start", x=0, y=2, width=0.3, height=0.3, rotation=0, label="START")],
            sinks=[Sink(id="out", x=10.5, y=0.0, width=1.0, height=1.0, label="out")])
        return cfg, PhysicsEngine(cfg)

    def test_beams_are_points_under_the_head(self):
        cfg, engine = self.plant()
        engine.add_parcel(4.0, 0.15, 0.5, 0.3)              # 0.5 along, 0.3 across, centred 0.15 to the left
        engine.run_for(0.02)
        bits = cfg.sensors[0].sensor_states
        blocked = [i for i, c in enumerate(bits) if c == "1"]
        # beam i at v = (i - 20) * 0.02: the parcel covers v in [0.0, 0.3]
        self.assertEqual((blocked[0], blocked[-1]), (20, 35))
        cfg2, engine2 = self.plant()
        engine2.add_parcel(3.96, 0.0, 0.07, 0.3)            # its front edge 0.005 m short of the beam line
        engine2.run_for(0.02)
        self.assertNotIn("1", cfg2.sensors[0].sensor_states)   # a point beam: not blocked before it is under

    def test_the_plc_measures_each_parcels_shape(self):
        from plc_logic import make_controller
        cfg, engine = self.plant()
        plc = make_controller(cfg, autostart=True)
        engine.add_parcel(2.0, 0.0, 0.5, 0.3)
        sizes = []
        while engine.sim_time < 5.0:
            sizes += [e for e in plc.step(now=engine.sim_time) if "long x" in e]
            engine.run_for(0.01)
        self.assertEqual(len(sizes), 1)
        length, width = plc.banners["banner"].measured
        self.assertAlmostEqual(length, 0.5, delta=0.03)      # one scan of travel either way
        self.assertAlmostEqual(width, 0.3, delta=0.02 + 1e-9)  # the pitch
        self.assertIn("banner: parcel 0.5", sizes[0])

    def test_the_beams_as_the_plc_receives_them(self):
        """Off the wire a banner's beams are one integer, bit i = beam i."""
        from mqtt_message_binding import ComponentMessage
        from plc_logic import beam_bits
        cfg, _engine = self.plant()
        received, _ = self.plant()
        banner = cfg.sensors[0]
        banner.sensor_states = "0" * 10 + "1" * 15 + "0" * 16
        mapping = [(t, f"banner/sensor_states[{i}]") for i, t in enumerate([__import__("io_layout").ByteType.BIT] * 41)]
        message = ComponentMessage(mapping, endianness="<")
        message.deserialize(message.serialize(cfg), received)
        self.assertIsInstance(received.sensors[0].sensor_states, int)
        self.assertEqual(beam_bits(received.sensors[0]), beam_bits(banner))
        self.assertEqual([i for i, b in enumerate(beam_bits(banner)) if b], list(range(10, 25)))
