import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
import pygame_gui

from config_parser import Belt, MqttSettings, Sensor, SimulatorConfig, TopicComponent, TopicConfig
from editor_ui import EditorUI


class MqttPanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pygame.init()
        pygame.display.set_mode((900, 700))

    @classmethod
    def tearDownClass(cls):
        pygame.quit()

    def setUp(self):
        self.manager = pygame_gui.UIManager((900, 700))
        self.config = SimulatorConfig(
            belts=[
                Belt(
                    id="belt_1",
                    type="linear",
                    x=0.0,
                    y=0.0,
                    length=2.0,
                    beltWidth=0.5,
                    rotation=0.0,
                    speed=1.25,
                    directionAngle=90.0,
                    direction=1,
                    color="#cccccc",
                    speedBinding="beltSpeed",
                    directionAngleBinding="beltAngle",
                    directionBinding="beltDir",
                )
            ],
            sensors=[
                Sensor(
                    id="sensor_1",
                    x=1.0,
                    y=1.0,
                    width=0.1,
                    height=1.0,
                    rotation=0.0,
                    label="S1",
                    color="#ff0000",
                    stateBinding="sensorState",
                )
            ],
            mqttSettings=MqttSettings(
                enabled=True,
                brokerUrl="mqtt://localhost:1883",
                topicPrefix="sim",
                topics=[
                    TopicConfig(
                        id="topic_1",
                        topic="status",
                        interval=250,
                        direction="out",
                        format="json",
                        endianness="LE",
                        components=[
                            TopicComponent(id="sensor_1", type="sensor"),
                            TopicComponent(id="belt_1", type="belt"),
                        ],
                    )
                ],
            ),
        )
        self.ui = EditorUI(self.manager, 900, 700, self.config)
        self.ui.open_mqtt_panel()
        self.ui.mqtt_selected_topic_id = "topic_1"
        self.ui.last_sensor_states = {"sensor_1": True}

    def test_configuration_accepts_supported_formats_and_prefixes_topic(self):
        for fmt in ["json", "raw", "string", "binary_struct"]:
            topic = TopicConfig(
                id=f"topic_{fmt}",
                topic="debug",
                interval=1000,
                direction="out",
                format=fmt,
                endianness="LE",
            )
            self.assertEqual(topic.format, fmt)

        self.assertEqual(self.ui._full_topic(self.config.mqttSettings.topics[0]), "sim/status")

    def test_json_encode_decode_uses_sensor_and_belt_status(self):
        topic = self.config.mqttSettings.topics[0]
        topic.format = "json"

        payload = self.ui._encode_topic_payload(topic)
        decoded = self.ui._decode_topic_payload(payload, topic)

        self.assertTrue(decoded["sensor_1"]["sensorState"])
        self.assertEqual(decoded["belt_1"]["beltSpeed"], 1.25)
        self.assertEqual(decoded["belt_1"]["beltAngle"], 90.0)
        self.assertEqual(decoded["belt_1"]["beltDir"], 1)

    def test_string_encode_decode_round_trip(self):
        topic = self.config.mqttSettings.topics[0]
        topic.format = "string"

        payload = self.ui._encode_topic_payload(topic)
        text = payload.decode("utf-8")
        decoded = self.ui._decode_topic_payload(payload, topic)

        self.assertIn("sensor_1.sensorState=True", text)
        self.assertEqual(decoded["sensor_1"]["sensorState"], "True")
        self.assertEqual(decoded["belt_1"]["beltSpeed"], "1.25")

    def test_binary_encode_decode_round_trip(self):
        topic = self.config.mqttSettings.topics[0]
        topic.format = "binary_struct"
        topic.endianness = "LE"

        payload = self.ui._encode_topic_payload(topic)
        decoded = self.ui._decode_topic_payload(payload, topic)

        self.assertGreater(len(payload), 0)
        self.assertTrue(decoded["sensor_1"]["sensorState"])
        self.assertAlmostEqual(decoded["belt_1"]["beltSpeed"], 1.25, places=5)
        self.assertAlmostEqual(decoded["belt_1"]["beltAngle"], 90.0, places=5)
        self.assertEqual(decoded["belt_1"]["beltDir"], 1)

    def test_decoded_values_apply_back_to_component_fields(self):
        decoded = {
            "sensor_1": {"sensorState": "false"},
            "belt_1": {"beltSpeed": "2.5", "beltAngle": "45", "beltDir": "0"},
        }

        self.ui._apply_decoded_values(decoded)

        self.assertFalse(self.config.sensors[0].isActive)
        self.assertEqual(self.config.belts[0].speed, 2.5)
        self.assertEqual(self.config.belts[0].directionAngle, 45.0)
        self.assertEqual(self.config.belts[0].direction, 0)

    def test_bandwidth_estimate_uses_payload_size_topic_and_interval(self):
        topic = self.config.mqttSettings.topics[0]
        topic.format = "json"
        topic.interval = 250

        estimate = self.ui._estimate_topic_bandwidth(topic)

        self.assertEqual(estimate["payload_bytes"], len(self.ui._encode_topic_payload(topic)))
        self.assertEqual(estimate["messages_per_second"], 4.0)
        self.assertEqual(
            estimate["estimated_packet_bytes"],
            4 + len("sim/status".encode("utf-8")) + estimate["payload_bytes"],
        )
        self.assertEqual(estimate["bytes_per_second"], estimate["estimated_packet_bytes"] * 4.0)


if __name__ == "__main__":
    unittest.main()
