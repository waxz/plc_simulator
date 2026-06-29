import unittest

from config_parser import Belt, MqttSettings, Sensor, SimulatorConfig, TopicComponent, TopicConfig
from mqtt_message_codec import (
    compose_fields,
    decode_message,
    encode_message,
    estimate_bandwidth,
    full_topic,
)


def make_config(format_name="json"):
    return SimulatorConfig(
        belts=[
            Belt(
                id="belt_1",
                type="linear",
                x=0.0,
                y=0.0,
                length=2.0,
                beltWidth=0.5,
                rotation=0.0,
                speed=1.5,
                directionAngle=30.0,
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
                x=0.0,
                y=0.0,
                width=0.1,
                height=1.0,
                rotation=0.0,
                sensorType = "ir",
                label="S1",
                color="#ff0000",
                stateBinding="sensorState",
            ),
              Sensor(
                id="sensor_2",
                x=0.0,
                y=0.0,
                width=0.1,
                height=1.0,
                rotation=0.0,
                sensorType = "laser_banner",
                label="S2",
                color="#ff0000",
                stateBinding="sensorState",
            )
        ],
        mqttSettings=MqttSettings(
            topicPrefix="sim",
            topics=[
                TopicConfig(
                    id="status",
                    topic="status",
                    interval=500,
                    direction="out",
                    format=format_name,
                    endianness="LE",
                    components=[
                        TopicComponent(id="sensor_1", type="sensor"),
                        TopicComponent(id="belt_1", type="belt"),
                    ],
                )
            ],
        ),
    )


class MqttMessageCodecTest(unittest.TestCase):
    def test_compose_fields_uses_bindings_and_live_sensor_state(self):
        config = make_config()
        topic = config.mqttSettings.topics[0]

        fields = compose_fields(config, topic, {"sensor_1": True, "sensor_2": False})
        rows = {(field.component_id, field.field): field for field in fields}

        self.assertEqual(full_topic(config, topic), "sim/status")
        self.assertTrue(rows[("sensor_1", "state")].value)
        self.assertEqual(rows[("sensor_1", "state")].binding, "sensorState")
        self.assertEqual(rows[("belt_1", "speed")].binding, "beltSpeed")

    def test_json_encode_decode(self):
        config = make_config("json")
        topic = config.mqttSettings.topics[0]

        payload = encode_message(config, topic, {"sensor_1": True})
        decoded = decode_message(config, topic, payload)

        self.assertTrue(decoded["sensor_1"]["sensorState"])
        self.assertEqual(decoded["belt_1"]["beltSpeed"], 1.5)

    def test_string_encode_decode(self):
        config = make_config("string")
        topic = config.mqttSettings.topics[0]

        payload = encode_message(config, topic, {"sensor_1": True})
        decoded = decode_message(config, topic, payload)

        self.assertIn(b"sensor_1.sensorState=True", payload)
        self.assertEqual(decoded["belt_1"]["beltSpeed"], "1.5")

    def test_binary_struct_encode_decode_uses_bitmap_then_floats(self):
        config = make_config("binary_struct")
        topic = config.mqttSettings.topics[0]

        payload = encode_message(config, topic, {"sensor_1": True})
        decoded = decode_message(config, topic, payload)

        self.assertEqual(payload[0], 0b10000000)
        self.assertTrue(decoded["sensor_1"]["sensorState"])
        self.assertAlmostEqual(decoded["belt_1"]["beltSpeed"], 1.5, places=5)
        self.assertAlmostEqual(decoded["belt_1"]["beltAngle"], 30.0, places=5)
        self.assertAlmostEqual(decoded["belt_1"]["beltDir"], 1.0, places=5)

    def test_bandwidth_estimate(self):
        config = make_config("json")
        topic = config.mqttSettings.topics[0]

        estimate = estimate_bandwidth(config, topic, {"sensor_1": False})

        self.assertEqual(estimate["messages_per_second"], 2.0)
        self.assertEqual(estimate["payload_bytes"], len(encode_message(config, topic, {"sensor_1": False})))
        self.assertEqual(
            estimate["estimated_packet_bytes"],
            4 + len("sim/status") + estimate["payload_bytes"],
        )


if __name__ == "__main__":
    unittest.main()
