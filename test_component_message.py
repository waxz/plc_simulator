import unittest

from config_parser import Sensor, load_config
from mqtt_message_binding import (
    ByteType,
    ComponentMessage,
    compact_component_mapping,
    custom_component_mapping,
    sensor_state_mapping,
    verify_component_list,
)
from mqtt_data_exchange import PUBLISH_TOPICS, SUBSCRIBE_TOPICS, _struct_endianness
from struct_utils import (
    print_byte_by_byte,
)


class TestComponentMessageWithRealConfig(unittest.TestCase):
    def test_real_config_bitmap_and_belt_values_round_trip(self):
        config = load_config("test_config.json")
        mapping = compact_component_mapping(config)
        verify_component_list(mapping, config)

        original_sensor_1 = config.sensors[0].sensor_states
        original_sensor_2 = config.sensors[1].sensor_states
        original_sensor_3 = config.sensors[2].sensor_states
        original_belt_1_speed = next(b for b in config.belts if b.id == "belt_1").speed
        original_belt_2_angle = next(b for b in config.belts if b.id == "belt_2").directionAngle

        message = ComponentMessage(mapping)
        self.assertEqual(message.format_string, ">BBBBBBBffff")
        self.assertEqual(mapping[0], (ByteType.BIT, "sensor_2/sensor_states[0]"))
        self.assertEqual(mapping[8], (ByteType.BIT, "sensor_1/sensor_states[0]"))
        self.assertEqual(mapping[40], (ByteType.BIT, "sensor_3/sensor_states[0]"))
        self.assertEqual(mapping[-4:], [
            (ByteType.FLOAT, "belt_1/speed"),
            (ByteType.FLOAT, "belt_1/directionAngle"),
            (ByteType.FLOAT, "belt_2/speed"),
            (ByteType.FLOAT, "belt_2/directionAngle"),
        ])
        payload = message.serialize(config)

        config.sensors[0].sensor_states = "0" * len(config.sensors[0].sensor_states)
        config.sensors[1].sensor_states = "1"
        config.sensors[2].sensor_states = "1" * len(config.sensors[2].sensor_states)
        next(b for b in config.belts if b.id == "belt_1").speed = 0.0
        next(b for b in config.belts if b.id == "belt_2").directionAngle = 45.0

        message.deserialize(payload, config)

        self.assertEqual(config.sensors[0].sensor_states, original_sensor_1)
        self.assertEqual(config.sensors[1].sensor_states, original_sensor_2)
        self.assertEqual(config.sensors[2].sensor_states, original_sensor_3)
        self.assertAlmostEqual(next(b for b in config.belts if b.id == "belt_1").speed, original_belt_1_speed, places=5)
        self.assertAlmostEqual(next(b for b in config.belts if b.id == "belt_2").directionAngle, original_belt_2_angle, places=5)

    def test_sensor_state_mapping_uses_one_bit_for_ir_and_laser_count_for_banner(self):
        ir = Sensor(
            id="sensor_ir",
            x=0.0,
            y=0.0,
            width=0.1,
            height=1.0,
            rotation=0.0,
            label="IR",
            color="#ff0000",
            sensorType="ir",
            sensor_states=True,
        )
        banner = Sensor(
            id="sensor_banner",
            x=0.0,
            y=0.0,
            width=0.1,
            height=1.0,
            rotation=0.0,
            label="LB",
            color="#ff0000",
            sensorType="laser_banner",
            laserCount=16,
            sensor_states="0" * 16,
        )

        class Config:
            sensors = [ir, banner]

        mapping = sensor_state_mapping(Config())

        self.assertEqual(mapping[:1], [(ByteType.BIT, "sensor_ir/sensor_states[0]")])
        self.assertEqual(mapping[1:8], [(ByteType.BIT, f"padding_{index}/bit") for index in range(7)])
        self.assertEqual(len(mapping), 24)
        self.assertEqual(mapping[8], (ByteType.BIT, "sensor_banner/sensor_states[0]"))
        self.assertEqual(mapping[-1], (ByteType.BIT, "sensor_banner/sensor_states[15]"))

    def test_sensor_mapping_sorts_ir_before_laser_banner_for_compact_layout(self):
        banner = Sensor(
            id="sensor_banner",
            x=0.0,
            y=0.0,
            width=0.1,
            height=1.0,
            rotation=0.0,
            label="LB",
            color="#ff0000",
            sensorType="laser_banner",
            laserCount=10,
            sensor_states="1" * 10,
        )
        ir = Sensor(
            id="sensor_ir",
            x=0.0,
            y=0.0,
            width=0.1,
            height=1.0,
            rotation=0.0,
            label="IR",
            color="#ff0000",
            sensorType="ir",
            sensor_states=True,
        )

        class Config:
            sensors = [banner, ir]

        mapping = sensor_state_mapping(Config())

        self.assertEqual(mapping[0], (ByteType.BIT, "sensor_ir/sensor_states[0]"))
        self.assertEqual(mapping[1:8], [(ByteType.BIT, f"padding_{index}/bit") for index in range(7)])
        self.assertEqual(mapping[8:18], [(ByteType.BIT, f"sensor_banner/sensor_states[{index}]") for index in range(10)])
        self.assertEqual(mapping[18:24], [(ByteType.BIT, f"padding_{index}/bit") for index in range(7, 13)])

    def test_custom_bindings_cycle(self):
        # Mimic an MQTT message workflow:
        # Load sender config (representing realtime simulator state)
        config_sender = load_config("test_config.json")
        # Load receiver config (representing receiver/PLC state, which will be updated)
        config_receiver = load_config("test_config.json")

        custom_ids = ["sensor_2", "button_0", "led_1", "sensor_1", "belt_1", "belt_2"]
        mapping = custom_component_mapping(config_sender, custom_ids)
        print(f"mapping:")
        print(mapping)
        verify_component_list(mapping, config_sender)

        # 1. Setup the "realtime simulator state" on config_sender
        sensor_1_s = next(s for s in config_sender.sensors if s.id == "sensor_1")
        sensor_2_s = next(s for s in config_sender.sensors if s.id == "sensor_2")
        belt_1_s = next(b for b in config_sender.belts if b.id == "belt_1")
        belt_2_s = next(b for b in config_sender.belts if b.id == "belt_2")
        button_0_s = next(b for b in config_sender.buttons if b.id == "button_0")
        led_1_s = next(l for l in config_sender.leds if l.id == "led_1")

        # Set specific active/realtime values
        sensor_1_s.sensor_states = "1" * len(sensor_1_s.sensor_states)
        sensor_2_s.sensor_states = "1"
        button_0_s.sensor_states = True
        led_1_s.isOn = True
        belt_1_s.speed = 4.25
        belt_2_s.directionAngle = -45.0

        # 2. Setup the "receiver state" (default or reset values) on config_receiver
        sensor_1_r = next(s for s in config_receiver.sensors if s.id == "sensor_1")
        sensor_2_r = next(s for s in config_receiver.sensors if s.id == "sensor_2")
        belt_1_r = next(b for b in config_receiver.belts if b.id == "belt_1")
        belt_2_r = next(b for b in config_receiver.belts if b.id == "belt_2")
        button_0_r = next(b for b in config_receiver.buttons if b.id == "button_0")
        led_1_r = next(l for l in config_receiver.leds if l.id == "led_1")

        sensor_1_r.sensor_states = "0" * len(sensor_1_r.sensor_states)
        sensor_2_r.sensor_states = "0"
        button_0_r.sensor_states = False
        led_1_r.isOn = False
        belt_1_r.speed = 0.0
        belt_2_r.directionAngle = 0.0

        # Create message layout handler
        message = ComponentMessage(mapping)

        # 3. Create the payload from custom_ids and realtime config (sender)
        payload = message.serialize(config_sender)
        print(f"test_custom_bindings_cycle: payload:{payload}")
        print_byte_by_byte(payload)

        # 4. Decode and update config from payload (receiver)
        message.deserialize(payload, config_receiver)

        # Assert that the receiver's state now matches the sender's realtime state
        self.assertEqual(sensor_1_r.sensor_states, sensor_1_s.sensor_states)
        self.assertEqual(sensor_2_r.sensor_states, sensor_2_s.sensor_states)
        self.assertAlmostEqual(belt_1_r.speed, belt_1_s.speed, places=5)
        self.assertAlmostEqual(belt_2_r.directionAngle, belt_2_s.directionAngle, places=5)
        self.assertEqual(button_0_r.sensor_states, button_0_s.sensor_states)
        self.assertEqual(led_1_r.isOn, led_1_s.isOn)

    def test_plc_output_payload_uses_little_endian_belt_reals(self):
        config = load_config("test_config.json")
        topic_def = SUBSCRIBE_TOPICS["output"]
        mapping = custom_component_mapping(config, topic_def["custom_ids"])
        message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def))
        payload = bytes.fromhex(
            "0000 0000 803f 0000 0000 0000 803f 0000 0000 0000 803f "
            "0000 0000 0000 803f 0000 0000 0000 803f 0000 0000 0000 "
            "0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 "
            "0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 "
            "0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 "
            "0000 0000 0000 0000 0000 0000"
        )

        self.assertEqual(len(payload), message.payload_size)
        message.deserialize(payload, config)

        for belt_id in ("belt_1_1_1", "belt_1_1_2", "belt_1_1_3", "belt_1_1_4", "belt_1_1_5"):
            self.assertAlmostEqual(next(b for b in config.belts if b.id == belt_id).speed, 1.0)

    def test_simulator_input_payload_includes_uint64_nanosecond_clock(self):
        config = load_config("test_config.json")
        topic_def = PUBLISH_TOPICS["input"]
        mapping = custom_component_mapping(config, topic_def["custom_ids"])
        verify_component_list(mapping, config)

        self.assertEqual(mapping[-1], (ByteType.UINT64, "clock/nanoseconds"))
        message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def))
        first_payload = message.serialize(config)
        second_payload = message.serialize(config)
        first_clock = int.from_bytes(first_payload[-8:], byteorder="little", signed=False)
        second_clock = int.from_bytes(second_payload[-8:], byteorder="little", signed=False)

        self.assertGreater(first_clock, 0)
        self.assertGreaterEqual(second_clock, first_clock)


if __name__ == "__main__":
    unittest.main()
