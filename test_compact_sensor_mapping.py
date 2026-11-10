"""test_compact_sensor_mapping.py - a sensor/button/LED's state can now be
mapped as a compact BYTE/UINT16/UINT32/FLOAT field (one register, PLC-style)
instead of always being spread across individual BIT rows. Covers
ComponentMessage's serialize/deserialize round trip through _compact_int/
_int_to_matching_shape, and verify_component_list's guard against a
too-narrow compact type silently dropping bits.
"""
import unittest

from config_parser import load_config
from mqtt_message_binding import ByteType, ComponentMessage, verify_component_list


def _sensor(config, sensor_id):
    return next(s for s in config.sensors if s.id == sensor_id)


class CompactBitFieldRoundTripTest(unittest.TestCase):
    def test_ir_sensor_bool_like_state_round_trips_through_a_compact_byte(self):
        config = load_config("station_1.json")
        ir_sensor = next(s for s in config.sensors if s.sensorType == "ir")
        mapping = [(ByteType.BYTE, f"{ir_sensor.id}/sensor_states")]
        verify_component_list(mapping, config)
        message = ComponentMessage(mapping)

        ir_sensor.sensor_states = "1"
        payload = message.serialize(config)
        self.assertEqual(payload, b"\x01")

        ir_sensor.sensor_states = "0" * len(ir_sensor.sensor_states)
        message.deserialize(payload, config)
        self.assertEqual(ir_sensor.sensor_states, "1")

    def test_laser_banner_round_trips_through_a_compact_uint32(self):
        config = load_config("station_1.json")
        banner = next(s for s in config.sensors if s.sensorType == "laser_banner" and s.laserCount)
        mapping = [(ByteType.UINT32, f"{banner.id}/sensor_states")]
        verify_component_list(mapping, config)
        message = ComponentMessage(mapping)

        original_width = len(banner.sensor_states)
        pattern = ("10" * ((original_width + 1) // 2))[:original_width]
        banner.sensor_states = pattern
        payload = message.serialize(config)

        banner.sensor_states = "0" * original_width
        message.deserialize(payload, config)
        self.assertEqual(banner.sensor_states, pattern)
        self.assertEqual(len(banner.sensor_states), original_width)

    def test_led_ison_bool_round_trips_through_a_compact_byte(self):
        config = load_config("station_1.json")
        led = config.leds[0]
        mapping = [(ByteType.BYTE, f"{led.id}/isOn")]
        message = ComponentMessage(mapping)

        led.isOn = True
        payload = message.serialize(config)
        led.isOn = False
        message.deserialize(payload, config)
        self.assertIs(led.isOn, True)

    def test_a_compact_field_alongside_ordinary_bit_and_float_fields(self):
        """Mixing a compact field into a normal mapping doesn't disturb the
        surrounding BIT-packing/float handling."""
        config = load_config("station_1.json")
        ir_sensor = next(s for s in config.sensors if s.sensorType == "ir")
        button = config.buttons[0]
        belt = config.belts[0]
        mapping = [
            (ByteType.BIT, f"{button.id}/sensor_states[0]"),
            (ByteType.BYTE, f"{ir_sensor.id}/sensor_states"),
            (ByteType.FLOAT, f"{belt.id}/speed"),
        ]
        verify_component_list(mapping, config)
        message = ComponentMessage(mapping)

        button.sensor_states = True
        ir_sensor.sensor_states = "1"
        belt.speed = 4.25
        payload = message.serialize(config)

        button.sensor_states = False
        ir_sensor.sensor_states = "0"
        belt.speed = 0.0
        message.deserialize(payload, config)

        self.assertEqual(button.sensor_states, True)
        self.assertEqual(ir_sensor.sensor_states, "1")
        self.assertAlmostEqual(belt.speed, 4.25, places=4)

    def test_top_bits_beyond_a_too_narrow_type_are_dropped_not_corrupting(self):
        """Deliberately bypassing verify_component_list's width guard (as
        if it had been skipped) to document the actual, honest behavior of
        an intentionally-too-small compact type: truncation, not garbage."""
        config = load_config("station_1.json")
        banner = next(s for s in config.sensors if s.sensorType == "laser_banner" and s.laserCount)
        mapping = [(ByteType.BYTE, f"{banner.id}/sensor_states")]  # 8 bits for a 32-bit banner
        message = ComponentMessage(mapping)

        original_width = len(banner.sensor_states)
        banner.sensor_states = "1" * original_width
        payload = message.serialize(config)

        banner.sensor_states = "0" * original_width
        message.deserialize(payload, config)
        # Only the low 8 bits - beams 0..7, the first 8 characters - survive
        # the round trip through one byte; the string is rebuilt at its
        # original width regardless.
        self.assertEqual(len(banner.sensor_states), original_width)
        self.assertEqual(banner.sensor_states[:8], "11111111")
        self.assertEqual(banner.sensor_states[8:], "0" * (original_width - 8))


class CompactWidthValidationTest(unittest.TestCase):
    def test_rejects_a_laser_banner_compacted_into_a_too_narrow_byte(self):
        config = load_config("station_1.json")
        banner = next(s for s in config.sensors if s.sensorType == "laser_banner" and s.laserCount)
        mapping = [(ByteType.BYTE, f"{banner.id}/sensor_states")]
        with self.assertRaises(ValueError) as ctx:
            verify_component_list(mapping, config)
        self.assertIn("ByteType.BYTE", str(ctx.exception))
        self.assertIn(banner.id, str(ctx.exception))

    def test_accepts_a_laser_banner_compacted_into_a_wide_enough_uint32(self):
        config = load_config("station_1.json")
        banner = next(s for s in config.sensors if s.sensorType == "laser_banner" and s.laserCount)
        mapping = [(ByteType.UINT32, f"{banner.id}/sensor_states")]
        verify_component_list(mapping, config)  # must not raise

    def test_ir_sensor_fits_in_any_compact_type(self):
        config = load_config("station_1.json")
        ir_sensor = next(s for s in config.sensors if s.sensorType == "ir")
        for byte_type in (ByteType.BYTE, ByteType.UINT16, ByteType.UINT32, ByteType.UINT64):
            verify_component_list([(byte_type, f"{ir_sensor.id}/sensor_states")], config)


if __name__ == "__main__":
    unittest.main()
