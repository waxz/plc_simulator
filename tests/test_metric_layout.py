"""test_metric_layout.py - the metric-layout-dataclass <-> wire-mapping
bridge (metric_layout.py). layout_mapping() must produce exactly the
(ByteType, "group/name") list MetricsMessage already consumes, and
layout_values() must flatten an instance to the {(group, name): value}
dict serialize() takes.
"""
import unittest
from dataclasses import dataclass

from metric_layout import Vec2, Vec3, bit, f32, layout_mapping, layout_values, u16, u32
from mqtt_message_binding import ByteType


class LayoutMappingTest(unittest.TestCase):
    def test_scalars_and_groups(self):
        @dataclass
        class Perf:
            scan_ms: f32
            count: u32

        @dataclass
        class Results:
            code: u16

        @dataclass
        class Layout:
            perf: Perf
            results: Results

        self.assertEqual(
            layout_mapping(Layout),
            [
                (ByteType.FLOAT, "perf/scan_ms"),
                (ByteType.UINT32, "perf/count"),
                (ByteType.UINT16, "results/code"),
            ],
        )

    def test_vector_field_expands_to_indexed_paths(self):
        @dataclass
        class G:
            velocity: Vec2
            accel: Vec3

        @dataclass
        class Layout:
            g: G

        self.assertEqual(
            layout_mapping(Layout),
            [
                (ByteType.FLOAT, "g/velocity[0]"),
                (ByteType.FLOAT, "g/velocity[1]"),
                (ByteType.FLOAT, "g/accel[0]"),
                (ByteType.FLOAT, "g/accel[1]"),
                (ByteType.FLOAT, "g/accel[2]"),
            ],
        )

    def test_bit_field(self):
        @dataclass
        class N:
            led: bit

        @dataclass
        class Layout:
            n: N

        self.assertEqual(layout_mapping(Layout), [(ByteType.BIT, "n/led")])

    def test_matches_the_real_layout1_mapping(self):
        # Freezes the hand-migration: MetricLayout1 must derive to exactly
        # what the old METRICS_MAPPING list was.
        from mqtt_data_exchange import MetricLayout1

        self.assertEqual(
            layout_mapping(MetricLayout1),
            [
                (ByteType.FLOAT, "perf/scan_time_ms"),
                (ByteType.FLOAT, "perf/cycle_time_ms"),
                (ByteType.FLOAT, "perf/fn_elapsed_us"),
                (ByteType.UINT32, "perf/sample_count"),
                (ByteType.UINT16, "results/last_result_code"),
                (ByteType.FLOAT, "results/belt_1_velocity[0]"),
                (ByteType.FLOAT, "results/belt_1_velocity[1]"),
            ],
        )


class LayoutValuesTest(unittest.TestCase):
    def test_flattens_instance_to_group_name_dict(self):
        from mqtt_data_exchange import MetricLayout1, MetricLayout1Perf, MetricLayout1Results

        sample = MetricLayout1(
            perf=MetricLayout1Perf(scan_time_ms=1.0, cycle_time_ms=2.0, fn_elapsed_us=3.0, sample_count=4),
            results=MetricLayout1Results(last_result_code=0, belt_1_velocity=(5.0, 6.0)),
        )
        self.assertEqual(
            layout_values(sample),
            {
                ("perf", "scan_time_ms"): 1.0,
                ("perf", "cycle_time_ms"): 2.0,
                ("perf", "fn_elapsed_us"): 3.0,
                ("perf", "sample_count"): 4,
                ("results", "last_result_code"): 0,
                ("results", "belt_1_velocity"): (5.0, 6.0),
            },
        )

    def test_rejects_a_class_instead_of_an_instance(self):
        from mqtt_data_exchange import MetricLayout1

        with self.assertRaises(TypeError):
            layout_values(MetricLayout1)


if __name__ == "__main__":
    unittest.main()
