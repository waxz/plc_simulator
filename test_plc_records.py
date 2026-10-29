"""test_plc_records.py - the record format (field list once, values only) and its JSON."""
import struct
import unittest

import plc_records as pr

FIELDS = [("link_ok", "BOOL"), ("input_frames", "UDINT"), ("offset", "DINT"), ("clock", "LINT"),
          ("belt_speed", "REAL"), ("precise", "LREAL"), ("stations.0.name", "STRING"),
          ("stations.0.running", "BOOL"), ("stations.1.name", "STRING"), ("stations.1.running", "BOOL")]
VALUES = [True, 1251, -7, 1793282500405782400, 0.5, 2.25, "L1", True, "L2", False]


class RecordTest(unittest.TestCase):
    def test_round_trip_to_nested_json(self):
        schema = pr.parse_schema(pr.pack_schema(2, "stations", FIELDS))
        self.assertEqual((schema.slot, schema.topic, len(schema.fields)), (2, "stations", 10))
        msg = pr.parse_values(pr.pack_values(2, FIELDS, VALUES), schema)
        self.assertEqual(msg, {"link_ok": True, "input_frames": 1251, "offset": -7, "clock": 1793282500405782400,
                               "belt_speed": 0.5, "precise": 2.25,
                               "stations": [{"name": "L1", "running": True}, {"name": "L2", "running": False}]})

    def test_the_wire_bytes(self):
        # What RecordWriter (record_writer.st) writes, byte for byte.
        self.assertEqual(pr.pack_schema(0, "t", [("ok", "BOOL"), ("v", "REAL")]),
                         bytes([0, 2, 0, 1]) + b"t" + bytes([1, 2]) + b"ok" + bytes([5, 1]) + b"v")
        self.assertEqual(pr.pack_values(0, [("ok", "BOOL"), ("v", "REAL")], [True, 0.5]),
                         bytes([0, 2, 0, 5, 0, 1]) + struct.pack("<f", 0.5))

    def test_mismatches_are_errors(self):
        schema = pr.parse_schema(pr.pack_schema(0, "stations", FIELDS))
        values = pr.pack_values(0, FIELDS, VALUES)
        with self.assertRaises(pr.RecordError):                      # one value missing
            pr.parse_values(pr.pack_values(0, FIELDS[:-1], VALUES[:-1]), schema)
        with self.assertRaises(pr.RecordError):                      # size field disagrees
            pr.parse_values(values[:3] + struct.pack("<H", 3) + values[5:], schema)
        with self.assertRaises(pr.RecordError):                      # truncated
            pr.parse_values(values[:-1], schema)
        with self.assertRaises(pr.RecordError):                      # unknown type
            pr.parse_schema(bytes([0, 1, 0, 1]) + b"t" + bytes([9, 1]) + b"x")
        with self.assertRaises(pr.RecordError):                      # a value and a group
            pr.nest([("a", 1), ("a.b", 2)])


if __name__ == "__main__":
    unittest.main()
