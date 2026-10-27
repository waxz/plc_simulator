"""
test_io_packing.py - the raw I/O frame, byte by byte, and its agreement
between the simulator and the PLC side (plc_mock.py):

  - every packed type at its exact offset and value, little and big endian:
    single bits, padding bits, a bit-packed laser banner, BYTE, WORD (uint16),
    DWORD (uint32), REAL (float), the 64-bit clock, belt REALs, LED bits
  - the layout rule matches the PLC serializer (generate_st_serde.py in the
    SIMATIC AX project): bits first, padded to a byte, then everything else
    in declaration order; st_struct() gives the matching STRUCT (checked
    against the project's Station_1_Input / Station_1_Output)
  - FrameType.LAYOUT_CHECK: plc_mock.py and the simulator compare layout
    fingerprints when it connects; a different layout is reported on both
    sides and its frames are counted and ignored, not applied
  - values round-trip exactly over a real TCP connection
"""
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

import io_layout
from config_parser import Belt, ControlButton, LedIndicator, Sensor, SimulatorConfig, load_config
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config, io_topics
from mqtt_message_binding import ByteType, ComponentMessage, custom_component_mapping, verify_component_list
from tcp_framed_transport import FrameType, TcpFramedClient

HERE = os.path.dirname(os.path.abspath(__file__))
CLOCK = 0x0102030405060708


def banner(id, count, state, pack="bit"):
    return Sensor(id=id, x=0, y=0, width=0.1, height=1, rotation=0, label=id, color="#f00",
                  sensorType="laser_banner", laserCount=count, sensor_states=state, packFormat=pack)


def every_type_config() -> SimulatorConfig:
    """One of each packed type, with known states."""
    return SimulatorConfig(
        buttons=[ControlButton(id="button_1", x=0, y=0, label="START", sensor_states=True),
                 ControlButton(id="button_2", x=0, y=0, label="STOP", sensor_states=False)],
        sensors=[Sensor(id="eye", x=0, y=0, width=0.1, height=1, rotation=0, label="IR", color="#f00",
                        sensor_states="1"),
                 banner("banner12", 12, "100000000011"),                  # beams 0, 10, 11
                 banner("byte8", 8, "10000001", "byte"),                   # 0x81
                 banner("word16", 16, "1000000001000000", "uint16"),       # beams 0, 9 -> 0x0201
                 banner("dword32", 32, "0" * 31 + "1", "uint32"),          # beam 31 -> 0x80000000
                 Sensor(id="eye_f", x=0, y=0, width=0.1, height=1, rotation=0, label="IRF", color="#f00",
                        sensor_states="1", packFormat="float")],          # 1.0
        leds=[LedIndicator(id="led_1", x=0, y=0, label="RUN", isOn=True),
              LedIndicator(id="led_2", x=0, y=0, label="JAM", isOn=False),
              LedIndicator(id="led_3", x=0, y=0, label="WARN", isOn=True)],
        belts=[Belt(id="belt_1", type="linear", x=0, y=0, length=2, beltWidth=1, rotation=0, speed=1.5,
                    directionAngle=-90.0, color="#00f")],
    )


INPUT_IDS = ["button_1", "eye", "banner12", "byte8", "button_2", "word16", "dword32", "eye_f", "clock"]
OUTPUT_IDS = ["led_1", "led_2", "led_3", "belt_1"]

# Simulator -> PLC, little endian, byte by byte.
INPUT_LE = bytes([
    0b011,                       # 0.0 button_1=1, 0.1 eye=1, 0.2 button_2=0, 0.3-7 padding
    0x01, 0x0C,                  # 1-2 banner12: beam 0 -> 1.0; beams 10, 11 -> 2.2, 2.3; 2.4-7 padding
    0x81,                        # 3 byte8 (BYTE)
    0x01, 0x02,                  # 4 word16 (WORD) 0x0201
    0x00, 0x00, 0x00, 0x80,      # 6 dword32 (DWORD) 0x80000000
    0x00, 0x00, 0x80, 0x3F,      # 10 eye_f (REAL) 1.0
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  # 14 clock (LINT)
])
INPUT_BE = INPUT_LE[:4] + bytes([0x02, 0x01, 0x80, 0, 0, 0, 0x3F, 0x80, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8])
# PLC -> simulator: LEDs, then belt_1 speed 1.5 and directionAngle -90.0.
OUTPUT_LE = bytes([0b101]) + struct.pack("<ff", 1.5, -90.0)
OUTPUT_BE = bytes([0b101]) + struct.pack(">ff", 1.5, -90.0)


class FrameBytesTest(unittest.TestCase):
    def mappings(self, config):
        return (custom_component_mapping(config, INPUT_IDS, direction="out"),
                custom_component_mapping(config, OUTPUT_IDS, direction="in"))

    def test_layout_rule_bits_first_then_declaration_order(self):
        inp, _ = self.mappings(every_type_config())
        verify_component_list(inp, every_type_config())
        table = [(f.address, f.byte_type, f.path) for f in io_layout.layout_fields(inp) if not f.is_padding]
        self.assertEqual(table[:3], [("0.0", ByteType.BIT, "button_1/sensor_states[0]"),
                                     ("0.1", ByteType.BIT, "eye/sensor_states[0]"),
                                     ("0.2", ByteType.BIT, "button_2/sensor_states[0]")])
        self.assertEqual(table[3][:2], ("1.0", ByteType.BIT))     # banner12 starts on a new byte
        self.assertEqual(table[14][:2], ("2.3", ByteType.BIT))    # its beam 11
        self.assertEqual(table[15:], [("3", ByteType.BYTE, "byte8/sensor_states"),
                                      ("4", ByteType.UINT16, "word16/sensor_states"),
                                      ("6", ByteType.UINT32, "dword32/sensor_states"),
                                      ("10", ByteType.FLOAT, "eye_f/sensor_states"),
                                      ("14", ByteType.UINT64, "clock/nanoseconds")])
        self.assertEqual(io_layout.layout_size(inp), 22)

    def test_exact_bytes_little_and_big_endian(self):
        config = every_type_config()
        inp, out = self.mappings(config)
        with mock.patch("mqtt_message_binding.time.time_ns", return_value=CLOCK):
            self.assertEqual(ComponentMessage(inp, "<").serialize(config), INPUT_LE)
            self.assertEqual(ComponentMessage(inp, ">").serialize(config), INPUT_BE)
        self.assertEqual(ComponentMessage(out, "<").serialize(config), OUTPUT_LE)
        self.assertEqual(ComponentMessage(out, ">").serialize(config), OUTPUT_BE)

    def test_every_value_decodes_back_exactly(self):
        sender = every_type_config()
        inp, out = self.mappings(sender)
        for endian, data in (("<", INPUT_LE), (">", INPUT_BE)):
            receiver = every_type_config()
            for s in receiver.sensors:
                s.sensor_states = "0" * len(s.sensor_states)
            for b in receiver.buttons:
                b.sensor_states = None
            ComponentMessage(inp, endian).deserialize(data, receiver)
            states = {s.id: s.sensor_states for s in receiver.sensors}
            self.assertEqual(states, {s.id: s.sensor_states for s in sender.sensors}, endian)
            self.assertTrue(receiver.buttons[0].sensor_states)
            self.assertFalse(receiver.buttons[1].sensor_states)
        receiver = every_type_config()
        receiver.leds[0].isOn = receiver.leds[2].isOn = False
        receiver.belts[0].speed = receiver.belts[0].directionAngle = 0.0
        ComponentMessage(out, "<").deserialize(OUTPUT_LE, receiver)
        self.assertEqual([l.isOn for l in receiver.leds], [True, False, True])
        self.assertEqual((receiver.belts[0].speed, receiver.belts[0].directionAngle), (1.5, -90.0))

    def test_banner_beam_i_is_bit_i_in_bit_and_register_formats(self):
        """A banner's DWORD on the PLC reads the same whichever packFormat."""
        state = "0" * 31 + "1"
        as_bits = custom_component_mapping(SimulatorConfig(sensors=[banner("b", 32, state)]), ["b"])
        as_dword = custom_component_mapping(SimulatorConfig(sensors=[banner("b", 32, state, "uint32")]), ["b"])
        bits_payload = ComponentMessage(as_bits, "<").serialize(SimulatorConfig(sensors=[banner("b", 32, state)]))
        dword_payload = ComponentMessage(as_dword, "<").serialize(
            SimulatorConfig(sensors=[banner("b", 32, state, "uint32")]))
        self.assertEqual(bits_payload, dword_payload)
        self.assertEqual(struct.unpack("<I", dword_payload)[0], 1 << 31)


class PlcStructTest(unittest.TestCase):
    """st_struct() vs the SIMATIC AX project's hand-written station types
    (ax_dev/src/project/site_1/types.st)."""

    def test_test_config_matches_station_1_input_and_output(self):
        config = load_config("test_config.json")
        publish, subscribe = io_topics(config)
        inp = custom_component_mapping(config, publish["input"]["custom_ids"], direction="out")
        out = custom_component_mapping(config, subscribe["output"]["custom_ids"], direction="in")
        bools = ["button_1_1_1", "button_1_1_2", "button_1_1_3", "sensor_1_1_1", "button_1_2_1", "button_1_2_2",
                 "button_1_2_3", "button_1_3_1", "button_1_3_2", "button_1_3_3"]
        self.assertEqual(io_layout.st_fields(inp), [(b, "BOOL") for b in bools] + [
            ("sensor_1_2_1", "DWORD"), ("sensor_1_3_1", "DWORD"), ("clock", "LINT")])
        leds = [f"led_1_{s}_{i}" for s in (1, 2, 3) for i in (1, 2, 3)]
        belts = [f"belt_1_{s}_{i}" for s in (1, 2, 3) for i in (1, 2, 3, 4, 5)]
        self.assertEqual(io_layout.st_fields(out), [(l, "BOOL") for l in leds] + [
            (b, "ARRAY[0..1] OF REAL") for b in belts])
        self.assertIn("END_STRUCT;  // 18 bytes", io_layout.st_struct("Station_1_Input", inp))

    def test_every_type_struct_and_its_size(self):
        config = every_type_config()
        inp = custom_component_mapping(config, INPUT_IDS, direction="out")
        fields = io_layout.st_fields(inp)
        self.assertEqual(fields, [("button_1", "BOOL"), ("eye", "BOOL"), ("button_2", "BOOL"),
                                  ("banner12", "WORD"), ("byte8", "BYTE"), ("word16", "WORD"),
                                  ("dword32", "DWORD"), ("eye_f", "REAL"), ("clock", "LINT")])
        self.assertEqual(io_layout.st_serialized_size(fields), io_layout.layout_size(inp))

    def test_belt_feedback_fields_get_their_own_names(self):
        config = every_type_config()
        out = custom_component_mapping(config, ["belt_1"], direction="out")
        self.assertEqual(io_layout.st_fields(out), [
            ("belt_1_running", "BOOL"), ("belt_1_atSpeed", "BOOL"), ("belt_1_reversing", "BOOL"),
            ("belt_1_fault", "BOOL"), ("belt_1_actualSpeed", "REAL"), ("belt_1_encoderCount", "DWORD")])
        self.assertEqual(io_layout.st_serialized_size(io_layout.st_fields(out)), io_layout.layout_size(out))

    def test_offset_table(self):
        inp = custom_component_mapping(every_type_config(), INPUT_IDS, direction="out")
        table = io_layout.format_layout(inp, "input")
        self.assertIn("input: 22 bytes, layout", table)
        self.assertIn("1.0-2.3", table)
        self.assertIn("banner12/sensor_states[0..11]  (12 bits)", table)
        self.assertIn("2.4-7", table)  # banner12's padding
        self.assertIn("14         uint64  LINT   clock/nanoseconds", table)


class LayoutCheckTest(unittest.TestCase):
    def test_fingerprint_changes_with_order_type_or_name(self):
        config = every_type_config()
        base = custom_component_mapping(config, INPUT_IDS, direction="out")
        fp = io_layout.layout_fingerprint(base)
        self.assertEqual(fp, io_layout.layout_fingerprint(list(base)))
        swapped = custom_component_mapping(config, ["eye"] + [i for i in INPUT_IDS if i != "eye"], direction="out")
        self.assertNotEqual(fp, io_layout.layout_fingerprint(swapped))
        config.sensors[2].packFormat = "uint16"
        self.assertNotEqual(fp, io_layout.layout_fingerprint(custom_component_mapping(config, INPUT_IDS, "out")))

    def test_check_frame_round_trip_and_compare(self):
        config = every_type_config()
        inp = custom_component_mapping(config, INPUT_IDS, direction="out")
        out = custom_component_mapping(config, OUTPUT_IDS, direction="in")
        frame = io_layout.pack_layout_check(inp, out, endianness=">")
        values = io_layout.unpack_layout_check(frame, ">")
        self.assertEqual(values[1], 22)
        self.assertEqual(values[4], io_layout.CHECK_REQUEST)
        self.assertEqual(io_layout.compare_layout_check(values[:4], values[:4]), [])
        other = (values[0], values[1], values[2] ^ 1, values[3] + 4)
        problems = io_layout.compare_layout_check(values[:4], other)
        self.assertEqual(len(problems), 1)
        self.assertTrue(problems[0].startswith("output:"))
        self.assertIsNone(io_layout.unpack_layout_check(b"\x00" * 3))


# ---------------------------------------------------------------------------
# Simulator <-> PLC over TCP
# ---------------------------------------------------------------------------

def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class TcpExchangeTest(unittest.TestCase):
    """The simulator's exchange engine without physics (so the states set
    here stay as set), synced on a thread like SimulationRunner does."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.addCleanup(self.tmp.cleanup)
        self.port = free_port()
        self.exchange = os.path.join(self.tmp.name, "exchange.json")
        with open(self.exchange, "w", encoding="utf-8") as f:
            json.dump({"transport": "tcp", "enabled": True, "tcpBindHost": "127.0.0.1",
                       "tcpServerHost": "127.0.0.1", "tcpPort": self.port, "endianness": "LE"}, f)

    def start_simulator(self, config):
        apply_exchange_config(config, self.exchange)
        self.sim = config
        self.lock = threading.Lock()
        self.engine = MQTTSyncEngine(config, lock=self.lock)
        self._running = True

        def loop():
            while self._running:
                self.engine.sync()
                time.sleep(0.005)
        thread = threading.Thread(target=loop, daemon=True)
        thread.start()

        def stop():
            self._running = False
            thread.join(timeout=2)
            self.engine.stop()
        self.addCleanup(stop)

    def write_layout(self, config, name):
        path = os.path.join(self.tmp.name, name)
        with open(path, "w", encoding="utf-8") as f:
            f.write(config.model_dump_json())
        return path

    def start_plc_mock(self, layout_path):
        proc = subprocess.Popen(
            [sys.executable, "-u", os.path.join(HERE, "plc_mock.py"), layout_path, "--exchange-config",
             self.exchange, "--scan", "0.02", "--send-interval", "60"],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8")
        lines = []

        def read():
            for line in proc.stdout:
                lines.append(line.rstrip())
        threading.Thread(target=read, daemon=True).start()

        def stop():
            proc.terminate()
            proc.wait(timeout=10)
            proc.stdout.close()
        self.addCleanup(stop)
        return proc, lines

    def test_values_cross_the_wire_exactly_both_ways(self):
        """The in-process PLC side: decode the simulator's input frame into
        its own I/O image, send commands back."""
        self.start_simulator(every_type_config())
        plc = every_type_config()
        for s in plc.sensors:
            s.sensor_states = "0" * len(s.sensor_states)
        publish, subscribe = io_topics(plc)
        inp = ComponentMessage(custom_component_mapping(plc, publish["input"]["custom_ids"], "out"), "<")
        out = ComponentMessage(custom_component_mapping(plc, subscribe["output"]["custom_ids"], "in"), "<")
        received = []

        def on_frame(frame):
            if frame.frame_type == FrameType.IO_MAPPING and len(frame.payload) == inp.payload_size:
                inp.deserialize(frame.payload, plc)
                received.append(frame.payload)
        client = TcpFramedClient("127.0.0.1", self.port, on_frame, length_endianness="LE")
        client.start()
        self.addCleanup(client.stop)
        self.assertTrue(wait_for(lambda: received), "no input frame")
        self.assertEqual({s.id: s.sensor_states for s in plc.sensors},
                         {s.id: s.sensor_states for s in self.sim.sensors})
        # The simulator orders inputs buttons first (io_topics): button_1,
        # button_2, eye -> 0b101; everything after the bit byte as above.
        self.assertEqual(received[-1][:14], bytes([0b101]) + INPUT_LE[1:14])  # all but the clock
        clock_ns = struct.unpack_from("<Q", received[-1], 14)[0]
        self.assertLess(abs(clock_ns - time.time_ns()), 5e9)

        plc.belts[0].speed, plc.belts[0].directionAngle = 2.75, 45.0
        plc.leds[0].isOn, plc.leds[1].isOn, plc.leds[2].isOn = False, True, False
        self.assertTrue(client.send(out.serialize(plc), frame_type=FrameType.IO_MAPPING))
        self.assertTrue(wait_for(lambda: self.sim.belts[0].speed == 2.75))
        self.assertEqual(self.sim.belts[0].directionAngle, 45.0)
        self.assertEqual([l.isOn for l in self.sim.leds], [False, True, False])

    def test_plc_mock_and_simulator_agree_on_the_layout(self):
        config = every_type_config()
        config.buttons[0].sensor_states = False             # START not pressed yet
        layout = self.write_layout(config, "line.json")
        self.start_simulator(load_config(layout))
        proc, lines = self.start_plc_mock(layout)
        self.assertTrue(wait_for(lambda: any("I/O layout check: OK" in l for l in lines), 15), "\n".join(lines))
        self.assertTrue(self.engine.layout_match)
        mine = self.engine.io_layout_check()
        self.assertTrue(any(f"input {mine[1]} bytes {mine[0]:08X}" in l for l in lines))
        # plc_mock drives the outputs: belts stopped until Start, RUN LED off.
        self.assertTrue(wait_for(lambda: self.engine.stats()["rx_frames"] > 5))
        self.assertEqual(self.engine.stats()["rx_size_errors"], 0)
        self.assertEqual(self.sim.belts[0].speed, 0.0)
        # Press START in the simulator: plc_mock sees it and runs the belt.
        with self.lock:
            self.sim.buttons[0].sensor_states = True
        self.assertTrue(wait_for(lambda: self.sim.belts[0].speed == 1.5), "\n".join(lines))
        self.assertTrue(self.sim.leds[0].isOn, "RUN LED")
        self.assertTrue(any("START (button_1)" in l for l in lines))
        self.assertIsNone(proc.poll())

    def test_a_plc_with_another_layout_is_reported_and_ignored(self):
        self.start_simulator(every_type_config())
        other = every_type_config()
        other.belts.append(Belt(id="belt_2", type="linear", x=0, y=0, length=2, beltWidth=1, rotation=0, speed=1,
                                directionAngle=0, color="#00f"))
        proc, lines = self.start_plc_mock(self.write_layout(other, "other.json"))
        self.assertTrue(wait_for(lambda: any("I/O layout MISMATCH" in l for l in lines), 15), "\n".join(lines))
        self.assertTrue(any("output: " in l for l in lines if "MISMATCH" in l))
        self.assertIs(self.engine.layout_match, False)
        self.assertTrue(wait_for(lambda: self.engine.stats()["rx_size_errors"] > 5))
        self.assertEqual(self.sim.belts[0].speed, 1.5, "frames of the wrong layout are not applied")
        self.assertIsNone(proc.poll(), "plc_mock keeps running")


if __name__ == "__main__":
    unittest.main()
