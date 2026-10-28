"""
test_exchange_options.py - the TCP exchange's robustness options, end to
end with real sockets (mqtt_data_exchange.MQTTSyncEngine +
tcp_framed_transport): the layout-check reply goes to the asking client
only (another client - a PLC that decodes any frame as its input image -
never sees it), output ownership by node id, and outputs accepted only
after a matching layout check.
"""
import socket
import threading
import time
import unittest

import io_layout
from mqtt_data_exchange import SUBSCRIBE_TOPICS, MQTTSyncEngine
from mqtt_message_binding import ComponentMessage, custom_component_mapping
from tcp_framed_transport import FrameType
from test_tcp_exchange import BELT, FrameRecorder as Collector, connect, endian, make_config, wait_for
from mqtt_data_exchange import PUBLISH_TOPICS


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class ExchangeOptionsTest(unittest.TestCase):
    def start(self, **settings):
        self.port = free_port()
        self.config = make_config(self.port)
        for key, value in settings.items():
            setattr(self.config.mqttSettings, key, value)
        self.engine = MQTTSyncEngine(self.config, lock=threading.Lock())
        self.addCleanup(self.engine.stop)
        self.clients = []

    def client(self):
        frames = Collector()
        c = connect(self.port, self.config, frames)
        self.addCleanup(c.stop)
        self.assertTrue(c.connected)
        self.clients.append(c)
        self.assertTrue(wait_for(lambda: self.engine.stats().get("clients") == len(self.clients)))
        return c, frames

    def mappings(self):
        inp = custom_component_mapping(self.config, PUBLISH_TOPICS["input"]["custom_ids"], direction="out")
        out = custom_component_mapping(self.config, SUBSCRIBE_TOPICS["output"]["custom_ids"], direction="in")
        return inp, out

    def output_frame(self, speed: float) -> bytes:
        from config_parser import load_config
        plc = load_config("test_config.json")
        next(b for b in plc.belts if b.id == BELT).speed = speed
        return ComponentMessage(self.mappings()[1], endianness=endian(self.config)).serialize(plc)

    def layout_check(self, client, node_id=7):
        inp, out = self.mappings()
        client.send(io_layout.pack_layout_check(inp, out, endianness=endian(self.config)),
                    frame_type=FrameType.LAYOUT_CHECK, node_id=node_id)

    def belt(self):
        return next(b for b in self.config.belts if b.id == BELT)

    def test_the_layout_check_reply_goes_to_the_asking_client_only(self):
        self.start()
        a, a_frames = self.client()
        b, b_frames = self.client()
        self.layout_check(a)
        self.assertTrue(wait_for(lambda: a_frames.of(FrameType.LAYOUT_CHECK)))
        time.sleep(0.3)
        self.assertEqual(b_frames.of(FrameType.LAYOUT_CHECK), [])
        self.assertTrue(self.engine.layout_match)

    def test_only_the_owner_node_drives_the_outputs(self):
        self.start(tcpOutputOwner=7)
        c, _ = self.client()
        c.send(self.output_frame(2.5), frame_type=FrameType.IO_MAPPING, node_id=9)
        self.assertTrue(wait_for(lambda: self.engine.stats()["rx_ignored"] == 1))
        self.assertNotEqual(self.belt().speed, 2.5)
        c.send(self.output_frame(2.5), frame_type=FrameType.IO_MAPPING, node_id=7)
        self.assertTrue(wait_for(lambda: self.belt().speed == 2.5))

    def test_outputs_wait_for_a_matching_layout_check(self):
        self.start(tcpRequireLayoutCheck=True)
        c, frames = self.client()
        c.send(self.output_frame(1.75), frame_type=FrameType.IO_MAPPING, node_id=7)
        self.assertTrue(wait_for(lambda: self.engine.stats()["rx_ignored"] == 1))
        self.layout_check(c)
        self.assertTrue(wait_for(lambda: frames.of(FrameType.LAYOUT_CHECK)))
        c.send(self.output_frame(1.75), frame_type=FrameType.IO_MAPPING, node_id=7)
        self.assertTrue(wait_for(lambda: self.belt().speed == 1.75))


if __name__ == "__main__":
    unittest.main()
