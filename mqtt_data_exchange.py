"""
mqtt_data_exchange.py - MQTT data exchange engine for the conveyor simulator.

Manages publish (simulator -> PLC) and subscribe (PLC -> simulator) channels
using ComponentMessage for binary serialization/deserialization. Each channel
has its own mapping, topic, and rate-limited interval.

Usage in main.py:
    mqtt = MQTTSyncEngine(config)
    # in game loop:
    mqtt.sync()
    # on shutdown:
    mqtt.stop()
"""

from paho.mqtt import packettypes
import json
import time
import threading
from typing import Dict, List, Any

from mqtt_manager import FlexibleProtocolMQTTManager
from config_parser import SimulatorConfig
from mqtt_message_binding import (
    ComponentMessage,
    custom_component_mapping,
    verify_component_list,print_mapping
)
from tcp_framed_transport import TcpFramedServer


# ---------------------------------------------------------------------------
# Default topic configurations.
# Each entry maps a topic string to its interval (ms) and custom_ids list.
# These are used when mqttSettings.topics is empty or doesn't cover bindings.
# ---------------------------------------------------------------------------
PUBLISH_TOPICS: Dict[str, Dict[str, Any]] = {
    "input": {
        "interval": 10,  # ms
        "custom_ids": [
            "button_1_1_1", "button_1_1_2","button_1_1_3","sensor_1_1_1",
            "button_1_2_1", "button_1_2_2", "button_1_2_3","sensor_1_2_1",
            "button_1_3_1", "button_1_3_2", "button_1_3_3","sensor_1_3_1",
            "clock",
        # "button_1_4_1", "button_1_4_2", "button_1_4_3",
        # "button_1_5_1", "button_1_5_2", "button_1_5_3",
        ],
    },
}

SUBSCRIBE_TOPICS: Dict[str, Dict[str, Any]] = {
    "output": {
        "interval": 10,  # ms
        "custom_ids": [
            "led_1_1_1", "led_1_1_2", "led_1_1_3", "belt_1_1_1", "belt_1_1_2", "belt_1_1_3", "belt_1_1_4", "belt_1_1_5",
            "led_1_2_1", "led_1_2_2", "led_1_2_3", "belt_1_2_1", "belt_1_2_2", "belt_1_2_3", "belt_1_2_4", "belt_1_2_5",
            "led_1_3_1", "led_1_3_2", "led_1_3_3", "belt_1_3_1", "belt_1_3_2", "belt_1_3_3", "belt_1_3_4", "belt_1_3_5"
        ],
    },
}


def _struct_endianness(topic_def: Dict[str, Any], config: SimulatorConfig | None = None) -> str:
    """Return the struct byte-order prefix for PLC payload values."""
    default_endianness = config.mqttSettings.endianness if config is not None else "LE"
    endianness = str(topic_def.get("endianness", default_endianness)).upper()
    return "<" if endianness == "LE" else ">"


def apply_exchange_config(config: SimulatorConfig, file_path: str) -> None:
    """Overlay exchange settings from a small JSON file onto the simulator config."""
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    settings_data = data.get("mqttSettings", data)
    merged = config.mqttSettings.model_dump()
    merged.update(settings_data)
    config.mqttSettings = type(config.mqttSettings)(**merged)


class _TopicChannel:
    """Internal helper: one publish or subscribe channel with rate limiting."""

    def __init__(self, topic: str, message: ComponentMessage, interval_ms: int):
        self.topic = topic
        self.message = message
        self.interval_s = interval_ms / 1000.0
        self.last_time = 0.0

    def is_due(self) -> bool:
        """Return True if enough time has elapsed since the last send/recv."""
        return (time.monotonic() - self.last_time) >= self.interval_s

    def mark(self) -> None:
        self.last_time = time.monotonic()


class MQTTSyncEngine:
    """
    Owns the MQTT connection and manages publish/subscribe channels.

    Constructor accepts a SimulatorConfig directly (not a file path) so that
    main.py can share the same live config object with the physics engine and
    renderer.
    """

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self._enabled = config.mqttSettings.enabled
        self._transport = config.mqttSettings.transport
        self._pub_channels: List[_TopicChannel] = []
        self._sub_channels: List[_TopicChannel] = []
        self._mqtt: FlexibleProtocolMQTTManager | None = None
        self._tcp: TcpFramedServer | None = None
        self._lock = threading.Lock()

        if not self._enabled:
            print("[DataExchange] Exchange disabled in config. Skipping.")
            return

        prefix = config.mqttSettings.topicPrefix or ""

        if self._transport == "mqtt":
            broker_url = config.mqttSettings.brokerUrl
            broker_port = config.mqttSettings.port
            self._mqtt = FlexibleProtocolMQTTManager(
                broker_url=broker_url,
                port=broker_port,
            )
        elif self._transport != "tcp":
            raise ValueError(f"Unsupported exchange transport: {self._transport}")


        # Build publish channels
        for topic_suffix, topic_def in PUBLISH_TOPICS.items():
            full_topic = f"{prefix}{topic_suffix}".replace("//", "/")
            custom_ids = topic_def["custom_ids"]
            interval_ms = topic_def.get("interval", 100)

            mapping = custom_component_mapping(config, custom_ids)
            verify_component_list(mapping, config)
            message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def, config))
            print("[MQTTSync] Publish mapping: ",mapping)
            print_mapping(mapping,f"mapping_{topic_suffix}.csv")

            channel = _TopicChannel(full_topic, message, interval_ms)
            self._pub_channels.append(channel)
            print(f"[DataExchange] Publish channel: {full_topic}  "
                  f"({len(mapping)} entries, {message.payload_size} payload bytes, {interval_ms} ms)")

        # Build subscribe channels and register callbacks
        for topic_suffix, topic_def in SUBSCRIBE_TOPICS.items():
            full_topic = f"{prefix}{topic_suffix}".replace("//", "/")
            custom_ids = topic_def["custom_ids"]
            interval_ms = topic_def.get("interval", 100)

            mapping = custom_component_mapping(config, custom_ids)
            print("[MQTTSync] Subscribe mapping: ",mapping)
            verify_component_list(mapping, config)
            message = ComponentMessage(mapping, endianness=_struct_endianness(topic_def, config))
            print_mapping(mapping,f"mapping_{topic_suffix}.csv")

            channel = _TopicChannel(full_topic, message, interval_ms)
            self._sub_channels.append(channel)

            # Register the deserialization callback
            if self._mqtt is not None:
                self._mqtt.register_callback(
                    full_topic,
                    self._make_recv_handler(channel),
                )
            print(f"[DataExchange] Subscribe channel: {full_topic}  "
                  f"({len(mapping)} entries, {message.payload_size} payload bytes, {interval_ms} ms)")

        if self._mqtt is not None:
            self._mqtt.start()
            print(f"[MQTTSync] Connecting to {broker_url}:{broker_port} ...")

        if self._transport == "tcp":
            bind_host = config.mqttSettings.tcpBindHost
            tcp_port = config.mqttSettings.tcpPort
            self._tcp = TcpFramedServer(
                bind_host,
                tcp_port,
                self._handle_tcp_payload,
                length_endianness=config.mqttSettings.endianness,
            )
            self._tcp.start()
            print(f"[TCPSync] Simulator server listening on {bind_host}:{tcp_port} ...")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def sync(self) -> None:
        """
        Called once per frame from the game loop.
        Publishes all due outbound channels. Inbound channels are updated
        asynchronously by MQTT callbacks.
        """
        if not self._enabled:
            return

        for ch in self._pub_channels:
            if ch.is_due():
                with self._lock:
                    try:
                        payload = ch.message.serialize(self.config)
                    except Exception as exc:
                        print(f"[MQTTSync] Error serializing message: {exc}")
                        continue
                    if self._mqtt is not None:
                        self._mqtt.publish(ch.topic, payload)
                    elif self._tcp is not None:
                        # add payload length in first word
                        # 1. Convert the 4-byte magic marker to bytes outside the loop/function for performance
                        # MAGIC_MARKER = (0x12345678).to_bytes(4, byteorder="big")
                        # length_bytes = len(payload).to_bytes(4, byteorder="big")
                        # payload = MAGIC_MARKER + length_bytes + payload
                        # print(f"[TCPSync] Payload length: {length_bytes}")
                        self._tcp.send(payload)
                ch.mark()

    def stop(self) -> None:
        """Cleanly disconnect from the broker."""
        if self._mqtt is not None:
            self._mqtt.stop()
            print("[MQTTSync] Disconnected.")
        if self._tcp is not None:
            self._tcp.stop()
            print("[TCPSync] Stopped.")

    @property
    def connected(self) -> bool:
        if self._mqtt is not None:
            return self._mqtt.client.is_connected()
        if self._tcp is not None:
            return self._tcp.connected
        return False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _make_recv_handler(self, channel: _TopicChannel):
        """Create a closure that deserializes incoming payloads into self.config."""
        def handler(raw):
            payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
            if len(payload) != channel.message.payload_size:
                print(
                    f"[MQTTSync] Ignoring {channel.topic} payload: "
                    f"expected {channel.message.payload_size} bytes, got {len(payload)}"
                )
                return

            with self._lock:
                channel.message.deserialize(payload, self.config)
            channel.mark()
        return handler

    def _handle_tcp_payload(self, payload: bytes) -> None:
        if not self._sub_channels:
            print("[TCPSync] Ignoring payload because no subscribe channels are configured.")
            return
        if len(self._sub_channels) > 1:
            print("[TCPSync] Multiple subscribe channels configured; using the first channel.")
        
        # # remove first word
        # length = int.from_bytes(payload[4:8], byteorder="big")
        # print(f"[TCPSync] Received payload length: {length}")
        # payload = payload[8:]
        self._make_recv_handler(self._sub_channels[0])(payload)
