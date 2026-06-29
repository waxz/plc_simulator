import argparse
import json
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from config_parser import Belt, Sensor, ControlButton, LedIndicator, SimulatorConfig, TopicConfig, load_config
from struct_utils import pack_data, string_to_bitmap, unpack_data


@dataclass
class MessageField:
    component_id: str
    component_type: str
    field: str
    binding: str
    value: Any
    binary_group: str


def full_topic(config: SimulatorConfig, topic: TopicConfig) -> str:
    prefix = config.mqttSettings.topicPrefix.strip("/")
    name = topic.topic.strip("/")
    if prefix and name:
        return f"{prefix}/{name}"
    return prefix or name


def find_topic(config: SimulatorConfig, topic_id: str) -> TopicConfig:
    for topic in config.mqttSettings.topics:
        if topic.id == topic_id or topic.topic == topic_id:
            return topic
    raise ValueError(f"Topic not found: {topic_id}")


def _find_component(config: SimulatorConfig, component_id: str):
    for collection in [config.belts, config.sensors, config.sources, config.sinks, config.buttons, config.leds]:
        for item in collection:
            if item.id == component_id:
                return item
    return None


def _sensor_state_value(sensor: Sensor, sensor_states: Optional[Dict[str, Any]]):
    if sensor_states and sensor.id in sensor_states:
        return sensor_states[sensor.id]
    return sensor.isActive


def compose_fields(
    config: SimulatorConfig,
    topic: TopicConfig,
    sensor_states: Optional[Dict[str, Any]] = None,
) -> List[MessageField]:
    fields: List[MessageField] = []

    for topic_component in topic.components:
        item = _find_component(config, topic_component.id)
        if item is None:
            fields.append(
                MessageField(topic_component.id, topic_component.type, "missing", "", None, "meta")
            )
            continue

        if isinstance(item, Sensor):
            state = _sensor_state_value(item, sensor_states)
            
            if item.sensorType == "laser_banner":
                fields.append(
                    MessageField(
                        item.id,
                        topic_component.type,
                        "word",
                        item.wordBinding or "word",
                        state,
                        "float",
                    )
                )
            else:
                fields.append(
                MessageField(
                    item.id,
                    topic_component.type,
                    "state",
                    item.stateBinding or "state",
                    state,
                    "bitmap",
                )
            )
            
        elif isinstance(item, Belt):
            fields.extend(
                [
                    MessageField(item.id, topic_component.type, "speed", item.speedBinding or "speed", item.speed, "float"),
                    MessageField(
                        item.id,
                        topic_component.type,
                        "directionAngle",
                        item.directionAngleBinding or "directionAngle",
                        item.directionAngle,
                        "float",
                    ),
                    MessageField(
                        item.id,
                        topic_component.type,
                        "direction",
                        item.directionBinding or "direction",
                        item.direction,
                        "float",
                    ),
                ]
            )
        elif isinstance(item, ControlButton):
            fields.append(
                MessageField(
                    item.id,
                    topic_component.type,
                    "state",
                    item.stateBinding or "state",
                    item.sensor_states,
                    "bitmap",
                )
            )
        elif isinstance(item, LedIndicator):
            fields.append(
                MessageField(
                    item.id,
                    topic_component.type,
                    "state",
                    item.stateBinding or "state",
                    item.isOn,
                    "bitmap",
                )
            )

    return fields


def fields_to_nested(fields: List[MessageField]) -> Dict[str, Dict[str, Any]]:
    message: Dict[str, Dict[str, Any]] = {}
    for field in fields:
        if field.field == "missing":
            continue
        message.setdefault(field.component_id, {})[field.binding or field.field] = field.value
    return message


def _state_to_bits(value: Any) -> str:
    if isinstance(value, str):
        cleaned = "".join(ch for ch in value if ch in "01")
        if cleaned:
            return cleaned
        return "1" if value.lower() in {"true", "yes", "on", "triggered"} else "0"
    return "1" if bool(value) else "0"


def _float_values(fields: List[MessageField]) -> List[float]:
    values = []
    for field in fields:
        if field.binary_group == "float":
            values.append(float(field.value or 0.0))
    return values


def encode_message(
    config: SimulatorConfig,
    topic: TopicConfig,
    sensor_states: Optional[Dict[str, Any]] = None,
) -> bytes:
    fields = compose_fields(config, topic, sensor_states)
    fmt = "string" if topic.format == "raw" else topic.format

    if fmt == "json":
        return json.dumps(fields_to_nested(fields), separators=(",", ":")).encode("utf-8")

    if fmt == "string":
        pairs = []
        for component_id, values in fields_to_nested(fields).items():
            for key, value in values.items():
                pairs.append(f"{component_id}.{key}={value}")
        return ";".join(pairs).encode("utf-8")

    if fmt == "binary_struct":
        bits = "".join(_state_to_bits(field.value) for field in fields if field.binary_group == "bitmap")
        bitmap = string_to_bitmap(bits or "0", alignment="left")
        endian = "big" if topic.endianness == "BE" else "little"
        return pack_data(bitmap, _float_values(fields), endian=endian)

    raise ValueError(f"Unsupported MQTT message format: {topic.format}")


def decode_message(
    config: SimulatorConfig,
    topic: TopicConfig,
    payload: bytes,
) -> Dict[str, Dict[str, Any]]:
    fields = compose_fields(config, topic)
    fmt = "string" if topic.format == "raw" else topic.format

    if fmt == "json":
        return json.loads(payload.decode("utf-8"))

    if fmt == "string":
        decoded: Dict[str, Dict[str, Any]] = {}
        text = payload.decode("utf-8", errors="replace")
        for part in text.split(";"):
            if "=" not in part:
                continue
            left, value = part.split("=", 1)
            component_id, key = left.split(".", 1) if "." in left else ("message", left)
            decoded.setdefault(component_id, {})[key] = value
        return decoded or {"message": {"value": text}}

    if fmt == "binary_struct":
        bitmap_fields = [field for field in fields if field.binary_group == "bitmap"]
        float_fields = [field for field in fields if field.binary_group == "float"]
        bitmap_len = max(1, (sum(max(1, len(_state_to_bits(field.value))) for field in bitmap_fields) + 7) // 8)
        endian = "big" if topic.endianness == "BE" else "little"
        bitmap, floats = unpack_data(payload, len(float_fields), bitmap_len_bytes=bitmap_len, endian=endian)

        decoded: Dict[str, Dict[str, Any]] = {}
        bit_string = "".join(f"{byte:08b}" for byte in bitmap)
        bit_offset = 0
        for field in bitmap_fields:
            bit_count = max(1, len(_state_to_bits(field.value)))
            raw_bits = bit_string[bit_offset:bit_offset + bit_count]
            value = raw_bits if bit_count > 1 else raw_bits == "1"
            decoded.setdefault(field.component_id, {})[field.binding or field.field] = value
            bit_offset += bit_count

        for field, value in zip(float_fields, floats):
            decoded.setdefault(field.component_id, {})[field.binding or field.field] = value

        return decoded

    raise ValueError(f"Unsupported MQTT message format: {topic.format}")


def estimate_bandwidth(config: SimulatorConfig, topic: TopicConfig, sensor_states: Optional[Dict[str, Any]] = None):
    payload = encode_message(config, topic, sensor_states)
    packet_bytes = 4 + len(full_topic(config, topic).encode("utf-8")) + len(payload)
    messages_per_second = 1000.0 / max(1, topic.interval)
    return {
        "payload_bytes": len(payload),
        "estimated_packet_bytes": packet_bytes,
        "messages_per_second": messages_per_second,
        "bytes_per_second": packet_bytes * messages_per_second,
    }


def _load_sensor_states(raw: Optional[str]) -> Optional[Dict[str, Any]]:
    if not raw:
        return None
    if raw.startswith("@"):
        with open(raw[1:], "r", encoding="utf-8") as f:
            return json.load(f)
    return json.loads(raw)


def _payload_from_arg(raw: str) -> bytes:
    if raw.startswith("hex:"):
        return bytes.fromhex(raw[4:].replace(" ", ""))
    if raw.startswith("@"):
        with open(raw[1:], "rb") as f:
            return f.read()
    return raw.encode("utf-8")


def main():
    parser = argparse.ArgumentParser(description="Compose, encode, decode, and size MQTT simulator messages.")
    parser.add_argument("config", help="Simulator JSON config path")
    parser.add_argument("topic", help="Topic id or topic name from mqttSettings.topics")
    parser.add_argument("--sensor-states", help='JSON object or @file, e.g. {"sensor_1": true}')
    parser.add_argument("--decode", help="Decode payload text, hex:<bytes>, or @file instead of encoding")
    parser.add_argument("--table", action="store_true", help="Print composer rows")
    parser.add_argument("--bandwidth", action="store_true", help="Print bandwidth estimate")
    args = parser.parse_args()

    config = load_config(args.config)
    topic = find_topic(config, args.topic)
    sensor_states = _load_sensor_states(args.sensor_states)

    if args.table:
        for field in compose_fields(config, topic, sensor_states):
            print(f"{field.component_id}\t{field.field}\t{field.binding or '-'}\t{field.value}\t{field.binary_group}")

    if args.decode is not None:
        decoded = decode_message(config, topic, _payload_from_arg(args.decode))
        print(json.dumps(decoded, indent=2))
    else:
        payload = encode_message(config, topic, sensor_states)
        if topic.format == "binary_struct":
            print(payload.hex(" "))
        else:
            print(payload.decode("utf-8"))

    if args.bandwidth:
        print(json.dumps(estimate_bandwidth(config, topic, sensor_states), indent=2))


if __name__ == "__main__":
    main()
