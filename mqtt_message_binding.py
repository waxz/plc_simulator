import re
import struct
import time
from enum import Enum
from typing import Any, List, Optional, Tuple, Union

from config_parser import Sensor, SimulatorConfig, load_config
import pandas as pd

class ByteType(Enum):
    BIT = "bit"
    BYTE = "byte"
    UINT16 = "uint16"
    UINT32 = "uint32"
    UINT64 = "uint64"
    FLOAT = "float"
    PADDING = "padding"


# Mapping format layout: (Type, "component_id/property_name")
# Use property_name[index] to address a single bit inside a string bitmap.
_component_list: List[Tuple[ByteType, str]] = [
    (ByteType.FLOAT, "belt_1/speed"),
    (ByteType.FLOAT, "belt_1/directionAngle"),
    (ByteType.FLOAT, "belt_2/speed"),
    (ByteType.FLOAT, "belt_2/directionAngle"),
]

_PATH_RE = re.compile(r"^(?P<component>[^/]+)/(?P<field>[^\[]+)(?:\[(?P<index>\d+)])?$")


def find_component_by_id(component_id: str, config: SimulatorConfig) -> Optional[Any]:
    """Find a component configuration by ID across simulator component lists."""
    if component_id.startswith("padding"):
        return True
    if component_id == "clock":
        return True

    for collection_name in ("sensors", "belts", "buttons", "leds", "sources", "sinks"):
        collection = getattr(config, collection_name, [])
        for component in collection:
            if component.id == component_id:
                return component

    raise ValueError(f"Component '{component_id}' not found in configuration lists.")


def parse_path(path: str) -> Tuple[str, str, Optional[int]]:
    """Extract component, variable, and optional bit-string index from a mapping path."""
    match = _PATH_RE.match(path)
    if not match:
        parts = path.split("/", 1)
        component_name = parts[0]
        variable = parts[1] if len(parts) == 2 else "state"
        return component_name, variable, None

    index = match.group("index")
    return match.group("component"), match.group("field"), int(index) if index is not None else None


def verify_component_list(components_mapping: List[Tuple[ByteType, str]], config: SimulatorConfig) -> None:
    """Validate that all mapped components, attributes, and bit indexes exist."""
    for byte_type, path in components_mapping:
        component_name, variable, index = parse_path(path)
        if component_name.startswith("padding"):
            continue
        if component_name == "clock":
            if byte_type != ByteType.UINT64 or variable != "nanoseconds":
                raise ValueError("Virtual clock must be mapped as ByteType.UINT64, 'clock/nanoseconds'.")
            continue

        component = find_component_by_id(component_name, config)
        if component is None:
            raise ValueError(f"Component '{component_name}' not found in config.")
        if not hasattr(component, variable):
            raise ValueError(f"Component '{component_name}' is missing attribute '{variable}'.")
        if (
            byte_type == ByteType.BIT
            and isinstance(component, Sensor)
            and component.sensorType == "laser_banner"
            and variable == "sensor_states"
            and index is None
        ):
            raise ValueError(
                f"Laser banner '{component_name}/sensor_states' must use indexed bit paths."
            )
        if byte_type == ByteType.BIT and index is not None:
            value = getattr(component, variable)
            if isinstance(value, str) and index >= len(value):
                raise ValueError(f"Bit index {index} is outside '{component_name}/{variable}'.")


def sensor_state_bit_count(sensor: Sensor) -> int:
    """Return how many BIT fields are needed for this sensor state."""
    if sensor.sensorType == "ir":
        return 1
    if sensor.laserCount is not None:
        return int(sensor.laserCount)
    if isinstance(sensor.sensor_states, str) and sensor.sensor_states:
        return len(sensor.sensor_states)
    return 16


def _padding_bit_path(index: int) -> str:
    return f"padding_{index}/bit"


def _append_padding_bits(mapping: List[Tuple[ByteType, str]], bit_offset: int, padding_index: int) -> Tuple[int, int]:
    if bit_offset == 0:
        return bit_offset, padding_index

    for _ in range(8 - bit_offset):
        mapping.append((ByteType.BIT, _padding_bit_path(padding_index)))
        padding_index += 1
    return 0, padding_index


def _sorted_sensors_for_packing(
    config: SimulatorConfig,
    sensor_id: Optional[Union[str, List[str]]] = None
) -> List[Sensor]:
    if sensor_id is None:
        allowed = None
    elif isinstance(sensor_id, str):
        allowed = {sensor_id}
    else:
        allowed = set(sensor_id)

    sensors = [
        sensor
        for sensor in config.sensors
        if allowed is None or sensor.id in allowed
    ]
    if isinstance(sensor_id, str):
        return sensors

    return sorted(
        sensors,
        key=lambda sensor: (
            0 if sensor.sensorType == "ir" else 1,
            sensor.id,
        ),
    )


def sensor_state_mapping(
    config: SimulatorConfig,
    sensor_id: Optional[Union[str, List[str]]] = None
) -> List[Tuple[ByteType, str]]:
    """Build byte-aligned BIT mappings for sensor states.

    Full-config mappings are sorted compactly: IR sensors consume the first bit
    range, then laser banners start on byte boundaries and are padded to
    complete bytes so their bitmap never shares a byte with another sensor
    state.
    """
    mapping: List[Tuple[ByteType, str]] = []
    bit_offset = 0
    padding_index = 0

    for sensor in _sorted_sensors_for_packing(config, sensor_id):
        bit_count = sensor_state_bit_count(sensor)

        if sensor.sensorType == "laser_banner":
            bit_offset, padding_index = _append_padding_bits(mapping, bit_offset, padding_index)

        mapping.extend(
            (ByteType.BIT, f"{sensor.id}/sensor_states[{index}]")
            for index in range(bit_count)
        )

        bit_offset = (bit_offset + bit_count) % 8
        if sensor.sensorType == "laser_banner":
            bit_offset, padding_index = _append_padding_bits(mapping, bit_offset, padding_index)

    return mapping


def compact_component_mapping(config: SimulatorConfig) -> List[Tuple[ByteType, str]]:
    """Build a compact default payload: single-bit sensors, bitmap sensors, then belt floats."""
    return [
        *sensor_state_mapping(config),
        *_component_list,
    ]

from typing import List, Tuple, Union, Optional
# Assuming ByteType and SimulatorConfig are imported at the top of your file
# from m3 import ByteType, SimulatorConfig

def paddng_component(config: SimulatorConfig, sensor_id: Optional[Union[str, List[str]]] = None) -> List[Tuple[ByteType, str]]:
    """
    Processes a list of sensor/button IDs, differentiates single-bit inputs from multi-byte/bit devices,
    and appends bit-level padding to cleanly align the layout to an 8-bit byte boundary.
    """
    mapping: List[Tuple[ByteType, str]] = []
    bit_offset = 0
    padding_index = 0

    if not sensor_id:
        return mapping

    # Standardize input to a list if a single string ID was provided
    sensor_ids = [sensor_id] if isinstance(sensor_id, str) else sensor_id

    # Create a lookup cache of all sensors available in the configuration
    sensors_in_config = {s.id: s for s in config.sensors}
    buttons_in_config = {b.id: b for b in config.buttons}
    leds_in_config = {b.id: b for b in config.leds}

    multi_byte_sensors = []

    # 1. First Pass: Process and append single-bit components (IR sensors and buttons)
    for sid in sensor_ids:
        if sid in sensors_in_config:
            sensor_obj = sensors_in_config[sid]
            # Check if it's a multi-byte sensor (e.g., "laser_banner") vs standard "ir" bit sensor
            if getattr(sensor_obj, "sensorType", "").lower() == "laser_banner":
                multi_byte_sensors.append(sid)
            else:
                mapping.append((ByteType.BIT, f"{sid}/sensor_states[0]"))
                bit_offset += 1
        elif sid in buttons_in_config:
            mapping.append((ByteType.BIT, f"{sid}/sensor_states[0]"))
            bit_offset += 1
        elif sid in leds_in_config:
            mapping.append((ByteType.BIT, f"{sid}/isOn"))
            bit_offset += 1

    # 2. Second Pass: Calculate and add bit-level padding to reach a clean 8-bit byte boundary
    remainder = bit_offset % 8
    if remainder != 0:
        needed_padding_bits = 8 - remainder
        for _ in range(needed_padding_bits):
            mapping.append((ByteType.BIT, f"padding_{padding_index}/none"))
            padding_index += 1

    # 3. Third Pass: Safely append the multi-byte layout items *after* bit alignment is locked
    for sid in multi_byte_sensors:
        sensor_obj = sensors_in_config[sid]
        bit_count = sensor_state_bit_count(sensor_obj)
        
        for b_idx in range(bit_count):
            mapping.append((ByteType.BIT, f"{sid}/sensor_states[{b_idx}]"))

    return mapping

def print_mapping(mapping, filename):
    # print like a table
    # create a dataframe and print it
    df = pd.DataFrame(mapping, columns=["ByteType", "Path"])
    # print(df)
    df.to_csv(filename, index=False)

def custom_component_mapping(config: SimulatorConfig, component_ids: List[str]) -> List[Tuple[ByteType, str]]:
    """Build a compact payload mapping for a custom list of component IDs instead of the whole config."""
    mapping: List[Tuple[ByteType, str]] = []

    sensors_in_config = {s.id: s for s in config.sensors}
    buttons_in_config = {b.id: b for b in config.buttons}
    leds_in_config = {l.id: l for l in config.leds}
    belts_in_config = {b.id: b for b in config.belts}
    virtual_ids = {"clock"}
    known_ids = set(sensors_in_config) | set(buttons_in_config) | set(leds_in_config) | set(belts_in_config) | virtual_ids
    missing_ids = [cid for cid in component_ids if cid not in known_ids]
    if missing_ids:
        print(
            "Custom component mapping contains unknown component IDs: {}".format(
                ", ".join(missing_ids)
            )
        )

    # Preserve the topic-declared order for bit items. paddng_component keeps
    # single-bit sensors/buttons/LEDs in this order, then byte-aligns before
    # appending multi-byte laser banner sensors.
    mapping.extend(paddng_component(config, sensor_id=component_ids))


    # 2. Float items: Belts
    custom_belts = [cid for cid in component_ids if cid in belts_in_config]

    for bid in custom_belts:
        mapping.extend([
            (ByteType.FLOAT, f"{bid}/speed"),
            (ByteType.FLOAT, f"{bid}/directionAngle"),
        ])

    if "clock" in component_ids:
        mapping.append((ByteType.UINT64, "clock/nanoseconds"))

    return mapping


def paddng_component1( config: SimulatorConfig,
    sensor_id: Optional[Union[str, List[str]]] = None):
    # find ir and buttons at first
    # check how many bit is used and how many padding bit
    # add other mutiple byte sensor like laser banner bytes
    mapping: List[Tuple[ByteType, str]] = []
    bit_offset = 0
    padding_index = 0
 
    return mapping


def custom_component_mapping1(config: SimulatorConfig, component_ids: List[str]) -> List[Tuple[ByteType, str]]:
    """Build a compact payload mapping for a custom list of component IDs instead of the whole config."""
    mapping: List[Tuple[ByteType, str]] = []

    sensors_in_config = {s.id: s for s in config.sensors}
    buttons_in_config = {b.id: b for b in config.buttons}
    leds_in_config = {l.id: l for l in config.leds}

    # 1. Bit items: Sensors, Buttons, LEDs
    custom_sensors = [cid for cid in component_ids if cid in sensors_in_config]
    custom_sensors.sort(key=lambda sid: (
        0 if sensors_in_config[sid].sensorType == "ir" else 1,
        sid
    ))


    custom_buttons = [cid for cid in component_ids if cid in buttons_in_config]
    custom_buttons.sort()
    # for bid in custom_buttons:
    #     mapping.append((ByteType.BIT, f"{bid}/sensor_states"))
    custom_sensors += custom_buttons

    mapping.extend(paddng_component(config, sensor_id=custom_sensors))

    custom_leds = [cid for cid in component_ids if cid in leds_in_config]
    custom_leds.sort()
    for lid in custom_leds:
        mapping.append((ByteType.BIT, f"{lid}/isOn"))

    # 2. Float items: Belts
    belts_in_config = {b.id: b for b in config.belts}
    custom_belts = [cid for cid in component_ids if cid in belts_in_config]
    custom_belts.sort()

    for bid in custom_belts:
        mapping.extend([
            (ByteType.FLOAT, f"{bid}/speed"),
            (ByteType.FLOAT, f"{bid}/directionAngle"),
        ])

    return mapping


def create_pack_format_string(components_mapping: List[Tuple[ByteType, str]], endianness: str = ">") -> str:
    """Generate a struct format string, packing sequential BIT entries into bytes."""
    type_map = {
        ByteType.BYTE: "b",
        ByteType.UINT16: "H",
        ByteType.UINT32: "I",
        ByteType.UINT64: "Q",
        ByteType.FLOAT: "f",
        ByteType.PADDING: "x",
    }

    format_parts = [endianness]
    bit_count = 0

    for byte_type, _ in components_mapping:
        if byte_type == ByteType.BIT:
            if bit_count == 0:
                format_parts.append("B")
            bit_count = (bit_count + 1) % 8
            continue

        bit_count = 0
        format_parts.append(type_map[byte_type])

    return "".join(format_parts)


def _read_bit(value: Any, index: Optional[int]) -> bool:
    if index is not None:
        if isinstance(value, str):
            if index >= len(value):
                return False
            return value[index] == "1"
        return bool((int(value or 0) >> index) & 0x01)

    if isinstance(value, str):
        cleaned = "".join(ch for ch in value if ch in "01")
        if cleaned:
            return cleaned != "0" * len(cleaned)
        return value.lower() in {"true", "yes", "on"}
    return bool(value)


def _write_bit(value: Any, index: Optional[int], bit_value: bool) -> Any:
    if index is None:
        return bit_value

    if isinstance(value, bool):
        return bit_value

    if isinstance(value, str):
        width = max(len(value), index + 1)
        chars = list(value.ljust(width, "0"))
        chars[index] = "1" if bit_value else "0"
        return "".join(chars)

    current = int(value or 0)
    if bit_value:
        return current | (1 << index)
    return current & ~(1 << index)


class ComponentMessage:
    def __init__(self, mapping: List[Tuple[ByteType, str]], endianness: str = ">"):
        self.mapping = mapping
        self.format_string = create_pack_format_string(mapping, endianness)
        self.payload_size = struct.calcsize(self.format_string)

    def serialize(self, config: SimulatorConfig) -> bytes:
        """Read mapped variables from config and pack them into a raw payload."""
        pack_values = []
        bit_accumulator = 0
        bit_shift_index = 0

        for byte_type, path in self.mapping:
            component_name, variable, index = parse_path(path)
            value = 0
            if component_name == "clock" and variable == "nanoseconds":
                value = time.time_ns()
            elif not component_name.startswith("padding"):
                component = find_component_by_id(component_name, config)
                if component and component is not True:
                    value = getattr(component, variable)
                else:
                    print("Component '{}' not found in config".format(component_name))

            if byte_type == ByteType.BIT:
                if _read_bit(value, index):
                    bit_accumulator |= 1 << bit_shift_index
                bit_shift_index += 1
                if bit_shift_index == 8:
                    pack_values.append(bit_accumulator)
                    bit_accumulator = 0
                    bit_shift_index = 0
                continue

            if bit_shift_index:
                pack_values.append(bit_accumulator)
                bit_accumulator = 0
                bit_shift_index = 0

            if byte_type == ByteType.PADDING:
                continue
            if byte_type in (ByteType.BYTE, ByteType.UINT16, ByteType.UINT32, ByteType.UINT64):
                pack_values.append(int(value or 0))
            elif byte_type == ByteType.FLOAT:
                pack_values.append(float(value or 0.0))

        if bit_shift_index:
            pack_values.append(bit_accumulator)

        return struct.pack(self.format_string, *pack_values)

    def deserialize(self, data: bytes, config: SimulatorConfig) -> None:
        """Unpack a raw payload and write values back into mapped config attributes."""
        if len(data) != self.payload_size:
            print(
                "Error unpacking data: expected {} bytes, got {}".format(
                    self.payload_size,
                    len(data),
                )
            )
            return

        try:
            unpacked_data = list(struct.unpack(self.format_string, data))
        except struct.error as exc:
            print(
                "Error unpacking data: {}. Expected {} bytes, got {}: {}".format(
                    data,
                    self.payload_size,
                    len(data),
                    exc,
                )
            )
            return

        unpacked_idx = 0
        bit_byte_value = 0
        bit_shift_index = 0

        for byte_type, path in self.mapping:
            component_name, variable, index = parse_path(path)

            if byte_type == ByteType.BIT:
                if bit_shift_index == 0:
                    bit_byte_value = unpacked_data[unpacked_idx]
                    unpacked_idx += 1

                bit_value = bool((bit_byte_value >> bit_shift_index) & 0x01)
                bit_shift_index = (bit_shift_index + 1) % 8

                if not component_name.startswith("padding"):
                    component = find_component_by_id(component_name, config)
                    current = getattr(component, variable)
                    setattr(component, variable, _write_bit(current, index, bit_value))
                continue

            bit_shift_index = 0
            if byte_type == ByteType.PADDING:
                continue

            value = unpacked_data[unpacked_idx]
            unpacked_idx += 1
            if component_name == "clock":
                continue
            component = find_component_by_id(component_name, config)
            setattr(component, variable, value)


def main():
    config = load_config("test_config.json")
    mapping = compact_component_mapping(config)
    verify_component_list(mapping, config)

    msg_handler = ComponentMessage(mapping)
    print(f"Generated layout: {msg_handler.format_string}")

    serialized_bytes = msg_handler.serialize(config)
    print(f"Serialized Payload (Hex): {serialized_bytes.hex()}")

    msg_handler.deserialize(serialized_bytes, config)
    print("Deserialization completely successful.")


if __name__ == "__main__":
    main()
