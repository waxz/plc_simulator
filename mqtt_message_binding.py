import re
import struct
import time
from dataclasses import is_dataclass
from typing import Any, List, Optional, Tuple, Union

from config_parser import Sensor, SimulatorConfig, load_config
# The field types and the device kinds on the exchange (ByteType is
# re-exported here for the modules that import it from here).
from device_kinds import DEVICE_KINDS, ByteType, exchanged_devices, kind_of
from metrics_store import MetricsStore
import pandas as pd


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

    for collection_name in ("sensors", "belts", "buttons", "leds", "sources", "pallets", *DEVICE_KINDS):
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


# Bit width of each ByteType that can hold a compacted bit-string (a whole
# sensor_states/isOn packed into one field instead of one BIT row per bit)
# - FLOAT is deliberately excluded: it has no clean "bit capacity" (a
# float32 only represents integers exactly up to 2**24), so a too-wide
# value silently loses precision instead of being caught here.
_COMPACT_BYTE_TYPE_BITS = {
    ByteType.BYTE: 8, ByteType.UINT16: 16, ByteType.UINT32: 32, ByteType.UINT64: 64,
}


def verify_component_list(components_mapping: List[Tuple[ByteType, str]], config: SimulatorConfig) -> None:
    """Validate that all mapped components, attributes, and bit indexes exist."""
    for byte_type, path in components_mapping:
        component_name, variable, index = parse_path(path)
        if component_name.startswith("padding"):
            continue
        if component_name == "clock":
            if byte_type != ByteType.UINT64 or variable not in ("nanoseconds", "simNanoseconds"):
                raise ValueError(
                    "Virtual clock must be mapped as ByteType.UINT64, 'clock/nanoseconds' (wall clock) "
                    "or 'clock/simNanoseconds' (simulated time)."
                )
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
        if (
            byte_type in _COMPACT_BYTE_TYPE_BITS
            and isinstance(component, Sensor)
            and variable == "sensor_states"
        ):
            needed_bits = sensor_state_bit_count(component)
            available_bits = _COMPACT_BYTE_TYPE_BITS[byte_type]
            if needed_bits > available_bits:
                raise ValueError(
                    f"'{component_name}/sensor_states' needs {needed_bits} bits but ByteType.{byte_type.name} "
                    f"only holds {available_bits} - use a wider compact type or map individual BIT entries."
                )


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

# ByteType a component's packFormat (config_parser.PackFormat) resolves to
# when it isn't "bit" - see paddng_component() below.
_PACK_FORMAT_BYTE_TYPES = {
    "byte": ByteType.BYTE,
    "uint16": ByteType.UINT16,
    "uint32": ByteType.UINT32,
    "float": ByteType.FLOAT,
}


def paddng_component(config: SimulatorConfig, sensor_id: Optional[Union[str, List[str]]] = None,
                     direction: str = "in") -> List[Tuple[ByteType, str]]:
    """
    Byte layout for a list of sensor/button/LED/belt ids, from what each
    component's own config declares (its `packFormat`, defaulting to "bit")
    - the same rule the PLC side's serializer uses (generate_st_serde.py in
    the SIMATIC AX project: every BOOL first, then the other fields in
    declaration order, no alignment):

      1. Every single-bit component (IR sensor, button, LED with packFormat
         "bit"), in the order given, 8 to a byte (bit 0 first).
      2. Padding bits up to the next whole byte.
      3. Then, in the order given, every multi-bit or register component:
           - a laser banner ("bit"): its beams as bits, padded to whole
             bytes, so two banners never share a byte (on the PLC a BYTE /
             WORD / DWORD / LWORD, or an ARRAY OF BYTE)
           - packFormat "byte" / "uint16" / "uint32" / "float": one register
             holding the whole state (BYTE / WORD / DWORD / REAL); an LED
             with packFormat "byte" sends its blink `mode` (devices.py)
           - a stopper / robot arm / elevator (machines.py), by
             `direction`: its command ("in") or feedback ("out") bits (in
             step 1) and values - see MACHINE_FIELDS
           - a belt (motor), by `direction`:
               "in"  (PLC -> simulator, the command): commandFormat "vfd":
                     run + reverse bits (in step 1) and speed REAL;
                     "frequency": run + reverse bits and frequency REAL (Hz);
                     "speed": speed + directionAngle REALs
               "out" (simulator -> PLC, the feedback): running, atSpeed,
                     reversing, fault bits (in step 1), then actualSpeed
                     REAL + encoderCount DWORD; a "frequency" belt adds
                     outputFrequency (Hz) and motorSpeed (rpm) REALs

    Padding entries are named padding_<n>/bit, numbered through the mapping.
    """
    if not sensor_id:
        return []
    sensor_ids = [sensor_id] if isinstance(sensor_id, str) else sensor_id

    sensors_in_config = {s.id: s for s in config.sensors}
    buttons_in_config = {b.id: b for b in config.buttons}
    leds_in_config = {b.id: b for b in config.leds}
    belts_in_config = {b.id: b for b in getattr(config, "belts", [])}
    machines_in_config = {m.id: m for kind in DEVICE_KINDS for m in getattr(config, kind, [])}

    bits: List[Tuple[ByteType, str]] = []
    wide: List[List[Tuple[ByteType, str]]] = []  # one chunk per multi-bit / register component
    for sid in sensor_ids:
        belt = belts_in_config.get(sid)
        if belt is not None:
            belt_bits, belt_values = belt_fields(belt, direction)
            bits.extend((bt, f"{sid}/{field}") for bt, field in belt_bits)
            wide.append([(bt, f"{sid}/{field}") for bt, field in belt_values])
            continue
        machine = machines_in_config.get(sid)
        if machine is not None:
            machine_bits, machine_values = machine_fields(machine, direction)
            bits.extend((bt, f"{sid}/{field}") for bt, field in machine_bits)
            if machine_values:
                wide.append([(bt, f"{sid}/{field}") for bt, field in machine_values])
            continue
        component = sensors_in_config.get(sid) or buttons_in_config.get(sid) or leds_in_config.get(sid)
        if component is None:
            continue
        pack_format = getattr(component, "packFormat", "bit") or "bit"
        is_led = sid in leds_in_config
        attribute = "isOn" if is_led else "sensor_states"
        if is_led and pack_format == "byte":
            attribute = "mode"  # 0 off, 1 on, 2/3 blink - devices.update_led
        if pack_format != "bit":
            wide.append([(_PACK_FORMAT_BYTE_TYPES[pack_format], f"{sid}/{attribute}")])
        elif sid in sensors_in_config and sensors_in_config[sid].sensorType == "laser_banner":
            count = sensor_state_bit_count(sensors_in_config[sid])
            wide.append([(ByteType.BIT, f"{sid}/sensor_states[{i}]") for i in range(count)])
        else:
            # LEDs are a single isOn bit; buttons and IR sensors bit 0 of sensor_states.
            bits.append((ByteType.BIT, f"{sid}/{attribute}" if is_led else f"{sid}/{attribute}[0]"))

    mapping: List[Tuple[ByteType, str]] = list(bits)
    _, padding_index = _append_padding_bits(mapping, len(bits) % 8, 0)
    for chunk in wide:
        mapping.extend(chunk)
        if chunk[0][0] == ByteType.BIT:
            _, padding_index = _append_padding_bits(mapping, len(chunk) % 8, padding_index)
    return mapping

def print_mapping(mapping, filename):
    # print like a table
    # create a dataframe and print it
    df = pd.DataFrame(mapping, columns=["ByteType", "Path"])
    # print(df)
    df.to_csv(filename, index=False)


def read_mapping_rows(filename: str) -> List[Tuple[str, str]]:
    """Raw (ByteType, Path) string pairs straight out of a mapping CSV -
    unexpanded (a "PAD" row stays "PAD") and unvalidated. This is what an
    editor round-trips through save_mapping_rows(): re-expanding PAD or
    resolving ByteType names on read would bake padding_N/none rows in
    permanently and defeat the point of writing "PAD,auto" by hand. Use
    load_mapping_csv() (or parse_mapping_rows() on this function's output)
    to get the fully-resolved mapping MQTTSyncEngine actually consumes."""
    df = pd.read_csv(filename)
    if list(df.columns[:2]) != ["ByteType", "Path"]:
        raise ValueError(f"{filename}: expected columns 'ByteType,Path', got {list(df.columns)}")
    return [(str(bt).strip(), str(p).strip()) for bt, p in zip(df["ByteType"], df["Path"])]


def save_mapping_rows(rows: List[Tuple[str, str]], filename: str) -> None:
    """Write raw (ByteType-name-or-PAD, Path) string rows to a CSV as
    authored. Unlike print_mapping() (which takes real ByteType members
    from an already-resolved mapping), this accepts whatever a person
    typed - including "PAD" - so an editor's save doesn't expand shorthand
    the person will want to see again next time they open the file."""
    df = pd.DataFrame(rows, columns=["ByteType", "Path"])
    df.to_csv(filename, index=False)


def parse_mapping_rows(
    rows: List[Tuple[str, str]], source: str = "<rows>", start_row: int = 1
) -> List[Tuple[ByteType, str]]:
    """Resolve raw (ByteType-name-or-PAD, Path) string rows (as read by
    read_mapping_rows(), or typed into an editor) into a real mapping list.
    Accepts both the "ByteType.BIT" form print_mapping() writes and a bare
    "BIT"/"bit" form, so a hand-edited row doesn't have to match Python's
    str(enum) formatting exactly. Raises ValueError naming `source` and the
    row for a value that isn't a real ByteType, since the intended reader
    of a bad row is someone who isn't reading this module's traceback.

    A row with ByteType "PAD" expands to however many padding BIT entries
    are needed to reach the next byte boundary from the current row (zero,
    if already aligned) - the same padding_N/none entries
    sensor_state_mapping()'s _append_padding_bits() inserts automatically,
    just available to a hand-edited row too. This is the one part of
    authoring a mapping by hand that isn't just "look up the id and pick a
    type": counting how many BIT rows came before a multi-byte field and
    padding to a full byte is exactly the kind of bookkeeping a non-expert
    editing a spreadsheet will get wrong. Writing "PAD,auto" after a run of
    BIT rows removes the counting entirely - it's always correct, and a
    no-op if the run was already byte-aligned."""
    mapping: List[Tuple[ByteType, str]] = []
    bit_pending = 0
    pad_index = 0
    for row_num, (raw_type, path) in enumerate(rows, start=start_row):
        name = str(raw_type).strip()
        if name.startswith("ByteType."):
            name = name[len("ByteType."):]
        name = name.upper()

        if name == "PAD":
            needed = (8 - bit_pending) % 8
            for _ in range(needed):
                mapping.append((ByteType.BIT, f"padding_{pad_index}/none"))
                pad_index += 1
            bit_pending = 0
            continue

        try:
            byte_type = ByteType[name]
        except KeyError:
            raise ValueError(f"{source}:{row_num}: unknown ByteType '{raw_type}'") from None
        mapping.append((byte_type, str(path).strip()))
        bit_pending = (bit_pending + 1) % 8 if byte_type == ByteType.BIT else 0
    return mapping


def load_mapping_csv(filename: str) -> List[Tuple[ByteType, str]]:
    """Inverse of print_mapping(): read a "ByteType,Path" CSV and fully
    resolve it (PAD expanded, ByteType names validated) - read_mapping_rows()
    + parse_mapping_rows(), the combination MQTTSyncEngine/channel_registry
    actually need. Row numbers in any error start at 2, matching the CSV's
    header line."""
    return parse_mapping_rows(read_mapping_rows(filename), source=filename, start_row=2)

def known_component_ids(config: SimulatorConfig) -> "set[str]":
    """Every id custom_component_mapping()/paddng_component() actually
    recognize - sensors/belts/buttons/leds plus the virtual "clock" id.
    Sources and plain sinks have no IO fields, so they're excluded (a
    chute - a sink with a capacity - has, see device_kinds.py).
    Shared by custom_component_mapping()'s own validation, the bootstrap
    CLI, and the web registry editor's live validation, so "is this a real
    component id" has exactly one answer everywhere it's asked."""
    ids = {s.id for s in config.sensors} | {b.id for b in config.belts}
    ids |= {b.id for b in config.buttons} | {l.id for l in config.leds}
    ids |= {m.id for m in exchanged_devices(config)}
    ids.add("clock")
    return ids


# Machines' fields per channel direction ("in" = PLC -> simulator, the
# command; "out" = simulator -> PLC, the feedback): (bits, values) like
# belt_fields - declared per device kind in device_kinds.py.
MACHINE_FIELDS = {name: {"in": kind.commands, "out": kind.feedback} for name, kind in DEVICE_KINDS.items()}
_MACHINE_KIND = {kind.model.__name__: name for name, kind in DEVICE_KINDS.items()}


def machine_fields(machine, direction: str) -> Tuple[List[Tuple[ByteType, str]], List[Tuple[ByteType, str]]]:
    """(bits, values) a machine / device contributes to a frame: its
    command ("in") or its feedback ("out")."""
    return kind_of(machine).fields(direction)


# Belt fields per channel direction (channel_registry's convention: "in" =
# client -> simulator, "out" = simulator -> client). Inbound carries the PLC's
# commands; outbound carries the drive model's feedback (drive.py).
BELT_COMMAND_FIELDS: List[Tuple[ByteType, str]] = [(ByteType.FLOAT, "speed"), (ByteType.FLOAT, "directionAngle")]
VFD_COMMAND_BITS: List[Tuple[ByteType, str]] = [(ByteType.BIT, "run"), (ByteType.BIT, "reverse")]
VFD_COMMAND_FIELDS: List[Tuple[ByteType, str]] = [(ByteType.FLOAT, "speed")]
FREQUENCY_COMMAND_FIELDS: List[Tuple[ByteType, str]] = [(ByteType.FLOAT, "frequency")]
BELT_STATUS_BITS: List[Tuple[ByteType, str]] = [
    (ByteType.BIT, "running"), (ByteType.BIT, "atSpeed"), (ByteType.BIT, "reversing"), (ByteType.BIT, "fault")]
BELT_FEEDBACK_FIELDS: List[Tuple[ByteType, str]] = [(ByteType.FLOAT, "actualSpeed"), (ByteType.UINT32, "encoderCount")]
# A belt commanded in Hz also reports what an inverter does: its output
# frequency (Hz) and the motor speed (rpm).
FREQUENCY_FEEDBACK_FIELDS: List[Tuple[ByteType, str]] = BELT_FEEDBACK_FIELDS + [
    (ByteType.FLOAT, "outputFrequency"), (ByteType.FLOAT, "motorSpeed")]


def belt_fields(belt, direction: str) -> Tuple[List[Tuple[ByteType, str]], List[Tuple[ByteType, str]]]:
    """(bits, values) a belt contributes to a frame: its command ("in") or
    its feedback ("out") - see paddng_component()."""
    command_format = getattr(belt, "commandFormat", "speed")
    if direction == "out":
        return BELT_STATUS_BITS, (FREQUENCY_FEEDBACK_FIELDS if command_format == "frequency" else BELT_FEEDBACK_FIELDS)
    if command_format == "frequency":
        return VFD_COMMAND_BITS, FREQUENCY_COMMAND_FIELDS
    if command_format == "vfd":
        return VFD_COMMAND_BITS, VFD_COMMAND_FIELDS
    return [], BELT_COMMAND_FIELDS


def belt_feedback_mapping(config: SimulatorConfig, belt_ids: Optional[List[str]] = None) -> List[Tuple[ByteType, str]]:
    """The BELT_FEEDBACK frame's payload: every belt's status bits (running,
    atSpeed, reversing, fault) first, padded to a byte, then per belt
    actualSpeed (REAL) + encoderCount (DWORD) - in config order (or
    `belt_ids` order)."""
    ids = belt_ids if belt_ids is not None else [b.id for b in config.belts]
    return paddng_component(config, sensor_id=list(ids), direction="out")


def custom_component_mapping(config: SimulatorConfig, component_ids: List[str], direction: str = "in") -> List[Tuple[ByteType, str]]:
    """Build a compact payload mapping for a custom list of component IDs instead of the whole config.

    `direction` decides what a belt contributes: "in" (client -> simulator,
    the default) maps the speed/directionAngle commands, "out" (simulator ->
    client) maps the drive feedback actualSpeed/encoderCount."""
    if direction not in ("in", "out"):
        raise ValueError(f"direction must be 'in' or 'out', got '{direction}'")
    mapping: List[Tuple[ByteType, str]] = []

    missing_ids = [cid for cid in component_ids if cid not in known_component_ids(config)]
    if missing_ids:
        print(
            "Custom component mapping contains unknown component IDs: {}".format(
                ", ".join(missing_ids)
            )
        )

    # Every component, bits first then the rest in declaration order
    # (paddng_component), belts included; the clock last.
    mapping.extend(paddng_component(config, sensor_id=component_ids, direction=direction))

    if "clock" in component_ids:
        mapping.append((ByteType.UINT64, "clock/nanoseconds"))

    return mapping


def create_pack_format_string(components_mapping: List[Tuple[ByteType, str]], endianness: str = ">") -> str:
    """Generate a struct format string, packing sequential BIT entries into bytes."""
    type_map = {
        ByteType.BYTE: "B",
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


def _compact_int(value: Any, byte_type: Optional["ByteType"] = None) -> int:
    """Coerce a mapped value to a single integer for BYTE/UINT16/UINT32/
    UINT64 packing. A bool or a "0"/"1" bit-string (e.g. a sensor's
    sensor_states, or an LED's isOn) packs a whole component's state into
    one compact field instead of one BIT row per bit: character i becomes
    bit i (beam 0 = bit 0), exactly where the "bit" packFormat puts it, so
    a banner's WORD/DWORD reads the same on the PLC either way. Anything
    else (an already-numeric field like a counter) is a plain int().

    Masked to `byte_type`'s bit width (a no-op for a value that already
    fits): verify_component_list() rejects a too-narrow compact type for
    sensor_states up front, but this mask is what keeps struct.pack() from
    raising OverflowError instead of silently truncating in the rarer case
    a value exceeds its declared width some other way."""
    if isinstance(value, bool):
        result = int(value)
    elif isinstance(value, str):
        cleaned = "".join(ch for ch in value if ch in "01")
        result = int(cleaned[::-1], 2) if cleaned else 0  # character i -> bit i
    else:
        result = int(value or 0)

    bits = _COMPACT_BYTE_TYPE_BITS.get(byte_type)
    if bits is not None:
        result &= (1 << bits) - 1
    return result


def _int_to_matching_shape(value: int, current: Any) -> Any:
    """Inverse of _compact_int(): write an unpacked integer back into
    whatever shape the attribute already had - a bool stays a bool, a
    bit-string is rebuilt at its original width (high bits beyond that
    width are dropped, matching what a too-narrow compact ByteType can
    actually represent), anything else is returned as-is."""
    if isinstance(current, bool):
        return bool(value)
    if isinstance(current, str):
        width = len(current) or 1
        return format(value & ((1 << width) - 1), f"0{width}b")[::-1]  # bit i -> character i
    return value


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
            elif component_name == "clock" and variable == "simNanoseconds":
                value = config.simTimeNs
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
                pack_values.append(_compact_int(value, byte_type))
            elif byte_type == ByteType.FLOAT:
                if isinstance(value, (str, bool)):
                    pack_values.append(float(_compact_int(value)))
                else:
                    pack_values.append(float(value or 0.0))

        if bit_shift_index:
            pack_values.append(bit_accumulator)

        return struct.pack(self.format_string, *pack_values)

    def deserialize(self, data: bytes, config: SimulatorConfig, skip_manual: bool = False) -> None:
        """Unpack a raw payload and write values back into mapped config attributes.
        `skip_manual` (the simulator's inbound PLC frames): components whose
        commands are set by hand (control "manual") keep theirs."""
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
                    if skip_manual and getattr(component, "control", None) == "manual":
                        continue
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
            if skip_manual and getattr(component, "control", None) == "manual":
                continue
            current = getattr(component, variable)
            if isinstance(current, (str, bool)):
                setattr(component, variable, _int_to_matching_shape(int(value), current))
            else:
                setattr(component, variable, value)


class MetricsMessage:
    """Like ComponentMessage, but for a pre-declared list of client-
    reported metrics (performance counters, cycle times, elapsed times,
    numeric results, vector values) instead of simulator components -
    the same ByteType-driven struct packing (create_pack_format_string),
    but reads/writes a MetricsStore by (group, name) instead of
    SimulatorConfig attributes, since metrics are a dynamic time series,
    not persistent component state.

    Mapping paths reuse the exact same "group/name" and "group/name[i]"
    syntax parse_path() already parses for simulator components - here
    "group" takes the place of a component id and "name" the place of an
    attribute. Entries sharing a (group, name) with different [i] are
    reassembled into one "vector" sample per deserialize() call, mirroring
    how ComponentMessage assembles a laser banner's bit-indexed
    sensor_states.

    There is no string/text ByteType (ComponentMessage doesn't have one
    either) - a metric that's inherently a free-form result string needs
    to be reported as a numeric code over this wire format instead.
    """

    def __init__(self, mapping: List[Tuple[ByteType, str]], endianness: str = ">"):
        self.mapping = mapping
        self.format_string = create_pack_format_string(mapping, endianness)
        self.payload_size = struct.calcsize(self.format_string)

    @classmethod
    def from_layout(cls, layout_cls: type, endianness: str = ">") -> "MetricsMessage":
        """Build from a metric layout dataclass (metric_layout.py) instead
        of a hand-written (ByteType, path) list - layout_mapping() derives
        the exact same list from the class's typed, grouped fields. This is
        how mqtt_data_exchange.py's METRICS_CHANNELS declares a layout now;
        the (ByteType, path) constructor above still works unchanged for
        the registry path (channel_registry.py) and tests."""
        from metric_layout import layout_mapping  # local: metric_layout imports ByteType from here

        return cls(layout_mapping(layout_cls), endianness)

    def serialize(self, values: "dict[Tuple[str, str], Any]") -> bytes:
        """`values`: {(group, name): scalar_value_or_list}, packed
        according to self.mapping - the inverse of deserialize(). A
        missing entry packs as 0. A filled-in metric layout dataclass
        instance (metric_layout.py) is also accepted and flattened to that
        dict. Used by metrics-reporting clients (see plc_mock.py), not by
        the simulator itself."""
        if is_dataclass(values) and not isinstance(values, type):
            from metric_layout import layout_values

            values = layout_values(values)

        pack_values = []
        bit_accumulator = 0
        bit_shift_index = 0

        for byte_type, path in self.mapping:
            group, name, index = parse_path(path)
            raw = values.get((group, name))
            if index is not None:
                value = raw[index] if isinstance(raw, (list, tuple)) and index < len(raw) else 0
            else:
                value = raw

            if byte_type == ByteType.BIT:
                if bool(value):
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
            if byte_type == ByteType.FLOAT:
                pack_values.append(float(value or 0.0))
            else:
                pack_values.append(int(value or 0))

        if bit_shift_index:
            pack_values.append(bit_accumulator)

        return struct.pack(self.format_string, *pack_values)

    def deserialize(self, data: bytes, store: MetricsStore, node_id: int = 0) -> None:
        """Unpack a raw payload and write each mapped entry into `store`
        as a sample, keyed by (group, name) - the inverse of serialize().
        Entries sharing a (group, name) with different [i] are assembled
        into one "vector" sample before writing, not written as N
        separate scalar samples.

        `node_id` - the sender's identity, carried in the TCP frame header
        (see FrameType/ParsedFrame in tcp_framed_transport.py) - is
        prefixed onto every group written here (`"perf"` becomes
        `"node0/perf"`), so multiple physical nodes reporting metrics to
        the same simulator land as distinct groups in MetricsStore/the
        data-visualization panel instead of overwriting each other."""
        if len(data) != self.payload_size:
            print(
                "Error unpacking metrics data: expected {} bytes, got {}".format(
                    self.payload_size, len(data)
                )
            )
            return
        try:
            unpacked_data = list(struct.unpack(self.format_string, data))
        except struct.error as exc:
            print(
                "Error unpacking metrics data: {}. Expected {} bytes, got {}: {}".format(
                    exc, self.payload_size, len(data), data
                )
            )
            return

        unpacked_idx = 0
        bit_byte_value = 0
        bit_shift_index = 0

        scalars: "dict[Tuple[str, str], Any]" = {}
        vector_components: "dict[Tuple[str, str], dict[int, Any]]" = {}

        for byte_type, path in self.mapping:
            group, name, index = parse_path(path)

            if byte_type == ByteType.BIT:
                if bit_shift_index == 0:
                    bit_byte_value = unpacked_data[unpacked_idx]
                    unpacked_idx += 1
                bit_value = bool((bit_byte_value >> bit_shift_index) & 0x01)
                bit_shift_index = (bit_shift_index + 1) % 8
                if not group.startswith("padding"):
                    scalars[(group, name)] = bit_value
                continue

            bit_shift_index = 0
            if byte_type == ByteType.PADDING:
                continue

            value = unpacked_data[unpacked_idx]
            unpacked_idx += 1
            if group.startswith("padding"):
                continue

            if index is None:
                scalars[(group, name)] = value
            else:
                vector_components.setdefault((group, name), {})[index] = value

        node_prefix = f"node{node_id}/"
        for (group, name), value in scalars.items():
            store.add_sample(name=name, value=value, group=node_prefix + group)
        for (group, name), components in vector_components.items():
            ordered = [components[i] for i in sorted(components)]
            store.add_sample(name=name, value=ordered, group=node_prefix + group, kind="vector")


def main():
    config = load_config("station_1.json")
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
