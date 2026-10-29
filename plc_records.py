"""
plc_records.py - a PLC's own records, sent as a field list once and then raw
values only (FrameType.RECORD_SCHEMA / RECORD_VALUES), turned back into JSON
messages by plc_foxglove_bridge.py.

The PLC (the AX project's RecordWriter, src/communication/record_writer.st)
names and types its fields once per connection, so each period it only
serializes values - no text formatting, no names. Little-endian throughout.

RECORD_SCHEMA payload
    slot   BYTE          which record of this connection (a PLC may send several)
    count  UINT          number of fields
    topic  STR8          the record's name: published on /plc/json/<topic>
    count x (type BYTE, name STR8)

RECORD_VALUES payload
    slot   BYTE
    count  UINT          must equal the schema's
    size   UINT          bytes of values that follow
    the values, in schema order

    STR8 = length BYTE + that many characters

Field types and their value encoding:
    1 BOOL  1 byte, 0 / 1        5 REAL   4 bytes float
    2 DINT  4 bytes signed       6 LREAL  8 bytes float
    3 UDINT 4 bytes unsigned     7 STRING STR8
    4 LINT  8 bytes signed

A field name is a path: "stations.0.running" puts running in the first
element of the array stations (a part that is a number is an array index).
"""
import struct
from typing import Any, Dict, List, Optional, Tuple

TYPES = {1: ("BOOL", "<?", 1), 2: ("DINT", "<i", 4), 3: ("UDINT", "<I", 4), 4: ("LINT", "<q", 8),
         5: ("REAL", "<f", 4), 6: ("LREAL", "<d", 8), 7: ("STRING", None, None)}


class RecordError(ValueError):
    """A record frame that cannot be decoded."""


class Schema:
    def __init__(self, slot: int, topic: str, fields: List[Tuple[int, str]]):
        self.slot, self.topic, self.fields = slot, topic, fields


def _str8(data: bytes, pos: int) -> Tuple[str, int]:
    if pos >= len(data):
        raise RecordError("truncated text")
    n = data[pos]
    if pos + 1 + n > len(data):
        raise RecordError("truncated text")
    return data[pos + 1:pos + 1 + n].decode("latin-1"), pos + 1 + n


def parse_schema(data: bytes) -> Schema:
    if len(data) < 3:
        raise RecordError("schema too short")
    slot, count = data[0], struct.unpack_from("<H", data, 1)[0]
    topic, pos = _str8(data, 3)
    fields = []
    for _ in range(count):
        if pos >= len(data):
            raise RecordError("schema has fewer fields than its count")
        type_code = data[pos]
        if type_code not in TYPES:
            raise RecordError(f"unknown field type {type_code}")
        name, pos = _str8(data, pos + 1)
        fields.append((type_code, name))
    if pos != len(data):
        raise RecordError("bytes after the last field")
    return Schema(slot, topic, fields)


def values_slot(data: bytes) -> int:
    if len(data) < 5:
        raise RecordError("values too short")
    return data[0]


def parse_values(data: bytes, schema: Schema) -> Dict[str, Any]:
    """The values as a nested JSON-ready dict, per the schema."""
    slot, count, size = data[0], *struct.unpack_from("<HH", data, 1)
    if count != len(schema.fields):
        raise RecordError(f"{count} values, the schema has {len(schema.fields)} fields")
    if size != len(data) - 5:
        raise RecordError(f"size {size}, {len(data) - 5} bytes of values")
    flat: List[Tuple[str, Any]] = []
    pos = 5
    for type_code, name in schema.fields:
        _t, fmt, width = TYPES[type_code]
        if fmt is None:
            value, pos = _str8(data, pos)
        else:
            if pos + width > len(data):
                raise RecordError("truncated values")
            value = struct.unpack_from(fmt, data, pos)[0]
            pos += width
        flat.append((name, value))
    if pos != len(data):
        raise RecordError("bytes after the last value")
    return nest(flat)


def nest(flat: List[Tuple[str, Any]]) -> Dict[str, Any]:
    """[("a.0.b", 1), ("c", 2)] -> {"a": [{"b": 1}], "c": 2}."""
    root: Dict[str, Any] = {}
    for path, value in flat:
        parts = path.split(".")
        node = root
        for part in parts[:-1]:
            node = node.setdefault(part, {})
            if not isinstance(node, dict):
                raise RecordError(f"{path}: {part} is a value and a group")
        node[parts[-1]] = value
    return _arrays(root)


def _arrays(node: Any) -> Any:
    if not isinstance(node, dict):
        return node
    node = {k: _arrays(v) for k, v in node.items()}
    if node and all(k.isdigit() for k in node):
        return [node[k] for k in sorted(node, key=int)]
    return node


# ---------------------------------------------------------------------------
# The PLC side in Python (tests, tools): what RecordWriter sends
# ---------------------------------------------------------------------------

_CODES = {name: code for code, (name, _f, _w) in TYPES.items()}


def pack_schema(slot: int, topic: str, fields: List[Tuple[str, str]]) -> bytes:
    """fields: [(name, type name)]."""
    out = bytearray([slot]) + struct.pack("<H", len(fields)) + bytes([len(topic)]) + topic.encode("latin-1")
    for name, type_name in fields:
        out += bytes([_CODES[type_name], len(name)]) + name.encode("latin-1")
    return bytes(out)


def pack_values(slot: int, fields: List[Tuple[str, str]], values: List[Any]) -> bytes:
    body = bytearray()
    for (_name, type_name), value in zip(fields, values):
        _t, fmt, _w = TYPES[_CODES[type_name]]
        body += bytes([len(value)]) + value.encode("latin-1") if fmt is None else struct.pack(fmt, value)
    return bytes([slot]) + struct.pack("<HH", len(fields), len(body)) + bytes(body)

