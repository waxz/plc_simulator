"""
io_layout.py - what an I/O frame looks like on the wire, byte by byte, and
the matching PLC data type.

A mapping (a list of (ByteType, "component/field[index]") entries, see
mqtt_message_binding.py) is packed without alignment, in mapping order:

  BIT       one bit; consecutive BITs share a byte, bit 0 first
  PADDING   one unused byte                      (padding_<n>/bit BITs are
            unused bits that fill a byte)
  BYTE      1 byte    -> PLC BYTE
  UINT16    2 bytes   -> PLC WORD
  UINT32    4 bytes   -> PLC DWORD
  UINT64    8 bytes   -> PLC LWORD (the clock: LINT, nanoseconds)
  FLOAT     4 bytes   -> PLC REAL (IEEE 754 single)

in the exchange's byte order (mqttSettings.endianness, "LE" by default).
A BIT followed by anything else ends its byte. paddng_component() orders
every layout like the PLC's serializer (generate_st_serde.py in the SIMATIC
AX project): every single bit first, padding to a whole byte, then all
other fields in declaration order - so the ST STRUCT from st_struct() (BOOLs
first, then the rest) serializes to exactly these bytes.

layout_fingerprint() is a CRC-32 of the layout (offsets, types, names). The
simulator and plc_mock.py print it, and plc_mock.py asks the simulator to
compare theirs when it connects (FrameType.LAYOUT_CHECK).

    python io_layout.py my_layout.json            # offset tables
    python io_layout.py my_layout.json --st       # + the PLC STRUCTs
"""
import argparse
import re
import struct
import zlib
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from mqtt_message_binding import ByteType, parse_path

Mapping = Sequence[Tuple[ByteType, str]]

BYTE_SIZES = {ByteType.BYTE: 1, ByteType.UINT16: 2, ByteType.UINT32: 4, ByteType.UINT64: 8, ByteType.FLOAT: 4,
              ByteType.PADDING: 1}
PLC_TYPES = {ByteType.BIT: "BOOL", ByteType.BYTE: "BYTE", ByteType.UINT16: "WORD", ByteType.UINT32: "DWORD",
             ByteType.UINT64: "LWORD", ByteType.FLOAT: "REAL"}
_WHOLE_BYTE_TYPES = {1: "BYTE", 2: "WORD", 4: "DWORD", 8: "LWORD"}


@dataclass(frozen=True)
class LayoutField:
    byte: int               # offset in the payload
    bit: Optional[int]      # 0..7 for a BIT, else None
    size: int               # bytes (0 for a BIT: it shares its byte)
    byte_type: ByteType
    path: str

    @property
    def is_padding(self) -> bool:
        return self.path.startswith("padding") or self.byte_type == ByteType.PADDING

    @property
    def plc_type(self) -> str:
        if self.path.startswith("clock/"):
            return "LINT"
        return PLC_TYPES.get(self.byte_type, "")

    @property
    def address(self) -> str:
        return f"{self.byte}.{self.bit}" if self.bit is not None else f"{self.byte}"


def layout_fields(mapping: Mapping) -> List[LayoutField]:
    """Every mapping entry with its byte (and bit) offset - the same walk
    create_pack_format_string() / ComponentMessage do."""
    fields: List[LayoutField] = []
    offset = 0
    bit = 0  # bits used in the current bit byte (0 = no bit byte open)
    for byte_type, path in mapping:
        if byte_type == ByteType.BIT:
            fields.append(LayoutField(offset, bit, 0, byte_type, path))
            bit += 1
            if bit == 8:
                offset, bit = offset + 1, 0
            continue
        if bit:
            offset, bit = offset + 1, 0
        size = BYTE_SIZES[byte_type]
        fields.append(LayoutField(offset, None, size, byte_type, path))
        offset += size
    return fields


def layout_size(mapping: Mapping) -> int:
    fields = layout_fields(mapping)
    if not fields:
        return 0
    last = fields[-1]
    return last.byte + (1 if last.bit is not None else last.size)


def layout_fingerprint(mapping: Mapping) -> int:
    """CRC-32 of the layout: every field's offset, type and name (padding
    counts by position only, so its numbering doesn't matter)."""
    lines = []
    for f in layout_fields(mapping):
        name = "pad" if f.is_padding else f.path
        lines.append(f"{f.address} {f.byte_type.value} {name}")
    lines.append(f"size {layout_size(mapping)}")
    return zlib.crc32("\n".join(lines).encode("utf-8")) & 0xFFFFFFFF


def describe(mapping: Mapping) -> str:
    """One line: size and fingerprint, for logs."""
    return f"{layout_size(mapping)} bytes, layout {layout_fingerprint(mapping):08X}"


def format_layout(mapping: Mapping, title: str = "") -> str:
    """The offset table: one row per field. Padding bits fold into one row
    per byte, and a component's run of indexed bits (a laser banner's
    beams) into one row."""
    rows = [f"{title}: {describe(mapping)}" if title else describe(mapping),
            f"  {'byte.bit':<11}{'type':<8}{'PLC':<7}field"]
    fields = layout_fields(mapping)
    i = 0
    while i < len(fields):
        f = fields[i]
        if f.bit is not None:
            j = i
            key = "pad" if f.is_padding else parse_path(f.path)[0]
            while (j + 1 < len(fields) and fields[j + 1].bit is not None
                   and ("pad" if fields[j + 1].is_padding else parse_path(fields[j + 1].path)[0]) == key
                   and (key != "pad" or fields[j + 1].byte == f.byte)):
                j += 1
            if j > i:
                last = fields[j]
                span = f"{f.address}-{last.address}" if last.byte != f.byte else f"{f.address}-{last.bit}"
                if key == "pad":
                    rows.append(f"  {span:<11}{'bit':<8}{'':<7}<padding>")
                else:
                    component, field, first_index = parse_path(f.path)
                    last_index = parse_path(last.path)[2]
                    rows.append(f"  {span:<11}{'bit':<8}{'BOOL':<7}{component}/{field}[{first_index}..{last_index}]"
                                f"  ({j - i + 1} bits)")
                i = j + 1
                continue
        name = "<padding>" if f.is_padding else f.path
        rows.append(f"  {f.address:<11}{f.byte_type.value:<8}{f.plc_type:<7}{name}")
        i += 1
    return "\n".join(rows)


# ---------------------------------------------------------------------------
# PLC data type
# ---------------------------------------------------------------------------

def _st_name(text: str) -> str:
    name = re.sub(r"[^A-Za-z0-9_]", "_", text)
    return name if not name[:1].isdigit() else f"_{name}"


def st_fields(mapping: Mapping) -> List[Tuple[str, str]]:
    """(name, ST type) per PLC STRUCT member, in an order whose PLC
    serialization (BOOLs first, then the rest as declared) is exactly this
    mapping's bytes:
      a single bit              -> <component> : BOOL
      a component's bit run     -> BYTE / WORD / DWORD / LWORD (8/16/32/64 bits
                                   with its padding), else ARRAY OF BYTE
      the clock                 -> clock : LINT
      repeated same-type values -> <component> : ARRAY[0..n-1] OF <type>
                                   (a belt's speed + directionAngle)
      other values              -> <component> : <type>, or
                                   <component>_<field> when it has several"""
    fields = [f for f in layout_fields(mapping)]
    # Group consecutive entries of one component (padding bits join the bit
    # run they complete).
    groups: List[Tuple[str, List[LayoutField]]] = []
    for f in fields:
        if f.is_padding:
            if groups and groups[-1][1][-1].bit is not None and f.bit is not None and f.bit != 0 \
                    and groups[-1][1][-1].byte == f.byte and len(groups[-1][1]) > 1:
                groups[-1][1].append(f)  # pads a multi-bit run to whole bytes
            continue
        component, field, index = parse_path(f.path)
        prev = groups[-1][1][-1] if groups else None
        if f.bit is not None:
            # Bits merge only as one field's indexed run (a banner's beams);
            # named bits (run, fault, ...) are BOOLs of their own.
            same_run = (prev is not None and groups[-1][0] == component and prev.bit is not None
                        and index is not None and parse_path(prev.path)[1:] == (field, index - 1))
        else:
            same_run = (prev is not None and groups[-1][0] == component and prev.bit is None
                        and prev.byte_type == f.byte_type)
        if same_run:
            groups[-1][1].append(f)
        else:
            groups.append((component, [f]))

    counts: Dict[str, int] = {}
    for component, _ in groups:
        counts[component] = counts.get(component, 0) + 1

    out: List[Tuple[str, str]] = []
    for component, members in groups:
        first = members[0]
        if component == "clock":
            out.append(("clock", "LINT"))
            continue
        name = _st_name(component)
        if counts[component] > 1:
            name = _st_name(f"{component}_{parse_path(first.path)[1]}")
        if first.bit is not None:
            if len(members) == 1:
                out.append((name, "BOOL"))
                continue
            nbytes = members[-1].byte - first.byte + 1
            out.append((name, _WHOLE_BYTE_TYPES.get(nbytes, f"ARRAY[0..{nbytes - 1}] OF BYTE")))
            continue
        plc = members[0].plc_type
        out.append((name, plc if len(members) == 1 else f"ARRAY[0..{len(members) - 1}] OF {plc}"))
    return out


def st_struct(type_name: str, mapping: Mapping) -> str:
    lines = [f"TYPE", f"    {_st_name(type_name)} : STRUCT"]
    lines += [f"        {name} : {st_type};" for name, st_type in st_fields(mapping)]
    lines += [f"    END_STRUCT;  // {describe(mapping)}", "END_TYPE"]
    return "\n".join(lines)


_ST_SIZES = {"BOOL": 0, "BYTE": 1, "WORD": 2, "DWORD": 4, "LWORD": 8, "LINT": 8, "REAL": 4}


def st_serialized_size(fields: List[Tuple[str, str]]) -> int:
    """Bytes the PLC serializer writes for these members (BOOLs packed into
    whole bytes first, then the rest) - to cross-check st_fields()."""
    bools = sum(1 for _, t in fields if t == "BOOL")
    size = (bools + 7) // 8
    for _, t in fields:
        m = re.match(r"ARRAY\[0\.\.(\d+)\] OF (\w+)", t)
        if m:
            size += (int(m.group(1)) + 1) * _ST_SIZES[m.group(2)]
        elif t != "BOOL":
            size += _ST_SIZES[t]
    return size


# ---------------------------------------------------------------------------
# Layout check between the simulator and a PLC (FrameType.LAYOUT_CHECK)
# ---------------------------------------------------------------------------

# input fingerprint, input size, output fingerprint, output size, status
_CHECK = "IHIHB"
CHECK_REQUEST, CHECK_MATCH, CHECK_MISMATCH = 0, 1, 2


def pack_layout_check(input_mapping: Mapping, output_mapping: Mapping, status: int = CHECK_REQUEST,
                      endianness: str = "<") -> bytes:
    return struct.pack(endianness + _CHECK, layout_fingerprint(input_mapping), layout_size(input_mapping),
                       layout_fingerprint(output_mapping), layout_size(output_mapping), status)


def unpack_layout_check(payload: bytes, endianness: str = "<") -> Optional[Tuple[int, int, int, int, int]]:
    if len(payload) != struct.calcsize(endianness + _CHECK):
        return None
    return struct.unpack(endianness + _CHECK, payload)


def compare_layout_check(mine: Tuple[int, int, int, int], theirs: Tuple[int, int, int, int]) -> List[str]:
    """Differences between two (in_crc, in_size, out_crc, out_size); [] if equal."""
    problems = []
    for label, (crc_a, size_a), (crc_b, size_b) in (
            ("input", mine[0:2], theirs[0:2]), ("output", mine[2:4], theirs[2:4])):
        if (crc_a, size_a) != (crc_b, size_b):
            problems.append(f"{label}: {size_a} bytes / layout {crc_a:08X} here, "
                            f"{size_b} bytes / layout {crc_b:08X} there")
    return problems


def main():
    from config_parser import load_config
    from mqtt_data_exchange import apply_exchange_config, io_topics
    from mqtt_message_binding import custom_component_mapping

    parser = argparse.ArgumentParser(description="Print a layout's I/O frame layouts (and PLC STRUCTs)")
    parser.add_argument("config", help="layout JSON")
    parser.add_argument("--exchange-config", help="JSON overriding exchange settings (endianness)")
    parser.add_argument("--st", action="store_true", help="also print the matching SIMATIC AX STRUCTs")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)
    publish, subscribe = io_topics(config)
    print(f"Byte order: {config.mqttSettings.endianness}")
    for title, topics, direction, type_name in (
            ("input  (simulator -> PLC)", publish, "out", "Sim_Input"),
            ("output (PLC -> simulator)", subscribe, "in", "Sim_Output")):
        for topic, topic_def in topics.items():
            mapping = custom_component_mapping(config, topic_def["custom_ids"], direction=direction)
            print()
            print(format_layout(mapping, title))
            if args.st:
                print()
                print(st_struct(type_name, mapping))


if __name__ == "__main__":
    main()
