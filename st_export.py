"""
st_export.py - the simulator <-> PLC exchange contract, from the one place it
is defined: this simulator's layout (io_topics / device_kinds / metric
layouts / FrameType) - and its SIMATIC AX ST, written by ax_export.py (the
one ST generator; the designer's File > Export to SIMATIC AX uses the same).

The export settings are saved in the layout (SimulatorConfig.axExport): set
them once, then a bare `st_export.py layout.json` writes the same file.

    python st_export.py layout.json -o <AX>/src/demo/sim_exchange/sim_io.st \
        --namespace SimDemo.Io --frames Sim_Input,Sim_Output --save   # export, and keep the settings
    python st_export.py layout.json                                    # export as saved
    python st_export.py layout.json --check                            # CI: exit 1 if out of date
    python st_export.py layout.json --contract c.json                  # the contract (+ tags) as data
    python st_export.py --from-contract c.json -o sim_io.st            # the ST from a saved contract

contract(): every frame's STRUCT members and every field's byte, bit, type,
path and member expression, its size and layout fingerprint (the CRC
io_layout / LAYOUT_CHECK use), the frame types and framing - what the ST,
the I/O table and the PLC tags are built from. pack() / sample_values() are
the golden packer the tests hold against the simulator's ComponentMessage.
"""
import argparse
import json
import os
import re
import struct
import sys
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import io_layout
from config_parser import SimulatorConfig, load_config
from config_parser import Belt
from device_kinds import kind_of
from mqtt_message_binding import (ByteType, belt_feedback_mapping, custom_component_mapping, find_component_by_id,
                                  parse_path)
from tcp_framed_transport import FrameType, TcpFrameCodec

ST_TYPES = {ByteType.BIT: "BOOL", ByteType.BYTE: "BYTE", ByteType.UINT16: "UINT", ByteType.UINT32: "UDINT",
            ByteType.UINT64: "ULINT", ByteType.FLOAT: "REAL"}
PACK = {ByteType.BYTE: "B", ByteType.UINT16: "H", ByteType.UINT32: "I", ByteType.UINT64: "Q", ByteType.FLOAT: "f"}
SIZES = {ByteType.BYTE: 1, ByteType.UINT16: 2, ByteType.UINT32: 4, ByteType.UINT64: 8, ByteType.FLOAT: 4}


# ---------------------------------------------------------------------------
# The contract
# ---------------------------------------------------------------------------

def frames(config: SimulatorConfig) -> List[Tuple[str, int, list]]:
    """(STRUCT name, frame type, mapping) of every frame this layout exchanges."""
    from mqtt_data_exchange import METRICS_CHANNELS, io_topics
    from metric_layout import layout_mapping
    pub, sub = io_topics(config)
    out = [("Sim_Input", int(FrameType.IO_MAPPING),
            custom_component_mapping(config, next(iter(pub.values()))["custom_ids"], direction="out")),
           ("Sim_Output", int(FrameType.IO_MAPPING),
            custom_component_mapping(config, next(iter(sub.values()))["custom_ids"], direction="in"))]
    if config.mqttSettings.beltFeedback:
        out.append(("Belt_Feedback", int(FrameType.BELT_FEEDBACK), belt_feedback_mapping(config)))
    for frame_type, channel in METRICS_CHANNELS.items():
        out.append((_metric_name(FrameType(frame_type).name), int(frame_type), layout_mapping(channel["layout"])))
    return out


# Which side of the exchange each frame carries: "out" = simulator -> PLC
# (feedback), "in" = PLC -> simulator (commands). Metric frames: neither.
FRAME_DIRECTIONS = {"Sim_Input": "out", "Sim_Output": "in", "Belt_Feedback": "out"}


def device_type(component, direction: Optional[str]) -> Optional[str]:
    """The STRUCT a component is in a frame of this direction, or None (a
    plain value: a sensor, a button, a lamp, the clock)."""
    if direction is None:
        return None
    side = "Feedback" if direction == "out" else "Command"
    kind = kind_of(component)
    if kind is not None:
        return f"{kind.st_name}_{side}"
    if isinstance(component, Belt):
        if component.commandFormat == "frequency":
            return f"BeltFreq_{side}"
        if side == "Command" and component.commandFormat == "vfd":
            return "BeltVfd_Command"
        return f"BeltDrive_{side}"
    return None


def _component(config: Optional[SimulatorConfig], component_id: str):
    if config is None or component_id == "clock" or component_id.startswith("padding"):
        return None
    try:
        return find_component_by_id(component_id, config)
    except ValueError:
        return None


def _metric_name(enum_name: str) -> str:
    return "_".join(part.capitalize() for part in enum_name.lower().split("_"))


ST_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
# IEC 61131-3 / AX ST words no generated name may be (compared ignoring case).
ST_KEYWORDS = {k.lower() for k in """
    ABSTRACT AND ARRAY AT BOOL BY BYTE CASE CHAR CLASS CONFIGURATION CONSTANT DATE DATE_AND_TIME DINT DO DT
    DWORD ELSE ELSIF END_CASE END_CLASS END_CONFIGURATION END_FOR END_FUNCTION END_FUNCTION_BLOCK END_IF
    END_INTERFACE END_METHOD END_NAMESPACE END_PROGRAM END_REPEAT END_STRUCT END_TYPE END_VAR END_WHILE EXIT
    EXTENDS FALSE FINAL FOR FUNCTION FUNCTION_BLOCK IF IMPLEMENTS INT INTERFACE INTERNAL LDATE LDT LINT LREAL
    LTIME LTOD LWORD METHOD MOD NAMESPACE NOT NULL OF OR OVERRIDE PRIVATE PROGRAM PROTECTED PUBLIC READ_ONLY
    READ_WRITE REAL REF REF_TO REPEAT RETAIN RETURN SINT STRING STRUCT SUPER TASK THEN THIS TIME TIME_OF_DAY TO
    TOD TRUE TYPE UDINT UINT ULINT UNTIL USINT USING VAR VAR_ACCESS VAR_CONFIG VAR_EXTERNAL VAR_GLOBAL VAR_INPUT
    VAR_IN_OUT VAR_OUTPUT VAR_STAT VAR_TEMP WCHAR WHILE WORD WSTRING XOR
""".split()}


def st_name(name: str) -> str:
    """`name` as an ST identifier: other characters to "_", "_" before a
    leading digit, "_" after a keyword."""
    clean = re.sub(r"[^A-Za-z0-9_]", "_", name.strip()) or "_"
    if clean[0].isdigit():
        clean = "_" + clean
    if clean.lower() in ST_KEYWORDS:
        clean += "_"
    return clean


_st_name = st_name


class _Names:
    """ST names for a frame's members: st_name, unique ignoring case (ST
    identifiers are not case-sensitive) with _2, _3, ... - two components
    whose ids clean to the same name ("belt-1", "belt_1") get two members,
    never one."""

    def __init__(self):
        self.names: Dict[str, str] = {}
        self.taken: set = set()

    def __call__(self, key: str, text: str) -> str:
        if key not in self.names:
            base = st_name(text)
            name, n = base, 2
            while name.lower() in self.taken:
                name, n = f"{base}_{n}", n + 1
            self.taken.add(name.lower())
            self.names[key] = name
        return self.names[key]


class Member:
    """One STRUCT member: a scalar, or an array of indexed entries."""

    def __init__(self, name: str, st_type: str, length: Optional[int] = None):
        self.name, self.st_type, self.length = name, st_type, length

    @property
    def declaration(self) -> str:
        if self.length is None:
            return f"{self.name} : {self.st_type};"
        return f"{self.name} : ARRAY[0..{self.length - 1}] OF {self.st_type};"


def members(mapping, config: Optional[SimulatorConfig] = None, direction: Optional[str] = None
            ) -> Tuple[List[Member], List[Tuple[io_layout.LayoutField, str]]]:
    """The STRUCT members of a frame and, per non-padding field, the ST
    expression (relative to `value.`) that holds it. With the layout and
    the frame's direction, a device or belt is one member of its kind's
    STRUCT (device_type) and its fields are reached through it."""
    fields = [f for f in io_layout.layout_fields(mapping) if not f.is_padding]
    grouped: Dict[str, str] = {}
    for f in fields:
        component = parse_path(f.path)[0]
        if component not in grouped:
            grouped[component] = device_type(_component(config, component), direction)
    per_component: Dict[str, set] = {}
    indexed: Dict[Tuple[str, str], int] = {}
    for f in fields:
        component, field, index = parse_path(f.path)
        per_component.setdefault(component, set()).add(field)
        if index is not None:
            indexed[(component, field)] = max(indexed.get((component, field), 0), index + 1)
    out: List[Member] = []
    access: List[Tuple[io_layout.LayoutField, str]] = []
    declared: Dict[str, Member] = {}
    unique = _Names()
    for f in fields:
        component, field, index = parse_path(f.path)
        if grouped.get(component):
            name = unique(component, component)
            if name not in declared:
                declared[name] = Member(name, grouped[component])
                out.append(declared[name])
            access.append((f, f"{name}.{_st_name(field)}"))
            continue
        if component == "clock":
            name, st_type = unique("clock", "clock"), "LINT"
        elif len(per_component[component]) == 1:
            name, st_type = unique(component, component), ST_TYPES[f.byte_type]
        else:
            name, st_type = unique(f"{component}/{field}", f"{component}_{field}"), ST_TYPES[f.byte_type]
        length = indexed.get((component, field))
        array = length is not None and length > 1
        if name not in declared:
            declared[name] = Member(name, st_type, length if array else None)
            out.append(declared[name])
        access.append((f, f"{name}[{index}]" if array else name))
    return out, access


def device_structs(config: SimulatorConfig) -> Dict[str, List[Tuple[str, str]]]:
    """The device STRUCTs the frames use: type name -> [(member, ST type)],
    in wire order (bits first). Every component of a kind has the same
    fields, so one STRUCT serves them all."""
    structs: Dict[str, List[Tuple[str, str]]] = {}
    for name, _frame_type, mapping in frames(config):
        direction = FRAME_DIRECTIONS.get(name)
        seen: Dict[str, List[Tuple[str, str]]] = {}
        for f in io_layout.layout_fields(mapping):
            if f.is_padding:
                continue
            component_id, field, _index = parse_path(f.path)
            type_name = device_type(_component(config, component_id), direction)
            if type_name:
                seen.setdefault(component_id, []).append((_st_name(field), ST_TYPES[f.byte_type]))
        for component_id, fields in seen.items():
            type_name = device_type(_component(config, component_id), direction)
            if structs.setdefault(type_name, fields) != fields:
                raise ValueError(f"{component_id}: its fields differ from the other {type_name}s")
    return dict(sorted(structs.items()))


def contract(config: SimulatorConfig, source: str = "") -> dict:
    endian = config.mqttSettings.endianness
    out = {"source": source, "endianness": endian,
           "lengthEndianness": config.mqttSettings.tcpLengthEndianness or endian,
           "magic": TcpFrameCodec.MAGIC_MARKER_INT, "headerSize": TcpFrameCodec.HEADER_SIZE,
           "trailerSize": TcpFrameCodec.TRAILER_SIZE,
           "frameTypes": {m.name: int(m) for m in FrameType},
           "structs": {name: [{"name": n, "type": t} for n, t in fields]
                       for name, fields in device_structs(config).items()},
           "frames": []}
    for name, frame_type, mapping in frames(config):
        mem, access = members(mapping, config, FRAME_DIRECTIONS.get(name))
        out["frames"].append({
            "name": name, "frameType": frame_type, "size": io_layout.layout_size(mapping),
            "fingerprint": io_layout.layout_fingerprint(mapping),
            "members": [{"name": m.name, "type": m.st_type, "length": m.length} for m in mem],
            "fields": [{"byte": f.byte, "bit": f.bit, "type": f.byte_type.value, "path": f.path, "member": member}
                       for f, member in access]})
    text = json.dumps(out, sort_keys=True)
    out["fingerprint"] = zlib.crc32(text.encode("utf-8")) & 0xFFFFFFFF
    return out


# ---------------------------------------------------------------------------
# Packing (the Python side of the golden vectors)
# ---------------------------------------------------------------------------

def sample_values(access) -> Dict[str, object]:
    """Deterministic, exactly representable test values per member access."""
    values = {}
    for i, (f, expr) in enumerate(access):
        if f.byte_type == ByteType.BIT:
            values[expr] = (i * 7 + 3) % 3 != 0
        elif f.byte_type == ByteType.FLOAT:
            values[expr] = (i % 17) * 1.25 - 4.5
        elif expr == "clock":
            values[expr] = 1_700_000_000_123_456_789 + i
        else:
            bits = SIZES[f.byte_type] * 8
            values[expr] = (i * 2654435761 + 97) % (1 << min(bits, 31))
    return values


def pack(size: int, access, values: Dict[str, object], endian: str) -> bytes:
    """The bytes the Python packer (ComponentMessage) puts on the wire for
    these values - written from the same layout_fields offsets."""
    data = bytearray(size)
    prefix = "<" if endian == "LE" else ">"
    for f, expr in access:
        v = values[expr]
        if f.byte_type == ByteType.BIT:
            if v:
                data[f.byte] |= 1 << f.bit
        else:
            fmt = "q" if expr == "clock" else PACK[f.byte_type]
            struct.pack_into(prefix + fmt, data, f.byte, v)
    return bytes(data)


def unpack(data: bytes, access, endian: str) -> Dict[str, object]:
    prefix = "<" if endian == "LE" else ">"
    values = {}
    for f, expr in access:
        if f.byte_type == ByteType.BIT:
            values[expr] = bool(data[f.byte] >> f.bit & 1)
        else:
            fmt = "q" if expr == "clock" else PACK[f.byte_type]
            values[expr] = struct.unpack_from(prefix + fmt, data, f.byte)[0]
    return values


# ---------------------------------------------------------------------------
# ST emitters
# ---------------------------------------------------------------------------

def _settings(config: Optional[SimulatorConfig], args):
    """The layout's saved export settings with the command line's on top
    (paths from the command line are relative to the current folder)."""
    from config_parser import AxExport
    settings = (config.axExport.model_copy() if config is not None and config.axExport is not None
                else AxExport())
    if args.output:
        settings.output = os.path.abspath(args.output)
    if args.vectors:
        settings.vectors = os.path.abspath(args.vectors)
    for name in ("namespace", "prefix"):
        if getattr(args, name) is not None:
            setattr(settings, name, getattr(args, name))
    if args.frames is not None:
        settings.frames = [f.strip() for f in args.frames.split(",") if f.strip()]
    if args.no_tags:
        settings.tags = False
    if args.bit_arrays:
        settings.bitArrays = True
    if args.max_payload is not None:
        settings.maxPayload = args.max_payload
    return settings


def save_settings(layout_path: str, settings) -> None:
    """The settings into the layout file - only its axExport, paths made
    relative to it; everything else in the file as it is."""
    with open(layout_path, encoding="utf-8") as f:
        data = json.load(f)
    saved = settings.model_dump()
    folder = os.path.dirname(os.path.abspath(layout_path))
    for key in ("output", "vectors"):
        if saved[key] and os.path.isabs(saved[key]):
            saved[key] = os.path.relpath(saved[key], folder).replace("\\", "/")
    data["axExport"] = saved
    with open(layout_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def main(argv: Optional[Sequence[str]] = None) -> int:
    import ax_export
    parser = argparse.ArgumentParser(description="The simulator <-> PLC exchange as SIMATIC AX ST (ax_export.py), "
                                                 "and its contract as data")
    parser.add_argument("config", nargs="?", help="layout JSON (its saved axExport settings apply)")
    parser.add_argument("-o", "--output", help="the .st file")
    parser.add_argument("--namespace", help="NAMESPACE around the ST")
    parser.add_argument("--prefix", help="STRUCT name prefix: Station_1 names Sim_Input Station_1_Input")
    parser.add_argument("--frames", help="comma-separated frames (default: every frame)")
    parser.add_argument("--no-tags", action="store_true", help="leave out the PLC tags STRUCT and ReadTags / WriteTags")
    parser.add_argument("--bit-arrays", action="store_true", help="a laser banner's beams as ARRAY OF BOOL")
    parser.add_argument("--max-payload", type=int, help="refuse a bigger frame (default 242); 0 = no limit")
    parser.add_argument("--vectors", help="also write an AxUnit golden-vector test file")
    parser.add_argument("--exchange-config", help="JSON overriding exchange settings (endianness, beltFeedback)")
    parser.add_argument("--from-contract", help="the ST from a saved contract (--contract) instead of a layout")
    parser.add_argument("--contract", help="also write the contract (with the PLC tags) as JSON here")
    parser.add_argument("--tags", help="also write the PLC tags as JSON here (plc_tags.tags_json)")
    parser.add_argument("--save", action="store_true", help="keep these settings in the layout file")
    parser.add_argument("--check", action="store_true", help="write nothing; exit 1 if a file is out of date")
    args = parser.parse_args(argv)
    if not args.config and not args.from_contract:
        parser.error("give a layout, or --from-contract")

    config = None
    if args.from_contract:
        with open(args.from_contract, encoding="utf-8") as f:
            c = json.load(f)
        tags = c.pop("plcTags", None)
        source = c.get("source") or os.path.basename(args.from_contract)
    else:
        config = load_config(args.config)
        if args.exchange_config:
            from mqtt_data_exchange import apply_exchange_config
            apply_exchange_config(config, args.exchange_config)
        source = os.path.basename(args.config)
        c = contract(config, source)
        import plc_tags
        tags = plc_tags.tags_json(config)
    settings = _settings(config, args)

    files = {}
    if args.contract:
        files[args.contract] = json.dumps({**c, "plcTags": tags} if tags is not None else c, indent=2) + "\n"
    if args.tags and tags is not None:
        files[args.tags] = json.dumps(tags, indent=2) + "\n"
    report = []
    if settings.output:
        try:
            result = ax_export.export(c, settings, tags, args.config)
        except ax_export.ExportError as exc:
            print(f"st_export: {exc}")
            return 1
        files.update(result.files)
        report = result.report
    elif not files:
        parser.error("nothing to write: give -o (or save it in the layout with --save), --contract or --tags")
    if args.save and args.config:
        save_settings(args.config, settings)

    stale = ax_export.write(ax_export.Export(files, report), check=args.check)
    sizes = ", ".join(report)
    if args.check:
        if stale:
            print(f"st_export: {len(stale)} file(s) out of date with {source} (contract {c['fingerprint']:08X}):")
            for path in stale:
                print(f"  {path}")
            return 1
        print(f"st_export: up to date (contract {c['fingerprint']:08X}){' - ' + sizes if sizes else ''}")
        return 0
    print(f"st_export: contract {c['fingerprint']:08X} - wrote {len(stale)} of {len(files)} file(s)"
          f"{' - ' + sizes if sizes else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
