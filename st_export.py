"""
st_export.py - the simulator <-> PLC exchange contract, generated for the
SIMATIC AX project from the one place it is defined: this simulator's
layout (io_topics / MACHINE_FIELDS / metric layouts / FrameType).

    python st_export.py layout.json --out <AX>/src/generated --contract <AX>/contract/io_contract.json
    python st_export.py layout.json --out <AX>/src/generated --check     # CI: exit 1 if out of date

Writes, in the namespace `--namespace` (default Simulator.Io):

  SimIoTypes.st      one STRUCT per frame: Sim_Input (simulator -> PLC),
                     Sim_Output (PLC -> simulator), Belt_Feedback (when the
                     layout sends it) and the metric layouts - a member per
                     signal, named after the component (and field)
  SimIoSerde.st      DeserializeFromBytes / SerializeToBytes overloads per
                     STRUCT, written from the byte / bit offsets of the
                     Python packer (io_layout.layout_fields) - not re-derived
                     from member order - in the layout's byte order
                     (System.Serialization, Endianness#Little / #Big)
  SimIoContract.st   CLASS SimIoContract: the frame magic, header / trailer
                     sizes, every FrameType value, each frame's size and
                     layout fingerprint (the CRC io_layout / LAYOUT_CHECK use)
  SimIoVectorsTest.st   an AxUnit test per frame: sample values serialize to
                     exactly the bytes the Python packer produces, and back
  io_contract.json   (--contract) the same contract as data: every field's
                     byte, bit, type, path and member name

Every file starts with the contract's fingerprint; --check regenerates in
memory and reports any file that differs (or is missing).
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
from mqtt_message_binding import ByteType, belt_feedback_mapping, custom_component_mapping, parse_path
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


def _metric_name(enum_name: str) -> str:
    return "_".join(part.capitalize() for part in enum_name.lower().split("_"))


def _st_name(text: str) -> str:
    name = re.sub(r"[^A-Za-z0-9_]", "_", text)
    return name if not name[:1].isdigit() else f"_{name}"


class Member:
    """One STRUCT member: a scalar, or an array of indexed entries."""

    def __init__(self, name: str, st_type: str, length: Optional[int] = None):
        self.name, self.st_type, self.length = name, st_type, length

    @property
    def declaration(self) -> str:
        if self.length is None:
            return f"{self.name} : {self.st_type};"
        return f"{self.name} : ARRAY[0..{self.length - 1}] OF {self.st_type};"


def members(mapping) -> Tuple[List[Member], List[Tuple[io_layout.LayoutField, str]]]:
    """The STRUCT members of a frame and, per non-padding field, the ST
    expression (relative to `value.`) that holds it."""
    fields = [f for f in io_layout.layout_fields(mapping) if not f.is_padding]
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
    for f in fields:
        component, field, index = parse_path(f.path)
        if component == "clock":
            name, st_type = "clock", "LINT"
        else:
            name = _st_name(component if len(per_component[component]) == 1 else f"{component}_{field}")
            st_type = ST_TYPES[f.byte_type]
        length = indexed.get((component, field))
        array = length is not None and length > 1
        if name not in declared:
            declared[name] = Member(name, st_type, length if array else None)
            out.append(declared[name])
        access.append((f, f"{name}[{index}]" if array else name))
    return out, access


def contract(config: SimulatorConfig, source: str = "") -> dict:
    endian = config.mqttSettings.endianness
    out = {"source": source, "endianness": endian,
           "lengthEndianness": config.mqttSettings.tcpLengthEndianness or endian,
           "magic": TcpFrameCodec.MAGIC_MARKER_INT, "headerSize": TcpFrameCodec.HEADER_SIZE,
           "trailerSize": TcpFrameCodec.TRAILER_SIZE,
           "frameTypes": {m.name: int(m) for m in FrameType}, "frames": []}
    for name, frame_type, mapping in frames(config):
        _members, access = members(mapping)
        out["frames"].append({
            "name": name, "frameType": frame_type, "size": io_layout.layout_size(mapping),
            "fingerprint": io_layout.layout_fingerprint(mapping),
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

def _header(c: dict, what: str) -> List[str]:
    return ["// " + "-" * 77,
            "// <auto-generated>",
            f"// {what}",
            f"// Generated by st_export.py (conveyor simulator) from {c['source'] or 'the layout'}.",
            f"// Contract fingerprint {c['fingerprint']:08X}. Do not edit by hand: regenerate.",
            "// " + "-" * 77]


def emit_types(c: dict, config: SimulatorConfig, namespace: str) -> str:
    lines = _header(c, "The simulator <-> PLC frames as STRUCTs.") + [f"NAMESPACE {namespace}", "    TYPE"]
    for name, frame_type, mapping in frames(config):
        mem, _access = members(mapping)
        lines.append(f"        // frame type {frame_type}, {io_layout.layout_size(mapping)} bytes, "
                     f"fingerprint {io_layout.layout_fingerprint(mapping):08X}")
        lines.append(f"        {name} : STRUCT")
        lines += [f"            {m.declaration}" for m in mem]
        lines.append("        END_STRUCT;")
    lines += ["    END_TYPE", "END_NAMESPACE", ""]
    return "\n".join(lines)


def _endianness(c: dict) -> str:
    return "Endianness#Little" if c["endianness"] == "LE" else "Endianness#Big"


def emit_serde(c: dict, config: SimulatorConfig, namespace: str) -> str:
    endian = _endianness(c)
    lines = _header(c, "Serializers written from the Python packer's byte / bit offsets.")
    lines += ["USING System.Serialization;", "USING System.BitAccess;", "", f"NAMESPACE {namespace}"]
    for name, _frame_type, mapping in frames(config):
        size = io_layout.layout_size(mapping)
        _mem, access = members(mapping)
        bit_bytes = sorted({f.byte for f, _ in access if f.byte_type == ByteType.BIT})
        layout = [f"    //   {expr} -> byte {f.byte}" + (f", bit {f.bit}" if f.bit is not None else
                                                          f"..{f.byte + SIZES.get(f.byte_type, 8) - 1}")
                  for f, expr in access]
        for direction in ("Deserialize", "Serialize"):
            fn = "DeserializeFromBytes" if direction == "Deserialize" else "SerializeToBytes"
            buf = "source" if direction == "Deserialize" else "dest"
            lines += ["", f"    // {name}: {size} bytes. Layout (relative to startOffset):"] + layout
            lines += [f"    FUNCTION {fn} : UDINT",
                      "        VAR_INPUT",
                      "            startOffset : UDINT := UDINT#0;",
                      "        END_VAR",
                      "        VAR_IN_OUT"]
            if direction == "Deserialize":
                lines += [f"            source : ARRAY[*] OF BYTE;", f"            value : {name};"]
            else:
                lines += [f"            value : {name};", f"            dest : ARRAY[*] OF BYTE;"]
            lines += ["        END_VAR",
                      "        VAR_TEMP",
                      "            offset : UDINT;",
                      "            byteIdx : DINT;",
                      "        END_VAR",
                      f"        byteIdx := LOWER_BOUND({buf}, 1) + TO_DINT(startOffset);"]
            if direction == "Serialize":
                lines += [f"        dest[byteIdx + DINT#{b}] := BYTE#16#00;" for b in bit_bytes]
            for f, expr in access:
                if f.byte_type == ByteType.BIT:
                    if direction == "Deserialize":
                        lines.append(f"        value.{expr} := GetBit(source[byteIdx + DINT#{f.byte}], USINT#{f.bit});")
                    else:
                        lines.append(f"        dest[byteIdx + DINT#{f.byte}] := SetBit(value := dest[byteIdx + "
                                     f"DINT#{f.byte}], index := USINT#{f.bit}, bitValue := value.{expr});")
                elif direction == "Deserialize":
                    lines.append(f"        offset := Deserialize(startOffset + UDINT#{f.byte}, source, value.{expr}, "
                                 f"{endian});")
                else:
                    lines.append(f"        offset := Serialize(startOffset + UDINT#{f.byte}, value.{expr}, dest, "
                                 f"{endian});")
            lines += [f"        {fn} := startOffset + UDINT#{size};", "    END_FUNCTION"]
    lines += ["END_NAMESPACE", ""]
    return "\n".join(lines)


def emit_contract_class(c: dict, namespace: str) -> str:
    lines = _header(c, "The exchange's constants: framing, frame types, sizes, fingerprints.")
    lines += [f"NAMESPACE {namespace}",
              "    CLASS PUBLIC SimIoContract",
              "        VAR CONSTANT PUBLIC",
              f"            CONTRACT_FINGERPRINT : DWORD := DWORD#16#{c['fingerprint']:08X};",
              f"            FRAME_MAGIC : DWORD := DWORD#16#{c['magic']:08X};",
              f"            HEADER_SIZE : UDINT := UDINT#{c['headerSize']};",
              f"            TRAILER_SIZE : UDINT := UDINT#{c['trailerSize']};",
              f"            LITTLE_ENDIAN : BOOL := {'TRUE' if c['endianness'] == 'LE' else 'FALSE'};",
              f"            LENGTH_LITTLE_ENDIAN : BOOL := {'TRUE' if c['lengthEndianness'] == 'LE' else 'FALSE'};"]
    for name, value in c["frameTypes"].items():
        lines.append(f"            FRAME_TYPE_{_st_name(name).upper()} : BYTE := BYTE#{value};")
    for frame in c["frames"]:
        n = frame["name"].upper()
        lines.append(f"            {n}_SIZE : UDINT := UDINT#{frame['size']};")
        lines.append(f"            {n}_FINGERPRINT : DWORD := DWORD#16#{frame['fingerprint']:08X};")
    lines += ["        END_VAR", "    END_CLASS", "END_NAMESPACE", ""]
    return "\n".join(lines)


def _st_literal(value, st_type: str) -> str:
    if st_type == "BOOL":
        return "TRUE" if value else "FALSE"
    if st_type == "REAL":
        return f"REAL#{value!r}"
    return f"{st_type}#{value}"


def emit_vectors(c: dict, config: SimulatorConfig, namespace: str) -> str:
    """AxUnit tests: the generated serializers reproduce the Python bytes."""
    lines = _header(c, "Golden vectors: the PLC serializers must produce the Python packer's bytes.")
    lines += ["USING AxUnit.Assert;", f"USING {namespace};", "", f"NAMESPACE {namespace}.Tests",
              "    {TestFixture}", "    CLASS SimIoVectorsTest"]
    for name, _frame_type, mapping in frames(config):
        size = io_layout.layout_size(mapping)
        mem, access = members(mapping)
        types = {m.name: m.st_type for m in mem}
        values = sample_values(access)
        expected = pack(size, access, values, c["endianness"])
        byte_list = ", ".join(f"BYTE#16#{b:02X}" for b in expected)
        lines += ["        {Test}",
                  f"        METHOD PUBLIC {name}_matches_python",
                  "            VAR_TEMP",
                  f"                value : {name};",
                  f"                back : {name};",
                  f"                bytes : ARRAY[0..{size - 1}] OF BYTE;",
                  f"                expected : ARRAY[0..{size - 1}] OF BYTE := [{byte_list}];",
                  "                size : UDINT;",
                  "                i : DINT;",
                  "            END_VAR"]
        for f, expr in access:
            base = expr.split("[")[0]
            lines.append(f"            value.{expr} := {_st_literal(values[expr], types[base])};")
        lines += [f"            size := SerializeToBytes(UDINT#0, value, bytes);",
                  f"            Equal(size, UDINT#{size});",
                  f"            FOR i := 0 TO {size - 1} DO",
                  "                Equal(bytes[i], expected[i]);",
                  "            END_FOR;",
                  f"            size := DeserializeFromBytes(UDINT#0, expected, back);",
                  f"            Equal(size, UDINT#{size});"]
        for f, expr in access:
            lines.append(f"            Equal(back.{expr}, value.{expr});")
        lines.append("        END_METHOD")
    lines += ["    END_CLASS", "END_NAMESPACE", ""]
    return "\n".join(lines)


def generate(config: SimulatorConfig, source: str, namespace: str) -> Dict[str, str]:
    c = contract(config, source)
    return {"SimIoTypes.st": emit_types(c, config, namespace),
            "SimIoSerde.st": emit_serde(c, config, namespace),
            "SimIoContract.st": emit_contract_class(c, namespace),
            "SimIoVectorsTest.st": emit_vectors(c, config, namespace)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Generate the SIMATIC AX side of the simulator exchange")
    parser.add_argument("config", help="layout JSON")
    parser.add_argument("--exchange-config", help="JSON overriding exchange settings (endianness, beltFeedback)")
    parser.add_argument("--out", required=True, help="directory for the .st files (e.g. <AX>/src/generated)")
    parser.add_argument("--tests-out", help="directory for SimIoVectorsTest.st (default: --out)")
    parser.add_argument("--contract", help="also write the contract as JSON here")
    parser.add_argument("--namespace", default="Simulator.Io")
    parser.add_argument("--check", action="store_true", help="write nothing; exit 1 if any file is out of date")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    if args.exchange_config:
        from mqtt_data_exchange import apply_exchange_config
        apply_exchange_config(config, args.exchange_config)
    source = os.path.basename(args.config)
    files = {os.path.join(args.tests_out or args.out, name) if name.endswith("Test.st") else
             os.path.join(args.out, name): text for name, text in generate(config, source, args.namespace).items()}
    if args.contract:
        files[args.contract] = json.dumps(contract(config, source), indent=2) + "\n"

    stale = []
    for path, text in files.items():
        try:
            with open(path, encoding="utf-8") as f:
                current = f.read()
        except OSError:
            current = None
        if current != text:
            stale.append(path)
            if not args.check:
                os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
                with open(path, "w", encoding="utf-8", newline="\n") as f:
                    f.write(text)
    fingerprint = contract(config, source)["fingerprint"]
    if args.check:
        if stale:
            print(f"st_export: {len(stale)} file(s) out of date with {source} (contract {fingerprint:08X}):")
            for path in stale:
                print(f"  {path}")
            print("Regenerate: python st_export.py " + " ".join(a for a in (argv or sys.argv[1:]) if a != "--check"))
            return 1
        print(f"st_export: up to date (contract {fingerprint:08X})")
        return 0
    print(f"st_export: contract {fingerprint:08X} - wrote {len(stale)} of {len(files)} file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
