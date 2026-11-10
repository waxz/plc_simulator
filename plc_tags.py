"""
plc_tags.py - the layout's PLC signals as PLC tags (no Qt): the data behind
the designer's tag table (io_table.py) and the tag names the AX generator
puts into the generated ST (st_export.py --tags).

Every PLC signal of the exchange contract (st_export.contract) is a tag: a
name, a tag table, a TIA data type and address, a comment. The address is
the signal's place in its frame in TIA notation: the frames the PLC reads
are inputs (%I, one after the other from byte 0), the frame it writes is
outputs (%Q). A run of indexed bits (a laser banner's beams) is one Array of
Bool tag. Your names / tables / comments are SimulatorConfig.plcTags (by
signal, "<component>/<signal>"); the others are generated.

A tag name follows TIA's rules (check_name). In the generated ST it must
also be an ST identifier - letters, digits and underscores, not starting
with a digit, not a keyword, unique ignoring case - so st_names() gives each
tag the name it has there: the same when it already is one, else a cleaned
copy (spaces and other characters to "_").
"""
import re
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import control_panel
import st_export
from config_parser import PlcTag

COLUMNS = ("Name", "Tag table", "Data type", "Address", "Comment", "Component", "Signal", "ST member")
NAME, TABLE, DATA_TYPE, ADDRESS, COMMENT, COMPONENT, SIGNAL, MEMBER = range(len(COLUMNS))
EDITABLE = (NAME, TABLE, COMMENT)
DEFAULT_TABLE = "Default tag table"
ALL_TAGS = "Show all tags"
# The frames the PLC exchanges I/O in, and their area seen from the PLC.
FRAMES = {"Sim_Input": "I", "Sim_Output": "Q", "Belt_Feedback": "I"}
INDEX = re.compile(r"\[(\d+)\]$")
# Contract type -> (TIA data type, size in bytes, address width letter).
TYPES = {"bit": ("Bool", 0, ""), "byte": ("Byte", 1, "B"), "uint16": ("UInt", 2, "W"), "uint32": ("UDInt", 4, "D"),
         "uint64": ("ULInt", 8, ""), "float": ("Real", 4, "D")}
MAX_NAME = 128


@dataclass
class Tag:
    key: str                # "<component>/<signal>": what an edit is kept by
    component: str
    signal: str
    area: str               # "I" (the PLC reads it) or "Q" (the PLC writes it)
    data_type: str          # TIA: Bool, Byte, UInt, UDInt, ULInt / LInt, Real, Array[0..n] of Bool
    byte: int               # in the PLC's area (frames of one area one after the other)
    bit: Optional[int]
    count: int = 1          # bits of an array
    member: str = ""        # the ST member, Sim_Input.<...>
    frame: str = ""
    name: str = ""
    table: str = DEFAULT_TABLE
    comment: str = ""

    @property
    def is_input(self) -> bool:
        return self.area == "I"

    @property
    def size(self) -> int:
        for tia, size, _w in TYPES.values():
            if tia == self.data_type:
                return size
        return 8 if self.data_type == "LInt" else 0

    @property
    def address(self) -> str:
        a = self.area
        if self.data_type == "Bool":
            return f"%{a}{self.byte}.{self.bit}"
        if self.count > 1:                          # an array of bits
            last = self.byte * 8 + self.bit + self.count - 1
            return f"%{a}{self.byte}.{self.bit}..%{a}{last // 8}.{last % 8}"
        width = {1: "B", 2: "W", 4: "D"}.get(self.size)
        if width is None:                           # 64 bits: no I/O address width in TIA
            return f"%{a}B{self.byte}..%{a}B{self.byte + self.size - 1}"
        return f"%{a}{width}{self.byte}"

    def cells(self) -> List[str]:
        return [self.name, self.table, self.data_type, self.address, self.comment, self.component, self.signal,
                self.member]


def _labels(config) -> Dict[str, str]:
    labels = {}
    for attr in ("belts", "sensors", "sources", "sinks", "buttons", "leds", "pallets", "stoppers", "robots",
                 "elevators", "scanners", "diverters", "turntables", "accumulators", "transfers", "doors",
                 "safety", "controlPanels"):
        for model in getattr(config, attr, []):
            label = getattr(model, "label", "")
            if label:
                labels.setdefault(model.id, str(label))
    return labels


def tags_of(config) -> Tuple[List[Tag], str]:
    """(tags, summary) for a layout: every PLC signal as a tag, with your
    names / tables / comments (config.plcTags) or the generated ones, and a
    one-line summary of the frames (sizes, fingerprints)."""
    control_panel.refresh(config)
    contract = st_export.contract(config)
    tags: List[Tag] = []
    parts = []
    base = {"I": 0, "Q": 0}
    for frame in contract["frames"]:
        area = FRAMES.get(frame["name"])
        if area is None:
            continue                                    # metric layouts: not I/O
        offset = base[area]
        base[area] += frame["size"]
        parts.append(f"{frame['name']} {frame['size']} B ({frame['fingerprint']:08X}) at %{area}{offset}")
        arrays: Dict[tuple, Tag] = {}
        for field in frame["fields"]:
            component, _sep, signal = field["path"].partition("/")
            tia, _size, _w = TYPES.get(field["type"], (field["type"], 0, ""))
            if component == "clock":
                tia = "LInt"
            member = field["member"]
            signal_base = INDEX.sub("", signal)
            byte, bit = offset + field["byte"], field["bit"]
            # An array in the PLC (its ST member is indexed, e.g. a laser
            # banner's beams): one tag for the whole run of bits. A single
            # bit (a button's sensor_states[0]) is a plain Bool.
            indexed = INDEX.search(member) if field["type"] == "bit" else None
            if indexed:
                key = (component, signal_base)
                tag = arrays.get(key)
                if tag is not None:
                    tag.count += 1
                    continue
                tag = arrays[key] = Tag(f"{component}/{signal_base}", component, signal_base, area, "Bool", byte,
                                        bit, member=f"{frame['name']}.{INDEX.sub('', member)}", frame=frame["name"])
                tags.append(tag)
                continue
            if field["type"] == "bit":
                signal = signal_base
            tags.append(Tag(f"{component}/{signal}", component, signal, area, tia, byte, bit,
                            member=f"{frame['name']}.{member}", frame=frame["name"]))
        for tag in arrays.values():
            tag.data_type = f"Array[0..{tag.count - 1}] of Bool"
            tag.signal = f"{tag.signal}[0..{tag.count - 1}]"
    _name(tags, config)
    return tags, "  ·  ".join(parts)


def _name(tags: List[Tag], config) -> None:
    """Your names / tables / comments, else the generated ones (unique)."""
    per_component: Dict[str, int] = {}
    for tag in tags:
        per_component[tag.component] = per_component.get(tag.component, 0) + 1
    labels = _labels(config)
    edits = {t.key: t for t in getattr(config, "plcTags", [])}
    taken = {e.name for e in edits.values() if e.name}
    for tag in tags:
        edit = edits.get(tag.key)
        tag.table = (edit.table if edit is not None and edit.table else DEFAULT_TABLE)
        default = "Simulated time (ns)" if tag.component == "clock" else labels.get(tag.component, "")
        tag.comment = edit.comment if edit is not None and edit.comment else default
        if edit is not None and edit.name:
            tag.name = edit.name
            continue
        signal = INDEX.sub("", tag.signal.split("[")[0])
        name = tag.component if per_component[tag.component] == 1 else f"{tag.component}_{signal}"
        unique, n = name, 2
        while unique in taken:
            unique, n = f"{name}_{n}", n + 1
        tag.name = unique
        taken.add(unique)


def rows_of(config) -> Tuple[List[list], str]:
    """(rows, summary): the tags as rows of strings in COLUMNS order."""
    tags, summary = tags_of(config)
    return [t.cells() for t in tags], summary


def check_name(name: str, tags: List[Tag], key: str) -> str:
    """"" when `name` can be the tag's name (TIA's rules: not empty, no
    quote, at most 128 characters, unique), else why not."""
    if not name.strip():
        return "A tag needs a name."
    if '"' in name:
        return "A tag name can't contain a double quote."
    if len(name) > MAX_NAME:
        return f"A tag name has at most {MAX_NAME} characters."
    if any(t.name == name and t.key != key for t in tags):
        return f"There is already a tag named '{name}'."
    return ""


def set_tag(config, key: str, edit: Optional[PlcTag]) -> None:
    """Your entry for one tag in the layout: `edit`, or none (generated)."""
    rest = [t for t in config.plcTags if t.key != key]
    config.plcTags = rest + ([edit] if edit is not None and (edit.name or edit.table or edit.comment) else [])


def tia_rows(tags: List[Tag]) -> Tuple[List[list], int]:
    """(rows, skipped): the "PLC Tags" sheet of TIA Portal's tag export,
    header first - arrays as one Bool per bit, 64-bit signals skipped."""
    rows = [["Name", "Path", "Data Type", "Logical Address", "Comment", "Hmi Visible", "Hmi Accessible",
             "Hmi Writeable", "Typeobject ID", "Version ID"]]
    skipped = 0
    for tag in tags:
        writeable = "False" if tag.is_input else "True"
        if tag.count > 1:
            for i in range(tag.count):
                bit = tag.byte * 8 + tag.bit + i
                rows.append([f"{tag.name}_{i}", tag.table, "Bool", f"%{tag.area}{bit // 8}.{bit % 8}",
                             f"{tag.comment} [{i}]".strip(), "True", "True", writeable, "", ""])
        elif tag.size == 8:
            skipped += 1
        else:
            rows.append([tag.name, tag.table, tag.data_type, tag.address, tag.comment, "True", "True", writeable,
                         "", ""])
    return rows, skipped


ST_IDENTIFIER = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
# IEC 61131-3 / AX ST words a tag can't be called (compared ignoring case).
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


def st_names(tags: List["Tag"]) -> Dict[str, str]:
    """key -> the tag's name in the generated ST: st_name, unique ignoring
    case (ST identifiers are not case-sensitive) with _2, _3, ..."""
    taken, names = set(), {}
    for tag in tags:
        base = st_name(tag.name)
        name, n = base, 2
        while name.lower() in taken:
            name, n = f"{base}_{n}", n + 1
        taken.add(name.lower())
        names[tag.key] = name
    return names


def tags_json(config) -> List[dict]:
    """The tags as data for the AX generator (st_export.py --tags): per tag
    its key, name, ST name, table, comment, TIA type and address, frame, the
    frame member it is (without the frame: "div_1.present",
    "sensor_1_2_1") and whether the PLC reads it."""
    tags, _ = tags_of(config)
    names = st_names(tags)
    return [{"key": t.key, "name": t.name, "stName": names[t.key], "table": t.table, "comment": t.comment,
             "dataType": t.data_type, "address": t.address, "frame": t.frame,
             "member": t.member[len(t.frame) + 1:], "input": t.is_input} for t in tags]
