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
from id_utils import own_label

COLUMNS = ("Name", "Tag table", "Data type", "Address", "Comment", "Component", "Signal", "ST member")
NAME, TABLE, DATA_TYPE, ADDRESS, COMMENT, COMPONENT, SIGNAL, MEMBER = range(len(COLUMNS))
EDITABLE = (NAME, TABLE, COMMENT)
DEFAULT_TABLE = "Default tag table"
ALL_TAGS = "Show all tags"
# The frames the PLC exchanges I/O in, and their area seen from the PLC.
FRAMES = {"Sim_Input": "I", "Sim_Output": "Q"}
INDEX = re.compile(r"\[(\d+)\]$")
# Contract type -> (TIA data type, size in bytes, address width letter).
TYPES = {"bit": ("Bool", 0, ""), "byte": ("Byte", 1, "B"), "uint16": ("UInt", 2, "W"), "uint32": ("UDInt", 4, "D"),
         "uint64": ("ULInt", 8, ""), "float": ("Real", 4, "D")}
MAX_NAME = 128
# The unit of a signal's value, by its name, as the simulator uses it (and
# so the PLC must): lengths m, speeds m/s, angles DEGREES, times ns. Put in
# every generated tag comment, so a PLC programmer never has to guess (the
# AX network library works in um and radians: convert at the tags).
UNITS = {"latencyMs": "ms, simulated", "speed": "m/s", "commandSpeed": "m/s", "actualSpeed": "m/s", "frequency": "Hz", "outputFrequency": "Hz",
         "motorSpeed": "rpm", "encoderCount": "counts", "commandAngle": "deg to the travel, + left",
         "directionAngle": "deg", "targetYaw": "deg", "targetX": "m", "targetY": "m", "targetZ": "m",
         "tcpX": "m", "tcpY": "m", "tcpZ": "m", "j1": "deg", "j2": "deg", "j3": "deg", "j4": "deg", "j5": "deg",
         "j6": "deg", "speedOverride": "%", "weight": "kg", "measuredLength": "m", "measuredWidth": "m",
         "measuredHeight": "m", "clock": "ns",
         "plcHeartbeat": "counter: +1 every PLC scan", "simHeartbeat": "counter: +1 every simulation step",
         "plcAlive": "the simulator sees the PLC's heartbeat",
         "loadX": "m", "loadY": "m", "loadYaw": "deg", "loadLength": "m", "loadWidth": "m",
         "frame": "the simHeartbeat its values are of", "alongError": "m, PLC - parcel along the travel",
         "lateralError": "m, PLC - parcel across, + left"}


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
                 "safety", "controlPanels", "plcValues", "watchdogs", "trackedLoads"):
        for model in getattr(config, attr, []):
            label = own_label(model)             # a label repeating the id is no comment
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
    value_units = {v.id: v.unit for v in getattr(config, "plcValues", [])}   # a PLC value: its own unit
    edits = {t.key: t for t in getattr(config, "plcTags", [])}
    taken = {e.name for e in edits.values() if e.name}
    for tag in tags:
        edit = edits.get(tag.key)
        tag.table = (edit.table if edit is not None and edit.table else DEFAULT_TABLE)
        if tag.component == "clock":
            default = "Simulator PC clock, wall time [ns]"        # clock/nanoseconds: time.time_ns(), not sim time
        else:
            unit = value_units.get(tag.component) or UNITS.get(INDEX.sub("", tag.signal.split("[")[0]))
            if tag.component in value_units:
                unit = f"{unit}, shown only" if unit else "shown only"     # drives nothing
            default = " ".join(part for part in (labels.get(tag.component, ""), f"[{unit}]" if unit else "") if part)
        tag.comment = edit.comment if edit is not None and edit.comment else default
        if edit is not None and edit.name:
            tag.name = edit.name
            continue
        signal = INDEX.sub("", tag.signal.split("[")[0])
        entry = re.fullmatch(r"[^\[]+\[(\d+)\]", tag.signal)       # one entry of a REAL array: its index
        if entry and tag.data_type != "Bool":
            signal = f"{signal}_{entry.group(1)}"
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


# One rule for ST names - the generated frame members and the tags: st_export.
ST_IDENTIFIER = st_export.ST_IDENTIFIER
ST_KEYWORDS = st_export.ST_KEYWORDS
st_name = st_export.st_name


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


def array_name(struct_type: str) -> str:
    """A tag array's name from its element STRUCT: BeltVfd_Command ->
    beltVfdCommands, TrackedLoad_Feedback -> trackedLoadFeedbacks."""
    stem, _sep, side = struct_type.rpartition("_")
    return stem[:1].lower() + stem[1:] + side + "s"


def tag_paths(config, tags: List[Tag], names: Dict[str, str]) -> Dict[str, str]:
    """key -> where the tag is in the generated PLC tags STRUCT: its ST name,
    or, for a device of a kind in the layout's axExport.tagArrays, an
    element of the array of its wire STRUCT: "beltVfdCommands[3].speed" -
    the index the device's place in its kind's list (every array of a kind
    indexed alike; an entry of a device of another type stays unused)."""
    kinds = list(getattr(getattr(config, "axExport", None), "tagArrays", None) or [])
    index: dict[str, int] = {}
    for kind in kinds:
        for i, model in enumerate(getattr(config, kind, None) or []):
            index[model.id] = i
    paths = {}
    by_id = {}
    for kind in kinds:
        for model in getattr(config, kind, None) or []:
            by_id[model.id] = model
    for t in tags:
        model = by_id.get(t.component)
        member = t.member[len(t.frame) + 1:]
        top, dot, field = member.partition(".")
        struct = st_export.device_type(model, "out" if t.is_input else "in") if model is not None else None
        if struct and dot:
            paths[t.key] = f"{array_name(struct)}[{index[t.component]}].{field}"
        else:
            paths[t.key] = names[t.key]
    return paths


def tags_json(config) -> List[dict]:
    """The tags as data for the AX generator (st_export.py --tags): per tag
    its key, name, ST name, table, comment, TIA type and address, frame, the
    frame member it is (without the frame: "div_1.present",
    "sensor_1_2_1"), whether the PLC reads it, and where it is in the PLC
    tags STRUCT (stPath: its ST name, or an array element - tag_paths)."""
    tags, _ = tags_of(config)
    names = st_names(tags)
    paths = tag_paths(config, tags, names)
    return [{"key": t.key, "name": t.name, "stName": names[t.key], "stPath": paths[t.key], "table": t.table,
             "comment": t.comment, "dataType": t.data_type, "address": t.address, "frame": t.frame,
             "member": t.member[len(t.frame) + 1:], "input": t.is_input} for t in tags]


def read_values(config, tags: List[Tag]) -> Tuple[List[float], List[str]]:
    """Every tag's value now: (numbers, texts), in the tags' order. The
    number plots it (a Bool 0 / 1, an array of bits: how many are 1); the
    text shows it (TRUE / FALSE, the bits, a REAL to 3 decimals)."""
    import io_signals                                   # here: plc_tags stays light to import
    ids = sorted({t.component for t in tags if t.component != "clock"})
    values = io_signals.read(config, ids) if ids else {}
    numbers, texts = [], []
    for tag in tags:
        if tag.component == "clock":
            numbers.append(0.0)
            texts.append("(PC clock)")
            continue
        signal = INDEX.sub("", tag.signal.split("[")[0])
        v = values.get(tag.component, {}).get("values", {}).get(signal)
        if v is None:
            numbers.append(0.0)
            texts.append("-")
        elif tag.count > 1:
            bits = v if isinstance(v, str) else "".join("1" if b else "0" for b in v)
            bits = bits[:tag.count]
            numbers.append(float(bits.count("1")))
            texts.append(bits)
        elif tag.data_type == "Bool":
            on = (v[:1] == "1") if isinstance(v, str) else bool(v[0] if isinstance(v, (list, tuple)) and v else v)
            numbers.append(1.0 if on else 0.0)
            texts.append("TRUE" if on else "FALSE")
        else:
            try:
                number = float(v)
            except (TypeError, ValueError):
                numbers.append(0.0)
                texts.append(str(v))
                continue
            numbers.append(number)
            texts.append(f"{number:.3f}" if tag.data_type == "Real" else str(int(number)))
    return numbers, texts

