"""mqtt_data_exchange_codegen.py - read and rewrite the hardcoded
PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS channel definitions in
mqtt_data_exchange.py. This is the "load existing config, edit it, and
have edits land back in mqtt_data_exchange.py" path - the alternative to
channel_registry.py's file-based channels.json for a deployment that
hasn't (or won't) migrate off the hardcoded dicts.

read_hardcoded_channels() introspects the ALREADY-IMPORTED PUBLISH_TOPICS/
SUBSCRIBE_TOPICS/METRICS_CHANNELS objects and returns them in the exact
document shape channel_registry.validate_entries() (and the web editor)
already understand - the same editor UI, the same live validation, and
the same component picker work against this source completely unchanged.
It reads whatever THIS PROCESS currently has imported; re-run it (or
restart the process) after write_hardcoded_channels() to see the change.

write_hardcoded_channels() is the higher-stakes half, so it earns extra
care:

  - It locates PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS as exact
    statements in mqtt_data_exchange.py's own source via the `ast` module
    (never line/regex matching - nested brackets, strings, and comments
    inside a dict literal can never be miscounted this way) and replaces
    only those statements' text.
  - METRICS_MAPPING/METRICS_MAPPING_2 (and any further METRICS_MAPPING_N
    this creates) are never deleted or renamed - a metric channel whose
    frame_type already had a named mapping constant gets that SAME
    constant's body regenerated in place; a brand new metric channel gets
    a brand new METRICS_MAPPING_N, numbered one past whatever already
    exists. This mirrors generate_frame_type.py's own "always append,
    never renumber" rule, for the same reason: other files (tests,
    plc_mock.py) import these constants by name directly, and silently
    renaming or deleting one out from under them would break every such
    import without warning.
  - Before ever touching the file, the freshly generated source is (1)
    compiled - a syntax gate, a bug here must never leave a broken .py
    file on disk - and (2) actually exec'd in an isolated namespace and
    run back through the exact same validation channel_registry.
    validate_entries() already uses - a semantics gate, so a bad edit
    (an unknown component id, a duplicate frame_type) is rejected before
    it's written, not after. The file is written only if both pass, and
    the previous version is always saved alongside first as
    mqtt_data_exchange.py.bak.
"""
import ast
import os
import re
from typing import Any, Dict, List, Optional, Tuple

from mqtt_message_binding import ByteType, parse_mapping_rows, parse_path
from metric_layout import ALIAS_NAME, VECTOR_ALIAS, layout_mapping
from tcp_framed_transport import FrameType

_ID_LIST_CHUNK = 6  # how many component ids per generated source line

# Imports a generated metric layout class needs; ensured present in the
# target file by generate_source() before the classes are written.
_LAYOUT_IMPORTS = [
    ("dataclasses", ["dataclass"]),
    ("metric_layout", ["f32", "u8", "u16", "u32", "u64", "bit", "Vec2", "Vec3"]),
]


def default_target_file() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "mqtt_data_exchange.py")


# ============================================================== read side

def _channels_from_dicts(
    publish_topics: Dict[str, Dict[str, Any]],
    subscribe_topics: Dict[str, Dict[str, Any]],
    metrics_channels: Dict[Any, Dict[str, Any]],
    metrics_interval_ms: int,
) -> List[Dict[str, Any]]:
    """Shared by read_hardcoded_channels() (reading the live module) and
    write_hardcoded_channels()'s own re-validation pass (reading a freshly
    exec'd, not-yet-written candidate) - one conversion from PUBLISH_TOPICS/
    SUBSCRIBE_TOPICS/METRICS_CHANNELS shape to the document shape
    channel_registry.validate_entries() understands, used both ways."""
    entries: List[Dict[str, Any]] = []
    for topic_suffix, topic_def in publish_topics.items():
        entries.append({
            "name": topic_suffix, "kind": "io_mapping", "direction": "out",
            "topic": topic_suffix, "interval_ms": topic_def.get("interval", 100),
            "component_ids": list(topic_def.get("custom_ids", [])),
        })
    for topic_suffix, topic_def in subscribe_topics.items():
        entries.append({
            "name": topic_suffix, "kind": "io_mapping", "direction": "in",
            "topic": topic_suffix, "interval_ms": topic_def.get("interval", 100),
            "component_ids": list(topic_def.get("custom_ids", [])),
        })
    for frame_type, channel_def in metrics_channels.items():
        # "layout" is a metric layout dataclass (metric_layout.py);
        # layout_mapping() derives the (ByteType, path) list the editor's
        # row model needs. (A pre-migration file with the old "mapping"
        # key - a plain (ByteType, path) list - is still read correctly.)
        if "layout" in channel_def:
            mapping = layout_mapping(channel_def["layout"])
        else:
            mapping = channel_def["mapping"]
        entries.append({
            "name": channel_def["topic_suffix"], "kind": "metric", "direction": "in",
            "topic": channel_def["topic_suffix"],
            "interval_ms": channel_def.get("interval", metrics_interval_ms),
            "frame_type": frame_type.name,
            "rows": [{"byte_type": bt.name, "path": p} for bt, p in mapping],
        })
    return entries


def read_hardcoded_channels() -> List[Dict[str, Any]]:
    """The live PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS, in the
    document shape channel_registry.validate_entries()/the web editor
    already understand - each io_mapping entry's component_ids is exactly
    its current custom_ids list (the two are the same concept), and each
    metric entry's frame_type/rows are resolved from its FrameType key and
    (ByteType, path) mapping. Reads whatever THIS PROCESS currently has
    imported; re-import (or restart) after write_hardcoded_channels() to
    see a change reflected here."""
    import mqtt_data_exchange as live  # imported lazily so read/write stay decoupled from import order
    return _channels_from_dicts(
        live.PUBLISH_TOPICS, live.SUBSCRIBE_TOPICS, live.METRICS_CHANNELS, live.METRICS_INTERVAL_MS
    )


# ============================================================= write side

class CodegenError(ValueError):
    """Raised for anything that stops a rewrite before the file is
    touched - a name codegen can't find, generated source that doesn't
    compile, or a document that fails re-validation."""


def _find_toplevel_assignment(tree: ast.Module, name: str) -> Optional[ast.stmt]:
    for node in tree.body:
        target = None
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            target = node.target.id
        elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            target = node.targets[0].id
        if target == name:
            return node
    return None


def _node_span(source: str, node: ast.AST) -> Tuple[int, int]:
    lines = source.splitlines(keepends=True)
    start = sum(len(l) for l in lines[: node.lineno - 1]) + node.col_offset
    end = sum(len(l) for l in lines[: node.end_lineno - 1]) + node.end_col_offset
    return start, end


def _apply_spans(source: str, spans: List[Tuple[int, int, str]]) -> str:
    """Replace each (start, end, text) span - applied from the end of the
    file backward so earlier offsets, computed once up front, stay valid
    no matter how many replacements come after them."""
    for start, end, text in sorted(spans, key=lambda s: -s[0]):
        source = source[:start] + text + source[end:]
    return source


def _layout_class_names(tree: ast.Module) -> Dict[int, str]:
    """{FrameType value -> layout class name} learned from
    METRICS_CHANNELS' CURRENT source - i.e. which class each already-
    declared metric channel's "layout" value points at, so a re-save
    regenerates that SAME class block instead of a new one. (A
    pre-migration file's "mapping" key is read the same way.)"""
    node = _find_toplevel_assignment(tree, "METRICS_CHANNELS")
    if node is None:
        return {}
    dict_node = node.value if isinstance(node, ast.AnnAssign) else node.value
    if not isinstance(dict_node, ast.Dict):
        return {}
    result: Dict[int, str] = {}
    for key_node, value_node in zip(dict_node.keys, dict_node.values):
        frame_type_value = _eval_frame_type_key(key_node)
        if frame_type_value is None or not isinstance(value_node, ast.Dict):
            continue
        for k, v in zip(value_node.keys, value_node.values):
            if isinstance(k, ast.Constant) and k.value in ("layout", "mapping") and isinstance(v, ast.Name):
                result[frame_type_value] = v.id
    return result


def _eval_frame_type_key(node: ast.AST) -> Optional[int]:
    """METRICS_CHANNELS keys are written as `FrameType.NAME` - pull the
    NAME back out without a full eval (the file may reference names that
    don't exist in this process's own FrameType yet, for a channel being
    added in the same save)."""
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "FrameType":
        try:
            return FrameType[node.attr].value
        except KeyError:
            return None
    return None


def _camel(name: str) -> str:
    """'perf' -> 'Perf', 'my_group' -> 'MyGroup', 'METRIC_LAYOUT_1' ->
    'MetricLayout1'. Deterministic so a metric group / frame_type always
    maps to the same class name across saves."""
    return "".join(part.capitalize() for part in name.split("_") if part)


def _layout_class_name(frame_type_name: str) -> str:
    return _camel(frame_type_name)


def _decorated_span(source: str, node: ast.stmt) -> Tuple[int, int]:
    """Char span of a class/def INCLUDING its decorator lines - ast puts
    node.lineno on the `class`/`def` keyword, not the `@dataclass` above
    it, so a span from the node alone would leave the decorator orphaned."""
    start_node = node.decorator_list[0] if getattr(node, "decorator_list", None) else node
    start, _ = _node_span(source, start_node)
    line_start = source.rfind("\n", 0, start) + 1  # include the '@'
    _, end = _node_span(source, node)
    return line_start, end


def _layout_block_span(tree: ast.Module, source: str, layout_class: str) -> Optional[Tuple[int, int]]:
    """Char span covering the layout class AND its per-group classes
    (`<layout_class><Group>`), as one contiguous block to replace. None if
    the layout isn't declared yet (a brand-new channel)."""
    def belongs(name: str) -> bool:
        if name == layout_class:
            return True
        rest = name[len(layout_class):]
        return name.startswith(layout_class) and bool(rest) and not rest[0].isdigit()

    blocks = [n for n in tree.body if isinstance(n, ast.ClassDef) and belongs(n.name)]
    if not blocks:
        return None
    spans = [_decorated_span(source, n) for n in blocks]
    return min(s for s, _ in spans), max(e for _, e in spans)


def _ensure_imports(source: str, spans: List[Tuple[int, int, str]]) -> None:
    """Add `from <mod> import <names>` for anything a generated layout
    class needs that the file doesn't already import, as a span just after
    the last existing top-level import."""
    missing = [
        f"from {mod} import {', '.join(names)}"
        for mod, names in _LAYOUT_IMPORTS
        if not re.search(rf"^from {re.escape(mod)} import ", source, re.MULTILINE)
    ]
    if not missing:
        return
    imports = list(re.finditer(r"^(?:from|import) .+$", source, re.MULTILINE))
    at = imports[-1].end() if imports else 0
    spans.append((at, at, "\n" + "\n".join(missing)))


def _format_component_id_list(ids: List[str]) -> str:
    if not ids:
        return "[]"
    if len(ids) <= _ID_LIST_CHUNK:
        return "[" + ", ".join(repr(i) for i in ids) + "]"
    lines = ["["]
    for i in range(0, len(ids), _ID_LIST_CHUNK):
        chunk = ids[i : i + _ID_LIST_CHUNK]
        lines.append("            " + ", ".join(repr(x) for x in chunk) + ",")
    lines.append("        ]")
    return "\n".join(lines)


def _format_topics_dict(name: str, entries: List[Dict[str, Any]]) -> str:
    lines = [f"{name}: Dict[str, Dict[str, Any]] = {{"]
    for e in entries:
        ids_src = _format_component_id_list(e["component_ids"])
        lines.append(f'    {e["topic"]!r}: {{')
        lines.append(f'        "interval": {e["interval_ms"]},')
        lines.append(f'        "custom_ids": {ids_src},')
        lines.append("    },")
    lines.append("}")
    return "\n".join(lines)


def _format_group_class(group_class: str, entries: List[Tuple[ByteType, str, Optional[int]]]) -> str:
    """One @dataclass for a metric group. `entries` are (byte_type, name,
    index) in wire order; a run of same-name indexed FLOAT entries becomes
    one Vec2/Vec3 field."""
    lines = ["@dataclass", f"class {group_class}:"]
    i = 0
    while i < len(entries):
        byte_type, name, index = entries[i]
        if index is None:
            lines.append(f"    {name}: {ALIAS_NAME[byte_type]}")
            i += 1
            continue
        j = i
        while j < len(entries) and entries[j][1] == name and entries[j][2] is not None:
            j += 1
        count = j - i
        alias = VECTOR_ALIAS.get(count)
        if alias is None or byte_type != ByteType.FLOAT:
            raise CodegenError(
                f"metric field {name!r}: only 2- or 3-element FLOAT vectors are "
                f"supported in a layout class (got {count} x {byte_type.name})"
            )
        lines.append(f"    {name}: {alias}")
        i = j
    return "\n".join(lines)


def _format_layout_classes(layout_class: str, rows: List[Dict[str, str]]) -> str:
    """The full source block for one metric layout: an @dataclass per
    metric group, then the @dataclass that groups them.

    parse_mapping_rows() (mqtt_message_binding.py, the same resolver
    channel_registry.validate_entries() uses) validates the byte types and
    expands a "PAD" shorthand row. Explicit padding entries are then
    dropped - a metric layout's byte layout comes from the class's typed
    fields, and a run of `bit` fields already rounds up to a whole byte in
    the wire format (create_pack_format_string); a layout that genuinely
    needs hand-placed padding belongs in a channels.json registry, which
    keeps the string-row form."""
    try:
        resolved = parse_mapping_rows(
            [(row.get("byte_type", ""), row.get("path", "")) for row in rows], source=layout_class
        )
    except ValueError as exc:
        raise CodegenError(str(exc)) from None

    groups: "Dict[str, List[Tuple[ByteType, str, Optional[int]]]]" = {}
    for byte_type, path in resolved:
        group, name, index = parse_path(path)
        if group.startswith("padding"):
            continue
        # A metric layout is a real class now: every row becomes a
        # `group` dataclass and a `name` field on it, so both have to be
        # valid Python identifiers. The editor lets a row be saved with an
        # empty/blank Group or Name (validation only checks the wire
        # layout), so catch it here with a message that names the row
        # instead of letting the generated file fail to compile.
        if not group.isidentifier() or group.startswith("_"):
            raise CodegenError(
                f"metric field {path!r}: Group {group!r} must be a plain name "
                f"(letters, digits, underscores; not starting with a digit or underscore)"
            )
        if not name.isidentifier() or name.startswith("_"):
            raise CodegenError(
                f"metric field {path!r}: Name {name!r} must be a plain name "
                f"(letters, digits, underscores; not starting with a digit or underscore) - "
                f"an empty Name usually means a row is still being filled in"
            )
        groups.setdefault(group, []).append((byte_type, name, index))

    if not groups:
        raise CodegenError(f"metric layout {layout_class!r} has no fields")

    for group, entries in groups.items():
        # One logical field per name (a Vec2/Vec3's repeated name counts
        # once); two rows in a group that resolve to the same field name
        # would generate a duplicate/shadowing dataclass field.
        logical: List[str] = []
        for _bt, name, index in entries:
            if index is None or name not in logical:
                logical.append(name)
        dupes = sorted({n for n in logical if logical.count(n) > 1})
        if dupes:
            raise CodegenError(
                f"metric layout {layout_class!r}, group {group!r}: duplicate field name(s) "
                f"{', '.join(dupes)} - each row in a group needs a distinct Name"
            )

    blocks = [_format_group_class(layout_class + _camel(group), entries) for group, entries in groups.items()]
    layout_lines = ["@dataclass", f"class {layout_class}:"]
    for group in groups:
        layout_lines.append(f"    {group}: {layout_class + _camel(group)}")
    blocks.append("\n".join(layout_lines))
    return "\n\n\n".join(blocks)


def _format_metrics_channels_dict(entries: List[Tuple[Any, str, Dict[str, Any]]]) -> str:
    """entries: [(FrameType member, layout_class_name, channel_def), ...]"""
    lines = ["METRICS_CHANNELS: Dict[int, Dict[str, Any]] = {"]
    for frame_type, layout_class, e in entries:
        lines.append(
            f'    FrameType.{frame_type.name}: {{"topic_suffix": {e["topic"]!r}, "layout": {layout_class}}},'
        )
    lines.append("}")
    return "\n".join(lines)


def generate_source(source: str, channels: List[Dict[str, Any]]) -> str:
    """The pure, file-free half of write_hardcoded_channels(): given the
    CURRENT source text and an edited channel list, return the fully
    regenerated source text. Split out from the disk-touching half so it
    can be unit tested without ever writing anything."""
    tree = ast.parse(source)
    existing_layout_names = _layout_class_names(tree)

    publish_entries = [c for c in channels if c["kind"] == "io_mapping" and c["direction"] == "out"]
    subscribe_entries = [c for c in channels if c["kind"] == "io_mapping" and c["direction"] == "in"]
    metric_entries = [c for c in channels if c["kind"] == "metric"]

    spans: List[Tuple[int, int, str]] = []

    publish_node = _find_toplevel_assignment(tree, "PUBLISH_TOPICS")
    if publish_node is None:
        raise CodegenError("PUBLISH_TOPICS assignment not found - is mqtt_data_exchange.py still structured as expected?")
    spans.append((*_node_span(source, publish_node), _format_topics_dict("PUBLISH_TOPICS", publish_entries)))

    subscribe_node = _find_toplevel_assignment(tree, "SUBSCRIBE_TOPICS")
    if subscribe_node is None:
        raise CodegenError("SUBSCRIBE_TOPICS assignment not found - is mqtt_data_exchange.py still structured as expected?")
    spans.append((*_node_span(source, subscribe_node), _format_topics_dict("SUBSCRIBE_TOPICS", subscribe_entries)))

    metrics_node = _find_toplevel_assignment(tree, "METRICS_CHANNELS")
    if metrics_node is None:
        raise CodegenError("METRICS_CHANNELS assignment not found - is mqtt_data_exchange.py still structured as expected?")

    metrics_channel_entries = []
    # A brand-new layout's class block can't be a separate zero-width
    # "insert" span at the same offset METRICS_CHANNELS' own replacement
    # span starts at - two spans sharing a start point corrupt each other
    # once _apply_spans splices the first in. New blocks are collected here
    # and prepended onto the ONE span that replaces METRICS_CHANNELS.
    new_blocks_source = ""
    for e in metric_entries:
        try:
            frame_type = FrameType[e["frame_type"]]
        except KeyError:
            raise CodegenError(f"unknown frame_type {e['frame_type']!r} for channel {e['name']!r}") from None

        # Class name is a pure function of the frame_type (MetricLayout1 <->
        # METRIC_LAYOUT_1), so a re-save always regenerates the same block
        # and removing a channel never renumbers another - the "always
        # append, never renumber" rule the old METRICS_MAPPING_N counter
        # enforced by hand is now automatic. A learned name is only honored
        # if a class by it actually exists (a pre-migration file's stale
        # "mapping": <list> reference resolves to the frame-type name).
        layout_class = existing_layout_names.get(frame_type.value)
        block_span = _layout_block_span(tree, source, layout_class) if layout_class else None
        if block_span is None:
            layout_class = _layout_class_name(e["frame_type"])
            block_span = _layout_block_span(tree, source, layout_class)
        block_source = _format_layout_classes(layout_class, e["rows"])
        if block_span is not None:
            spans.append((*block_span, block_source))
        else:
            new_blocks_source += block_source + "\n\n\n"

        metrics_channel_entries.append((frame_type, layout_class, e))

    metrics_channels_source = new_blocks_source + _format_metrics_channels_dict(metrics_channel_entries)
    spans.append((*_node_span(source, metrics_node), metrics_channels_source))

    if metric_entries:
        _ensure_imports(source, spans)

    return _apply_spans(source, spans)


def _revalidate_generated_source(source: str, target_file: str, config) -> None:
    """The semantics gate: exec the candidate source in an isolated
    namespace (never sys.modules - this must never touch the real,
    already-imported mqtt_data_exchange module) and run what it would
    actually produce back through channel_registry.validate_entries(),
    the exact function the web editor's own /api/validate already uses.
    Raises CodegenError naming every problem found; raises nothing if the
    candidate is clean."""
    from channel_registry import validate_entries

    try:
        code = compile(source, target_file, "exec")
    except SyntaxError as exc:
        raise CodegenError(f"generated source does not compile: {exc}") from exc

    namespace: Dict[str, Any] = {"__name__": "_mqtt_data_exchange_codegen_check__"}
    try:
        exec(code, namespace)
    except Exception as exc:
        raise CodegenError(f"generated source raised {type(exc).__name__} on import: {exc}") from exc

    for required in ("PUBLISH_TOPICS", "SUBSCRIBE_TOPICS", "METRICS_CHANNELS", "METRICS_INTERVAL_MS"):
        if required not in namespace:
            raise CodegenError(f"generated source is missing {required} after exec - refusing to write it")

    entries = _channels_from_dicts(
        namespace["PUBLISH_TOPICS"], namespace["SUBSCRIBE_TOPICS"],
        namespace["METRICS_CHANNELS"], namespace["METRICS_INTERVAL_MS"],
    )
    results = validate_entries(entries, config)
    failures = [r for r in results if not r["ok"]]
    if failures:
        detail = "; ".join(f"{r['name']}: {', '.join(r['errors'])}" for r in failures)
        raise CodegenError(f"generated source fails validation, nothing written: {detail}")


def write_hardcoded_channels(channels: List[Dict[str, Any]], config, target_file: Optional[str] = None) -> str:
    """Regenerate PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS (and any
    metric mapping constants they need) in mqtt_data_exchange.py from an
    edited channel document, gated by generate_source() compiling AND
    re-validating clean (see _revalidate_generated_source) before a single
    byte of the real file changes. The previous version is copied to
    `target_file + ".bak"` first (overwriting any earlier backup) as a
    last-resort manual revert path. Returns the new source that was
    written.

    Like generate_frame_type.py's target file, this process (and any
    other already-running one) keeps the OLD PUBLISH_TOPICS/SUBSCRIBE_TOPICS/
    METRICS_CHANNELS in memory until it restarts - writing the file alone
    does not change what's currently running.
    """
    target_file = target_file or default_target_file()
    with open(target_file, "r", encoding="utf-8") as f:
        source = f.read()

    new_source = generate_source(source, channels)
    _revalidate_generated_source(new_source, target_file, config)  # raises CodegenError; nothing written yet

    backup_path = target_file + ".bak"
    with open(backup_path, "w", encoding="utf-8") as f:
        f.write(source)
    with open(target_file, "w", encoding="utf-8") as f:
        f.write(new_source)

    return new_source
