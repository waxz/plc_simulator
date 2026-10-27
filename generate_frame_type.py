"""generate_frame_type.py - add a FrameType enum member (tcp_framed_
transport.py) for every metric channel's frame_type in a channel registry
that doesn't already have one, instead of hand-editing the enum.

A metric channel's frame_type is just a string in channels.json - nothing
stops someone from naming a new one ("METRIC_LAYOUT_3", "METRIC_ENV_2",
whatever) that FrameType doesn't define yet. Validation then correctly
reports "unknown frame_type", because FrameType is a real Python IntEnum
(the frame_type byte on the wire has to resolve to one specific small
integer, and that mapping needs to survive a restart) - it can't just
invent a value at request time the way an in-memory-only registry could.
This script is the other half: point it at the registry and it appends
whatever's missing.

Usage:
    python generate_frame_type.py --registry channels.json
    python generate_frame_type.py --registry channels.json --check   # report only, don't write

Existing members are never renumbered - a frame_type's integer value is
what's actually on the wire, so an already-deployed client and a
regenerated simulator need to keep agreeing on what that byte means. Only
missing names get a value, always one higher than the current maximum
found either on the live FrameType enum or already written in the target
file - checking both (not just the live enum) is what makes running this
twice in the same process safe: the second run sees what the first one
already wrote, even though the live import hasn't picked it up yet.

A process that already imported FrameType keeps the old enum in memory
until it restarts - this script only changes the source file on disk.
"""
import argparse
import json
import os
import re
import sys
from typing import Dict, List, Optional

from tcp_framed_transport import FrameType

_CLASS_RE = re.compile(r"^class FrameType\(IntEnum\):")
_MEMBER_RE = re.compile(r"^(\s+)([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(\d+)")


def default_target_file() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "tcp_framed_transport.py")


def registry_metric_frame_types(registry_path: str) -> List[str]:
    """Every frame_type name a metric channel in this registry declares, in
    file order, de-duplicated. io_mapping channels aren't included - their
    frame_type isn't wired up to anything yet (see the Channel Registry
    PRD's M3 phase), so generating a member for one would be premature."""
    with open(registry_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    names: List[str] = []
    seen = set()
    for entry in data.get("channels", []):
        if entry.get("kind") != "metric":
            continue
        name = entry.get("frame_type")
        if name and name not in seen:
            names.append(name)
            seen.add(name)
    return names


def members_defined_in_source(source: str) -> Dict[str, int]:
    """Name -> value for every member line found in FrameType's class body
    in this exact source text - independent of what's actually imported,
    so a second run against the same (not-yet-restarted) target file sees
    what an earlier run in this same process already wrote."""
    members: Dict[str, int] = {}
    in_class = False
    for line in source.splitlines():
        if _CLASS_RE.match(line):
            in_class = True
            continue
        if not in_class:
            continue
        member_match = _MEMBER_RE.match(line)
        if member_match:
            members[member_match.group(2)] = int(member_match.group(3))
        elif line.strip() and not line[:1].isspace():
            break
    return members


def _insert_new_members(source: str, new_members: Dict[str, int]) -> str:
    """Append `new_members` right after FrameType's last existing member
    line, leaving every other line - including the class's own docstring
    and every existing member's own trailing comment - untouched."""
    lines = source.splitlines(keepends=True)
    in_class = False
    last_member_line_index: Optional[int] = None
    indent = "    "

    for i, line in enumerate(lines):
        if _CLASS_RE.match(line):
            in_class = True
            continue
        if not in_class:
            continue
        member_match = _MEMBER_RE.match(line)
        if member_match:
            indent = member_match.group(1)
            last_member_line_index = i
        elif line.strip() and not line[:1].isspace():
            break  # a new top-level statement - we've left the class body

    if last_member_line_index is None:
        raise RuntimeError("Could not find FrameType's last member - is the class definition still there?")

    inserted = "".join(
        f"{indent}{name} = {value}  # added by generate_frame_type.py\n"
        for name, value in new_members.items()
    )
    insert_at = last_member_line_index + 1
    return "".join(lines[:insert_at]) + inserted + "".join(lines[insert_at:])


def compute_missing_members(target_file: str, names: List[str]) -> Dict[str, int]:
    """What add_missing_members() would add for `names`, without writing
    anything - the read-only half, reused by both --check and by anything
    that just wants to know the values before committing to them (a web
    endpoint that wants to show "would add X = 4" before doing it, say)."""
    with open(target_file, "r", encoding="utf-8") as f:
        source = f.read()

    existing = {member.name: member.value for member in FrameType}
    existing.update(members_defined_in_source(source))  # a prior run this session, not yet restarted

    missing = [name for name in dict.fromkeys(names) if name not in existing]
    next_value = max(existing.values(), default=0) + 1
    new_members: Dict[str, int] = {}
    for name in missing:
        new_members[name] = next_value
        next_value += 1
    return new_members


def add_missing_members(target_file: str, names: List[str]) -> Dict[str, int]:
    """compute_missing_members(), then actually append the result to
    target_file. Returns the same {name: value} mapping (empty if nothing
    was missing, in which case the file is left untouched) - the single
    function both the CLI and the web editor's "Generate FrameType" action
    call, so there's exactly one implementation of what counts as already-
    defined and what value a new name gets."""
    new_members = compute_missing_members(target_file, names)
    if not new_members:
        return {}

    with open(target_file, "r", encoding="utf-8") as f:
        source = f.read()
    updated = _insert_new_members(source, new_members)
    with open(target_file, "w", encoding="utf-8") as f:
        f.write(updated)
    return new_members


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Add a FrameType member for every metric channel frame_type not already defined."
    )
    parser.add_argument("--registry", required=True, help="channels.json to read frame_type names from")
    parser.add_argument("--check", action="store_true", help="report what would change, without writing")
    parser.add_argument("--target", help="tcp_framed_transport.py to modify (defaults to the one next to this script)")
    return parser


def run(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)
    target_file = args.target or default_target_file()

    try:
        names = registry_metric_frame_types(args.registry)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Failed to read {args.registry}: {exc}", file=sys.stderr)
        return 1

    try:
        new_members = compute_missing_members(target_file, names) if args.check else add_missing_members(target_file, names)
    except OSError as exc:
        print(f"Failed to read/update {target_file}: {exc}", file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f"Failed to update {target_file}: {exc}", file=sys.stderr)
        return 1

    if not new_members:
        print("FrameType already covers every metric channel in the registry - nothing to do.")
        return 0

    for name, value in new_members.items():
        print(f"{name} = {value}")

    if args.check:
        print(f"(--check: {target_file} not written)")
        return 0

    print(f"Wrote {len(new_members)} new FrameType member(s) to {target_file}")
    print("Any already-running process (including this one) keeps the old enum in memory until it restarts.")
    return 0


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
