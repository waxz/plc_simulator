"""bootstrap_channel_csv.py - generate a channel mapping CSV from a list of
component ids, so adding a channel starts from a valid file instead of a
blank spreadsheet. Wraps custom_component_mapping() + print_mapping()
(mqtt_message_binding.py) - the exact functions MQTTSyncEngine itself uses
to build a channel - so the CSV this produces is a valid starting point,
not an approximation of one, and already has any required bit padding.

Pass --registry to also create or update a channels.json entry pointing at
the new CSV, so "add a channel" is one command instead of hand-writing
JSON afterward - the registry entry is created if the file doesn't exist,
appended if --name is new, or replaced in place if --name already exists
(safe to re-run after regenerating a CSV).

Usage:
    python bootstrap_channel_csv.py --config sim_config.json \\
        --ids led_1_1_1,led_1_1_2,belt_1_1_1 --out channels/output_node2.csv \\
        --registry channels.json --name output_node2 --direction in \\
        --frame-type IO_MAPPING --topic output2

    python bootstrap_channel_csv.py --config sim_config.json \\
        --ids-file node2_ids.txt --out channels/output_node2.csv

A line in --ids-file starting with "#" is a comment; blank lines are
skipped. An unknown component id is reported and stops the script (exit
code 1) instead of silently building a mapping that's missing what the
caller asked for.
"""
import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional

from config_parser import load_config
from mqtt_message_binding import custom_component_mapping, known_component_ids, print_mapping


def _read_ids(args: argparse.Namespace) -> List[str]:
    if args.ids:
        return [i.strip() for i in args.ids.split(",") if i.strip()]
    with open(args.ids_file, "r", encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip() and not line.startswith("#")]


def _load_or_create_registry(registry_path: str) -> Dict[str, Any]:
    if not os.path.exists(registry_path):
        return {"channels": []}
    with open(registry_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    data.setdefault("channels", [])
    return data


def _upsert_channel_entry(registry_data: Dict[str, Any], entry: Dict[str, Any]) -> bool:
    """Replace the channel with the same name if one exists, else append.
    Returns True if an existing entry was replaced (vs. a new one added),
    so the caller can report which happened."""
    channels = registry_data["channels"]
    for i, existing in enumerate(channels):
        if existing.get("name") == entry["name"]:
            channels[i] = entry
            return True
    channels.append(entry)
    return False


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a channel mapping CSV from a list of component ids.")
    parser.add_argument("--config", required=True, help="Simulator config JSON (e.g. sim_config.json)")
    ids_group = parser.add_mutually_exclusive_group(required=True)
    ids_group.add_argument("--ids", help="Comma-separated component ids")
    ids_group.add_argument("--ids-file", help="File with one component id per line")
    parser.add_argument("--out", required=True, help="Where to write the mapping CSV")

    registry_group = parser.add_argument_group(
        "registry (optional - creates/updates a channels.json entry pointing at --out)"
    )
    registry_group.add_argument("--registry", help="channels.json to create or update")
    registry_group.add_argument("--name", help="Channel name - required with --registry")
    registry_group.add_argument("--kind", choices=["io_mapping", "metric"], default="io_mapping")
    registry_group.add_argument(
        "--direction", choices=["in", "out"],
        help="'in' = client -> simulator, 'out' = simulator -> client - required with --registry",
    )
    registry_group.add_argument("--topic", help="Topic suffix for this channel - required with --registry")
    registry_group.add_argument(
        "--frame-type", help="FrameType name, e.g. METRIC_LAYOUT_1 - required when --kind metric",
    )
    registry_group.add_argument("--interval-ms", type=int, default=100)
    return parser


def _validate_registry_args(args: argparse.Namespace) -> Optional[str]:
    if not args.registry:
        return None
    missing = [flag for flag, value in (("--name", args.name), ("--direction", args.direction),
                                         ("--topic", args.topic)) if not value]
    if args.kind == "metric" and not args.frame_type:
        missing.append("--frame-type")
    if missing:
        return f"--registry requires {', '.join(missing)}"
    return None


def run(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)

    registry_error = _validate_registry_args(args)
    if registry_error:
        print(registry_error, file=sys.stderr)
        return 1

    try:
        config = load_config(args.config)
    except Exception as exc:
        print(f"Failed to load {args.config}: {exc}", file=sys.stderr)
        return 1

    ids = _read_ids(args)
    if not ids:
        print("No component ids given.", file=sys.stderr)
        return 1

    known = known_component_ids(config)
    unknown = [component_id for component_id in ids if component_id not in known]
    if unknown:
        print(f"Unknown component id(s): {', '.join(unknown)}", file=sys.stderr)
        print(
            f"Known ids come from {args.config}'s sensors/belts/buttons/leds (plus 'clock').",
            file=sys.stderr,
        )
        return 1

    mapping = custom_component_mapping(config, ids)
    print_mapping(mapping, args.out)
    print(f"Wrote {len(mapping)} entries to {args.out}")

    if not args.registry:
        print(
            "Next: point a channel's \"mapping_csv\" at this file in your registry "
            "(see Channel Registry PRD), then run:\n"
            f"  python validate_channel_registry.py --registry channels.json --config {args.config}"
        )
        return 0

    registry_dir = os.path.dirname(os.path.abspath(args.registry)) or "."
    relative_csv = os.path.relpath(os.path.abspath(args.out), start=registry_dir).replace(os.sep, "/")

    entry: Dict[str, Any] = {
        "name": args.name,
        "kind": args.kind,
        "direction": args.direction,
        "topic": args.topic,
        "interval_ms": args.interval_ms,
        "mapping_csv": relative_csv,
    }
    if args.frame_type:
        entry["frame_type"] = args.frame_type

    registry_data = _load_or_create_registry(args.registry)
    replaced = _upsert_channel_entry(registry_data, entry)
    with open(args.registry, "w", encoding="utf-8") as f:
        json.dump(registry_data, f, indent=2)
        f.write("\n")

    verb = "Updated" if replaced else "Added"
    print(f"{verb} channel '{args.name}' in {args.registry}")

    # Self-check: catch a broken registry (e.g. a duplicate frame_type)
    # immediately instead of leaving it for the next validate/startup run.
    from channel_registry import ChannelRegistryError, load_channel_registry
    try:
        load_channel_registry(args.registry)
    except ChannelRegistryError as exc:
        print(f"Warning: {args.registry} was written but does not validate: {exc}", file=sys.stderr)
        return 1

    print(f"{args.registry} is valid. Next: python validate_channel_registry.py "
          f"--registry {args.registry} --config {args.config}")
    return 0


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
