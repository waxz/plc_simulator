"""validate_channel_registry.py - check a channels.json (and the CSVs it
references) against a simulator config without starting the simulator, so
someone iterating on a mapping CSV gets an answer in under a second instead
of booting the TCP server and physics engine just to find a bad row.

Usage:
    python validate_channel_registry.py --registry channels.json --config sim_config.json

Runs the exact same checks MQTTSyncEngine runs at startup for each channel
(load_channel_registry's own parsing, then verify_component_list() for
every io_mapping channel) and reports every channel's entry count and
payload byte size on success - the same numbers that would otherwise only
show up in the simulator's own startup log.
"""
import argparse
import sys
from typing import List, Optional

from config_parser import load_config
from mqtt_message_binding import ComponentMessage, MetricsMessage, verify_component_list
from channel_registry import ChannelRegistryError, load_channel_registry


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Validate a channel registry file against a simulator config.")
    parser.add_argument("--registry", required=True, help="channels.json path")
    parser.add_argument("--config", required=True, help="Simulator config JSON")
    return parser


def run(argv: Optional[List[str]] = None) -> int:
    args = build_argparser().parse_args(argv)

    try:
        config = load_config(args.config)
    except Exception as exc:
        print(f"Failed to load {args.config}: {exc}", file=sys.stderr)
        return 1

    try:
        publish_topics, subscribe_topics, metrics_channels = load_channel_registry(args.registry)
    except ChannelRegistryError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    ok = True

    io_channels = [("in", topic, topic_def) for topic, topic_def in subscribe_topics.items()]
    io_channels += [("out", topic, topic_def) for topic, topic_def in publish_topics.items()]

    for direction, topic, topic_def in io_channels:
        mapping = topic_def["mapping"]
        try:
            verify_component_list(mapping, config)
        except ValueError as exc:
            print(f"[io_mapping:{topic} ({direction})] {exc}", file=sys.stderr)
            ok = False
            continue
        message = ComponentMessage(mapping)
        print(f"[io_mapping:{topic} ({direction})] OK - {len(mapping)} entries, {message.payload_size} payload bytes")

    for frame_type, channel_def in metrics_channels.items():
        mapping = channel_def["mapping"]
        message = MetricsMessage(mapping)
        print(
            f"[metric:{channel_def['topic_suffix']}] OK - frame_type={frame_type.name}, "
            f"{len(mapping)} entries, {message.payload_size} payload bytes"
        )

    if not ok:
        print("Registry has errors - see above.", file=sys.stderr)
        return 1

    print("Registry is valid.")
    return 0


def main() -> None:
    sys.exit(run())


if __name__ == "__main__":
    main()
