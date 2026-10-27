"""channel_registry.py - load IO-mapping and metric channel definitions
from an external registry file instead of the hardcoded PUBLISH_TOPICS/
SUBSCRIBE_TOPICS/METRICS_CHANNELS dicts in mqtt_data_exchange.py.

Fully additive: MQTTSyncEngine only calls load_channel_registry() when a
channel_registry_path is given. Nothing here changes behavior for a
deployment that doesn't pass one - PUBLISH_TOPICS/SUBSCRIBE_TOPICS/
METRICS_CHANNELS stay exactly as they are, byte-for-byte.

Registry file shape (channels.json):
    {
      "channels": [
        {
          "name": "output",
          "kind": "io_mapping",   # or "metric"
          "direction": "in",      # "in" = client -> simulator, "out" = simulator -> client
          "frame_type": "IO_MAPPING",  # required for "metric"; optional for "io_mapping" today
          "topic": "output",
          "interval_ms": 10,
          "mapping_csv": "channels/output.csv"
        }
      ]
    }

Each channel's field-level layout lives in its own CSV, in the exact
"ByteType,Path" format mqtt_message_binding.print_mapping() already
writes (see load_mapping_csv there) - mapping_csv paths are resolved
relative to the registry file's own directory, not the process's current
working directory, so a registry can be referenced from anywhere.
"""
import json
import os
from typing import Any, Dict, List, Optional, Tuple

from mqtt_message_binding import (
    ComponentMessage,
    MetricsMessage,
    custom_component_mapping,
    known_component_ids,
    load_mapping_csv,
    parse_mapping_rows,
    verify_component_list,
)
from tcp_framed_transport import FrameType

_REQUIRED_FIELDS = ("name", "kind", "direction", "topic", "mapping_csv")


class ChannelRegistryError(ValueError):
    """Raised for a malformed registry file or an invalid channel entry.
    Always names the registry file and the offending channel, since the
    intended reader is someone editing channels.json by hand, not
    debugging Python - a bare KeyError/JSONDecodeError wouldn't tell them
    which entry or field to fix."""


def load_channel_registry(
    registry_path: str,
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]], Dict[int, Dict[str, Any]]]:
    """Read a channels.json registry file and return three dicts shaped
    exactly like PUBLISH_TOPICS, SUBSCRIBE_TOPICS, and METRICS_CHANNELS in
    mqtt_data_exchange.py, so MQTTSyncEngine's existing channel-building
    loops work unmodified regardless of which source they iterate over.
    Each returned entry carries an already-resolved "mapping" (loaded from
    its mapping_csv) instead of the legacy "custom_ids" list.
    """
    try:
        with open(registry_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        raise ChannelRegistryError(f"{registry_path}: failed to read registry file: {exc}") from exc

    channels = data.get("channels")
    if not isinstance(channels, list):
        raise ChannelRegistryError(f"{registry_path}: expected a top-level \"channels\" list")

    base_dir = os.path.dirname(os.path.abspath(registry_path))
    publish_topics: Dict[str, Dict[str, Any]] = {}
    subscribe_topics: Dict[str, Dict[str, Any]] = {}
    metrics_channels: Dict[int, Dict[str, Any]] = {}

    for entry in channels:
        name = entry.get("name", "<unnamed channel>")
        for field in _REQUIRED_FIELDS:
            if field not in entry:
                raise ChannelRegistryError(f"{registry_path}: channel '{name}' is missing required field '{field}'")

        kind = entry["kind"]
        direction = entry["direction"]
        topic = entry["topic"]
        interval_ms = entry.get("interval_ms", 100)
        csv_path = entry["mapping_csv"]
        if not os.path.isabs(csv_path):
            csv_path = os.path.join(base_dir, csv_path)

        try:
            mapping = load_mapping_csv(csv_path)
        except (OSError, ValueError) as exc:
            raise ChannelRegistryError(f"{registry_path}: channel '{name}': {exc}") from exc

        topic_def: Dict[str, Any] = {"interval": interval_ms, "mapping": mapping}
        if "endianness" in entry:
            topic_def["endianness"] = entry["endianness"]

        if kind == "io_mapping":
            if entry.get("frame_type"):
                try:
                    topic_def["frame_type"] = int(FrameType[entry["frame_type"]])
                except KeyError:
                    raise ChannelRegistryError(
                        f"{registry_path}: channel '{name}': unknown frame_type '{entry['frame_type']}'"
                    ) from None
            if direction == "out":
                publish_topics[topic] = topic_def
            elif direction == "in":
                subscribe_topics[topic] = topic_def
            else:
                raise ChannelRegistryError(
                    f"{registry_path}: channel '{name}': direction must be 'in' or 'out', got '{direction}'"
                )
        elif kind == "metric":
            frame_type_name = entry.get("frame_type")
            if not frame_type_name:
                raise ChannelRegistryError(f"{registry_path}: metric channel '{name}' is missing 'frame_type'")
            try:
                frame_type = FrameType[frame_type_name]
            except KeyError:
                raise ChannelRegistryError(
                    f"{registry_path}: channel '{name}': unknown frame_type '{frame_type_name}' - "
                    f"run generate_frame_type.py --registry {registry_path} to add it"
                ) from None
            if frame_type in metrics_channels:
                raise ChannelRegistryError(
                    f"{registry_path}: frame_type '{frame_type_name}' is used by more than one metric channel"
                )
            topic_def["topic_suffix"] = topic
            metrics_channels[frame_type] = topic_def
        else:
            raise ChannelRegistryError(
                f"{registry_path}: channel '{name}': unknown kind '{kind}' (expected 'io_mapping' or 'metric')"
            )

    return publish_topics, subscribe_topics, metrics_channels


def validate_entries(entries: List[Dict[str, Any]], config) -> List[Dict[str, Any]]:
    """Validate a batch of in-progress, unsaved channel entries against a
    config, collecting every problem across every entry instead of
    stopping at the first - the batch-feedback counterpart to
    load_channel_registry()'s fail-fast file loader. Built for an
    interactive editor: nothing here touches disk, so it can run on
    whatever a person has typed so far.

    An io_mapping entry is built one of two ways:
      - "component_ids": [...] - a plain list of component ids. The
        mapping (which ByteType each field packs as, where padding goes)
        is entirely computed here via custom_component_mapping(), which
        reads each component's own packFormat (config_parser.py) - the
        caller never picks a ByteType or inserts padding itself. This is
        the only path the web editor's UI uses for io_mapping channels.
      - "rows": [{"byte_type", "path"}, ...] - a raw, already-authored
        mapping (PAD included), for a hand-edited CSV loaded from disk, or
        a metric channel (metrics have no config-backed component to pick
        from, so they're always authored this way).
    Every entry's resolved mapping (however it was built) is echoed back
    as "rows" in the result, so the caller always has something to display
    - including a component_ids-driven channel's auto-generated fields.

    "frame_type" is required when kind is "metric". Returns one
    {"name", "ok", "errors", "payload_size", "rows"} dict per entry, in the
    same order as `entries`.
    """
    results: List[Dict[str, Any]] = []
    name_counts: Dict[str, int] = {}
    frame_type_owner: Dict[Any, str] = {}

    for entry in entries:
        name = entry.get("name") or "<unnamed channel>"
        name_counts[name] = name_counts.get(name, 0) + 1

    for entry in entries:
        name = entry.get("name") or "<unnamed channel>"
        kind = entry.get("kind")
        direction = entry.get("direction")
        component_ids = entry.get("component_ids")
        errors: List[str] = []
        payload_size: Optional[int] = None

        if not entry.get("name"):
            errors.append("missing 'name'")
        elif name_counts.get(name, 0) > 1:
            errors.append(f"channel name '{name}' is used by more than one channel")
        if kind not in ("io_mapping", "metric"):
            errors.append(f"unknown kind '{kind}' (expected 'io_mapping' or 'metric')")
        if kind == "io_mapping" and direction not in ("in", "out"):
            errors.append(f"direction must be 'in' or 'out', got '{direction}'")
        if not entry.get("topic"):
            errors.append("missing 'topic'")

        mapping = None
        if kind == "io_mapping" and component_ids is not None:
            unknown = [cid for cid in component_ids if cid not in known_component_ids(config)]
            if unknown:
                errors.append(f"unknown component id(s): {', '.join(unknown)}")
            else:
                mapping = custom_component_mapping(config, component_ids, direction=direction if direction in ("in", "out") else "in")
        else:
            rows = entry.get("rows") or []
            try:
                mapping = parse_mapping_rows(
                    [(row.get("byte_type", ""), row.get("path", "")) for row in rows],
                    source=name,
                    start_row=1,
                )
            except ValueError as exc:
                errors.append(str(exc))
            if kind == "metric":
                # Every metric row becomes a (group, name) key in
                # MetricsStore / the Live Data tree - and, in the web
                # editor's hardcoded mode, a dataclass group + typed field
                # (mqtt_data_exchange_codegen.py). Both need a real Group
                # AND Name; flag a blank one against its row number so the
                # editor highlights it as you type, instead of it only
                # failing later at save.
                for row_num, row in enumerate(rows, start=1):
                    if str(row.get("byte_type", "")).strip().upper() in ("PAD", "PADDING"):
                        continue
                    group, sep, field = str(row.get("path", "")).partition("/")
                    if not group.strip() or not sep or not field.strip():
                        errors.append(
                            f"{name}:{row_num}: metric field needs a Group and a Name "
                            f"(got {row.get('path', '') or ''!r})"
                        )

        if mapping is not None and kind == "io_mapping":
            # Only io_mapping fields are real config components (sensors/
            # belts/buttons/leds) - a metric field like "perf/scan_time_ms"
            # is a free-form group/name pair MetricsMessage writes straight
            # into MetricsStore, never checked against config anywhere at
            # runtime either (see _make_metrics_recv_handler in
            # mqtt_data_exchange.py). Running this check on a metric
            # mapping would reject perfectly valid field names just
            # because they don't happen to name a real component.
            try:
                verify_component_list(mapping, config)
            except ValueError as exc:
                errors.append(str(exc))
                mapping = None

        if mapping is not None:
            if kind == "io_mapping":
                payload_size = ComponentMessage(mapping).payload_size
            elif kind == "metric":
                payload_size = MetricsMessage(mapping).payload_size

        if kind == "metric":
            frame_type_name = entry.get("frame_type")
            if not frame_type_name:
                errors.append("metric channel is missing 'frame_type'")
            else:
                try:
                    frame_type = FrameType[frame_type_name]
                except KeyError:
                    errors.append(
                        f"unknown frame_type '{frame_type_name}' - "
                        f"run generate_frame_type.py --registry <path> to add it"
                    )
                else:
                    owner = frame_type_owner.get(frame_type)
                    if owner is not None and owner != name:
                        errors.append(f"frame_type '{frame_type_name}' is also used by channel '{owner}'")
                    else:
                        frame_type_owner[frame_type] = name

        result: Dict[str, Any] = {"name": name, "ok": not errors, "errors": errors, "payload_size": payload_size}
        if mapping is not None:
            result["rows"] = [{"byte_type": bt.name, "path": p} for bt, p in mapping]
        results.append(result)

    return results
