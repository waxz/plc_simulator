"""web_app.py - Flask app serving two things over one local server:

1. A browser-based channel registry editor with live validation, replacing
   hand-edited JSON/CSV + a separate CLI validate step with immediate,
   per-row feedback as you type.
2. (Only when wired to a running simulator's live MetricsStore) a metrics
   dashboard - browse groups/metrics and watch live charts, the web
   counterpart to data_visualization_panel.py's Qt dock.

Every check the registry editor reports comes from the SAME functions the
CLI tools and MQTTSyncEngine itself use (verify_component_list,
parse_mapping_rows, channel_registry.validate_entries) - the browser is
presentation only, not a second implementation of what's valid. This
matters more than it sounds: a validator reimplemented in JavaScript would
drift from what the simulator actually accepts the first time either side
changed, and nobody would notice until a "valid" registry failed at
startup.

Standalone (registry editor only, no live simulator needed):
    python web_app.py --config sim_config.json --registry channels.json

Embedded (adds the live metrics dashboard - called from main.py):
    from web_app import create_app
    app = create_app(config_path="sim_config.json", registry_path="channels.json",
                      metrics_store=metrics_store)
    threading.Thread(target=lambda: app.run(port=5050, use_reloader=False), daemon=True).start()
"""
import argparse
import json
import os
from typing import Any, Dict, List, Optional, Tuple

from flask import Flask, jsonify, request, send_from_directory

from config_parser import SimulatorConfig, load_config
from channel_registry import validate_entries
import mqtt_data_exchange_codegen
from mqtt_message_binding import (
    custom_component_mapping,
    print_mapping,
    read_mapping_rows,
    save_mapping_rows,
    sensor_state_bit_count,
)
from metrics_store import MetricsStore
import generate_frame_type

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_static")


def _component_catalog(config: SimulatorConfig) -> Dict[str, Any]:
    """Every id an io_mapping channel can include, for the editor's
    component picker - `pack_format` is purely informational (a badge next
    to the id): it's already decided by the component's own config
    (Sensor/ControlButton/LedIndicator.packFormat), not something this
    catalog offers as a choice. Adding an id to a channel and letting
    custom_component_mapping() (validate_entries in channel_registry.py)
    build the actual ByteType/padding is the only path - there is no
    per-field type picker here."""
    components = [{"id": "clock", "type": "clock", "pack_format": None, "bits": None}]
    for sensor in config.sensors:
        components.append({
            "id": sensor.id, "type": "sensor", "pack_format": sensor.packFormat,
            "bits": sensor_state_bit_count(sensor),
        })
    for button in config.buttons:
        components.append({"id": button.id, "type": "button", "pack_format": button.packFormat, "bits": 1})
    for led in config.leds:
        components.append({"id": led.id, "type": "led", "pack_format": led.packFormat, "bits": 1})
    for belt in config.belts:
        components.append({"id": belt.id, "type": "belt", "pack_format": None, "bits": None})
    return {"components": components}


def _resolve(base_dir: str, path: str) -> str:
    return path if os.path.isabs(path) else os.path.join(base_dir, path)


def create_app(
    config_path: Optional[str] = None,
    registry_path: Optional[str] = None,
    metrics_store: Optional[MetricsStore] = None,
) -> Flask:
    app = Flask(__name__, static_folder=None)
    app.config.update(CONFIG_PATH=config_path, REGISTRY_PATH=registry_path, METRICS_STORE=metrics_store)

    def _load_config() -> Tuple[Optional[SimulatorConfig], Optional[Tuple[Any, int]]]:
        path = app.config["CONFIG_PATH"]
        if not path:
            return None, (jsonify(error="server was started without --config"), 400)
        try:
            return load_config(path), None
        except Exception as exc:
            return None, (jsonify(error=f"failed to load {path}: {exc}"), 400)

    # ---------------------------------------------------------------- pages

    @app.get("/")
    def index():
        return send_from_directory(STATIC_DIR, "index.html")

    @app.get("/static/<path:filename>")
    def static_files(filename):
        return send_from_directory(STATIC_DIR, filename)

    # ------------------------------------------------------- registry editor

    @app.get("/api/config")
    def api_config():
        config, err = _load_config()
        if err:
            return err
        return jsonify(_component_catalog(config))

    @app.get("/api/registry")
    def api_get_registry():
        registry_path = app.config["REGISTRY_PATH"]
        config, _ = _load_config()  # optional here - only needed to preview component_ids-driven channels

        if registry_path is None:
            # No channels.json configured - fall back to what's actually
            # running: the hardcoded PUBLISH_TOPICS/SUBSCRIBE_TOPICS/
            # METRICS_CHANNELS in mqtt_data_exchange.py, so the editor never
            # opens to a blank page just because nobody's migrated to a
            # registry file yet.
            channels = mqtt_data_exchange_codegen.read_hardcoded_channels()
            if config is not None:
                for entry in channels:
                    if entry["kind"] == "io_mapping":
                        mapping = custom_component_mapping(config, entry["component_ids"])
                        entry["rows"] = [{"byte_type": bt.name, "path": p} for bt, p in mapping]
            return jsonify({
                "path": mqtt_data_exchange_codegen.default_target_file(),
                "source": "hardcoded",
                "channels": channels,
            })

        channels: List[Dict[str, Any]] = []
        if os.path.exists(registry_path):
            with open(registry_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            base_dir = os.path.dirname(os.path.abspath(registry_path))
            for entry in data.get("channels", []):
                component_ids = entry.get("component_ids")
                rows: List[Tuple[str, str]] = []
                if component_ids is not None and config is not None:
                    # The mapping is entirely computed from config (each
                    # component's own packFormat) - never read back from
                    # the CSV, which is just this computation's last output.
                    rows = [(bt.name, p) for bt, p in custom_component_mapping(config, component_ids)]
                else:
                    csv_path = entry.get("mapping_csv", "")
                    if csv_path:
                        try:
                            rows = read_mapping_rows(_resolve(base_dir, csv_path))
                        except (OSError, ValueError):
                            rows = []
                channels.append({**entry, "rows": [{"byte_type": bt, "path": p} for bt, p in rows]})

        return jsonify({"path": registry_path, "channels": channels})

    @app.post("/api/validate")
    def api_validate():
        config, err = _load_config()
        if err:
            return err
        body = request.get_json(force=True, silent=True) or {}
        results = validate_entries(body.get("channels", []), config)
        return jsonify({"results": results})

    @app.post("/api/registry")
    def api_save_registry():
        registry_path = app.config["REGISTRY_PATH"]
        config, err = _load_config()
        if err:
            return err

        body = request.get_json(force=True, silent=True) or {}
        entries = body.get("channels", [])
        results = validate_entries(entries, config)
        if any(not r["ok"] for r in results):
            return jsonify({"saved": False, "results": results}), 400

        if registry_path is None:
            # No channels.json configured - write straight back into
            # mqtt_data_exchange.py's own PUBLISH_TOPICS/SUBSCRIBE_TOPICS/
            # METRICS_CHANNELS (see mqtt_data_exchange_codegen.py for the
            # safety gates: compiled, re-validated, and backed up before a
            # single byte of the real file changes).
            try:
                mqtt_data_exchange_codegen.write_hardcoded_channels(entries, config)
            except mqtt_data_exchange_codegen.CodegenError as exc:
                return jsonify({"saved": False, "results": results, "error": str(exc)}), 400
            return jsonify({
                "saved": True, "results": results,
                "message": (
                    f"Wrote {mqtt_data_exchange_codegen.default_target_file()} "
                    "(previous version backed up as .bak). This server (and anything else "
                    "that already imported mqtt_data_exchange) needs to restart before it "
                    "runs with these definitions."
                ),
            })

        base_dir = os.path.dirname(os.path.abspath(registry_path)) or "."
        os.makedirs(base_dir, exist_ok=True)

        saved_channels = []
        for entry in entries:
            csv_path = entry.get("mapping_csv") or f"{entry['name']}.csv"
            component_ids = entry.get("component_ids")
            if component_ids is not None:
                # Regenerated from config, not from whatever rows the
                # client happened to send - component_ids is the only
                # thing a component_ids-driven channel actually authors.
                mapping = custom_component_mapping(config, component_ids)
                print_mapping(mapping, _resolve(base_dir, csv_path))
            else:
                rows = [(row.get("byte_type", ""), row.get("path", "")) for row in entry.get("rows", [])]
                save_mapping_rows(rows, _resolve(base_dir, csv_path))

            saved_entry = {
                "name": entry["name"], "kind": entry["kind"], "direction": entry["direction"],
                "topic": entry["topic"], "interval_ms": entry.get("interval_ms", 100),
                "mapping_csv": csv_path,
            }
            if entry.get("frame_type"):
                saved_entry["frame_type"] = entry["frame_type"]
            if component_ids is not None:
                saved_entry["component_ids"] = component_ids
            saved_channels.append(saved_entry)

        with open(registry_path, "w", encoding="utf-8") as f:
            json.dump({"channels": saved_channels}, f, indent=2)
            f.write("\n")

        return jsonify({"saved": True, "results": results})

    @app.post("/api/frame-types/generate")
    def api_generate_frame_types():
        """The web counterpart to `python generate_frame_type.py --registry
        ...`: append a FrameType member for each given name that doesn't
        already have one. Calls the exact same add_missing_members() the
        CLI does - this endpoint only supplies the names and the target
        file, it doesn't re-decide what's missing or what value it gets.

        The file is a real Python source file, imported once at process
        start - writing to it here does not change what FrameType already
        is in THIS running server. Every response says so plainly rather
        than implying the fix is already live.
        """
        body = request.get_json(force=True, silent=True) or {}
        names = [n for n in (body.get("names") or []) if n]
        if not names:
            return jsonify(error="no frame_type names given"), 400

        target_file = generate_frame_type.default_target_file()
        try:
            added = generate_frame_type.add_missing_members(target_file, names)
        except OSError as exc:
            return jsonify(error=f"failed to update {target_file}: {exc}"), 500
        except RuntimeError as exc:
            return jsonify(error=str(exc)), 500

        if not added:
            return jsonify(added={}, message="Already defined in FrameType - nothing to add.")

        added_str = ", ".join(f"{name} = {value}" for name, value in added.items())
        return jsonify(
            added=added,
            message=(
                f"Added {added_str} to {os.path.basename(target_file)}. "
                "This server (and anything else that already imported FrameType) "
                "needs to restart before it recognizes the new value."
            ),
        )

    # -------------------------------------------------------- live metrics

    @app.get("/api/metrics/tree")
    def api_metrics_tree():
        store = app.config["METRICS_STORE"]
        if store is None:
            return jsonify(error="no live simulator connected"), 503
        return jsonify({
            group: {name: store.kind_of(group, name) for name in store.names(group)}
            for group in store.groups()
        })

    @app.get("/api/metrics/latest")
    def api_metrics_latest():
        store = app.config["METRICS_STORE"]
        if store is None:
            return jsonify(error="no live simulator connected"), 503
        rows = store.latest_rows(group=request.args.get("group") or None)
        return jsonify([{"ts": ts, "group": g, "name": n, "kind": k, "value": v} for ts, g, n, k, v in rows])

    @app.get("/api/metrics/series")
    def api_metrics_series():
        store = app.config["METRICS_STORE"]
        if store is None:
            return jsonify(error="no live simulator connected"), 503
        group = request.args.get("group", "")
        name = request.args.get("name", "")
        since = float(request.args.get("since") or 0)
        samples = store.series(group, name)
        return jsonify([{"ts": s.ts, "value": s.value} for s in samples if s.ts > since])

    return app


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the channel registry web editor (and metrics dashboard, if embedded)."
    )
    parser.add_argument("--config", help="Simulator config JSON (e.g. sim_config.json)")
    parser.add_argument("--registry", help="channels.json to edit - created on first save if missing")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5050)
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    app = create_app(config_path=args.config, registry_path=args.registry)
    app.run(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
