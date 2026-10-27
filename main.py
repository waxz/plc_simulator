import argparse
import threading
import time
import pygame
from config_parser import load_config
from visualizer import Renderer
from physics import PhysicsEngine
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config
from metrics_store import MetricsStore
from sim_runner import SimulationRunner


def run_pygame(config, physics_engine, mqtt, headless: bool, runner=None):
    print(f"Initializing visualizer (Headless: {headless})...")
    renderer = Renderer(config, headless=headless)
    if headless:
        # There's no Play button to press without a window.
        renderer.playing = True
    print("Starting simulation loop. Press Ctrl+C to exit.")
    running = True
    clock = pygame.time.Clock()

    # Console fallback for the visual "Sim/Real" indicator (visualizer.py's
    # overlay) - the one place that still verifies timing when running
    # --headless, where there's no on-screen overlay to look at.
    real_start = time.perf_counter()
    last_timing_report = real_start

    try:
        while running:
            dt = clock.tick(60) / 1000.0
            # Event processing
            # TODO:   dt pass to process_events
            cmd = renderer.process_events()
            if cmd == 'QUIT':
                running = False
            elif cmd == 'TOGGLE_PLAY':
                print(f"Simulation {'Playing' if renderer.playing else 'Paused'}")
            elif cmd == 'RESET':
                print("Simulation Reset (TODO)")
            elif cmd == 'CLEAR':
                print("Simulation Cleared (TODO)")

            if runner is not None:
                # Physics + exchange run on the runner's own thread; this
                # loop only renders and forwards play/pause.
                runner.playing = renderer.playing
            else:
                if renderer.playing:
                    physics_engine.update(dt)
                mqtt.sync()

            renderer.render(physics_engine)
            if headless:
                # No display: nothing else paces this loop.
                time.sleep(1 / 60.0)

            now = time.perf_counter()
            if now - last_timing_report >= 5.0:
                real_elapsed = now - real_start
                ratio = physics_engine.sim_time / real_elapsed if real_elapsed > 0 else 0.0
                print(f"[TIMING] sim={physics_engine.sim_time:.1f}s real={real_elapsed:.1f}s ratio={ratio:.3f}x "
                      f"dropped={physics_engine.dropped_time:.2f}s exchange={mqtt.stats()}")
                last_timing_report = now
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        renderer.quit()


def build_engine(config, config_lock, metrics_store, channel_registry_path=None):
    """The one place that decides whether the exchange server uses the
    hardcoded PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS or a channel
    registry file - passing channel_registry_path straight through to
    MQTTSyncEngine is the entire fix for "a channel I defined in the web
    editor is invisible to the running simulator": without it, a frame
    tagged with a frame_type the registry declares but the hardcoded dicts
    don't is silently dropped by _handle_tcp_payload, registry or no
    registry, web UI or no web UI."""
    return MQTTSyncEngine(config, lock=config_lock, metrics_store=metrics_store, channel_registry_path=channel_registry_path)


def build_web_app(config_path, channel_registry_path, metrics_store):
    """The channel registry editor + Live Data dashboard, sharing the SAME
    metrics_store the exchange engine (build_engine above) writes into -
    this is what makes a frame the engine just parsed show up in the Live
    Data tab. Returns a plain Flask app (no server started) so this stays
    trivially testable via Flask's own test client - start_web_ui() below
    is the only place that actually opens a socket."""
    from web_app import create_app
    return create_app(config_path=config_path, registry_path=channel_registry_path, metrics_store=metrics_store)


def start_web_ui(config_path, channel_registry_path, metrics_store, host="127.0.0.1", port=5050):
    """build_web_app() plus actually serving it, on a background daemon
    thread so it never blocks (or is blocked by) the pygame/Qt loop
    running on the main thread. use_reloader=False matters here - the
    reloader spawns a child process, which would re-run this whole
    module's main() a second time."""
    app = build_web_app(config_path, channel_registry_path, metrics_store)
    thread = threading.Thread(
        target=lambda: app.run(host=host, port=port, use_reloader=False),
        daemon=True,
    )
    thread.start()
    print(f"[WebUI] Channel registry editor + Live Data at http://{host}:{port}")
    return app, thread


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Python Conveyor Flow Simulator")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    parser.add_argument("--exchange-config", help="Optional JSON file overriding MQTT/TCP exchange settings")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (no window)")
    parser.add_argument(
        "--channel-registry",
        help="channels.json to load IO-mapping/metric channels from (see channel_registry.py) - "
             "also what --web-ui edits and validates against. Omit to use the hardcoded "
             "PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS in mqtt_data_exchange.py, unchanged.",
    )
    parser.add_argument(
        "--web-ui", action="store_true",
        help="Serve the channel registry editor and live metrics dashboard (web_app.py) "
             "alongside the simulator, sharing its live MetricsStore.",
    )
    parser.add_argument("--web-host", default="127.0.0.1", help="Host for --web-ui (default 127.0.0.1)")
    parser.add_argument("--web-port", type=int, default=5050, help="Port for --web-ui (default 5050)")
    parser.add_argument(
        "--sim-thread", action="store_true",
        help="Run physics and the PLC exchange on a dedicated thread (see sim_runner.py) instead of once "
             "per GUI frame, so exchange channels keep their configured interval (e.g. 10 ms). "
             "Always on with --headless.",
    )
    parser.add_argument("--sim-tick-ms", type=float, default=2.0, help="Tick period for --sim-thread (default 2 ms)")
    return parser


def main():
    args = build_argparser().parse_args()

    print(f"Loading configuration from {args.config}...")
    config = load_config(args.config)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)
        print(f"Loaded exchange settings from {args.exchange_config}.")
    print(f"Loaded {len(config.belts)} belts, {len(config.sensors)} sensors.")

    # Shared by both: physics mutates config.belts/config.sensors on this
    # (main) thread every update(), while MQTTSyncEngine reads/writes those
    # same fields from background TCP client threads - one lock, passed to
    # both, is what actually makes that safe (previously PhysicsEngine had
    # no lock of its own at all).
    config_lock = threading.Lock()
    physics_engine = PhysicsEngine(config, lock=config_lock)
    # Client-reported metrics storage, read by the web UI's Live Data tab.
    # MQTTSyncEngine's own metrics channel (see METRICS_MAPPING in
    # mqtt_data_exchange.py) feeds it over the SAME MQTT/TCP connection as
    # the rest of the exchange - no separate server/port for metrics.
    metrics_store = MetricsStore()
    mqtt = build_engine(config, config_lock, metrics_store, channel_registry_path=args.channel_registry)

    if args.web_ui:
        start_web_ui(args.config, args.channel_registry, metrics_store, host=args.web_host, port=args.web_port)
        if args.channel_registry is None:
            print(
                "[WebUI] --channel-registry was not given: the editor is showing/editing the "
                "hardcoded PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS in mqtt_data_exchange.py "
                "directly (see mqtt_data_exchange_codegen.py) - saving rewrites that file in place, "
                "backing up the previous version as .bak. A restart is still needed for THIS process "
                "(or any other already-running one) to pick up the change."
            )

    runner = None
    if args.sim_thread or args.headless:
        runner = SimulationRunner(physics_engine, mqtt, tick_s=args.sim_tick_ms / 1000.0)
        runner.playing = False  # the GUI's play state is forwarded every frame
        runner.start()
        print(f"[main] Physics + exchange on a dedicated thread, tick {args.sim_tick_ms:g} ms.")

    try:
        run_pygame(config, physics_engine, mqtt, args.headless, runner=runner)
    finally:
        if runner is not None:
            runner.stop()
        mqtt.stop()


if __name__ == "__main__":
    main()
