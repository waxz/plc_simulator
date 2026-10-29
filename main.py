import argparse
import threading
import time
import pygame
from config_parser import load_config
from visualizer import FRAME_LOCKED_MAX_SPEED, Renderer
from physics import PhysicsEngine
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config
from metrics_store import MetricsStore
from sim_runner import SimulationRunner
import view3d


def run_pygame(config, physics_engine, mqtt, headless: bool, runner=None, config_path=None,
               speed=1.0, render_fps: int = 30, sim_lock=None, link=None, source_path=None):
    """The window loop. Returns a layout path when the designer asked this
    simulator to reload (sim_link.py), else None."""
    print(f"Initializing visualizer (Headless: {headless})...")
    renderer = Renderer(config, headless=headless)
    renderer.speed = speed
    renderer.fps = render_fps
    if sim_lock is not None and runner is not None:
        renderer.sim_lock = sim_lock   # draw the scene while the runner thread isn't mid-step
    renderer.exchange = mqtt  # the status bar shows the PLC link
    renderer.physics = physics_engine  # Clear Parcels / Reset
    renderer.config_path = config_path
    renderer.source_path = source_path   # Edit in Designer: the layout file (config_path may be a run copy)
    if headless:
        # There's no Play button to press without a window.
        renderer.playing = True
    print("Starting simulation loop. Press Ctrl+C to exit.")
    running = True
    clock = pygame.time.Clock()

    # Console fallback for the visual "Sim/Real" indicator (visualizer.py's
    # overlay) - the one place that still verifies timing when running
    # --headless, where there's no on-screen overlay to look at.
    last_timing_report = time.perf_counter()
    reload_path = None

    try:
        while running:
            dt = clock.tick(render_fps) / 1000.0
            if link is not None:
                reload_path = link.take_reload()
                if reload_path:
                    break
            # Event processing
            # TODO:   dt pass to process_events
            cmd = renderer.process_events()
            if cmd == 'QUIT':
                running = False
            elif cmd == 'TOGGLE_PLAY':
                print(f"Simulation {'Playing' if renderer.playing else 'Paused'}")
            elif cmd == 'RESET':
                print("Simulation reset: parcels cleared, time and counters back to 0")
            elif cmd == 'CLEAR':
                print("Parcels cleared")

            if hasattr(physics_engine, "poll"):
                # Simulation process (sim_process.py): take its newest
                # snapshot, forward play / speed.
                physics_engine.poll()
                physics_engine.set_playing(renderer.playing)
                physics_engine.set_speed(renderer.speed)
                if not physics_engine.alive:
                    print("[main] The simulation process stopped.")
                    running = False
            elif runner is not None:
                # Physics + exchange run on the runner's own thread; this
                # loop only renders and forwards play/pause.
                runner.playing = renderer.playing
                runner.speed = renderer.speed
            else:
                if renderer.playing:
                    # Frame-locked (--no-sim-thread): physics advances once per
                    # frame, so "max" can only mean a fixed factor here.
                    physics_engine.update(dt * (renderer.speed or FRAME_LOCKED_MAX_SPEED))
                mqtt.sync()

            renderer.render(physics_engine)
            view3d.pump(renderer, physics_engine)
            if headless:
                # No display: nothing else paces this loop.
                time.sleep(1 / 60.0)

            play_real = renderer.track_play_time()  # wall time spent playing, not since launch
            now = time.perf_counter()
            if now - last_timing_report >= 5.0:
                real_elapsed = play_real
                ratio = physics_engine.sim_time / real_elapsed if real_elapsed > 0 else 0.0
                print(f"[TIMING] speed={'max' if renderer.speed is None else f'{renderer.speed:g}x'} "
                      f"sim={physics_engine.sim_time:.1f}s real={real_elapsed:.1f}s ratio={ratio:.3f}x "
                      f"dropped={physics_engine.dropped_time:.2f}s exchange={mqtt.stats()}")
                last_timing_report = now
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        renderer.quit()
    return reload_path


def build_engine(config, config_lock, metrics_store, channel_registry_path=None, show_layout=False):
    """The one place that decides whether the exchange server uses the
    hardcoded PUBLISH_TOPICS/SUBSCRIBE_TOPICS/METRICS_CHANNELS or a channel
    registry file - passing channel_registry_path straight through to
    MQTTSyncEngine is the entire fix for "a channel I defined in the web
    editor is invisible to the running simulator": without it, a frame
    tagged with a frame_type the registry declares but the hardcoded dicts
    don't is silently dropped by _handle_tcp_payload, registry or no
    registry, web UI or no web UI."""
    return MQTTSyncEngine(config, lock=config_lock, metrics_store=metrics_store,
                          channel_registry_path=channel_registry_path, show_layout=show_layout)


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
    parser.add_argument("--show-layout", action="store_true",
                        help="Print each I/O channel's byte layout (offsets, types, PLC types) at start")
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
        "--sim-thread", action=argparse.BooleanOptionalAction, default=True,
        help="Run physics and the PLC exchange on their own thread (sim_runner.py), decoupled from "
             "rendering: the window draws at --render-fps while the simulation keeps its speed and the "
             "exchange channels their interval (default on; --no-sim-thread steps physics once per frame). "
             "Always on with --headless.",
    )
    parser.add_argument(
        "--sim-process", action=argparse.BooleanOptionalAction, default=True,
        help="Run the simulation (physics, PLC exchange, web UI) in its own process, with a CPU core of "
             "its own, and only the window here (default on; --no-sim-process keeps it on a thread of this "
             "process). Not with --headless.",
    )
    parser.add_argument("--sim-tick-ms", type=float, default=2.0, help="Tick period of the simulation thread (default 2 ms)")
    parser.add_argument(
        "--speed", default="1",
        help="Simulation speed: a real-time factor (1 = real time, 4 = four times faster) or 'max' "
             "(as fast as the host can). Also the Speed button / [ ] keys in the window.",
    )
    parser.add_argument("--view3d", nargs="?", const="open", choices=("open", "serve"),
                        help="The 3D view of the simulation (view3d.py): 'open' (the default) serves it and opens "
                             "it in the browser, 'serve' only serves it (headless / another PC via a tunnel). "
                             "Also the window's 3D View button")
    parser.add_argument("--view3d-port", type=int, default=view3d.DEFAULT_PORT,
                        help=f"Port of the 3D view on 127.0.0.1 (default {view3d.DEFAULT_PORT})")
    parser.add_argument("--source", help="The layout file this one is a run copy of (the designer's Run in "
                                         "Simulator): Edit in Designer opens that file")
    parser.add_argument("--render-fps", type=int, default=30,
                        help="Window frame rate (default 30); the simulation doesn't wait for it")
    return parser


def run_with_simulation_process(args, config, speed, link=None):
    """The window here, the simulation (physics, PLC exchange, web UI) in a
    child process with a CPU core of its own - see sim_process.py."""
    from sim_process import PhysicsProxy
    settings = {"config": args.config, "exchange_config": args.exchange_config,
                "channel_registry": args.channel_registry, "show_layout": args.show_layout,
                "web_ui": args.web_ui, "web_host": args.web_host, "web_port": args.web_port,
                "tick_s": args.sim_tick_ms / 1000.0, "speed": speed}
    proxy = PhysicsProxy(config, settings)
    print("[main] Starting the simulation process...")
    proxy.start()
    print(f"[main] Physics + exchange in their own process (pid {proxy.process.pid}, speed "
          f"{'max' if speed is None else f'{speed:g}x'}); the window renders at {args.render_fps} fps.")
    try:
        return run_pygame(config, proxy, proxy.exchange, False, config_path=args.config,
                          source_path=getattr(args, "source", None), speed=speed,
                          render_fps=args.render_fps, link=link)
    finally:
        proxy.stop()


def main():
    args = build_argparser().parse_args()
    try:
        speed = None if str(args.speed).lower() == "max" else float(args.speed)
    except ValueError:
        raise SystemExit(f"--speed must be a number or 'max', not {args.speed!r}")
    if speed is not None and speed <= 0:
        raise SystemExit("--speed must be > 0")

    # One simulator window: the designer's Run in Simulator asks this one to
    # reload the saved layout instead of opening another (sim_link.py).
    link = None
    if not args.headless:
        from sim_link import SimulatorLink
        link = SimulatorLink(args.config, args.source)
    try:
        while True:
            reload_path = run_layout(args, speed, link)
            if not reload_path:
                break
            print(f"[main] Reloading {reload_path} (Run in Simulator from the designer)...")
            args.config = reload_path
            args.source = link.pending_source
            link.set_config(reload_path, args.source)
    finally:
        if link is not None:
            link.close()


_web_ui_started = False


def run_layout(args, speed, link=None):
    """Load args.config and run it until the window closes. Returns a layout
    path to run next (the designer asked for a reload), else None."""
    global _web_ui_started
    print(f"Loading configuration from {args.config}...")
    config = load_config(args.config)
    view3d.server(args.view3d_port)
    if args.view3d and not view3d.SERVER.running:
        view3d.open_view(config, args.config, browser=args.view3d == "open", port=args.view3d_port)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)
        print(f"Loaded exchange settings from {args.exchange_config}.")
    print(f"Loaded {len(config.belts)} belts, {len(config.sensors)} sensors.")

    if args.sim_process and not args.headless:
        return run_with_simulation_process(args, config, speed, link)

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
    mqtt = build_engine(config, config_lock, metrics_store, channel_registry_path=args.channel_registry,
                        show_layout=args.show_layout)

    if args.web_ui and not _web_ui_started:          # its thread outlives a reload
        _web_ui_started = True
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
        runner = SimulationRunner(physics_engine, mqtt, tick_s=args.sim_tick_ms / 1000.0, speed=speed)
        runner.playing = False  # the GUI's play state is forwarded every frame
        runner.start()
        print(f"[main] Physics + exchange on their own thread (tick {args.sim_tick_ms:g} ms, speed "
              f"{'max' if speed is None else f'{speed:g}x'}); the window renders at {args.render_fps} fps.")

    try:
        return run_pygame(config, physics_engine, mqtt, args.headless, runner=runner, config_path=args.config,
                          source_path=getattr(args, "source", None),
                          speed=speed, render_fps=args.render_fps, sim_lock=config_lock, link=link)
    finally:
        if runner is not None:
            runner.stop()
        mqtt.stop()


if __name__ == "__main__":
    main()
