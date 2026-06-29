import argparse
import time
import pygame
from config_parser import load_config
from visualizer import Renderer
from physics import PhysicsEngine
from mqtt_data_exchange import MQTTSyncEngine, apply_exchange_config

def main():
    parser = argparse.ArgumentParser(description="Python Conveyor Flow Simulator")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    parser.add_argument("--exchange-config", help="Optional JSON file overriding MQTT/TCP exchange settings")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (no UI)")
    args = parser.parse_args()

    print(f"Loading configuration from {args.config}...")
    config = load_config(args.config)
    if args.exchange_config:
        apply_exchange_config(config, args.exchange_config)
        print(f"Loaded exchange settings from {args.exchange_config}.")
    print(f"Loaded {len(config.belts)} belts, {len(config.sensors)} sensors.")

    print(f"Initializing visualizer (Headless: {args.headless})...")
    renderer = Renderer(config, headless=args.headless)
    physics_engine = PhysicsEngine(config)

    mqtt = MQTTSyncEngine(config)
    print("Starting simulation loop. Press Ctrl+C to exit.")
    running = True
    clock = pygame.time.Clock()

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
            
            if renderer.playing:
                physics_engine.update(dt)
            
            # Render
            renderer.render(physics_engine)
            mqtt.sync()
            
            if args.headless:
                # In headless, we should throttle logic manually since pygame.clock.tick is skipped
                time.sleep(1/60.0) 
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        mqtt.stop()
        renderer.quit()

if __name__ == "__main__":
    main()
