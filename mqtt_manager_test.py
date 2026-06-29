"""
mqtt_manager_test.py  -  Integration test for the MQTT payload pipeline.

Workflow per scenario:
  1. Apply a set of "realtime" overrides to config_sender.
  2. Serialize config_sender -> binary payload -> publish to MQTT broker.
  3. Broker echoes the message back (loopback via self-subscribe).
  4. Deserialize received payload -> update config_receiver.
  5. Verify each overridden field in config_receiver matches config_sender.
"""

import sys
import time
import threading
import argparse

from mqtt_manager import FlexibleProtocolMQTTManager
from config_parser import load_config
from mqtt_message_binding import (
    ComponentMessage,
    custom_component_mapping,
    verify_component_list,
)
from struct_utils import print_byte_by_byte


# ---------------------------------------------------------------------------
# Scenario definitions
# Each scenario is {label, overrides} where overrides = {comp_id: {field: val}}.
# They are applied to config_sender before each publish cycle so we can
# validate that every distinct realtime state round-trips correctly.
# ---------------------------------------------------------------------------
SCENARIOS = [
    {
        "label": "All sensors ACTIVE / belt_1 running",
        "overrides": {
            "sensor_1": {"sensor_states": "1" * 32},
            "sensor_2": {"sensor_states": "1"},
            "button_0": {"sensor_states": True},
            "led_1":    {"isOn": True},
            "belt_1":   {"speed": 3.5,  "directionAngle":   0.0},
            "belt_2":   {"speed": 0.0,  "directionAngle":  90.0},
        },
    },
    {
        "label": "All sensors IDLE / both belts stopped",
        "overrides": {
            "sensor_1": {"sensor_states": "0" * 32},
            "sensor_2": {"sensor_states": "0"},
            "button_0": {"sensor_states": False},
            "led_1":    {"isOn": False},
            "belt_1":   {"speed": 0.0,  "directionAngle":   0.0},
            "belt_2":   {"speed": 0.0,  "directionAngle":   0.0},
        },
    },
    {
        "label": "Partial detection / belt_2 reverse",
        "overrides": {
            "sensor_1": {"sensor_states": "10101010" * 4},
            "sensor_2": {"sensor_states": "0"},
            "button_0": {"sensor_states": False},
            "led_1":    {"isOn": True},
            "belt_1":   {"speed": 1.2,  "directionAngle":  45.0},
            "belt_2":   {"speed": 2.8,  "directionAngle": -45.0},
        },
    },
]

TOPIC_CONVEYOR = "conveyor/payload"

# ---------------------------------------------------------------------------
# Shared state between publish thread and MQTT callback thread
# ---------------------------------------------------------------------------
_lock = threading.Lock()
_last_received_payload: bytes | None = None
_receive_event = threading.Event()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def apply_overrides(config, overrides: dict) -> None:
    """Apply {comp_id: {field: value}} overrides to config in-place."""
    lookup = {
        c.id: c
        for c in (
            list(config.sensors)
            + list(config.belts)
            + list(config.buttons)
            + list(config.leds)
        )
    }
    for comp_id, fields in overrides.items():
        obj = lookup.get(comp_id)
        if obj is None:
            print(f"  [WARN] component '{comp_id}' not found in config")
            continue
        for field, val in fields.items():
            setattr(obj, field, val)


def verify(config_sender, config_receiver, overrides: dict) -> bool:
    """
    For every field listed in overrides, compare sender vs receiver value.
    Returns True only if all fields match.
    """
    s_lookup = {
        c.id: c
        for c in (
            list(config_sender.sensors)
            + list(config_sender.belts)
            + list(config_sender.buttons)
            + list(config_sender.leds)
        )
    }
    r_lookup = {
        c.id: c
        for c in (
            list(config_receiver.sensors)
            + list(config_receiver.belts)
            + list(config_receiver.buttons)
            + list(config_receiver.leds)
        )
    }

    ok = True
    for comp_id, fields in overrides.items():
        s_obj = s_lookup.get(comp_id)
        r_obj = r_lookup.get(comp_id)
        if s_obj is None or r_obj is None:
            continue
        for field in fields:
            sv = getattr(s_obj, field, None)
            rv = getattr(r_obj, field, None)
            matched = (
                (abs(sv - rv) < 1e-4) if isinstance(sv, float) else (sv == rv)
            )
            status = "[OK]" if matched else "[FAIL]"
            print(f"  {status}  {comp_id}.{field}:  sender={sv!r}  receiver={rv!r}")
            if not matched:
                ok = False
    return ok


# ---------------------------------------------------------------------------
# MQTT callback factory
# ---------------------------------------------------------------------------

def make_payload_handler(message: ComponentMessage, config_receiver):
    """Return a closure that deserializes binary payloads into config_receiver."""
    def handle_payload(raw):
        global _last_received_payload
        payload = raw if isinstance(raw, (bytes, bytearray)) else raw.encode()
        with _lock:
            _last_received_payload = payload
        message.deserialize(payload, config_receiver)
        _receive_event.set()
    return handle_payload


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MQTT Manager Integration Test")
    parser.add_argument("config", help="Path to the JSON workspace configuration")
    args = parser.parse_args()

    # Load two independent configs: one is the "realtime simulator", the other
    # is the "PLC / subscriber" whose state will be updated from MQTT messages.
    print(f"Loading configuration from {args.config}...")
    config_sender   = load_config(args.config)
    config_receiver = load_config(args.config)
    print(f"  Sender  : {len(config_sender.belts)} belts, "
          f"{len(config_sender.sensors)} sensors, "
          f"{len(config_sender.buttons)} buttons, "
          f"{len(config_sender.leds)} leds")
    print(f"  Receiver: {len(config_receiver.belts)} belts, "
          f"{len(config_receiver.sensors)} sensors, "
          f"{len(config_receiver.buttons)} buttons, "
          f"{len(config_receiver.leds)} leds")

    # Build the shared component mapping
    custom_ids = ["sensor_2", "button_0", "led_1", "sensor_1", "belt_1", "belt_2"]
    mapping = custom_component_mapping(config_sender, custom_ids)
    print(f"\nComponent mapping ({len(mapping)} entries):")
    for entry in mapping:
        print(f"  {entry}")
    verify_component_list(mapping, config_sender)

    message = ComponentMessage(mapping)

    # Connect to the broker (from test_config.json mqttSettings)
    broker_url  = config_sender.mqttSettings.brokerUrl
    broker_port = config_sender.mqttSettings.port
    print(f"\nConnecting to {broker_url}:{broker_port} ...")

    mqtt = FlexibleProtocolMQTTManager(broker_url=broker_url, port=broker_port)
    mqtt.register_callback(
        TOPIC_CONVEYOR,
        make_payload_handler(message, config_receiver)
    )
    mqtt.start()

    # Wait until the broker confirms connection (up to 8 s for WebSocket handshake)
    deadline = time.time() + 8.0
    while not mqtt.client.is_connected() and time.time() < deadline:
        time.sleep(0.1)
    if not mqtt.client.is_connected():
        print("[ERROR] Could not connect to MQTT broker within 8 s. Aborting.")
        mqtt.stop()
        sys.exit(1)
    print(f"[MQTT] Connected to {broker_url}:{broker_port}")
    time.sleep(0.2)   # let subscriptions propagate

    # -------------------------------------------------------------------------
    # Scenario loop
    # -------------------------------------------------------------------------
    passed = 0
    failed = 0

    try:
        for cycle, scenario in enumerate(SCENARIOS, start=1):
            label     = scenario["label"]
            overrides = scenario["overrides"]

            print(f"\n{'-'*60}")
            print(f"[Cycle {cycle}] {label}")
            print(f"{'-'*60}")

            # 1. Update sender config with the new "realtime" values
            apply_overrides(config_sender, overrides)

            # 2. Serialize -> publish
            payload = message.serialize(config_sender)
            print(f"\n>> Publishing payload ({len(payload)} bytes):")
            print_byte_by_byte(payload)

            _receive_event.clear()
            mqtt.publish(TOPIC_CONVEYOR, payload)

            # 3. Wait for the loopback echo from the broker (self-subscribe)
            received = _receive_event.wait(timeout=3.0)
            if not received:
                print("  [TIMEOUT] No message received within 3 s")
                failed += 1
                continue

            # 4. Verify receiver now matches sender for all overridden fields
            print("\n<< Received and deserialized. Verifying receiver config:")
            ok = verify(config_sender, config_receiver, overrides)
            if ok:
                print(f"  => Cycle {cycle} PASSED")
                passed += 1
            else:
                print(f"  => Cycle {cycle} FAILED")
                failed += 1

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        mqtt.stop()

    print(f"\n{'='*60}")
    print(f"Results: {passed} passed, {failed} failed  "
          f"(out of {len(SCENARIOS)} scenarios)")
    print(f"{'='*60}")
    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
