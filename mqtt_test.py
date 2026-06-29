import paho.mqtt.client as mqtt
import time
import sys

def on_connect(client, userdata, flags, reason_code, properties=None):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("sim/test_topic")

def on_message(client, userdata, msg):
    print(f"Received message '{msg.payload.decode()}' on topic '{msg.topic}'")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

broker = "broker.emqx.io"
port = 1883
print(f"Connecting to broker {broker}:{port}...")

try:
    client.connect(broker, port, 60)
except Exception as e:
    print(f"Failed to connect to {broker}: {e}")
    sys.exit(1)

client.loop_start()

# Wait for connection
time.sleep(1)

# Publish a test message
print("Publishing test message...")
client.publish("sim/test_topic", "Hello from Python Simulator")

# Wait for message to be received
time.sleep(2)

client.loop_stop()
client.disconnect()
print("Done.")
