import time
import threading
from urllib.parse import urlparse
import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion

class FlexibleProtocolMQTTManager:
    def __init__(self, broker_url, port=None, keepalive=60):
        """
        Accepts full broker URLs and an optional explicit port overwrite.
        - Url examples: 'mqtt://://hivemq.com', 'ws://://hivemq.com/mqtt'
        - Explicit Port overwrite: If `port` parameter is passed, it overrides everything.
        """
        self.keepalive = keepalive
        self.topic_callbacks = {}
        self.is_running = False
        self.worker_thread = None

        # 1. Parse the broker URL into components
        parsed_url = urlparse(broker_url)
        
        # Fallback defaults if the user omitted schemes
        scheme = parsed_url.scheme if parsed_url.scheme else "mqtt"
        self.host = parsed_url.hostname if parsed_url.hostname else broker_url
        
        # 2. Determine transport protocol
        if scheme in ["ws", "wss"]:
            self.transport = "websockets"
            self.ws_path = parsed_url.path if parsed_url.path else "/mqtt"
            default_port = 443 if scheme == "wss" else 80
        else:
            self.transport = "tcp"
            self.ws_path = None
            default_port = 8883 if scheme == "mqtts" else 1883

        # 3. Determine the final port (Explicit parameter > URL port > Scheme default)
        if port is not None:
            self.port = int(port)
        elif parsed_url.port is not None:
            self.port = int(parsed_url.port)
        else:
            self.port = default_port

        # 4. Initialize client with the dynamic transport type
        self.client = mqtt.Client(
            callback_api_version=CallbackAPIVersion.VERSION2,
            transport=self.transport
        )
        
        if self.transport == "websockets" and self.ws_path:
            self.client.ws_set_options(path=self.ws_path)

        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def register_callback(self, topic, callback_function):
        self.topic_callbacks[topic] = callback_function
        if self.client.is_connected():
            self.client.subscribe(topic, qos=1)

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"[MQTT] Connected via [{self.transport.upper()}] to {self.host}:{self.port}")
            for topic in self.topic_callbacks.keys():
                self.client.subscribe(topic, qos=1)
        else:
            print(f"[MQTT] Connection failed with code: {rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
        except UnicodeDecodeError:
            payload = msg.payload
        if msg.topic in self.topic_callbacks:
            self.topic_callbacks[msg.topic](payload)

    def _thread_loop(self):
        try:
            self.client.connect(self.host, self.port, self.keepalive)
            self.client.loop_start()
            while self.is_running:
                time.sleep(0.1)
        except Exception as e:
            print(f"[MQTT Error] Thread crashed: {e}")
        finally:
            self.client.loop_stop()
            self.client.disconnect()

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._thread_loop, daemon=True)
        self.worker_thread.start()

    def publish(self, topic, payload, qos=1):
        if self.client.is_connected():
            self.client.publish(topic, payload, qos=qos)

    def stop(self):
        self.is_running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=3)
