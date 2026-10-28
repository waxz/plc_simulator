"""
foxglove_ws_probe.py - a small Foxglove WebSocket client (standard library
only), to see what a Foxglove WebSocket server publishes without opening
Foxglove: it connects, lists the advertised channels, subscribes and prints
the messages (JSON decoded; other encodings as their size).

    python foxglove_ws_probe.py                          # ws://127.0.0.1:8765, all topics, 5 s
    python foxglove_ws_probe.py --topic /plc/lines --seconds 10
    python foxglove_ws_probe.py ws://192.168.0.10:8765 --list

The tests use FoxgloveProbe to check plc_foxglove_bridge.py end to end over
the real protocol (https://github.com/foxglove/ws-protocol).
"""
import argparse
import base64
import json
import os
import socket
import struct
import time
from typing import Dict, List, Optional, Tuple
from urllib.parse import urlparse

SUBPROTOCOLS = ("foxglove.sdk.v1", "foxglove.websocket.v1")
OP_MESSAGE_DATA = 0x01


class FoxgloveProbe:
    def __init__(self, url: str = "ws://127.0.0.1:8765", timeout: float = 5.0):
        u = urlparse(url)
        self.host, self.port = u.hostname or "127.0.0.1", u.port or 8765
        self.sock = socket.create_connection((self.host, self.port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = b""
        self.channels: Dict[int, dict] = {}     # channel id -> advertised channel
        self.subscriptions: Dict[int, int] = {}  # subscription id -> channel id
        self.server_info: Optional[dict] = None
        self._handshake()

    # -- WebSocket ------------------------------------------------------------------

    def _handshake(self):
        key = base64.b64encode(os.urandom(16)).decode()
        request = (f"GET / HTTP/1.1\r\nHost: {self.host}:{self.port}\r\nUpgrade: websocket\r\n"
                   f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
                   f"Sec-WebSocket-Protocol: {', '.join(SUBPROTOCOLS)}\r\n\r\n")
        self.sock.sendall(request.encode())
        while b"\r\n\r\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("closed during the WebSocket handshake")
            self._buf += chunk
        head, self._buf = self._buf.split(b"\r\n\r\n", 1)
        if b" 101 " not in head.split(b"\r\n", 1)[0]:
            raise ConnectionError(f"not a WebSocket server: {head.split(b'\r\n', 1)[0].decode(errors='replace')}")

    def _send_frame(self, opcode: int, payload: bytes):
        mask = os.urandom(4)
        n = len(payload)
        head = bytes([0x80 | opcode])
        if n < 126:
            head += bytes([0x80 | n])
        elif n < 65536:
            head += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            head += bytes([0x80 | 127]) + struct.pack(">Q", n)
        masked = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        self.sock.sendall(head + mask + masked)

    def _parse_frame(self) -> Optional[Tuple[int, bytes]]:
        """One complete frame from the buffer (consumed), else None - a
        read that times out half way through a frame loses nothing."""
        buf = self._buf
        if len(buf) < 2:
            return None
        opcode, n, pos = buf[0] & 0x0F, buf[1] & 0x7F, 2
        if n == 126:
            if len(buf) < 4:
                return None
            n, pos = struct.unpack_from(">H", buf, 2)[0], 4
        elif n == 127:
            if len(buf) < 10:
                return None
            n, pos = struct.unpack_from(">Q", buf, 2)[0], 10
        mask = None
        if buf[1] & 0x80:
            if len(buf) < pos + 4:
                return None
            mask, pos = buf[pos:pos + 4], pos + 4
        if len(buf) < pos + n:
            return None
        payload, self._buf = buf[pos:pos + n], buf[pos + n:]
        if mask:
            payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
        return opcode, payload

    def _recv_frame(self) -> Tuple[int, bytes]:
        while True:
            frame = self._parse_frame()
            if frame is None:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError("connection closed")
                self._buf += chunk
                continue
            opcode, payload = frame
            if opcode == 0x9:          # ping
                self._send_frame(0xA, payload)
                continue
            if opcode == 0x8:
                raise ConnectionError("server closed the connection")
            return opcode, payload

    # -- Foxglove protocol ------------------------------------------------------------

    def _handle_text(self, text: str):
        msg = json.loads(text)
        op = msg.get("op")
        if op == "serverInfo":
            self.server_info = msg
        elif op == "advertise":
            for ch in msg.get("channels", []):
                self.channels[ch["id"]] = ch
        elif op == "unadvertise":
            for cid in msg.get("channelIds", []):
                self.channels.pop(cid, None)

    def wait_for_topics(self, topics, timeout: float = 5.0) -> Dict[str, dict]:
        """Read until every topic is advertised - with no topics, until the
        first advertisement (or timeout); {topic: channel}."""
        end = time.monotonic() + timeout

        def done():
            advertised = [c["topic"] for c in self.channels.values()]
            if not topics:
                return bool(advertised)
            return all(any(self.topic_matches(a, t) for a in advertised) for t in topics)
        while time.monotonic() < end and not done():
            self._read_one(end - time.monotonic())
        return {c["topic"]: c for c in self.channels.values()}

    @staticmethod
    def topic_matches(topic: str, wanted: str) -> bool:
        """"/plc/lines" also matches "plc/lines" and a Git Bash path-converted
        "C:/Program Files/Git/plc/lines" (MSYS rewrites arguments starting
        with "/")."""
        wanted = wanted.replace("\\", "/").rstrip("/")
        return wanted.strip("/") == topic.strip("/") or wanted.endswith(topic)

    def subscribe(self, topics=None):
        """Subscribe to these topics (all advertised ones if None)."""
        subs = []
        for cid, ch in self.channels.items():
            if topics is None or any(self.topic_matches(ch["topic"], t) for t in topics):
                sid = len(self.subscriptions) + 1
                self.subscriptions[sid] = cid
                subs.append({"id": sid, "channelId": cid})
        if subs:
            self._send_frame(0x1, json.dumps({"op": "subscribe", "subscriptions": subs}).encode())
        return len(subs)

    def _read_one(self, timeout: float):
        self.sock.settimeout(max(0.01, timeout))
        try:
            opcode, payload = self._recv_frame()
        except socket.timeout:
            return None
        if opcode == 0x1:
            self._handle_text(payload.decode())
            return None
        if opcode == 0x2 and payload and payload[0] == OP_MESSAGE_DATA:
            sid, log_time = struct.unpack_from("<IQ", payload, 1)
            channel = self.channels.get(self.subscriptions.get(sid, -1), {})
            return channel.get("topic", f"?{sid}"), log_time, channel.get("encoding"), payload[13:]
        return None

    def messages(self, seconds: float):
        """Yield (topic, log_time_ns, decoded) for `seconds`; JSON decoded,
        other encodings as bytes."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            got = self._read_one(end - time.monotonic())
            if got is None:
                continue
            topic, log_time, encoding, data = got
            yield topic, log_time, (json.loads(data) if encoding == "json" else data)

    def close(self):
        try:
            self._send_frame(0x8, b"")
        except OSError:
            pass
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description="Print what a Foxglove WebSocket server publishes")
    parser.add_argument("url", nargs="?", default="ws://127.0.0.1:8765")
    parser.add_argument("--topic", action="append", help="Only this topic (repeatable)")
    parser.add_argument("--seconds", type=float, default=5.0, help="How long to listen (default 5)")
    parser.add_argument("--list", action="store_true", help="Only list the advertised channels")
    parser.add_argument("--width", type=int, default=300, help="Characters printed per message (0 = all)")
    args = parser.parse_args()

    probe = FoxgloveProbe(args.url)
    probe.wait_for_topics(args.topic or [], timeout=2.0)
    name = (probe.server_info or {}).get("name", "?")
    print(f"Connected to {args.url} ({name}); channels:")
    for ch in sorted(probe.channels.values(), key=lambda c: c["topic"]):
        print(f"  {ch['topic']:28s} {ch.get('encoding', ''):9s} {ch.get('schemaName', '')}")
    if args.list:
        probe.close()
        return
    probe.subscribe(args.topic)
    counts: Dict[str, int] = {}
    for topic, _t, msg in probe.messages(args.seconds):
        counts[topic] = counts.get(topic, 0) + 1
        text = json.dumps(msg, separators=(",", ":")) if not isinstance(msg, bytes) else f"<{len(msg)} bytes>"
        print(f"{topic}: {text[:args.width] if args.width else text}")
    probe.close()
    print("Received: " + (", ".join(f"{t} {n}" for t, n in sorted(counts.items())) or "nothing"))


if __name__ == "__main__":
    main()
