import socket
import struct
import threading
import time
from collections import deque
from enum import IntEnum
from typing import Callable, Deque, Literal, NamedTuple, Optional


class FrameType(IntEnum):
    """What a frame's payload actually is, carried explicitly in the
    header instead of being guessed from payload size. 0 is deliberately
    unused (reserved/invalid), so an all-zero or otherwise-garbage header
    byte reads as "not a recognized type" rather than aliasing a real one.

    Different nodes can report different metric data - a node's frame_type
    selects which binary layout its payload uses, not just "this is a
    metric frame" (see METRICS_CHANNELS/METRICS_MAPPING_BY_FRAME_TYPE in
    mqtt_data_exchange.py, which maps each METRIC_LAYOUT_* value to its own
    MetricsMessage/mapping). A metric channel declared in a channel
    registry (channel_registry.py) names its frame_type as a plain string;
    run `python generate_frame_type.py --registry channels.json` to add a
    member here for any name that registry uses but this enum doesn't have
    yet, instead of hand-editing this class - it only ever appends (an
    existing member's value never changes, since that value is what's
    actually on the wire)."""
    IO_MAPPING = 1      # ComponentMessage-based structured I/O (input/output channels)
    METRIC_LAYOUT_1 = 2  # MetricsMessage-based metrics, layout 1 (see METRICS_MAPPING)
    METRIC_LAYOUT_2 = 3  # MetricsMessage-based metrics, layout 2 (see METRICS_MAPPING_2)
    metri_1 = 4  # added by generate_frame_type.py
    METRIC_LAYOUT_3 = 5  # added by generate_frame_type.py
    BELT_FEEDBACK = 6   # simulator -> PLC: per-belt actualSpeed (FLOAT) + encoderCount (UINT32), see drive.py
    LAYOUT_CHECK = 7    # PLC <-> simulator: I/O layout fingerprints + sizes, see io_layout.py
    INPUT_IMAGE = 8     # PLC -> monitor (plc_foxglove_bridge.py): the input image the PLC works on,
                        # same layout as the simulator's IO_MAPPING input frame


class ParsedFrame(NamedTuple):
    """One fully-decoded frame: which node sent it, what kind of payload
    it carries, and the payload itself. `on_frame` callbacks receive this
    instead of bare payload bytes, so routing (which channel/handler a
    frame belongs to) never has to guess from payload size."""
    node_id: int
    frame_type: int
    payload: bytes
    conn: Optional[int] = None      # TcpFramedServer: the client connection it came from (send_to)


FrameHandler = Callable[[ParsedFrame], None]
EndianName = Literal["LE", "BE"]

# Applied to every accepted/connected socket (both server and client sides).
# Bounds recv() so a background thread can notice shutdown and exit
# cleanly, and - since a socket's timeout applies to ALL blocking calls on
# it, not just the one that set it - also bounds sendall(), so a peer that
# stops reading causes a bounded stall + clean disconnect instead of
# blocking the caller (the main/GUI thread, for the simulator's TCP server)
# forever.
RECV_TIMEOUT_S = 2.0
RECV_CHUNK = 4096


def _tune(sock: socket.socket) -> None:
    """Small cyclic frames: no Nagle delay (with delayed ACKs it can hold a
    10 ms frame back for tens of ms); keep-alive probes to notice a peer
    that vanished without closing (a PLC powered off, a pulled cable)."""
    for level, option in ((socket.IPPROTO_TCP, socket.TCP_NODELAY), (socket.SOL_SOCKET, socket.SO_KEEPALIVE)):
        try:
            sock.setsockopt(level, option, 1)
        except OSError:
            pass


def _length_struct_prefix(endianness: str = "BE") -> str:
    return "<" if str(endianness).upper() == "LE" else ">"


class TCP_StreamParser:
    """Splits a byte stream into frames (see TcpFrameCodec): magic, node id,
    frame type, payload length, payload, trailing magic.

    Resynchronizes on anything that isn't a valid frame - garbage, a
    header-magic false positive (its trailer doesn't match), a length
    beyond PAYLOAD_LENGTH - by skipping ONE byte and searching for the next
    magic marker, so frames already received behind it are never lost.
    Zero-length payloads are valid frames.

    The field names mirror the PLC's TCP_StreamParser (ST) for comparison:
    PayloadReady / PayloadReadyCount / Extracted* describe the last frame of
    the latest Execute(); PendingPayloads (Python only) queues every frame."""

    BUFFER_LENGTH = 4095            # bytes kept between reads at most (beyond: resync)
    PAYLOAD_LENGTH = 2048           # the largest payload accepted
    HEADER_SIZE = 10                # magic (4) + node id (1) + frame type (1) + length (4)
    TRAILER_SIZE = 4                # the trailing copy of the magic
    MAGIC = bytes((0x12, 0x34, 0x56, 0x78))

    def __init__(self, length_endianness: EndianName = "BE"):
        self.length_endianness = length_endianness
        self._length = struct.Struct(_length_struct_prefix(length_endianness) + "I")
        self.Buffer = bytearray()
        self.PayloadReady = False
        self.PayloadReadyCount = 0
        self.ExtractedNodeId = 0
        self.ExtractedFrameType = 0
        self.ExtractedPayload = b""
        self.ExtractedLength = 0
        self.PendingPayloads: Deque[ParsedFrame] = deque()
        # Bytes skipped while resynchronizing (garbage, false headers, bad lengths).
        self.SkippedBytes = 0

    @property
    def BufferLength(self) -> int:
        return len(self.Buffer)

    def Execute(self, RxTemp: bytes | bytearray, RxCount: int):
        """Append RxTemp[:RxCount] and extract every complete frame."""
        self.PayloadReady = False
        if RxCount > 0:
            self.Buffer += RxTemp[:RxCount]
        buf = self.Buffer
        while True:
            start = buf.find(self.MAGIC)
            if start < 0:
                # No marker: keep only a possible marker prefix at the end.
                keep = min(len(buf), len(self.MAGIC) - 1)
                self.SkippedBytes += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if start:
                self.SkippedBytes += start
                del buf[:start]
            if len(buf) < self.HEADER_SIZE:
                break
            length = self._length.unpack_from(buf, 6)[0]
            if length > self.PAYLOAD_LENGTH:
                # Not a real header (or a frame we can never take): skip this marker.
                self.SkippedBytes += 1
                del buf[:1]
                continue
            total = self.HEADER_SIZE + length + self.TRAILER_SIZE
            if len(buf) < total:
                break                                   # wait for the rest
            if buf[total - self.TRAILER_SIZE:total] != self.MAGIC:
                self.SkippedBytes += 1                  # a false positive: resync from the next byte
                del buf[:1]
                continue
            frame = ParsedFrame(buf[4], buf[5], bytes(buf[self.HEADER_SIZE:self.HEADER_SIZE + length]))
            del buf[:total]
            self.ExtractedNodeId, self.ExtractedFrameType = frame.node_id, frame.frame_type
            self.ExtractedPayload, self.ExtractedLength = frame.payload, length
            self.PayloadReady = True
            self.PayloadReadyCount += 1
            self.PendingPayloads.append(frame)
        if len(buf) > self.BUFFER_LENGTH + 1:
            # More than any valid frame needs: drop the oldest, keep resyncing.
            self.SkippedBytes += len(buf) - (self.BUFFER_LENGTH + 1)
            del buf[:len(buf) - (self.BUFFER_LENGTH + 1)]


class TcpFrameCodec:
    """Handles structured binary framing using a 4-Byte Magic Marker,
    a 1-Byte Node ID, a 1-Byte Frame Type, a 4-Byte Length prefix, and a
    trailing 4-Byte copy of the Magic Marker."""

    # 4 bytes Magic Marker (0x12345678) + 1 byte NodeId + 1 byte FrameType
    # + 4 bytes Length DINT = 10 bytes.
    HEADER_SIZE = 10
    # Trailing copy of the Magic Marker - lets the receiver (TCP_StreamParser)
    # confirm the length field was read at a genuine frame boundary before
    # trusting the extracted payload, rather than accepting whatever bytes
    # happen to follow a header-magic false positive as if they were real.
    TRAILER_SIZE = 4
    MAGIC_MARKER_INT = 0x12345678
    MAGIC_MARKER_BYTES = b"\x12\x34\x56\x78"
    MAX_PAYLOAD_SIZE = 16 * 1024 * 1024
    DEFAULT_NODE_ID = 0

    @classmethod
    def pack(cls, payload: bytes, frame_type: int, node_id: int = DEFAULT_NODE_ID, length_endianness: EndianName = "BE") -> bytes:
        """Packs a raw payload with a fixed Magic Marker, a Node ID and
        Frame Type byte, a configurable Length prefix, and a trailing
        copy of the Magic Marker confirming the frame is complete.

        `frame_type` has no default - every frame is one specific kind of
        payload (see FrameType), and a caller that doesn't know which one
        it's sending is a bug, not something to paper over with a silent
        default."""
        if not (0 <= node_id <= 0xFF):
            raise ValueError(f"node_id must fit in one byte (0-255), got {node_id}")
        if not (0 <= frame_type <= 0xFF):
            raise ValueError(f"frame_type must fit in one byte (0-255), got {frame_type}")
        return (
            cls.MAGIC_MARKER_BYTES
            + bytes((node_id, frame_type))
            + struct.pack(_length_struct_prefix(length_endianness) + "I", len(payload))
            + payload
            + cls.MAGIC_MARKER_BYTES
        )

    @classmethod
    def read(cls, sock: socket.socket, parser: TCP_StreamParser) -> Optional[ParsedFrame]:
        """Reads from a socket, scans for the Magic Marker, and extracts a full frame safely."""
        if parser.PendingPayloads:
            return parser.PendingPayloads.popleft()

        chunk = sock.recv(RECV_CHUNK)
        if not chunk:
            raise ConnectionError("socket closed by peer")

        parser.Execute(chunk, len(chunk))
        if not parser.PendingPayloads:
            return None

        return parser.PendingPayloads.popleft()



class _ClientWriter:
    """One connected client's outbound side: a background thread that owns
    every sendall() on the socket, fed through a "latest frame wins" slot per
    (frame_type, node_id).

    Every frame this server sends is cyclic state (the current I/O image,
    the current belt feedback), so when a client falls behind, the right
    thing to drop is a stale frame of the same kind - not to block the
    caller, and not to queue an unbounded backlog the client then has to
    chew through before it sees current data. A frame overwritten before it
    was sent is counted in `dropped`."""

    def __init__(self, sock: socket.socket, on_error: Callable[[], None]):
        self._sock = sock
        self.address = None
        self.conn_id = 0
        self.last_rx = time.monotonic()
        self.last_node_id: Optional[int] = None
        self.received = 0
        self._on_error = on_error
        self._cond = threading.Condition()
        self._pending: "dict[tuple[int, int], bytes]" = {}
        self._stopped = False
        self.sent = 0
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def enqueue(self, key: "tuple[int, int]", packet: bytes) -> None:
        with self._cond:
            if self._stopped:
                return
            if key in self._pending:
                self.dropped += 1
                # Re-insert so the replacement keeps arrival order.
                del self._pending[key]
            self._pending[key] = packet
            self._cond.notify()

    def stop(self) -> None:
        with self._cond:
            self._stopped = True
            self._pending.clear()
            self._cond.notify()

    def _run(self) -> None:
        while True:
            with self._cond:
                while not self._pending and not self._stopped:
                    self._cond.wait()
                if self._stopped:
                    return
                batch = list(self._pending.values())
                self._pending.clear()
            try:
                for packet in batch:
                    self._sock.sendall(packet)
                    self.sent += 1
            except OSError as exc:
                with self._cond:
                    stopped = self._stopped
                if not stopped:
                    print(f"[TCP] Send failed: {exc}")
                    self._on_error()
                return


class TcpFramedServer:
    """Multi-client TCP server used by the simulator exchange side. Runs its
    accept loop on a background daemon thread (started by `start()`) and
    spawns one additional background daemon thread per connected client to
    receive its frames, so any number of PLC mocks/monitoring tools can
    stay connected at once. `send()` broadcasts a payload to every
    currently-connected client without blocking: each client has its own
    writer thread (_ClientWriter), so one stalled client can never delay
    the caller (the simulation loop) or any other client.

    Every connected client's recv thread - and the caller's own thread
    calling `send()` - eventually reach into the SAME shared
    `SimulatorConfig` object (via the `on_frame` callback and the payload
    passed to `send()`, respectively). This class itself only touches its
    own client-socket bookkeeping (guarded by `_clients_lock`); it's
    `MQTTSyncEngine`'s `_lock` (in mqtt_data_exchange.py, wrapping both
    `ComponentMessage.serialize()`/`deserialize()` calls into `self.config`)
    that actually serializes concurrent access to the config across
    however many client threads end up calling `on_frame` at once - that
    lock's correctness doesn't depend on how many clients there are.
    """

    def __init__(
        self,
        host: str,
        port: int,
        on_frame: FrameHandler,
        length_endianness: EndianName = "BE",
        max_clients: int = 8,
        rx_timeout: Optional[float] = None,
        max_payload: int = 0,
    ):
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.length_endianness = length_endianness
        self.max_clients = max_clients
        # Close a client that has sent nothing this long (None / 0: never) -
        # a silently dead peer otherwise stays "connected" forever.
        self.rx_timeout = rx_timeout or None
        # Frames with a longer payload are not sent (0: no limit) - e.g. the
        # PLC's receive buffer; counted in stats()["oversize"].
        self.max_payload = max_payload
        self.oversize = 0
        self._next_conn = 0
        self.is_running = False
        self._server_socket: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        # Guards the client set itself (add/remove/iterate) - separate from
        # MQTTSyncEngine's config lock, which this class knows nothing
        # about. Every socket operation below happens outside this lock
        # (only the bookkeeping dict is protected), so a slow/stalled
        # client's bounded (RECV_TIMEOUT_S) stall in sendall() never blocks
        # other clients from being registered/removed concurrently.
        self._clients_lock = threading.Lock()
        self._clients: dict[socket.socket, threading.Thread] = {}
        self._writers: dict[socket.socket, _ClientWriter] = {}
        # Totals over clients that have since disconnected (see stats()).
        self._retired_sent = 0
        self._retired_dropped = 0

    def start(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def _frame(self, payload: bytes, frame_type: int, node_id: int) -> Optional[bytes]:
        if self.max_payload and len(payload) > self.max_payload:
            self.oversize += 1
            if self.oversize == 1 or self.oversize % 1000 == 0:
                print(f"[TCP] Not sending a {len(payload)}-byte frame (type {frame_type}): over the "
                      f"{self.max_payload}-byte limit (tcpMaxPayload) - {self.oversize} so far")
            return None
        return TcpFrameCodec.pack(payload, frame_type=frame_type, node_id=node_id,
                                  length_endianness=self.length_endianness)

    def send(self, payload: bytes, frame_type: int, node_id: int = TcpFrameCodec.DEFAULT_NODE_ID) -> bool:
        """Queue a frame for every connected client and return immediately.
        Returns True if at least one client is connected to receive it.
        Delivery happens on each client's writer thread; a newer frame of
        the same (frame_type, node_id) replaces one still waiting."""
        framed_packet = self._frame(payload, frame_type, node_id)
        if framed_packet is None:
            return False
        with self._clients_lock:
            writers = list(self._writers.values())
        for writer in writers:
            writer.enqueue((frame_type, node_id), framed_packet)
        return bool(writers)

    def send_to(self, conn: Optional[int], payload: bytes, frame_type: int,
                node_id: int = TcpFrameCodec.DEFAULT_NODE_ID) -> bool:
        """Queue a frame for ONE client - the connection a request came from
        (ParsedFrame.conn): a reply, which every other client (a PLC that
        decodes whatever arrives as its input image) must not receive."""
        framed_packet = self._frame(payload, frame_type, node_id)
        if framed_packet is None:
            return False
        with self._clients_lock:
            writer = next((w for w in self._writers.values() if w.conn_id == conn), None)
        if writer is None:
            return False
        writer.enqueue((frame_type, node_id), framed_packet)
        return True

    def stats(self) -> "dict[str, int]":
        """Frames written to sockets and frames replaced before sending
        (client too slow), summed over all clients ever connected."""
        with self._clients_lock:
            writers = list(self._writers.values())
            sent = self._retired_sent
            dropped = self._retired_dropped
        return {
            "clients": len(writers),
            "sent": sent + sum(w.sent for w in writers),
            "dropped": dropped + sum(w.dropped for w in writers),
            "oversize": self.oversize,
        }

    def client_info(self) -> "list[dict]":
        """Per connected client: connection id, address, the node id it
        sends as, seconds since it last sent, frames received / sent /
        dropped."""
        now = time.monotonic()
        with self._clients_lock:
            writers = list(self._writers.values())
        return [{"conn": w.conn_id, "address": f"{w.address[0]}:{w.address[1]}" if w.address else "",
                 "node_id": w.last_node_id, "rx_age_s": round(now - w.last_rx, 3), "received": w.received,
                 "sent": w.sent, "dropped": w.dropped} for w in writers]

    def stop(self) -> None:
        self.is_running = False
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        with self._clients_lock:
            clients = list(self._clients.keys())
        for client in clients:
            self._remove_client(client)
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=3)

    @property
    def connected(self) -> bool:
        return self.client_count > 0

    @property
    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def _accept_loop(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(self.max_clients)
                server.settimeout(0.5)
                self._server_socket = server
                print(f"[TCP] Listening on {self.host}:{self.port}")

                while self.is_running:
                    try:
                        client, address = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    if self.client_count >= self.max_clients:
                        print(f"[TCP] Rejecting {address[0]}:{address[1]} - max_clients ({self.max_clients}) reached")
                        try:
                            client.close()
                        except OSError:
                            pass
                        continue

                    # Accepted sockets start in blocking mode with NO
                    # timeout of their own - the listening socket's
                    # settimeout() above only ever applied to accept()
                    # itself. Without this, send() (sendall(), called every
                    # tick from the main/GUI thread via
                    # MQTTSyncEngine.sync()) could block forever the moment
                    # this peer stops reading - stalls, crashes without
                    # closing the socket, a network hiccup - freezing the
                    # whole app, not just this transport. recv() in
                    # _recv_loop needs the same bound so a permanently
                    # silent (but not disconnected) peer doesn't leave that
                    # thread stuck forever either, blocking a clean stop().
                    client.settimeout(RECV_TIMEOUT_S)
                    _tune(client)
                    print(f"[TCP] Client connected: {address[0]}:{address[1]} "
                          f"({self.client_count + 1}/{self.max_clients})")
                    thread = threading.Thread(
                        target=self._recv_loop, args=(client, address), daemon=True
                    )
                    writer = _ClientWriter(client, on_error=lambda c=client: self._remove_client(c))
                    self._next_conn += 1
                    writer.conn_id, writer.address = self._next_conn, address
                    with self._clients_lock:
                        self._clients[client] = thread
                        self._writers[client] = writer
                    thread.start()
        except OSError as exc:
            if self.is_running:
                print(f"[TCP] Server stopped after error: {exc}")

    def _recv_loop(self, client: socket.socket, address) -> None:
        parser = TCP_StreamParser(self.length_endianness)
        with self._clients_lock:
            writer = self._writers.get(client)
        conn = writer.conn_id if writer is not None else None
        try:
            while self.is_running:
                try:
                    frame = TcpFrameCodec.read(client, parser)
                except socket.timeout:
                    # No data within RECV_TIMEOUT_S - not an error, just
                    # gives this loop a chance to notice is_running went
                    # False (or the connection is otherwise still healthy
                    # with a legitimately idle gap between sends) - unless
                    # the watchdog (rx_timeout) says the peer is gone.
                    if (self.rx_timeout and writer is not None
                            and time.monotonic() - writer.last_rx > self.rx_timeout):
                        print(f"[TCP] {address[0]}:{address[1]} silent for over {self.rx_timeout:g} s - closing")
                        break
                    continue
                if writer is not None:
                    writer.last_rx = time.monotonic()
                if frame is None:
                    continue
                if writer is not None:
                    writer.received += 1
                    writer.last_node_id = frame.node_id
                self.on_frame(frame._replace(conn=conn))
        except (ConnectionError, OSError, ValueError) as exc:
            if self.is_running:
                print(f"[TCP] Receive failed for {address[0]}:{address[1]}: {exc}")
        finally:
            self._remove_client(client)
            print(f"[TCP] Client disconnected: {address[0]}:{address[1]} ({self.client_count}/{self.max_clients})")

    def _remove_client(self, client: socket.socket) -> None:
        with self._clients_lock:
            self._clients.pop(client, None)
            writer = self._writers.pop(client, None)
            if writer is not None:
                self._retired_sent += writer.sent
                self._retired_dropped += writer.dropped
        if writer is not None:
            writer.stop()
        try:
            client.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            client.close()
        except OSError:
            pass


class TcpFramedClient:
    """TCP client used by plc_mock and PLC-side integrations."""

    def __init__(
        self,
        host: str,
        port: int,
        on_frame: FrameHandler,
        reconnect_delay: float = 1.0,
        length_endianness: EndianName = "BE",
    ):
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.reconnect_delay = reconnect_delay
        self.length_endianness = length_endianness
        self.is_running = False
        self._socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._send_lock = threading.Lock()


    def start(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, payload: bytes, frame_type: int, node_id: int = TcpFrameCodec.DEFAULT_NODE_ID) -> bool:

        with self._send_lock:
            if self._socket is None:
                return False
            try:
                # FIX: Let TcpFrameCodec handle all header wrapping natively
                # to avoid compounding length calculations.
                framed_packet = TcpFrameCodec.pack(payload, frame_type=frame_type, node_id=node_id, length_endianness=self.length_endianness)
                self._socket.sendall(framed_packet)
                return True
            except OSError as exc:
                print(f"[TCP] Send failed: {exc}")
                self._close_socket()
                return False

    def stop(self) -> None:
        self.is_running = False
        self._close_socket()
        if self._thread is not None:
            self._thread.join(timeout=3)

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def _run(self) -> None:
        while self.is_running:
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
                # RECV_TIMEOUT_S, not None: this used to drop the timeout
                # entirely once connected, so both recv() (below) and
                # send() (above, sharing this same socket) could block
                # forever - recv() if the simulator ever stops sending,
                # send() if it stops reading. A bounded timeout turns
                # either into a clean reconnect instead of a permanent hang.
                sock.settimeout(RECV_TIMEOUT_S)
                _tune(sock)
                self._socket = sock
                print(f"[TCP] Connected to {self.host}:{self.port}")
                parser =TCP_StreamParser(self.length_endianness)

                while self.is_running and sock is self._socket:
                    try:
                        frame = TcpFrameCodec.read(sock, parser)
                    except socket.timeout:
                        continue
                    if frame is None:
                        continue

                    self.on_frame(frame)
            except OSError as exc:
                if self.is_running:
                    print(f"[TCP] Connection failed: {exc}")
            except (ConnectionError, ValueError) as exc:
                if self.is_running:
                    print(f"[TCP] Receive failed: {exc}")
            finally:
                self._close_socket()

            if self.is_running:
                time.sleep(self.reconnect_delay)

    def _close_socket(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
