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


class ParsedFrame(NamedTuple):
    """One fully-decoded frame: which node sent it, what kind of payload
    it carries, and the payload itself. `on_frame` callbacks receive this
    instead of bare payload bytes, so routing (which channel/handler a
    frame belongs to) never has to guess from payload size."""
    node_id: int
    frame_type: int
    payload: bytes


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


def _length_struct_prefix(endianness: str = "BE") -> str:
    return "<" if str(endianness).upper() == "LE" else ">"


class TCP_StreamParser:
    # VAR CONSTANT equivalents
    BUFFER_LENGTH = 4095
    PAYLOAD_LENGTH = 2048
    # 4 bytes Magic Marker + 1 byte NodeId + 1 byte FrameType + 4 bytes
    # Length DINT = 10 bytes. NodeId/FrameType sit between the magic
    # marker and the length field, matching the wire order TcpFrameCodec
    # packs them in below.
    HEADER_SIZE = 10
    TRAILER_SIZE = 4

    def __init__(self, length_endianness: EndianName = "BE"):
        self.length_endianness = length_endianness
        # VAR variables (Persistent internal stream buffer)
        # Using bytearray for fixed-size pre-allocation and indexing parity
        self.Buffer = bytearray(self.BUFFER_LENGTH + 1)
        self.BufferLength = 0

        # VAR PUBLIC variables (State and data interface)
        self.MagicFound = False
        self.MarkerIndex = 0
        self.NodeId = 0
        self.FrameType = 0
        self.PayloadLength = 0
        self.TotalPacketSize = 0

        self.PayloadReady = False
        self.PayloadReadyCount = 0
        self.ExtractedNodeId = 0
        self.ExtractedFrameType = 0
        self.ExtractedPayload = bytearray(self.PAYLOAD_LENGTH + 1)
        self.ExtractedLength = 0
        self.PendingPayloads: Deque[ParsedFrame] = deque()

    def Execute(self, RxTemp: bytes | bytearray, RxCount: int):
        """
        Executes the cyclic parsing loop.
        RxTemp can be a slice of any length; its elements are read from index 0.
        """
        self.PayloadReady = False  # Reset flag every cycle

        # 1. APPEND NEW NETWORK DATA TO STREAM BUFFER
        if RxCount > 0:
            # Match against the upper bound of the constant size
            if (self.BufferLength + RxCount) <= (self.BUFFER_LENGTH + 1):
                # Copy temporary network chunk to the end of our stream buffer
                for i in range(RxCount):
                    # Emulating standard index 0 start for the incoming byte stream
                    self.Buffer[self.BufferLength + i] = RxTemp[i]
                self.BufferLength += RxCount
            else:
                # Buffer Overflow Safety: Wipe pointer
                self.BufferLength = 0

        # 2. PARSING LOOP: Process internal stream buffer
        while self.BufferLength >= self.HEADER_SIZE:
            self.MagicFound = False

            # Look for the 4-byte Magic Marker: 0x12345678
            for i in range(self.BufferLength - 3):
                if (self.Buffer[i] == 0x12 and
                    self.Buffer[i+1] == 0x34 and
                    self.Buffer[i+2] == 0x56 and
                    self.Buffer[i+3] == 0x78):
                    self.MagicFound = True
                    self.MarkerIndex = i
                    break  # Found earliest marker, break FOR loop

            # Handle garbage data or unmatched fragments safely
            if not self.MagicFound:
                if self.BufferLength > 3:
                    for i in range(3):
                        self.Buffer[i] = self.Buffer[self.BufferLength - 3 + i]
                    self.BufferLength = 3
                return  # Force break of loop/method to wait for more data

            # If the magic marker was found deeper in the stream, slide buffer to align
            if self.MarkerIndex > 0:
                for i in range(self.BufferLength - self.MarkerIndex):
                    self.Buffer[i] = self.Buffer[self.MarkerIndex + i]
                self.BufferLength -= self.MarkerIndex

            # Re-verify we still have a header window left after shifting
            if self.BufferLength < self.HEADER_SIZE:
                return

            # 3. EXTRACT NODE ID, FRAME TYPE, AND LENGTH
            self.NodeId = self.Buffer[4]
            self.FrameType = self.Buffer[5]
            self.PayloadLength = struct.unpack(
                _length_struct_prefix(self.length_endianness) + "I",
                self.Buffer[6:10],
            )[0]
            self.TotalPacketSize = self.HEADER_SIZE + self.PayloadLength + self.TRAILER_SIZE

            # Safety verification against internal memory sizing bounds
            if self.TotalPacketSize > (self.BUFFER_LENGTH + 1) or self.PayloadLength > (self.PAYLOAD_LENGTH + 1):
                # Protocol violation or data corruption. Flush buffer.
                self.BufferLength = 0
                return

            # 4. CHECK COMPLETENESS: Ensure full payload + trailing marker have arrived
            if self.BufferLength < self.TotalPacketSize:
                return  # Stop parsing this cycle, await more TCP fragments

            # 4b. VERIFY TRAILING MAGIC MARKER: confirms the length field was
            # read at a genuine frame boundary, not a false-positive match of
            # the header magic bytes occurring inside unrelated payload data
            # (with a garbage length happening to follow it). A mismatch
            # means MarkerIndex was NOT a real frame start - drop just that
            # one leading byte and let the loop's own header search (top of
            # `while`) find the next candidate marker, instead of extracting
            # corrupted/misaligned data as if it were a real payload.
            trailer_start = self.HEADER_SIZE + self.PayloadLength
            if not (
                self.Buffer[trailer_start] == 0x12
                and self.Buffer[trailer_start + 1] == 0x34
                and self.Buffer[trailer_start + 2] == 0x56
                and self.Buffer[trailer_start + 3] == 0x78
            ):
                for i in range(self.BufferLength - 1):
                    self.Buffer[i] = self.Buffer[i + 1]
                self.BufferLength -= 1
                continue  # re-scan for the magic marker from the shifted buffer

            # 5. EXTRACT PAYLOAD
            for i in range(self.PayloadLength):
                self.ExtractedPayload[i] = self.Buffer[self.HEADER_SIZE + i]

            self.ExtractedNodeId = self.NodeId
            self.ExtractedFrameType = self.FrameType
            self.ExtractedLength = self.PayloadLength
            self.PayloadReady = True
            self.PendingPayloads.append(
                ParsedFrame(self.NodeId, self.FrameType, bytes(self.ExtractedPayload[:self.ExtractedLength]))
            )

            if self.PayloadReady:
                self.PayloadReadyCount += 1

            # FIX 4: Put application callback trigger HERE if handling multiple frames in 1 cycle

            # 6. SLIDE STREAM BUFFER AHEAD
            self.BufferLength -= self.TotalPacketSize
            if self.BufferLength > 0:
                for i in range(self.BufferLength):
                    self.Buffer[i] = self.Buffer[self.TotalPacketSize + i]

            # Prevent infinite loops if a corrupted packet sets PayloadLength to 0
            if self.TotalPacketSize <= self.HEADER_SIZE + self.TRAILER_SIZE:
                self.BufferLength = 0
                return


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

        chunk = sock.recv(256)
        if not chunk:
            raise ConnectionError("socket closed by peer")

        parser.Execute(chunk, len(chunk))
        if not parser.PendingPayloads:
            return None

        return parser.PendingPayloads.popleft()



class TcpFramedServer:
    """Multi-client TCP server used by the simulator exchange side. Runs its
    accept loop on a background daemon thread (started by `start()`) and
    spawns one additional background daemon thread per connected client to
    receive its frames, so any number of PLC mocks/monitoring tools can
    stay connected at once. `send()` broadcasts a payload to every
    currently-connected client.

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
    ):
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.length_endianness = length_endianness
        self.max_clients = max_clients
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

    def start(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def send(self, payload: bytes, frame_type: int, node_id: int = TcpFrameCodec.DEFAULT_NODE_ID) -> bool:
        """Broadcast to every connected client. Returns True if at least
        one client received it (False if there were none, or all failed)."""
        framed_packet = TcpFrameCodec.pack(payload, frame_type=frame_type, node_id=node_id, length_endianness=self.length_endianness)
        with self._clients_lock:
            clients = list(self._clients.keys())
        sent_to_any = False
        for client in clients:
            try:
                client.sendall(framed_packet)
                sent_to_any = True
            except OSError as exc:
                print(f"[TCP] Send failed: {exc}")
                self._remove_client(client)
        return sent_to_any

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
                    print(f"[TCP] Client connected: {address[0]}:{address[1]} "
                          f"({self.client_count + 1}/{self.max_clients})")
                    thread = threading.Thread(
                        target=self._recv_loop, args=(client, address), daemon=True
                    )
                    with self._clients_lock:
                        self._clients[client] = thread
                    thread.start()
        except OSError as exc:
            if self.is_running:
                print(f"[TCP] Server stopped after error: {exc}")

    def _recv_loop(self, client: socket.socket, address) -> None:
        parser = TCP_StreamParser(self.length_endianness)
        try:
            while self.is_running:
                try:
                    frame = TcpFrameCodec.read(client, parser)
                except socket.timeout:
                    # No data within RECV_TIMEOUT_S - not an error, just
                    # gives this loop a chance to notice is_running went
                    # False (or the connection is otherwise still healthy
                    # with a legitimately idle gap between sends).
                    continue
                if frame is None:
                    continue
                self.on_frame(frame)
        except (ConnectionError, OSError, ValueError) as exc:
            if self.is_running:
                print(f"[TCP] Receive failed for {address[0]}:{address[1]}: {exc}")
        finally:
            self._remove_client(client)
            print(f"[TCP] Client disconnected: {address[0]}:{address[1]} ({self.client_count}/{self.max_clients})")

    def _remove_client(self, client: socket.socket) -> None:
        with self._clients_lock:
            self._clients.pop(client, None)
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
