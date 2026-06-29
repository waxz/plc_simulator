import socket
import struct
import threading
import time
from collections import deque
from typing import Callable, Deque, Literal, Optional


FrameHandler = Callable[[bytes], None]
EndianName = Literal["LE", "BE"]


def _length_struct_prefix(endianness: str = "BE") -> str:
    return "<" if str(endianness).upper() == "LE" else ">"


class TCP_StreamParser:
    # VAR CONSTANT equivalents
    BUFFER_LENGTH = 4095
    PAYLOAD_LENGTH = 2048
    HEADER_SIZE = 8

    def __init__(self, length_endianness: EndianName = "BE"):
        self.length_endianness = length_endianness
        # VAR variables (Persistent internal stream buffer)
        # Using bytearray for fixed-size pre-allocation and indexing parity
        self.Buffer = bytearray(self.BUFFER_LENGTH + 1)
        self.BufferLength = 0

        # VAR PUBLIC variables (State and data interface)
        self.MagicFound = False
        self.MarkerIndex = 0
        self.PayloadLength = 0
        self.TotalPacketSize = 0
        
        self.PayloadReady = False
        self.PayloadReadyCount = 0
        self.ExtractedPayload = bytearray(self.PAYLOAD_LENGTH + 1)
        self.ExtractedLength = 0
        self.PendingPayloads: Deque[bytes] = deque()

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

            # 3. EXTRACT LENGTH (4-byte unsigned int)
            # Unpacking Buffer[4:8] directly handles the shift-and-OR logic
            self.PayloadLength = struct.unpack(
                _length_struct_prefix(self.length_endianness) + "I",
                self.Buffer[4:8],
            )[0]
            self.TotalPacketSize = self.HEADER_SIZE + self.PayloadLength

            # Safety verification against internal memory sizing bounds
            if self.TotalPacketSize > (self.BUFFER_LENGTH + 1) or self.PayloadLength > (self.PAYLOAD_LENGTH + 1):
                # Protocol violation or data corruption. Flush buffer.
                self.BufferLength = 0
                return

            # 4. CHECK COMPLETENESS: Ensure full payload has arrived
            if self.BufferLength < self.TotalPacketSize:
                return  # Stop parsing this cycle, await more TCP fragments

            # 5. EXTRACT PAYLOAD
            for i in range(self.PayloadLength):
                self.ExtractedPayload[i] = self.Buffer[self.HEADER_SIZE + i]
            
            self.ExtractedLength = self.PayloadLength
            self.PayloadReady = True
            self.PendingPayloads.append(bytes(self.ExtractedPayload[:self.ExtractedLength]))

            if self.PayloadReady:
                self.PayloadReadyCount += 1

            # FIX 4: Put application callback trigger HERE if handling multiple frames in 1 cycle

            # 6. SLIDE STREAM BUFFER AHEAD
            self.BufferLength -= self.TotalPacketSize
            if self.BufferLength > 0:
                for i in range(self.BufferLength):
                    self.Buffer[i] = self.Buffer[self.TotalPacketSize + i]

            # Prevent infinite loops if a corrupted packet sets PayloadLength to 0
            if self.TotalPacketSize <= self.HEADER_SIZE:
                self.BufferLength = 0
                return


class TcpFrameCodec:
    """Handles structured binary framing using a 4-Byte Magic Marker and a 4-Byte Length prefix."""

    # 4 bytes Magic Marker (0x12345678) + 4 bytes Length DINT = 8 bytes
    HEADER_SIZE = 8
    MAGIC_MARKER_INT = 0x12345678
    MAGIC_MARKER_BYTES = b"\x12\x34\x56\x78"
    MAX_PAYLOAD_SIZE = 16 * 1024 * 1024

    @classmethod
    def pack(cls, payload: bytes, length_endianness: EndianName = "BE") -> bytes:
        """Packs a raw payload with a fixed Magic Marker and configurable Length prefix."""
        return (
            cls.MAGIC_MARKER_BYTES
            + struct.pack(_length_struct_prefix(length_endianness) + "I", len(payload))
            + payload
        )

    @classmethod
    def read(cls, sock: socket.socket, parser: TCP_StreamParser) -> Optional[bytes]:
        """Reads from a socket, scans for the Magic Marker, and extracts the payload safely."""
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
    """Single-client TCP server used by the simulator exchange side."""

    def __init__(self, host: str, port: int, on_frame: FrameHandler, length_endianness: EndianName = "BE"):
        self.host = host
        self.port = port
        self.on_frame = on_frame
        self.length_endianness = length_endianness
        self.is_running = False
        self._server_socket: Optional[socket.socket] = None
        self._client_socket: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._send_lock = threading.Lock()
        

    def start(self) -> None:
        if self.is_running:
            return
        self.is_running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def send(self, payload: bytes) -> bool:
        with self._send_lock:
            if self._client_socket is None:
                return False
            try:
                framed_packet = TcpFrameCodec.pack(payload, self.length_endianness)
                self._client_socket.sendall(framed_packet)
                return True
            except OSError as exc:
                print(f"[TCP] Send failed: {exc}")
                self._close_client()
                return False

    def stop(self) -> None:
        self.is_running = False
        self._close_client()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3)

    @property
    def connected(self) -> bool:
        return self._client_socket is not None

    def _run(self) -> None:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self.host, self.port))
                server.listen(1)
                server.settimeout(0.5)
                self._server_socket = server
                print(f"[TCP] Listening on {self.host}:{self.port}")
                parser =TCP_StreamParser(self.length_endianness)

                while self.is_running:
                    try:
                        client, address = server.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    self._close_client()
                    self._client_socket = client
                    print(f"[TCP] Client connected: {address[0]}:{address[1]}")
                    self._recv_loop(client,parser)
        except OSError as exc:
            if self.is_running:
                print(f"[TCP] Server stopped after error: {exc}")
        finally:
            self._close_client()

    def _recv_loop(self, client: socket.socket, parser: TCP_StreamParser) -> None:
        try:
            while self.is_running and client is self._client_socket:
                payload = TcpFrameCodec.read(client, parser)
                if payload is None:
                    continue
                self.on_frame(payload)
        except (ConnectionError, OSError, ValueError) as exc:
            if self.is_running:
                print(f"[TCP] Receive failed: {exc}")
        finally:
            self._close_client()
            print("[TCP] Client disconnected.")

    def _close_client(self) -> None:
        client = self._client_socket
        self._client_socket = None
        if client is not None:
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

    def send(self, payload: bytes) -> bool:

        with self._send_lock:
            if self._socket is None:
                return False
            try:
                # FIX: Let TcpFrameCodec handle all header wrapping natively 
                # to avoid compounding length calculations.
                framed_packet = TcpFrameCodec.pack(payload, self.length_endianness)
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
                sock.settimeout(None)
                self._socket = sock
                print(f"[TCP] Connected to {self.host}:{self.port}")
                parser =TCP_StreamParser(self.length_endianness)

                while self.is_running and sock is self._socket:
                    payload = TcpFrameCodec.read(sock, parser)
                    if payload is None:
                        continue

                    self.on_frame(payload)
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

