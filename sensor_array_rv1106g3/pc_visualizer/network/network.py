import socket
import threading
import time

from PyQt5.QtCore import QObject, pyqtSignal

from network.protocol import (
    SlipDecoder, 
    parse_message, parse_frame, 
    MSG_FRAME, MSG_RESP, MSG_EVENT)

from config.defaults import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_PROTO,
    DEBUG_NET,
)


# ═══════════════════════════════════════════════════════════════════════════════
#  Network Worker


class NetworkWorker(QObject):
    frame_received  = pyqtSignal(object)   # SyncedFrame
    text_received   = pyqtSignal(str)
    connection_lost = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._sock    = None
        self._running = False
        self._lock    = threading.Lock()
        self._slip    = SlipDecoder(self._on_packet)
        self._proto   = DEFAULT_PROTO
        self._host    = DEFAULT_HOST
        self._port    = DEFAULT_PORT

    # ── public API ─────────────────────────────────────────────────────────────

    def connect_to(self, host: str, port: int, proto: str = 'TCP') -> bool:
        self._host, self._port, self._proto = host, port, proto.upper()
        try:
            if self._proto == 'TCP':
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # Disable Nagle — we want every SLIP frame sent immediately
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.setsockopt(socket.SOL_SOCKET,  socket.SO_KEEPALIVE, 1)
                s.settimeout(3.0)
                s.connect((host, port))
                s.settimeout(0.05)   # non-blocking read loop
            else:                    # UDP
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.settimeout(0.05)
                # Send an empty SLIP packet so the device learns our address
                s.sendto(b'\xc0\xc0', (host, port))
            self._sock    = s
            self._running = True
            self._slip.reset()
            return True
        except Exception as e:
            self.text_received.emit(f'[ERR] {host}:{port}/{proto} — {e}')
            return False

    def disconnect(self):
        self._running = False
        with self._lock:
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
        self._slip.reset()

    def send(self, data: bytes):
        with self._lock:
            if not self._sock:
                return
            try:
                if self._proto == 'TCP':
                    self._sock.sendall(data)
                else:
                    self._sock.sendto(data, (self._host, self._port))
            except Exception as e:
                self.text_received.emit(f'[ERR] send: {e}')

    def run(self):
        """Runs in a dedicated QThread — reads data and feeds the SLIP decoder."""
        total_bytes = 0
        while self._running:
            try:
                with self._lock:
                    sock = self._sock
                if sock is None:
                    time.sleep(0.05)
                    continue

                if self._proto == 'TCP':
                    try:
                        chunk = sock.recv(65536)
                        if not chunk:
                            # Graceful close from server
                            self.connection_lost.emit('Server closed the connection')
                            self._running = False
                            break
                        total_bytes += len(chunk)
                        if DEBUG_NET and (total_bytes % 100000 < len(chunk)):
                            print(f'[NET] Received {len(chunk)} bytes (total: {total_bytes})')
                        self._slip.feed(chunk)
                    except socket.timeout:
                        pass          # normal — keep polling
                else:
                    # UDP: each datagram is already one complete protocol message
                    try:
                        data, _ = sock.recvfrom(65536)
                        if data:
                            self._on_packet(data)
                    except socket.timeout:
                        pass

            except OSError as e:
                if self._running:
                    self.connection_lost.emit(str(e))
                self._running = False

    def _on_packet(self, raw: bytes):
        if DEBUG_NET:
            print(f'[SLIP] Decoded packet: {len(raw)} bytes')
        result = parse_message(raw)
        if not result:
            if DEBUG_NET:
                print(f'[PARSE] parse_message returned None')
            return
        mtype, seq, ts_us, payload = result
        if DEBUG_NET:
            print(f'[MSG] type={mtype} seq={seq} payload_len={len(payload)}')
        if mtype == MSG_FRAME:
            sf = parse_frame(payload)
            if sf:
                if DEBUG_NET:
                    print(f'[FRAME] Parsed frame seq={sf.seq}')
                self.frame_received.emit(sf)
            else:
                if DEBUG_NET:
                    print(f'[FRAME] parse_frame returned None')
        elif mtype in (MSG_RESP, MSG_EVENT):
            self.text_received.emit(
                f'[{seq}] {payload.decode("utf-8", errors="replace")}')
