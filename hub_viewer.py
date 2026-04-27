#!/usr/bin/env python3
"""hub_viewer.py — ESP32-S3 sensor hub viewer + command console

- Decodes SLIP-framed messages with CRC32
- Renders stacked OpenCV visualization: Camera (MJPEG) + ToF + MLX90640
- Sends commands interactively while streaming

Requirements:
  pip install pyserial numpy opencv-contrib-python

Usage:
  python3 hub_viewer.py                   # auto-detect port
  python3 hub_viewer.py /dev/ttyUSB0      # explicit port

Controls:
  q / ESC  quit
  1        STREAM mode=all
  2        STREAM mode=tof
  3        STREAM mode=mlx
  4        STREAM mode=cam
  0        STREAM mode=none
  space    STREAM enable=0/1 toggle

Interactive console:
  Type commands into the terminal (stdin), e.g.:
    GET INFO
    SET TOF side=8 hz=15 it_ms=50 continuous=1
    SET MLX mode=chess res=18 refresh=16
    SET I2C clock_hz=1000000
"""

from __future__ import annotations

import sys
import time
import threading
import queue
import struct
import binascii

import numpy as np
import cv2
import serial
import serial.tools.list_ports

# ------------------- Protocol constants -------------------
END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD

MAGIC = 0x53454E53  # 'SENS'
VERSION = 1

TYPE_FRAME = 1
TYPE_CMD = 2
TYPE_RESP = 3
TYPE_EVENT = 4

HEADER_FMT = "<IHHIQI"  # magic, version, type, seq, ts_us, payload_len
HEADER_SIZE = struct.calcsize(HEADER_FMT)
CRC_SIZE = 4

# FrameFixedV1 layout sizes (must match firmware)
FRAME_FIXED_SIZE = 2984

# Flags
FLAG_TOF = 1 << 0
FLAG_MLX = 1 << 1
FLAG_CAM = 1 << 2

FOURCC_MJPG = 0x47504A4D


def _iter_ports():
    return list(serial.tools.list_ports.comports())


def _is_usb_serial_device(dev: str) -> bool:
    return dev.startswith("/dev/ttyUSB") or dev.startswith("/dev/ttyACM")


def _score_port(p):
    score = 0
    dev = p.device or ""
    label = ((p.description or "") + " " + (p.manufacturer or "")).lower()
    if "ch343" in label:
        score += 200
    if "wch" in label:
        score += 40
    if "usb serial" in label:
        score += 30
    if _is_usb_serial_device(dev):
        score += 100
    # Heavily de-prioritize platform UARTs (often unusable for this app).
    if dev.startswith("/dev/ttyS"):
        score -= 500
    return score


def find_port() -> str | None:
    ports = _iter_ports()
    if not ports:
        return None

    usb_ports = [p for p in ports if _is_usb_serial_device(p.device or "")]
    cand = usb_ports if usb_ports else ports
    best = max(cand, key=_score_port)
    return best.device


def _port_list_text() -> str:
    ports = _iter_ports()
    if not ports:
        return "(none)"
    rows = []
    for p in ports:
        rows.append(f"- {p.device}: {p.description or 'unknown'}")
    return "\n".join(rows)


class SlipDecoder:
    def __init__(self):
        self._buf = bytearray()
        self._esc = False

    def feed(self, data: bytes):
        frames = []
        for b in data:
            if b == END:
                if self._buf:
                    frames.append(bytes(self._buf))
                self._buf.clear()
                self._esc = False
                continue

            if self._esc:
                self._esc = False
                if b == ESC_END:
                    self._buf.append(END)
                elif b == ESC_ESC:
                    self._buf.append(ESC)
                else:
                    # Invalid escape -> drop current frame
                    self._buf.clear()
                continue

            if b == ESC:
                self._esc = True
                continue

            self._buf.append(b)
        return frames


def slip_encode(payload: bytes) -> bytes:
    out = bytearray([END])
    for b in payload:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)


def build_cmd(seq: int, text: str) -> bytes:
    payload = text.encode("utf-8")
    hdr = struct.pack(HEADER_FMT, MAGIC, VERSION, TYPE_CMD, seq, 0, len(payload))
    crc = binascii.crc32(hdr + payload) & 0xFFFFFFFF
    return slip_encode(hdr + payload + struct.pack("<I", crc))


def parse_msg(frame: bytes):
    if len(frame) < HEADER_SIZE + CRC_SIZE:
        return None
    (magic, ver, typ, seq, ts_us, payload_len) = struct.unpack_from(HEADER_FMT, frame, 0)
    if magic != MAGIC or ver != VERSION:
        return None
    need = HEADER_SIZE + payload_len + CRC_SIZE
    if need != len(frame):
        return None
    payload = frame[HEADER_SIZE:HEADER_SIZE + payload_len]
    crc_rx = struct.unpack_from("<I", frame, HEADER_SIZE + payload_len)[0]
    crc = binascii.crc32(frame[:HEADER_SIZE + payload_len]) & 0xFFFFFFFF
    if crc != crc_rx:
        return None
    return typ, seq, ts_us, payload


class LatestState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_frame_t = 0.0
        self.fps = 0.0
        self._fps_count = 0
        self._fps_t0 = time.monotonic()

        self.cam = None  # np.uint8 BGR
        self.tof = None  # np.float32 mm, shape (8,8)
        self.mlx = None  # np.float32 C, shape (24,32)

        self.info_text = ""

    def tick_fps(self):
        self._fps_count += 1
        now = time.monotonic()
        if (now - self._fps_t0) >= 0.5:
            self.fps = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now


def decode_frame(payload: bytes):
    if len(payload) < FRAME_FIXED_SIZE:
        return None
    fixed = payload[:FRAME_FIXED_SIZE]

    # Header of FrameFixedV1
    frame_seq = struct.unpack_from("<I", fixed, 0)[0]
    hub_ts_us = struct.unpack_from("<Q", fixed, 4)[0]
    flags = struct.unpack_from("<I", fixed, 12)[0]

    # Offsets (must match hub_frame.h)
    off = 20

    # --- ToF ---
    tof = None
    if flags & FLAG_TOF:
        tof_ts_us = struct.unpack_from("<Q", fixed, off)[0]
        side = fixed[off + 8]
        targets_per_zone = fixed[off + 9]
        # Skip cfg and nb_targets etc. We read first-target distances only for visualization.
        # Layout within TofDataV1:
        # ts(8) + cfg(8) + nb_targets(64) + distance(64*4*2) + sigma(64*4*2) + status(64*4)
        tof_base = off
        nb_off = tof_base + 16
        dist_off = nb_off + 64

        # distances array is zone-major, target-minor, but stored in output buffer as [zone*4 + t]
        dist_raw = np.frombuffer(fixed, dtype=np.uint16, count=64 * 4, offset=dist_off)
        dist0 = dist_raw[0::4].astype(np.float32)
        dist0[dist0 == 0xFFFF] = np.nan
        if side in (4, 8):
            tof = dist0.reshape((8, 8))
            if side == 4:
                tof = cv2.resize(tof, (8, 8), interpolation=cv2.INTER_NEAREST)

    off += 1360

    # --- MLX ---
    mlx = None
    if flags & FLAG_MLX:
        mlx_ts_us = struct.unpack_from("<Q", fixed, off)[0]
        w, h = struct.unpack_from("<HH", fixed, off + 8)
        ta_cC = struct.unpack_from("<h", fixed, off + 16)[0]
        frame_off = off + 20
        vals = np.frombuffer(fixed, dtype=np.int16, count=768, offset=frame_off).astype(np.float32) / 100.0
        if (w, h) == (32, 24):
            mlx = vals.reshape((24, 32))

    off += 1556

    # --- CamSync (ignored) ---
    off += 24

    # --- Camera ---
    cam = None
    cam_ts_us = struct.unpack_from("<Q", fixed, off)[0]
    cam_w, cam_h, fourcc = struct.unpack_from("<III", fixed, off + 8)
    cam_len = struct.unpack_from("<I", fixed, off + 20)[0]

    cam_bytes = payload[FRAME_FIXED_SIZE:FRAME_FIXED_SIZE + cam_len]
    if (flags & FLAG_CAM) and fourcc == FOURCC_MJPG and cam_len > 0:
        arr = np.frombuffer(cam_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        cam = img

    return frame_seq, hub_ts_us, flags, cam, tof, mlx


def render_stack(state: LatestState):
    with state.lock:
        cam = state.cam
        tof = state.tof
        mlx = state.mlx
        fps = state.fps
        info = state.info_text

    panels = []

    if cam is None:
        cam_panel = np.zeros((240, 320, 3), np.uint8)
        cv2.putText(cam_panel, "CAM: (none)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    else:
        cam_panel = cam

    cam_panel = cv2.resize(cam_panel, (640, 360), interpolation=cv2.INTER_AREA)
    cv2.putText(cam_panel, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    if info:
        for i, line in enumerate(info.splitlines()[:6]):
            cv2.putText(cam_panel, line, (10, 60 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    panels.append(cam_panel)

    # ToF heatmap
    if tof is None:
        tof_panel = np.zeros((240, 640, 3), np.uint8)
        cv2.putText(tof_panel, "TOF: (none)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    else:
        z = tof.copy()
        # Normalize 0.2m..4m
        z_mm = z
        z_mm = np.nan_to_num(z_mm, nan=0.0)
        norm = np.clip((z_mm - 200.0) / (4000.0 - 200.0), 0.0, 1.0)
        img8 = (norm * 255.0).astype(np.uint8)
        hm = cv2.applyColorMap(img8, cv2.COLORMAP_INFERNO)
        hm = cv2.resize(hm, (640, 240), interpolation=cv2.INTER_NEAREST)
        tof_panel = hm
    panels.append(tof_panel)

    # MLX heatmap
    if mlx is None:
        mlx_panel = np.zeros((240, 640, 3), np.uint8)
        cv2.putText(mlx_panel, "MLX: (none)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    else:
        t = mlx
        t0, t1 = np.nanpercentile(t, [5, 95])
        if not np.isfinite(t0) or not np.isfinite(t1) or t1 <= t0:
            t0, t1 = 20.0, 40.0
        norm = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
        img8 = (norm * 255.0).astype(np.uint8)
        hm = cv2.applyColorMap(img8, cv2.COLORMAP_TURBO)
        hm = cv2.resize(hm, (640, 240), interpolation=cv2.INTER_NEAREST)
        mlx_panel = hm
    panels.append(mlx_panel)

    return np.vstack(panels)


def serial_reader(ser: serial.Serial, out_q: queue.Queue, stop: threading.Event):
    dec = SlipDecoder()
    buf = bytearray(16384)

    while not stop.is_set():
        n = ser.readinto(buf)
        if not n:
            continue
        for fr in dec.feed(buf[:n]):
            msg = parse_msg(fr)
            if msg:
                out_q.put(msg)


def command_sender(ser: serial.Serial, write_lock: threading.Lock, cmd_q: queue.Queue, stop: threading.Event):
    seq = 1
    while not stop.is_set():
        try:
            cmd = cmd_q.get(timeout=0.1)
        except queue.Empty:
            continue
        if cmd is None:
            continue
        pkt = build_cmd(seq, cmd)
        seq += 1
        with write_lock:
            ser.write(pkt)
            ser.flush()


def stdin_thread(cmd_q: queue.Queue, stop: threading.Event):
    while not stop.is_set():
        line = sys.stdin.readline()
        if not line:
            time.sleep(0.05)
            continue
        line = line.strip()
        if not line:
            continue
        cmd_q.put(line)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("No serial port found")
        return 2

    baud = 2_000_000
    print(f"[hub_viewer] Using {port} @ {baud}")

    msg_q: queue.Queue = queue.Queue(maxsize=200)
    cmd_q: queue.Queue = queue.Queue(maxsize=50)
    stop = threading.Event()

    state = LatestState()

    try:
        ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.2)
    except serial.SerialException as e:
        print(f"[hub_viewer] Failed to open {port}: {e}")
        print("[hub_viewer] Available ports:")
        print(_port_list_text())
        print("[hub_viewer] Hint: run with explicit USB serial port, e.g. python3 hub_viewer.py /dev/ttyUSB0")
        return 2
    try:
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass

        write_lock = threading.Lock()

        t_r = threading.Thread(target=serial_reader, args=(ser, msg_q, stop), daemon=True)
        t_c = threading.Thread(target=command_sender, args=(ser, write_lock, cmd_q, stop), daemon=True)
        t_i = threading.Thread(target=stdin_thread, args=(cmd_q, stop), daemon=True)
        t_r.start(); t_c.start(); t_i.start()

        # Ensure stream is on by default
        cmd_q.put("STREAM enable=1 mode=all")

        win = "ESP32-S3 Hub"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        stream_enabled = True

        try:
            while True:
                # Drain messages quickly
                drained = 0
                while drained < 20:
                    try:
                        typ, seq, ts_us, payload = msg_q.get_nowait()
                    except queue.Empty:
                        break
                    drained += 1

                    if typ == TYPE_FRAME:
                        decoded = decode_frame(payload)
                        if decoded:
                            frame_seq, hub_ts, flags, cam, tof, mlx = decoded
                            with state.lock:
                                if cam is not None:
                                    state.cam = cam
                                if tof is not None:
                                    state.tof = tof
                                if mlx is not None:
                                    state.mlx = mlx
                            state.tick_fps()
                    elif typ in (TYPE_RESP, TYPE_EVENT):
                        try:
                            text = payload.decode("utf-8", errors="replace")
                        except Exception:
                            text = str(payload)
                        print(text.rstrip())
                        with state.lock:
                            if "OK info" in text:
                                state.info_text = text.strip()

                img = render_stack(state)
                cv2.imshow(win, img)

                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    break
                elif k == ord('1'):
                    cmd_q.put("STREAM mode=all")
                elif k == ord('2'):
                    cmd_q.put("STREAM mode=tof")
                elif k == ord('3'):
                    cmd_q.put("STREAM mode=mlx")
                elif k == ord('4'):
                    cmd_q.put("STREAM mode=cam")
                elif k == ord('0'):
                    cmd_q.put("STREAM mode=none")
                elif k == ord(' '):
                    stream_enabled = not stream_enabled
                    cmd_q.put(f"STREAM enable={1 if stream_enabled else 0}")

        finally:
            stop.set()
            cv2.destroyAllWindows()
    finally:
        try:
            ser.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
