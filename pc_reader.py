"""
pc_reader.py — Synchronized sensor hub receiver
================================================
Connects via USB-UART to the ESP32-S3, reads synchronized tuples of:
  (JPEG camera frame, VL53L8CH 8×8 ToF map, MLX90640 32×24 thermal frame)

Requirements:
    pip install pyserial numpy opencv-python matplotlib

Usage:
    python pc_reader.py --port COM3          # Windows
    python pc_reader.py --port /dev/ttyUSB0  # Linux / Mac
    python pc_reader.py --port /dev/ttyUSB0 --save ./frames
"""

import argparse
import struct
import zlib
import time
import os
import sys
from dataclasses import dataclass, field
from typing import Optional

import serial
import numpy as np

# Optional visualization (gracefully disabled if not installed)
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ─────────────────────────────────────────────────────────────────────────────
# Protocol constants  (must match firmware)
# ─────────────────────────────────────────────────────────────────────────────
MAGIC       = 0xDEADBEEF
MAGIC_BYTES = struct.pack("<I", MAGIC)
BAUD        = 2_000_000

TOF_ZONES   = 64          # 8×8
MLX_W, MLX_H = 32, 24
MLX_PIXELS  = MLX_W * MLX_H


# ─────────────────────────────────────────────────────────────────────────────
# Data container
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SensorFrame:
    seq:        int
    ts_us:      int                       # µs since ESP32 boot
    tof_dist:   np.ndarray = field(default_factory=lambda: np.zeros((8,8), dtype=np.uint16))
    tof_sigma:  np.ndarray = field(default_factory=lambda: np.zeros((8,8), dtype=np.uint16))
    tof_status: np.ndarray = field(default_factory=lambda: np.zeros((8,8), dtype=np.uint8))
    mlx_frame:  np.ndarray = field(default_factory=lambda: np.zeros((24,32), dtype=np.float32))
    cam_w:      int = 0
    cam_h:      int = 0
    cam_jpeg:   Optional[bytes] = None

    @property
    def cam_image(self) -> Optional[np.ndarray]:
        """Decode JPEG to BGR numpy array (requires OpenCV)."""
        if not HAS_CV2 or not self.cam_jpeg:
            return None
        arr = np.frombuffer(self.cam_jpeg, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)


# ─────────────────────────────────────────────────────────────────────────────
# Parser
# ─────────────────────────────────────────────────────────────────────────────
class SensorHub:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = 5.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._buf = b""
        self.frames_received = 0
        self.frames_corrupt   = 0
        print(f"[SensorHub] Connected to {port} @ {baud} baud")

    def close(self):
        self.ser.close()

    # ── Low-level read ────────────────────────────────────────────────────────
    def _read_exact(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            chunk = self.ser.read(n - len(data))
            if not chunk:
                raise TimeoutError(f"Serial timeout waiting for {n} bytes")
            data += chunk
        return data

    def _sync(self) -> bool:
        """Scan the stream until MAGIC bytes are found."""
        buf = b""
        while True:
            b = self.ser.read(1)
            if not b:
                return False
            buf = (buf + b)[-4:]
            if buf == MAGIC_BYTES:
                return True

    # ── Frame parser ─────────────────────────────────────────────────────────
    def read_frame(self) -> Optional[SensorFrame]:
        if not self._sync():
            return None

        # We'll accumulate raw bytes for CRC check
        raw = MAGIC_BYTES

        def read_and_track(n: int) -> bytes:
            nonlocal raw
            d = self._read_exact(n)
            raw += d
            return d

        try:
            # Header
            seq,    = struct.unpack_from("<I", read_and_track(4))
            ts_us,  = struct.unpack_from("<Q", read_and_track(8))

            # ── ToF ──
            zones,  = struct.unpack_from("<H", read_and_track(2))
            dist_raw   = read_and_track(zones * 2)
            sigma_raw  = read_and_track(zones * 2)
            status_raw = read_and_track(zones)

            tof_dist   = np.frombuffer(dist_raw,   dtype="<u2").reshape(8, 8)
            tof_sigma  = np.frombuffer(sigma_raw,  dtype="<u2").reshape(8, 8)
            tof_status = np.frombuffer(status_raw, dtype="u1").reshape(8, 8)

            # ── MLX ──
            mlx_w, = struct.unpack_from("<H", read_and_track(2))
            mlx_h, = struct.unpack_from("<H", read_and_track(2))
            mlx_raw = read_and_track(mlx_w * mlx_h * 4)
            mlx_frame = np.frombuffer(mlx_raw, dtype="<f4").reshape(mlx_h, mlx_w).copy()

            # ── Camera ──
            cam_w,   = struct.unpack_from("<I", read_and_track(4))
            cam_h,   = struct.unpack_from("<I", read_and_track(4))
            cam_len, = struct.unpack_from("<I", read_and_track(4))
            cam_jpeg = read_and_track(cam_len) if cam_len > 0 else None

            # ── CRC ──
            # CRC covers everything in raw (magic + all fields before CRC)
            crc_recv, = struct.unpack_from("<I", self._read_exact(4))
            crc_calc  = zlib.crc32(raw) & 0xFFFFFFFF

            if crc_recv != crc_calc:
                print(f"[WARN] CRC mismatch seq={seq}: recv={crc_recv:#010x} calc={crc_calc:#010x}")
                self.frames_corrupt += 1
                return None

        except (struct.error, TimeoutError, ValueError) as e:
            print(f"[ERR] Parse error: {e}")
            self.frames_corrupt += 1
            return None

        self.frames_received += 1

        return SensorFrame(
            seq=seq,
            ts_us=ts_us,
            tof_dist=tof_dist,
            tof_sigma=tof_sigma,
            tof_status=tof_status,
            mlx_frame=mlx_frame,
            cam_w=cam_w,
            cam_h=cam_h,
            cam_jpeg=cam_jpeg,
        )

    def __iter__(self):
        while True:
            frame = self.read_frame()
            if frame is not None:
                yield frame


# ─────────────────────────────────────────────────────────────────────────────
# Save utilities
# ─────────────────────────────────────────────────────────────────────────────
def save_frame(frame: SensorFrame, out_dir: str):
    base = os.path.join(out_dir, f"{frame.seq:08d}")
    # NumPy arrays
    np.save(f"{base}_tof_dist.npy",   frame.tof_dist)
    np.save(f"{base}_tof_sigma.npy",  frame.tof_sigma)
    np.save(f"{base}_tof_status.npy", frame.tof_status)
    np.save(f"{base}_thermal.npy",    frame.mlx_frame)
    # JPEG
    if frame.cam_jpeg:
        with open(f"{base}_camera.jpg", "wb") as f:
            f.write(frame.cam_jpeg)
    # Metadata (CSV-friendly one-liner)
    with open(f"{base}_meta.txt", "w") as f:
        f.write(f"seq={frame.seq} ts_us={frame.ts_us} "
                f"cam={frame.cam_w}x{frame.cam_h} "
                f"mlx={frame.mlx_frame.min():.1f}~{frame.mlx_frame.max():.1f}C\n")


# ─────────────────────────────────────────────────────────────────────────────
# Live visualization (matplotlib)
# ─────────────────────────────────────────────────────────────────────────────
def visualize_live(hub: SensorHub, save_dir: Optional[str] = None):
    if not HAS_MPL:
        print("[WARN] matplotlib not installed — no live viz")
        return

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle("ESP32-S3 Sensor Hub — live", fontsize=13)

    ax_cam, ax_tof, ax_mlx = axes

    ax_cam.set_title("Camera (JPEG)")
    ax_tof.set_title("ToF 8×8 distance (mm)")
    ax_mlx.set_title("Thermal 32×24 (°C)")

    # Placeholder images
    im_cam = ax_cam.imshow(np.zeros((240, 320, 3), dtype=np.uint8))
    im_tof = ax_tof.imshow(np.zeros((8, 8)),    cmap="plasma_r", vmin=0, vmax=4000)
    im_mlx = ax_mlx.imshow(np.zeros((24, 32)),  cmap="inferno",  vmin=15, vmax=40)

    cb_tof = fig.colorbar(im_tof, ax=ax_tof, fraction=0.046, pad=0.04)
    cb_mlx = fig.colorbar(im_mlx, ax=ax_mlx, fraction=0.046, pad=0.04)
    cb_tof.set_label("mm")
    cb_mlx.set_label("°C")

    stats_text = fig.text(0.5, 0.02, "", ha="center", fontsize=9, color="gray")

    fps_ts = [time.time()]
    fps_buf = [0.0]

    def update(_):
        frame = hub.read_frame()
        if frame is None:
            return

        # FPS
        now = time.time()
        dt = now - fps_ts[0]
        fps_ts[0] = now
        fps_buf[0] = 0.9 * fps_buf[0] + 0.1 * (1.0 / max(dt, 1e-9))

        # Camera
        if frame.cam_jpeg and HAS_CV2:
            img = frame.cam_image
            if img is not None:
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                im_cam.set_data(img_rgb)
                im_cam.set_extent([0, img_rgb.shape[1], img_rgb.shape[0], 0])
                ax_cam.set_xlim(0, img_rgb.shape[1])
                ax_cam.set_ylim(img_rgb.shape[0], 0)

        # ToF (filter invalid zones: status != 5 → set to 0)
        tof_disp = frame.tof_dist.astype(float)
        tof_disp[frame.tof_status != 5] = np.nan
        valid = tof_disp[~np.isnan(tof_disp)]
        if len(valid):
            im_tof.set_clim(vmin=0, vmax=max(valid.max(), 1))
        im_tof.set_data(tof_disp)

        # MLX
        im_mlx.set_data(frame.mlx_frame)
        im_mlx.set_clim(vmin=frame.mlx_frame.min(), vmax=frame.mlx_frame.max())

        # Stats
        tof_valid_pct = 100.0 * np.sum(frame.tof_status == 5) / TOF_ZONES
        stats_text.set_text(
            f"seq={frame.seq}  ts={frame.ts_us/1e6:.2f}s  fps≈{fps_buf[0]:.1f}  "
            f"ToF valid zones={tof_valid_pct:.0f}%  "
            f"thermal={frame.mlx_frame.min():.1f}–{frame.mlx_frame.max():.1f}°C  "
            f"corrupt={hub.frames_corrupt}"
        )

        if save_dir:
            save_frame(frame, save_dir)

        return im_cam, im_tof, im_mlx

    ani = animation.FuncAnimation(fig, update, interval=1, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
# Headless mode (just print stats + optionally save)
# ─────────────────────────────────────────────────────────────────────────────
def headless(hub: SensorHub, save_dir: Optional[str] = None, max_frames: int = 0):
    t0 = time.time()
    for frame in hub:
        elapsed = time.time() - t0
        fps = frame.seq / max(elapsed, 1e-9)
        tof_valid = np.sum(frame.tof_status == 5)
        print(
            f"seq={frame.seq:6d}  ts={frame.ts_us/1e6:8.2f}s  fps={fps:5.1f}  "
            f"tof_valid={tof_valid:2d}/64  "
            f"thermal=[{frame.mlx_frame.min():5.1f} – {frame.mlx_frame.max():5.1f}°C]  "
            f"cam_bytes={len(frame.cam_jpeg) if frame.cam_jpeg else 0:6d}",
            flush=True,
        )
        if save_dir:
            save_frame(frame, save_dir)
        if max_frames and frame.seq >= max_frames:
            break


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ESP32-S3 Sensor Hub reader")
    parser.add_argument("--port",   required=True,  help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud",   default=BAUD,   type=int, help=f"Baud rate (default {BAUD})")
    parser.add_argument("--save",   default=None,   help="Directory to save frames (optional)")
    parser.add_argument("--headless", action="store_true", help="Disable live visualization")
    parser.add_argument("--frames", default=0,      type=int, help="Stop after N frames (0=infinite)")
    args = parser.parse_args()

    if args.save:
        os.makedirs(args.save, exist_ok=True)
        print(f"[INFO] Saving frames to: {args.save}")

    hub = SensorHub(args.port, args.baud)

    try:
        if args.headless or not HAS_MPL:
            headless(hub, args.save, args.frames)
        else:
            visualize_live(hub, args.save)
    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped. Received={hub.frames_received} corrupt={hub.frames_corrupt}")
    finally:
        hub.close()


if __name__ == "__main__":
    main()
