#!/usr/bin/env python3
"""\
tof_viewer.py — Robust live OpenCV viewer for VL53L8CH (ToF)
───────────────────────────────────────────────────────────
Matches the MLX viewer "pipeline":
- Survives ESP32 resets / ttyACM renumbering
- Re-syncs on READY or first valid FRAME line
- Displays a live heatmap (distance in mm)

Requirements:
  pip install pyserial numpy opencv-contrib-python

Usage:
  python3 tof_viewer.py                          # auto-detect port
  python3 tof_viewer.py /dev/ttyACM0             # explicit port
  python3 tof_viewer.py /dev/serial/by-id/...    # recommended on Linux

Controls:
  q / ESC  — quit
  c        — cycle colormap
  s        — save snapshot PNG
  +/-      — increase/decrease display scale
"""

import sys
import time
import threading

import numpy as np
import serial
import serial.tools.list_ports
import cv2

# ── Config ────────────────────────────────────────────────────────────────────
BAUD = 460800
SERIAL_PORT = sys.argv[1] if len(sys.argv) > 1 else None
SCALE = 32
WIN_NAME = "VL53L8CH ToF"

ESPRESSIF_VID = 0x303A  # 12346

COLORMAPS = [
    ("Inferno", cv2.COLORMAP_INFERNO),
    ("Jet", cv2.COLORMAP_JET),
    ("Hot", cv2.COLORMAP_HOT),
    ("Plasma", cv2.COLORMAP_PLASMA),
    ("Viridis", cv2.COLORMAP_VIRIDIS),
    ("Bone", cv2.COLORMAP_BONE),
    ("Rainbow", cv2.COLORMAP_RAINBOW),
]

cmap_idx = 0

# ── Shared state ──────────────────────────────────────────────────────────────
lock = threading.Lock()
latest_grid = None  # float32, shape (side, side), NaN for invalid
latest_seq = 0
last_fps = 0.0
sensor_ready = False
snapshot_req = False


# ── Serial port resolution ────────────────────────────────────────────────────

def _iter_ports():
    return list(serial.tools.list_ports.comports())


def _port_label(p):
    desc = (p.description or "").strip()
    mfg = (p.manufacturer or "").strip()
    if desc and mfg and mfg not in desc:
        return f"{desc} ({mfg})"
    return desc or mfg or "n/a"


def _score_port(p):
    score = 0
    if p.vid == ESPRESSIF_VID:
        score += 200
    label = ((p.description or "") + " " + (p.manufacturer or "")).lower()
    if "espressif" in label:
        score += 120
    if "jtag" in label:
        score += 20
    if "serial" in label:
        score += 10
    if (p.device or "").startswith("/dev/ttyACM"):
        score += 5
    if (p.device or "").startswith("/dev/ttyUSB"):
        score += 2
    return score


def _select_port(preferred_device=None, identity=None):
    ports = _iter_ports()
    if not ports:
        return None

    if preferred_device:
        for p in ports:
            if p.device == preferred_device:
                return p

    if identity:
        serial_number = identity.get("serial_number")
        location = identity.get("location")
        vid = identity.get("vid")
        pid = identity.get("pid")

        if serial_number:
            for p in ports:
                if p.serial_number and p.serial_number == serial_number:
                    return p

        if location:
            for p in ports:
                if getattr(p, "location", None) == location:
                    return p

        if vid is not None and pid is not None:
            for p in ports:
                if p.vid == vid and p.pid == pid:
                    return p

    return max(ports, key=_score_port)


def find_port():
    p = _select_port()
    return p.device if p else None


def _open_serial(device):
    kwargs = dict(timeout=1, write_timeout=1)
    try:
        ser = serial.Serial(device, BAUD, exclusive=True, **kwargs)
    except TypeError:
        ser = serial.Serial(device, BAUD, **kwargs)

    # Avoid toggling DTR/RTS (some boards reset on it)
    try:
        ser.dtr = False
        ser.rts = False
    except Exception:
        pass

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    time.sleep(0.15)
    return ser


def _parse_frame_line(raw: str):
    if not raw.startswith("FRAME,"):
        return None
    parts = raw.split(",")
    if len(parts) < 4:
        return None
    try:
        seq_n = int(parts[1])
        side = int(parts[2])
    except ValueError:
        return None

    if side not in (4, 8):
        return None

    n = side * side
    if len(parts) != 3 + n:
        return None

    try:
        vals = np.array(parts[3:], dtype=np.float32)
    except ValueError:
        return None

    # -1 means invalid
    vals[vals < 0] = np.nan
    grid = vals.reshape(side, side)
    return seq_n, grid


# ── Serial reader thread ──────────────────────────────────────────────────────

def serial_thread(port):
    global latest_grid, latest_seq, last_fps, sensor_ready

    preferred_device = port
    identity = None

    print(f"[Serial] Opening {preferred_device} @ {BAUD}...")
    backoff_s = 0.5
    while True:
        pinfo = _select_port(preferred_device=preferred_device, identity=identity)
        if not pinfo:
            print("[Serial] No serial ports found — retrying")
            time.sleep(1.0)
            continue

        device = pinfo.device
        try:
            ser = _open_serial(device)
            identity = {
                "device": device,
                "vid": pinfo.vid,
                "pid": pinfo.pid,
                "serial_number": pinfo.serial_number,
                "location": getattr(pinfo, "location", None),
                "label": _port_label(pinfo),
            }
            if device != preferred_device:
                print(f"[Serial] Using {device} ({identity['label']})")
            break
        except serial.SerialException as e:
            print(f"[Serial] Cannot open {device}: {e} — retrying")
            time.sleep(min(backoff_s, 2.0))
            backoff_s = min(backoff_s * 1.5, 2.0)

    print("[Serial] Connected. Waiting for sensor stream...")

    fps_count = 0
    fps_t0 = time.monotonic()
    synced = False
    last_misc_print = 0.0

    while True:
        try:
            line = ser.readline()
            if not line:
                continue
            raw = line.decode("ascii", errors="ignore").strip()
        except (serial.SerialException, OSError) as e:
            dev = identity.get("device") if identity else preferred_device
            print(f"[Serial] Lost connection on {dev}: {e}")
            with lock:
                sensor_ready = False
            synced = False
            try:
                ser.close()
            except Exception:
                pass

            backoff_s = 0.5
            while True:
                pinfo = _select_port(preferred_device=preferred_device, identity=identity)
                if not pinfo:
                    time.sleep(1.0)
                    continue
                try:
                    ser = _open_serial(pinfo.device)
                    identity = {
                        "device": pinfo.device,
                        "vid": pinfo.vid,
                        "pid": pinfo.pid,
                        "serial_number": pinfo.serial_number,
                        "location": getattr(pinfo, "location", None),
                        "label": _port_label(pinfo),
                    }
                    print(f"[Serial] Reconnected on {identity['device']} ({identity['label']})")
                    break
                except serial.SerialException:
                    time.sleep(min(backoff_s, 2.0))
                    backoff_s = min(backoff_s * 1.5, 2.0)
            continue

        if not raw:
            continue

        if raw == "READY":
            if not synced:
                print("[Serial] Sensor READY — receiving frames")
            synced = True
            with lock:
                sensor_ready = True
            continue

        if raw.startswith("ERROR"):
            print(f"[Sensor] {raw}")
            continue

        if raw.startswith("#"):
            print(f"[Sensor] {raw[2:]}")
            continue

        parsed = _parse_frame_line(raw)
        if not synced:
            if parsed:
                synced = True
                with lock:
                    sensor_ready = True
                print("[Serial] Synced on FRAME — receiving frames")
            else:
                now = time.monotonic()
                if (now - last_misc_print) >= 0.25:
                    print(f"[Sensor] {raw}")
                    last_misc_print = now
                continue

        if not parsed:
            continue

        seq_n, grid = parsed
        with lock:
            latest_grid = grid
            latest_seq = seq_n

        fps_count += 1
        elapsed = time.monotonic() - fps_t0
        if elapsed >= 1.0:
            last_fps = fps_count / elapsed
            fps_count = 0
            fps_t0 = time.monotonic()


# ── Render ────────────────────────────────────────────────────────────────────

def _valid_min_max(grid):
    if grid is None:
        return None
    if np.all(np.isnan(grid)):
        return None
    return float(np.nanmin(grid)), float(np.nanmax(grid))


def render(grid, seq_n, fps, cmap, scale):
    side = grid.shape[0]
    mm_min, mm_max = _valid_min_max(grid)
    if mm_min is None:
        colored = np.zeros((side, side, 3), dtype=np.uint8)
        disp = cv2.resize(colored, (side * scale, side * scale), interpolation=cv2.INTER_NEAREST)
        cv2.putText(disp, "No valid targets", (10, disp.shape[0] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 200, 80), 1)
        return disp

    mm_range = max(mm_max - mm_min, 1.0)

    g = grid.copy()
    # Map NaN to min for visualization (will appear dark)
    g = np.nan_to_num(g, nan=mm_min)

    norm = ((g - mm_min) / mm_range * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cmap)

    disp = cv2.resize(colored, (side * scale, side * scale), interpolation=cv2.INTER_NEAREST)
    dh, dw = disp.shape[:2]

    # Colorbar
    bar_w, bar_h = 20, dh - 60
    bar_x, bar_y = dw - bar_w - 10, 30
    bar_img = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
    bar_col = cv2.applyColorMap(bar_img, cmap)
    disp[bar_y:bar_y + bar_h, bar_x:bar_x + bar_w] = bar_col
    cv2.rectangle(disp, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (180, 180, 180), 1)
    cv2.putText(disp, f"{mm_max:.0f}mm", (bar_x - 70, bar_y + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)
    cv2.putText(disp, f"{mm_min:.0f}mm", (bar_x - 70, bar_y + bar_h),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

    # Center crosshair
    cx, cy = dw // 2, dh // 2
    center = grid[side // 2, side // 2]
    cv2.line(disp, (cx - 12, cy), (cx + 12, cy), (255, 255, 255), 1)
    cv2.line(disp, (cx, cy - 12), (cx, cy + 12), (255, 255, 255), 1)
    if not np.isnan(center):
        cv2.putText(disp, f"{center:.0f}mm", (cx + 6, cy - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Min/max markers (ignore NaNs)
    valid = np.isfinite(grid)
    if np.any(valid):
        flat = grid.copy()
        flat[~valid] = np.nan
        min_pos = np.unravel_index(np.nanargmin(flat), flat.shape)
        max_pos = np.unravel_index(np.nanargmax(flat), flat.shape)
        for pos, label, color in [
            (min_pos, f"min {mm_min:.0f}mm", (255, 150, 0)),
            (max_pos, f"max {mm_max:.0f}mm", (0, 100, 255)),
        ]:
            px = int(pos[1] * scale + scale // 2)
            py = int(pos[0] * scale + scale // 2)
            cv2.drawMarker(disp, (px, py), color, cv2.MARKER_CROSS, 14, 1)
            cv2.putText(disp, label, (px + 6, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

    # Status bar
    cmap_name = COLORMAPS[cmap_idx][0]
    status = (
        f"seq={seq_n}  fps={fps:.1f}  grid={side}x{side}  "
        f"scale={scale}x  map={cmap_name}  [q]quit [c]cmap [s]save [+/-]scale"
    )
    overlay = disp.copy()
    cv2.rectangle(overlay, (0, dh - 22), (dw, dh), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, disp, 0.45, 0, disp)
    cv2.putText(disp, status, (6, dh - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (220, 220, 220), 1)

    return disp


def waiting_screen(scale, msg="Waiting for sensor stream..."):
    img = np.zeros((8 * scale, 8 * scale, 3), dtype=np.uint8)
    cv2.putText(img, msg, (10, (8 * scale) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 200, 80), 1)
    cv2.putText(img, "Reset ESP32 if stuck", (10, (8 * scale) // 2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
    return img


def main():
    global cmap_idx, SCALE, snapshot_req

    port = SERIAL_PORT or find_port()
    if not port:
        print("[FATAL] No serial port found.")
        print("  Usage: python3 tof_viewer.py /dev/ttyACM0")
        sys.exit(1)

    t = threading.Thread(target=serial_thread, args=(port,), daemon=True)
    t.start()

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)
    print("[Display] Window open — press q or ESC to quit")

    snapshot_count = 0

    try:
        while True:
            key = cv2.waitKey(60) & 0xFF

            if key in (ord('q'), 27):
                break
            elif key == ord('c'):
                cmap_idx = (cmap_idx + 1) % len(COLORMAPS)
                print(f"[Display] Colormap → {COLORMAPS[cmap_idx][0]}")
            elif key == ord('s'):
                snapshot_req = True
            elif key in (ord('+'), ord('=')):
                SCALE = min(SCALE + 4, 96)
            elif key == ord('-'):
                SCALE = max(SCALE - 4, 16)

            with lock:
                grid = latest_grid
                seq_n = latest_seq
                fps = last_fps
                ready = sensor_ready

            if not ready or grid is None:
                cv2.imshow(WIN_NAME, waiting_screen(SCALE))
                continue

            cmap = COLORMAPS[cmap_idx][1]
            img = render(grid, seq_n, fps, cmap, SCALE)

            if snapshot_req:
                fname = f"tof_{snapshot_count:04d}.png"
                cv2.imwrite(fname, img)
                print(f"[Display] Saved {fname}")
                snapshot_count += 1
                snapshot_req = False

            cv2.imshow(WIN_NAME, img)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
