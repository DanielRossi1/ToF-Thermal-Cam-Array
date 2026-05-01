#!/usr/bin/env python3
"""
Sensor Hub Visualizer
Real-time display of synchronized ToF + Thermal + RGB data from ESP32-S3 hub.
"""

import sys
import os
import time
import struct
import argparse
import threading
import queue
import io
import numpy as np

import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit, QTextEdit,
    QGroupBox, QSplitter, QStatusBar, QAction, QFileDialog, QSpinBox,
    QCheckBox, QTabWidget, QSlider, QFormLayout, QMessageBox, QPlainTextEdit
)
from PyQt5.QtCore import (
    Qt, QTimer, pyqtSignal, QObject, QThread, QByteArray
)
from PyQt5.QtGui import (
    QImage, QPixmap, QColor, QPainter, QFont, QIcon
)

import pyqtgraph as pg

from sensor_array_idf.pc_visualizer.protocol import (
    SlipDecoder, parse_message, parse_frame, build_cmd,
    SyncedFrame, TofFrame, MlxFrame,
    MSG_FRAME, MSG_RESP, MSG_EVENT,
    TOF_ZONES, TOF_TPZ, MLX_W, MLX_H, MLX_PIXELS
)


# ── Colour maps ────────────────────────────────────────────────────────────────

def _make_colormap(name: str):
    """Return a pyqtgraph colormap by name (with fallback)."""
    try:
        return pg.colormap.get(name)
    except Exception:
        return pg.colormap.get('viridis')


# ── Serial worker ─────────────────────────────────────────────────────────────

class SerialWorker(QObject):
    frame_received  = pyqtSignal(object)   # SyncedFrame
    text_received   = pyqtSignal(str)
    connection_lost = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._port: serial.Serial | None = None
        self._running = False
        self._lock = threading.Lock()
        self._slip = SlipDecoder(self._on_slip_frame)

    def connect(self, port: str, baud: int) -> bool:
        try:
            self._port = serial.Serial(port, baud, timeout=0.05)
            self._running = True
            return True
        except Exception as e:
            self.text_received.emit(f"[ERR] Cannot open {port}: {e}")
            return False

    def disconnect(self):
        self._running = False
        with self._lock:
            if self._port and self._port.is_open:
                self._port.close()
            self._port = None
        self._slip.reset()

    def send(self, data: bytes):
        with self._lock:
            if self._port and self._port.is_open:
                try:
                    self._port.write(data)
                except Exception as e:
                    self.text_received.emit(f"[ERR] write: {e}")

    def run(self):
        while self._running:
            try:
                with self._lock:
                    port = self._port
                if port is None or not port.is_open:
                    time.sleep(0.05)
                    continue
                data = port.read(4096)
                if data:
                    self._slip.feed(data)
            except serial.SerialException as e:
                self.connection_lost.emit(str(e))
                self._running = False
            except Exception as e:
                self.text_received.emit(f"[ERR] read: {e}")
                time.sleep(0.01)

    def _on_slip_frame(self, raw: bytes):
        result = parse_message(raw)
        if result is None:
            return
        mtype, seq, ts_us, payload = result
        if mtype == MSG_FRAME:
            sf = parse_frame(payload)
            if sf:
                self.frame_received.emit(sf)
        elif mtype in (MSG_RESP, MSG_EVENT):
            try:
                self.text_received.emit(f"[{seq}] {payload.decode('utf-8', errors='replace')}")
            except Exception:
                pass


# ── ToF visualizer widget ─────────────────────────────────────────────────────

class TofWidget(QGroupBox):
    MODES = ['distance_mm', 'sigma_mm', 'signal_per_spad', 'reflectance',
             'status', 'ambient_per_spad', 'nb_targets']

    def __init__(self, parent=None):
        super().__init__("ToF VL53L8CH  8×8", parent)
        self._mode = 'distance_mm'
        self._cmap = _make_colormap('plasma')
        self._frame: TofFrame | None = None
        self._target_idx = 0  # which target to show (0..3)
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # Image view
        self._plot = pg.GraphicsLayoutWidget()
        self._view = self._plot.addViewBox()
        self._view.setAspectLocked(True)
        self._img  = pg.ImageItem()
        self._view.addItem(self._img)
        self._img.setLookupTable(self._cmap.getLookupTable(nPts=256))
        layout.addWidget(self._plot, stretch=3)

        # Controls row
        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel("Metric:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(self.MODES)
        self._mode_combo.currentTextChanged.connect(self._set_mode)
        ctrl.addWidget(self._mode_combo, stretch=2)

        ctrl.addWidget(QLabel("Target:"))
        self._tgt_spin = QSpinBox()
        self._tgt_spin.setRange(0, 3)
        self._tgt_spin.valueChanged.connect(self._set_target)
        ctrl.addWidget(self._tgt_spin)
        layout.addLayout(ctrl)

        # Stats label
        self._stats = QLabel("—")
        self._stats.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._stats)

    def _set_mode(self, mode: str):
        self._mode = mode
        self._refresh()

    def _set_target(self, idx: int):
        self._target_idx = idx
        self._refresh()

    def update_frame(self, tof: TofFrame):
        self._frame = tof
        self._refresh()

    def _refresh(self):
        if self._frame is None:
            return
        tf = self._frame
        t  = self._target_idx

        if   self._mode == 'distance_mm':
            data = tf.distance_mm[:, t].astype(np.float32)
            unit = 'mm'
        elif self._mode == 'sigma_mm':
            data = tf.sigma_mm[:, t].astype(np.float32)
            unit = 'mm'
        elif self._mode == 'signal_per_spad':
            data = tf.signal_per_spad[:, t].astype(np.float32)
            unit = 'kcps/SPAD'
        elif self._mode == 'reflectance':
            data = tf.reflectance[:, t].astype(np.float32)
            unit = '%'
        elif self._mode == 'status':
            data = tf.status[:, t].astype(np.float32)
            unit = ''
        elif self._mode == 'ambient_per_spad':
            data = tf.ambient_per_spad.astype(np.float32)
            unit = 'kcps/SPAD'
        elif self._mode == 'nb_targets':
            data = tf.nb_targets.astype(np.float32)
            unit = ''
        else:
            return

        grid = data.reshape(8, 8)
        # Normalise for display
        vmin, vmax = grid.min(), grid.max()
        if vmax > vmin:
            norm = (grid - vmin) / (vmax - vmin)
        else:
            norm = np.zeros_like(grid)

        # ImageItem expects (col, row) → transpose
        self._img.setImage(norm.T * 255, autoLevels=False, levels=(0, 255))

        # Stats
        valid = tf.status[:, t] == 5  # 5 = valid in VL53L8CX
        n_valid = valid.sum()
        if self._mode == 'distance_mm' and n_valid > 0:
            mean_d = tf.distance_mm[valid, t].mean()
            self._stats.setText(
                f"valid={n_valid}/64  mean={mean_d:.0f} mm  "
                f"T={tf.silicon_temp}°C"
            )
        else:
            self._stats.setText(f"min={vmin:.1f}  max={vmax:.1f}  {unit}")


# ── Thermal visualizer widget ──────────────────────────────────────────────────

class ThermalWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("MLX90640  24×32", parent)
        self._cmap = _make_colormap('inferno')
        self._auto_range = True
        self._vmin = 20.0
        self._vmax = 40.0
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        self._plot = pg.GraphicsLayoutWidget()
        self._view = self._plot.addViewBox()
        self._view.setAspectLocked(True)
        self._img  = pg.ImageItem()
        self._view.addItem(self._img)
        self._img.setLookupTable(self._cmap.getLookupTable(nPts=256))
        layout.addWidget(self._plot, stretch=3)

        ctrl = QHBoxLayout()
        self._auto_cb = QCheckBox("Auto range")
        self._auto_cb.setChecked(True)
        self._auto_cb.toggled.connect(self._toggle_auto)
        ctrl.addWidget(self._auto_cb)
        ctrl.addWidget(QLabel("Min °C:"))
        self._min_spin = QSpinBox(); self._min_spin.setRange(-40, 300); self._min_spin.setValue(20)
        self._min_spin.valueChanged.connect(self._set_range)
        ctrl.addWidget(self._min_spin)
        ctrl.addWidget(QLabel("Max °C:"))
        self._max_spin = QSpinBox(); self._max_spin.setRange(-40, 300); self._max_spin.setValue(40)
        self._max_spin.valueChanged.connect(self._set_range)
        ctrl.addWidget(self._max_spin)
        layout.addLayout(ctrl)

        self._stats = QLabel("—")
        self._stats.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._stats)

    def _toggle_auto(self, checked: bool):
        self._auto_range = checked

    def _set_range(self):
        self._vmin = self._min_spin.value()
        self._vmax = self._max_spin.value()

    def update_frame(self, mlx: MlxFrame):
        pixels = mlx.pixels_c.reshape(MLX_H, MLX_W)  # 24 rows × 32 cols
        if self._auto_range:
            vmin, vmax = pixels.min(), pixels.max()
        else:
            vmin, vmax = self._vmin, self._vmax
        if vmax > vmin:
            norm = (np.clip(pixels, vmin, vmax) - vmin) / (vmax - vmin)
        else:
            norm = np.zeros_like(pixels)
        self._img.setImage(norm.T * 255, autoLevels=False, levels=(0, 255))
        self._stats.setText(
            f"Ta={mlx.ta_celsius:.1f}°C  "
            f"min={pixels.min():.1f}°C  max={pixels.max():.1f}°C"
        )


# ── Camera widget ─────────────────────────────────────────────────────────────

class CameraWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("RGB Camera", parent)
        layout = QVBoxLayout(self)
        self._label = QLabel()
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setMinimumSize(320, 240)
        self._label.setStyleSheet("background: #111; color: #888;")
        self._label.setText("No frame")
        layout.addWidget(self._label)
        self._stats = QLabel("—")
        self._stats.setAlignment(Qt.AlignCenter)
        layout.addWidget(self._stats)

    def update_frame(self, jpeg: bytes, w: int, h: int, ts_us: int):
        if not jpeg:
            return
        img = QImage.fromData(QByteArray(jpeg), "JPEG")
        if img.isNull():
            return
        pix = QPixmap.fromImage(img).scaled(
            self._label.width(), self._label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self._label.setPixmap(pix)
        self._stats.setText(f"{w}×{h}  {len(jpeg)//1024} KiB  ts={ts_us//1000} ms")


# ── Status panel ──────────────────────────────────────────────────────────────

class StatsWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("Statistics", parent)
        layout = QFormLayout(self)
        self._fps   = QLabel("0")
        self._seq   = QLabel("—")
        self._tof   = QLabel("—")
        self._mlx   = QLabel("—")
        self._cam   = QLabel("—")
        self._lag   = QLabel("—")
        layout.addRow("FPS:",    self._fps)
        layout.addRow("Seq:",    self._seq)
        layout.addRow("ToF:",    self._tof)
        layout.addRow("MLX:",    self._mlx)
        layout.addRow("Cam:",    self._cam)
        layout.addRow("Latency:",self._lag)
        self._frame_times = []

    def update(self, sf: SyncedFrame):
        now = time.time()
        self._frame_times.append(now)
        self._frame_times = [t for t in self._frame_times if now - t < 2.0]
        fps = len(self._frame_times) / 2.0
        self._fps.setText(f"{fps:.1f}")
        self._seq.setText(str(sf.seq))
        self._tof.setText("✓" if sf.tof else "✗")
        self._mlx.setText("✓" if sf.mlx else "✗")
        self._cam.setText(
            f"✓ {len(sf.cam_jpeg)//1024}KiB" if sf.cam_jpeg else "✗"
        )
        self._lag.setText("—")


# ── Main window ───────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self, port: str = '', baud: int = 2000000):
        super().__init__()
        self.setWindowTitle("Sensor Hub Visualizer")
        self.resize(1400, 900)

        self._worker    = SerialWorker()
        self._thread    = QThread()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.frame_received.connect(self._on_frame)
        self._worker.text_received.connect(self._on_text)
        self._worker.connection_lost.connect(self._on_conn_lost)

        self._save_dir: str | None = None
        self._save_count = 0
        self._pending_frames: queue.Queue[SyncedFrame] = queue.Queue(maxsize=4)

        self._setup_ui()

        # Refresh UI at ~30 Hz from a timer — avoids Qt cross-thread pixmap issues.
        self._ui_timer = QTimer()
        self._ui_timer.timeout.connect(self._drain_queue)
        self._ui_timer.start(33)

        if port:
            self._port_combo.setCurrentText(port)
            self._baud_combo.setCurrentText(str(baud))
            QTimer.singleShot(200, self._toggle_connect)

    def _setup_ui(self):
        # ── Menu ──────────────────────────────────────────────────────────────
        mb = self.menuBar()
        file_m = mb.addMenu("File")
        save_a = QAction("Save frames to …", self)
        save_a.triggered.connect(self._choose_save_dir)
        file_m.addAction(save_a)
        quit_a = QAction("Quit", self)
        quit_a.triggered.connect(self.close)
        file_m.addAction(quit_a)

        # ── Status bar ────────────────────────────────────────────────────────
        self._status_bar = self.statusBar()
        self._conn_label = QLabel("Disconnected")
        self._status_bar.addPermanentWidget(self._conn_label)

        # ── Central splitter ─────────────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        top = QVBoxLayout(central)

        # Connection bar
        conn_bar = QHBoxLayout()
        conn_bar.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setEditable(True)
        self._refresh_ports()
        conn_bar.addWidget(self._port_combo, stretch=2)

        self._refresh_btn = QPushButton("↻")
        self._refresh_btn.setFixedWidth(30)
        self._refresh_btn.clicked.connect(self._refresh_ports)
        conn_bar.addWidget(self._refresh_btn)

        conn_bar.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        for b in ["2000000", "921600", "460800", "115200"]:
            self._baud_combo.addItem(b)
        conn_bar.addWidget(self._baud_combo)

        self._conn_btn = QPushButton("Connect")
        self._conn_btn.clicked.connect(self._toggle_connect)
        conn_bar.addWidget(self._conn_btn)
        conn_bar.addStretch()
        top.addLayout(conn_bar)

        # Main area
        splitter = QSplitter(Qt.Horizontal)
        top.addWidget(splitter, stretch=1)

        # Left: sensors
        sensors = QWidget()
        sl = QGridLayout(sensors)
        self._tof_w  = TofWidget()
        self._mlx_w  = ThermalWidget()
        self._cam_w  = CameraWidget()
        self._stat_w = StatsWidget()
        sl.addWidget(self._cam_w,  0, 0, 2, 2)
        sl.addWidget(self._tof_w,  0, 2)
        sl.addWidget(self._mlx_w,  1, 2)
        sl.addWidget(self._stat_w, 0, 3, 2, 1)
        sl.setColumnStretch(0, 3)
        sl.setColumnStretch(1, 3)
        sl.setColumnStretch(2, 2)
        sl.setColumnStretch(3, 1)
        splitter.addWidget(sensors)

        # Right: controls + console
        right = QWidget()
        rl = QVBoxLayout(right)
        right.setMaximumWidth(340)

        # Stream controls
        stream_box = QGroupBox("Stream")
        sbl = QFormLayout(stream_box)
        self._stream_mode = QComboBox()
        self._stream_mode.addItems(["all", "tof", "mlx", "cam", "none"])
        sbl.addRow("Mode:", self._stream_mode)

        stream_btn_row = QHBoxLayout()
        btn_start = QPushButton("Start")
        btn_stop  = QPushButton("Stop")
        btn_start.clicked.connect(lambda: self._send_stream(enable=True))
        btn_stop.clicked.connect(lambda:  self._send_stream(enable=False))
        stream_btn_row.addWidget(btn_start)
        stream_btn_row.addWidget(btn_stop)
        sbl.addRow(stream_btn_row)
        rl.addWidget(stream_box)

        # ToF controls
        tof_box = QGroupBox("ToF settings")
        tbl = QFormLayout(tof_box)
        self._tof_side = QComboBox(); self._tof_side.addItems(["8", "4"])
        self._tof_hz   = QSpinBox();  self._tof_hz.setRange(1, 60); self._tof_hz.setValue(15)
        self._tof_it   = QSpinBox();  self._tof_it.setRange(1, 1000); self._tof_it.setValue(50)
        tbl.addRow("Resolution:", self._tof_side)
        tbl.addRow("Rate (Hz):", self._tof_hz)
        tbl.addRow("Int. time (ms):", self._tof_it)
        btn_tof = QPushButton("Apply ToF")
        btn_tof.clicked.connect(self._apply_tof)
        tbl.addRow(btn_tof)
        rl.addWidget(tof_box)

        # MLX controls
        mlx_box = QGroupBox("MLX settings")
        mbl = QFormLayout(mlx_box)
        self._mlx_refresh = QComboBox()
        self._mlx_refresh.addItems(["1", "2", "4", "8", "16", "32"])
        mbl.addRow("Refresh (Hz):", self._mlx_refresh)
        self._mlx_mode = QComboBox(); self._mlx_mode.addItems(["chess", "interleaved"])
        mbl.addRow("Mode:", self._mlx_mode)
        btn_mlx = QPushButton("Apply MLX")
        btn_mlx.clicked.connect(self._apply_mlx)
        mbl.addRow(btn_mlx)
        rl.addWidget(mlx_box)

        # Save toggle
        self._save_cb = QCheckBox("Save frames")
        self._save_cb.toggled.connect(self._toggle_save)
        rl.addWidget(self._save_cb)

        # Command line
        cmd_box = QGroupBox("Raw command")
        cbl = QVBoxLayout(cmd_box)
        self._cmd_entry = QLineEdit()
        self._cmd_entry.setPlaceholderText("e.g.  PING  or  GET INFO")
        self._cmd_entry.returnPressed.connect(self._send_raw_cmd)
        cbl.addWidget(self._cmd_entry)
        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self._send_raw_cmd)
        cbl.addWidget(btn_send)
        rl.addWidget(cmd_box)

        # Console
        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(500)
        self._console.setFont(QFont("Monospace", 9))

        splitter.addWidget(right)
        splitter.setSizes([1060, 340])

    # ── Connection ────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        current = self._port_combo.currentText()
        self._port_combo.clear()
        for p in serial.tools.list_ports.comports():
            # Skip ports with no hardware info (phantom ttyS* entries)
            if p.description == 'n/a' and p.hwid == 'n/a':
                continue
            self._port_combo.addItem(p.device)
        if current:
            idx = self._port_combo.findText(current)
            if idx >= 0:
                self._port_combo.setCurrentIndex(idx)

    def _toggle_connect(self):
        if self._conn_btn.text() == "Connect":
            port = self._port_combo.currentText().strip()
            baud = int(self._baud_combo.currentText())
            if self._worker.connect(port, baud):
                self._thread.start()
                self._conn_btn.setText("Disconnect")
                self._conn_label.setText(f"Connected  {port} @ {baud}")
                self._log(f"Connected to {port} @ {baud}")
            # If connect failed, worker emitted text_received with the error
        else:
            self._worker.disconnect()
            self._conn_btn.setText("Connect")
            self._conn_label.setText("Disconnected")
            self._log("Disconnected")

    def _on_conn_lost(self, reason: str):
        self._conn_label.setText("Connection lost")
        self._conn_btn.setText("Connect")
        self._log(f"[LOST] {reason}")

    # ── Frame processing ──────────────────────────────────────────────────────

    def _on_frame(self, sf: SyncedFrame):
        try:
            self._pending_frames.put_nowait(sf)
        except queue.Full:
            pass  # drop if UI can't keep up

    def _drain_queue(self):
        while not self._pending_frames.empty():
            try:
                sf = self._pending_frames.get_nowait()
                self._display_frame(sf)
                if self._save_dir and self._save_cb.isChecked():
                    self._save_frame(sf)
            except queue.Empty:
                break

    def _display_frame(self, sf: SyncedFrame):
        if sf.tof:
            self._tof_w.update_frame(sf.tof)
        if sf.mlx:
            self._mlx_w.update_frame(sf.mlx)
        if sf.cam_jpeg:
            self._cam_w.update_frame(sf.cam_jpeg, sf.cam_w, sf.cam_h, sf.cam_ts_us)
        self._stat_w.update(sf)

    # ── Save frames ───────────────────────────────────────────────────────────

    def _choose_save_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Select save directory")
        if d:
            self._save_dir   = d
            self._save_count = 0
            self._save_cb.setChecked(True)
            self._log(f"Saving to {d}")

    def _toggle_save(self, checked: bool):
        if checked and not self._save_dir:
            self._choose_save_dir()

    def _save_frame(self, sf: SyncedFrame):
        n = self._save_count
        self._save_count += 1
        base = os.path.join(self._save_dir, f"{n:08d}")
        try:
            if sf.tof:
                np.save(f"{base}_tof_dist.npy",       sf.tof.distance_mm)
                np.save(f"{base}_tof_sigma.npy",      sf.tof.sigma_mm)
                np.save(f"{base}_tof_status.npy",     sf.tof.status)
                np.save(f"{base}_tof_signal.npy",     sf.tof.signal_per_spad)
                np.save(f"{base}_tof_reflect.npy",    sf.tof.reflectance)
                np.save(f"{base}_tof_ambient.npy",    sf.tof.ambient_per_spad)
            if sf.mlx:
                np.save(f"{base}_thermal.npy", sf.mlx.pixels_c.reshape(MLX_H, MLX_W))
            if sf.cam_jpeg:
                with open(f"{base}_camera.jpg", 'wb') as f:
                    f.write(sf.cam_jpeg)
            with open(f"{base}_meta.txt", 'w') as f:
                f.write(f"seq={sf.seq} hub_ts_us={sf.hub_ts_us} flags={sf.flags:#010x}\n")
        except Exception as e:
            self._log(f"[SAVE ERR] {e}")

    # ── Commands ──────────────────────────────────────────────────────────────

    def _send(self, text: str):
        self._worker.send(build_cmd(text))
        self._log(f"→ {text.strip()}")

    def _send_stream(self, enable: bool):
        mode = self._stream_mode.currentText()
        self._send(f"STREAM enable={1 if enable else 0} mode={mode}")

    def _apply_tof(self):
        side = self._tof_side.currentText()
        hz   = self._tof_hz.value()
        it   = self._tof_it.value()
        self._send(f"SET TOF side={side} hz={hz} it_ms={it} continuous=1")

    def _apply_mlx(self):
        refresh = self._mlx_refresh.currentText()
        mode    = self._mlx_mode.currentText()
        self._send(f"SET MLX mode={mode} res=18 refresh={refresh}")

    def _send_raw_cmd(self):
        text = self._cmd_entry.text().strip()
        if text:
            self._send(text)
            self._cmd_entry.clear()

    # ── Console ───────────────────────────────────────────────────────────────

    def _on_text(self, text: str):
        self._log(f"← {text.rstrip()}")

    def _log(self, msg: str):
        self._console.appendPlainText(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._worker.disconnect()
        self._thread.quit()
        self._thread.wait(1000)
        super().closeEvent(event)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Sensor Hub Visualizer")
    parser.add_argument("--port", default="", help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate")
    args = parser.parse_args()

    pg.setConfigOption('background', '#1a1a2e')
    pg.setConfigOption('foreground', '#e0e0e0')
    pg.setConfigOption('antialias', True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    from PyQt5.QtGui import QPalette
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(26, 26, 46))
    pal.setColor(QPalette.WindowText,      QColor(224, 224, 224))
    pal.setColor(QPalette.Base,            QColor(18, 18, 30))
    pal.setColor(QPalette.AlternateBase,   QColor(36, 36, 60))
    pal.setColor(QPalette.ToolTipBase,     QColor(200, 200, 200))
    pal.setColor(QPalette.ToolTipText,     QColor(30, 30, 30))
    pal.setColor(QPalette.Text,            QColor(224, 224, 224))
    pal.setColor(QPalette.Button,          QColor(48, 48, 80))
    pal.setColor(QPalette.ButtonText,      QColor(224, 224, 224))
    pal.setColor(QPalette.Highlight,       QColor(80, 120, 200))
    pal.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(pal)

    win = MainWindow(port=args.port, baud=args.baud)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
