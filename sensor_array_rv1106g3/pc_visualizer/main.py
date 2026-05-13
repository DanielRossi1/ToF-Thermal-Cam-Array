#!/usr/bin/env python3
"""
Sensor Hub Visualizer  v2
─────────────────────────
Transport  : TCP (default) or UDP — configurable at runtime
Graphics   : HistogramLUT colour editors, colormap pickers, FPS sparkline,
             drop counter, connection indicator, auto-reconnect
"""

import sys, os, time, socket, threading, queue, argparse, collections
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QGroupBox, QSplitter, QAction, QFileDialog, QSpinBox,
    QCheckBox, QFormLayout, QPlainTextEdit, QTabWidget,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, QByteArray, QSettings
from PyQt5.QtGui  import QImage, QPixmap, QColor, QFont, QPalette, QTransform

import pyqtgraph as pg

from calibration_page import CalibrationPage
from config_page import ConfigPage

from protocol import (
    SlipDecoder, parse_message, parse_frame, build_cmd,
    SyncedFrame, TofFrame, MlxFrame,
    MSG_FRAME, MSG_RESP, MSG_EVENT,
    TOF_ZONES, TOF_TPZ, MLX_W, MLX_H, MLX_PIXELS,
)

# ── Defaults ───────────────────────────────────────────────────────────────────
DEFAULT_HOST  = '192.168.1.67'
DEFAULT_PORT  = 9000
DEFAULT_PROTO = 'TCP'

# Hot-path logging (packet/frame prints can severely reduce FPS)
DEBUG_NET = False

TOF_MODES  = ['distance_mm', 'sigma_mm', 'signal_per_spad',
               'reflectance', 'status', 'ambient_per_spad', 'nb_targets']
COLORMAPS  = ['plasma', 'inferno', 'viridis', 'magma', 'turbo',
               'CET-L4', 'CET-D1', 'hot']


def _cmap(name: str):
    for n in (name, 'plasma', 'viridis'):
        try:
            return pg.colormap.get(n)
        except Exception:
            pass
    return None


# ═══════════════════════════════════════════════════════════════════════════════
#  Network Worker  (TCP stream with SLIP framing  OR  UDP datagrams)
# ═══════════════════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════════════════
#  Shared image primitive: ImageItem + HistogramLUT side by side
# ═══════════════════════════════════════════════════════════════════════════════

class SensorImageView(QWidget):
    """
    GraphicsLayoutWidget with an ImageItem on the left and a
    HistogramLUTItem (colour range editor + gradient bar) on the right.
    """

    def __init__(self, cmap_name: str = 'plasma', parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self._glw = pg.GraphicsLayoutWidget()
        layout.addWidget(self._glw)

        self._vb  = self._glw.addViewBox(row=0, col=0)
        self._vb.setAspectLocked(True)
        self._vb.invertY(False)

        self._img = pg.ImageItem()
        self._vb.addItem(self._img)

        self._hist = pg.HistogramLUTItem()
        self._hist.setImageItem(self._img)
        self._glw.addItem(self._hist, row=0, col=1)

        # Give the image 4× more horizontal space than the histogram bar
        self._glw.ci.layout.setColumnStretchFactor(0, 4)
        self._glw.ci.layout.setColumnStretchFactor(1, 1)

        self.set_colormap(cmap_name)

    def set_colormap(self, name: str):
        cm = _cmap(name)
        if cm:
            self._hist.gradient.setColorMap(cm)

    def set_image(self, data2d: np.ndarray):
        """
        data2d: 2-D float array in (cols, rows) order (pyqtgraph convention).
        The histogram range is updated automatically.
        """
        self._img.setImage(data2d, autoLevels=True)
        lo, hi = float(data2d.min()), float(data2d.max())
        if hi > lo:
            self._hist.setLevels(lo, hi)


# ═══════════════════════════════════════════════════════════════════════════════
#  ToF widget
# ═══════════════════════════════════════════════════════════════════════════════

class TofWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('ToF  VL53L8CH  8×8', parent)
        self._frame      = None
        self._mode       = 'distance_mm'
        self._target_idx = 0
        self._rot_k      = 0
        self._flip_x     = False
        self._flip_y     = False
        self._setup_ui()

    def set_transform(self, rot_deg: int = 0, flip_x: bool = False, flip_y: bool = False):
        try:
            self._rot_k = (int(rot_deg) // 90) % 4
        except Exception:
            self._rot_k = 0
        self._flip_x = bool(flip_x)
        self._flip_y = bool(flip_y)
        self._refresh()

    def _apply_transform(self, img2d: np.ndarray) -> np.ndarray:
        if self._rot_k:
            img2d = np.rot90(img2d, self._rot_k)
        if self._flip_x:
            img2d = np.fliplr(img2d)
        if self._flip_y:
            img2d = np.flipud(img2d)
        return img2d

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        self._view = SensorImageView('plasma')
        layout.addWidget(self._view, stretch=4)

        ctrl = QHBoxLayout()
        ctrl.addWidget(QLabel('Metric:'))
        self._mode_cb = QComboBox()
        self._mode_cb.addItems(TOF_MODES)
        self._mode_cb.currentTextChanged.connect(self._set_mode)
        ctrl.addWidget(self._mode_cb, stretch=2)

        ctrl.addWidget(QLabel('Target:'))
        self._tgt_sb = QSpinBox()
        self._tgt_sb.setRange(0, TOF_TPZ - 1)
        self._tgt_sb.valueChanged.connect(self._set_target)
        ctrl.addWidget(self._tgt_sb)

        ctrl.addWidget(QLabel('Cmap:'))
        self._cmap_cb = QComboBox()
        self._cmap_cb.addItems(COLORMAPS)
        self._cmap_cb.currentTextChanged.connect(self._view.set_colormap)
        ctrl.addWidget(self._cmap_cb, stretch=1)
        layout.addLayout(ctrl)

        self._stats = QLabel('—')
        self._stats.setAlignment(Qt.AlignCenter)
        self._stats.setStyleSheet('color: #90caf9; font-family: monospace;')
        layout.addWidget(self._stats)

    def _set_mode(self, mode):
        self._mode = mode
        self._refresh()

    def _set_target(self, idx):
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
        m  = self._mode

        if   m == 'distance_mm':      data, unit = tf.distance_mm[:, t].astype(np.float32),     'mm'
        elif m == 'sigma_mm':         data, unit = tf.sigma_mm[:, t].astype(np.float32),         'mm'
        elif m == 'signal_per_spad':  data, unit = tf.signal_per_spad[:, t].astype(np.float32),  'kcps/SPAD'
        elif m == 'reflectance':      data, unit = tf.reflectance[:, t].astype(np.float32),      '%'
        elif m == 'status':           data, unit = tf.status[:, t].astype(np.float32),           ''
        elif m == 'ambient_per_spad': data, unit = tf.ambient_per_spad.astype(np.float32),       'kcps/SPAD'
        elif m == 'nb_targets':       data, unit = tf.nb_targets.astype(np.float32),             ''
        else:
            return

        # Reshape flat 64-zone array to 8×8 grid; transpose for (col, row)
        img = data.reshape(8, 8).T
        img = self._apply_transform(img)
        self._view.set_image(img)

        valid = tf.status[:, t] == 5   # VL53L8CX status 5 = valid range
        nv    = int(valid.sum())
        if m == 'distance_mm' and nv > 0:
            dv = tf.distance_mm[valid, t]
            self._stats.setText(
                f'valid={nv}/64  mean={dv.mean():.0f}mm  '
                f'min={dv.min()}  max={dv.max()}  '
                f'T={tf.silicon_temp}°C')
        else:
            self._stats.setText(f'min={data.min():.1f}  max={data.max():.1f}  {unit}')


# ═══════════════════════════════════════════════════════════════════════════════
#  Thermal widget
# ═══════════════════════════════════════════════════════════════════════════════

class ThermalWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('Thermal  MLX90640  24×32', parent)
        self._rot_k  = 0
        self._flip_x = False
        self._flip_y = False
        self._setup_ui()

    def set_transform(self, rot_deg: int = 0, flip_x: bool = False, flip_y: bool = False):
        try:
            self._rot_k = (int(rot_deg) // 90) % 4
        except Exception:
            self._rot_k = 0
        self._flip_x = bool(flip_x)
        self._flip_y = bool(flip_y)

    def _apply_transform(self, img2d: np.ndarray) -> np.ndarray:
        if self._rot_k:
            img2d = np.rot90(img2d, self._rot_k)
        if self._flip_x:
            img2d = np.fliplr(img2d)
        if self._flip_y:
            img2d = np.flipud(img2d)
        return img2d

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        self._view = SensorImageView('inferno')
        layout.addWidget(self._view, stretch=4)

        ctrl = QHBoxLayout()
        self._auto_cb = QCheckBox('Auto range')
        self._auto_cb.setChecked(True)
        ctrl.addWidget(self._auto_cb)
        ctrl.addWidget(QLabel('Min °C:'))
        self._min_sb = QSpinBox()
        self._min_sb.setRange(-40, 300); self._min_sb.setValue(20)
        ctrl.addWidget(self._min_sb)
        ctrl.addWidget(QLabel('Max °C:'))
        self._max_sb = QSpinBox()
        self._max_sb.setRange(-40, 300); self._max_sb.setValue(40)
        ctrl.addWidget(self._max_sb)
        ctrl.addWidget(QLabel('Cmap:'))
        self._cmap_cb = QComboBox()
        self._cmap_cb.addItems(COLORMAPS)
        self._cmap_cb.setCurrentText('inferno')
        self._cmap_cb.currentTextChanged.connect(self._view.set_colormap)
        ctrl.addWidget(self._cmap_cb)
        layout.addLayout(ctrl)

        self._stats = QLabel('—')
        self._stats.setAlignment(Qt.AlignCenter)
        self._stats.setStyleSheet('color: #ffcc80; font-family: monospace;')
        layout.addWidget(self._stats)

    def update_frame(self, mlx: MlxFrame):
        pixels = mlx.pixels_c.reshape(MLX_H, MLX_W)   # 24 rows × 32 cols
        auto   = self._auto_cb.isChecked()
        if auto:
            vmin, vmax = float(pixels.min()), float(pixels.max())
        else:
            vmin, vmax = float(self._min_sb.value()), float(self._max_sb.value())

        clipped = np.clip(pixels, vmin, vmax)
        img = clipped.T  # (32 cols, 24 rows)
        img = self._apply_transform(img)
        self._view.set_image(img)
        if not auto:
            self._view._hist.setLevels(vmin, vmax)

        self._stats.setText(
            f'Ta={mlx.ta_celsius:.1f}°C  '
            f'min={pixels.min():.1f}°C  '
            f'max={pixels.max():.1f}°C  '
            f'mean={pixels.mean():.1f}°C')


# ═══════════════════════════════════════════════════════════════════════════════
#  Camera widget
# ═══════════════════════════════════════════════════════════════════════════════

class CameraWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('RGB Camera', parent)
        self._rot_deg = 0
        self._flip_x = False
        self._flip_y = False
        layout = QVBoxLayout(self)

        self._label = QLabel('No frame')
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setMinimumSize(320, 240)
        self._label.setStyleSheet(
            'background: #0a0a14; color: #444; border-radius: 6px;')
        layout.addWidget(self._label, stretch=1)

        self._stats = QLabel('—')
        self._stats.setAlignment(Qt.AlignCenter)
        self._stats.setStyleSheet('color: #a5d6a7; font-family: monospace;')
        layout.addWidget(self._stats)

    def set_transform(self, rot_deg: int = 0, flip_x: bool = False, flip_y: bool = False):
        try:
            self._rot_deg = int(rot_deg) % 360
        except Exception:
            self._rot_deg = 0
        self._flip_x = bool(flip_x)
        self._flip_y = bool(flip_y)

    def update_frame(self, jpeg: bytes, w: int, h: int, ts_us: int):
        img = QImage.fromData(QByteArray(jpeg), 'JPEG')
        if img.isNull():
            return

        if self._rot_deg:
            img = img.transformed(QTransform().rotate(self._rot_deg))
        if self._flip_x or self._flip_y:
            img = img.mirrored(self._flip_x, self._flip_y)

        pix = QPixmap.fromImage(img).scaled(
            self._label.width(), self._label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._label.setPixmap(pix)
        self._stats.setText(
            f'{w}×{h}  {len(jpeg)//1024} KiB ({len(jpeg)} B)  '
            f'ts={ts_us//1000} ms')


# ═══════════════════════════════════════════════════════════════════════════════
#  Statistics + FPS sparkline
# ═══════════════════════════════════════════════════════════════════════════════

class StatsWidget(QGroupBox):
    _HIST = 90   # sparkline width in samples

    def __init__(self, parent=None):
        super().__init__('Statistics', parent)
        layout = QVBoxLayout(self)

        form = QFormLayout()
        self._fps_lbl  = QLabel('0.0')
        self._seq_lbl  = QLabel('—')
        self._drop_lbl = QLabel('0')
        self._tof_lbl  = QLabel('—')
        self._mlx_lbl  = QLabel('—')
        self._cam_lbl  = QLabel('—')

        for lbl, w in [('FPS:', self._fps_lbl),     ('Seq:', self._seq_lbl),
                        ('Dropped:', self._drop_lbl), ('ToF:', self._tof_lbl),
                        ('MLX:', self._mlx_lbl),     ('Camera:', self._cam_lbl)]:
            form.addRow(lbl, w)
        layout.addLayout(form)

        # FPS sparkline
        self._spark = pg.PlotWidget()
        self._spark.setMaximumHeight(90)
        self._spark.setBackground('#0a0a14')
        self._spark.hideAxis('bottom')
        self._spark.showGrid(y=True, alpha=0.25)
        self._spark.setYRange(0, 35)
        self._spark.setLabel('left', 'FPS', color='#aaa', size='8pt')
        self._curve = self._spark.plot(
            pen=pg.mkPen(color='#4fc3f7', width=2))
        self._fill  = pg.FillBetweenItem(
            self._curve,
            self._spark.plot([0] * self._HIST, pen=None),
            brush=pg.mkBrush(79, 195, 247, 40))
        self._spark.addItem(self._fill)
        layout.addWidget(self._spark)

        self._times     = collections.deque(maxlen=300)
        self._fps_hist  = collections.deque([0.0] * self._HIST, maxlen=self._HIST)
        self._prev_seq  = None
        self._drops     = 0

    def update(self, sf: SyncedFrame):
        now = time.monotonic()
        self._times.append(now)

        window = [t for t in self._times if now - t < 2.0]
        fps    = len(window) / 2.0
        self._fps_hist.append(fps)
        self._fps_lbl.setText(f'{fps:.1f}')
        self._curve.setData(list(self._fps_hist))

        if self._prev_seq is not None:
            gap = int((sf.seq - self._prev_seq - 1) & 0xFFFF_FFFF)
            if gap:
                self._drops += gap
                self._drop_lbl.setText(str(self._drops))
        self._prev_seq = sf.seq
        self._seq_lbl.setText(str(sf.seq))

        self._tof_lbl.setText('✓' if sf.tof else '✗')
        self._mlx_lbl.setText('✓' if sf.mlx else '✗')
        self._cam_lbl.setText(
            f'✓  {len(sf.cam_jpeg)//1024} KiB' if sf.cam_jpeg else '✗')


# ═══════════════════════════════════════════════════════════════════════════════
#  Main Window
# ═══════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self, host: str = DEFAULT_HOST,
                 port: int  = DEFAULT_PORT,
                 proto: str = DEFAULT_PROTO):
        super().__init__()
        self.setWindowTitle('Sensor Hub Visualizer  v2')
        self.resize(1550, 980)

        self._worker = NetworkWorker()
        self._thread = QThread()
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.frame_received.connect(self._on_frame)
        self._worker.text_received.connect(self._log)
        self._worker.connection_lost.connect(self._on_conn_lost)

        self._q        = queue.Queue(maxsize=4)
        self._save_dir = None
        self._save_n   = 0
        self._first_frame_time = None
        self._frame_count = 0

        self._build_ui()

        self._settings = QSettings('ToF-Thermal-Cam-Array', 'pc_visualizer')
        self._config_page.applied.connect(self._apply_config)
        self._config_page.saved.connect(self._save_config)

        # 30 Hz UI refresh timer — keeps Qt repaints off the worker thread
        self._ui_timer = QTimer()
        self._ui_timer.timeout.connect(self._drain)
        self._ui_timer.start(33)

        # Auto-reconnect timer
        self._reconn_timer = QTimer()
        self._reconn_timer.timeout.connect(self._try_reconnect)

        # Load persisted config (overrides CLI defaults if present)
        self._load_persisted_config(host, port, proto)

    # ── UI construction ────────────────────────────────────────────────────────

    def _build_ui(self):
        # Menu
        mb = self.menuBar()
        fm = mb.addMenu('File')
        fm.addAction('Record to …', self._choose_save_dir)
        fm.addSeparator()
        fm.addAction('Quit', self.close)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 4, 6, 4)
        root.setSpacing(4)

        # ── Network config bar ─────────────────────────────────────────────────
        net_box = QGroupBox('Network')
        nl = QHBoxLayout(net_box)
        nl.setSpacing(10)

        nl.addWidget(QLabel('Protocol:'))
        self._proto_cb = QComboBox()
        self._proto_cb.addItems(['TCP', 'UDP'])
        self._proto_cb.setFixedWidth(72)
        nl.addWidget(self._proto_cb)

        nl.addWidget(QLabel('Host:'))
        self._host_edit = QLineEdit(DEFAULT_HOST)
        self._host_edit.setFixedWidth(150)
        nl.addWidget(self._host_edit)

        nl.addWidget(QLabel('Port:'))
        self._port_edit = QSpinBox()
        self._port_edit.setRange(1, 65535)
        self._port_edit.setValue(DEFAULT_PORT)
        self._port_edit.setFixedWidth(80)
        nl.addWidget(self._port_edit)

        self._conn_btn = QPushButton('Connect')
        self._conn_btn.setFixedWidth(110)
        self._conn_btn.setCheckable(True)
        self._conn_btn.clicked.connect(self._toggle_connect)
        nl.addWidget(self._conn_btn)

        self._auto_reconn_cb = QCheckBox('Auto-reconnect')
        nl.addWidget(self._auto_reconn_cb)

        nl.addStretch()

        # Connection status indicator
        self._conn_dot = QLabel('●')
        self._conn_dot.setStyleSheet('color:#ef5350; font-size:20px;')
        nl.addWidget(self._conn_dot)

        self._conn_label = QLabel('Disconnected')
        self._conn_label.setStyleSheet('color:#888;')
        nl.addWidget(self._conn_label)

        root.addWidget(net_box)

        # ── Main splitter ──────────────────────────────────────────────────────
        split = QSplitter(Qt.Horizontal)
        root.addWidget(split, stretch=1)

        # Left: pages (Live + Calibration)
        self._tabs = QTabWidget()
        split.addWidget(self._tabs)

        live = QWidget()
        sg = QGridLayout(live)
        sg.setSpacing(4)

        self._cam_w  = CameraWidget()
        self._tof_w  = TofWidget()
        self._mlx_w  = ThermalWidget()
        self._stat_w = StatsWidget()

        sg.addWidget(self._cam_w,  0, 0, 2, 2)
        sg.addWidget(self._tof_w,  0, 2)
        sg.addWidget(self._mlx_w,  1, 2)
        sg.addWidget(self._stat_w, 0, 3, 2, 1)
        sg.setColumnStretch(0, 3)
        sg.setColumnStretch(1, 3)
        sg.setColumnStretch(2, 2)
        sg.setColumnStretch(3, 1)

        self._calib_page = CalibrationPage()
        self._config_page = ConfigPage()
        self._tabs.addTab(live, "Live")
        self._tabs.addTab(self._calib_page, "Calibration")
        self._tabs.addTab(self._config_page, "Config")

        # Right: control panel
        rw = QWidget()
        rw.setMaximumWidth(340)
        rl = QVBoxLayout(rw)
        rl.setSpacing(6)

        # Stream
        sb = QGroupBox('Stream')
        sbl = QFormLayout(sb)
        self._stream_mode = QComboBox()
        self._stream_mode.addItems(['all', 'tof', 'mlx', 'cam', 'none'])
        sbl.addRow('Mode:', self._stream_mode)
        row = QHBoxLayout()
        start_b = QPushButton('▶  Start')
        stop_b  = QPushButton('■  Stop')
        start_b.clicked.connect(lambda: self._stream(True))
        stop_b.clicked.connect( lambda: self._stream(False))
        row.addWidget(start_b); row.addWidget(stop_b)
        sbl.addRow(row)
        rl.addWidget(sb)

        # ToF settings
        tb = QGroupBox('ToF Settings')
        tbl = QFormLayout(tb)
        self._tof_side = QComboBox(); self._tof_side.addItems(['8', '4'])
        self._tof_hz   = QSpinBox(); self._tof_hz.setRange(1, 60); self._tof_hz.setValue(15)
        self._tof_it   = QSpinBox(); self._tof_it.setRange(1, 1000); self._tof_it.setValue(50)
        tbl.addRow('Resolution:', self._tof_side)
        tbl.addRow('Rate (Hz):',  self._tof_hz)
        tbl.addRow('Int. (ms):',  self._tof_it)
        ab = QPushButton('Apply ToF')
        ab.clicked.connect(self._apply_tof)
        tbl.addRow(ab)
        rl.addWidget(tb)

        # MLX settings
        mb2 = QGroupBox('MLX Settings')
        mbl = QFormLayout(mb2)
        self._mlx_refresh = QComboBox()
        self._mlx_refresh.addItems(['1', '2', '4', '8', '16', '32'])
        self._mlx_mode_cb = QComboBox()
        self._mlx_mode_cb.addItems(['chess', 'interleaved'])
        mbl.addRow('Refresh (Hz):', self._mlx_refresh)
        mbl.addRow('Mode:',         self._mlx_mode_cb)
        mb3 = QPushButton('Apply MLX')
        mb3.clicked.connect(self._apply_mlx)
        mbl.addRow(mb3)
        rl.addWidget(mb2)

        # Recording
        rec_box = QGroupBox('Recording')
        recl    = QVBoxLayout(rec_box)
        self._save_cb = QCheckBox('Record frames')
        self._save_cb.toggled.connect(self._toggle_save)
        recl.addWidget(self._save_cb)
        self._save_path_lbl = QLabel('No directory selected')
        self._save_path_lbl.setWordWrap(True)
        self._save_path_lbl.setStyleSheet('color:#666; font-size:10px;')
        recl.addWidget(self._save_path_lbl)
        pick = QPushButton('Choose directory …')
        pick.clicked.connect(self._choose_save_dir)
        recl.addWidget(pick)
        rl.addWidget(rec_box)

        # Raw command
        cmd_box = QGroupBox('Raw Command')
        cmbl    = QVBoxLayout(cmd_box)
        self._cmd_edit = QLineEdit()
        self._cmd_edit.setPlaceholderText('PING  /  GET INFO  /  …')
        self._cmd_edit.returnPressed.connect(self._send_raw)
        cmbl.addWidget(self._cmd_edit)
        send_b = QPushButton('Send')
        send_b.clicked.connect(self._send_raw)
        cmbl.addWidget(send_b)
        rl.addWidget(cmd_box)

        # Console
        self._console = QPlainTextEdit()
        self._console.setReadOnly(True)
        self._console.setMaximumBlockCount(800)
        self._console.setFont(QFont('Monospace', 9))
        self._console.setStyleSheet(
            'background:#08080f; color:#90caf9; border: 1px solid #1e1e3a;')
        rl.addWidget(self._console, stretch=1)

        split.addWidget(rw)
        split.setSizes([1200, 340])

    # ── Persistent config ───────────────────────────────────────────────────

    def _default_config(self, host: str, port: int, proto: str) -> dict:
        return {
            'proto': str(proto).upper(),
            'host': str(host),
            'port': int(port),
            'auto_reconnect': False,
            'tof_rot': 0,
            'tof_flip_x': False,
            'tof_flip_y': False,
            'mlx_rot': 0,
            'mlx_flip_x': False,
            'mlx_flip_y': False,
            'cam_rot': 0,
            'cam_flip_x': False,
            'cam_flip_y': False,
        }

    def _load_persisted_config(self, host: str, port: int, proto: str):
        cfg = self._default_config(host, port, proto)
        s = self._settings

        cfg['proto'] = s.value('net/proto', cfg['proto'], type=str).upper()
        cfg['host'] = s.value('net/host', cfg['host'], type=str)
        cfg['port'] = s.value('net/port', cfg['port'], type=int)
        cfg['auto_reconnect'] = s.value('net/auto_reconnect', cfg['auto_reconnect'], type=bool)

        for p in ('tof', 'mlx', 'cam'):
            cfg[f'{p}_rot'] = s.value(f'view/{p}_rot', cfg[f'{p}_rot'], type=int)
            cfg[f'{p}_flip_x'] = s.value(f'view/{p}_flip_x', cfg[f'{p}_flip_x'], type=bool)
            cfg[f'{p}_flip_y'] = s.value(f'view/{p}_flip_y', cfg[f'{p}_flip_y'], type=bool)

        self._apply_config(cfg)
        self._config_page.set_config(cfg)

    def _apply_config(self, cfg: dict):
        # Network bar
        self._host_edit.setText(str(cfg.get('host', '')).strip())
        self._port_edit.setValue(int(cfg.get('port', DEFAULT_PORT)))
        proto = str(cfg.get('proto', 'TCP')).upper()
        idx = self._proto_cb.findText(proto)
        if idx >= 0:
            self._proto_cb.setCurrentIndex(idx)
        self._auto_reconn_cb.setChecked(bool(cfg.get('auto_reconnect', False)))

        # View transforms
        self._tof_w.set_transform(cfg.get('tof_rot', 0), cfg.get('tof_flip_x', False), cfg.get('tof_flip_y', False))
        self._mlx_w.set_transform(cfg.get('mlx_rot', 0), cfg.get('mlx_flip_x', False), cfg.get('mlx_flip_y', False))
        self._cam_w.set_transform(cfg.get('cam_rot', 0), cfg.get('cam_flip_x', False), cfg.get('cam_flip_y', False))

        # Keep Config tab in sync
        if hasattr(self, '_config_page') and self._config_page is not None:
            self._config_page.set_config(cfg)

    def _save_config(self, cfg: dict):
        s = self._settings
        s.setValue('net/proto', str(cfg.get('proto', 'TCP')).upper())
        s.setValue('net/host', str(cfg.get('host', '')).strip())
        s.setValue('net/port', int(cfg.get('port', DEFAULT_PORT)))
        s.setValue('net/auto_reconnect', bool(cfg.get('auto_reconnect', False)))

        for p in ('tof', 'mlx', 'cam'):
            s.setValue(f'view/{p}_rot', int(cfg.get(f'{p}_rot', 0)))
            s.setValue(f'view/{p}_flip_x', bool(cfg.get(f'{p}_flip_x', False)))
            s.setValue(f'view/{p}_flip_y', bool(cfg.get(f'{p}_flip_y', False)))

        s.sync()
        self._apply_config(cfg)
        self._config_page.set_status('Saved')

    # ── Connection ─────────────────────────────────────────────────────────────

    def _toggle_connect(self, checked: bool):
        if checked:
            host  = self._host_edit.text().strip()
            port  = self._port_edit.value()
            proto = self._proto_cb.currentText()

            ok = self._worker.connect_to(host, port, proto)
            if ok:
                # Start thread only once
                if not self._thread.isRunning():
                    self._thread.start()
                self._conn_btn.setText('Disconnect')
                self._conn_dot.setStyleSheet('color:#66bb6a; font-size:20px;')
                self._conn_label.setText(f'{proto}  {host}:{port}')
                self._log(f'Connected → {proto}  {host}:{port}')
                self._log('[INFO] Waiting for device boot event...')
                # Auto-start streaming after a brief delay to let device boot
                QTimer.singleShot(1000, lambda: self._stream(True))
                self._reconn_timer.stop()
            else:
                self._conn_btn.setChecked(False)
        else:
            self._stream(False)
            self._worker.disconnect()
            self._conn_btn.setText('Connect')
            self._conn_dot.setStyleSheet('color:#ef5350; font-size:20px;')
            self._conn_label.setText('Disconnected')
            self._log('Disconnected')
            self._reconn_timer.stop()

    def _on_conn_lost(self, reason: str):
        self._conn_btn.setChecked(False)
        self._conn_btn.setText('Connect')
        self._conn_dot.setStyleSheet('color:#ffa726; font-size:20px;')
        self._conn_label.setText('Connection lost')
        self._log(f'[LOST] {reason}')
        if self._auto_reconn_cb.isChecked():
            self._reconn_timer.start(3000)

    def _try_reconnect(self):
        if self._conn_btn.isChecked():
            self._reconn_timer.stop()
            return
        self._log('[INFO] Reconnecting …')
        self._conn_btn.setChecked(True)
        self._toggle_connect(True)

    # ── Frame pipeline ─────────────────────────────────────────────────────────

    def _on_frame(self, sf: SyncedFrame):
        now = time.monotonic()
        if self._first_frame_time is None:
            self._first_frame_time = now
            self._log(f'[OK] First frame received! (seq={sf.seq})')
        self._frame_count += 1
        try:
            self._q.put_nowait(sf)
        except queue.Full:
            pass   # drop oldest; the UI can't keep up — that's fine

    def _drain(self):
        while not self._q.empty():
            try:
                sf = self._q.get_nowait()
                self._display(sf)
                if self._save_cb.isChecked() and self._save_dir:
                    self._save(sf)
            except queue.Empty:
                break

    def _display(self, sf: SyncedFrame):
        if sf.tof:      self._tof_w.update_frame(sf.tof)
        if sf.mlx:      self._mlx_w.update_frame(sf.mlx)
        if sf.cam_jpeg: self._cam_w.update_frame(
            sf.cam_jpeg, sf.cam_w, sf.cam_h, sf.cam_ts_us)
        self._stat_w.update(sf)
        if hasattr(self, '_calib_page') and self._calib_page is not None:
            self._calib_page.update_from_synced_frame(sf)

    # ── Recording ──────────────────────────────────────────────────────────────

    def _choose_save_dir(self):
        d = QFileDialog.getExistingDirectory(self, 'Select recording directory')
        if d:
            self._save_dir = d
            self._save_n   = 0
            self._save_path_lbl.setText(d)
            self._save_cb.setChecked(True)
            self._log(f'Recording → {d}')

    def _toggle_save(self, on: bool):
        if on and not self._save_dir:
            self._choose_save_dir()

    def _save(self, sf: SyncedFrame):
        base = os.path.join(self._save_dir, f'{self._save_n:08d}')
        self._save_n += 1
        try:
            if sf.tof:
                np.save(f'{base}_tof_dist.npy',    sf.tof.distance_mm)
                np.save(f'{base}_tof_sigma.npy',   sf.tof.sigma_mm)
                np.save(f'{base}_tof_status.npy',  sf.tof.status)
                np.save(f'{base}_tof_signal.npy',  sf.tof.signal_per_spad)
                np.save(f'{base}_tof_reflect.npy', sf.tof.reflectance)
                np.save(f'{base}_tof_ambient.npy', sf.tof.ambient_per_spad)
            if sf.mlx:
                np.save(f'{base}_thermal.npy', sf.mlx.pixels_c.reshape(MLX_H, MLX_W))
            if sf.cam_jpeg:
                with open(f'{base}_cam.jpg', 'wb') as f:
                    f.write(sf.cam_jpeg)
            with open(f'{base}_meta.txt', 'w') as f:
                f.write(f'seq={sf.seq} hub_ts_us={sf.hub_ts_us} '
                        f'flags={sf.flags:#010x}\n')
        except Exception as e:
            self._log(f'[SAVE ERR] {e}')

    # ── Commands ───────────────────────────────────────────────────────────────

    def _cmd(self, text: str):
        self._worker.send(build_cmd(text))
        self._log(f'→ {text}')

    def _stream(self, enable: bool):
        mode = self._stream_mode.currentText()
        self._cmd(f'STREAM enable={1 if enable else 0} mode={mode}')

    def _apply_tof(self):
        self._cmd(f'SET TOF side={self._tof_side.currentText()} '
                  f'hz={self._tof_hz.value()} it_ms={self._tof_it.value()} continuous=1')

    def _apply_mlx(self):
        self._cmd(f'SET MLX mode={self._mlx_mode_cb.currentText()} '
                  f'res=18 refresh={self._mlx_refresh.currentText()}')

    def _send_raw(self):
        t = self._cmd_edit.text().strip()
        if t:
            self._cmd(t)
            self._cmd_edit.clear()

    # ── Console ────────────────────────────────────────────────────────────────

    def _log(self, msg: str):
        self._console.appendPlainText(msg)

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        self._worker.disconnect()
        self._thread.quit()
        self._thread.wait(1000)
        super().closeEvent(event)


# ═══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Sensor Hub Visualizer v2')
    parser.add_argument('--host',  default=DEFAULT_HOST,
                        help='Device IP address (default: %(default)s)')
    parser.add_argument('--port',  type=int, default=DEFAULT_PORT,
                        help='TCP/UDP port (default: %(default)s)')
    parser.add_argument('--proto', default=DEFAULT_PROTO,
                        choices=['TCP', 'UDP'],
                        help='Transport protocol (default: %(default)s)')
    args = parser.parse_args()

    # pyqtgraph global config
    pg.setConfigOption('background', '#0d0d1a')
    pg.setConfigOption('foreground', '#d0d0d0')
    pg.setConfigOption('antialias',  True)

    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Deep blue-black dark palette
    pal = QPalette()
    c = {
        QPalette.Window:          QColor( 13,  13,  26),
        QPalette.WindowText:      QColor(220, 220, 220),
        QPalette.Base:            QColor(  8,   8,  20),
        QPalette.AlternateBase:   QColor( 22,  22,  40),
        QPalette.ToolTipBase:     QColor(200, 200, 200),
        QPalette.ToolTipText:     QColor( 20,  20,  30),
        QPalette.Text:            QColor(215, 215, 215),
        QPalette.Button:          QColor( 35,  35,  65),
        QPalette.ButtonText:      QColor(215, 215, 215),
        QPalette.BrightText:      QColor(255, 255, 255),
        QPalette.Highlight:       QColor( 55,  90, 180),
        QPalette.HighlightedText: QColor(255, 255, 255),
        QPalette.Link:            QColor( 79, 195, 247),
        QPalette.LinkVisited:     QColor(149, 117, 205),
    }
    for role, colour in c.items():
        pal.setColor(role, colour)
    app.setPalette(pal)

    win = MainWindow(host=args.host, port=args.port, proto=args.proto)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()