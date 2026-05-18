#!/usr/bin/env python3
"""
Sensor Hub Visualizer  v2
─────────────────────────
Transport  : TCP (default) or UDP — configurable at runtime
Graphics   : HistogramLUT colour editors, colormap pickers, FPS sparkline,
             drop counter, connection indicator, auto-reconnect
"""

import os, time, queue
import numpy as np

from PyQt5.QtWidgets import QDesktopWidget 
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit,
    QGroupBox, QSplitter, QFileDialog, QSpinBox,
    QCheckBox, QFormLayout, QPlainTextEdit, QTabWidget,
)
from PyQt5.QtCore import Qt, QTimer,QThread, QSettings
from PyQt5.QtGui  import QFont

from calibration.calibration_page import CalibrationPage
from config.config_page import ConfigPage

from network.protocol import (
    build_cmd,
    SyncedFrame,
    MLX_W, MLX_H,
)

from combined_view.overlapped_page import OverlapPage
import json

from network.network import NetworkWorker
from widgets.ToF_widget import TofWidget
from widgets.thermal_widget import ThermalWidget
from widgets.camera_widget import CameraWidget
from widgets.stats_widget import StatsWidget

from config.defaults import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_PROTO,
)
class MainWindow(QMainWindow):
    def __init__(self, host: str = DEFAULT_HOST,
                 port: int  = DEFAULT_PORT,
                 proto: str = DEFAULT_PROTO):
        super().__init__()
        self.setWindowTitle('Sensor Hub Visualizer')
        self.setMinimumSize(800, 600) 
        self.resize(1200, 800)

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
        self._overlap_page = OverlapPage()
        self._config_page = ConfigPage()
        self._tabs.addTab(live, "Live")
        self._tabs.addTab(self._overlap_page, "Overlap View")
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
        
        # 1. Override any hidden minimum sizes in your imported widgets
        self._cam_w.setMinimumSize(50, 50)
        self._tof_w.setMinimumSize(50, 50)
        self._mlx_w.setMinimumSize(50, 50)
        self._stat_w.setMinimumSize(50, 50)

        # 2. Use proportional stretching instead of rigid pixel counts
        split.setStretchFactor(0, 3) # Left side gets 75%
        split.setStretchFactor(1, 1) # Right side gets 25%

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

        try:
            if s.contains('calib/camera_matrix'):
                cfg['camera_matrix'] = json.loads(s.value('calib/camera_matrix', type=str))
                cfg['dist_coeffs'] = json.loads(s.value('calib/dist_coeffs', type=str))
                if s.contains('calib/camera_model'):
                    cfg['camera_model'] = s.value('calib/camera_model', 'pinhole', type=str)
            if s.contains('calib/R_tof_to_rgb'):
                cfg['R_tof_to_rgb'] = json.loads(s.value('calib/R_tof_to_rgb', type=str))
                cfg['t_tof_to_rgb'] = json.loads(s.value('calib/t_tof_to_rgb', type=str))
        except Exception as e:
            print(f"Could not parse persisted calibration JSON: {e}")

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

        if hasattr(self, '_calib_page'):
            self._calib_page.set_config(cfg)

        if hasattr(self, '_overlap_page'):
            self._overlap_page.set_calibration(cfg)

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
            
        if 'camera_matrix' in cfg:
            s.setValue('calib/camera_matrix', json.dumps(cfg['camera_matrix']))
            s.setValue('calib/dist_coeffs', json.dumps(cfg['dist_coeffs']))
            if 'camera_model' in cfg:
                s.setValue('calib/camera_model', str(cfg.get('camera_model', 'pinhole')))
            if 'R_tof_to_rgb' in cfg:
                s.setValue('calib/R_tof_to_rgb', json.dumps(cfg['R_tof_to_rgb']))
                s.setValue('calib/t_tof_to_rgb', json.dumps(cfg['t_tof_to_rgb']))

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
            
        # Add this line to feed frames to the new overlap page:
        if hasattr(self, '_overlap_page') and self._overlap_page is not None:
            self._overlap_page.update_from_synced_frame(sf)

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

