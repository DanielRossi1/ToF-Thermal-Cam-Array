import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QSpinBox)

from widgets.sensor_image_view import SensorImageView
from config.defaults import COLORMAPS
from network.protocol import TofFrame
from config.defaults import (
    TOF_MODES, TOF_TPZ, TOF_ZONES
)

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
            img2d = np.flipud(img2d)
        if self._flip_y:
            img2d = np.fliplr(img2d)
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
