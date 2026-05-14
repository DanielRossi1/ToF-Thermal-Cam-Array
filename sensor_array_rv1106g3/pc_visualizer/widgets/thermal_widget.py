import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QSpinBox, QCheckBox)

from widgets.sensor_image_view import SensorImageView
from config.defaults import COLORMAPS
from network.protocol import MlxFrame
from config.defaults import (
    MLX_W, MLX_H,
)

class ThermalWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__('Thermal  MLX90640  24×32', parent)
        self._rot_k  = 0
        self._flip_x = False
        self._flip_y = False
        self._last_mlx = None
        self._setup_ui()

    def set_transform(self, rot_deg: int = 0, flip_x: bool = False, flip_y: bool = False):
        try:
            self._rot_k = (int(rot_deg) // 90) % 4
        except Exception:
            self._rot_k = 0
        self._flip_x = bool(flip_x)
        self._flip_y = bool(flip_y)
        if self._last_mlx is not None:
            self.update_frame(self._last_mlx)

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
        self._last_mlx = mlx
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
