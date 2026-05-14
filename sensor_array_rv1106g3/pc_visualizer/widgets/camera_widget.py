import numpy as np
from PyQt5.QtCore import Qt, QByteArray
from PyQt5.QtGui import QImage, QPixmap, QTransform
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QLabel

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
            # ← was rotate(self._rot_deg); negated to match np.rot90 CCW convention
            img = img.transformed(QTransform().rotate(-self._rot_deg))
        if self._flip_x or self._flip_y:
            img = img.mirrored(self._flip_x, self._flip_y)

        pix = QPixmap.fromImage(img).scaled(
            self._label.width(), self._label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._label.setPixmap(pix)
        self._stats.setText(
            f'{w}×{h}  {len(jpeg)//1024} KiB ({len(jpeg)} B)  '
            f'ts={ts_us//1000} ms')

