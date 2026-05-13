from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLineEdit, QSpinBox,
    QComboBox, QCheckBox, QPushButton, QHBoxLayout, QLabel,
)
from PyQt5.QtCore import pyqtSignal


class ConfigPage(QWidget):
    applied = pyqtSignal(dict)
    saved = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        root = QVBoxLayout(self)

        # Network
        net = QGroupBox('Network')
        netl = QFormLayout(net)

        self._proto = QComboBox()
        self._proto.addItems(['TCP', 'UDP'])

        self._host = QLineEdit()

        self._port = QSpinBox()
        self._port.setRange(1, 65535)

        self._auto_reconnect = QCheckBox('Auto-reconnect')

        netl.addRow('Protocol:', self._proto)
        netl.addRow('Host:', self._host)
        netl.addRow('Port:', self._port)
        netl.addRow('', self._auto_reconnect)
        root.addWidget(net)

        # Views
        views = QGroupBox('View Transforms')
        vsl = QVBoxLayout(views)
        vsl.addWidget(self._make_transform_group('ToF', 'tof'))
        vsl.addWidget(self._make_transform_group('MLX', 'mlx'))
        vsl.addWidget(self._make_transform_group('Camera', 'cam'))
        root.addWidget(views)

        # Buttons
        row = QHBoxLayout()
        row.addStretch(1)
        self._apply_btn = QPushButton('Apply')
        self._save_btn = QPushButton('Save')
        row.addWidget(self._apply_btn)
        row.addWidget(self._save_btn)
        root.addLayout(row)

        self._status = QLabel('')
        self._status.setStyleSheet('color:#888; font-size:10px;')
        root.addWidget(self._status)

        self._apply_btn.clicked.connect(lambda: self.applied.emit(self.get_config()))
        self._save_btn.clicked.connect(lambda: self.saved.emit(self.get_config()))

    def _make_transform_group(self, title: str, prefix: str) -> QGroupBox:
        g = QGroupBox(title)
        fl = QFormLayout(g)

        rot = QComboBox()
        rot.addItems(['0', '90', '180', '270'])
        flip_x = QCheckBox('Flip X')
        flip_y = QCheckBox('Flip Y')

        setattr(self, f'_{prefix}_rot', rot)
        setattr(self, f'_{prefix}_flip_x', flip_x)
        setattr(self, f'_{prefix}_flip_y', flip_y)

        fl.addRow('Rotation (deg):', rot)
        fl.addRow('', flip_x)
        fl.addRow('', flip_y)
        return g

    def set_status(self, text: str):
        self._status.setText(text)

    def set_config(self, cfg: dict):
        self._proto.setCurrentText(str(cfg.get('proto', 'TCP')).upper())
        self._host.setText(str(cfg.get('host', '')))
        self._port.setValue(int(cfg.get('port', 9000)))
        self._auto_reconnect.setChecked(bool(cfg.get('auto_reconnect', False)))

        for p in ('tof', 'mlx', 'cam'):
            getattr(self, f'_{p}_rot').setCurrentText(str(int(cfg.get(f'{p}_rot', 0))))
            getattr(self, f'_{p}_flip_x').setChecked(bool(cfg.get(f'{p}_flip_x', False)))
            getattr(self, f'_{p}_flip_y').setChecked(bool(cfg.get(f'{p}_flip_y', False)))

    def get_config(self) -> dict:
        def _rot(prefix: str) -> int:
            try:
                return int(getattr(self, f'_{prefix}_rot').currentText())
            except Exception:
                return 0

        cfg = {
            'proto': self._proto.currentText().upper(),
            'host': self._host.text().strip(),
            'port': int(self._port.value()),
            'auto_reconnect': bool(self._auto_reconnect.isChecked()),
        }

        for p in ('tof', 'mlx', 'cam'):
            cfg[f'{p}_rot'] = _rot(p)
            cfg[f'{p}_flip_x'] = bool(getattr(self, f'_{p}_flip_x').isChecked())
            cfg[f'{p}_flip_y'] = bool(getattr(self, f'_{p}_flip_y').isChecked())

        return cfg
