import os
import json
import numpy as np
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLineEdit, QSpinBox,
    QComboBox, QCheckBox, QPushButton, QHBoxLayout, QLabel, QFileDialog,
    QGridLayout, QDoubleSpinBox, QSizePolicy, QFrame,
)
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont


# ── Palette (match Calibration tab) ───────────────────────────────────────────

_BG       = "#0a0a14"
_PANEL    = "#10101e"
_BORDER   = "#1e2244"
_ACCENT   = "#00e5ff"
_GOOD     = "#4caf50"
_WARN     = "#ff9800"
_BAD      = "#f44336"
_TEXT     = "#dde3f0"
_MUTED    = "#5a6080"
_MONO     = "Consolas, 'Courier New', monospace"

_BASE_STYLE = f"""
QWidget {{
    background: {_BG};
    color: {_TEXT};
    font-family: 'Segoe UI', sans-serif;
    font-size: 12px;
}}
QGroupBox {{
    background: {_PANEL};
    border: 1px solid {_BORDER};
    border-radius: 6px;
    margin-top: 14px;
    padding-top: 6px;
    font-size: 11px;
    font-weight: bold;
    color: {_MUTED};
    letter-spacing: 1px;
    text-transform: uppercase;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}}
QPushButton {{
    background: #1a1a30;
    border: 1px solid {_BORDER};
    border-radius: 5px;
    padding: 6px 16px;
    color: {_TEXT};
    font-size: 12px;
}}
QPushButton:hover {{ background: #22224a; border-color: {_ACCENT}; color: {_ACCENT}; }}
QPushButton:pressed {{ background: #0d0d20; }}
QPushButton:disabled {{ color: {_MUTED}; border-color: #1a1a30; }}
QSpinBox, QDoubleSpinBox, QLineEdit, QComboBox {{
    background: #0d0d1a;
    border: 1px solid {_BORDER};
    border-radius: 4px;
    padding: 3px 6px;
    color: {_TEXT};
}}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus, QComboBox:focus {{
    border-color: {_ACCENT};
}}
QLabel {{ color: {_TEXT}; }}
QCheckBox {{ color: {_TEXT}; spacing: 6px; }}
QCheckBox::indicator {{
    width: 14px; height: 14px;
    border: 1px solid {_BORDER};
    border-radius: 3px;
    background: #0d0d1a;
}}
QCheckBox::indicator:checked {{ background: {_ACCENT}; border-color: {_ACCENT}; }}
"""


# ── Helpers ────────────────────────────────────────────────────────────────────

def _float_edit(value: float = 0.0, tip: str = "") -> QLineEdit:
    """Single float line-edit with a monospace font."""
    w = QLineEdit(f"{value:.6g}")
    w.setFont(QFont("Consolas", 9))
    w.setAlignment(Qt.AlignRight)
    if tip:
        w.setToolTip(tip)
    return w


def _parse(edit: QLineEdit, fallback: float = 0.0) -> float:
    try:
        return float(edit.text())
    except ValueError:
        return fallback


def _section_label(text: str) -> QLabel:
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color:{_MUTED}; font-size:10px; font-weight:bold; "
        "letter-spacing:1px; padding-top:6px;"
    )
    return lbl


# ── Matrix widget (3×3 grid of QLineEdits) ─────────────────────────────────────

class MatrixEdit(QWidget):
    """Compact 3-row × 3-col grid of float line-edits."""

    def __init__(self, rows: int = 3, cols: int = 3, parent=None):
        super().__init__(parent)
        self._rows = rows
        self._cols = cols
        g = QGridLayout(self)
        g.setSpacing(3)
        g.setContentsMargins(0, 0, 0, 0)
        self._edits: list[list[QLineEdit]] = []
        for r in range(rows):
            row_edits = []
            for c in range(cols):
                e = _float_edit(1.0 if r == c else 0.0)
                e.setFixedWidth(72)
                g.addWidget(e, r, c)
                row_edits.append(e)
            self._edits.append(row_edits)

    def set_matrix(self, arr: np.ndarray):
        arr = np.asarray(arr, dtype=float)
        for r in range(self._rows):
            for c in range(self._cols):
                self._edits[r][c].setText(f"{arr[r, c]:.6g}")

    def get_matrix(self) -> np.ndarray:
        out = np.zeros((self._rows, self._cols))
        for r in range(self._rows):
            for c in range(self._cols):
                out[r, c] = _parse(self._edits[r][c])
        return out


# ── Row of N float edits ────────────────────────────────────────────────────────

class VectorEdit(QWidget):
    """Horizontal row of N float line-edits."""

    def __init__(self, n: int, labels: list[str] | None = None, parent=None):
        super().__init__(parent)
        self._n = n
        h = QHBoxLayout(self)
        h.setSpacing(3)
        h.setContentsMargins(0, 0, 0, 0)
        self._edits: list[QLineEdit] = []
        for i in range(n):
            if labels:
                h.addWidget(QLabel(labels[i]))
            e = _float_edit()
            e.setFixedWidth(72)
            h.addWidget(e)
            self._edits.append(e)
        h.addStretch()

    def set_vector(self, arr):
        arr = np.asarray(arr, dtype=float).flatten()
        for i, e in enumerate(self._edits):
            e.setText(f"{arr[i]:.6g}" if i < len(arr) else "0")

    def get_vector(self) -> np.ndarray:
        return np.array([_parse(e) for e in self._edits])


# ── Main ConfigPage ─────────────────────────────────────────────────────────────

class ConfigPage(QWidget):
    applied = pyqtSignal(dict)
    saved   = pyqtSignal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(_BASE_STYLE)
        root = QVBoxLayout(self)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # ── Network ───────────────────────────────────────────────────────────
        net  = QGroupBox("Network")
        netl = QFormLayout(net)
        netl.setLabelAlignment(Qt.AlignRight)

        self._proto = QComboBox()
        self._proto.addItems(["TCP", "UDP"])

        self._host = QLineEdit()

        self._port = QSpinBox()
        self._port.setRange(1, 65535)

        self._auto_reconnect = QCheckBox("Auto-reconnect")

        netl.addRow("Protocol:", self._proto)
        netl.addRow("Host:",     self._host)
        netl.addRow("Port:",     self._port)
        netl.addRow("",          self._auto_reconnect)
        root.addWidget(net)

        # ── Calibration ───────────────────────────────────────────────────────
        calib   = QGroupBox("Calibration — Intrinsics & Extrinsics")
        calib_l = QVBoxLayout(calib)
        calib_l.setSpacing(4)

        # Load from file
        load_row = QHBoxLayout()
        self._load_btn = QPushButton("Load .npz …")
        self._load_btn.clicked.connect(self._load_npz)
        self._load_btn.setFixedWidth(110)
        self._calib_info = QLabel("No file loaded — edit fields below or load .npz")
        self._calib_info.setStyleSheet(f"color:{_MUTED}; font-size:10px;")
        load_row.addWidget(self._load_btn)
        load_row.addWidget(self._calib_info, stretch=1)
        calib_l.addLayout(load_row)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background:{_BORDER}; max-height:1px;")
        calib_l.addWidget(sep)

        # ── Intrinsics ──
        calib_l.addWidget(_section_label("CAMERA INTRINSICS  (pixel units)"))

        fx_cy_row = QHBoxLayout()
        fx_cy_row.setSpacing(12)
        for lbl_text, attr in [("fx", "_fx"), ("fy", "_fy"), ("cx", "_cx"), ("cy", "_cy")]:
            col = QVBoxLayout()
            col.setSpacing(2)
            col.addWidget(QLabel(lbl_text))
            edit = _float_edit(tip=f"Focal length / principal point — {lbl_text}")
            edit.setFixedWidth(90)
            setattr(self, attr, edit)
            col.addWidget(edit)
            fx_cy_row.addLayout(col)
        fx_cy_row.addStretch()
        calib_l.addLayout(fx_cy_row)

        # Camera model
        cm_row = QHBoxLayout()
        cm_row.setSpacing(8)
        cm_row.addWidget(QLabel("Camera model"))
        self._cam_model = QComboBox()
        self._cam_model.addItems(["Pinhole", "Fisheye"])
        self._cam_model.setCurrentText("Pinhole")
        cm_row.addWidget(self._cam_model)
        cm_row.addStretch()
        calib_l.addLayout(cm_row)

        # Distortion
        self._dist_lbl = _section_label("DISTORTION  (k1  k2  p1  p2  k3)")
        calib_l.addWidget(self._dist_lbl)
        self._dist_pinhole = VectorEdit(5, ["k1", "k2", "p1", "p2", "k3"])
        self._dist_fisheye = VectorEdit(4, ["k1", "k2", "k3", "k4"])
        calib_l.addWidget(self._dist_pinhole)
        calib_l.addWidget(self._dist_fisheye)
        self._dist_fisheye.setVisible(False)

        def _on_cam_model_changed():
            is_fish = self._cam_model.currentText().startswith("Fish")
            self._dist_pinhole.setVisible(not is_fish)
            self._dist_fisheye.setVisible(is_fish)
            self._dist_lbl.setText(
                "DISTORTION  (k1  k2  k3  k4)" if is_fish else "DISTORTION  (k1  k2  p1  p2  k3)"
            )

        self._cam_model.currentIndexChanged.connect(_on_cam_model_changed)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(f"background:{_BORDER}; max-height:1px;")
        calib_l.addWidget(sep2)

        # ── Extrinsics ──
        calib_l.addWidget(_section_label("EXTRINSICS  ToF → RGB"))

        calib_l.addWidget(QLabel("R  (3×3 rotation):"))
        self._R_edit = MatrixEdit(3, 3)
        calib_l.addWidget(self._R_edit)

        calib_l.addWidget(QLabel("t  (translation, metres)  [tx  ty  tz]:"))
        self._t_edit = VectorEdit(3, ["tx", "ty", "tz"])
        calib_l.addWidget(self._t_edit)

        # ToF FoV
        fov_row = QHBoxLayout()
        fov_row.addWidget(QLabel("ToF FoV (°):"))
        self._tof_fov = QDoubleSpinBox()
        self._tof_fov.setRange(1.0, 180.0)
        self._tof_fov.setValue(45.0)
        self._tof_fov.setSingleStep(0.5)
        self._tof_fov.setDecimals(1)
        self._tof_fov.setFixedWidth(80)
        self._tof_fov.setToolTip("Full field-of-view of the VL53L8CH sensor (degrees)")
        fov_row.addWidget(self._tof_fov)
        fov_row.addStretch()
        calib_l.addLayout(fov_row)

        # Validation label
        self._calib_valid = QLabel("")
        self._calib_valid.setStyleSheet("font-size:10px;")
        calib_l.addWidget(self._calib_valid)

        root.addWidget(calib)

        # ── View transforms ───────────────────────────────────────────────────
        views = QGroupBox("View Transforms")
        vsl   = QVBoxLayout(views)
        vsl.addWidget(self._make_transform_group("ToF",    "tof"))
        vsl.addWidget(self._make_transform_group("MLX",    "mlx"))
        vsl.addWidget(self._make_transform_group("Camera", "cam"))
        root.addWidget(views)

        root.addStretch()

        # ── Buttons ───────────────────────────────────────────────────────────
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        self._apply_btn = QPushButton("Apply")
        self._save_btn  = QPushButton("Save")
        btn_row.addWidget(self._apply_btn)
        btn_row.addWidget(self._save_btn)
        root.addLayout(btn_row)

        self._status = QLabel("")
        self._status.setStyleSheet(f"color:{_MUTED}; font-size:10px;")
        root.addWidget(self._status)

        self._apply_btn.clicked.connect(lambda: self.applied.emit(self.get_config()))
        self._save_btn.clicked.connect( lambda: self.saved.emit(self.get_config()))

    # ── Transform group ────────────────────────────────────────────────────────

    def _make_transform_group(self, title: str, prefix: str) -> QGroupBox:
        g  = QGroupBox(title)
        fl = QFormLayout(g)

        rot    = QComboBox()
        rot.addItems(["0", "90", "180", "270"])
        flip_x = QCheckBox("Flip X")
        flip_y = QCheckBox("Flip Y")

        setattr(self, f"_{prefix}_rot",    rot)
        setattr(self, f"_{prefix}_flip_x", flip_x)
        setattr(self, f"_{prefix}_flip_y", flip_y)

        fl.addRow("Rotation (deg):", rot)
        fl.addRow("",                flip_x)
        fl.addRow("",                flip_y)
        return g

    # ── Load .npz ─────────────────────────────────────────────────────────────

    def _load_npz(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Calibration .npz", "", "NumPy Archives (*.npz)"
        )
        if not path:
            return
        try:
            data = np.load(path)
            self._populate_from_arrays(
                K    = data.get("camera_matrix"),
                dist = data.get("dist_coeffs"),
                R    = data.get("R_tof_to_rgb"),
                t    = data.get("t_tof_to_rgb"),
            )
            # Camera model: prefer explicit key, else infer from dist length.
            try:
                cm = data.get("camera_model")
                if cm is not None:
                    cm_val = int(np.asarray(cm).flatten()[0])
                    self._cam_model.setCurrentText("Fisheye" if cm_val == 1 else "Pinhole")
            except Exception:
                pass
            tof_fov = data.get("tof_fov_deg")
            if tof_fov is not None:
                try:
                    self._tof_fov.setValue(float(np.asarray(tof_fov).flatten()[0]))
                except Exception:
                    pass
            self._calib_info.setText(f"Loaded: {os.path.basename(path)}")
            self._calib_info.setStyleSheet(f"color:{_GOOD}; font-size:10px;")
            self._validate_calib()
        except Exception as e:
            self._calib_info.setText("Error loading file.")
            self._calib_info.setStyleSheet(f"color:{_BAD}; font-size:10px;")
            print(f"[ConfigPage] .npz load error: {e}")

    def _populate_from_arrays(
        self,
        K=None, dist=None, R=None, t=None,
    ):
        """Write numpy arrays into the manual edit fields."""
        if K is not None:
            K = np.asarray(K, dtype=float)
            self._fx.setText(f"{K[0, 0]:.6g}")
            self._fy.setText(f"{K[1, 1]:.6g}")
            self._cx.setText(f"{K[0, 2]:.6g}")
            self._cy.setText(f"{K[1, 2]:.6g}")
        if dist is not None:
            d = np.asarray(dist, dtype=float).flatten()
            # If it looks like a fisheye vector (4 coeffs), switch the UI.
            if d.size == 4:
                self._cam_model.setCurrentText("Fisheye")
                self._dist_fisheye.set_vector(d[:4])
            else:
                self._cam_model.setCurrentText("Pinhole")
                self._dist_pinhole.set_vector(d[:5])
        if R is not None:
            self._R_edit.set_matrix(np.asarray(R, dtype=float).reshape(3, 3))
        if t is not None:
            self._t_edit.set_vector(np.asarray(t, dtype=float).flatten()[:3])

    # ── Validation ────────────────────────────────────────────────────────────

    def _validate_calib(self):
        """Check R is orthogonal and t is plausible; show coloured status."""
        R = self._R_edit.get_matrix()
        t = self._t_edit.get_vector()
        det = np.linalg.det(R)
        err = np.linalg.norm(R.T @ R - np.eye(3))

        rot_ok = (abs(det - 1.0) < 0.01 and err < 0.01)
        msg = (
            f"✓ R is valid rotation  (det={det:.4f})"
            if rot_ok else
            f"⚠  R may not be a pure rotation  (det={det:.4f}, RᵀR err={err:.4f})"
        )

        # Heuristic: warn if |t| is very large.
        try:
            t_norm_mm = float(np.linalg.norm(np.asarray(t, dtype=float).reshape(-1)[:3])) * 1000.0
        except Exception:
            t_norm_mm = 0.0
        if t_norm_mm > 500.0:
            msg += f"\n⚠  |t| is large: {t_norm_mm:.0f} mm (check ToF flip/orientation and near-range bias)"

        self._calib_valid.setText(msg)
        self._calib_valid.setStyleSheet(f"color:{_GOOD if rot_ok and t_norm_mm <= 500.0 else _WARN}; font-size:10px;")

    # ── Public API ────────────────────────────────────────────────────────────

    def set_status(self, text: str):
        self._status.setText(text)

    def set_config(self, cfg: dict):
        self._proto.setCurrentText(str(cfg.get("proto", "TCP")).upper())
        self._host.setText(str(cfg.get("host", "")))
        self._port.setValue(int(cfg.get("port", 9000)))
        self._auto_reconnect.setChecked(bool(cfg.get("auto_reconnect", False)))
        self._tof_fov.setValue(float(cfg.get("tof_fov_deg", 45.0)))

        # Populate calibration fields if present in cfg
        K    = cfg.get("camera_matrix")
        dist = cfg.get("dist_coeffs")
        R    = cfg.get("R_tof_to_rgb")
        t    = cfg.get("t_tof_to_rgb")

        if K is not None:
            self._populate_from_arrays(
                K    = np.array(K),
                dist = np.array(dist) if dist is not None else None,
                R    = np.array(R)    if R    is not None else None,
                t    = np.array(t)    if t    is not None else None,
            )
            try:
                cm = str(cfg.get('camera_model', '')).strip().lower()
                if cm == 'fisheye':
                    self._cam_model.setCurrentText('Fisheye')
                elif cm:
                    self._cam_model.setCurrentText('Pinhole')
            except Exception:
                pass
            self._calib_info.setText("Calibration loaded from config.")
            self._calib_info.setStyleSheet(f"color:{_GOOD}; font-size:10px;")
            self._validate_calib()

        for p in ("tof", "mlx", "cam"):
            getattr(self, f"_{p}_rot").setCurrentText(str(int(cfg.get(f"{p}_rot", 0))))
            getattr(self, f"_{p}_flip_x").setChecked(bool(cfg.get(f"{p}_flip_x", False)))
            getattr(self, f"_{p}_flip_y").setChecked(bool(cfg.get(f"{p}_flip_y", False)))

    def get_config(self) -> dict:
        self._validate_calib()

        def _rot(prefix: str) -> int:
            try:
                return int(getattr(self, f"_{prefix}_rot").currentText())
            except Exception:
                return 0

        cfg: dict = {
            "proto":          self._proto.currentText().upper(),
            "host":           self._host.text().strip(),
            "port":           int(self._port.value()),
            "auto_reconnect": bool(self._auto_reconnect.isChecked()),
            "tof_fov_deg":    float(self._tof_fov.value()),
        }

        # ── Build camera matrix from individual fields ──
        fx = _parse(self._fx)
        fy = _parse(self._fy)
        cx = _parse(self._cx)
        cy = _parse(self._cy)

        # Only embed calibration if at least fx/fy are non-zero
        if fx != 0.0 or fy != 0.0:
            K = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=float)
            cfg["camera_matrix"] = K.tolist()

            is_fish = self._cam_model.currentText().startswith('Fish')
            cfg["camera_model"] = "fisheye" if is_fish else "pinhole"
            if is_fish:
                cfg["dist_coeffs"] = self._dist_fisheye.get_vector().tolist()
            else:
                cfg["dist_coeffs"] = self._dist_pinhole.get_vector().tolist()

            R = self._R_edit.get_matrix()
            t = self._t_edit.get_vector()

            # Only embed extrinsics if R looks non-identity (user set something)
            r_is_identity = np.allclose(R, np.eye(3), atol=1e-6)
            t_is_zero     = np.allclose(t, np.zeros(3), atol=1e-9)
            if not (r_is_identity and t_is_zero):
                cfg["R_tof_to_rgb"] = R.tolist()
                cfg["t_tof_to_rgb"] = t.tolist()

        for p in ("tof", "mlx", "cam"):
            cfg[f"{p}_rot"]    = _rot(p)
            cfg[f"{p}_flip_x"] = bool(getattr(self, f"_{p}_flip_x").isChecked())
            cfg[f"{p}_flip_y"] = bool(getattr(self, f"_{p}_flip_y").isChecked())

        return cfg