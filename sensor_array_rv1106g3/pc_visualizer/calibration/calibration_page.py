"""
calibration_page.py — Calibration UI tab.

Features
--------
• Live camera feed with ArUco detection overlay
• "Start Acquisition" → streams frames, auto-selects the best ones
• Configurable: quality threshold, max frames, target RMS
• Real-time metrics: per-frame quality bar, frames counter, live RMS
• Auto-stop when max-frames OR target-RMS is reached
• Results panel: camera matrix, distortion, extrinsic RGB↔ToF, save button
• Compatible with MainWindow.update_from_synced_frame(sf) API
"""

from __future__ import annotations

import os
from typing import Optional

import numpy as np

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QDoubleSpinBox,
    QComboBox,
    QVBoxLayout,
    QWidget,
)

from calibration.calibration import CalibrationSession, CalibResult


# ──────────────────────────────────────────────────────────────────────────────
# Colour palette
# ──────────────────────────────────────────────────────────────────────────────

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
QScrollBar:vertical {{
    background: {_BG};
    width: 6px;
}}
QScrollBar::handle:vertical {{
    background: {_BORDER};
    border-radius: 3px;
}}
QProgressBar {{
    background: #1a1a30;
    border: 1px solid {_BORDER};
    border-radius: 4px;
    height: 10px;
    text-align: center;
    color: transparent;
}}
QProgressBar::chunk {{ border-radius: 4px; }}
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


def _mono_label(text: str = "", align=Qt.AlignLeft) -> QLabel:
    lbl = QLabel(text)
    lbl.setFont(QFont("Consolas", 11))
    lbl.setAlignment(align)
    return lbl


def _section(title: str) -> QGroupBox:
    gb = QGroupBox(title)
    gb.setStyleSheet(f"QGroupBox {{ background: {_PANEL}; }}")
    return gb


# ──────────────────────────────────────────────────────────────────────────────
# Quality bar widget
# ──────────────────────────────────────────────────────────────────────────────

class QualityBar(QWidget):
    """Horizontal quality bar with colour interpolation and numeric label."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0.0
        self.setMinimumHeight(20)
        self.setMinimumWidth(120)

    def set_value(self, v: float):
        self._value = float(np.clip(v, 0.0, 1.0))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Track
        p.setBrush(QColor("#1a1a30"))
        p.setPen(QColor(_BORDER))
        p.drawRoundedRect(0, 0, w, h, 4, 4)

        # Fill — green/orange/red
        fill_w = int(w * self._value)
        if fill_w > 0:
            v = self._value
            if v >= 0.65:
                colour = QColor(_GOOD)
            elif v >= 0.35:
                colour = QColor(_WARN)
            else:
                colour = QColor(_BAD)
            p.setBrush(colour)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(0, 0, fill_w, h, 4, 4)

        # Text
        p.setPen(QColor(_TEXT))
        p.setFont(QFont("Consolas", 9))
        p.drawText(0, 0, w, h, Qt.AlignCenter, f"{self._value * 100:.0f}%")


# ──────────────────────────────────────────────────────────────────────────────
# Results panel
# ──────────────────────────────────────────────────────────────────────────────

class ResultsPanel(QWidget):
    save_requested = pyqtSignal(str)   # emits chosen directory

    def __init__(self, parent=None):
        super().__init__(parent)
        self._result: Optional[CalibResult] = None
        self._session: Optional[CalibrationSession] = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        gb = _section("Calibration Result")
        inner = QVBoxLayout(gb)

        # Mono text dump
        self._txt = QLabel("—")
        self._txt.setFont(QFont("Consolas", 10))
        self._txt.setTextFormat(Qt.RichText)
        self._txt.setWordWrap(True)
        self._txt.setStyleSheet(f"color:{_TEXT}; background:{_PANEL}; padding:8px;")
        self._txt.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        scroll = QScrollArea()
        scroll.setWidget(self._txt)
        scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(180)
        scroll.setStyleSheet(f"background:{_PANEL}; border:none;")
        inner.addWidget(scroll)

        # Save button
        btn_row = QHBoxLayout()
        self._save_btn = QPushButton("💾  Save calibration …")
        self._save_btn.setEnabled(False)
        self._save_btn.clicked.connect(self._on_save)
        btn_row.addStretch()
        btn_row.addWidget(self._save_btn)
        inner.addLayout(btn_row)

        lay.addWidget(gb)

    def show_result(self, result: CalibResult, session: CalibrationSession):
        self._result = result
        self._session = session
        self._save_btn.setEnabled(True)

        K = result.camera_matrix
        d = result.dist_coeffs.flatten()

        rms_col = _GOOD if result.rms_error < 1.0 else (_WARN if result.rms_error < 2.0 else _BAD)

        lines = [
            f"<b>Frames used</b>: {result.n_frames}",
            f"<b>RMS reprojection</b>: "
            f"<span style='color:{rms_col}'>{result.rms_error:.4f} px</span>",
            "",
            "<b>Camera matrix K</b>",
            f"  fx = {K[0,0]:.2f}  fy = {K[1,1]:.2f}",
            f"  cx = {K[0,2]:.2f}  cy = {K[1,2]:.2f}",
            "",
            "<b>Distortion (k1 k2 p1 p2 k3)</b>",
            "  " + "  ".join(f"{v:+.5f}" for v in d[:5]),
        ]

        if result.R_tof_to_rgb is not None:
            ext_col = _GOOD if result.extrinsic_rms_mm < 15 else _WARN
            lines += [
                "",
                f"<b>Extrinsic RGB↔ToF</b>  "
                f"(RMS fit: <span style='color:{ext_col}'>"
                f"{result.extrinsic_rms_mm:.1f} mm</span>)",
                "<b>R (ToF→RGB)</b>",
            ]
            for row in result.R_tof_to_rgb:
                lines.append("  " + "  ".join(f"{v:+.5f}" for v in row))
            lines.append("<b>t (ToF→RGB) [mm]</b>")
            lines.append("  " + "  ".join(f"{v*1000:+.1f}" for v in result.t_tof_to_rgb))
        else:
            lines += ["", "<span style='color:#5a6080'>Extrinsic not computed (no ToF pairs)</span>"]

        self._txt.setText("<br>".join(lines))

    def clear(self):
        self._result = None
        self._session = None
        self._save_btn.setEnabled(False)
        self._txt.setText("—")

    def _on_save(self):
        if not self._result or not self._session:
            return
        d = QFileDialog.getExistingDirectory(self, "Select output directory")
        if d:
            path = self._session.save_result(self._result, d)
            self.save_requested.emit(path)


# ──────────────────────────────────────────────────────────────────────────────
# Main calibration page
# ──────────────────────────────────────────────────────────────────────────────

class CalibrationPage(QWidget):
    """
    Drop-in replacement for the old CalibrationPage.
    Plug into MainWindow with:
        page.update_from_synced_frame(sf)
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(_BASE_STYLE)

        self._coverage_map = np.zeros((10, 10), dtype=bool)

        # Session (lazily created / re-created on Start)
        self._session: Optional[CalibrationSession] = None
        self._last_w = 0
        self._last_h = 0
        self._flash_remaining = 0   # countdown ticks for green flash

        self._build_ui()

        # Tick timer: drives flash overlay on accepted frames
        self._tick = QTimer(self)
        self._tick.setInterval(80)
        self._tick.timeout.connect(self._on_tick)
        self._tick.start()

    def update_coverage(self, corners):
        # Get the center of the detected marker
        c = corners[0].reshape(4, 2).mean(axis=0)
        w, h = self._last_w, self._last_h  # image width/height
        if w <= 0 or h <= 0:
            return
        
        # Map pixel to 0-9 grid
        ix = int(np.clip(c[0] / w * 10, 0, 9))
        iy = int(np.clip(c[1] / h * 10, 0, 9))
        self._coverage_map[iy, ix] = True
        
        # Calculate % covered
        percent = (self._coverage_map.sum() / 100.0) * 100
        self._lbl_status.setText(f"Screen Coverage: {percent:.0f}%")

    # ── UI construction ────────────────────────────────────────────────────

    def _build_ui(self):
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── LEFT: camera preview ─────────────────────────────────────────
        left = QVBoxLayout()

        self._cam_label = QLabel("No camera frame")
        self._cam_label.setAlignment(Qt.AlignCenter)
        self._cam_label.setMinimumSize(640, 360)
        self._cam_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._cam_label.setStyleSheet(
            f"background:{_BG}; color:{_MUTED}; border:1px solid {_BORDER}; border-radius:6px;"
        )
        left.addWidget(self._cam_label, stretch=1)

        # Quality bar under preview
        qbar_row = QHBoxLayout()
        qbar_row.addWidget(QLabel("Frame quality"))
        self._qbar = QualityBar()
        qbar_row.addWidget(self._qbar, stretch=1)
        self._qbar_lbl = _mono_label("—", Qt.AlignRight)
        self._qbar_lbl.setFixedWidth(60)
        qbar_row.addWidget(self._qbar_lbl)
        left.addLayout(qbar_row)

        root.addLayout(left, stretch=3)

        # ── RIGHT: controls + metrics + results ───────────────────────────
        right = QVBoxLayout()
        right.setSpacing(6)

        right.addWidget(self._build_aruco_box())
        right.addWidget(self._build_acquisition_box())
        right.addWidget(self._build_metrics_box())

        self._results = ResultsPanel()
        self._results.save_requested.connect(self._on_saved)
        right.addWidget(self._results, stretch=1)

        right.addStretch()
        root.addLayout(right, stretch=2)

    def _build_aruco_box(self) -> QGroupBox:
        gb = _section("ArUco Config")
        form = QFormLayout(gb)
        form.setLabelAlignment(Qt.AlignRight)

        self._pattern_cb = QComboBox()
        self._pattern_cb.addItems(["Single ArUco", "ChArUco Board"])
        self._pattern_cb.currentIndexChanged.connect(self._on_pattern_changed)
        form.addRow("Pattern", self._pattern_cb)

        self._dict_cb = QComboBox()
        self._dict_cb.addItems([
            "DICT_4X4_50", "DICT_4X4_100",
            "DICT_5X5_50", "DICT_5X5_100",
            "DICT_6X6_50", "DICT_6X6_100",
            "DICT_ARUCO_ORIGINAL",
        ])
        form.addRow("Dictionary", self._dict_cb)

        self._marker_sz = QDoubleSpinBox()
        self._marker_sz.setRange(0.01, 1.0)
        self._marker_sz.setDecimals(3)
        self._marker_sz.setSingleStep(0.01)
        self._marker_sz.setValue(0.18)
        self._marker_sz.setSuffix(" m")
        form.addRow("Marker size", self._marker_sz)

        # ChArUco parameters
        self._charuco_x = QSpinBox(); self._charuco_x.setRange(3, 40); self._charuco_x.setValue(7)
        self._charuco_y = QSpinBox(); self._charuco_y.setRange(3, 40); self._charuco_y.setValue(5)
        self._charuco_sq = QDoubleSpinBox(); self._charuco_sq.setRange(0.005, 0.20); self._charuco_sq.setDecimals(4); self._charuco_sq.setValue(0.030); self._charuco_sq.setSuffix(" m")
        self._charuco_mk = QDoubleSpinBox(); self._charuco_mk.setRange(0.003, 0.20); self._charuco_mk.setDecimals(4); self._charuco_mk.setValue(0.022); self._charuco_mk.setSuffix(" m")

        self._charuco_rows = []
        for label, widget in (
            ("ChArUco squares X", self._charuco_x),
            ("ChArUco squares Y", self._charuco_y),
            ("Square length", self._charuco_sq),
            ("Marker length", self._charuco_mk),
        ):
            lbl = QLabel(label)
            form.addRow(lbl, widget)
            self._charuco_rows.append((lbl, widget))

        self._tof_fov = QDoubleSpinBox()
        self._tof_fov.setRange(10.0, 120.0)
        self._tof_fov.setDecimals(1)
        self._tof_fov.setValue(45.0)
        self._tof_fov.setSuffix(" °")
        form.addRow("ToF FoV", self._tof_fov)

        self._on_pattern_changed()

        return gb

    def _build_acquisition_box(self) -> QGroupBox:
        gb = _section("Acquisition")
        lay = QVBoxLayout(gb)

        # Start / Stop button
        self._start_btn = QPushButton("▶  Start Acquisition")
        self._start_btn.setMinimumHeight(36)
        self._start_btn.setStyleSheet(
            f"QPushButton {{ background:#0a1f10; border:1px solid {_GOOD}; "
            f"color:{_GOOD}; font-weight:bold; border-radius:5px; }}"
            f"QPushButton:hover {{ background:#122a18; }}"
            f"QPushButton:pressed {{ background:#0d2212; }}"
        )
        self._start_btn.clicked.connect(self._on_start_stop)
        lay.addWidget(self._start_btn)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)

        # N: Total Frames to collect
        self._n_total = QSpinBox()
        self._n_total.setRange(10, 500)
        self._n_total.setValue(60)
        form.addRow("Total Pool (N)", self._n_total)

        # k: Best frames to keep
        self._top_k = QSpinBox()
        self._top_k.setRange(5, 100)
        self._top_k.setValue(20)
        form.addRow("Top-k Selection", self._top_k)

        self._quality_thr = QDoubleSpinBox()
        self._quality_thr.setRange(0.0, 1.0)
        self._quality_thr.setDecimals(2)
        self._quality_thr.setSingleStep(0.05)
        self._quality_thr.setValue(0.45)
        form.addRow("Quality threshold", self._quality_thr)

        self._target_rms = QDoubleSpinBox()
        self._target_rms.setRange(0.1, 10.0)
        self._target_rms.setDecimals(2)
        self._target_rms.setSingleStep(0.1)
        self._target_rms.setValue(0.5)
        self._target_rms.setSuffix(" px")
        form.addRow("Target RMS", self._target_rms)

        self._recalib_every = QSpinBox()
        self._recalib_every.setRange(1, 20)
        self._recalib_every.setValue(3)
        form.addRow("Recalib every N", self._recalib_every)

        lay.addLayout(form)
        return gb

    def _build_metrics_box(self) -> QGroupBox:
        gb = _section("Live Metrics")
        lay = QVBoxLayout(gb)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setVerticalSpacing(4)

        self._lbl_frames = _mono_label("0 / 30")
        self._lbl_rms    = _mono_label("—")
        self._lbl_status = _mono_label("idle")
        self._lbl_status.setStyleSheet(f"color:{_MUTED};")

        form.addRow("Frames captured", self._lbl_frames)
        form.addRow("RMS reprojection", self._lbl_rms)
        form.addRow("Status", self._lbl_status)
        lay.addLayout(form)

        # Progress bar (frames / max_frames)
        self._prog = QProgressBar()
        self._prog.setRange(0, self._n_total.value())
        self._prog.setValue(0)
        self._prog.setStyleSheet(
            f"QProgressBar::chunk {{ background: {_ACCENT}; }}"
        )
        lay.addWidget(self._prog)

        return gb

    # ── Slot: start / stop ─────────────────────────────────────────────────

    def _on_start_stop(self):
        if self._session and self._session.is_running:
            self._finish_session("manual stop")
        else:
            self._start_session()

    def _start_session(self):
        try:
            import cv2
            dict_name = self._dict_cb.currentText().strip()
            dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50)
        except Exception:
            dict_id = 0  # DICT_4X4_50

        pattern = "charuco" if self._pattern_cb.currentText().startswith("Ch") else "aruco"
        self._session = CalibrationSession(
            aruco_dict_id=dict_id,
            pattern=pattern,
            marker_length_m=self._marker_sz.value(),
            charuco_squares_x=self._charuco_x.value(),
            charuco_squares_y=self._charuco_y.value(),
            charuco_square_length_m=self._charuco_sq.value(),
            charuco_marker_length_m=self._charuco_mk.value(),
            n_total=self._n_total.value(),
            top_k=self._top_k.value(),
            quality_threshold=self._quality_thr.value(),
            target_rms=self._target_rms.value(),
            tof_fov_deg=self._tof_fov.value(),
            recalib_every=self._recalib_every.value(),
        )
        self._session.start()

        self._results.clear()
        self._prog.setRange(0, self._n_total.value())
        self._prog.setValue(0)
        self._prog.setStyleSheet(
            f"QProgressBar::chunk {{ background: {_ACCENT}; }}"
        )
        self._lbl_frames.setText(f"0 / {self._n_total.value()}")
        self._lbl_rms.setText("—")
        self._set_status("acquiring …", _ACCENT)

        self._start_btn.setText("■  Stop Acquisition")
        self._start_btn.setStyleSheet(
            f"QPushButton {{ background:#1f0a0a; border:1px solid {_BAD}; "
            f"color:{_BAD}; font-weight:bold; border-radius:5px; }}"
            f"QPushButton:hover {{ background:#2a1212; }}"
        )
        self._set_controls_enabled(False)

    def _finish_session(self, reason: str = ""):
        if not self._session:
            return

        self._session.stop()
        result = self._session.finalize()

        self._start_btn.setText("▶  Start Acquisition")
        self._start_btn.setStyleSheet(
            f"QPushButton {{ background:#0a1f10; border:1px solid {_GOOD}; "
            f"color:{_GOOD}; font-weight:bold; border-radius:5px; }}"
            f"QPushButton:hover {{ background:#122a18; }}"
        )
        self._set_controls_enabled(True)

        if result:
            col = _GOOD if result.rms_error < 1.0 else (_WARN if result.rms_error < 2.0 else _BAD)
            self._set_rms(result.rms_error)
            self._set_status(f"done — {reason}", col)
            self._prog.setStyleSheet(
                f"QProgressBar::chunk {{ background: {col}; }}"
            )
            self._results.show_result(result, self._session)
        else:
            self._set_status("not enough frames", _BAD)

    # ── Public API (called by MainWindow every frame) ──────────────────────

    def update_from_synced_frame(self, sf) -> None:
        """Called by MainWindow on each new SyncedFrame."""
        cam_jpeg = getattr(sf, "cam_jpeg", None)
        cam_w    = int(getattr(sf, "cam_w", 0))
        cam_h    = int(getattr(sf, "cam_h", 0))

        tof = getattr(sf, "tof", None)
        # Pass the full ToF frame so calibration can pick the best target using
        # status/sigma, not only distance_mm[:,0].
        tof_dist = tof

        if cam_w:
            self._last_w = cam_w
        if cam_h:
            self._last_h = cam_h

        if cam_jpeg:
            if self._session and self._session.is_running:
                self._process_acquisition(cam_jpeg, tof_dist, cam_w, cam_h)
            else:
                self._render_jpeg(cam_jpeg, cam_w, cam_h)

    # ── Acquisition processing ─────────────────────────────────────────────

    def _process_acquisition(
        self,
        jpeg: bytes,
        tof_dist: Optional[np.ndarray],
        w: int,
        h: int,
    ):
        info = self._session.process_frame(jpeg, tof_dist, w, h)

        # Update preview with annotated frame
        if info['annotated_bgr'] is not None:
            self._render_bgr(info['annotated_bgr'], flash=info['accepted'])

        # Quality bar
        q = info['quality']
        self._qbar.set_value(q)
        self._qbar_lbl.setText(f"{q * 100:.0f}%")

        # Metrics
        n    = info['n_frames']
        rms  = info['rms']
        maxf = self._n_total.value()

        self._lbl_frames.setText(f"{n} / {maxf}")
        self._prog.setValue(min(n, maxf))

        if rms is not None:
            self._set_rms(rms)

        # Auto-stop check
        if self._session.should_stop():
            self._finish_session(self._session.stop_reason())

    # ── Rendering helpers ──────────────────────────────────────────────────

    def _render_jpeg(self, jpeg: bytes, w: int, h: int):
        img = QImage.fromData(jpeg, "JPEG")
        if img.isNull():
            return
        pix = QPixmap.fromImage(img).scaled(
            self._cam_label.width(), self._cam_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._cam_label.setPixmap(pix)

    def _render_bgr(self, bgr: np.ndarray, flash: bool = False):
        try:
            import cv2
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except Exception:
            return

        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()

        if flash:
            self._flash_remaining = 4

        pix = QPixmap.fromImage(qimg).scaled(
            self._cam_label.width(), self._cam_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )

        if self._flash_remaining > 0:
            p = QPainter(pix)
            p.setOpacity(0.25 * (self._flash_remaining / 4))
            p.setBrush(QColor(_GOOD))
            p.setPen(Qt.NoPen)
            p.drawRect(0, 0, pix.width(), pix.height())
            p.end()

        self._cam_label.setPixmap(pix)

    # ── Misc helpers ───────────────────────────────────────────────────────

    def _set_rms(self, rms: float):
        col = _GOOD if rms < 1.0 else (_WARN if rms < 2.0 else _BAD)
        self._lbl_rms.setText(f"{rms:.3f} px")
        self._lbl_rms.setStyleSheet(f"color:{col};")

    def _set_status(self, msg: str, col: str = _TEXT):
        self._lbl_status.setText(msg)
        self._lbl_status.setStyleSheet(f"color:{col};")

    def _set_controls_enabled(self, enabled: bool):
        for w in (
            self._pattern_cb, self._dict_cb, self._marker_sz, self._tof_fov,
            self._charuco_x, self._charuco_y, self._charuco_sq, self._charuco_mk,
            self._quality_thr, self._n_total, self._top_k,
            self._target_rms, self._recalib_every,
        ):
            w.setEnabled(enabled)

    def _on_pattern_changed(self):
        is_charuco = self._pattern_cb.currentText().startswith("Ch")
        # Show/hide rows (QFormLayout keeps labels otherwise).
        self._marker_sz.setVisible(not is_charuco)
        for lbl, w in getattr(self, '_charuco_rows', []):
            lbl.setVisible(is_charuco)
            w.setVisible(is_charuco)

    def _on_saved(self, path: str):
        self._set_status(f"saved → {os.path.basename(path)}", _GOOD)

    def _on_tick(self):
        if self._flash_remaining > 0:
            self._flash_remaining -= 1