"""Calibration UI page.

Provides:
- dToF calibration capture (records ToF + metadata to a chosen directory)
- Camera calibration capture with ArUco detection overlay (flat wall)

Designed to plug into the existing MainWindow via a tab.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QFileDialog,
    QFormLayout,
    QSpinBox,
    QComboBox,
    QLineEdit,
    QCheckBox,
)


@dataclass
class CalibCapture:
    seq: int
    hub_ts_us: int
    cam_ts_us: int
    cam_w: int
    cam_h: int
    cam_jpeg: Optional[bytes]
    tof_distance_mm: Optional[np.ndarray]


class CalibrationPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self._save_dir: Optional[str] = None
        self._save_n: int = 0

        self._last_capture: Optional[CalibCapture] = None

        # Camera calib state
        self._aruco_enabled = True
        self._last_markers = "—"

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── Camera calibration group ─────────────────────────────────────────
        cam_box = QGroupBox("Camera Calibration (ArUco on flat wall)")
        cam_l = QVBoxLayout(cam_box)

        self._cam_label = QLabel("No camera frame")
        self._cam_label.setAlignment(Qt.AlignCenter)
        self._cam_label.setMinimumSize(640, 360)
        self._cam_label.setStyleSheet(
            "background: #0a0a14; color: #444; border-radius: 6px;"
        )
        cam_l.addWidget(self._cam_label, stretch=1)

        controls = QHBoxLayout()

        self._aruco_cb = QCheckBox("Detect ArUco")
        self._aruco_cb.setChecked(True)
        self._aruco_cb.toggled.connect(self._on_aruco_toggle)
        controls.addWidget(self._aruco_cb)

        controls.addWidget(QLabel("Dict:"))
        self._dict_cb = QComboBox()
        self._dict_cb.addItems([
            "DICT_4X4_50",
            "DICT_4X4_100",
            "DICT_5X5_50",
            "DICT_5X5_100",
            "DICT_6X6_50",
            "DICT_6X6_100",
            "DICT_ARUCO_ORIGINAL",
        ])
        controls.addWidget(self._dict_cb)

        controls.addWidget(QLabel("Expected IDs (comma):"))
        self._expected_ids = QLineEdit("")
        self._expected_ids.setPlaceholderText("e.g. 0,1,2,3")
        controls.addWidget(self._expected_ids, stretch=1)

        self._cap_cam_btn = QPushButton("Capture frame")
        self._cap_cam_btn.clicked.connect(self._capture_now)
        controls.addWidget(self._cap_cam_btn)

        cam_l.addLayout(controls)

        self._cam_stats = QLabel("—")
        self._cam_stats.setStyleSheet("color: #a5d6a7; font-family: monospace;")
        self._cam_stats.setAlignment(Qt.AlignCenter)
        cam_l.addWidget(self._cam_stats)

        # ── dToF calibration group ──────────────────────────────────────────
        tof_box = QGroupBox("dToF Calibration")
        tof_form = QFormLayout(tof_box)

        self._known_mm = QSpinBox()
        self._known_mm.setRange(10, 10000)
        self._known_mm.setValue(500)
        tof_form.addRow("Known distance (mm)", self._known_mm)

        self._cap_tof_btn = QPushButton("Capture ToF+Cam snapshot")
        self._cap_tof_btn.clicked.connect(self._capture_now)
        tof_form.addRow(self._cap_tof_btn)

        # ── Save directory ──────────────────────────────────────────────────
        save_box = QGroupBox("Capture Output")
        save_l = QHBoxLayout(save_box)
        self._dir_lbl = QLabel("No directory selected")
        self._dir_lbl.setStyleSheet("color:#888;")
        choose_btn = QPushButton("Choose directory …")
        choose_btn.clicked.connect(self._choose_dir)
        save_l.addWidget(self._dir_lbl, stretch=1)
        save_l.addWidget(choose_btn)

        root.addWidget(cam_box, stretch=3)
        root.addWidget(tof_box, stretch=0)
        root.addWidget(save_box, stretch=0)

        # Lazy import (keeps app usable if OpenCV missing)
        self._cv2 = None
        self._aruco = None
        self._ensure_cv()

    # ── Public API (called by MainWindow) ───────────────────────────────────

    def update_from_synced_frame(self, sf) -> None:
        """Update page with the latest SyncedFrame (from protocol.py)."""
        cam_jpeg = getattr(sf, "cam_jpeg", None)
        cam_w = getattr(sf, "cam_w", 0)
        cam_h = getattr(sf, "cam_h", 0)
        cam_ts_us = getattr(sf, "cam_ts_us", 0)

        tof = getattr(sf, "tof", None)
        tof_dist = None
        if tof is not None and hasattr(tof, "distance_mm"):
            tof_dist = tof.distance_mm.copy()

        self._last_capture = CalibCapture(
            seq=int(getattr(sf, "seq", 0)),
            hub_ts_us=int(getattr(sf, "hub_ts_us", 0)),
            cam_ts_us=int(cam_ts_us),
            cam_w=int(cam_w),
            cam_h=int(cam_h),
            cam_jpeg=cam_jpeg,
            tof_distance_mm=tof_dist,
        )

        if cam_jpeg:
            self._render_camera(cam_jpeg, cam_w, cam_h, cam_ts_us)

    # ── Internals ───────────────────────────────────────────────────────────

    def _choose_dir(self):
        d = QFileDialog.getExistingDirectory(self, "Select calibration capture directory")
        if d:
            self._save_dir = d
            self._save_n = 0
            self._dir_lbl.setText(d)

    def _on_aruco_toggle(self, on: bool):
        self._aruco_enabled = bool(on)

    def _ensure_cv(self):
        if self._cv2 is not None:
            return
        try:
            import cv2  # type: ignore
            self._cv2 = cv2
            self._aruco = cv2.aruco
        except Exception:
            self._cv2 = None
            self._aruco = None
            self._cam_stats.setText("OpenCV ArUco not available (install requirements)")

    def _aruco_dict(self):
        if not self._aruco:
            return None
        name = self._dict_cb.currentText().strip()
        try:
            return getattr(self._aruco, name)
        except Exception:
            return getattr(self._aruco, "DICT_4X4_50")

    def _parse_expected_ids(self):
        raw = self._expected_ids.text().strip()
        if not raw:
            return None
        out = set()
        for part in raw.split(","):
            part = part.strip()
            if not part:
                continue
            try:
                out.add(int(part))
            except ValueError:
                continue
        return out if out else None

    def _render_camera(self, jpeg: bytes, w: int, h: int, ts_us: int):
        # If ArUco is off or OpenCV missing, just display JPEG.
        if not self._aruco_enabled or not self._cv2 or not self._aruco:
            img = QImage.fromData(jpeg, "JPEG")
            if img.isNull():
                return
            pix = QPixmap.fromImage(img).scaled(
                self._cam_label.width(), self._cam_label.height(),
                Qt.KeepAspectRatio, Qt.SmoothTransformation,
            )
            self._cam_label.setPixmap(pix)
            self._cam_stats.setText(f"{w}×{h} ts={ts_us//1000} ms  (Aruco off)")
            return

        # Decode JPEG with OpenCV and run ArUco detection.
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = self._cv2.imdecode(arr, self._cv2.IMREAD_COLOR)
        if bgr is None:
            return

        gray = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2GRAY)

        dict_id = self._aruco_dict()
        dictionary = self._aruco.getPredefinedDictionary(dict_id)
        params = self._aruco.DetectorParameters()
        detector = self._aruco.ArucoDetector(dictionary, params)

        corners, ids, _rej = detector.detectMarkers(gray)

        expected = self._parse_expected_ids()
        n = 0
        ids_list = []
        if ids is not None and len(ids) > 0:
            ids_flat = [int(x) for x in ids.flatten().tolist()]
            for i, mid in enumerate(ids_flat):
                if expected is not None and mid not in expected:
                    continue
                ids_list.append(mid)
                n += 1
            self._aruco.drawDetectedMarkers(bgr, corners, ids)

        self._last_markers = ",".join(str(i) for i in sorted(ids_list)) if ids_list else "—"

        rgb = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)
        h2, w2, _ = rgb.shape
        qimg = QImage(rgb.data, w2, h2, 3 * w2, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self._cam_label.width(), self._cam_label.height(),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self._cam_label.setPixmap(pix)
        self._cam_stats.setText(
            f"{w}×{h} ts={ts_us//1000} ms  markers={n} ids={self._last_markers}"
        )

    def _capture_now(self):
        if not self._save_dir:
            self._choose_dir()
            if not self._save_dir:
                return

        if not self._last_capture:
            return

        cap = self._last_capture
        base = os.path.join(self._save_dir, f"calib_{self._save_n:06d}")
        self._save_n += 1

        # Save camera jpeg + ToF + metadata.
        if cap.cam_jpeg:
            with open(base + "_cam.jpg", "wb") as f:
                f.write(cap.cam_jpeg)

        if cap.tof_distance_mm is not None:
            np.save(base + "_tof_dist_mm.npy", cap.tof_distance_mm)

        meta = {
            "seq": cap.seq,
            "hub_ts_us": cap.hub_ts_us,
            "cam_ts_us": cap.cam_ts_us,
            "cam_w": cap.cam_w,
            "cam_h": cap.cam_h,
            "known_mm": int(self._known_mm.value()),
            "aruco_enabled": bool(self._aruco_enabled),
            "aruco_dict": self._dict_cb.currentText(),
            "aruco_expected_ids": self._expected_ids.text().strip(),
            "aruco_seen_ids": self._last_markers,
            "saved_unix_s": time.time(),
        }
        with open(base + "_meta.txt", "w", encoding="utf-8") as f:
            for k, v in meta.items():
                f.write(f"{k}={v}\n")
