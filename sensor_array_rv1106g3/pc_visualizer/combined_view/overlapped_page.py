"""
overlap_page.py — Fused Camera + ToF view.
Projects ToF zones into the camera image using intrinsic/extrinsic calibration.

Notes
-----
- Matches `TofWidget` grid ordering: data is reshaped as `reshape(res,res).T`.
- Applies the same ToF rotation/flip settings (tof_rot/tof_flip_x/tof_flip_y)
  so zone ordering matches what the user sees in the ToF widget.
- Highlights the foreground object (box) using the depth step vs background.
"""

import time
import numpy as np
import cv2
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QGroupBox,
    QComboBox,
    QCheckBox,
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

from config.defaults import COLORMAPS


class OverlapPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Calibration state
        self._K = None
        self._D = None
        self._R = None
        self._t = None
        self._camera_model = None  # 'pinhole' | 'fisheye' | None

        self._undist_map1 = None
        self._undist_map2 = None
        self._undist_K_new = None
        self._undist_size = None

        self._last_tof_time_s = None
        self._tof_stale_ms = 500.0

        self._tof_fov_rad = np.radians(45.0)
        self._rays = None  # Precomputed unit rays (zones, 4, 3)
        self._last_res = None

        # Match camera widget view transforms (applied after drawing overlay)
        self._cam_rot_deg = 0
        self._cam_flip_x = False
        self._cam_flip_y = False

        # Match ToF widget view transforms
        self._tof_rot_k = 0
        self._tof_flip_x = False
        self._tof_flip_y = False

        # Target status codes treated as valid range
        self._valid_tof_status = {5, 6, 9, 10}

        self._build_ui()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        # Top controls
        ctrl_lay = QHBoxLayout()

        gb = QGroupBox("Overlay Settings")
        gb_lay = QHBoxLayout(gb)

        self._enable_overlay = QCheckBox("Show ToF Overlay")
        self._enable_overlay.setChecked(True)
        gb_lay.addWidget(self._enable_overlay)

        self._show_all_zones = QCheckBox("All zones")
        self._show_all_zones.setChecked(True)
        gb_lay.addWidget(self._show_all_zones)

        gb_lay.addWidget(QLabel("Colormap:"))
        self._cmap_cb = QComboBox()
        self._cmap_cb.addItems(COLORMAPS)
        self._cmap_cb.setCurrentText("turbo")
        gb_lay.addWidget(self._cmap_cb)

        self._calib_status = QLabel("Calibration: NOT LOADED")
        self._calib_status.setStyleSheet("color: #f44336; font-weight: bold;")
        gb_lay.addWidget(self._calib_status)

        self._tof_age_lbl = QLabel("ToF age: —")
        self._tof_age_lbl.setStyleSheet("color: #5a6080;")
        gb_lay.addWidget(self._tof_age_lbl)

        gb_lay.addStretch()
        ctrl_lay.addWidget(gb)
        root.addLayout(ctrl_lay)

        # Image Display
        self._cam_label = QLabel("Waiting for camera/ToF frame...")
        self._cam_label.setAlignment(Qt.AlignCenter)
        self._cam_label.setStyleSheet("background: #0a0a14; border: 1px solid #1e2244;")
        root.addWidget(self._cam_label, stretch=1)

    def set_calibration(self, cfg: dict):
        """Update intrinsics/extrinsics and view transforms from config dictionary."""
        try:
            # Camera view transforms (match CameraWidget)
            try:
                self._cam_rot_deg = int(cfg.get('cam_rot', 0)) % 360
            except Exception:
                self._cam_rot_deg = 0
            self._cam_flip_x = bool(cfg.get('cam_flip_x', False))
            self._cam_flip_y = bool(cfg.get('cam_flip_y', False))

            # ToF view transforms (match TofWidget)
            try:
                self._tof_rot_k = (int(cfg.get('tof_rot', 0)) // 90) % 4
            except Exception:
                self._tof_rot_k = 0
            self._tof_flip_x = bool(cfg.get('tof_flip_x', False))
            self._tof_flip_y = bool(cfg.get('tof_flip_y', False))

            if 'camera_matrix' in cfg and 'R_tof_to_rgb' in cfg:
                self._K = np.array(cfg['camera_matrix'], dtype=np.float64)
                self._D = np.array(cfg.get('dist_coeffs', np.zeros((1, 5))), dtype=np.float64)
                self._R = np.array(cfg['R_tof_to_rgb'], dtype=np.float64)
                self._t = np.array(cfg['t_tof_to_rgb'], dtype=np.float64).flatten()
                try:
                    cm = cfg.get('camera_model', None)
                    self._camera_model = (str(cm).strip().lower() if cm is not None else None)
                except Exception:
                    self._camera_model = None

                self._undist_map1 = None
                self._undist_map2 = None
                self._undist_K_new = None
                self._undist_size = None

                # Update FoV and invalidate cached rays so they are recomputed.
                fov_deg = float(cfg.get('tof_fov_deg', np.degrees(self._tof_fov_rad)))
                new_fov = np.radians(fov_deg)
                if new_fov != self._tof_fov_rad:
                    self._tof_fov_rad = new_fov
                    self._rays = None
                    self._last_res = None

                self._calib_status.setText("Calibration: LOADED")
                self._calib_status.setStyleSheet("color: #4caf50; font-weight: bold;")
            else:
                self._K = None
                self._calib_status.setText("Calibration: NOT LOADED")
                self._calib_status.setStyleSheet("color: #f44336; font-weight: bold;")
        except Exception as e:
            print(f"Error loading calibration into overlap page: {e}")
            self._K = None

    def _get_colormap(self):
        name = self._cmap_cb.currentText()
        if name == 'turbo':
            return cv2.COLORMAP_TURBO
        if name == 'plasma':
            return cv2.COLORMAP_PLASMA
        if name == 'inferno':
            return cv2.COLORMAP_INFERNO
        if name == 'viridis':
            return cv2.COLORMAP_VIRIDIS
        if name == 'magma':
            return cv2.COLORMAP_MAGMA
        if name == 'hot':
            return cv2.COLORMAP_HOT
        return cv2.COLORMAP_JET

    def _apply_view_transform(self, bgr: np.ndarray) -> np.ndarray:
        # CameraWidget uses QTransform.rotate(-deg) then mirrored(horizontal, vertical)
        if self._cam_rot_deg:
            if self._cam_rot_deg == 90:
                bgr = cv2.rotate(bgr, cv2.ROTATE_90_CLOCKWISE)
            elif self._cam_rot_deg == 180:
                bgr = cv2.rotate(bgr, cv2.ROTATE_180)
            elif self._cam_rot_deg == 270:
                bgr = cv2.rotate(bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self._cam_flip_x and self._cam_flip_y:
            bgr = cv2.flip(bgr, -1)
        elif self._cam_flip_x:
            bgr = cv2.flip(bgr, 1)
        elif self._cam_flip_y:
            bgr = cv2.flip(bgr, 0)
        return bgr

    def _apply_tof_grid_transform(self, grid: np.ndarray) -> np.ndarray:
        if self._tof_rot_k:
            grid = np.rot90(grid, self._tof_rot_k)
        if self._tof_flip_x:
            grid = np.flipud(grid)
        if self._tof_flip_y:
            grid = np.fliplr(grid)
        return grid

    def _get_undistort_maps(self, w: int, h: int):
        """Build or return cached undistortion maps for image size (w, h)."""
        if self._undist_map1 is not None and self._undist_size == (w, h):
            return self._undist_map1, self._undist_map2, self._undist_K_new

        model = (self._camera_model or "").lower()
        d_flat = np.asarray(self._D, dtype=np.float64).reshape(-1)
        is_fisheye = (model == "fisheye") or (model == "fish") or (d_flat.size == 4)

        if is_fisheye:
            D4 = d_flat[:4].reshape(4, 1)
            R = np.eye(3, dtype=np.float64)
            # balance=0.0 behaves like alpha=0.0 (crop to fill)
            K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self._K, D4, (w, h), R, balance=0.0
            )
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                self._K, D4, R, K_new, (w, h), cv2.CV_16SC2
            )
        else:
            # alpha=0.0 crops to a normal full-frame view; alpha=1.0 can shrink
            # valid content to a tiny region for extreme distortion.
            K_new, _ = cv2.getOptimalNewCameraMatrix(
                self._K, self._D, (w, h), alpha=0.0
            )
            map1, map2 = cv2.initUndistortRectifyMap(
                self._K, self._D, None, K_new, (w, h), cv2.CV_16SC2
            )
        self._undist_map1 = map1
        self._undist_map2 = map2
        self._undist_K_new = K_new
        self._undist_size = (w, h)
        return map1, map2, K_new

    def _precompute_rays(self, resolution: int):
        """Precompute unit-direction rays for each ToF zone corner.

        Assumes ToF distance is a slant/radial range along the zone ray, so
        P = ray_unit * range.
        """
        self._last_res = resolution
        apz = self._tof_fov_rad / resolution

        c_offset = (resolution - 1) / 2.0
        r_offset = (resolution - 1) / 2.0

        rays = []
        for r in range(resolution):
            for c in range(resolution):
                corners = [
                    (c - c_offset - 0.5, r - r_offset - 0.5),
                    (c - c_offset + 0.5, r - r_offset - 0.5),
                    (c - c_offset + 0.5, r - r_offset + 0.5),
                    (c - c_offset - 0.5, r - r_offset + 0.5),
                ]
                zone_rays = []
                for cx, cy in corners:
                    ah = cx * apz
                    av = cy * apz
                    tan_ah, tan_av = np.tan(ah), np.tan(av)
                    ray = np.array([tan_ah, tan_av, 1.0], dtype=np.float32)
                    ray /= np.linalg.norm(ray)
                    zone_rays.append(ray)
                rays.append(zone_rays)

        self._rays = np.array(rays, dtype=np.float32)

    def _largest_component(self, mask: np.ndarray, score: np.ndarray):
        """Pick the 4-connected component with the highest summed score."""
        if mask is None or mask.ndim != 2:
            return None
        h, w = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        best = None
        best_score = -1.0

        for r0 in range(h):
            for c0 in range(w):
                if not mask[r0, c0] or visited[r0, c0]:
                    continue
                stack = [(r0, c0)]
                visited[r0, c0] = True
                comp = []
                s = 0.0
                while stack:
                    r, c = stack.pop()
                    comp.append((r, c))
                    s += float(score[r, c])
                    for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        rr = r + dr
                        cc = c + dc
                        if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not visited[rr, cc]:
                            visited[rr, cc] = True
                            stack.append((rr, cc))
                if s > best_score:
                    best_score = s
                    best = comp

        if best is None:
            return None
        out = np.zeros_like(mask, dtype=bool)
        for r, c in best:
            out[r, c] = True
        return out

    def _select_primary_distance(self, tof) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
        """Return per-zone primary distance (and selected status/sigma if available)."""
        dist = np.asarray(tof.distance_mm)
        status = np.asarray(getattr(tof, 'status', None)) if hasattr(tof, 'status') else None
        sigma = np.asarray(getattr(tof, 'sigma_mm', None)) if hasattr(tof, 'sigma_mm') else None

        if dist.ndim == 1:
            dist_1d = dist.astype(np.float32).reshape(-1)
            sel_status = None
            if status is not None:
                if status.shape == dist.shape:
                    sel_status = status.astype(np.uint8).reshape(-1)
                elif status.size == dist_1d.size:
                    sel_status = status.reshape(-1).astype(np.uint8)
            sel_sigma = None
            if sigma is not None:
                if sigma.shape == dist.shape:
                    sel_sigma = sigma.astype(np.float32).reshape(-1)
                elif sigma.size == dist_1d.size:
                    sel_sigma = sigma.reshape(-1).astype(np.float32)
            return dist_1d, sel_status, sel_sigma

        # Multi-target per zone: (N_zones, N_targets)
        if dist.ndim == 2 and dist.shape[0] != dist.shape[1]:
            n_zones, _ = dist.shape
            valid = (dist > 0) & (dist < 8000)
            if status is not None and status.shape == dist.shape:
                valid &= np.isin(status, list(self._valid_tof_status))
            dist_masked = np.where(valid, dist.astype(np.float32), np.inf)
            idx = np.argmin(dist_masked, axis=1)
            sel = dist_masked[np.arange(n_zones), idx]
            sel[~np.isfinite(sel)] = 0

            sel_status = None
            if status is not None and status.shape == dist.shape:
                sel_status = status[np.arange(n_zones), idx].astype(np.uint8)
            sel_sigma = None
            if sigma is not None and sigma.shape == dist.shape:
                sel_sigma = sigma[np.arange(n_zones), idx].astype(np.float32)

            return sel.astype(np.float32), sel_status, sel_sigma

        return dist.reshape(-1).astype(np.float32), None, None

    def update_from_synced_frame(self, sf):
        if not sf.cam_jpeg:
            return

        now = time.monotonic()
        if sf.tof is not None:
            self._last_tof_time_s = now

        age_ms = None
        if self._last_tof_time_s is not None:
            age_ms = (now - self._last_tof_time_s) * 1000.0
            self._tof_age_lbl.setText(f"ToF age: {age_ms:.0f} ms")
            if age_ms > self._tof_stale_ms:
                self._tof_age_lbl.setStyleSheet("color: #ff9800;")
            else:
                self._tof_age_lbl.setStyleSheet("color: #90caf9;")
        else:
            self._tof_age_lbl.setText("ToF age: —")
            self._tof_age_lbl.setStyleSheet("color: #5a6080;")

        arr = np.frombuffer(sf.cam_jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return

        if self._K is not None and self._D is not None:
            h_img, w_img = bgr.shape[:2]
            map1, map2, K_new = self._get_undistort_maps(w_img, h_img)
            bgr = cv2.remap(bgr, map1, map2, cv2.INTER_LINEAR)
        else:
            K_new = None

        overlay_drawn = False
        if (
            self._enable_overlay.isChecked()
            and sf.tof is not None
            and self._K is not None
            and self._R is not None
            and self._t is not None
        ):
            dist_mm_1d, status_1d, sigma_1d = self._select_primary_distance(sf.tof)
            res = int(np.sqrt(len(dist_mm_1d)))
            if res * res != len(dist_mm_1d):
                bgr = self._apply_view_transform(bgr)
                self._show_bgr(bgr)
                return

            # Match TofWidget ordering + transforms
            dist_grid = dist_mm_1d.reshape(res, res).T
            dist_grid = self._apply_tof_grid_transform(dist_grid)

            status_grid = None
            if status_1d is not None and len(status_1d) == res * res:
                status_grid = self._apply_tof_grid_transform(status_1d.reshape(res, res).T)

            sigma_grid = None
            if sigma_1d is not None and len(sigma_1d) == res * res:
                sigma_grid = self._apply_tof_grid_transform(sigma_1d.reshape(res, res).T)

            dist_mm = dist_grid.reshape(-1)

            valid_mask = (dist_mm > 0) & (dist_mm < 8000)
            if status_grid is not None:
                valid_mask &= np.isin(status_grid.reshape(-1), list(self._valid_tof_status))

            if valid_mask.any():
                if self._rays is None or self._last_res != res:
                    self._precompute_rays(res)

                # Background vs foreground step detection
                dv = dist_mm[valid_mask]
                bg_mm = float(np.median(dv)) if dv.size else 0.0
                delta = (bg_mm - dist_grid).astype(np.float32)

                step_mm = 40.0
                if sigma_grid is not None:
                    sigv = sigma_grid[(dist_grid > 0) & (dist_grid < 8000)].astype(np.float32)
                    if sigv.size:
                        step_mm = float(max(25.0, 2.5 * np.median(np.clip(sigv, 1.0, 500.0))))

                fg0 = (dist_grid > 0) & (dist_grid < 8000) & (delta > step_mm)
                if status_grid is not None:
                    fg0 &= np.isin(status_grid, list(self._valid_tof_status))

                fg = self._largest_component(fg0, delta)
                if self._show_all_zones.isChecked():
                    draw_mask = valid_mask
                else:
                    draw_mask = fg.reshape(-1) if (fg is not None and fg.sum() >= 2) else valid_mask

                dist_draw = dist_mm[draw_mask]
                d_m = (dist_draw / 1000.0).reshape(-1, 1, 1)
                P_tof = self._rays[draw_mask] * d_m

                pts_cam = (self._R @ P_tof.reshape(-1, 3).T).T + self._t

                proj_K = K_new if K_new is not None else self._K
                pts_2d, _ = cv2.projectPoints(
                    pts_cam.reshape(-1, 1, 3).astype(np.float64),
                    np.zeros((3, 1)),
                    np.zeros((3, 1)),
                    proj_K,
                    np.zeros((1, 5)),
                )
                polygons = pts_2d.reshape(-1, 4, 2).astype(np.int32)
                overlay_drawn = True

                # Auto-scale colormap for readability
                d_m_valid = dist_draw / 1000.0
                if d_m_valid.size >= 4:
                    lo, hi = np.percentile(d_m_valid, [5, 95]).tolist()
                    if hi - lo < 0.05:
                        lo, hi = 0.2, 4.0
                else:
                    lo, hi = 0.2, 4.0

                norm_d = np.clip((d_m_valid - lo) / (hi - lo), 0.0, 1.0)
                lut_input = (norm_d * 255).astype(np.uint8).reshape(-1, 1)
                colored = cv2.applyColorMap(lut_input, self._get_colormap())

                overlay = bgr.copy()
                h_img, w_img = bgr.shape[:2]
                for poly, color_px in zip(polygons, colored):
                    b, g, r = int(color_px[0, 0]), int(color_px[0, 1]), int(color_px[0, 2])
                    poly_clipped = np.clip(poly, [0, 0], [w_img - 1, h_img - 1])
                    cv2.fillPoly(overlay, [poly_clipped.reshape(-1, 1, 2)], color=(b, g, r))
                    cv2.polylines(
                        overlay,
                        [poly_clipped.reshape(-1, 1, 2)],
                        isClosed=True,
                        color=(255, 255, 255),
                        thickness=1,
                    )

                bgr = cv2.addWeighted(overlay, 0.25, bgr, 0.75, 0.0)

        # Apply view transforms after drawing overlay so it stays aligned.
        bgr = self._apply_view_transform(bgr)

        if self._enable_overlay.isChecked():
            stale = age_ms is not None and age_ms > self._tof_stale_ms
            no_tof = self._last_tof_time_s is None
            if (stale or no_tof) and not overlay_drawn:
                h_img, w_img = bgr.shape[:2]
                overlay = bgr.copy()
                cv2.rectangle(overlay, (0, 0), (w_img, 40), (0, 0, 0), -1)
                if no_tof:
                    msg = "ToF data missing"
                else:
                    msg = f"ToF data stale: {age_ms:.0f} ms"
                cv2.putText(
                    overlay,
                    msg,
                    (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                bgr = cv2.addWeighted(overlay, 0.6, bgr, 0.4, 0.0)
        self._show_bgr(bgr)

    def _show_bgr(self, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg).scaled(
            self._cam_label.width(),
            self._cam_label.height(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self._cam_label.setPixmap(pix)
