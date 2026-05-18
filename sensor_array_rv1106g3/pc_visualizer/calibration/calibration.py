"""
calibration.py — Core calibration engine (no UI dependencies).

Supports
--------
• Intrinsic calibration of an RGB camera via a single ArUco marker
  viewed from many different angles (streaming, best-frame selection).
• Extrinsic calibration RGB ↔ VL53L8CH ToF (8×8 grid) using the same
  cube-on-wall target:  ArUco → cube pose in RGB frame,
                        ToF depth jump → cube center in ToF frame.
  The rigid transform is solved with the Kabsch (SVD) algorithm.

Usage (from calibration_page.py)
---------------------------------
    session = CalibrationSession(
        aruco_dict_id = cv2.aruco.DICT_4X4_50,
        marker_length_m = 0.18,
        n_total = 50,
        top_k = 20,
        quality_threshold = 0.45,
        target_rms = 0.5,
    )
    session.start()

    # on every incoming frame:
    info = session.process_frame(jpeg_bytes, tof_dist_mm, width, height)
    # info keys: quality, rms, n_frames, accepted, annotated_bgr

    if session.should_stop():
        result = session.finalize()
"""

from __future__ import annotations
import threading

import logging
import time
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import numpy as np


# VL53L8CH target status codes considered valid in the older offline pipeline.
# This helps avoid selecting spurious multi-target returns.
_VALID_TOF_STATUS = {5, 6, 9, 10}


# ──────────────────────────────────────────────────────────────────────────────
# Data containers
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class CalibFrame:
    idx: int
    timestamp_s: float
    jpeg: bytes
    corners: list                               # list of corner arrays
    ids: np.ndarray                             # (N,1)
    quality_score: float
    image_size: Tuple[int, int]                 # (w, h)
    tof_dist_mm: Optional[np.ndarray]          # shape (side,side) or None
    tof_sigma_mm: Optional[np.ndarray] = None  # shape (side,side) or None
    tof_status: Optional[np.ndarray] = None    # shape (side,side) or None
    # ChArUco (optional)
    charuco_corners: Optional[np.ndarray] = None  # (M,1,2)
    charuco_ids: Optional[np.ndarray] = None      # (M,1)


@dataclass
class ExtrinsicPair:
    frame_idx: int      # Added to link pair back to the specific frame
    p_rgb: np.ndarray   # (3,) metres
    p_tof: np.ndarray   # (3,) metres


@dataclass
class CalibResult:
    rms_error: float                            # px — intrinsic reprojection RMS
    camera_matrix: np.ndarray                  # (3,3)
    dist_coeffs: np.ndarray                    # (N,) / (1,N) where N=5 (pinhole) or N=4 (fisheye)
    rvecs: list
    tvecs: list
    n_frames: int
    timestamp_s: float
    camera_model: str = "pinhole"              # 'pinhole' | 'fisheye'
    # Extrinsic RGB ↔ ToF  (None if not enough pairs)
    R_tof_to_rgb: Optional[np.ndarray] = None  # (3,3)
    t_tof_to_rgb: Optional[np.ndarray] = None  # (3,)  metres
    extrinsic_rms_mm: Optional[float] = None


# ──────────────────────────────────────────────────────────────────────────────
# Quality scorer
# ──────────────────────────────────────────────────────────────────────────────

# ──────────────────────────────────────────────────────────────────────────────
# Quality scorer
# ──────────────────────────────────────────────────────────────────────────────

class FrameQualityScorer:
    """
    Scores a detected frame 0→1 based on sharpness, marker presence,
    spread, and pose diversity.
    """

    def score(
        self,
        bgr: np.ndarray,
        corners: list,
        ids,
        existing: List[CalibFrame],
    ) -> float:
        import cv2

        if ids is None or len(ids) == 0:
            return 0.0

        # 1. Sharpness
        gray     = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        lap_var  = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        sharpness = float(np.clip(lap_var / 120.0, 0.0, 1.0))

        # 2. Marker count
        n            = len(ids)
        marker_score = float(np.clip(n / 1.0, 0.0, 1.0))

        # 3. Corner spread
        h, w    = bgr.shape[:2]
        all_c   = np.vstack([c.reshape(-1, 2) for c in corners])
        cx, cy  = all_c[:, 0] / w, all_c[:, 1] / h
        spread  = float(np.clip((np.std(cx) + np.std(cy)) / 0.25, 0.0, 1.0))

        # 4. Pose diversity (Fix: This method was missing)
        diversity = self._diversity(corners, (w, h), existing)

        total = (
            sharpness    * 0.35 +
            marker_score * 0.25 +
            spread       * 0.15 +
            diversity    * 0.25
        )
        return float(np.clip(total, 0.0, 1.0))

    def _diversity(
        self,
        corners: list,
        size: Tuple[int, int],
        existing: List[CalibFrame],
    ) -> float:
        """Ensures the user moves the marker to different parts of the screen."""
        if not existing or not corners:
            return 1.0

        w, h = size
        # Calculate center of current marker detection (normalized 0-1)
        all_c = np.vstack([c.reshape(-1, 2) for c in corners])
        cm = all_c.mean(0) / np.array([w, h], dtype=float)

        min_d = 1.0
        for ef in existing:
            # Calculate distance to existing frames
            ec = np.vstack([c.reshape(-1, 2) for c in ef.corners])
            ecm = ec.mean(0) / np.array(ef.image_size, dtype=float)
            d = float(np.linalg.norm(cm - ecm))
            if d < min_d:
                min_d = d

        # 0.10 image-fraction displacement (10% movement) -> full diversity score
        return float(np.clip(min_d / 0.10, 0.0, 1.0))


# ──────────────────────────────────────────────────────────────────────────────
# Calibration session
# ──────────────────────────────────────────────────────────────────────────────

class CalibrationSession:
    """
    Manages a streaming calibration session.

    Parameters
    ----------
    aruco_dict_id     : cv2.aruco dict constant  (e.g. cv2.aruco.DICT_4X4_50)
    marker_length_m   : physical side length of the printed marker (metres)
    max_frames        : stop capturing after this many accepted frames
    quality_threshold : minimum quality score to accept a frame (0–1)
    target_rms        : stop automatically when RMS reprojection ≤ this (px)
    tof_fov_deg       : full FoV of the VL53L8CH (default 45°)
    recalib_every     : re-run calibrateCamera every N new accepted frames
    min_frames_calib  : minimum accepted frames before first calibration attempt
    """

    def __init__(
        self,
        aruco_dict_id: int,
        pattern: str = "aruco",     # 'aruco' or 'charuco'
        marker_length_m: float = 0.18,
        # ChArUco board parameters (metres)
        charuco_squares_x: int = 7,
        charuco_squares_y: int = 5,
        charuco_square_length_m: float = 0.03,
        charuco_marker_length_m: float = 0.022,
        n_total: int = 50,          # Pool size
        top_k: int = 20,            # Selection size
        quality_threshold: float = 0.45,
        target_rms: float = 0.5,
        tof_fov_deg: float = 45.0,
        recalib_every: int = 3,
        min_frames_calib: int = 5,
        tof_rot_k: int = 0,
        tof_flip_x: bool = False,
        tof_flip_y: bool = False,
        board_offset_xy_mm: Tuple[float, float] = (0.0, 0.0),
        camera_model: str = "pinhole",
    ):
        self.aruco_dict_id = aruco_dict_id
        self.pattern = str(pattern).lower().strip() or "aruco"
        self.marker_length_m = marker_length_m
        self.charuco_squares_x = int(charuco_squares_x)
        self.charuco_squares_y = int(charuco_squares_y)
        self.charuco_square_length_m = float(charuco_square_length_m)
        self.charuco_marker_length_m = float(charuco_marker_length_m)
        self.n_total = n_total
        self.top_k = top_k
        self.quality_threshold = quality_threshold
        self.target_rms = target_rms
        self.tof_fov_deg = tof_fov_deg
        self.recalib_every = recalib_every
        self.min_frames_calib = min_frames_calib

        self.camera_model = str(camera_model or "pinhole").strip().lower()
        if self.camera_model not in ("pinhole", "fisheye"):
            self.camera_model = "pinhole"

        try:
            self._tof_rot_k = int(tof_rot_k) % 4
        except Exception:
            self._tof_rot_k = 0
        self._tof_flip_x = bool(tof_flip_x)
        self._tof_flip_y = bool(tof_flip_y)
        self._set_board_offset(board_offset_xy_mm)

        self.frames: List[CalibFrame] = []
        self.ext_pairs: List[ExtrinsicPair] = []
        self.latest_result: Optional[CalibResult] = None

        self._extrinsic_only = False

        self._scorer = FrameQualityScorer()
        self._frame_idx = 0
        self._since_last_calib = 0
        self._running = False
        self._image_size: Optional[Tuple[int, int]] = None

        # OpenCV lazy init
        self._cv2 = None
        self._aruco = None
        self._detector = None
        self._board = None
        self._setup_cv()

    # ── Setup ──────────────────────────────────────────────────────────────

    def _setup_cv(self):
        try:
            import cv2
            self._cv2 = cv2
            self._aruco = cv2.aruco
            d = cv2.aruco.getPredefinedDictionary(self.aruco_dict_id)
            p = cv2.aruco.DetectorParameters()

            # Wider adaptive threshold window → survives dark/low-contrast frames.
            p.adaptiveThreshWinSizeMin  = 3
            p.adaptiveThreshWinSizeMax  = 53
            p.adaptiveThreshWinSizeStep = 10
            # Allow markers that appear small due to wide-angle FOV.
            p.minMarkerPerimeterRate = 0.02
            p.maxMarkerPerimeterRate = 4.0
            # Sub-pixel corner refinement for better reprojection RMS.
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

            self._detector = cv2.aruco.ArucoDetector(d, p)
            # CLAHE used to normalise dark frames before detection.
            self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

            # Optional ChArUco board
            self._board = None
            if self.pattern == "charuco":
                try:
                    self._board = cv2.aruco.CharucoBoard(
                        (self.charuco_squares_x, self.charuco_squares_y),
                        self.charuco_square_length_m,
                        self.charuco_marker_length_m,
                        d,
                    )
                except Exception:
                    # Older OpenCV Python bindings might use a factory.
                    try:
                        self._board = cv2.aruco.CharucoBoard_create(
                            self.charuco_squares_x,
                            self.charuco_squares_y,
                            self.charuco_square_length_m,
                            self.charuco_marker_length_m,
                            d,
                        )
                    except Exception:
                        self._board = None
        except Exception as e:
            import logging
            logging.error("CalibrationSession._setup_cv failed: %s", e)
            self._cv2   = None
            self._clahe = None
            self._board = None

    def update_params(self, **kwargs):
        """Hot-update parameters; rebuilds detector if dict_id changes."""
        rebuild = any(k in kwargs for k in (
            'aruco_dict_id', 'pattern',
            'charuco_squares_x', 'charuco_squares_y',
            'charuco_square_length_m', 'charuco_marker_length_m',
        ))
        for k, v in kwargs.items():
            if k == 'board_offset_xy_mm':
                self._set_board_offset(v)
                continue
            if hasattr(self, k):
                setattr(self, k, v)
        if 'tof_rot_k' in kwargs:
            try:
                self._tof_rot_k = int(kwargs.get('tof_rot_k', 0)) % 4
            except Exception:
                self._tof_rot_k = 0
        if 'tof_flip_x' in kwargs:
            self._tof_flip_x = bool(kwargs.get('tof_flip_x', False))
        if 'tof_flip_y' in kwargs:
            self._tof_flip_y = bool(kwargs.get('tof_flip_y', False))
        if rebuild:
            self._setup_cv()

    def _set_board_offset(self, offset_xy_mm):
        try:
            off_x_mm, off_y_mm = offset_xy_mm
        except Exception:
            off_x_mm, off_y_mm = 0.0, 0.0
        self._board_offset_m = np.array(
            [float(off_x_mm), float(off_y_mm), 0.0], dtype=np.float64
        ) / 1000.0

    # ── Lifecycle ──────────────────────────────────────────────────────────

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def n_frames(self) -> int:
        return len(self.frames)

    def start(self):
        self.frames.clear()
        self.ext_pairs.clear()
        self.latest_result   = None
        self._frame_idx      = 0
        self._since_last_calib = 0
        self._image_size     = None
        self._running        = True
        self._extrinsic_only = False
        # Background calibration threading.
        self._calib_lock     = threading.Lock()
        self._calib_thread: Optional[threading.Thread] = None
        self._pending_calib: Optional[CalibResult] = None

    def start_extrinsic(self, intrinsic: CalibResult):
        """Start an extrinsic-only acquisition phase using fixed intrinsics."""
        self.frames.clear()
        self.ext_pairs.clear()
        self.latest_result = intrinsic
        try:
            self.camera_model = str(getattr(intrinsic, 'camera_model', self.camera_model) or self.camera_model).strip().lower()
        except Exception:
            pass
        self._frame_idx = 0
        self._since_last_calib = 0
        self._image_size = None
        self._running = True
        self._extrinsic_only = True
        self._calib_lock = threading.Lock()
        self._calib_thread = None
        self._pending_calib = None

    def stop(self):
        self._running = False

    # ── Main entry point ───────────────────────────────────────────────────

    def process_frame(
        self,
        jpeg: bytes,
        tof_dist_mm,
        w: int,
        h: int,
    ) -> dict:
        out = {
            'quality':       0.0,
            'rms':           self.latest_result.rms_error if self.latest_result else None,
            'n_frames':      len(self.frames),
            'accepted':      False,
            'annotated_bgr': None,
        }

        if not self._running or not self._cv2:
            return out

        # Promote any background calibration result that finished.
        with self._calib_lock:
            if self._pending_calib is not None:
                self.latest_result  = self._pending_calib
                self._pending_calib = None
                out['rms']          = self.latest_result.rms_error
                self._since_last_calib = 0

        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = self._cv2.imdecode(arr, self._cv2.IMREAD_COLOR)
        if bgr is None:
            return out

        tof_grid_mm, tof_sigma_mm, tof_status = self._tof_primary_grid(tof_dist_mm)

        h_img, w_img = bgr.shape[:2]
        if w <= 0 or h <= 0:
            w, h = w_img, h_img
        self._image_size = (w, h)

        # CLAHE equalisation — makes dark / low-contrast frames detectable.
        if self._clahe is not None:
            gray    = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2GRAY)
            gray_eq = self._clahe.apply(gray)
            detect_bgr = self._cv2.cvtColor(gray_eq, self._cv2.COLOR_GRAY2BGR)
        else:
            detect_bgr = bgr

        corners, ids, _ = self._detector.detectMarkers(detect_bgr)

        # If in ChArUco mode, refine to chessboard corners
        charuco_corners = None
        charuco_ids = None
        if self.pattern == "charuco" and self._board is not None and ids is not None and len(ids) > 0:
            try:
                gray = self._cv2.cvtColor(detect_bgr, self._cv2.COLOR_BGR2GRAY)

                if hasattr(self._cv2.aruco, "CharucoDetector"):
                    detector = self._cv2.aruco.CharucoDetector(self._board)
                    charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)

                if charuco_corners is None or charuco_ids is None or len(charuco_ids) == 0:
                    try:
                        result = self._aruco.interpolateCornersCharuco(
                            markerCorners=corners,
                            markerIds=ids,
                            image=gray,
                            board=self._board,
                            minMarkers=1,
                        )
                        if isinstance(result, tuple) and len(result) == 3:
                            retval, charuco_corners, charuco_ids = result
                        elif isinstance(result, tuple) and len(result) == 2:
                            charuco_corners, charuco_ids = result
                            retval = len(charuco_ids) if charuco_ids is not None else 0
                        else:
                            retval = 0
                        if retval == 0:
                            charuco_corners, charuco_ids = None, None
                    except Exception:
                        charuco_corners, charuco_ids = None, None
            except Exception:
                charuco_corners, charuco_ids = None, None

        annot = bgr.copy()
        if ids is not None and len(ids) > 0:
            self._aruco.drawDetectedMarkers(annot, corners, ids)
        if charuco_corners is not None and charuco_ids is not None:
            try:
                self._aruco.drawDetectedCornersCharuco(annot, charuco_corners, charuco_ids)
            except Exception:
                pass
        out['annotated_bgr'] = annot

        if self.pattern == "charuco":
            if charuco_ids is None or len(charuco_ids) < 6:
                return out
        else:
            if ids is None or len(ids) == 0:
                return out

        # Reuse marker-based scorer (sharpness/spread/diversity). In ChArUco
        # mode, ensure we still have marker corners for diversity.
        quality = self._scorer.score(detect_bgr, corners, ids, self.frames)
        out['quality'] = quality

        if quality >= self.quality_threshold and len(self.frames) < self.n_total:
            frame = CalibFrame(
                idx          = self._frame_idx,
                timestamp_s  = time.time(),
                jpeg         = jpeg,
                tof_dist_mm  = tof_grid_mm.copy() if tof_grid_mm is not None else None,
                tof_sigma_mm = tof_sigma_mm.copy() if tof_sigma_mm is not None else None,
                tof_status   = tof_status.copy() if tof_status is not None else None,
                corners      = corners,
                ids          = ids,
                quality_score= quality,
                image_size   = (w, h),
                charuco_corners = charuco_corners.copy() if charuco_corners is not None else None,
                charuco_ids     = charuco_ids.copy() if charuco_ids is not None else None,
            )
            self.frames.append(frame)
            self._frame_idx       += 1
            out['accepted']        = True
            self._since_last_calib += 1

            if frame.tof_dist_mm is not None and self.latest_result is not None:
                pair = self._extract_ext_pair_from_frame(frame)
                if pair is not None:
                    self.ext_pairs.append(pair)
                    
            if not self._extrinsic_only:
                # Trigger background calibration — never blocks the stream.
                calib_due = (
                    self._since_last_calib >= self.recalib_every
                    and len(self.frames)    >= self.min_frames_calib
                )
                thread_idle = (self._calib_thread is None or not self._calib_thread.is_alive())

                if calib_due and thread_idle:
                    snap_frames = list(self.frames)     # snapshot; list() is thread-safe enough here
                    snap_pairs  = list(self.ext_pairs)
                    _w, _h      = w, h

                    def _bg_calib():
                        best = sorted(snap_frames, key=lambda f: f.quality_score, reverse=True)[:self.top_k]
                        calib = (
                            self._calibrate_charuco_on(best, _w, _h)
                            if self.pattern == "charuco" else
                            self._calibrate_on(best, _w, _h)
                        )
                        if calib is None:
                            return
                        if len(snap_pairs) >= 4:
                            R, t, rms_mm = self._solve_extrinsic_on(snap_pairs)
                            calib.R_tof_to_rgb    = R
                            calib.t_tof_to_rgb    = t
                            calib.extrinsic_rms_mm = rms_mm
                            self._warn_if_implausible_translation(t, context="background")
                        with self._calib_lock:
                            self._pending_calib = calib

                    self._calib_thread = threading.Thread(target=_bg_calib, daemon=True)
                    self._calib_thread.start()

        out['n_frames'] = len(self.frames)
        return out

    # ── Stop criteria & finalisation ───────────────────────────────────────

    def should_stop(self) -> bool:
        """Stop when pool is full; only allow RMS stop after enough frames."""
        if len(self.frames) >= self.n_total:
            return True
        if self._extrinsic_only:
            return len(self.ext_pairs) >= min(self.top_k, self.n_total)
        # Prevent premature stop before we can meaningfully select top_k.
        if (
            len(self.frames) >= min(self.top_k, self.n_total)
            and self.latest_result
            and self.latest_result.rms_error <= self.target_rms
        ):
            return True
        return False

    def stop_reason(self) -> str:
        if self._extrinsic_only and len(self.ext_pairs) >= min(self.top_k, self.n_total):
            return f"extrinsic pairs reached ({len(self.ext_pairs)})"
        if len(self.frames) >= self.n_total:
            return f"max frames reached ({self.n_total})"
        if (
            len(self.frames) >= min(self.top_k, self.n_total)
            and self.latest_result
            and self.latest_result.rms_error <= self.target_rms
        ):
            return f"target RMS reached ({self.target_rms:.2f} px)"
        return "manual stop"

    def finalize(self) -> Optional[CalibResult]:
        if not self.frames:
            return None

        if self._extrinsic_only:
            if self.latest_result is None:
                return None
            calib = CalibResult(
                rms_error=float(self.latest_result.rms_error),
                camera_matrix=np.asarray(self.latest_result.camera_matrix, dtype=np.float64),
                dist_coeffs=np.asarray(self.latest_result.dist_coeffs, dtype=np.float64),
                rvecs=list(getattr(self.latest_result, 'rvecs', [])),
                tvecs=list(getattr(self.latest_result, 'tvecs', [])),
                n_frames=len(self.frames),
                timestamp_s=time.time(),
                camera_model=str(getattr(self.latest_result, 'camera_model', self.camera_model) or self.camera_model),
            )
            # Use a quality-based subset for robustness.
            frames_for_ext = sorted(self.frames, key=lambda f: f.quality_score, reverse=True)[:self.top_k]
            pairs: List[ExtrinsicPair] = []
            K_final, D_final = calib.camera_matrix, calib.dist_coeffs
            for f in frames_for_ext:
                if f.tof_dist_mm is None:
                    continue
                p_rgb = self._p_rgb_from_frame(f, K_final, D_final)
                if p_rgb is None:
                    continue
                p_tof = self._tof_cube_centre(f.tof_dist_mm, f.tof_sigma_mm, f.tof_status)
                if p_tof is None:
                    continue
                pairs.append(ExtrinsicPair(frame_idx=f.idx, p_rgb=p_rgb, p_tof=p_tof))

            if len(pairs) >= 4:
                result = self._solve_extrinsic_robust(pairs)
                if result[0] is not None:
                    calib.R_tof_to_rgb, calib.t_tof_to_rgb, calib.extrinsic_rms_mm = result
                    self._warn_if_implausible_translation(calib.t_tof_to_rgb, context="extrinsic-only finalize")
            return calib

        # 1. First Pass: Initial calibration using the top_k best quality frames
        best_frames = sorted(self.frames, key=lambda f: f.quality_score, reverse=True)[:self.top_k]
        if self._image_size is None:
            w, h = best_frames[0].image_size
        else:
            w, h = self._image_size
        calib = (
            self._calibrate_charuco_on(best_frames, w, h)
            if self.pattern == "charuco" else
            self._calibrate_on(best_frames, w, h)
        )
        if calib is None:
            return None

        # 2. Refinement Pass: pick top_k by lowest reprojection error
        # This aligns selection with the requested "minimize error" criteria.
        K, D = calib.camera_matrix, calib.dist_coeffs
        err_frames = []
        for f in self.frames:
            err = (
                self._frame_reproj_error_charuco(f, K, D)
                if self.pattern == "charuco" else
                self._frame_reproj_error(f, K, D)
            )
            if err is not None:
                err_frames.append((err, f))

        err_frames.sort(key=lambda x: x[0])
        top_err_frames = [f for _, f in err_frames[:self.top_k]]
        refined_frames = [f for err, f in err_frames[:self.top_k] if err < 1.5]

        # Re-run intrinsic calibration with only the "clean" frames
        if len(refined_frames) >= self.min_frames_calib:
            calib = (
                self._calibrate_charuco_on(refined_frames, w, h)
                if self.pattern == "charuco" else
                self._calibrate_on(refined_frames, w, h)
            )
        else:
            # Fallback to the best-by-error frames, then to quality frames
            refined_frames = top_err_frames if top_err_frames else best_frames

        # 3. Robust Extrinsic Pass
        final_pairs: List[ExtrinsicPair] = []
        K_final, D_final = calib.camera_matrix, calib.dist_coeffs

        for f in refined_frames:
            if f.tof_dist_mm is None:
                continue
            p_rgb = self._p_rgb_from_frame(f, K_final, D_final)
            if p_rgb is None:
                continue
            p_tof = self._tof_cube_centre(f.tof_dist_mm, f.tof_sigma_mm, f.tof_status)
            if p_tof is not None:
                final_pairs.append(ExtrinsicPair(frame_idx=f.idx, p_rgb=p_rgb, p_tof=p_tof))

        if len(final_pairs) >= 4:
            # Use RANSAC-style solver to ignore bad ToF depth jumps
            result = self._solve_extrinsic_robust(final_pairs)
            if result[0] is not None:
                calib.R_tof_to_rgb, calib.t_tof_to_rgb, calib.extrinsic_rms_mm = result
                self._warn_if_implausible_translation(calib.t_tof_to_rgb, context="finalize")

        return calib

    def _warn_if_implausible_translation(self, t_m: Optional[np.ndarray], context: str = ""):
        """Warn when |t| is so large it likely indicates a correspondence issue."""
        if t_m is None:
            return
        try:
            t = np.asarray(t_m, dtype=np.float64).reshape(-1)[:3]
            if t.size != 3:
                return
            t_norm_mm = float(np.linalg.norm(t)) * 1000.0
        except Exception:
            return

        # Heuristic: camera + ToF on the same small rig/PCB should be well below this.
        if t_norm_mm > 500.0:
            prefix = f"[{context}] " if context else ""
            logging.warning(
                "%sExtrinsic translation magnitude is large: |t|=%.0f mm (t=%s). "
                "This often means the ToF zone orientation/flip is wrong, the target is too close (<60cm xtalk bias), "
                "or the ToF foreground detection picked the wrong surface.",
                prefix,
                t_norm_mm,
                np.array2string(t, precision=3, suppress_small=True),
            )

    def _solve_extrinsic_robust(self, pairs: list, iterations: int = 100):
        """
        Picks the best subset of 4 points that minimizes the error for the whole set.
        This prevents one bad ToF reading from ruining the sensor alignment.
        """
        import random
        best_R, best_t, min_rms = None, None, float('inf')
        
        if len(pairs) < 6: # Not enough data for RANSAC
            return self._solve_extrinsic_on(pairs)

        for _ in range(iterations):
            subset = random.sample(pairs, 4)
            try:
                R, t, _ = self._solve_extrinsic_on(subset)
            except Exception:
                continue
            if R is None:
                continue
            
            # Check error against ALL pairs
            P_tof = np.array([p.p_tof for p in pairs])
            P_rgb = np.array([p.p_rgb for p in pairs])
            P_pred = (R @ P_tof.T).T + t
            rms = float(np.sqrt(((P_pred - P_rgb) ** 2).sum(1).mean()))
            
            if rms < min_rms:
                min_rms = rms
                best_R, best_t = R, t

        if best_R is None:
            return self._solve_extrinsic_on(pairs)

        return best_R, best_t, min_rms * 1000.0

    def _frame_reproj_error(
        self,
        frame: CalibFrame,
        K: np.ndarray,
        D: np.ndarray,
    ) -> Optional[float]:
        if frame.ids is None or len(frame.corners) == 0:
            return None
        obj_p = self._obj_pts()
        img_p = frame.corners[0].reshape(4, 2).astype(np.float32)

        if self.camera_model == "fisheye" and hasattr(self._cv2, "fisheye"):
            D4 = np.asarray(D, dtype=np.float64).reshape(-1)[:4].reshape(4, 1)
            und = self._cv2.fisheye.undistortPoints(
                img_p.reshape(-1, 1, 2).astype(np.float64), K, D4, P=K
            )
            img_u = und.reshape(-1, 2).astype(np.float32)
            ok, r, t = self._cv2.solvePnP(obj_p, img_u, K, np.zeros((1, 5), dtype=np.float64))
            if not ok:
                return None
            proj, _ = self._cv2.projectPoints(obj_p, r, t, K, np.zeros((1, 5), dtype=np.float64))
            return float(np.linalg.norm(img_u - proj.reshape(4, 2), axis=1).mean())

        ok, r, t = self._cv2.solvePnP(obj_p, img_p, K, D)
        if not ok:
            return None
        proj, _ = self._cv2.projectPoints(obj_p, r, t, K, D)
        return float(np.linalg.norm(img_p - proj.reshape(4, 2), axis=1).mean())

    def _charuco_board_center_obj(self) -> Optional[np.ndarray]:
        """Board center in board coordinates (metres)."""
        if self._board is None:
            return None
        try:
            chess = self._board.getChessboardCorners()
        except Exception:
            chess = getattr(self._board, "chessboardCorners", None)
        if chess is None:
            return None
        return np.asarray(chess, dtype=np.float64).reshape(-1, 3).mean(axis=0)

    def _charuco_obj_points_for_ids(self, charuco_ids: np.ndarray) -> Optional[np.ndarray]:
        if self._board is None:
            return None
        try:
            chess = self._board.getChessboardCorners()
        except Exception:
            chess = getattr(self._board, "chessboardCorners", None)
        if chess is None:
            return None
        chess = np.asarray(chess, dtype=np.float64).reshape(-1, 3)
        ids = np.asarray(charuco_ids, dtype=int).flatten()
        if ids.size == 0:
            return None
        if ids.max() >= len(chess) or ids.min() < 0:
            return None
        return chess[ids]

    def _p_rgb_from_frame(self, frame: CalibFrame, K: np.ndarray, D: np.ndarray) -> Optional[np.ndarray]:
        """Returns the 3D point in RGB camera frame that corresponds to the target center."""
        if self.pattern == "charuco":
            if frame.charuco_corners is None or frame.charuco_ids is None or len(frame.charuco_ids) < 6:
                return None
            obj = self._charuco_obj_points_for_ids(frame.charuco_ids)
            if obj is None:
                return None
            img = np.asarray(frame.charuco_corners, dtype=np.float32).reshape(-1, 2)
            if self.camera_model == "fisheye" and hasattr(self._cv2, "fisheye"):
                D4 = np.asarray(D, dtype=np.float64).reshape(-1)[:4].reshape(4, 1)
                und = self._cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2).astype(np.float64), K, D4, P=K)
                img_u = und.reshape(-1, 2).astype(np.float32)
                ok, rvec, tvec = self._cv2.solvePnP(
                    obj.astype(np.float32), img_u, K, np.zeros((1, 5), dtype=np.float64)
                )
            else:
                ok, rvec, tvec = self._cv2.solvePnP(obj.astype(np.float32), img.astype(np.float32), K, D)
            if not ok:
                return None
            R, _ = self._cv2.Rodrigues(rvec)
            c = self._charuco_board_center_obj()
            if c is None:
                return None
            c = c + self._board_offset_m
            p = (R @ c.reshape(3, 1) + tvec.reshape(3, 1)).reshape(3)
            return p.astype(np.float64)

        # ArUco single marker
        if frame.ids is None or len(frame.corners) == 0:
            return None
        img_p = frame.corners[0].reshape(4, 2).astype(np.float32)
        if self.camera_model == "fisheye" and hasattr(self._cv2, "fisheye"):
            D4 = np.asarray(D, dtype=np.float64).reshape(-1)[:4].reshape(4, 1)
            und = self._cv2.fisheye.undistortPoints(
                img_p.reshape(-1, 1, 2).astype(np.float64), K, D4, P=K
            )
            img_u = und.reshape(-1, 2).astype(np.float32)
            ok, _, tvec = self._cv2.solvePnP(
                self._obj_pts(), img_u, K, np.zeros((1, 5), dtype=np.float64)
            )
        else:
            ok, _, tvec = self._cv2.solvePnP(self._obj_pts(), img_p, K, D)
        if not ok:
            return None
        return tvec.flatten().astype(np.float64)

    def _frame_reproj_error_charuco(
        self,
        frame: CalibFrame,
        K: np.ndarray,
        D: np.ndarray,
    ) -> Optional[float]:
        if frame.charuco_corners is None or frame.charuco_ids is None:
            return None
        if len(frame.charuco_ids) < 6:
            return None
        obj = self._charuco_obj_points_for_ids(frame.charuco_ids)
        if obj is None:
            return None
        img = np.asarray(frame.charuco_corners, dtype=np.float64).reshape(-1, 2)

        if self.camera_model == "fisheye" and hasattr(self._cv2, "fisheye"):
            D4 = np.asarray(D, dtype=np.float64).reshape(-1)[:4].reshape(4, 1)
            und = self._cv2.fisheye.undistortPoints(img.reshape(-1, 1, 2).astype(np.float64), K, D4, P=K)
            img_u = und.reshape(-1, 2).astype(np.float32)
            ok, rvec, tvec = self._cv2.solvePnP(
                obj.astype(np.float32), img_u, K, np.zeros((1, 5), dtype=np.float64)
            )
            if not ok:
                return None
            proj, _ = self._cv2.projectPoints(
                obj.astype(np.float32), rvec, tvec, K, np.zeros((1, 5), dtype=np.float64)
            )
            return float(np.linalg.norm(img_u - proj.reshape(-1, 2), axis=1).mean())

        ok, rvec, tvec = self._cv2.solvePnP(obj.astype(np.float32), img.astype(np.float32), K, D)
        if not ok:
            return None
        proj, _ = self._cv2.projectPoints(obj.astype(np.float32), rvec, tvec, K, D)
        proj2 = proj.reshape(-1, 2)
        return float(np.linalg.norm(img - proj2, axis=1).mean())

    # ── Intrinsic calibration ──────────────────────────────────────────────

    def _obj_pts(self) -> np.ndarray:
        """4 corners of the ArUco marker in its local 3-D frame (z = 0)."""
        L = self.marker_length_m / 2.0
        return np.array(
            [[-L, L, 0], [L, L, 0], [L, -L, 0], [-L, -L, 0]],
            dtype=np.float32,
        )

    def _calibrate_on(self, frames: list, w: int, h: int) -> Optional[CalibResult]:
        """Thread-safe: operates on a snapshot list, not self.frames."""
        cv2 = self._cv2
        obj_single = self._obj_pts()

        obj_list, img_list = [], []
        for f in frames:
            if f.ids is None:
                continue
            for i in range(len(f.ids)):
                c = f.corners[i].reshape(4, 2).astype(np.float32)
                obj_list.append(obj_single)
                img_list.append(c)

        if len(obj_list) < self.min_frames_calib:
            return None

        try:
            if self.camera_model == "fisheye" and hasattr(cv2, "fisheye"):
                obj_f = [np.asarray(o, dtype=np.float64).reshape(-1, 1, 3) for o in obj_list]
                img_f = [np.asarray(i, dtype=np.float64).reshape(-1, 1, 2) for i in img_list]
                K = np.eye(3, dtype=np.float64)
                dist = np.zeros((4, 1), dtype=np.float64)
                flags = (
                    cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                    cv2.fisheye.CALIB_FIX_SKEW
                )
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
                rms, K, dist, rvecs, tvecs = cv2.fisheye.calibrate(
                    obj_f, img_f, (w, h), K, dist,
                    flags=flags, criteria=criteria,
                )
                model = "fisheye"
            else:
                rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
                    obj_list, img_list, (w, h),
                    None, None,
                )
                model = "pinhole"
            return CalibResult(
                rms_error     = float(rms),
                camera_matrix = K,
                dist_coeffs   = dist,
                rvecs         = list(rvecs),
                tvecs         = list(tvecs),
                n_frames      = len(frames),
                timestamp_s   = time.time(),
                camera_model  = model,
            )
        except Exception:
            return None

    def _calibrate_charuco_on(self, frames: list, w: int, h: int) -> Optional[CalibResult]:
        """Intrinsic calibration using ChArUco corners."""
        if self._board is None or self._cv2 is None:
            return None

        all_cc = []
        all_ci = []
        for f in frames:
            if f.charuco_corners is None or f.charuco_ids is None:
                continue
            if len(f.charuco_ids) < 6:
                continue
            all_cc.append(np.asarray(f.charuco_corners, dtype=np.float32))
            all_ci.append(np.asarray(f.charuco_ids, dtype=np.int32))

        if len(all_cc) < self.min_frames_calib:
            return None

        aruco = self._aruco
        try:
            if self.camera_model == "fisheye" and hasattr(self._cv2, "fisheye"):
                obj_list, img_list = [], []
                for cc, ci in zip(all_cc, all_ci):
                    obj = self._charuco_obj_points_for_ids(ci)
                    if obj is None:
                        continue
                    img = np.asarray(cc, dtype=np.float64).reshape(-1, 2)
                    if len(img) < 6:
                        continue
                    obj_list.append(np.asarray(obj, dtype=np.float64).reshape(-1, 1, 3))
                    img_list.append(np.asarray(img, dtype=np.float64).reshape(-1, 1, 2))
                if len(obj_list) < self.min_frames_calib:
                    return None

                K = np.eye(3, dtype=np.float64)
                dist = np.zeros((4, 1), dtype=np.float64)
                flags = (
                    self._cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                    self._cv2.fisheye.CALIB_FIX_SKEW
                )
                criteria = (self._cv2.TERM_CRITERIA_EPS + self._cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
                rms, K, dist, rvecs, tvecs = self._cv2.fisheye.calibrate(
                    obj_list, img_list, (w, h), K, dist,
                    flags=flags, criteria=criteria,
                )
                model = "fisheye"
            else:
                if hasattr(aruco, 'calibrateCameraCharuco'):
                    rms, K, dist, rvecs, tvecs = aruco.calibrateCameraCharuco(
                        charucoCorners=all_cc,
                        charucoIds=all_ci,
                        board=self._board,
                        imageSize=(w, h),
                        cameraMatrix=None,
                        distCoeffs=None,
                    )
                else:
                    # Fallback: treat each ChArUco corner as a generic 2D-3D correspondence.
                    obj_list, img_list = [], []
                    for cc, ci in zip(all_cc, all_ci):
                        obj = self._charuco_obj_points_for_ids(ci)
                        if obj is None:
                            continue
                        img = np.asarray(cc, dtype=np.float32).reshape(-1, 2)
                        obj_list.append(obj.astype(np.float32))
                        img_list.append(img.astype(np.float32))
                    if len(obj_list) < self.min_frames_calib:
                        return None
                    rms, K, dist, rvecs, tvecs = self._cv2.calibrateCamera(
                        obj_list, img_list, (w, h), None, None
                    )
                model = "pinhole"

            return CalibResult(
                rms_error     = float(rms),
                camera_matrix = K,
                dist_coeffs   = dist,
                rvecs         = list(rvecs),
                tvecs         = list(tvecs),
                n_frames      = len(frames),
                timestamp_s   = time.time(),
                camera_model  = model,
            )
        except Exception:
            return None

    def _solve_extrinsic_on(
        self, pairs: list
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Thread-safe: operates on a snapshot list."""
        P_tof = np.array([p.p_tof for p in pairs])
        P_rgb = np.array([p.p_rgb for p in pairs])

        mu_tof = P_tof.mean(0)
        mu_rgb = P_rgb.mean(0)
        A = P_tof - mu_tof
        B = P_rgb - mu_rgb
        H = A.T @ B
        U, _, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
        t = mu_rgb - R @ mu_tof
        P_pred   = (R @ P_tof.T).T + t
        rms_m    = float(np.sqrt(((P_pred - P_rgb) ** 2).sum(1).mean()))
        return R, t, rms_m * 1000.0


    # ── Extrinsic calibration ──────────────────────────────────────────────


    def _extract_ext_pair_from_frame(self, frame: CalibFrame) -> Optional[ExtrinsicPair]:
        if self.latest_result is None:
            return None
        K = self.latest_result.camera_matrix
        D = self.latest_result.dist_coeffs

        # Note: uses board center vs ToF depth centroid; adjust board_offset_xy_mm if needed.
        p_rgb = self._p_rgb_from_frame(frame, K, D)
        if p_rgb is None:
            return None
        if frame.tof_dist_mm is None:
            return None
        p_tof = self._tof_cube_centre(frame.tof_dist_mm, frame.tof_sigma_mm, frame.tof_status)
        if p_tof is None:
            return None
        return ExtrinsicPair(frame_idx=frame.idx, p_rgb=p_rgb, p_tof=p_tof)

    def _tof_primary_grid(
        self,
        tof,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """Convert a ToF payload to a single distance grid (and optional sigma/status grids).

        Accepts either:
          - None
          - a numpy array (already a grid, or (N_zones,N_targets))
          - a parsed protocol `TofFrame` (has distance_mm, sigma_mm, status)

        For multi-target per zone, selects the closest non-zero range per zone.
        """
        if tof is None:
            return None, None, None

        # If a numpy array was passed, keep previous behaviour.
        if isinstance(tof, np.ndarray):
            dist = tof
            if dist.ndim == 2:
                r0, c0 = dist.shape
                side = int(np.sqrt(r0))
                if side * side == r0 and c0 != side:
                    dist = dist[:, 0].reshape(side, side)
            if dist.ndim != 2:
                return None, None, None
            return dist, None, None

        # Try to interpret as TofFrame-like object.
        if not hasattr(tof, 'distance_mm'):
            return None, None, None

        dist = np.asarray(getattr(tof, 'distance_mm'))
        sigma = np.asarray(getattr(tof, 'sigma_mm', None)) if hasattr(tof, 'sigma_mm') else None
        status = np.asarray(getattr(tof, 'status', None)) if hasattr(tof, 'status') else None

        if dist.ndim != 2:
            return None, None, None

        r0, c0 = dist.shape
        side = int(np.sqrt(r0))
        if side * side == r0 and c0 != side:
            # (N_zones, N_targets)
            valid = (dist > 0) & (dist < 8000)
            if status is not None and status.shape == dist.shape:
                try:
                    valid &= np.isin(status, list(_VALID_TOF_STATUS))
                except Exception:
                    pass
            dist_masked = np.where(valid, dist.astype(np.float32), np.inf)
            idx = np.argmin(dist_masked, axis=1)
            sel = dist_masked[np.arange(r0), idx]
            sel[~np.isfinite(sel)] = 0
            dist_grid = sel.astype(np.float32).reshape(side, side)

            sigma_grid = None
            if sigma is not None and sigma.shape == dist.shape:
                sigma_grid = sigma[np.arange(r0), idx].astype(np.float32).reshape(side, side)
            status_grid = None
            if status is not None and status.shape == dist.shape:
                status_grid = status[np.arange(r0), idx].astype(np.uint8).reshape(side, side)
            return dist_grid, sigma_grid, status_grid

        # Already spatial grid (side, side)
        if dist.shape[0] == dist.shape[1]:
            return dist.astype(np.float32), None, None
        return None, None, None

    def _tof_cube_centre(
        self,
        dist_mm: np.ndarray,
        sigma_mm: Optional[np.ndarray] = None,
        status: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Detect the cube in the ToF depth map and return its
        estimated centre in the ToF sensor frame (metres).
        Adapts dynamically to 4x4 or 8x8 grids.

        dist_mm must be a spatial grid (side, side) in millimetres.
        """
        if dist_mm is None or not isinstance(dist_mm, np.ndarray) or dist_mm.ndim != 2:
            return None

        # Match overlap_page reshape(res,res).T convention and transforms.
        dist_mm = dist_mm.T
        if sigma_mm is not None:
            sigma_mm = sigma_mm.T
        if status is not None:
            status = status.T
        if self._tof_rot_k:
            dist_mm = np.rot90(dist_mm, self._tof_rot_k)
            if sigma_mm is not None:
                sigma_mm = np.rot90(sigma_mm, self._tof_rot_k)
            if status is not None:
                status = np.rot90(status, self._tof_rot_k)
        if self._tof_flip_x:
            dist_mm = np.flipud(dist_mm)
            if sigma_mm is not None:
                sigma_mm = np.flipud(sigma_mm)
            if status is not None:
                status = np.flipud(status)
        if self._tof_flip_y:
            dist_mm = np.fliplr(dist_mm)
            if sigma_mm is not None:
                sigma_mm = np.fliplr(sigma_mm)
            if status is not None:
                status = np.fliplr(status)

        rows, cols = dist_mm.shape

        base_valid = (dist_mm > 0) & (dist_mm < 8000)
        if status is not None and isinstance(status, np.ndarray) and status.shape == dist_mm.shape:
            try:
                base_valid &= np.isin(status, list(_VALID_TOF_STATUS))
            except Exception:
                pass
        valid = dist_mm[base_valid]
        if len(valid) == 0:
            return None

        bg_mm = float(np.median(valid))

        # Adaptive step threshold: require the foreground to be sufficiently
        # closer than the background median. Use sigma if available.
        step_mm = 40.0
        if sigma_mm is not None and isinstance(sigma_mm, np.ndarray) and sigma_mm.shape == dist_mm.shape:
            sig_valid = sigma_mm[base_valid].astype(np.float32)
            if sig_valid.size:
                step_mm = float(max(25.0, 2.5 * np.median(np.clip(sig_valid, 1.0, 500.0))))

        delta = (bg_mm - dist_mm).astype(np.float32)  # positive for nearer-than-bg
        mask0 = base_valid & (delta > step_mm)

        if mask0.sum() < 2:
            return None

        # Keep only the strongest connected component so isolated noisy zones
        # don't pull the centroid.
        mask = self._largest_component(mask0, delta)
        if mask is None or mask.sum() < 2:
            return None

        invsig2 = None
        if sigma_mm is not None and isinstance(sigma_mm, np.ndarray) and sigma_mm.shape == dist_mm.shape:
            sig = np.clip(sigma_mm.astype(np.float32), 1.0, 500.0)
            invsig2 = 1.0 / (sig * sig)

        # 2D centroid in the ToF grid (continuous r,c), weighted by step size.
        w = delta.copy()
        w[~mask] = 0.0
        if invsig2 is not None:
            w *= invsig2
        w_sum = float(w.sum())
        if w_sum <= 0.0:
            return None
        rr, cc = np.indices(dist_mm.shape, dtype=np.float32)
        r_c = float((rr * w).sum() / w_sum)
        c_c = float((cc * w).sum() / w_sum)

        # Representative distance for the object: weighted average over the component.
        d_mm = float((dist_mm.astype(np.float32) * w).sum() / w_sum)

        fov_rad = np.radians(self.tof_fov_deg)
        # BUG FIX: use separate angle-per-zone for each axis (rows ≠ cols case)
        apz_h = fov_rad / float(cols)
        apz_v = fov_rad / float(rows)

        c_offset = (cols - 1) / 2.0
        r_offset = (rows - 1) / 2.0

        # Use centroid ray (zone center angles) + representative range.
        # Interpret distance as SLANT range along the ray.
        d = d_mm / 1000.0
        ah = (c_c - c_offset) * apz_h
        av = (r_c - r_offset) * apz_v
        tan_ah, tan_av = np.tan(ah), np.tan(av)
        norm = float(np.sqrt(tan_ah**2 + tan_av**2 + 1.0))
        x = d * tan_ah / norm
        y = d * tan_av / norm
        z = d / norm
        return np.array([x, y, z], dtype=np.float64)

    def _largest_component(self, mask: np.ndarray, score: np.ndarray) -> Optional[np.ndarray]:
        """Return the best 4-connected component of `mask`.

        Picks the component with the largest summed `score` (typically delta depth).
        """
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

    # ── Save helpers ───────────────────────────────────────────────────────

    def save_result(self, result: CalibResult, directory: str):
        """Save calibration result to *directory* as .npz + readable .txt."""
        import os

        ts = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(directory, f"calib_{ts}")

        # NumPy archive
        save_dict = dict(
            camera_matrix=result.camera_matrix,
            dist_coeffs=result.dist_coeffs,
            rms_error=np.array([result.rms_error]),
            n_frames=np.array([result.n_frames]),
            tof_fov_deg=np.array([self.tof_fov_deg]),
            camera_model=np.array([
                1 if str(getattr(result, 'camera_model', 'pinhole')).lower() == 'fisheye' else 0
            ], dtype=np.int32),
        )
        if result.R_tof_to_rgb is not None:
            save_dict["R_tof_to_rgb"] = result.R_tof_to_rgb
            save_dict["t_tof_to_rgb"] = result.t_tof_to_rgb
            save_dict["extrinsic_rms_mm"] = np.array([result.extrinsic_rms_mm])

        np.savez(base + ".npz", **save_dict)

        # Human-readable summary
        with open(base + ".txt", "w", encoding="utf-8") as f:
            f.write(f"Calibration result — {ts}\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Frames used     : {result.n_frames}\n")
            f.write(f"RMS reprojection: {result.rms_error:.4f} px\n\n")
            f.write("ToF FoV (deg)   : %.2f\n\n" % self.tof_fov_deg)
            f.write("Camera matrix (K):\n")
            for row in result.camera_matrix:
                f.write("  " + "  ".join(f"{v:12.4f}" for v in row) + "\n")

            d = np.asarray(result.dist_coeffs, dtype=np.float64).reshape(-1)
            is_fisheye = (str(getattr(result, 'camera_model', 'pinhole')).lower() == 'fisheye') or (d.size == 4)
            if is_fisheye:
                f.write("\nDistortion coefficients (fisheye k1 k2 k3 k4):\n")
                f.write("  " + "  ".join(f"{v:10.6f}" for v in d[:4]) + "\n")
            else:
                f.write("\nDistortion coefficients (k1 k2 p1 p2 k3):\n")
                f.write("  " + "  ".join(f"{v:10.6f}" for v in d[:5]) + "\n")
            if result.R_tof_to_rgb is not None:
                f.write(f"\nExtrinsic RMS   : {result.extrinsic_rms_mm:.2f} mm\n")
                f.write("R (ToF → RGB):\n")
                for row in result.R_tof_to_rgb:
                    f.write("  " + "  ".join(f"{v:10.6f}" for v in row) + "\n")
                f.write("t (ToF → RGB) [m]:\n")
                f.write("  " + "  ".join(f"{v:10.6f}" for v in result.t_tof_to_rgb) + "\n")

        return base