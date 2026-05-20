"""calibration.py — Calibration engine used by the RV1106G3 PC visualizer.

This module replaces the legacy engine (now in `calibration_old.py`) and
implements the newer plane-based extrinsic calibration procedure (ported from
`newcal.py`) in a framework-friendly, streaming form.

Design goals
------------
- No UI dependencies (Qt lives in `calibration_page.py`).
- Streaming API compatible with `CalibrationPage`:
    - `CalibrationSession.start()` / `stop()` / `finalize()`
    - `CalibrationSession.process_frame(jpeg, tof, w, h) -> dict`
- Minimal external dependencies: numpy + opencv.

What it computes
----------------
- Extrinsics ToF→RGB from many observations of the same planar target.
  Each accepted frame yields:
    - plane in ToF frame (from 8×8 depth grid)
    - plane in RGB camera frame (from AprilTag/ArUco marker pose)
  Then solve:
    - rotation via Kabsch on plane normals
    - translation via robust least-squares on plane offsets

Notes
-----
- Camera intrinsics (K, dist) are expected to be provided by the app config.
- Distances are handled in millimetres internally; outputs are metres.

"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Any

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


# VL53L8CH target status codes considered valid.
_VALID_TOF_STATUS = {5, 6, 9, 10}


@dataclass
class CalibResult:
    # Keep fields that the UI expects.
    rms_error: float
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rvecs: list
    tvecs: list
    n_frames: int
    timestamp_s: float
    camera_model: str = "pinhole"  # 'pinhole' | 'fisheye'

    R_tof_to_rgb: Optional[np.ndarray] = None  # (3,3)
    t_tof_to_rgb: Optional[np.ndarray] = None  # (3,) metres
    extrinsic_rms_mm: Optional[float] = None


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        return v.astype(np.float64)
    return (v / n).astype(np.float64)


def _fit_plane_svd(points_mm: np.ndarray, weights: Optional[np.ndarray] = None) -> Optional[dict]:
    if points_mm is None or len(points_mm) < 4:
        return None

    pts = np.asarray(points_mm, dtype=np.float64)
    if weights is not None:
        w = np.asarray(weights, dtype=np.float64).reshape(-1)
        if len(w) != len(pts):
            w = None
    else:
        w = None

    if w is None:
        centroid = pts.mean(axis=0)
        A = pts - centroid
    else:
        w = w / (w.sum() + 1e-12)
        centroid = (w[:, None] * pts).sum(axis=0)
        A = np.sqrt(w[:, None]) * (pts - centroid)

    _, _, vt = np.linalg.svd(A, full_matrices=False)
    normal = _unit(vt[-1])
    d = -float(np.dot(normal, centroid))
    if d < 0:
        normal, d = -normal, -d

    return {"normal": normal, "d": d, "centroid": centroid}


def _fit_plane_ransac(points_mm: np.ndarray, threshold_mm: float = 25.0, n_iter: int = 300) -> Optional[dict]:
    """Small-point-set RANSAC plane fit (good for ToF 8×8)."""
    pts = np.asarray(points_mm, dtype=np.float64)
    if len(pts) < 4:
        return None

    rng = np.random.default_rng()
    best_inliers: Optional[np.ndarray] = None
    best_cnt = -1

    for _ in range(int(max(1, n_iter))):
        idx = rng.choice(len(pts), size=3, replace=False)
        p1, p2, p3 = pts[idx]
        n = np.cross(p2 - p1, p3 - p1)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        d = -float(np.dot(n, p1))

        dist = np.abs(pts @ n + d)
        inl = dist < float(threshold_mm)
        cnt = int(inl.sum())
        if cnt > best_cnt:
            best_cnt = cnt
            best_inliers = inl

    if best_inliers is None or best_cnt < 4:
        return None

    fitted = _fit_plane_svd(pts[best_inliers])
    if fitted is None:
        return None

    # Re-evaluate inliers with refined plane
    n = fitted["normal"]
    d = float(fitted["d"])
    dist = np.abs(pts @ n + d)
    inl = dist < float(threshold_mm)
    fitted["inliers"] = np.where(inl)[0]
    return fitted


def _tof_to_pointcloud_mm(dist_grid_mm: np.ndarray, valid: np.ndarray, fov_deg: float) -> np.ndarray:
    """Convert an 8×8 depth grid to an (N,3) point cloud in ToF frame.

    Coordinate convention: Z forward, X right, Y down (camera-like).
    """
    dist = np.asarray(dist_grid_mm, dtype=np.float64)
    valid = np.asarray(valid, dtype=bool)

    if dist.shape != (8, 8) or valid.shape != (8, 8):
        return np.empty((0, 3), dtype=np.float64)

    half = np.radians(float(fov_deg) / 2.0)
    rows, cols = np.meshgrid(np.arange(8), np.arange(8), indexing="ij")
    ax = (cols + 0.5 - 4.0) / 4.0 * half
    ay = (rows + 0.5 - 4.0) / 4.0 * half

    dx = np.tan(ax)
    dy = np.tan(ay)

    mask = valid & np.isfinite(dist) & (dist > 0)
    if not np.any(mask):
        return np.empty((0, 3), dtype=np.float64)

    z = dist[mask]
    x = z * dx[mask]
    y = z * dy[mask]
    return np.column_stack([x, y, z]).astype(np.float64)


def _apply_tof_view_transform(grid: np.ndarray, rot_k: int, flip_x: bool, flip_y: bool) -> np.ndarray:
    """Apply view-style transform to a 2D grid.

    rot_k: number of 90° rotations clockwise.
    flip_x: horizontal flip.
    flip_y: vertical flip.
    """
    g = np.asarray(grid)
    k = int(rot_k) % 4
    if k:
        # np.rot90 is CCW; clockwise = -k
        g = np.rot90(g, -k)
    if flip_x:
        g = np.fliplr(g)
    if flip_y:
        g = np.flipud(g)
    return g


def _tof_primary_grid(tof: Any) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Convert ToF payload to a single distance grid, plus optional sigma/status grids."""
    if tof is None:
        return None, None, None

    if isinstance(tof, np.ndarray):
        dist = tof
        if dist.ndim == 2:
            r0, c0 = dist.shape
            side = int(np.sqrt(r0))
            if side * side == r0 and c0 != side:
                dist = dist[:, 0].reshape(side, side)
        if dist.ndim != 2:
            return None, None, None
        return dist.astype(np.float32), None, None

    if not hasattr(tof, "distance_mm"):
        return None, None, None

    dist = np.asarray(getattr(tof, "distance_mm"))
    sigma = np.asarray(getattr(tof, "sigma_mm", None)) if hasattr(tof, "sigma_mm") else None
    status = np.asarray(getattr(tof, "status", None)) if hasattr(tof, "status") else None

    if dist.ndim != 2:
        return None, None, None

    r0, c0 = dist.shape
    side = int(np.sqrt(r0))
    if side * side == r0 and c0 != side:
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

    if dist.shape[0] == dist.shape[1]:
        return dist.astype(np.float32), None, None

    return None, None, None


def _kabsch_rotation(P: np.ndarray, Q: np.ndarray) -> np.ndarray:
    """Return R minimizing ||R P - Q|| for corresponding vectors."""
    P = np.asarray(P, dtype=np.float64)
    Q = np.asarray(Q, dtype=np.float64)
    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R


def _solve_extrinsics_from_planes(plane_pairs: list[dict]) -> Tuple[np.ndarray, np.ndarray, float]:
    """Solve ToF→RGB extrinsics from plane correspondences.

    Returns (R(3,3), t_m(3,), rms_mm).
    """
    normals_tof = np.array([_unit(p["plane_tof"]["normal"]) for p in plane_pairs], dtype=np.float64)
    normals_rgb = np.array([_unit(p["plane_rgb"]["normal"]) for p in plane_pairs], dtype=np.float64)
    d_tof = np.array([float(p["plane_tof"]["d"]) for p in plane_pairs], dtype=np.float64)
    d_rgb = np.array([float(p["plane_rgb"]["d"]) for p in plane_pairs], dtype=np.float64)

    if len(normals_tof) < 3:
        raise RuntimeError("Need at least 3 plane pairs")

    # Initial rotation
    R = _kabsch_rotation(normals_tof, normals_rgb)

    # Resolve per-pair normal sign ambiguity by flipping RGB planes when needed.
    # (Flipping a plane: n,d -> -n,-d represents the same set.)
    n_rot = (R @ normals_tof.T).T
    flip = (np.einsum("ij,ij->i", n_rot, normals_rgb) < 0)
    if np.any(flip):
        normals_rgb = normals_rgb.copy()
        d_rgb = d_rgb.copy()
        normals_rgb[flip] *= -1.0
        d_rgb[flip] *= -1.0
        R = _kabsch_rotation(normals_tof, normals_rgb)

    # Translation from: n_rgb^T t = d_tof - d_rgb  (mm)
    A = normals_rgb
    b = (d_tof - d_rgb)

    # Robust iteratively reweighted LSQ using MAD
    t = np.linalg.lstsq(A, b, rcond=None)[0]
    for _ in range(3):
        r = (A @ t - b)
        med = np.median(r)
        mad = np.median(np.abs(r - med)) + 1e-9
        inl = np.abs(r - med) < (3.0 * 1.4826 * mad + 5.0)
        if int(inl.sum()) < 3:
            break
        t = np.linalg.lstsq(A[inl], b[inl], rcond=None)[0]

    resid = A @ t - b
    rms_mm = float(np.sqrt(np.mean(resid ** 2)))

    # convert mm -> m
    t_m = (t / 1000.0).astype(np.float64)
    return R.astype(np.float64), t_m, rms_mm


def _build_aruco_detector(dict_id: int, marker_border_bits: int = 2):
    aruco = cv2.aruco
    aruco_dict = aruco.getPredefinedDictionary(int(dict_id))
    params = aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 73
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate = 0.01
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 50
    # Non-standard AprilTag border support (newcal procedure)
    if hasattr(params, "markerBorderBits"):
        try:
            params.markerBorderBits = int(marker_border_bits)
        except Exception:
            pass
    if hasattr(aruco, "ArucoDetector"):
        return aruco.ArucoDetector(aruco_dict, params)
    return (aruco_dict, params)


def _detect_markers(gray: np.ndarray, dict_id: int, marker_border_bits: int = 2):
    """Version-tolerant wrapper for ArUco marker detection."""
    aruco = cv2.aruco
    det = _build_aruco_detector(dict_id, marker_border_bits=marker_border_bits)
    if hasattr(det, "detectMarkers"):
        corners, ids, rejected = det.detectMarkers(gray)
        return corners, ids, rejected
    aruco_dict, params = det
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids, rejected


def _marker_obj_points_mm(marker_size_mm: float) -> np.ndarray:
    s = float(marker_size_mm) / 2.0
    # (top-left, top-right, bottom-right, bottom-left)
    return np.array(
        [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]], dtype=np.float64
    )


def detect_board_plane_rgb(
    bgr: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    dict_id: int,
    marker_size_mm: float,
    marker_border_bits: int = 2,
    min_markers: int = 1,
) -> Optional[dict]:
    """Detect markers in an RGB frame and fit the board plane in camera frame."""
    if cv2 is None:
        return None

    if bgr is None:
        return None

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr

    corners, ids, _ = _detect_markers(gray, dict_id=dict_id, marker_border_bits=marker_border_bits)
    if ids is None or len(ids) < int(min_markers):
        return None

    obj_local = _marker_obj_points_mm(marker_size_mm)

    pts_cam_all = []
    for img_pts in corners:
        ip = np.asarray(img_pts, dtype=np.float64).reshape(4, 2)
        pnp_flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(
            obj_local,
            ip,
            np.asarray(K, dtype=np.float64),
            np.asarray(dist, dtype=np.float64),
            flags=pnp_flag,
        )
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        pts_cam_all.append((R @ obj_local.T).T + tvec.reshape(1, 3))

    if len(pts_cam_all) < int(min_markers):
        return None

    pts_cam = np.vstack(pts_cam_all)
    plane = _fit_plane_svd(pts_cam)
    if plane is None:
        return None

    plane.update(
        {
            "points": pts_cam,
            "n_corners": int(len(pts_cam)),
            "n_markers": int(len(pts_cam_all)),
            "corners": corners,
            "ids": ids,
        }
    )
    return plane


class CalibrationSession:
    """Streaming session for plane-based ToF→RGB extrinsic calibration."""

    def __init__(
        self,
        aruco_dict_id: int,
        pattern: str = "aruco",
        marker_length_m: float = 0.18,
        # Unused but kept for compatibility with existing UI code
        charuco_squares_x: int = 7,
        charuco_squares_y: int = 5,
        charuco_square_length_m: float = 0.03,
        charuco_marker_length_m: float = 0.022,
        n_total: int = 60,
        top_k: int = 20,
        quality_threshold: float = 0.45,
        target_rms: float = 0.5,
        tof_fov_deg: float = 45.0,
        recalib_every: int = 3,
        min_frames_calib: int = 10,
        tof_rot_k: int = 0,
        tof_flip_x: bool = False,
        tof_flip_y: bool = False,
        board_offset_xy_mm: Tuple[float, float] = (0.0, 0.0),
        camera_model: str = "pinhole",
    ):
        self.aruco_dict_id = int(aruco_dict_id)
        self.pattern = str(pattern).lower().strip() or "aruco"
        self.marker_length_m = float(marker_length_m)

        self.n_total = int(n_total)
        self.top_k = int(top_k)
        self.quality_threshold = float(quality_threshold)
        self.target_rms = float(target_rms)
        self.tof_fov_deg = float(tof_fov_deg)

        self._tof_rot_k = int(tof_rot_k) % 4
        self._tof_flip_x = bool(tof_flip_x)
        self._tof_flip_y = bool(tof_flip_y)

        self.camera_model = str(camera_model or "pinhole").strip().lower()

        self._running = False
        self._plane_pairs: list[dict] = []

        self._K: Optional[np.ndarray] = None
        self._dist: Optional[np.ndarray] = None

    @property
    def is_running(self) -> bool:
        return bool(self._running)

    @property
    def n_frames(self) -> int:
        return int(len(self._plane_pairs))

    def start(self):
        # Start extrinsic acquisition; intrinsics must be provided via start_extrinsic.
        self._plane_pairs.clear()
        self._running = True

    def start_extrinsic(self, intrinsic: CalibResult):
        """Start acquisition with fixed camera intrinsics (from config)."""
        self._plane_pairs.clear()
        self._K = np.asarray(intrinsic.camera_matrix, dtype=np.float64)
        self._dist = np.asarray(intrinsic.dist_coeffs, dtype=np.float64).reshape(-1)
        self.camera_model = str(getattr(intrinsic, "camera_model", self.camera_model) or self.camera_model)
        self._running = True

    def stop(self):
        self._running = False

    def should_stop(self) -> bool:
        return len(self._plane_pairs) >= self.n_total

    def stop_reason(self) -> str:
        if len(self._plane_pairs) >= self.n_total:
            return "pool full"
        return "manual stop"

    def process_frame(self, jpeg: bytes, tof, w: int, h: int) -> dict:
        out = {
            "quality": 0.0,
            "rms": None,
            "n_frames": len(self._plane_pairs),
            "accepted": False,
            "annotated_bgr": None,
        }

        if not self._running or cv2 is None:
            return out

        if self._K is None or self._dist is None:
            return out

        if jpeg is None:
            return out

        arr = np.frombuffer(jpeg, dtype=np.uint8)
        bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if bgr is None:
            return out

        # ToF
        dist_grid, _, _ = _tof_primary_grid(tof)
        if dist_grid is None:
            valid_grid = np.zeros((8, 8), dtype=bool)
        else:
            dist_grid = _apply_tof_view_transform(dist_grid, self._tof_rot_k, self._tof_flip_x, self._tof_flip_y)
            try:
                valid_grid = (np.isfinite(dist_grid) & (dist_grid > 0)).astype(bool)
            except Exception:
                valid_grid = np.zeros((8, 8), dtype=bool)

        pc_tof = (
            _tof_to_pointcloud_mm(dist_grid, valid_grid, self.tof_fov_deg)
            if dist_grid is not None
            else np.empty((0, 3), dtype=np.float64)
        )

        plane_tof = None
        if len(pc_tof) >= 8:
            plane_tof = _fit_plane_ransac(pc_tof, threshold_mm=25.0, n_iter=300)

        # RGB plane (board)
        plane_rgb = detect_board_plane_rgb(
            bgr,
            self._K,
            self._dist,
            dict_id=self.aruco_dict_id,
            marker_size_mm=self.marker_length_m * 1000.0,
            marker_border_bits=2,
            min_markers=1,
        )

        # Annotate
        annot = bgr.copy()
        if plane_rgb is not None and "corners" in plane_rgb and "ids" in plane_rgb:
            try:
                cv2.aruco.drawDetectedMarkers(annot, plane_rgb["corners"], plane_rgb["ids"])
            except Exception:
                pass

        out["annotated_bgr"] = annot

        # Quality heuristic
        q = 0.0
        if plane_rgb is not None:
            q += 0.55
            n_m = int(plane_rgb.get("n_markers", 1))
            q += min(0.25, 0.08 * n_m)
        if plane_tof is not None:
            q += 0.20
        if dist_grid is not None:
            q += 0.10 * float(np.clip(np.sum(valid_grid) / 64.0, 0.0, 1.0))

        q = float(np.clip(q, 0.0, 1.0))
        out["quality"] = q

        if q >= self.quality_threshold and plane_rgb is not None and plane_tof is not None:
            if len(self._plane_pairs) < self.n_total:
                self._plane_pairs.append(
                    {
                        "ts_s": time.time(),
                        "plane_tof": {"normal": plane_tof["normal"], "d": float(plane_tof["d"])},
                        "plane_rgb": {"normal": plane_rgb["normal"], "d": float(plane_rgb["d"])},
                        "quality": q,
                    }
                )
                out["accepted"] = True
                out["n_frames"] = len(self._plane_pairs)

        return out

    def finalize(self) -> Optional[CalibResult]:
        if self._K is None or self._dist is None:
            return None
        if len(self._plane_pairs) < max(4, min(self.top_k, self.n_total)):
            return None

        best = sorted(self._plane_pairs, key=lambda r: float(r.get("quality", 0.0)), reverse=True)[: self.top_k]

        try:
            R, t_m, rms_mm = _solve_extrinsics_from_planes(best)
        except Exception:
            return None

        result = CalibResult(
            rms_error=0.0,
            camera_matrix=np.asarray(self._K, dtype=np.float64),
            dist_coeffs=np.asarray(self._dist, dtype=np.float64).reshape(-1),
            rvecs=[],
            tvecs=[],
            n_frames=len(best),
            timestamp_s=time.time(),
            camera_model=str(self.camera_model or "pinhole"),
            R_tof_to_rgb=R,
            t_tof_to_rgb=t_m,
            extrinsic_rms_mm=float(rms_mm),
        )
        return result

    def save_result(self, result: CalibResult, directory: str) -> str:
        os.makedirs(directory, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(directory, f"calib_{ts}")
        out_npz = base + ".npz"

        payload = {
            "camera_matrix": np.asarray(result.camera_matrix, dtype=np.float64),
            "dist_coeffs": np.asarray(result.dist_coeffs, dtype=np.float64).reshape(-1),
            "camera_model": np.array([1 if str(result.camera_model).lower() == "fisheye" else 0], dtype=np.int32),
            "tof_fov_deg": np.array([float(self.tof_fov_deg)], dtype=np.float64),
        }
        if result.R_tof_to_rgb is not None:
            payload["R_tof_to_rgb"] = np.asarray(result.R_tof_to_rgb, dtype=np.float64)
        if result.t_tof_to_rgb is not None:
            payload["t_tof_to_rgb"] = np.asarray(result.t_tof_to_rgb, dtype=np.float64)

        np.savez(out_npz, **payload)

        return out_npz
