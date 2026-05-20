#!/usr/bin/env python3
"""
Calibrazione estrinseca ToF 8x8 ↔ RealSense D435i.

"""

from __future__ import annotations
import json
import logging
import os
import sys
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from scipy.spatial.transform import Rotation

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Cartella sessione: contiene RS/rgb/, RS/depth/, TOF/tof.csv, RS/config.json
SESSION_DIR = r"C:\Users\besoz\Documents\GitHub\Depth_Dataset\data\TOF_CAL"

SESSION_CAMERA = "RS" #  usa RS/depth/ + intrinsics da RS/config.json

SESSION_TOF = "TOF_H" # usa TOF_H/tof.csv o TOF_V/tof.csv

VALID_TOF_STATUS = {5, 6, 9, 10} # Status ToF validi

TIMESTAMP_TOLERANCE_MS  = 10 # Tolleranza sincronizzazione timestamp coppie (ms)
TIMESTAMP_FILTER_MS     = 200 # Finestra di media temporale per sincronizzazione (ms)

# ---------------------------------------------------------------------------
# SOGLIE RANSAC — le due soglie più importanti da capire e regolare
# ---------------------------------------------------------------------------
#
#  Il RANSAC plane fitting funziona così:
#    1. Estrae N_ITER volte 3 punti casuali e calcola il piano passante per essi.
#    2. Conta quanti punti del cloud distano MENO di THRESHOLD dal piano
#       → questi sono gli "inlier".
#    3. Tiene il piano con più inlier, poi lo raffina con LSQ su tutti gli inlier.
#
#  THRESHOLD_TOF_MM  (per il cloud ToF, massimo 64 punti)
#    Il VL53L8CH ha una precisione tipica di ±15 mm a 1 m, ±30 mm a 2 m.
#    • Se la imposti troppo BASSA (es. 5 mm): molti punti validi vengono
#      esclusi come outlier → il piano viene trovato su pochi punti, risultato
#      instabile. Con rumore ToF da 15 mm, 5 mm è troppo restrittivo.
#    • Se la imposti troppo ALTA (es. 80 mm): anche superfici curve vengono
#      modellate come piane → piano sbagliato, calibrazione peggiore.
#    Regola pratica: metti THRESHOLD ≈ 2 × (sigma del sensore alla distanza
#    media di lavoro). A 1 m → 20-30 mm. A 2 m → 40-50 mm.
#
#  THRESHOLD_RS_MM  (per il cloud RS, decine/centinaia di migliaia di punti)
#    La D435i ha rumore ~0.1-0.3 % della distanza:
#      a 1 m → 1-3 mm,  a 2 m → 2-6 mm.
#    • Troppo BASSA (es. 1 mm): RANSAC fallisce sulle zone dove la profondità
#      è leggermente curva o ai bordi.
#    • Troppo ALTA (es. 50 mm): include punti di piani diversi → normale
#      errata, angoli di calibrazione sbagliati.
#    Regola pratica: 5-10 mm è buono per la maggior parte delle scene.
#    Aumenta a 15-20 mm se la scena non è perfettamente piatta (pareti
#    intonacate, pavimenti con giunto).
#
RANSAC_THRESHOLD_TOF_MM = 20.0   # vedi spiegazione sopra
RANSAC_THRESHOLD_RS_MM  = 20.0   # (non più usata: il piano RS arriva dal ChArUco)
RANSAC_N_ITER           = 1000    # più iterazioni = più affidabile ma più lento

# Numero massimo di punti RS da usare per il RANSAC (non più usato in questa pipeline)
RANSAC_SUBSAMPLE_RS = 5000

# ---------------------------------------------------------------------------
# APRILTAG BOARD — il piano RS è definito dalla pose dei marker rilevati
# nell'RGB. Il board ha marker AprilTag 36h11 con 2 bit di bordo (non
# standard OpenCV → va impostato markerBorderBits=2 nel detector).
#
# Pipeline: detectMarkers → solvePnP per ogni marker → tutti gli angoli 3D
# vengono concatenati e si fitta il piano via SVD. Non serve conoscere
# l'esatto layout del board (numero di celle, dimensione del checker),
# basta la dimensione fisica del singolo marker.
#
# Misure da prendere col calibro:
#   CHARUCO_MARKER_MM  = lato del quadrato NERO esterno del marker (bordo
#                         da 2 bit incluso), come lo "vede" detectMarkers.
# ---------------------------------------------------------------------------
CHARUCO_DICT          = cv2.aruco.DICT_APRILTAG_36h11
CHARUCO_SQUARES_X     = 13      # solo metadato (per riferimento, non usato)
CHARUCO_SQUARES_Y     = 13      # solo metadato
CHARUCO_MARKER_MM     = 22.3   # ← MISURA TU: lato esterno del quadrato nero del marker
CHARUCO_MIN_MARKERS   = 3       # minimo n. di marker rilevati per accettare il frame
MARKER_BORDER_BITS    = 2       # bit di bordo nero attorno al codice (default OpenCV=1)

# Minimo numero di coppie di piani per fare la calibrazione
MIN_PLANE_PAIRS = 50

AUTO_EXCLUDE_DIST_MM  = 150.0  # soglia differenza distanze piani [mm] — None per off

# ---------------------------------------------------------------------------
# ALGORITMO DI CALIBRAZIONE E METODO DI FITTING
# ---------------------------------------------------------------------------
#
#  SURFACE_FIT_METHOD : metodo per fittare la superficie su ogni point cloud.
#    "piano"       → fit lineare WLS sugli inlier RANSAC (default, più veloce)
#    "polinomiale" → Z = aX²+bXY+cY²+dX+eY+f, normale = gradiente al centroide
#    "esponenziale"→ Z = exp(a+bX+cY), fit in log-spazio
#    "biharmonico" → thin-plate spline via scipy.interpolate.RBFInterpolator
#
SURFACE_FIT_METHOD = "polinomiale"    # "piano" | "polinomiale" | "esponenziale" | "biharmonico"

# ---------------------------------------------------------------------------
# PESO PIXEL ToF — pesi gaussiani per il fit del piano ToF
# ---------------------------------------------------------------------------
#
#  I pixel centrali del ToF sono più affidabili per due motivi:
#    1. Minor distorsione angolare (punto quasi-coassiale con l'ottica).
#    2. Il VL53L8CH tende ad avere maggiore crosstalk ai bordi del FoV.
#
#  TOF_PLANE_WEIGHT_SIGMA controlla quanto velocemente i pesi decadono
#  allontanandosi dal centro della griglia 8×8 (centro = pixel 3.5,3.5):
#
#    sigma = 1.0 → bordo pesa ~2% del centro  (molto selettivo)
#    sigma = 2.0 → bordo pesa ~17% del centro (default, bilanciato)
#    sigma = 3.0 → bordo pesa ~47% del centro (quasi-uniforme)
#    sigma = None o molto grande → pesi uniformi (comportamento originale)
#
TOF_PLANE_WEIGHT_SIGMA = None   # None = pesi uniformi
TOF_VALID_POINTS = 64
TOF_INLIERS = 64

# =============================================================================
# 1. CARICAMENTO DATI
# =============================================================================

def load_rs_config(session_dir: str):
    """Carica intrinsics RS RGB + coefficienti di distorsione da RS/config.json."""
    path = os.path.join(session_dir, "config.json")
    if not os.path.exists(path):
        raise FileNotFoundError(f"RS/config.json non trovato in {session_dir}")
    with open(path) as f:
        cfg = json.load(f)

    ri = cfg.get("RS_int", cfg.get("rgb_int", {}))
    if not ri:
        raise KeyError("Chiave 'RS_int' non trovata in config.json")

    K = np.array([[ri["fx"], 0,         ri["ppx"]],
                  [0,        ri["fy"],   ri["ppy"]],
                  [0,        0,          1        ]], dtype=np.float64)

    coeffs = ri.get("coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
    dist = np.array(coeffs, dtype=np.float64).reshape(-1)

    acq = cfg.get("acquisition", {})
    W, H = acq["rgb_width"], acq["rgb_height"]

    depth_scale_mm = float(cfg.get("depth_scale_mm", cfg.get("depth_scale", 1.0)))
    if depth_scale_mm < 0.1:
        depth_scale_mm *= 1000.0

    return K, dist, depth_scale_mm, W, H


def load_rs_depth_frames(session_dir: str):
    """Carica [(timestamp_ms, path)] dei frame depth RS."""
    dep_dir = os.path.join(session_dir, "RS", "depth")
    if not os.path.exists(dep_dir):
        raise FileNotFoundError(f"RS/depth non trovata in {session_dir}")

    entries = []
    for fname in os.listdir(dep_dir):
        stem, ext = os.path.splitext(fname)
        if ext.lower() not in {".png", ".tiff", ".tif"}:
            continue
        for pfx in ("depth_", ""):
            if stem.lower().startswith(pfx):
                stem = stem[len(pfx):]
                break
        try:
            entries.append((float(stem) * 1000.0, os.path.join(dep_dir, fname)))
        except ValueError:
            pass
    entries.sort()
    return entries


def load_rs_rgb_frames(session_dir: str) -> dict:
    """Carica {timestamp_ms → path} dei frame RGB RS da RS/rgb/."""
    rgb_dir = os.path.join(session_dir, "RS", "rgb")
    if not os.path.exists(rgb_dir):
        return {}
    lookup = {}
    for fname in os.listdir(rgb_dir):
        stem, ext = os.path.splitext(fname)
        if ext.lower() not in {".jpg", ".jpeg", ".png"}:
            continue
        for pfx in ("RS_", ""):
            if stem.startswith(pfx):
                stem = stem[len(pfx):]
                break
        try:
            lookup[float(stem) * 1000.0] = os.path.join(rgb_dir, fname)
        except ValueError:
            pass
    return lookup


def load_rs_rgb_frames_list(session_dir: str):
    """Restituisce [(timestamp_ms, path)] dei frame RGB RS, ordinati per timestamp.
    Versione lista per essere passata a synchronize_frames."""
    lookup = load_rs_rgb_frames(session_dir)
    return sorted(lookup.items(), key=lambda x: x[0])


def load_tof_frames(session_dir: str):
    """Carica [(timestamp_ms, dist_8x8, valid_8x8)] da {SESSION_TOF}/tof.csv."""

    path = os.path.join(session_dir, SESSION_TOF, "tof.csv")
    if not os.path.exists(path):
        raise FileNotFoundError(f"{SESSION_TOF}/tof.csv non trovato in {session_dir}")

    df = pd.read_csv(path)
    val_cols = [f"Val_{i}" for i in range(64)]

    frames = []
    for ts_raw, grp in df.groupby("Timestamp_MCU", sort=False):
        row_d  = grp[grp["Sensor_Type"] == "TOF8_D"]
        row_st = grp[grp["Sensor_Type"] == "TOF8_ST"]
        if row_d.empty or row_st.empty:
            continue
        d  = pd.to_numeric(row_d[val_cols].iloc[0],  errors="coerce").values.astype(float)
        st = pd.to_numeric(row_st[val_cols].iloc[0], errors="coerce").values.astype(float)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            valid = np.isin(st.astype(int), list(VALID_TOF_STATUS))
        dist_8x8  = np.where(valid, d,  np.nan).reshape(8, 8)
        valid_8x8 = valid.reshape(8, 8)
        frames.append((int(ts_raw) / 1000.0, dist_8x8, valid_8x8))

    frames.sort(key=lambda x: x[0])
    return frames


def synchronize_frames(tof_frames, rs_frames, tol_ms=TIMESTAMP_TOLERANCE_MS):
    """Abbina ogni frame ToF al frame RS più vicino (esclusivo, |dt| < tol_ms)."""
    rs_times = np.array([t for t, _ in rs_frames])
    pairs, used_rs = [], set()
    for tof_entry in tof_frames:
        diffs = np.abs(rs_times - tof_entry[0])
        idx   = int(np.argmin(diffs))
        if diffs[idx] < tol_ms and idx not in used_rs:
            used_rs.add(idx)
            pairs.append((tof_entry, rs_frames[idx]))
    return pairs

def synchronize_and_average_frames(tof_frames, rs_frames, window_ms=TIMESTAMP_FILTER_MS):
    """
    Raggruppa i frame ToF e RS in finestre temporali di `window_ms` 
    e ne restituisce la media (sia per i timestamp che per i dati).
    """
    # Assicuriamoci che i frame siano ordinati per timestamp
    tof_frames = sorted(tof_frames, key=lambda x: x[0])
    rs_frames = sorted(rs_frames, key=lambda x: x[0])
    
    pairs = []
    i_tof, i_rs = 0, 0
    
    while i_tof < len(tof_frames):
        start_t = tof_frames[i_tof][0]
        end_t = start_t + window_ms
        
        # Raccogliamo TUTTI i frame ToF che cadono in questa finestra
        group_tof = []
        while i_tof < len(tof_frames) and tof_frames[i_tof][0] <= end_t:
            group_tof.append(tof_frames[i_tof])
            i_tof += 1
            
        # Raccogliamo TUTTI i frame RS che cadono in questa stessa finestra
        group_rs = []
        while i_rs < len(rs_frames) and rs_frames[i_rs][0] <= end_t:
            if rs_frames[i_rs][0] >= start_t:
                group_rs.append(rs_frames[i_rs])
            i_rs += 1
            
        # Se abbiamo dati validi da ENTRAMBI i sensori nel time-bin, calcoliamo la media
        if len(group_tof) > 0 and len(group_rs) > 0:
            avg_tof = _average_sensor_data(group_tof)
            avg_rs  = _average_sensor_data(group_rs)
            pairs.append((avg_tof, avg_rs))
        else:
            print(f"Attenzione: finestra {start_t:.3f} - {end_t:.3f} ha solo {len(group_tof)} frame ToF e {len(group_rs)} frame RS.")
            
    return pairs


def _average_sensor_data(frame_group):
    """
    Calcola il timestamp medio e fa la media elemento per elemento.
    Preserva magicamente l'esatta struttura della tupla originale
    per non rompere le funzioni a valle.
    """
    # 1. Media del timestamp (il primo elemento di ogni frame)
    avg_t = np.mean([f[0] for f in frame_group])
    
    # 2. Estraiamo tutti gli altri dati: f[1:] prende tutto dal secondo elemento in poi.
    # Se il frame era (timestamp, dist_8x8, valid_8x8), datas contiene [(dist, valid), ...]
    datas = [f[1:] for f in frame_group]
    
    avg_data = []
    # Usiamo zip per iterare colonna per colonna (tutte le dist assieme, tutte le valid assieme)
    for col in zip(*datas):
        first_elem = col[0]
        
        # Caso array numerico (es. dist_8x8 o depth_img)
        if isinstance(first_elem, np.ndarray) and np.issubdtype(first_elem.dtype, np.number):
            avg_data.append(np.mean(col, axis=0))
            
        # Caso array booleano (es. valid_8x8) -> RESTA BOOLEANO
        elif isinstance(first_elem, np.ndarray) and first_elem.dtype == bool:
            stacked = np.stack(col)
            # Un pixel è valido solo se lo era in TUTTI i frame mediati
            avg_data.append(np.all(stacked, axis=0))
            
        # Caso dizionario
        elif isinstance(first_elem, dict):
            avg_dict = {}
            for k in first_elem.keys():
                v_first = first_elem[k]
                if isinstance(v_first, np.ndarray) and np.issubdtype(v_first.dtype, np.number):
                    avg_dict[k] = np.mean([d[k] for d in col], axis=0)
                elif isinstance(v_first, np.ndarray) and v_first.dtype == bool:
                    avg_dict[k] = np.all(np.stack([d[k] for d in col]), axis=0)
                else:
                    avg_dict[k] = col[-1][k]
            avg_data.append(avg_dict)
            
        # Fallback (numeri interi, stringhe, ecc. -> prendiamo l'ultimo valore valido)
        else:
            avg_data.append(col[-1])
            
    # 3. Ricostruiamo e restituiamo la tupla piatta: (timestamp_medio, dist_media, valid_medio, ...)
    return (avg_t, *avg_data)

# =============================================================================
# 2. POINT CLOUD
# =============================================================================

def tof_to_pointcloud(dist_8x8: np.ndarray, valid_8x8: np.ndarray,
                      fov_h_deg: float, fov_v_deg: float,
                      weight_sigma: float | None = None):
    """8×8 distanze ToF → cloud 3D nel frame ToF (Z avanti, X destra, Y basso).
    dist_8x8 in mm, già convertita in distanza perpendicolare (Z-depth).

    Se weight_sigma è fornito restituisce (points (N,3), weights (N,)) dove
    weights sono pesi gaussiani centrati sul pixel centrale — pixel centrali
    hanno peso maggiore. Altrimenti restituisce solo points (N,3).
    """

    half_h = np.radians(fov_h_deg / 2.0)
    half_v = np.radians(fov_v_deg / 2.0)

    rows, cols = np.meshgrid(np.arange(8), np.arange(8), indexing="ij")
    angle_x = (cols + 0.5 - 4.0) / 4.0 * half_h
    angle_y = (rows + 0.5 - 4.0) / 4.0 * half_v
    dx = np.tan(angle_x)
    dy = np.tan(angle_y)

    mask = valid_8x8 & np.isfinite(dist_8x8) & (dist_8x8 > 0)
    d_m = dist_8x8[mask]
    x = d_m * dx[mask]   # Z * tan(angle_x)
    y = d_m * dy[mask]   # Z * tan(angle_y)
    z = d_m              # Z-depth direttamente
    points = np.column_stack([x, y, z]) if len(d_m) else np.empty((0, 3))

    mask_1d = mask.flatten()

    if weight_sigma is None:
        return points, mask_1d

    # Pesi gaussiani: w(r,c) = exp(-((r-3.5)²+(c-3.5)²) / (2σ²))
    r2 = (rows - 3.5) ** 2 + (cols - 3.5) ** 2
    w_grid = np.exp(-r2 / (2.0 * weight_sigma ** 2))
    weights = w_grid[mask]
    weights = weights / weights.sum()   # normalizza → somma 1
    return points, weights, mask_1d


def depth_to_pointcloud(depth_img: np.ndarray, K: np.ndarray,
                        depth_scale_mm: float = 1.0,
                        max_depth_mm: float = 8000.0) -> np.ndarray:
    """Depth image RS → cloud 3D nel frame RS (mm). Restituisce (N, 3)."""
    h, w = depth_img.shape
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    z = depth_img.astype(np.float64) * depth_scale_mm
    mask = (z > 0) & (z < max_depth_mm)
    z_v = z[mask]
    x_v = (us[mask] - K[0, 2]) * z_v / K[0, 0]
    y_v = (vs[mask] - K[1, 2]) * z_v / K[1, 1]
    return np.column_stack([x_v, y_v, z_v])


# =============================================================================
# 2b. RILEVAMENTO CHARUCO  (piano RS dalla pose del board nell'immagine RGB)
# =============================================================================

def _build_aruco_detector():
    """Costruisce un ArucoDetector con parametri robusti per marker piccoli/sfocati,
    impostando markerBorderBits per supportare board con bordo non-standard.
    Nota: CORNER_REFINE_APRILTAG dà risultati peggiori su questo board (è tarato
    per AprilTag standard con 1 bit di bordo), SUBPIX è più affidabile."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(CHARUCO_DICT)
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin     = 3
    p.adaptiveThreshWinSizeMax     = 73
    p.adaptiveThreshWinSizeStep    = 4
    p.minMarkerPerimeterRate       = 0.01
    p.maxMarkerPerimeterRate       = 4.0
    p.polygonalApproxAccuracyRate  = 0.05
    p.cornerRefinementMethod       = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize      = 5
    p.cornerRefinementMaxIterations = 50
    p.markerBorderBits             = MARKER_BORDER_BITS
    return cv2.aruco.ArucoDetector(aruco_dict, p)


def _marker_obj_points(marker_mm: float) -> np.ndarray:
    """Quattro angoli di un marker nel proprio sistema di riferimento locale,
    nello stesso ordine prodotto da cv2.aruco.detectMarkers
    (top-left, top-right, bottom-right, bottom-left)."""
    s = marker_mm / 2.0
    return np.array([[-s,  s, 0.0],
                     [ s,  s, 0.0],
                     [ s, -s, 0.0],
                     [-s, -s, 0.0]], dtype=np.float64)


# Kernel di sharpening leggero (unsharp mask 3×3): rinforza i bordi dei marker
# senza saturare. Empiricamente raddoppia il numero di marker rilevati su frame
# con illuminazione debole o leggero blur (vedi commento sopra).
_SHARPEN_KERNEL = np.array([[ 0, -1,  0],
                             [-1,  5, -1],
                             [ 0, -1,  0]], dtype=np.float32)


def detect_charuco_plane(rgb_img: np.ndarray,
                         K: np.ndarray,
                         dist: np.ndarray) -> dict | None:
    """Rileva i marker AprilTag nell'RGB e stima il piano del board nel frame
    della camera RS aggregando le pose dei singoli marker (solvePnP per ognuno).

    Strategia: ogni marker ha 4 angoli con coordinate 3D note nel proprio sistema
    locale; solvePnP fornisce R, t per ciascun marker → un'osservazione del piano
    (normale = R[:,2], punto = t). Si trasportano tutti i 4 angoli di tutti i
    marker nel frame camera e si fitta un piano via SVD sull'insieme.

    Output (mm, sistema RS): {normal, d, centroid, points, n_corners} o None.
    """
    if rgb_img is None:
        return None
    gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY) if rgb_img.ndim == 3 else rgb_img
    # Sharpening: empiricamente recupera frame leggermente sfocati e aumenta
    # il numero di marker rilevati anche sui frame già "buoni".
    gray = cv2.filter2D(gray, -1, _SHARPEN_KERNEL)

    detector = _build_aruco_detector()
    m_corners, m_ids, _ = detector.detectMarkers(gray)
    if m_ids is None or len(m_ids) < CHARUCO_MIN_MARKERS:
        return None

    obj_local = _marker_obj_points(CHARUCO_MARKER_MM)

    pts_cam_all = []
    for img_pts in m_corners:
        ip = img_pts.reshape(4, 2).astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj_local, ip, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        pts_cam_all.append((R @ obj_local.T).T + tvec.reshape(1, 3))

    if len(pts_cam_all) < CHARUCO_MIN_MARKERS:
        return None

    pts_cam = np.vstack(pts_cam_all)            # (4*N_markers, 3) mm

    # Fit del piano (tutti i marker sono coplanari) via SVD
    centroid = pts_cam.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts_cam - centroid, full_matrices=False)
    normal   = Vt[-1].astype(np.float64)
    d        = -float(np.dot(normal, centroid))
    if d < 0:
        normal, d = -normal, -d

    return {
        "normal":    normal,
        "d":         d,
        "centroid":  centroid,
        "points":    pts_cam,
        "n_corners": int(len(pts_cam)),
        "n_markers": int(len(pts_cam_all)),
    }


# =============================================================================
# 3. RANSAC VETTORIZZATO  (CPU, con subsampling per cloud grandi)
# =============================================================================

def _ransac_batch_cpu(pts_sub: np.ndarray, n_iter: int,
                      threshold_mm: float,
                      weights: np.ndarray | None = None) -> tuple[np.ndarray, float]:
    """Genera n_iter ipotesi e trova la migliore con operazioni matriciali batch.

    weights : (n_sub,) pesi per punto. Se forniti:
      - i triplet vengono campionati con probabilità proporzionale ai pesi
        (i pixel centrali ToF partecipano di più alla generazione delle ipotesi)
      - il punteggio di ogni ipotesi è la somma pesata degli inlier
        (un inlier centrale conta più di uno periferico)
    """
    n   = len(pts_sub)
    rng = np.random.default_rng()

    if weights is not None:
        p   = weights / weights.sum()
        idx = rng.choice(n, size=(n_iter, 3), replace=True, p=p)
    else:
        idx = rng.integers(0, n, size=(n_iter, 3))

    p1 = pts_sub[idx[:, 0]]
    p2 = pts_sub[idx[:, 1]]
    p3 = pts_sub[idx[:, 2]]

    normals = np.cross(p2 - p1, p3 - p1)
    norms   = np.linalg.norm(normals, axis=1)
    ok      = norms > 1e-9
    normals[ok]  /= norms[ok, np.newaxis]
    normals[~ok]  = 0.0
    d_vals = -np.einsum("ij,ij->i", normals, p1)

    BATCH    = 64
    best_cnt = 0.0
    best_n   = normals[0].copy()
    best_d   = float(d_vals[0])

    for s in range(0, n_iter, BATCH):
        e     = min(s + BATCH, n_iter)
        nb    = normals[s:e]
        db    = d_vals[s:e]
        dists = np.abs(pts_sub @ nb.T + db)          # (n_sub, b)
        inl   = dists < threshold_mm                  # (n_sub, b) bool
        if weights is not None:
            counts = (weights[:, None] * inl).sum(axis=0)   # somma pesata
        else:
            counts = inl.sum(axis=0)
        bi = counts.argmax()
        if counts[bi] > best_cnt:
            best_cnt = float(counts[bi])
            best_n   = nb[bi].copy()
            best_d   = float(db[bi])

    return best_n, best_d


def fit_plane_ransac_fast(points: np.ndarray,
                          n_iter: int = RANSAC_N_ITER,
                          threshold_mm: float = RANSAC_THRESHOLD_RS_MM,
                          max_pts: int = RANSAC_SUBSAMPLE_RS,
                          weights: np.ndarray | None = None) -> dict | None:
    """RANSAC vettorizzato con subsampling e pesi per punto opzionali.

    points      : (N, 3) float64, in mm
    weights     : (N,) float64, pesi per punto (es. gaussiani per il ToF).
                  Influenzano: campionamento dei triplet, punteggio degli inlier,
                  e il raffinamento WLS (minimo quadrati pesati).
                  None → comportamento uniforme originale.
    """
    N = len(points)
    if N < 4:
        return None

    # Subsampling — preserva i pesi corrispondenti
    if N > max_pts:
        sub_idx = np.random.choice(N, max_pts, replace=False)
        pts_sub = points[sub_idx]
        w_sub   = weights[sub_idx] if weights is not None else None
    else:
        pts_sub = points
        w_sub   = weights

    # Normalizza pesi subset
    if w_sub is not None:
        w_sub = w_sub / w_sub.sum()

    best_n, best_d = _ransac_batch_cpu(pts_sub, n_iter, threshold_mm, w_sub)

    # Inlier sul cloud COMPLETO
    all_dists = np.abs(points @ best_n + best_d)
    inliers   = np.where(all_dists < threshold_mm)[0]
    if len(inliers) < 4:
        return None

    # Raffinamento: WLS se pesi disponibili, altrimenti LSQ standard
    pts_in = points[inliers]
    if weights is not None:
        w_in = weights[inliers]
        w_in = w_in / w_in.sum()                          # normalizza
        centroid = (w_in[:, None] * pts_in).sum(axis=0)   # centroide pesato
        # SVD della matrice pesata: min Σ w_i (n·(p_i - c))²
        _, _, Vt = np.linalg.svd(
            np.sqrt(w_in[:, None]) * (pts_in - centroid), full_matrices=False)
    else:
        centroid = pts_in.mean(axis=0)
        _, _, Vt = np.linalg.svd(pts_in - centroid, full_matrices=False)

    normal = Vt[-1]
    d      = -float(normal @ centroid)
    if d < 0:
        normal, d = -normal, -d

    # Aggiorna inlier con il piano raffinato
    inliers = np.where(np.abs(points @ normal + d) < threshold_mm)[0]

    return {"normal": normal, "d": d, "centroid": centroid, "inliers": inliers}


# =============================================================================
# 3b. SURFACE FITTING — metodi alternativi al piano puro
# =============================================================================
#
#  Tutti i metodi ricevono un cloud (N,3) in mm e restituiscono lo stesso
#  dict {normal, d, centroid} usato dal resto della pipeline: si ricava la
#  retta normale alla superficie nel punto centroide, così l'algoritmo di
#  calibrazione non cambia.
#
#  Piano   : fit diretto WLS/SVD (invariante a RANSAC, già sugli inlier).
#  Polinomiale : Z = aX²+bXY+cY²+dX+eY+f  →  normale = gradiente al centroide.
#  Esponenziale: Z = exp(a+bX+cY)          →  fit lineare in log-spazio.
#  Biharmonico : thin-plate spline (scipy)  →  gradiente numerico al centroide.
#
# =============================================================================

def _fit_surface_plane(points: np.ndarray,
                       weights: np.ndarray | None = None) -> dict:
    """Fit lineare (piano) via WLS–SVD senza RANSAC."""
    w = weights / weights.sum() if weights is not None else None
    centroid = ((w[:, None] * points).sum(0) if w is not None
                else points.mean(0))
    A = ((np.sqrt(w[:, None]) * (points - centroid)) if w is not None
         else (points - centroid))
    _, _, Vt = np.linalg.svd(A, full_matrices=False)
    normal = Vt[-1]
    # d = distanza perpendicolare dall'origine al piano:
    # Calcolato proiettando il centroide sulla normale.
    d = -float(np.dot(centroid, normal))

    if d < 0:
        normal, d = -normal, -d
    return {"normal": normal, "d": d, "centroid": centroid}


def _fit_surface_polynomial(points: np.ndarray,
                             weights: np.ndarray | None = None) -> dict | None:
    """Fit quadratico Z = aX²+bXY+cY²+dX+eY+f via WLS.
    La normale al centroide è derivata dal gradiente analitico."""
    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
    w = weights / weights.sum() if weights is not None else np.ones(len(X)) / len(X)
    sw = np.sqrt(w)
    A = np.column_stack([X**2, X*Y, Y**2, X, Y, np.ones(len(X))])
    try:
        c, _, _, _ = np.linalg.lstsq(A * sw[:, None], Z * sw, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a, b, cc, d, e, f = c
    Xc = (w * X).sum();  Yc = (w * Y).sum()
    Zc = a*Xc**2 + b*Xc*Yc + cc*Yc**2 + d*Xc + e*Yc + f
    dz_dx = 2*a*Xc + b*Yc + d
    dz_dy = b*Xc + 2*cc*Yc + e
    normal = np.array([-dz_dx, -dz_dy, 1.0])
    n_len = np.linalg.norm(normal)
    if n_len < 1e-9:
        return None
    normal /= n_len
    centroid = np.array([Xc, Yc, Zc])
    dv = -float(np.dot(centroid, normal))
    if dv < 0:
        normal, dv = -normal, -dv
    return {"normal": normal, "d": dv, "centroid": centroid}


def _fit_surface_exponential(points: np.ndarray,
                              weights: np.ndarray | None = None) -> dict | None:
    """Fit esponenziale Z = exp(a+bX+cY) tramite regressione lineare in log-spazio.
    Modella superfici in cui la profondità varia esponenzialmente con X,Y
    (es. superfici curve viste in prospettiva)."""
    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
    ok = Z > 0
    X, Y, Z = X[ok], Y[ok], Z[ok]
    if len(X) < 4:
        return None
    w = (weights[ok] if weights is not None else np.ones(len(X)))
    w = w / w.sum()
    log_Z = np.log(Z)
    A  = np.column_stack([np.ones(len(X)), X, Y])
    sw = np.sqrt(w)
    try:
        c, _, _, _ = np.linalg.lstsq(A * sw[:, None], log_Z * sw, rcond=None)
    except np.linalg.LinAlgError:
        return None
    a_c, b_c, c_c = c
    Xc = (w * X).sum();  Yc = (w * Y).sum()
    Zc = float(np.exp(a_c + b_c*Xc + c_c*Yc))
    dz_dx = b_c * Zc
    dz_dy = c_c * Zc
    normal = np.array([-dz_dx, -dz_dy, 1.0])
    n_len  = np.linalg.norm(normal)
    if n_len < 1e-9:
        return None
    normal /= n_len
    centroid = np.array([Xc, Yc, Zc])
    points_ok = np.column_stack([X, Y, Z])   # punti dopo filtro Z>0
    dv = -float(np.dot(centroid, normal))
    if dv < 0:
        normal, dv = -normal, -dv
    return {"normal": normal, "d": dv, "centroid": centroid}


def _fit_surface_biharmonic(points: np.ndarray,
                             weights: np.ndarray | None = None) -> dict | None:
    """Fit thin-plate spline (biharmonico) tramite scipy.interpolate.RBFInterpolator.
    Gradiente della superficie calcolato numericamente al centroide pesato."""
    try:
        from scipy.interpolate import RBFInterpolator   # scipy >= 1.7
    except ImportError:
        logging.warning("scipy non disponibile: fallback a _fit_surface_polynomial")
        return _fit_surface_polynomial(points, weights)

    X, Y, Z = points[:, 0], points[:, 1], points[:, 2]
    w = weights / weights.sum() if weights is not None else None
    Xc = (w * X).sum() if w is not None else X.mean()
    Yc = (w * Y).sum() if w is not None else Y.mean()

    # Smoothing: piccolo per dati puliti RS, maggiore per dati rumorosi ToF
    n = len(X)
    smoothing = n * (0.05 if w is None else float((w**2).sum()) * n)
    try:
        rbf = RBFInterpolator(
            np.column_stack([X, Y]), Z,
            kernel="thin_plate_spline",
            smoothing=max(0.0, smoothing),
        )
    except Exception as e:
        logging.warning("RBFInterpolator fallito (%s: %s): fallback a _fit_surface_polynomial",
                        type(e).__name__, e)
        return _fit_surface_polynomial(points, weights)

    Zc = float(rbf([[Xc, Yc]])[0])
    # gradiente numerico con passo adattivo
    eps = max(1.0, (X.max() - X.min()) / 50)
    dz_dx = float(rbf([[Xc+eps, Yc]])[0] - rbf([[Xc-eps, Yc]])[0]) / (2*eps)
    dz_dy = float(rbf([[Xc, Yc+eps]])[0] - rbf([[Xc, Yc-eps]])[0]) / (2*eps)

    normal = np.array([-dz_dx, -dz_dy, 1.0])
    n_len  = np.linalg.norm(normal)
    if n_len < 1e-9:
        return None
    normal /= n_len
    centroid = np.array([Xc, Yc, Zc])
    dv = -float(np.dot(centroid, normal))
    if dv < 0:
        normal, dv = -normal, -dv
    return {"normal": normal, "d": dv, "centroid": centroid}


# Metodi disponibili: chiave usata internamente → etichetta GUI
SURFACE_FIT_METHODS: dict[str, str] = {
    "piano":        "Piano",
    "polinomiale":  "Polinomiale 2°",
    "esponenziale": "Esponenziale",
    "biharmonico":  "Biharmonico (spline)",
}


def fit_surface(points: np.ndarray, method: str,
                weights: np.ndarray | None = None) -> dict | None:
    """Adattatore centralizzato per il fitting della superficie.
    Tutti i metodi restituiscono {normal, d, centroid} con le stesse convenzioni."""
    if points is None or len(points) < 4:
        return None
    if method == "piano":
        return _fit_surface_plane(points, weights)
    if method == "polinomiale":
        return _fit_surface_polynomial(points, weights)
    if method == "esponenziale":
        return _fit_surface_exponential(points, weights)
    if method == "biharmonico":
        return _fit_surface_biharmonic(points, weights)
    return None


# =============================================================================
# 4. WORKER PARALLELO  (top-level per pickle su Windows)
# =============================================================================

def _process_pair(args: tuple) -> dict | None:
    """Elabora una singola coppia (tof, rgb) → risultato o None.
    Il piano RS è ricavato dalla pose del board ChArUco rilevato nell'immagine RGB.
    Funzione top-level per essere picklabile da ProcessPoolExecutor."""
    (ts_tof, dist_8x8, valid_8x8,
     ts_rs, rgb_path,
     K_flat, dist_flat,
     fov_h, fov_v,
     thr_tof, n_iter) = args

    n_valid = int(np.sum(valid_8x8))
    if n_valid < TOF_VALID_POINTS:
        return None

    K    = K_flat.reshape(3, 3)
    dist = dist_flat.reshape(-1)

    # --- ToF cloud & piano (con pesi gaussiani sui pixel centrali) ---
    w_sigma = TOF_PLANE_WEIGHT_SIGMA
    if w_sigma is not None:
        pc_tof, w_tof, mask_1d = tof_to_pointcloud(dist_8x8, valid_8x8, fov_h, fov_v, weight_sigma=w_sigma)
    else:
        pc_tof, mask_1d = tof_to_pointcloud(dist_8x8, valid_8x8, fov_h, fov_v)
        w_tof  = None
    if len(pc_tof) < 4:
        return None

    # RANSAC per individuare gli inlier (robusto per entrambi i metodi)
    ransac_tof = fit_plane_ransac_fast(pc_tof, n_iter, thr_tof, max_pts=TOF_INLIERS,
                                       weights=w_tof)
    if ransac_tof is None:
        return None
    if len(ransac_tof["inliers"]) < TOF_INLIERS:
        return None

    # Fit finale sul cloud ToF con il metodo scelto
    pc_tof_inl = pc_tof[ransac_tof["inliers"]]
    w_tof_inl  = w_tof[ransac_tof["inliers"]] if w_tof is not None else None
    plane_tof = fit_surface(pc_tof_inl, SURFACE_FIT_METHOD, weights=w_tof_inl)
    if plane_tof is None:
        return None

    # --- Piano RS dalla pose del ChArUco nell'RGB ---
    rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    if rgb_img is None:
        return None
    plane_rs = detect_charuco_plane(rgb_img, K, dist)
    if plane_rs is None:
        return None

    # Cloud "RS" per la GUI = angoli ChArUco proiettati nel frame della camera
    pc_rs_sample = plane_rs["points"]

    # Angolo di inclinazione del piano RS rispetto alla camera (0° = frontale)
    tilt_deg = float(np.degrees(np.arccos(np.clip(np.abs(plane_rs["normal"][2]), 0.0, 1.0))))

    return {
        # ── dati per calibrazione ──────────────────────────────────
        "ts_tof":        ts_tof,
        "normal_tof":    plane_tof["normal"],
        "normal_rs":     plane_rs["normal"],
        "d_tof":         plane_tof["d"],
        "d_rs":          plane_rs["d"],
        "centroid_tof":  plane_tof["centroid"],
        "centroid_rs":   plane_rs["centroid"],
        "tilt_deg":      tilt_deg,
        "tof_mask_id":   mask_1d,
        "n_charuco_corners": plane_rs["n_corners"],
        # ── dati extra per la GUI ──────────────────────────────────
        "rgb_path":      rgb_path,
        "dist_8x8":      dist_8x8,
        "valid_8x8":     valid_8x8,
        "pc_tof":        pc_tof,
        "w_tof":         w_tof,
        "pc_rs_sample":  pc_rs_sample,
        "dt_ms_sync":    abs(ts_tof - ts_rs),
    }


# =============================================================================
# 5. ESTRAZIONE CORRISPONDENZE DI PIANI  (parallela, pesante, fatta una volta)
# =============================================================================

def extract_plane_pairs(pairs, K_rs: np.ndarray, dist_rs: np.ndarray) -> list[dict]:
    """Processa tutte le coppie (tof, rgb) in parallelo.
    Il piano RS è ricavato dalla pose ChArUco rilevata nell'RGB.
    Restituisce lista di dict con normali e distanze dei piani corrispondenti."""
    K_flat    = K_rs.flatten()
    dist_flat = np.asarray(dist_rs, dtype=np.float64).reshape(-1)
    n_workers = os.cpu_count()

    worker_args = [
        (tof[0], tof[1], tof[2],
         rs[0], rs[1],
         K_flat, dist_flat,
         45.0, 45.0,
         RANSAC_THRESHOLD_TOF_MM, RANSAC_N_ITER)
        for tof, rs in pairs
    ]

    print(f"\n[1/3] Estrazione piani da {len(pairs)} coppie "
          f"su {n_workers} worker (CPU)")
    results, done = [], 0
    with ProcessPoolExecutor(max_workers=n_workers) as ex:
        futs = {ex.submit(_process_pair, a): i for i, a in enumerate(worker_args)}
        for fut in as_completed(futs):
            done += 1
            r = fut.result()
            if r is not None:
                results.append(r)

            if done % 50 == 0:
                print(f"  {done}/{len(pairs)} — {len(results)} piani validi")

    print(f"  Coppie di piani estratte: {len(results)}")
    
    if len(results) < MIN_PLANE_PAIRS:
        raise RuntimeError(
            f"Solo {len(results)} coppie di piani (minimo {MIN_PLANE_PAIRS}). "
            "Punta a superfici piane a distanze e angoli variati."
        )

    return results


# =============================================================================
# STIMA R SU TUTTE le coppie e costruisce il piano del TOF
# =============================================================================

def _solve_for_tof_plane(plane_pairs: list[dict]) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    """
    Calcola la rotazione indipendente per ogni coppia (shortest-arc), 
    estrae la matrice R mediana e calcola l'errore angolare rispetto ad essa.
    Alla fine mostra un visualizzatore 3D interattivo delle normali.
    """
    normals_tof = np.array([r["normal_tof"] for r in plane_pairs])
    normals_rs  = np.array([r["normal_rs"]  for r in plane_pairs])
    n = len(plane_pairs)

    # --- Helper: Algoritmo di Kabsch per allineare vettori (Origine fissa) ---
    def kabsch_rotation(P, Q):
        # Trova la rotazione R pura che minimizza le distanze tra R@P e Q
        H = P.T @ Q
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        return R
    
    R_list = []
    T_list = []
    
    for i, r in enumerate(plane_pairs):
        v1 = normals_tof[i]
        v2 = normals_rs[i]
        
        c = np.dot(v1, v2)
        
        if c > 0.999999:
            R_list.append(np.eye(3))
        elif c < -0.999999:
            continue
        else:
            v = np.cross(v1, v2)
            s = np.linalg.norm(v)
            vx = np.array([
                [ 0,    -v[2],  v[1]],
                [ v[2],  0,    -v[0]],
                [-v[1],  v[0],  0   ]
            ])
            R_ind = np.eye(3) + vx + (vx @ vx) * ((1 - c) / (s ** 2))
            R_list.append(R_ind)
        
        n_rot_final = (R_ind @ v1.T).T

        
        delta_d = r["d_tof"] - r["d_rs"] 
        r["delta_plane"] = {
            "normal": n_rot_final, 
            "d": delta_d
        }
        T_list.append(delta_d * n_rot_final)
            
    R_list = np.array(R_list)
    
    # --- 2. Estrazione della R "mediana" ---
    R_med_raw = np.median(R_list, axis=0)
    
    U, _, Vt = np.linalg.svd(R_med_raw)
    R_final = U @ Vt
    if np.linalg.det(R_final) < 0:
        Vt[-1, :] *= -1
        R_final = U @ Vt
    
    # # METODO ALTERNATIVO KABSH + RANSAC

    # best_inliers = []
    # best_R = np.eye(3)
    # for _ in range(10000):
    #     # Estraiamo 2 frame casuali (il minimo per bloccare 3 gradi di libertà)
    #     idx = np.random.choice(n, 2, replace=False)
        
    #     # Scartiamo la coppia se i due piani sono troppo paralleli (> 95% allineati)
    #     # Altrimenti l'asse di rotazione tra di essi rimarrebbe indefinito
    #     if np.abs(np.dot(normals_tof[idx[0]], normals_tof[idx[1]])) > 0.95:
    #         continue
            
    #     # Calcoliamo l'ipotesi di Rotazione sui 2 piani campionati
    #     R_hyp = kabsch_rotation(normals_tof[idx], normals_rs[idx])
        
    #     # Valutiamo l'errore angolare di questa R su TUTTI i piani
    #     n_rot = (R_hyp @ normals_tof.T).T
    #     cos_a = np.clip(np.sum(n_rot * normals_rs, axis=1), -1.0, 1.0)
    #     err_deg = np.degrees(np.arccos(cos_a))
        
    #     # Contiamo gli Inlier
    #     inliers = np.where(err_deg < 3)[0]
        
    #     if len(inliers) > len(best_inliers):
    #         best_inliers = inliers
    #         best_R = R_hyp
            
    # # --- 2. Estrazione R "Finale" ---
    # # Ricalcoliamo la rotazione finale usando TUTTI gli inlier trovati assieme
    # if len(best_inliers) >= 2:
    #     R_final = kabsch_rotation(normals_tof[best_inliers], normals_rs[best_inliers])
    #     print(f"  RANSAC Rotazione: {len(best_inliers)} inlier trovati su {n} (soglia 3°).")
    # else:
    #     print("  ⚠ RANSAC fallito: fallback su allineamento globale sporco.")
    #     R_final = kabsch_rotation(normals_tof, normals_rs)

    # # # ------------------------------------------
        
    # --- 3. Calcolo dell'errore angolare usando la R_final ---
    n_rot_final = (R_final @ normals_tof.T).T
    cos_ang_final = np.clip((n_rot_final * normals_rs).sum(axis=1), -1.0, 1.0)
    ang_err_final = np.degrees(np.arccos(cos_ang_final))
    
    # # --- 4. Creazione dei Delta Plane per TUTTE le coppie ---
    # for i, r in enumerate(plane_pairs):
    #     delta_d = r["d_tof"] - r["d_rs"] 
    #     r["delta_plane"] = {
    #         "normal": n_rot_final[i], 
    #         "d": delta_d
    #     }

    

    # # =========================================================================
    # # ── DEBUG INTERATTIVO 3D: Visualizzatore delle Pointcloud ────────────────
    # # =========================================================================
    # print("  Avvio visualizzatore 3D delle pointcloud...")
    # import matplotlib
    # _prev_backend = matplotlib.get_backend()
    # matplotlib.use("TkAgg")  # Forza la GUI interattiva
    # import matplotlib.pyplot as _plt
    # from matplotlib.widgets import Slider

    # _fig = _plt.figure(figsize=(12, 9))
    # _ax = _fig.add_subplot(111, projection='3d')
    # _plt.subplots_adjust(bottom=0.25)

    # # Funzione helper per applicare il sistema di coordinate swappato di OpenCV
    # def _swap_axes(pts):
    #     # pt: Array Nx3. Mappa X->X, Y->Z, Z->-Y
    #     return pts[:, 0], pts[:, 2], -pts[:, 1]

    # # Pre-calcoliamo i limiti globali guardando i dati della prima coppia 
    # # (o di tutte) per non far "saltare" la telecamera tra un frame e l'altro
    # all_x, all_y, all_z = [], [], []
    # for r in plane_pairs:
    #     x, y, z = _swap_axes(r["pc_rs_sample"]) # Assicurati di avere "pts_rs" salvato in r
    #     all_x.extend([x.min(), x.max()])
    #     all_y.extend([y.min(), y.max()])
    #     all_z.extend([z.min(), z.max()])
    
    # _ax.set_xlim(min(all_x), max(all_x))
    # _ax.set_ylim(min(all_y), max(all_y))
    # _ax.set_zlim(min(all_z), max(all_z))
    
    # try: _ax.set_box_aspect([1, 1, 1])
    # except AttributeError: pass

    # # Labels
    # _ax.set_xlabel("X [mm] (Destra)", fontsize=8)
    # _ax.set_ylabel("Z [mm] (Profondità)", fontsize=8)
    # _ax.set_zlabel("-Y [mm] (Alto)", fontsize=8)

    # # Dizionario per mantenere i riferimenti agli oggetti scatter
    # grafica = {"scatters": {}}

    # def update(val):
    #     idx = int(slider.val)
    #     r = plane_pairs[idx]

    #     # 1. Pointcloud RS originale
    #     # N.B. Inserisci le tue chiavi corrette se non usi "pts_rs" e "pts_tof"
    #     pts_rs = r["pc_rs_sample"]  
    #     x_rs, y_rs, z_rs = _swap_axes(pts_rs)

    #     # 2. Pointcloud TOF Rotata usando R_final (la rotazione globale stimata)
    #     pts_tof = r["pc_tof"]
    #     pts_tof_rot = (R_list[idx] @ pts_tof.T).T
    #     x_rot, y_rot, z_rot = _swap_axes(pts_tof_rot)

    #     # 3. Pointcloud TOF Rotata + Traslata lungo la normale delta
    #     delta_n = r["delta_plane"]["normal"]
    #     delta_d = r["delta_plane"]["d"]
    #       # Salva la traslazione per eventuali analisi future
    #     pts_tof_shift = pts_tof_rot + (delta_n * delta_d)
    #     x_s, y_s, z_s = _swap_axes(pts_tof_shift)

    #     # Aggiornamento Titolo
    #     _ax.set_title(f"Coppia {idx+1}/{n} | Errore Angolare R_final: {ang_err_final[idx]:.2f}°\n"
    #                   f"Traslazione (lungo normale): {delta_d:.1f} mm", 
    #                   fontsize=10, fontweight="bold")

    #     # Se è la prima esecuzione, disegniamo, altrimenti modifichiamo solo i dati
    #     if not grafica["scatters"]:
    #         grafica["scatters"]["rs"]    = _ax.scatter(x_rs, y_rs, z_rs, c="steelblue", s=2, alpha=0.3, label="RS Target")
    #         grafica["scatters"]["rot"]   = _ax.scatter(x_rot, y_rot, z_rot, c="red", s=2, alpha=0.3, label="ToF Ruotato")
    #         grafica["scatters"]["shift"] = _ax.scatter(x_s, y_s, z_s, c="green", s=4, alpha=0.8, label="ToF Ruotato + Traslato")
    #         _ax.legend(loc="upper left", markerscale=5, fontsize=9)
    #     else:
    #         grafica["scatters"]["rs"]._offsets3d    = (x_rs, y_rs, z_rs)
    #         grafica["scatters"]["rot"]._offsets3d   = (x_rot, y_rot, z_rot)
    #         grafica["scatters"]["shift"]._offsets3d = (x_s, y_s, z_s)

    #     _fig.canvas.draw_idle()

    # # Setup dello slider
    # ax_slider = _plt.axes([0.15, 0.05, 0.7, 0.03])
    # slider = Slider(
    #     ax=ax_slider, label='Frame',
    #     valmin=0, valmax=n - 1, valinit=0, valstep=1, color='gray'
    # )
    # slider.on_changed(update)
    
    # # Forza il disegno del primo frame
    # update(0)

    # # Aggiunge una 'X' nera per referenza origine
    # _ax.scatter(0, 0, 0, color="black", s=100, marker="X", zorder=10, label="Origine (0,0,0)")
    
    # _plt.show(block=True)
    # matplotlib.use(_prev_backend)
    # # =========================================================================
    
    R = np.median(R_list, axis=0)
    T_list = np.array(T_list)
    T = np.median(T_list, axis=0)
    print(f"\nRotazione : {R}")
    print(f"\nTraslazione : {T}")
    return R_final, ang_err_final, plane_pairs



def get_translation_cloud(plane_pairs: list[dict], _iter: int, det_threshold: float = 0.05) -> np.ndarray:
    """
    Estrae iterativamente 3 piani random, ne calcola l'intersezione (traslazione) 
    e restituisce una nuvola di punti con tutte le ipotesi di traslazione.
    """
    n = len(plane_pairs)
    if n < 3:
        raise ValueError("Servono almeno 3 coppie di piani per calcolare un'intersezione.")

    rng = np.random.default_rng()
    subset_size = 3
    intersections = []

    attempts = 0
    max_attempts = _iter * 10 # Previene loop infiniti se tutti i piani sono paralleli

    while len(intersections) < _iter and attempts < max_attempts:
        attempts += 1
        
        # 1. Selezione randomica
        idx = rng.choice(n, subset_size, replace=False)
        subset = [plane_pairs[j] for j in idx]

        # 2. Estrazione Matrice A (normali) e Vettore B (distanze)
        A = np.array([p["delta_plane"]["normal"] for p in subset])
        B = np.array([p["delta_plane"]["d"] for p in subset])

        # 3. CONTROLLO DI QUALITÀ GEOMETRICA
        # Calcoliamo il determinante di A
        det_A = np.linalg.det(A)
        
        # Se il determinante è troppo vicino a 0, i piani non formano un buon "angolo" 
        # (sono quasi paralleli/complanari). Scartiamo la combinazione.
        if np.abs(det_A) < det_threshold:
            continue

        # 4. Calcolo dell'intersezione
        try:
            intersection_point = np.linalg.solve(A, B)
            intersections.append(intersection_point)
        except np.linalg.LinAlgError:
            # Cattura di sicurezza per matrici numericamente singolari
            continue

    if len(intersections) < _iter:
        print(f"Attenzione: Trovate solo {len(intersections)} intersezioni stabili dopo {max_attempts} tentativi. I piani nel dataset potrebbero essere per lo più paralleli.")
    # 4. Convertiamo la lista di punti in una nuvola di punti numpy (N, 3)
    cloud = np.array(intersections)
    
    return cloud

# =============================================================================
# 7. OUTER RANSAC  (media su N_OUTER_ITER subset random)
# =============================================================================


def outer_ransac_calibrate(plane_pairs: list[dict],
                           n_iter: int | None = None) -> tuple[np.ndarray, dict]:
    """Calibrazione robusta con outer RANSAC su subset random.
    n_iter      : numero di iterazioni 
    """
    _iter = n_iter      if n_iter      is not None else 1000
    n = len(plane_pairs)

    # Stima R su tutte le coppie
    R_final, ang_err_tot, plane_pairs = _solve_for_tof_plane(plane_pairs) 

    translation_cloud = get_translation_cloud(plane_pairs, _iter)

    t_final = np.median(translation_cloud, axis=0)
    t_final[2] = 0  # Forza la componente z a 0

    # 1. Calcola gli scarti assoluti dalla mediana
    abs_deviations = np.abs(translation_cloud - t_final)
    
    # 2. Calcola la MAD (la mediana di questi scarti)
    mad = np.median(abs_deviations, axis=0)
    
    # 3. Converti in stima robusta della Deviazione Standard
    t_std = 1.4826 * mad

    T = np.eye(4)
    T[:3, :3] = R_final
    T[:3,  3] = t_final
# =========================================================================
    # ── CALCOLO ERRORI POINT-TO-PLANE (ToF vs Piano RS) ────────────
    # =========================================================================
    pixel_errors = [{"raw": [], "rot": [], "rt": []} for _ in range(64)]
    
    for r in plane_pairs:
        pc_tof_raw = np.asarray(r.get("pc_tof", np.empty((0, 3))))
        
        if len(pc_tof_raw) == 0:
            continue
            
        # 1. Calcolo dei tre stati
        pc_tof_rot = (R_final @ pc_tof_raw.T).T
        pc_tof_rt  = pc_tof_rot + t_final
        
        # 2. Distanze Punto-Piano
        n_rs = r["normal_rs"]
        d_rs = r["d_rs"]
        
        err_raw = np.abs(np.dot(pc_tof_raw, n_rs) + d_rs)
        err_rot = np.abs(np.dot(pc_tof_rot, n_rs) + d_rs)
        err_rt  = np.abs(np.dot(pc_tof_rt, n_rs)  + d_rs)

        r["err_raw"] = err_raw
        r["err_rot"] = err_rot
        r["err_rt"]  = err_rt
        
        # 3. Assegnazione dell'errore al singolo pixel (1-64)
        mask_1d = r.get("tof_mask_1d")
        if mask_1d is not None:
            valid_indices = np.where(mask_1d)[0]
        else:
            valid_indices = range(min(64, len(err_raw)))
            
        for idx, e_raw, e_rot, e_rt in zip(valid_indices, err_raw, err_rot, err_rt):
            pixel_errors[idx]["raw"].append(e_raw)
            pixel_errors[idx]["rot"].append(e_rot)
            pixel_errors[idx]["rt"].append(e_rt)

    # ── Aggregazione Statistiche Pixel (su tutti i frame) ──
    # Usiamo np.nan invece di None per permettere il reshape matematico nella GUI
    pixel_stats = {
        "raw": {"mean": [], "median": [], "rmse": []},
        "rot": {"mean": [], "median": [], "rmse": []},
        "rt":  {"mean": [], "median": [], "rmse": []}
    }
    
    for p_errs in pixel_errors:
        for state in ["raw", "rot", "rt"]:
            if len(p_errs[state]) > 0:
                arr = np.array(p_errs[state])
                pixel_stats[state]["mean"].append(float(np.mean(arr)))
                pixel_stats[state]["median"].append(float(np.median(arr)))
                pixel_stats[state]["rmse"].append(float(np.sqrt(np.mean(arr**2))))
            else:
                pixel_stats[state]["mean"].append(np.nan)
                pixel_stats[state]["median"].append(np.nan)
                pixel_stats[state]["rmse"].append(np.nan)
    # =========================================================================

    diag = {
        "n_plane_pairs":   n,
        "n_outer_iter":    _iter,
        "angular_errors": ang_err_tot.tolist(),
        "translation_std_mm":  t_std.tolist(),
        "translation_cloud":   translation_cloud,
        
        # Salviamo la statistica dei pixel per la GUI
        "pixel_stats": pixel_stats,
        
        "frame_info": [
            {k: (v.tolist() if isinstance(v, np.ndarray) else v)
             for k, v in r.items()}
            for r in plane_pairs
        ],
    }
    return T, diag

# =============================================================================
# 10. GUI INTERATTIVA
# =============================================================================

from PyQt5.QtWidgets import (                                  # noqa: E402
    QApplication, QGridLayout, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QPushButton, QGroupBox, QTextBrowser, QSizePolicy,
    QDoubleSpinBox, QComboBox, QTabWidget,
)
from PyQt5.QtGui  import QPixmap, QImage                       # noqa: E402
from PyQt5.QtCore import Qt                                    # noqa: E402
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection        # noqa: E402
import matplotlib.figure as _mpl_fig                           # noqa: E402


# Distanza massima condivisa da depth RS e ToF per la scala cromatica (mm)
COLORBAR_MAX_MM = 1200

# ── helper immagini ──────────────────────────────────────────────────────────

def _arr_to_pixmap(bgr: np.ndarray, w: int, h: int) -> QPixmap:
    rgb  = bgr[:, :, ::-1].copy()
    qh, qw = rgb.shape[:2]
    qimg = QImage(rgb.data, qw, qh, qw * 3, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg).scaled(w, h, Qt.KeepAspectRatio,
                                          Qt.SmoothTransformation)

def _depth_bgr(depth16: np.ndarray,
               max_mm: int = COLORBAR_MAX_MM) -> np.ndarray:
    """Depth image RS → BGR con scala fissa 0–max_mm (JET)."""
    norm = np.clip(depth16.astype(np.float32) / max_mm * 255, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)

def _tof_bgr(dist_8x8: np.ndarray, valid_8x8: np.ndarray,
             size: int = 256, max_mm: int = COLORBAR_MAX_MM) -> np.ndarray:
    """ToF heatmap → BGR con la stessa scala 0–max_mm (JET)."""
    d    = dist_8x8.astype(float)
    norm = np.clip(d / max_mm * 255, 0, 255)
    norm = np.where(np.isnan(d) | ~valid_8x8, 0, norm).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    colored[~valid_8x8] = [30, 30, 30]
    return cv2.resize(colored, (size, size), interpolation=cv2.INTER_NEAREST)

def _make_colorbar_pixmap(height: int,
                          max_mm: int = COLORBAR_MAX_MM,
                          bar_w: int = 18, lbl_w: int = 42) -> QPixmap:
    """Colorbar verticale JET: bottom=0 (blu), top=max_mm (rosso).
    Scala condivisa da depth RS e ToF."""
    w   = bar_w + lbl_w
    img = np.zeros((height, w, 3), dtype=np.uint8)
    # gradiente dal basso (0, blu) verso l'alto (max, rosso)
    grad = np.linspace(255, 0, height, dtype=np.uint8).reshape(-1, 1)
    img[:, :bar_w] = cv2.applyColorMap(np.tile(grad, (1, bar_w)), cv2.COLORMAP_JET)
    # tick labels
    n_ticks = 7
    for i in range(n_ticks):
        mm = int(max_mm * i / (n_ticks - 1))
        y  = int((height - 1) * (1 - i / (n_ticks - 1)))
        cv2.line(img, (bar_w, y), (bar_w + 4, y), (200, 200, 200), 1)
        label = f"{mm/1000:.1f}m" if mm else "0"
        cv2.putText(img, label, (bar_w + 6, y + 4),
                    cv2.FONT_HERSHEY_PLAIN, 0.72, (200, 200, 200), 1, cv2.LINE_AA)
    rgb  = img[:, :, ::-1].copy()
    qimg = QImage(rgb.data, w, height, w * 3, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)


# ── helpers per la visualizzazione pose sensori ───────────────────────────────
def _draw_sensor_frame(ax, origin: np.ndarray, R: np.ndarray,
                       scale: float, labels: tuple[str, ...] = ("X","Y","Z"),
                       alpha: float = 1.0, linestyle: str = "-") -> None:
    """Disegna i tre assi di un sistema di riferimento come quiver colorati."""
    colors = ("red", "limegreen", "dodgerblue")
    
    # Prepariamo l'origine swappata (X, Z, -Y)
    o_x, o_y, o_z = origin[0], origin[2], -origin[1]
    
    for i, (col, lab) in enumerate(zip(colors, labels)):
        d = R[:, i] * scale
        # Direzione swappata
        d_x, d_y, d_z = d[0], d[2], -d[1]
        
        ax.quiver(o_x, o_y, o_z, d_x, d_y, d_z, color=col, alpha=alpha, linewidth=2,
                  arrow_length_ratio=0.18, linestyle=linestyle)
        
        # Posizione del testo swappata
        pos = origin + d * 1.15
        ax.text(pos[0], pos[2], -pos[1], lab, color=col,
                fontsize=7, alpha=alpha, fontweight="bold")


def _draw_frustum(ax, origin: np.ndarray, R: np.ndarray,
                  K: np.ndarray, depth: float, img_w: int, img_h: int,
                  color: str = "steelblue", alpha: float = 0.2) -> None:
    """Frustum rettangolare della RS camera (4 linee + rettangolo frontale)."""
    h, w = img_h, img_w
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    uvs = [(0,0),(w,0),(w,h),(0,h)]
    
    # Matematica intatta: calcola i vertici nel sistema OpenCV
    corners = [origin + R @ np.array([(u-cx)*depth/fx, (v-cy)*depth/fy, depth])
               for u, v in uvs]
               
    # Scambiamo gli assi (X, Z, -Y) prima di fare plot
    o_sw = (origin[0], origin[2], -origin[1])
    corners_sw = [(c[0], c[2], -c[1]) for c in corners]
    
    # Linee dall'origine agli angoli
    for c in corners_sw:
        ax.plot(*zip(o_sw, c), color=color, alpha=alpha, lw=0.8)
        
    # Linee che uniscono gli angoli (il rettangolo frontale)
    for i in range(4):
        a, b = corners_sw[i], corners_sw[(i+1)%4]
        ax.plot(*zip(a, b), color=color, alpha=alpha, lw=0.8)


def _draw_tof_frustum(ax, origin: np.ndarray, R: np.ndarray,
                      depth: float,
                      fov_h_deg: float = 45.0, # Sostituisci se hai costanti globali diverse
                      fov_v_deg: float = 45.0,
                      color: str = "tomato", alpha: float = 0.2) -> None:
    """Frustum piramidale del ToF con il FoV configurato."""
    hh = np.tan(np.radians(fov_h_deg / 2))
    hv = np.tan(np.radians(fov_v_deg / 2))
    
    # Matematica intatta
    corners = [origin + R @ (np.array([sx*hh, sy*hv, 1.0]) * depth)
               for sx, sy in ((-1,-1),(1,-1),(1,1),(-1,1))]
               
    # Scambiamo gli assi (X, Z, -Y)
    o_sw = (origin[0], origin[2], -origin[1])
    corners_sw = [(c[0], c[2], -c[1]) for c in corners]
    
    # Linee dall'origine agli angoli
    for c in corners_sw:
        ax.plot(*zip(o_sw, c), color=color, alpha=alpha*1.5, lw=0.8)
        
    # Linee che uniscono gli angoli frontali
    for i in range(4):
        a, b = corners_sw[i], corners_sw[(i+1)%4]
        ax.plot(*zip(a, b), color=color, alpha=alpha*1.5, lw=0.8)

# ── helper 3D ────────────────────────────────────────────────────────────────

def _set_equal_aspect_3d(ax, pts: np.ndarray, margin: float = 0.08) -> None:
    """Impone la stessa scala su tutti e tre gli assi di un axes 3D.

    Senza questa funzione matplotlib scala ogni asse indipendentemente:
    se i punti sono a z ≈ D costante (parete piatta), il range Z diventa
    quasi nullo mentre X e Y sono centinaia di mm, rendendo il piano
    visivamente deformato. Con assi isometrici una parete piatta
    appare effettivamente piatta.

    pts : (N, 3) numpy array che contiene tutti i punti da visualizzare.
    margin : margine relativo (es. 0.08 = 8% del range).
    """
    if pts is None or len(pts) == 0:
        return
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    center = (mn + mx) / 2.0
    r = max(float((mx - mn).max()) / 2.0, 10.0)  # almeno 10 mm
    r *= (1.0 + margin)
    ax.set_xlim(center[0] - r, center[0] + r)
    ax.set_ylim(center[1] - r, center[1] + r)
    ax.set_zlim(center[2] - r, center[2] + r)
    try:
        ax.set_box_aspect([1, 1, 1])
    except AttributeError:
        print("Attenzione: serve Matplotlib >= 3.3 per set_box_aspect")


def _plane_verts(normal: np.ndarray, centroid: np.ndarray, size: float) -> list:
    """Quattro vertici di un patch quadrato sul piano dato."""
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(normal, ref))) > 0.85:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, ref);  u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    s = size / 2.0
    return [[centroid + s*u + s*v, centroid + s*u - s*v,
             centroid - s*u - s*v, centroid - s*u + s*v]]


# ── GUI principale ────────────────────────────────────────────────────────────

class CalibrationGUI(QMainWindow):

    def __init__(self, frames: list[dict], T: np.ndarray,
                 K_rs: np.ndarray, depth_scale_mm: float, session_dir: str, W: int, H: int,
                 excluded: "set[int] | None" = None,
                 output_json: str = "tof_rs_extrinsics.json", diag: dict | None = None):
        super().__init__()
        self.frames       = frames
        self.T            = T.copy()
        self.K_rs         = K_rs
        self.W = W
        self.H = H
        self.depth_scale  = depth_scale_mm
        self.session_dir  = session_dir
        self.output_json  = output_json
        self.manual_excluded: set[int] = set(excluded) if excluded else set()
        self.excluded: set[int] = set(self.manual_excluded)   # aggiornato a ogni ricalibrazione
        self.translation_cloud: np.ndarray = np.empty((0, 3))
        self.current_idx  = 0
        self.diag = diag or {}
        
        self.setWindowTitle("ToF–RS Calibration Review")
        self.resize(1620, 950)
        self._build_ui()
        # colorbar: altezza = circa la somma delle due immagini + titoli GroupBox
        self.lbl_colorbar.setPixmap(_make_colorbar_pixmap(600, COLORBAR_MAX_MM))
        self.slider.setValue(0)
        self._show_frame(0)
        # self._update_sensor_pose()

        if self.diag:
            self._update_error_heatmaps(self.diag)

    # ── costruzione UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        root_w = QWidget();  self.setCentralWidget(root_w)
        root   = QVBoxLayout(root_w);  root.setSpacing(6)

        # ── barra navigazione ────────────────────────────────────────────────
        nav = QHBoxLayout()
        self.btn_prev = QPushButton("◄");  self.btn_prev.setFixedWidth(44)
        self.btn_prev.clicked.connect(lambda: self._nav(-1))

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, len(self.frames) - 1))
        self.slider.valueChanged.connect(self._show_frame)

        self.btn_next = QPushButton("►");  self.btn_next.setFixedWidth(44)
        self.btn_next.clicked.connect(lambda: self._nav(+1))

        self.lbl_frame   = QLabel("Frame 0/0");   self.lbl_frame.setFixedWidth(120)

        self.btn_excl = QPushButton("Escludi");    self.btn_excl.setFixedWidth(110)
        self.btn_excl.clicked.connect(self._on_toggle_exclude)

        for w in (self.btn_prev, self.slider, self.btn_next,
                  self.lbl_frame, self.btn_excl):
            nav.addWidget(w, 1 if w is self.slider else 0)
        root.addLayout(nav)

        # ── pannello parametri calibrazione ──────────────────────────────────
        ctrl_box = QGroupBox("Parametri calibrazione")
        ctrl = QHBoxLayout(ctrl_box)
        ctrl.setSpacing(12)

        def _spin(lo, hi, step, val, suffix, decimals=1):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step)
            s.setValue(val); s.setSuffix(suffix)
            s.setDecimals(decimals); s.setFixedWidth(110)
            return s

        ctrl.addWidget(QLabel("Fitting"))
        self.combo_fit = QComboBox()
        for key, label in SURFACE_FIT_METHODS.items():
            self.combo_fit.addItem(label, key)
        self.combo_fit.setFixedWidth(200)
        ctrl.addWidget(self.combo_fit)

        ctrl.addStretch(1)
        root.addWidget(ctrl_box)

        # ── pannelli centrali ─────────────────────────────────────────────────
        mid = QHBoxLayout();  mid.setSpacing(6)

        # colonna sinistra: depth + tof
        left = QVBoxLayout();  left.setSpacing(4)

        box_dep = QGroupBox("RS Depth");  dl = QVBoxLayout(box_dep)
        self.lbl_depth = QLabel()
        self.lbl_depth.setAlignment(Qt.AlignCenter)
        self.lbl_depth.setFixedSize(420, 310)
        self.lbl_depth.setStyleSheet("background:#111;")
        dl.addWidget(self.lbl_depth)
        left.addWidget(box_dep)

        box_rgb = QGroupBox("RS RGB");  rl = QVBoxLayout(box_rgb)
        self.lbl_rgb = QLabel()
        self.lbl_rgb.setAlignment(Qt.AlignCenter)
        self.lbl_rgb.setFixedSize(420, 310)
        self.lbl_rgb.setStyleSheet("background:#111;")
        rl.addWidget(self.lbl_rgb)
        left.addWidget(box_rgb)

        box_tof = QGroupBox("ToF Distance (mm)")
        tl = QGridLayout(box_tof)
        tl.setSpacing(2)  # Piccolo spazio tra le celle

        self.tof_labels = []  # Matrice per salvare i riferimenti alle label
        tof_dim = 8
        cell_size = 256 // tof_dim  # Circa 32x32 pixel per mantenere l'ingombro a 256

        for r in range(tof_dim):
            row_labels = []
            for c in range(tof_dim):
                lbl = QLabel("0")
                lbl.setAlignment(Qt.AlignCenter)
                lbl.setFixedSize(cell_size, cell_size)
                
                # Stile per ogni singolo sub-box (sfondo scuro, testo bianco, bordo)
                lbl.setStyleSheet("""
                    background-color: #111; 
                    color: #FFF; 
                    border: 1px solid #333;
                    font-size: 10px;
                """)
                
                tl.addWidget(lbl, r, c)
                row_labels.append(lbl)
            self.tof_labels.append(row_labels)

        left.addWidget(box_tof)

        mid.addLayout(left)

        # ── colorbar condivisa depth+ToF ─────────────────────────────────────
        self.lbl_colorbar = QLabel()
        self.lbl_colorbar.setFixedWidth(62)
        self.lbl_colorbar.setAlignment(Qt.AlignTop | Qt.AlignHCenter)
        self.lbl_colorbar.setStyleSheet("background:#1a1a1a;border-radius:3px;")
        mid.addWidget(self.lbl_colorbar)

        # colonna centrale: metriche testo
        box_info = QGroupBox("Metriche");  box_info.setFixedWidth(195)
        il = QVBoxLayout(box_info)
        self.txt_info = QTextBrowser();  self.txt_info.setReadOnly(True)
        il.addWidget(self.txt_info)
        mid.addWidget(box_info)

        # colonna destra: tab con (1) piani del frame e (2) pose sensori
        self.tab_3d = QTabWidget()
        self.tab_3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Tab 0 — piani del frame corrente
        tab_planes = QWidget()
        vl_planes  = QVBoxLayout(tab_planes)
        vl_planes.setContentsMargins(2, 2, 2, 2)
        fig_planes     = _mpl_fig.Figure(figsize=(7.5, 6), tight_layout=True)
        self.ax3d      = fig_planes.add_subplot(111, projection="3d")
        self.canvas3d  = FigureCanvasQTAgg(fig_planes)
        self.canvas3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vl_planes.addWidget(self.canvas3d)
        self.tab_3d.addTab(tab_planes, "Piani del frame")

        # Tab 2 — FoV ToF proiettato sull'immagine depth
        tab_fov  = QWidget()
        vl_fov   = QVBoxLayout(tab_fov)
        vl_fov.setContentsMargins(2, 2, 2, 2)
        fig_fov      = _mpl_fig.Figure(figsize=(7.5, 6), tight_layout=True)
        
        self.ax_fov  = fig_fov.add_subplot(111)          # axes 2D
        
        # ── BLOCCA IL FORM FACTOR DELL'IMMAGINE ──────────────────────────────
        self.ax_fov.set_aspect('equal', adjustable='box')
        # ─────────────────────────────────────────────────────────────────────

        self.canvas_fov = FigureCanvasQTAgg(fig_fov)
        self.canvas_fov.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vl_fov.addWidget(self.canvas_fov)
        self.tab_3d.addTab(tab_fov, "Proiezione FoV ToF")
        
        # ── NUOVO: Tab 3 — Analisi Errori 8x8 Cumulativi (Heatmaps) ──────────
        tab_err  = QWidget()
        vl_err   = QVBoxLayout(tab_err)
        vl_err.setContentsMargins(2, 2, 2, 2)
        
        # Sostituito tight_layout con constrained_layout e alzata la figura a 5
        self.fig_err  = _mpl_fig.Figure(figsize=(10, 5), constrained_layout=True)
        self.axes_err = self.fig_err.subplots(1, 3) 
        
        titles = ["Pre-Allineamento (Raw)", "Post-Rotazione (R)", "Post-Rototraslazione (R+t)"]
        for ax, title in zip(self.axes_err, titles):
            ax.set_title(title, fontsize=9, fontweight="bold")
            ax.axis('off') 

        self.canvas_err = FigureCanvasQTAgg(self.fig_err)
        self.canvas_err.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        vl_err.addWidget(self.canvas_err)
        self.tab_3d.addTab(tab_err, "Errori ToF 8x8")
        # ─────────────────────────────────────────────────────────────────────

        # aggiorna i viewer quando l'utente cambia tab
        self.tab_3d.currentChanged.connect(self._on_tab_changed)

        mid.addWidget(self.tab_3d, 1)

        root.addLayout(mid, 1)

        # ── barra calibrazione ────────────────────────────────────────────────
        bot = QHBoxLayout()
        self.btn_recal = QPushButton("🔄  Ricalibrare")
        self.btn_recal.setMinimumHeight(34)
        self.btn_recal.clicked.connect(self._on_recalibrate)

        btn_reset = QPushButton("♻  Ripristina esclusioni")
        btn_reset.setMinimumHeight(34)
        btn_reset.setToolTip("Rimuove tutte le esclusioni manuali e automatiche")
        btn_reset.clicked.connect(self._on_reset_exclusions)

        self.lbl_calib = QLabel("—")
        self.lbl_calib.setStyleSheet("font-weight:bold;padding:0 8px;")

        self.btn_save = QPushButton("💾  Salva JSON")
        self.btn_save.setMinimumHeight(34);  self.btn_save.setFixedWidth(150)
        self.btn_save.clicked.connect(self._on_save)

        bot.addWidget(self.btn_recal)
        bot.addWidget(btn_reset)
        bot.addWidget(self.lbl_calib, 1)
        bot.addWidget(self.btn_save)
        root.addLayout(bot)

        self._refresh_status_bar()

    # ── navigazione ───────────────────────────────────────────────────────────

    def _nav(self, delta: int):
        self.slider.setValue(
            max(0, min(len(self.frames) - 1, self.current_idx + delta))
        )

    # ── mostra frame ─────────────────────────────────────────────────────────

    def _show_frame(self, idx: int):
        self.current_idx = idx
        frame = self.frames[idx]
        excl  = idx in self.excluded
        n     = len(self.frames)

        # navigazione label
        self.lbl_frame.setText(f"{'[X] ' if excl else ''}Frame {idx+1}/{n}")

        # badge qualità
        ae = frame.get("ang_err", float("nan"))

        # bottone escludi
        self.btn_excl.setText("Includi" if excl else "Escludi")
        self.btn_excl.setStyleSheet(
            "background:#993333;color:white;font-weight:bold;" if excl else ""
        )

        # ── depth RS ─────────────────────────────────────────────────────────
        dep_path = frame.get("dep_path", "")
        if dep_path and os.path.exists(dep_path):
            dep_img = cv2.imread(dep_path, cv2.IMREAD_ANYDEPTH)
            if dep_img is not None:
                self.lbl_depth.setPixmap(_arr_to_pixmap(_depth_bgr(dep_img), 420, 310))
            else:
                self.lbl_depth.clear()
        else:
            self.lbl_depth.clear()

        # ── RGB RS ───────────────────────────────────────────────────────────
        rgb_path = frame.get("rgb_path", "")
        if rgb_path and os.path.exists(rgb_path):
            rgb_img = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
            if rgb_img is not None:
                self.lbl_rgb.setPixmap(_arr_to_pixmap(rgb_img, 420, 310))
            else:
                self.lbl_rgb.clear()
        else:
            self.lbl_rgb.clear()

        # ── ToF heatmap (Numerica) ───────────────────────────────────────────
        d8 = np.asarray(frame.get("dist_8x8", np.full((8, 8), np.nan)))
        v8 = np.asarray(frame.get("valid_8x8", np.zeros((8, 8), bool)))
        
        # Sfruttiamo la tua funzione _tof_bgr chiedendole una matrice 8x8 
        # per recuperare gli stessi identici colori della colorbar
        colored_bgr = _tof_bgr(d8, v8, size=8, max_mm=COLORBAR_MAX_MM)
        
        for r in range(8):
            for c in range(8):
                if v8[r, c] and np.isfinite(d8[r, c]):
                    dist_mm = d8[r, c]
                    self.tof_labels[r][c].setText(f"{dist_mm:.0f}")
                    
                    # Estrai il colore (BGR) e convertilo in esadecimale per il CSS
                    b, g, r_color = colored_bgr[r, c]
                    hex_color = f"#{r_color:02x}{g:02x}{b:02x}"
                    
                    self.tof_labels[r][c].setStyleSheet(f"""
                        background-color: {hex_color}; 
                        color: white; 
                        border: 1px solid #222;
                        font-size: 10px;
                        font-weight: bold;
                    """)
                else:
                    self.tof_labels[r][c].setText("-")
                    self.tof_labels[r][c].setStyleSheet("""
                        background-color: #1a1a1a; 
                        color: #555; 
                        border: 1px solid #222;
                        font-size: 10px;
                    """)

        # ── metriche testo ────────────────────────────────────────────────────
        dt  = frame.get("dt_ms_sync", float("nan"))
        ae = self.diag["angular_errors"][idx]
        qstr = f"<br><b>Err angolare:</b> {ae:.2f}°" if np.isfinite(ae) else ""
        self.txt_info.setHtml(
            f"<b>{'⛔ ESCLUSO' if excl else '✅ attivo'}</b>"
            f"<br><br><b>Frame:</b> {idx+1}/{n}"
            f"<br><b>Δt sync:</b> {dt:.1f} ms"
            + qstr
        )

        # ── 3D + FoV projection (se il tab è attivo) ─────────────────────────
        self._update_3d(frame)
        if self.tab_3d.currentIndex() == 2:
            self._update_fov_projection(idx)

    def _on_tab_changed(self, tab_idx: int) -> None:
        """Aggiorna il viewer corretto quando l'utente cambia tab."""
        if tab_idx == 1:
            self._update_fov_projection(self.current_idx)
        elif tab_idx == 2:
            self._update_error_heatmaps(self.diag)

    # ── visualizzazione 3D ────────────────────────────────────────────────────

    def _update_3d(self, frame: dict):
        """Visualizza RS e ToF nello stesso sistema di riferimento (frame RS).

        Convenzione assi (camera / RealSense):
          X  →  laterale destra
          Y  ↓  verticale basso   (camera convention)
          Z  →  distanza verso il piano (profondità)

        Per il plot si nega solo Y:
          X_plot =  X_cam   (laterale destra, invariato)
          Y_plot = -Y_cam   (verticale su = +)
          Z_plot =  Z_cam   (distanza verso il piano = asse Z del plot)

        M = diag(1,-1,1).  Per trasformare direzioni verso il plot: M @ v_cam.
        Per frustum/assi sensori si passa M @ R_cam (non M @ R @ M.T), perché
        le direzioni template interne a quelle funzioni sono già in camera frame.
        """
        ax = self.ax3d;  ax.cla()
        R, t = self.T[:3, :3], self.T[:3, 3]

        # ── dati in camera frame ─────────────────────────────────────────────
        n_tof    = np.asarray(frame["normal_tof"])
        n_rs     = np.asarray(frame["normal_rs"])
        c_tof    = np.asarray(frame["centroid_tof"])
        c_rs     = np.asarray(frame.get("centroid_rs", np.zeros(3)))
        n_tof_rs = R @ n_tof
        c_tof_rs = R @ c_tof + t
        tof_orig = t.copy()

        pc_tof_raw = np.asarray(frame.get("pc_tof",       np.empty((0,3))))
        pc_rs_s    = np.asarray(frame.get("pc_rs_sample",  np.empty((0,3))))
        pc_tof_ruot  = (R @ pc_tof_raw.T).T if len(pc_tof_raw) else np.empty((0,3))
        pc_tof_rs  = (R @ pc_tof_raw.T + t[:,None]).T if len(pc_tof_raw) else np.empty((0,3))

        d_avg    = (frame.get("d_tof",1000) + frame.get("d_rs",1000)) / 2
        patch_sz = max(100., float(d_avg) * 0.25)
        frust_d  = 1000
        axis_sc  = max(30., float(d_avg) * 0.06)

        # ── RS cloud (azzurro) ───────────────────────────────────────────────
        if len(pc_rs_s):
            ax.scatter(pc_rs_s[:,0],pc_rs_s[:,2],-pc_rs_s[:,1], c="steelblue", s=3, alpha=0.30,
                       label=f"RS ({len(pc_rs_s)} pt)")

        # ── ToF cloud @ origine (T=I, oro) ───────────────────────────────────
        if len(pc_tof_raw):
            ax.scatter(pc_tof_raw[:,0],pc_tof_raw[:,2],-pc_tof_raw[:,1], c="gold", s=40, alpha=0.60,
                       label=f"ToF @ origine ({len(pc_tof_raw)} pt)", zorder=4)
        
        # ── ToF cloud ruotato────────────────────
        if len(pc_tof_ruot):
            ax.scatter(pc_tof_ruot[:,0],pc_tof_ruot[:,2],-pc_tof_ruot[:,1], c="green", s=50, alpha=0.95,
                       label=f"ToF ruotato ({len(pc_tof_ruot)} pt)", zorder=5)
        
        # ── ToF cloud trasformato con T corrente (rosso) ─────────────────────
        if len(pc_tof_rs):
            ax.scatter(pc_tof_rs[:,0],pc_tof_rs[:,2],-pc_tof_rs[:,1], c="orangered", s=50, alpha=0.95,
                       label=f"ToF→RS ({len(pc_tof_rs)} pt)", zorder=5)

        # ── helper al volo per riordinare gli assi dei vertici (X, Z, -Y) ────
        def _swap_verts(verts):
            return [[[p[0], p[2], -p[1]] for p in face] for face in verts]

        # ── patch piani ──────────────────────────────────────────────────────
        verts_rs = _swap_verts(_plane_verts(n_rs, c_rs, patch_sz))
        ax.add_collection3d(Poly3DCollection(
            verts_rs, alpha=0.22, facecolor="steelblue", edgecolor="darkblue", linewidth=1))
        
        verts_tof = _swap_verts(_plane_verts(n_tof_rs, c_tof_rs, patch_sz))
        ax.add_collection3d(Poly3DCollection(
            verts_tof, alpha=0.22, facecolor="tomato", edgecolor="darkred", linewidth=1))

        # ── normali dei piani ────────────────────────────────────────────────
        arr = patch_sz * 0.40
        # ax.quiver(X, Y, Z,  U, V, W) -> le prime 3 sono l'origine, le ultime 3 la direzione
        ax.quiver(c_rs[0], c_rs[2], -c_rs[1], 
                  n_rs[0]*arr, n_rs[2]*arr, -n_rs[1]*arr, 
                  color="darkblue", arrow_length_ratio=0.25, linewidth=2, label="n_RS")
        
        ax.quiver(c_tof_rs[0], c_tof_rs[2], -c_tof_rs[1], 
                  n_tof_rs[0]*arr, n_tof_rs[2]*arr, -n_tof_rs[1]*arr, 
                  color="darkred", arrow_length_ratio=0.25, linewidth=2, label="n_ToF→RS")

        # ── centroidi + linea di errore ──────────────────────────────────────
        # Applichiamo lo swap direttamente ai centroidi per i punti
        c_rs_p  = [c_rs[0], c_rs[2], -c_rs[1]]
        c_tof_p = [c_tof_rs[0], c_tof_rs[2], -c_tof_rs[1]]
        
        # Ora possiamo usare l'asterisco perché le liste sono già nell'ordine corretto
        ax.scatter(*c_rs_p, c="darkblue", s=90, marker="*", zorder=6)
        ax.scatter(*c_tof_p, c="darkred", s=90, marker="*", zorder=6)
        
        # Disegnamo la linea unendo le coordinate X con X, Y con Y, e Z con Z dei punti scambiati
        ax.plot([c_rs_p[0], c_tof_p[0]],
                [c_rs_p[1], c_tof_p[1]],
                [c_rs_p[2], c_tof_p[2]], "k--", lw=1, alpha=0.6)


        _draw_frustum(ax, np.zeros(3), np.eye(3),
                          self.K_rs, frust_d, self.W, self.H,
                          color="steelblue", alpha=0.12)
        _draw_sensor_frame(ax, np.zeros(3),  np.eye(3),       axis_sc,
                           labels=("Xrs","Yrs","Zrs"), alpha=0.9)

        _draw_tof_frustum(ax, (tof_orig), (R),
                          frust_d, color="tomato", alpha=0.12)
        _draw_sensor_frame(ax, (tof_orig), (R),  axis_sc,
                           labels=("Xtof","Ytof","Ztof"),
                           alpha=0.85, linestyle="--")

        # ── titolo con R e t ─────────────────────────────────────────────────
        eu = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
        rt = (f"Rx={eu[0]:.1f}° Ry={eu[1]:.1f}° Rz={eu[2]:.1f}°"
                  f"   t=[{t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}] mm")
        ae = frame.get("ang_err", float("nan"))
        ax.set_title(rt + (f"  |  err ang {ae:.2f}°" if np.isfinite(ae) else ""),
                     fontsize=8)

# ── aggregazione errore nuvola-piano precalcolato ─────────────
        err_strings = []
        
        # Estraiamo gli array di distanze (max 64 valori) salvati nel frame
        err_raw = frame.get("err_raw")
        err_rot = frame.get("err_rot")
        err_rt  = frame.get("err_rt")
        
        # 2a. Statistiche Pre-rotazione
        if err_raw is not None and len(err_raw) > 0:
            arr_raw = np.array(err_raw)
            mean_raw = np.mean(arr_raw)
            rmse_raw = np.sqrt(np.mean(arr_raw**2))
            err_strings.append(f"Pre-Allineamento : Med={mean_raw:6.1f}mm | RMSE={rmse_raw:6.1f}mm")
            
        # 2b. Statistiche Post-rotazione
        if err_rot is not None and len(err_rot) > 0:
            arr_rot = np.array(err_rot)
            mean_rot = np.mean(arr_rot)
            rmse_rot = np.sqrt(np.mean(arr_rot**2))
            err_strings.append(f"Post-Rotazione   : Med={mean_rot:6.1f}mm | RMSE={rmse_rot:6.1f}mm")
            
        # 2c. Statistiche Post-rototraslazione
        if err_rt is not None and len(err_rt) > 0:
            arr_rt = np.array(err_rt)
            mean_rt = np.mean(arr_rt)
            rmse_rt = np.sqrt(np.mean(arr_rt**2))
            err_strings.append(f"Post-Rototrasl.  : Med={mean_rt:6.1f}mm | RMSE={rmse_rt:6.1f}mm")
        
        # 3. Stampa a schermo
        if err_strings:
            final_text = "Err Punto-Piano ToF -> RS:\n" + "\n".join(err_strings)
            
            # ── stampa a schermo in basso a sinistra ─────────────────────────
            ax.text2D(0.02, 0.02, 
                      final_text, 
                      transform=ax.transAxes, 
                      fontsize=9, 
                      color="black",
                      fontweight="bold",
                      family="monospace",
                      bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray', boxstyle='round,pad=0.4'),
                      zorder=10)
            
        # ── scaling isometrico ───────────────────────────────────────────────
        anchor_p = np.array([[0,0,0], c_rs, c_tof_rs, tof_orig])
        clouds_p = [p for p in (pc_rs_s, pc_tof_rs, pc_tof_raw) if len(p)]
        all_pts  = np.vstack(clouds_p + [anchor_p]) if clouds_p else anchor_p
        
        # Riorganizziamo gli assi di TUTTI i punti per il calcolo del bounding box (X, Z, -Y)
        all_pts_swapped = np.column_stack((all_pts[:, 0], all_pts[:, 2], -all_pts[:, 1]))
        
        # Passiamo i punti riordinati alla funzione di scaling
        _set_equal_aspect_3d(ax, all_pts_swapped)

        # Aggiorniamo i nomi degli assi per riflettere il nuovo orientamento
        ax.set_xlabel("X [mm]",  fontsize=8)
        ax.set_ylabel("Z [mm]", fontsize=8)
        ax.set_zlabel("Y [mm]", fontsize=8)
        
        ax.legend(loc="upper left", fontsize=7)
        ax.tick_params(labelsize=7)
        self.canvas3d.draw_idle()


    # ── FoV ToF proiettato sull'immagine depth ────────────────────────────────

    def _update_fov_projection(self, frame_idx: int | None = None) -> None:
        """Mostra il depth della camera con il FoV ToF 8×8 proiettato sopra.

        Ogni zona è disegnata come un poligono nell'immagine:
          - colore JET = distanza ToF misurata (stessa scala della colorbar)
          - label = distanza in mm
          - zona non valida = bordo tratteggiato grigio, nessun riempimento
        Il bordo giallo indica il limite esterno del FoV (45°×45°).
        """
        import matplotlib.cm       as _mcm       # noqa: PLC0415
        import matplotlib.patches  as _mpatch    # noqa: PLC0415

        ax = self.ax_fov
        ax.cla()

        idx   = frame_idx if frame_idx is not None else self.current_idx
        frame = self.frames[min(idx, len(self.frames) - 1)]

        R, t = self.T[:3, :3], self.T[:3, 3]
        K    = self.K_rs
        fx, fy, cx_k, cy_k = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        # ── carica depth ──────────────────────────────────────────────────────
        dep_path = frame.get("dep_path", "")
        if not dep_path or not os.path.exists(dep_path):
            ax.text(0.5, 0.5, "Depth image non disponibile",
                    ha="center", va="center", transform=ax.transAxes, fontsize=12)
            self.canvas_fov.draw_idle()
            return

        dep_raw = cv2.imread(dep_path, cv2.IMREAD_ANYDEPTH)
        if dep_raw is None:
            return

        h, w = dep_raw.shape
        dep_f   = dep_raw.astype(np.float32) * self.depth_scale
        dep_u8  = np.clip(dep_f / COLORBAR_MAX_MM * 255, 0, 255).astype(np.uint8)
        dep_rgb = cv2.cvtColor(cv2.applyColorMap(dep_u8, cv2.COLORMAP_JET),
                               cv2.COLOR_BGR2RGB)
        ax.imshow(dep_rgb, origin="upper", extent=[0, w, h, 0])

        # ── dati ToF ─────────────────────────────────────────────────────────
        dist_8x8  = np.asarray(frame.get("dist_8x8",  np.full((8, 8), np.nan)))
        valid_8x8 = np.asarray(frame.get("valid_8x8", np.zeros((8, 8), bool)))

        ok_dists = dist_8x8[valid_8x8 & np.isfinite(dist_8x8)]
        ref_dist = float(ok_dists.mean()) if len(ok_dists) else 1000.0

        half_h = np.radians(45.0 / 2.0)
        half_v = np.radians(45.0 / 2.0)

        def _proj(col_f: float, row_f: float, dist: float):
            """Proietta un angolo del grid ToF nell'immagine depth.
            col_f, row_f ∈ [0, 8]: coordinate angolari del grid (0=bordo sinistro/alto,
            8=bordo destro/basso; il centro del grid è a 4.0).
            dist: Z-depth in mm (distanza perpendicolare al piano immagine ToF)."""
            ax_ = np.tan((col_f - 4.0) / 4.0 * half_h)
            ay_ = np.tan((row_f - 4.0) / 4.0 * half_v)
            pt  = R @ (np.array([ax_, ay_, 1.0]) * dist) + t
            if pt[2] <= 0:
                return None
            return float(fx * pt[0] / pt[2] + cx_k), float(fy * pt[1] / pt[2] + cy_k)

        jet = _mcm.get_cmap("jet")

        # ── disegna le 8×8 zone ───────────────────────────────────────────────
        for row in range(8):
            for col in range(8):
                d_zone  = dist_8x8[row, col]
                is_ok   = bool(valid_8x8[row, col]) and np.isfinite(d_zone)
                d_draw  = d_zone if is_ok else ref_dist

                # 4 angoli della zona in coordinate grid (0–8)
                corners = [
                    _proj(col,     row,     d_draw),
                    _proj(col + 1, row,     d_draw),
                    _proj(col + 1, row + 1, d_draw),
                    _proj(col,     row + 1, d_draw),
                ]
                if any(c is None for c in corners):
                    continue

                norm_d = np.clip(d_draw / COLORBAR_MAX_MM, 0.0, 1.0)
                fc_rgb = jet(norm_d)[:3]

                poly = _mpatch.Polygon(
                    corners, closed=True,
                    facecolor=(*fc_rgb, 0.38 if is_ok else 0.0),
                    edgecolor="white" if is_ok else "#888888",
                    linewidth=0.9,
                    linestyle="-" if is_ok else "--",
                )
                ax.add_patch(poly)

                if is_ok:
                    cx_z = float(np.mean([c[0] for c in corners]))
                    cy_z = float(np.mean([c[1] for c in corners]))
                    ax.text(cx_z, cy_z, f"{d_zone:.0f}",
                            fontsize=5.5, ha="center", va="center",
                            color="white", fontweight="bold",
                            bbox=dict(facecolor="black", alpha=0.4, pad=0.4,
                                      boxstyle="round,pad=0.1"))

        # ── bordo esterno del FoV ─────────────────────────────────────────────
        # Proietta i 4 angoli del FoV totale alla distanza di riferimento
        fov_corners = [
            _proj(0, 0, ref_dist), _proj(8, 0, ref_dist),
            _proj(8, 8, ref_dist), _proj(0, 8, ref_dist),
        ]
        if all(c is not None for c in fov_corners):
            xs = [c[0] for c in fov_corners] + [fov_corners[0][0]]
            ys = [c[1] for c in fov_corners] + [fov_corners[0][1]]
            ax.plot(xs, ys, "y-", lw=2.2,
                    label=f"FoV ToF {45.0:.0f}°×{45.0:.0f}°")

        # ── assi e titolo ─────────────────────────────────────────────────────
        ax.set_xlim(0, w)
        ax.set_ylim(h, 0)    # origine in alto-sinistra come nelle immagini
        ax.set_xlabel("u [pixel]", fontsize=8)
        ax.set_ylabel("v [pixel]", fontsize=8)
        ax.set_title(
            f"FoV ToF proiettato su depth — frame {idx + 1}/{len(self.frames)}"
            f"   (d_ref = {ref_dist:.0f} mm)",
            fontsize=9,
        )
        ax.legend(fontsize=8, loc="upper right")
        ax.tick_params(labelsize=7)
        self.canvas_fov.draw_idle()

    # ── azioni pulsanti ───────────────────────────────────────────────────────

    def _on_toggle_exclude(self):
        idx = self.current_idx
        if idx in self.manual_excluded:
            self.manual_excluded.discard(idx)
        else:
            self.manual_excluded.add(idx)
        self.excluded = set(self.manual_excluded)   # auto-excl ricalcolate alla prossima calibrazione
        self._show_frame(idx)
        self._refresh_status_bar()

    def _on_recalibrate(self):
        """Legge tutti i parametri dalla GUI, rifittà i piani, applica i filtri
        e ricalibra: un solo pulsante, nessuna ambiguità."""
        method_key  = self.combo_fit.currentData()

        self.btn_recal.setEnabled(False)
        QApplication.processEvents()

        # ── 1. Refit piani con il metodo scelto ──────────────────────────────
        n_ok = n_fail = 0
        for f in self.frames:
            pc_tof = np.asarray(f.get("pc_tof", np.empty((0, 3))))
            pc_rs  = np.asarray(f.get("pc_rs_sample", np.empty((0, 3))))
            w_raw  = f.get("w_tof")
            w_tof  = np.asarray(w_raw) if w_raw is not None else None
            res_tof = fit_surface(pc_tof, method_key, weights=w_tof)
            res_rs  = fit_surface(pc_rs,  method_key)
            if res_tof is None or res_rs is None:
                n_fail += 1
                continue
            f["normal_tof"]   = res_tof["normal"];  f["d_tof"]   = res_tof["d"]
            f["centroid_tof"] = res_tof["centroid"]
            f["normal_rs"]    = res_rs["normal"];   f["d_rs"]    = res_rs["d"]
            f["centroid_rs"]  = res_rs["centroid"]; f["dist_diff_mm"] = abs(f["d_tof"] - f["d_rs"])
            n_ok += 1

        # ── 2. Auto-esclusione dai filtri GUI ────────────────────────────────
        auto_excl: set[int] = set()
        n_dist = n_tilt = 0
        for i, f in enumerate(self.frames):
            if f.get("dist_diff_mm", 0.0) > AUTO_EXCLUDE_DIST_MM:
                auto_excl.add(i);  n_dist += 1
            
        # auto-esclusi calcolati fresh ogni volta; gli esclusi manuali rimangono sempre
        self.excluded = self.manual_excluded | auto_excl

        active = [f for i, f in enumerate(self.frames) if i not in self.excluded]
        if len(active) < MIN_PLANE_PAIRS:
            self.lbl_calib.setText(f"⚠  Solo {len(active)} frame attivi — troppo pochi.")
            self.btn_recal.setEnabled(True)
            return

        QApplication.processEvents()

        calibrate_fn =  outer_ransac_calibrate
        try:
            self.T, self.diag = calibrate_fn(active)
            self.translation_cloud = self.diag.get("translation_cloud", np.empty((0, 3)))
            self._show_frame(self.current_idx)
            ae = self.diag["angular_errors"]
            t_std = self.diag["translation_std_mm"]

            self.lbl_calib.setText(
                f"✅ [{method_key}]  "
                f"{len(active)} frame  |  "
                f"Errore angolare median={np.median(ae):.2f}°  |  "
                f"T std=[{t_std[0]:.3f}, {t_std[1]:.3f}, {t_std[2]:.3f}] mm"
            )
        except Exception as exc:
            self.lbl_calib.setText(f"❌  Errore: {exc}")
        finally:
            self.btn_recal.setEnabled(True)

    def _on_reset_exclusions(self):
        """Rimuove tutte le esclusioni (manuali e automatiche) e aggiorna la vista."""
        self.manual_excluded.clear()
        self.excluded.clear()
        self._show_frame(self.current_idx)
        self._refresh_status_bar()

    def _on_save(self):
        R, t = self.T[:3, :3], self.T[:3, 3]
        # Chiave generica: "T_tof2cam"
        out  = {
            "T_tof2cam":         self.T.tolist(),
            "rotation":          R.tolist(),
            "translation_mm":    t.tolist(),
            "camera":            "RS",
            "n_frames_used":     len(self.frames) - len(self.excluded),
            "n_frames_excluded": len(self.excluded),
            "excluded_indices":  sorted(self.excluded),
        }
        path = os.path.join(self.session_dir, self.output_json)
        with open(path, "w") as fh:
            json.dump(out, fh, indent=2)
        self.lbl_calib.setText(f"✅  Salvato → {path}")

    def _refresh_status_bar(self):
        n_tot  = len(self.frames)
        n_excl = len(self.excluded)
        ae = self.diag["angular_errors"]
        t_std = self.diag["translation_std_mm"]

        self.lbl_calib.setText(
            f"{n_tot - n_excl}/{n_tot} frame  |  "
            f"Errore angolare median={np.median(ae):.2f}°  |  "
            f"T std=[{t_std[0]:.3f}, {t_std[1]:.3f}, {t_std[2]:.3f}] mm"
            "— premi 🔄 per ricalibrare"
        )
    
    def _update_error_heatmaps(self, diag: dict) -> None:
        """Disegna le heatmaps degli errori sui 3 subplot del Tab 3."""
        p_stats = diag.get("pixel_stats")
        if not p_stats:
            return

        # 1. Rimuovi la colorbar PRIMA di pulire gli assi!
        if hasattr(self, '_err_colorbar') and self._err_colorbar is not None:
            try:
                self._err_colorbar.remove()
            except Exception:
                pass
            self._err_colorbar = None

        # 2. Pulisce i vecchi grafici
        for ax in self.axes_err:
            ax.cla()
            ax.axis('off')

        titles = ["Pre-Allineamento (Raw)", "Post-Rotazione (R)", "Post-Rototraslazione (R+t)"]
        
        # 3. Formatta i dati in griglie 8x8 (ignorando i NaN se ci sono)
        grid_raw = np.array(p_stats["raw"]["rmse"], dtype=float).reshape(8, 8)
        grid_rot = np.array(p_stats["rot"]["rmse"], dtype=float).reshape(8, 8)
        grid_rt  = np.array(p_stats["rt"]["rmse"], dtype=float).reshape(8, 8)
        grids = [grid_raw, grid_rot, grid_rt]
        
        # 4. Calcola il max globale per uniformare la scala colori
        vmax = float(np.nanmax([grid_raw, grid_rot, grid_rt]))
        if np.isnan(vmax) or vmax <= 0.0:
            vmax = 1.0  # Fallback di sicurezza
        vmin = 0.0

        im = None
        for ax, grid, title in zip(self.axes_err, grids, titles):
            ax.set_title(title, fontsize=9, fontweight="bold")
            
            # ── NUOVA COLORMAP: "YlOrRd" (0 = Giallo chiaro, Max = Rosso scuro) ──
            im = ax.imshow(grid, cmap="YlOrRd", vmin=vmin, vmax=vmax)
            
            # Stampa i valori numerici
            for i in range(8):
                for j in range(8):
                    val = grid[i, j]
                    if not np.isnan(val):
                        # Con YlOrRd, il testo diventa bianco solo sui rossi molto scuri (alto errore)
                        color = "white" if val > (vmax * 0.7) else "black"
                        ax.text(j, i, f"{val:.1f}", ha="center", va="center", color=color, fontsize=7)
        # 5. Aggiunge la Colorbar condivisa IN BASSO
        if im is not None:
            self._err_colorbar = self.fig_err.colorbar(
                im, 
                ax=self.axes_err.tolist(), 
                location='bottom',
                shrink=0.5,         # Mantiene la larghezza al 50%
                pad=0.08,           # Spinge la barra più in basso rispetto ai plot
                aspect=25           # Rende la barra un po' più sottile ed elegante
            )
            self._err_colorbar.set_label("Errore RMSE [mm]", fontsize=9)
            self._err_colorbar.ax.tick_params(labelsize=8)

        # 6. Richiede l'aggiornamento del widget Qt
        self.canvas_err.draw_idle()

# ── avvio GUI ─────────────────────────────────────────────────────────────────

def launch_gui(plane_pairs: list[dict], T: np.ndarray,
               K_cam: np.ndarray, depth_scale_mm: float,
               session_dir: str, W: int, H: int,
               excluded: set[int] | None = None,
               output_json: str = "tof_rs_extrinsics.json",
               diag: dict | None = None) -> None:  # <--- Aggiunto diag qui
    app = QApplication.instance() or QApplication(sys.argv)
    win = CalibrationGUI(plane_pairs, T, K_cam, depth_scale_mm, session_dir, W, H,
                         excluded=excluded, output_json=output_json, diag=diag) # <--- Passato qui
    win.show()
    sys.exit(app.exec_())

# =============================================================================
# 11. MAIN
# =============================================================================

def main():
    if not os.path.isdir(SESSION_DIR):
        print(f"ERRORE: SESSION_DIR non trovata:\n  {SESSION_DIR}")
        sys.exit(1)


    print(f"Sessione : {SESSION_DIR}")
    print(f"Camera   : {SESSION_CAMERA}")


    K_cam, dist_cam, depth_scale_mm, W, H = load_rs_config(SESSION_DIR)
    depth_frames = load_rs_depth_frames(SESSION_DIR)   # solo per visualizzazione GUI
    rgb_frames   = load_rs_rgb_frames_list(SESSION_DIR)
    output_json  = "tof_rs_extrinsics.json"


    print(f"Intrinsics K ({SESSION_CAMERA}):\n{K_cam}")
    print(f"Distortion : {dist_cam}")
    print(f"Depth scale: {depth_scale_mm} mm/unit (solo GUI)")
    print(f"AprilTag   : dict={CHARUCO_DICT} marker={CHARUCO_MARKER_MM:.2f}mm borderBits={MARKER_BORDER_BITS}")

    tof_frames = load_tof_frames(SESSION_DIR)
    print(f"Frame RS RGB  : {len(rgb_frames)}")
    print(f"Frame RS depth: {len(depth_frames)} (solo GUI)")
    print(f"Frame ToF     : {len(tof_frames)}")

    if not rgb_frames:
        print("ERRORE: nessun frame RGB trovato in RS/rgb/.")
        sys.exit(1)

    # Sincronizza ToF ↔ RGB (il piano RS arriva dal ChArUco rilevato nell'RGB)
    pairs = synchronize_frames(tof_frames, rgb_frames, TIMESTAMP_TOLERANCE_MS)
    print(f"Coppie sincronizzate (ToF↔RGB): {len(pairs)}")
    if not pairs:
        print("ERRORE: nessuna coppia. Aumenta TIMESTAMP_TOLERANCE_MS.")
        sys.exit(1)

    # Passo 1 — estrazione piani (parallela: ToF cloud + ChArUco pose per frame)
    plane_pairs = extract_plane_pairs(pairs, K_cam, dist_cam)

    # Associa path depth (per la visualizzazione nella GUI) a ogni frame, dal timestamp RGB
    if depth_frames:
        dep_times  = np.array([t for t, _ in depth_frames])
        dep_paths  = [p for _, p in depth_frames]
        for f in plane_pairs:
            rgb_ts = float(os.path.splitext(os.path.basename(f["rgb_path"]))[0]
                           .replace("RS_", "").replace("rgb_", "")) * 1000.0
            idx = int(np.argmin(np.abs(dep_times - rgb_ts)))
            f["dep_path"] = dep_paths[idx]
        print(f"Frame depth abbinati per GUI: {len(plane_pairs)}")
    else:
        for f in plane_pairs:
            f["dep_path"] = ""


    # Passo 2 — esclusione automatica + calibrazione
    auto_excluded: set[int] = set()
    reasons: list[str] = []

    for i, f in enumerate(plane_pairs):
        # differenza di distanza tra i due sensori
        dist_diff = abs(float(f["d_tof"]) - float(f["d_rs"]))

        if dist_diff > AUTO_EXCLUDE_DIST_MM:
            print(f"Δd={dist_diff:.0f}mm>{AUTO_EXCLUDE_DIST_MM}mm")
            auto_excluded.add(i)


    n_active = len(plane_pairs) - len(auto_excluded)
    print(f"\n[2/3] Esclusione automatica: {len(auto_excluded)} frame esclusi "
          f"({n_active} attivi)")
    diag = None  # Inizializziamo la variabile

    if n_active >= MIN_PLANE_PAIRS:
        active_pairs = [f for i, f in enumerate(plane_pairs)
                        if i not in auto_excluded]
        print(f"\n[3/3] Calibrazione su {n_active} frame attivi...")
        T, diag = outer_ransac_calibrate(active_pairs)
        ae = diag["angular_errors"]
        t_std = diag["translation_std_mm"]
        print(f"Err angolare median={np.median(ae):.2f}°  max={np.max(ae):.2f}°  |  "
               f"T std=[{t_std[0]:.3f}, {t_std[1]:.3f}, {t_std[2]:.3f}] mm")
        
    else:
        print(f"  ⚠ Troppo pochi frame attivi ({n_active}), "
              "mantengo la calibrazione iniziale senza esclusioni.")
        auto_excluded = set()
        # Calcoliamo comunque diag su tutti i frame per avere le heatmaps!
        _, diag = outer_ransac_calibrate(plane_pairs)

    print(f"\nT_tof2{SESSION_CAMERA.lower()}:\n{T}")

    # Passo 4 — GUI interattiva
    print(f"\nAvvio GUI interattiva (camera={SESSION_CAMERA}, output={output_json})...")
    # Aggiungiamo diag come parametro!
    launch_gui(plane_pairs, T, K_cam, depth_scale_mm, SESSION_DIR,
               excluded=auto_excluded, output_json=output_json, diag=diag, W=W, H=H)


if __name__ == "__main__":
    main()