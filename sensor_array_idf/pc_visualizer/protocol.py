"""
Hub protocol decoder.

Mirrors hub_protocol.h + hub_frame.h exactly.
VL53L8CX_NB_TARGET_PER_ZONE = 4
TOF_ZONES = 64
MLX_PIXELS = 768 (24×32)
"""

import struct
import zlib
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

# ── SLIP constants ─────────────────────────────────────────────────────────────
SLIP_END     = 0xC0
SLIP_ESC     = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

# ── Protocol constants ────────────────────────────────────────────────────────
MAGIC   = 0x53454E53  # 'SENS' little-endian
VERSION = 1

MSG_FRAME = 1
MSG_CMD   = 2
MSG_RESP  = 3
MSG_EVENT = 4

# Sensor layout
TOF_ZONES    = 64
TOF_TPZ      = 4   # targets per zone
MLX_W, MLX_H = 32, 24
MLX_PIXELS   = MLX_W * MLX_H

# Frame flags
FLAG_TOF_VALID      = 1 << 0
FLAG_MLX_VALID      = 1 << 1
FLAG_CAM_VALID      = 1 << 2
FLAG_CAM_SYNC_VALID = 1 << 3

# MsgHeader: magic(4) version(2) type(2) seq(4) ts_us(8) payload_len(4) = 24 bytes
HDR_FMT  = '<IHHIQI'   # I(4)+H(2)+H(2)+I(4)+Q(8)+I(4) = 24
HDR_SIZE = struct.calcsize(HDR_FMT)
CRC_SIZE = 4
assert HDR_SIZE == 24

# TofConfigV1: side(1) tpz(1) hz(2) it_ms(2) reserved(2) = 8 bytes
TOF_CFG_FMT  = '<BBHHH'
TOF_CFG_SIZE = struct.calcsize(TOF_CFG_FMT)

# TofDataV1 full layout (packed):
#   ts_us(8) + cfg(8) + temp(1) + pad(3)
#   + nb_target(64) + nb_spads(64)
#   + distance_mm[256] int16 (512B)
#   + range_sigma[256] uint16 (512B)
#   + status[256] uint8 (256B)
#   + reflectance[256] uint8 (256B)
#   + signal[256] uint32 (1024B)
#   + ambient[64] uint32 (256B)
TOF_N = TOF_ZONES * TOF_TPZ  # 256

def _tof_fmt():
    # ts_us  cfg_side cfg_tpz cfg_hz cfg_it cfg_res  temp pad
    parts = ['Q', 'BB', 'HH', 'H', 'B', '3s']
    # nb_target, nb_spads
    parts += [f'{TOF_ZONES}B', f'{TOF_ZONES}B']
    # distance_mm (int16), sigma (uint16), status (uint8), reflectance (uint8)
    parts += [f'{TOF_N}h', f'{TOF_N}H', f'{TOF_N}B', f'{TOF_N}B']
    # signal_per_spad (uint32), ambient_per_spad (uint32, zone-level)
    parts += [f'{TOF_N}I', f'{TOF_ZONES}I']
    return '<' + ''.join(parts)

TOF_FMT  = _tof_fmt()
TOF_SIZE = struct.calcsize(TOF_FMT)

# MlxDataV1: ts_us(8) cfg{w(2)h(2)mode(1)res(1)refresh(1)reserved(1)}(8) ta_cC(2) vdd(2) frame_cC[768](1536) = 1556
# [0]=ts [1]=w [2]=h [3]=mode [4]=res [5]=refresh  x=pad  [6]=ta_cC [7]=vdd  [8..775]=pixels
MLX_FMT  = f'<Q HH BBBx hh {MLX_PIXELS}h'
MLX_SIZE = struct.calcsize(MLX_FMT)

# CamSyncV1: two uint64 + two uint32
CAM_SYNC_FMT  = '<QQ II'
CAM_SYNC_SIZE = struct.calcsize(CAM_SYNC_FMT)

# CamDataV1: ts_us(8) + cfg w(4) h(4) fourcc(4) + len(4) = 24
CAM_DATA_FMT  = '<Q III I'
CAM_DATA_SIZE = struct.calcsize(CAM_DATA_FMT)

# FrameFixedV1 header (before cam bytes):
#   frame_seq(4) hub_ts_us(8) flags(4) reserved(4) + tof + mlx + cam_sync + cam_data
FRAME_HDR_FMT  = '<I Q II'
FRAME_HDR_SIZE = struct.calcsize(FRAME_HDR_FMT)
FRAME_FIXED_SIZE = FRAME_HDR_SIZE + TOF_SIZE + MLX_SIZE + CAM_SYNC_SIZE + CAM_DATA_SIZE


# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class TofFrame:
    ts_us:             int = 0
    side:              int = 8
    targets_per_zone:  int = 4
    ranging_hz:        int = 0
    integration_ms:    int = 0
    silicon_temp:      int = 0
    nb_targets:        np.ndarray = field(default_factory=lambda: np.zeros(TOF_ZONES, np.uint8))
    nb_spads:          np.ndarray = field(default_factory=lambda: np.zeros(TOF_ZONES, np.uint8))
    distance_mm:       np.ndarray = field(default_factory=lambda: np.zeros((TOF_ZONES, TOF_TPZ), np.int16))
    sigma_mm:          np.ndarray = field(default_factory=lambda: np.zeros((TOF_ZONES, TOF_TPZ), np.uint16))
    status:            np.ndarray = field(default_factory=lambda: np.zeros((TOF_ZONES, TOF_TPZ), np.uint8))
    reflectance:       np.ndarray = field(default_factory=lambda: np.zeros((TOF_ZONES, TOF_TPZ), np.uint8))
    signal_per_spad:   np.ndarray = field(default_factory=lambda: np.zeros((TOF_ZONES, TOF_TPZ), np.uint32))
    ambient_per_spad:  np.ndarray = field(default_factory=lambda: np.zeros(TOF_ZONES, np.uint32))

@dataclass
class MlxFrame:
    ts_us:      int = 0
    w:          int = MLX_W
    h:          int = MLX_H
    ta_celsius: float = 0.0
    pixels_c:   np.ndarray = field(default_factory=lambda: np.zeros(MLX_PIXELS, np.float32))

@dataclass
class SyncedFrame:
    seq:        int = 0
    hub_ts_us:  int = 0
    flags:      int = 0
    tof:        Optional[TofFrame] = None
    mlx:        Optional[MlxFrame] = None
    cam_jpeg:   Optional[bytes] = None
    cam_w:      int = 0
    cam_h:      int = 0
    cam_ts_us:  int = 0


# ── SLIP decoder ──────────────────────────────────────────────────────────────

class SlipDecoder:
    def __init__(self, callback):
        self._cb  = callback
        self._buf = bytearray()
        self._esc = False

    def feed(self, data: bytes):
        for b in data:
            if b == SLIP_END:
                if self._buf:
                    self._cb(bytes(self._buf))
                self._buf.clear()
                self._esc = False
            elif self._esc:
                self._esc = False
                if   b == SLIP_ESC_END: self._buf.append(SLIP_END)
                elif b == SLIP_ESC_ESC: self._buf.append(SLIP_ESC)
                # else: invalid — drop
            elif b == SLIP_ESC:
                self._esc = True
            else:
                self._buf.append(b)

    def reset(self):
        self._buf.clear()
        self._esc = False


# ── Message builder (CMD) ─────────────────────────────────────────────────────

_cmd_seq = 0

def build_cmd(text: str) -> bytes:
    global _cmd_seq
    _cmd_seq += 1
    payload_bytes = text.encode('utf-8')
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, MSG_CMD, _cmd_seq, 0, len(payload_bytes))
    crc = zlib.crc32(hdr + payload_bytes) & 0xFFFFFFFF
    raw = hdr + payload_bytes + struct.pack('<I', crc)
    return _slip_encode(raw)

def _slip_encode(data: bytes) -> bytes:
    out = bytearray([SLIP_END])
    for b in data:
        if   b == SLIP_END: out += bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out += bytes([SLIP_ESC, SLIP_ESC_ESC])
        else:               out.append(b)
    out.append(SLIP_END)
    return bytes(out)


# ── Frame parser ──────────────────────────────────────────────────────────────

def parse_message(raw: bytes):
    """
    Parse a decoded SLIP frame.
    Returns (msg_type, seq, ts_us, payload_bytes) or None on error.
    """
    if len(raw) < HDR_SIZE + CRC_SIZE:
        return None
    magic, ver, mtype, seq, ts_us, plen = struct.unpack_from(HDR_FMT, raw, 0)
    if magic != MAGIC or ver != VERSION:
        return None
    total = HDR_SIZE + plen + CRC_SIZE
    if len(raw) != total:
        return None
    payload = raw[HDR_SIZE:HDR_SIZE + plen]
    crc_rx, = struct.unpack_from('<I', raw, HDR_SIZE + plen)
    crc = zlib.crc32(raw[:HDR_SIZE + plen]) & 0xFFFFFFFF
    if crc != crc_rx:
        return None
    return mtype, seq, ts_us, payload


def parse_frame(payload: bytes) -> Optional[SyncedFrame]:
    if len(payload) < FRAME_FIXED_SIZE:
        return None

    off = 0
    seq, hub_ts, flags, _res = struct.unpack_from(FRAME_HDR_FMT, payload, off)
    off += FRAME_HDR_SIZE

    sf = SyncedFrame(seq=seq, hub_ts_us=hub_ts, flags=flags)

    # ── ToF ──────────────────────────────────────────────────────────────────
    tof_vals = struct.unpack_from(TOF_FMT, payload, off)
    off += TOF_SIZE
    if flags & FLAG_TOF_VALID:
        tf = TofFrame()
        i = 0
        tf.ts_us            = tof_vals[i]; i+=1
        tf.side             = tof_vals[i]; i+=1
        tf.targets_per_zone = tof_vals[i]; i+=1
        tf.ranging_hz       = tof_vals[i]; i+=1
        tf.integration_ms   = tof_vals[i]; i+=1
        # cfg reserved
        i += 1
        tf.silicon_temp     = tof_vals[i]; i+=1
        # 3 pad bytes as one token
        i += 1
        tf.nb_targets       = np.array(tof_vals[i:i+TOF_ZONES], np.uint8);  i+=TOF_ZONES
        tf.nb_spads         = np.array(tof_vals[i:i+TOF_ZONES], np.uint8);  i+=TOF_ZONES
        tf.distance_mm      = np.array(tof_vals[i:i+TOF_N],     np.int16).reshape(TOF_ZONES, TOF_TPZ);  i+=TOF_N
        tf.sigma_mm         = np.array(tof_vals[i:i+TOF_N],     np.uint16).reshape(TOF_ZONES, TOF_TPZ); i+=TOF_N
        tf.status           = np.array(tof_vals[i:i+TOF_N],     np.uint8).reshape(TOF_ZONES, TOF_TPZ);  i+=TOF_N
        tf.reflectance      = np.array(tof_vals[i:i+TOF_N],     np.uint8).reshape(TOF_ZONES, TOF_TPZ);  i+=TOF_N
        tf.signal_per_spad  = np.array(tof_vals[i:i+TOF_N],     np.uint32).reshape(TOF_ZONES, TOF_TPZ); i+=TOF_N
        tf.ambient_per_spad = np.array(tof_vals[i:i+TOF_ZONES], np.uint32); i+=TOF_ZONES
        sf.tof = tf

    # ── MLX ──────────────────────────────────────────────────────────────────
    mlx_vals = struct.unpack_from(MLX_FMT, payload, off)
    off += MLX_SIZE
    if flags & FLAG_MLX_VALID:
        mf = MlxFrame()
        mf.ts_us      = mlx_vals[0]
        # cfg: w=mlx_vals[1] h=mlx_vals[2] mode=mlx_vals[3] res=mlx_vals[4] refresh=mlx_vals[5]
        mf.w          = mlx_vals[1]
        mf.h          = mlx_vals[2]
        mf.ta_celsius = mlx_vals[6] / 100.0   # ta_cC: [0]=ts [1]=w [2]=h [3]=mode [4]=res [5]=refresh x=pad [6]=ta
        raw_cC        = np.array(mlx_vals[8:8+MLX_PIXELS], np.int16)
        mf.pixels_c   = raw_cC.astype(np.float32) / 100.0
        sf.mlx = mf

    # ── CamSync (skip) ───────────────────────────────────────────────────────
    off += CAM_SYNC_SIZE

    # ── Camera ───────────────────────────────────────────────────────────────
    cam_ts, cam_w, cam_h, cam_fourcc, cam_len = struct.unpack_from(CAM_DATA_FMT, payload, off)
    off += CAM_DATA_SIZE
    if (flags & FLAG_CAM_VALID) and cam_len > 0:
        sf.cam_jpeg  = payload[off:off + cam_len]
        sf.cam_w     = cam_w
        sf.cam_h     = cam_h
        sf.cam_ts_us = cam_ts

    return sf
