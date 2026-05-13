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

# Simple CRC32 matching C implementation
_crc_table = (
    0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,
    0xE963A535,0x9E6495A3,0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,
    0x09B64C2B,0x7EB17CBD,0xE7B82D07,0x90BF1D91,0x1DB71064,0x6AB020F2,
    0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
    0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,
    0xFA0F3D63,0x8D080DF5,0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,
    0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,0x35B5A8FA,0x42B2986C,
    0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
    0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,
    0xCFBA9599,0xB8BDA50F,0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,
    0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,0x76DC4190,0x01DB7106,
    0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
    0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,0x086D3D2D,
    0x91646C97,0xE6635C01,0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,
    0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,0x65B0D9C6,0x12B7E950,
    0x8BBEB8EA,0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
    0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,
    0xA4D1C46D,0xD3D6F4FB,0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,
    0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,0x5005713C,0x270241AA,
    0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
    0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8B4,0x59B33D17,0x2EB40D81,
    0xB7BD5C3B,0xC0BA6CAD,0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,
    0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,0xE3630B12,0x94643B84,
    0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
    0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,
    0x196C3671,0x6E6B06E7,0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,
    0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,0xD6D6A3E8,0xA1D1937E,
    0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
    0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,
    0x316E8EEF,0x4669BE79,0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,
    0xCC0C7795,0xBB0B4703,0x220216B9,0x5505262F,0xC5BA3BBE,0xB2BD0B28,
    0x2BB45A92,0x5CB36A04,0xC2D7FFA7,0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,
    0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,0x9C0906A9,0xEB0E363F,
    0x72076785,0x05005713,0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,
    0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,0x86D3D2D4,0xF1D4E242,
    0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
    0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,
    0x616BFFD3,0x166CCF45,0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,
    0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,0xAED16A4A,0xD9D65ADC,
    0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
    0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,
    0x54DE5729,0x23D967BF,0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,
    0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
)

def simple_crc32_python(data: bytes, crc: int = 0) -> int:
    crc ^= 0xFFFFFFFF
    for b in data:
        crc = _crc_table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF
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

_crc_mismatch_count = 0

def parse_message(raw: bytes):
    """
    Parse a decoded SLIP frame.
    Returns (msg_type, seq, ts_us, payload_bytes) or None on error.
    """
    global _crc_mismatch_count
    if len(raw) < HDR_SIZE + CRC_SIZE:
        print(f"[PARSE-DBG] short packet: len={len(raw)} need>={HDR_SIZE+CRC_SIZE}")
        return None

    try:
        magic, ver, mtype, seq, ts_us, plen = struct.unpack_from(HDR_FMT, raw, 0)
    except struct.error:
        print("[PARSE-DBG] unpack header failed")
        return None

    if magic != MAGIC or ver != VERSION:
        print(f"[PARSE-DBG] bad magic/ver: magic=0x{magic:08X} ver={ver}")
        return None

    total = HDR_SIZE + plen + CRC_SIZE
    if len(raw) != total:
        print(f"[PARSE-DBG] length mismatch: hdr.plen={plen} hdr_total={total} raw_len={len(raw)} seq={seq}")
        return None

    payload = raw[HDR_SIZE:HDR_SIZE + plen]
    try:
        crc_rx, = struct.unpack_from('<I', raw, HDR_SIZE + plen)
    except struct.error:
        print("[PARSE-DBG] unpack crc failed")
        return None

    crc_zlib = zlib.crc32(raw[:HDR_SIZE + plen]) & 0xFFFFFFFF
    if crc_zlib != crc_rx:
        _crc_mismatch_count += 1
        # Printing per packet can destroy throughput; throttle aggressively.
        if _crc_mismatch_count <= 5 or (_crc_mismatch_count % 200) == 0:
            crc_simple = simple_crc32_python(raw[:HDR_SIZE + plen]) & 0xFFFFFFFF
            print(
                f"[PARSE-DBG] crc mismatch #{_crc_mismatch_count}: "
                f"zlib=0x{crc_zlib:08X} simple=0x{crc_simple:08X} rx=0x{crc_rx:08X} "
                f"type={mtype} seq={seq} plen={plen}"
            )
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
