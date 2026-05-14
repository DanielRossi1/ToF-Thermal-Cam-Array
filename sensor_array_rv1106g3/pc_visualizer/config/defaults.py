# ── Defaults ───────────────────────────────────────────────────────────────────
DEFAULT_HOST  = '192.168.1.67'
DEFAULT_PORT  = 9000
DEFAULT_PROTO = 'TCP'

DEBUG_NET = False

TOF_MODES  = ['distance_mm', 'sigma_mm', 'signal_per_spad',
               'reflectance', 'status', 'ambient_per_spad', 'nb_targets']
COLORMAPS  = ['plasma', 'inferno', 'viridis', 'magma', 'turbo',
               'CET-L4', 'CET-D1', 'hot']

TOF_ZONES    = 64
TOF_TPZ      = 4   # targets per zone
MLX_W, MLX_H = 32, 24