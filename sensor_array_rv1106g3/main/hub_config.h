#pragma once

#include <stdint.h>

// ── Luckfox RV1106G3 hardware pins ─────────────────────────────────────────
// I2C bus on Luckfox Pico (check with: i2cdetect -l)
#define I2C_DEVICE_PATH     "/dev/i2c-3"

// ToF VL53L8CH — adjust GPIO numbers to your wiring
// Luckfox Pico GPIO layout (from /sys/kernel/debug/gpio):
//   gpio0: 0-31   gpio1: 32-63   gpio2: 64-95
//   gpio3: 96-127 gpio4: 128-151
#define PIN_TOF_INT         55   // GPIO1_C7 (GPIO1 bank, offset 23 = number 32+23=55)
#define PIN_TOF_LPN         56   // GPIO1_D0

// ── Transport: use TCP instead of UART (Luckfox has no exposed UART) ───────
#ifndef USE_TCP_TRANSPORT
#define USE_TCP_TRANSPORT   1
#endif
#ifndef TCP_LISTEN_PORT
#define TCP_LISTEN_PORT     9000
#endif

// Fallback UART (unused when USE_TCP_TRANSPORT=1)
#define UART_DEVICE_PATH    "/dev/ttyS0"

// Camera sync (optional external trigger)
#ifndef USE_CAM_SYNC
#define USE_CAM_SYNC        0
#endif

#if USE_CAM_SYNC
#define PIN_CAM_SYNC_OUT    57
#define PIN_CAM_SYNC_IN     58
#endif

// UVC camera via V4L2
#ifndef USE_UVC_CAMERA
#define USE_UVC_CAMERA      1
#endif
#ifndef UVC_AUTOSTART
#define UVC_AUTOSTART  0
#endif

// UVC defaults (used for autostart + initial settings)
#ifndef UVC_DEVICE_PATH
#define UVC_DEVICE_PATH     "/dev/video0"
#endif
#ifndef UVC_DEFAULT_W
#define UVC_DEFAULT_W       640
#endif
#ifndef UVC_DEFAULT_H
#define UVC_DEFAULT_H       480
#endif
// Camera capture cadence. Using 30 FPS helps reduce camera-vs-ToF skew when sampling at 15 Hz.
#ifndef UVC_DEFAULT_INTERVAL_US
#define UVC_DEFAULT_INTERVAL_US  33333u
#endif

// When sampling is paced by ToF IRQ, optionally wait briefly for a camera frame with ts >= ToF ts.
#ifndef UVC_SNAPSHOT_WAIT_MS
#define UVC_SNAPSHOT_WAIT_MS  20u
#endif

// ── Link / performance ─────────────────────────────────────────────────────
#define SERIAL_BAUD         2000000

// ── Sensor sizing ──────────────────────────────────────────────────────────
#define TOF_ZONES            64
#define TOF_MAX_TARGETS_PER_ZONE  4

#define MLX_W                32
#define MLX_H                24
#define MLX_PIXELS           (MLX_W * MLX_H)

// ── Camera transport sizing
#define CAM_JPEG_MAX        (55u * 1024u)

// ── I2C tuning
#define I2C_CLOCK_HZ        400000