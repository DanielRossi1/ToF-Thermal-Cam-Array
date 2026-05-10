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
#define USE_TCP_TRANSPORT   1
#define TCP_LISTEN_PORT     9000

// Fallback UART (unused when USE_TCP_TRANSPORT=1)
#define UART_DEVICE_PATH    "/dev/ttyS0"

// Camera sync (optional external trigger)
#define USE_CAM_SYNC        0

#if USE_CAM_SYNC
#define PIN_CAM_SYNC_OUT    57
#define PIN_CAM_SYNC_IN     58
#endif

// UVC camera via V4L2
#define USE_UVC_CAMERA      1
#ifndef UVC_AUTOSTART
#define UVC_AUTOSTART  0
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