#pragma once

#include <stdint.h>

// Hardware pins (ESP32-S3)
static constexpr int kPinSda = 8;
static constexpr int kPinScl = 9;

static constexpr int kPinTofInt = 4;  // active-low data-ready
static constexpr int kPinTofLpn = 5;  // optional (some breakouts tie it high)

// IMPORTANT: GPIO19/20 are USB D-/D+ on ESP32-S3.
// In your hardware, the global-shutter UVC camera uses these lines.
// Do NOT use GPIO19/20 as normal GPIO while using native USB.

// UVC camera feature toggle.
// Set to 0 to build/test ToF+MLX only.
#ifndef HUB_USE_UVC_CAMERA
#define HUB_USE_UVC_CAMERA 1
#endif

// If enabled, UVC start is attempted during setup().
// Keep disabled by default so ToF/MLX debugging is always available even if
// a specific camera triggers USB host enum crashes.
#ifndef HUB_UVC_AUTOSTART
#define HUB_UVC_AUTOSTART 1
#endif

// Optional external camera sync (separate GPIOs, NOT 19/20).
// Disabled by default.
#ifndef HUB_USE_CAM_SYNC
#define HUB_USE_CAM_SYNC 0
#endif
#if HUB_USE_CAM_SYNC
static constexpr int kPinCamSyncOut = -1;  // set to a real GPIO if used
static constexpr int kPinCamSyncIn = -1;   // set to a real GPIO if used
#endif

// Link / performance
static constexpr uint32_t kSerialBaud = 2000000;  // CH343p UART (right USB-C)

// Sensor sizing
static constexpr uint8_t kTofZones = 64;
static constexpr uint8_t kTofMaxTargetsPerZone = 4;  // VL53L8CH supports up to 4 targets/zone

static constexpr uint16_t kMlxW = 32;
static constexpr uint16_t kMlxH = 24;
static constexpr uint16_t kMlxPixels = kMlxW * kMlxH;  // 768

// Camera transport sizing (when embedding JPEG in frames)
static constexpr uint32_t kCamJpegMax = 55u * 1024u;

// I2C tuning
static constexpr uint32_t kI2cClockHz = 400000;  // safe default; can be increased by command
static constexpr uint16_t kI2cBufSize = 1024;     // needed for VL53L8CH firmware upload
