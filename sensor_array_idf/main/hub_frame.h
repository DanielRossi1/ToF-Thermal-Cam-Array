#pragma once

#include <stdint.h>

#include "hub_config.h"

namespace hub {

// Frame payload v1 (little-endian, packed).
// Sent as MsgType::Frame payload.

enum FrameFlags : uint32_t {
  kFlagTofValid     = 1u << 0,
  kFlagMlxValid     = 1u << 1,
  kFlagCamValid     = 1u << 2,
  kFlagCamSyncValid = 1u << 3,
};

#pragma pack(push, 1)

struct TofConfigV1 {
  uint8_t  side;               // 4 or 8
  uint8_t  targets_per_zone;   // 1..4
  uint16_t ranging_hz;
  uint16_t integration_time_ms;
  uint16_t reserved;
};

// Full VL53L8CX output — all fields enabled.
// Zone ordering: row-major 8×8 (or 4×4 padded to 64).
// Per-zone-per-target arrays: idx = zone * targets_per_zone + target.
struct TofDataV1 {
  uint64_t    ts_us;
  TofConfigV1 cfg;
  uint8_t     silicon_temp_degc;
  uint8_t     _pad[3];

  uint8_t  nb_target_detected[kTofZones];          // per zone
  uint8_t  nb_spads_enabled[kTofZones];            // per zone

  // Per zone × per target (flattened, zone-major)
  int16_t  distance_mm  [kTofZones * kTofMaxTargetsPerZone];
  uint16_t range_sigma_mm[kTofZones * kTofMaxTargetsPerZone];
  uint8_t  target_status[kTofZones * kTofMaxTargetsPerZone];
  uint8_t  reflectance   [kTofZones * kTofMaxTargetsPerZone];
  uint32_t signal_per_spad[kTofZones * kTofMaxTargetsPerZone];
  uint32_t ambient_per_spad[kTofZones];             // zone-level (no per-target)
};

struct MlxConfigV1 {
  uint16_t w;
  uint16_t h;
  uint8_t  mode;        // MLX90640_CHESS or MLX90640_INTERLEAVED
  uint8_t  resolution;  // MLX90640_ADC_xxBIT
  uint8_t  refresh;     // MLX90640_xx_HZ
  uint8_t  reserved;
};

struct MlxDataV1 {
  uint64_t    ts_us;
  MlxConfigV1 cfg;
  int16_t     ta_cC;        // ambient temperature in centi-Celsius
  int16_t     vdd_mV;       // 0 if unavailable
  int16_t     frame_cC[kMlxPixels];
};

struct CamConfigV1 {
  uint32_t w;
  uint32_t h;
  uint32_t format_fourcc;
};

struct CamDataV1 {
  uint64_t    ts_us;
  CamConfigV1 cfg;
  uint32_t    len;
  // JPEG bytes follow (len bytes)
};

struct CamSyncV1 {
  uint64_t last_trigger_ts_us;
  uint64_t last_edge_ts_us;
  uint32_t trigger_period_us;
  uint32_t trigger_pulse_us;
};

struct FrameFixedV1 {
  uint32_t frame_seq;
  uint64_t hub_ts_us;
  uint32_t flags;
  uint32_t reserved;

  TofDataV1  tof;
  MlxDataV1  mlx;
  CamSyncV1  cam_sync;
  CamDataV1  cam;
  // JPEG bytes follow (cam.len bytes)
};

#pragma pack(pop)

static constexpr uint32_t kFourCC_MJPG = 0x47504A4D;  // 'MJPG' LE

struct FrameBuffer {
  FrameFixedV1 fixed;
  uint8_t      cam_bytes[kCamJpegMax];
};

}  // namespace hub
