#pragma once

#include <stdint.h>

#include "hub_config.h"

namespace hub {

// Frame payload v1 (little-endian, packed).
// Sent as MsgType::Frame payload.

enum FrameFlags : uint32_t {
  kFlagTofValid = 1u << 0,
  kFlagMlxValid = 1u << 1,
  kFlagCamValid = 1u << 2,
  kFlagCamSyncValid = 1u << 3,
};

#pragma pack(push, 1)

struct TofConfigV1 {
  uint8_t side;               // 4 or 8
  uint8_t targets_per_zone;   // typically 1..4
  uint16_t ranging_hz;
  uint16_t integration_time_ms;
  uint16_t reserved;
};

struct TofDataV1 {
  uint64_t ts_us;
  TofConfigV1 cfg;
  uint8_t nb_targets[kTofZones];
  // Flattened by zone-major, target-minor:
  // idx = zone * targets_per_zone + t
  uint16_t distance_mm[kTofZones * kTofMaxTargetsPerZone];
  uint16_t sigma_mm[kTofZones * kTofMaxTargetsPerZone];
  uint8_t status[kTofZones * kTofMaxTargetsPerZone];
};

struct MlxConfigV1 {
  uint16_t w;
  uint16_t h;
  uint8_t mode;        // library enum value (best-effort)
  uint8_t resolution;  // library enum value (best-effort)
  uint8_t refresh;     // library enum value (best-effort)
  uint8_t reserved;
};

struct MlxDataV1 {
  uint64_t ts_us;
  MlxConfigV1 cfg;
  int16_t ta_cC;
  int16_t vdd_mV;  // 0 if unavailable
  int16_t frame_cC[kMlxPixels];
};

struct CamConfigV1 {
  uint32_t w;
  uint32_t h;
  uint32_t format_fourcc;  // e.g. 'MJPG'
};

struct CamDataV1 {
  uint64_t ts_us;
  CamConfigV1 cfg;
  uint32_t len;
  // bytes follow (len)
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

  TofDataV1 tof;
  MlxDataV1 mlx;

  CamSyncV1 cam_sync;
  CamDataV1 cam;
  // cam bytes follow
};

#pragma pack(pop)

static constexpr uint32_t kFourCC_MJPG = 0x47504A4D;  // 'MJPG' little-endian

// Helper describing a frame buffer that owns room for a max JPEG.
struct FrameBuffer {
  FrameFixedV1 fixed;
  uint8_t cam_bytes[kCamJpegMax];
};

}  // namespace hub
