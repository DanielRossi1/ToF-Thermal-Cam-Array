#pragma once

#include <stdint.h>

#include "hub_frame.h"
#include "hub_config.h"

namespace hub {

struct CamSyncSettings {
  bool enabled = false;
  uint32_t trigger_period_us = 0;   // 0 disables periodic trigger
  uint32_t trigger_pulse_us = 10;   // pulse width
};

#if HUB_USE_CAM_SYNC

class CameraSync {
 public:
  CameraSync();

  void begin();

  const CamSyncSettings& settings() const { return settings_; }
  void applySettings(const CamSyncSettings& s);

  // Call from the acquisition loop; emits triggers if configured.
  void service();

  // Snapshot sync status into packet.
  void fill(CamSyncV1& out) const;

 private:
  static void IRAM_ATTR isrEdge();

  static volatile uint64_t last_edge_ts_us_;

  CamSyncSettings settings_;
  uint64_t next_trigger_ts_us_ = 0;
  volatile uint64_t last_trigger_ts_us_ = 0;
};

#else

// Stub implementation when external cam-sync is not used.
class CameraSync {
 public:
  void begin() {}
  void applySettings(const CamSyncSettings&) {}
  void service() {}
  void fill(CamSyncV1& out) const {
    out.last_trigger_ts_us = 0;
    out.last_edge_ts_us = 0;
    out.trigger_period_us = 0;
    out.trigger_pulse_us = 0;
  }
  const CamSyncSettings& settings() const { return settings_; }

 private:
  CamSyncSettings settings_{};
};

#endif  // HUB_USE_CAM_SYNC

}  // namespace hub
