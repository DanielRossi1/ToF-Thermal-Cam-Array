#pragma once

#include <stdint.h>

namespace hub {

enum class StreamMode : uint8_t {
  All = 0,
  TofOnly = 1,
  MlxOnly = 2,
  CamOnly = 3,
  None = 4,
};

// Current streaming mode (set via STREAM mode=...)
extern volatile StreamMode g_mode;

}  // namespace hub
