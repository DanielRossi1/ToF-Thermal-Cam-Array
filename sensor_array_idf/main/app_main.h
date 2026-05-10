#pragma once

// Forward declarations for hub classes
namespace hub {
  class I2CBus;
  class TofVl53l8ch;
  class Mlx90640Driver;
  class CameraSync;
  class UvcCamera;
  class Transport;
  class ControlContext;
  class SlipDecoder;
  struct FrameBuffer;
  enum class StreamMode : uint8_t;
  struct FrameFixedV1;
  enum class MsgType : uint8_t;
  constexpr uint32_t kVersion = 2;
  constexpr uint32_t kFourCC_MJPG = 0x4D4A5047;
  constexpr uint32_t kFlagTofValid = 0x01;
  constexpr uint32_t kFlagMlxValid = 0x02;
  constexpr uint32_t kFlagCamSyncValid = 0x04;
  constexpr uint32_t kFlagCamValid = 0x08;
  constexpr uint32_t kCamJpegMax = 65536;
  constexpr uint32_t kSerialBaud = 115200;
  constexpr uint8_t kPinTofInt = 4;
  constexpr uint8_t kPinTofLpn = 5;
  constexpr uint8_t kPinCamSync1 = 19;
  constexpr uint8_t kPinCamSync2 = 20;
  constexpr bool HUB_USE_UVC_CAMERA = true;
  constexpr bool HUB_USE_CAM_SYNC = true;
  constexpr bool HUB_UVC_AUTOSTART = false;
}