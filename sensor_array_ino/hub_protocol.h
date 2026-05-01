#pragma once

#include <stdint.h>

// Hub message protocol (framed over SLIP).
//
// Rationale:
// - Binary sensor frames + bidirectional commands on the same link
// - Robust re-sync via SLIP END delimiters
// - Integrity via CRC32 (little-endian)

namespace hub {

static constexpr uint32_t kMagic = 0x53454E53;  // 'SENS' little-endian on wire
static constexpr uint16_t kVersion = 1;

enum class MsgType : uint16_t {
  Frame = 1,
  Cmd = 2,
  Resp = 3,
  Event = 4,
};

#pragma pack(push, 1)
struct MsgHeader {
  uint32_t magic;        // kMagic
  uint16_t version;      // kVersion
  uint16_t type;         // MsgType
  uint32_t seq;          // sender sequence
  uint64_t ts_us;        // sender timestamp (0 if not applicable)
  uint32_t payload_len;  // bytes after header (before crc)
};
#pragma pack(pop)

static constexpr size_t kHeaderSize = sizeof(MsgHeader);
static constexpr size_t kCrcSize = 4;

}  // namespace hub
