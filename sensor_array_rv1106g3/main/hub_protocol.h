#pragma once

#include <stdint.h>

// Hub message protocol (framed over SLIP).
// Binary sensor frames + bidirectional commands on the same link
// Robust re-sync via SLIP END delimiters
// Integrity via CRC32

#define HUB_MAGIC       0x53454E53  // 'SENS' little-endian
#define HUB_VERSION     1

enum MsgType {
    MSG_FRAME = 1,
    MSG_CMD   = 2,
    MSG_RESP  = 3,
    MSG_EVENT = 4,
};

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t seq;
    uint64_t ts_us;
    uint32_t payload_len;
} MsgHeader;
#pragma pack(pop)

#define MSG_HEADER_SIZE  sizeof(MsgHeader)
#define MSG_CRC_SIZE     4