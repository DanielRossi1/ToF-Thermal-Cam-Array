#include "hub_transport.h"

#include <string.h>

#include "esp_crc.h"

namespace hub {

Transport::Transport(Stream& stream) : s_(stream) {}

void Transport::slipWriteByte(uint8_t b) {
  switch (b) {
    case kSlipEnd:
      s_.write(kSlipEsc);
      s_.write(kSlipEscEnd);
      break;
    case kSlipEsc:
      s_.write(kSlipEsc);
      s_.write(kSlipEscEsc);
      break;
    default:
      s_.write(b);
      break;
  }
}

void Transport::slipWrite(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    slipWriteByte(data[i]);
  }
}

bool Transport::send(MsgType type, uint32_t seq, uint64_t ts_us, const void* payload, uint32_t payload_len) {
  MsgHeader hdr{};
  hdr.magic = kMagic;
  hdr.version = kVersion;
  hdr.type = static_cast<uint16_t>(type);
  hdr.seq = seq;
  hdr.ts_us = ts_us;
  hdr.payload_len = payload_len;

  uint32_t crc = 0;
  crc = esp_crc32_le(crc, reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
  if (payload_len && payload) {
    crc = esp_crc32_le(crc, reinterpret_cast<const uint8_t*>(payload), payload_len);
  }

  // SLIP frame: END, encoded(hdr + payload + crc), END
  s_.write(kSlipEnd);
  slipWrite(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
  if (payload_len && payload) {
    slipWrite(reinterpret_cast<const uint8_t*>(payload), payload_len);
  }
  slipWrite(reinterpret_cast<const uint8_t*>(&crc), sizeof(crc));
  s_.write(kSlipEnd);
  return true;
}

bool Transport::sendTextResp(uint32_t seq, const char* text) {
  if (!text) text = "";
  return send(MsgType::Resp, seq, 0, text, (uint32_t)strlen(text));
}

SlipDecoder::SlipDecoder(uint8_t* buffer, size_t capacity, FrameCb cb, void* user)
    : buf_(buffer), cap_(capacity), n_(0), esc_(false), cb_(cb), user_(user) {}

void SlipDecoder::reset() {
  n_ = 0;
  esc_ = false;
}

void SlipDecoder::pushByte(uint8_t b) {
  if (n_ >= cap_) {
    // Overflow: drop frame.
    reset();
    return;
  }
  buf_[n_++] = b;
}

void SlipDecoder::feed(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];

    if (b == kSlipEnd) {
      if (n_ > 0 && cb_) {
        cb_(buf_, n_, user_);
      }
      reset();
      continue;
    }

    if (esc_) {
      esc_ = false;
      if (b == kSlipEscEnd) {
        pushByte(kSlipEnd);
      } else if (b == kSlipEscEsc) {
        pushByte(kSlipEsc);
      } else {
        // Invalid escape, drop.
        reset();
      }
      continue;
    }

    if (b == kSlipEsc) {
      esc_ = true;
      continue;
    }

    pushByte(b);
  }
}

}  // namespace hub
