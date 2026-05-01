#pragma once

#include <stdint.h>
#include "driver/uart.h"
#include "hub_protocol.h"

namespace hub {

// SLIP special bytes
static constexpr uint8_t kSlipEnd = 0xC0;
static constexpr uint8_t kSlipEsc = 0xDB;
static constexpr uint8_t kSlipEscEnd = 0xDC;
static constexpr uint8_t kSlipEscEsc = 0xDD;

class Transport {
 public:
  explicit Transport(uart_port_t uart);

  // Sends one Hub message (header + payload + crc32) framed via SLIP.
  bool send(MsgType type, uint32_t seq, uint64_t ts_us, const void* payload, uint32_t payload_len);

  // Convenience: send a RESP as UTF-8 text.
  bool sendTextResp(uint32_t seq, const char* text);

 private:
  uart_port_t uart_;

  void slipWriteByte(uint8_t b);
  void slipWrite(const uint8_t* data, size_t len);
};

// Incremental SLIP decoder for CMD messages.
// Usage: feed() bytes; when a full SLIP frame arrives, onFrame() is called.
class SlipDecoder {
 public:
  using FrameCb = void (*)(const uint8_t* data, size_t len, void* user);

  SlipDecoder(uint8_t* buffer, size_t capacity, FrameCb cb, void* user);

  void reset();
  void feed(const uint8_t* data, size_t len);

 private:
  uint8_t* buf_;
  size_t cap_;
  size_t n_;
  bool esc_;
  FrameCb cb_;
  void* user_;

  void pushByte(uint8_t b);
};

}  // namespace hub
