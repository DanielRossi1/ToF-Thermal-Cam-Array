#pragma once

#include <stdint.h>
#include <stddef.h>
#include "hub_protocol.h"

#define SLIP_END     0xC0
#define SLIP_ESC     0xDB
#define SLIP_ESC_END 0xDC
#define SLIP_ESC_ESC 0xDD

typedef struct {
    int fd;  // serial port fd
} Transport;

int  transport_open(Transport *t, const char *path, int baud);
void transport_close(Transport *t);

// Atomically swap the underlying fd used by transport_send().
// Closes the previous fd (if any). Safe to call while other threads are sending.
void transport_set_fd(Transport *t, int fd);

void transport_send(Transport *t, int type, uint32_t seq, uint64_t ts_us,
                    const void *payload, uint32_t payload_len);

void transport_send_text_resp(Transport *t, uint32_t seq, const char *text);

// ── SLIP decoder (incremental) ──────────────────────────────────────────────

typedef void (*SlipFrameCb)(const uint8_t *data, size_t len, void *user);

typedef struct {
    uint8_t    *buf;
    size_t      cap;
    size_t      n;
    int         esc;
    SlipFrameCb cb;
    void       *user;
} SlipDecoder;

void slip_decoder_init(SlipDecoder *d, uint8_t *buf, size_t cap,
                       SlipFrameCb cb, void *user);
void slip_decoder_reset(SlipDecoder *d);
void slip_decoder_feed(SlipDecoder *d, const uint8_t *data, size_t len);