#include "hub_control.h"

#include <ctype.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "hub_runtime.h"
#include "esp_crc.h"

namespace hub {

static bool streqi(const char* a, const char* b);

static const char* modeToStr(StreamMode m) {
  switch (m) {
    case StreamMode::All:     return "all";
    case StreamMode::TofOnly: return "tof";
    case StreamMode::MlxOnly: return "mlx";
    case StreamMode::CamOnly: return "cam";
    case StreamMode::None:    return "none";
    default:                  return "unknown";
  }
}

static bool parseMode(const char* v, StreamMode& out) {
  if (!v) return false;
  if (streqi(v, "all"))                        { out = StreamMode::All;     return true; }
  if (streqi(v, "tof") || streqi(v, "tofonly")){ out = StreamMode::TofOnly; return true; }
  if (streqi(v, "mlx") || streqi(v, "mlxonly")){ out = StreamMode::MlxOnly; return true; }
  if (streqi(v, "cam") || streqi(v, "camonly")){ out = StreamMode::CamOnly; return true; }
  if (streqi(v, "none") || streqi(v, "off"))   { out = StreamMode::None;    return true; }
  return false;
}

static void trim(char*& p) {
  while (*p && isspace((unsigned char)*p)) p++;
}

static bool streqi(const char* a, const char* b) {
  while (*a && *b) {
    if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return false;
  }
  return *a == 0 && *b == 0;
}

static bool parseU32(const char* s, uint32_t& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  unsigned long v = strtoul(s, &end, 10);
  if (end == s) return false;
  out = (uint32_t)v;
  return true;
}

static const char* kvValue(char* token) {
  char* eq = strchr(token, '=');
  if (!eq) return nullptr;
  *eq = 0;
  return eq + 1;
}

static void cmdHelp(Transport& tx, uint32_t seq) {
  tx.sendTextResp(seq,
    "Commands:\n"
    "  PING\n"
    "  GET INFO\n"
    "  STREAM enable=0|1 mode=all|tof|mlx|cam|none\n"
    "  SET I2C clock_hz=<n>\n"
    "  SET TOF side=4|8 hz=<n> it_ms=<n> continuous=0|1\n"
    "  SET MLX mode=chess|interleaved res=16..19 refresh=0.5|1|2|4|8|16|32|64\n"
    "  SET CAMSYNC enabled=0|1 period_us=<n> pulse_us=<n>\n"
#if HUB_USE_UVC_CAMERA
    "  SET CAM enable=0|1 w=<n> h=<n> interval_us=<n>\n"
#endif
    "  DIAG I2C SCAN\n"
    "  HELP\n");
}

static uint8_t parseMlxMode(const char* v, uint8_t fb) {
  if (!v) return fb;
  if (streqi(v, "chess"))      return MLX90640_CHESS;
  if (streqi(v, "interleaved"))return MLX90640_INTERLEAVED;
  return fb;
}

static uint8_t parseMlxRes(const char* v, uint8_t fb) {
  if (!v) return fb;
  if (streqi(v, "16")) return MLX90640_ADC_16BIT;
  if (streqi(v, "17")) return MLX90640_ADC_17BIT;
  if (streqi(v, "18")) return MLX90640_ADC_18BIT;
  if (streqi(v, "19")) return MLX90640_ADC_19BIT;
  return fb;
}

static uint8_t parseMlxRefresh(const char* v, uint8_t fb) {
  if (!v) return fb;
  if (streqi(v, "0.5")) return MLX90640_0_5_HZ;
  if (streqi(v, "1"))   return MLX90640_1_HZ;
  if (streqi(v, "2"))   return MLX90640_2_HZ;
  if (streqi(v, "4"))   return MLX90640_4_HZ;
  if (streqi(v, "8"))   return MLX90640_8_HZ;
  if (streqi(v, "16"))  return MLX90640_16_HZ;
  if (streqi(v, "32"))  return MLX90640_32_HZ;
  if (streqi(v, "64"))  return MLX90640_64_HZ;
  return fb;
}

static void handleCmdText(ControlContext& ctx, Transport& tx,
                           uint32_t seq, char* cmd) {
  trim(cmd);
  if (!*cmd) { tx.sendTextResp(seq, "ERR empty\n"); return; }

  char* save = nullptr;
  char* t0   = strtok_r(cmd, " \t\r\n", &save);
  if (!t0)   { tx.sendTextResp(seq, "ERR empty\n"); return; }

  if (streqi(t0, "help")) { cmdHelp(tx, seq); return; }
  if (streqi(t0, "ping")) { tx.sendTextResp(seq, "PONG\n"); return; }

  // ── GET ──────────────────────────────────────────────────────────────────
  if (streqi(t0, "get")) {
    char* what = strtok_r(nullptr, " \t\r\n", &save);
    if (what && streqi(what, "info")) {
      char buf[512];
      const auto& ts = ctx.tof->settings();
      const auto& ms = ctx.mlx->settings();
      const auto& cs = ctx.cam_sync->settings();
      size_t n = 0;
      n += snprintf(buf + n, sizeof(buf) - n,
        "OK info\n"
        "heap_free=%u\npsram_free=%u\n"
        "stream_enabled=%u\nstream_mode=%s\n"
        "tof_side=%u tof_hz=%u tof_it_ms=%u tof_cont=%u\n"
        "mlx_mode=%u mlx_res=%u mlx_refresh=%u\n"
        "camsync_enabled=%u camsync_period_us=%u camsync_pulse_us=%u\n",
        (unsigned)esp_get_free_heap_size(),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        (unsigned)(ctx.stream_enabled && *ctx.stream_enabled),
        modeToStr(g_mode),
        ts.side, ts.ranging_hz, ts.integration_time_ms, (unsigned)ts.continuous,
        ms.mode, ms.resolution, ms.refresh,
        (unsigned)cs.enabled, (unsigned)cs.trigger_period_us, (unsigned)cs.trigger_pulse_us);
#if HUB_USE_UVC_CAMERA
      if (ctx.cam_started) {
        n += snprintf(buf + n, sizeof(buf) - n,
          "cam_started=%u cam_ready=%u\n",
          (unsigned)(*ctx.cam_started ? 1 : 0),
          (unsigned)(ctx.cam && ctx.cam->isReady() ? 1 : 0));
      }
#endif
      tx.sendTextResp(seq, buf);
      return;
    }
    tx.sendTextResp(seq, "ERR unknown GET\n");
    return;
  }

  // ── STREAM ────────────────────────────────────────────────────────────────
  if (streqi(t0, "stream")) {
    bool saw_any = false;
    for (;;) {
      char* kv = strtok_r(nullptr, " \t\r\n", &save);
      if (!kv) break;
      const char* v = kvValue(kv);
      if (!v) continue;
      saw_any = true;
      if (streqi(kv, "enable")) {
        uint32_t en = 0;
        if (!parseU32(v, en)) { tx.sendTextResp(seq, "ERR invalid enable\n"); return; }
        if (ctx.stream_enabled) *ctx.stream_enabled = (en != 0);
      } else if (streqi(kv, "mode")) {
        StreamMode m;
        if (!parseMode(v, m)) { tx.sendTextResp(seq, "ERR invalid mode\n"); return; }
        g_mode = m;
      }
    }
    if (!saw_any) { tx.sendTextResp(seq, "ERR stream expects enable= and/or mode=\n"); return; }
    tx.sendTextResp(seq, "OK\n");
    return;
  }

  // ── SET ───────────────────────────────────────────────────────────────────
  if (streqi(t0, "set")) {
    char* which = strtok_r(nullptr, " \t\r\n", &save);
    if (!which) { tx.sendTextResp(seq, "ERR set expects TOF|MLX|CAM|CAMSYNC|I2C\n"); return; }

    if (streqi(which, "i2c")) {
      uint32_t clock_hz = 0;
      for (;;) {
        char* kv = strtok_r(nullptr, " \t\r\n", &save);
        if (!kv) break;
        const char* v = kvValue(kv);
        if (!v) continue;
        if (streqi(kv, "clock_hz")) parseU32(v, clock_hz);
      }
      if (!clock_hz) { tx.sendTextResp(seq, "ERR i2c expects clock_hz=\n"); return; }
      I2CLock lk(*ctx.i2c, pdMS_TO_TICKS(50));
      if (!lk.ok()) { tx.sendTextResp(seq, "ERR i2c busy\n"); return; }
      // Reconfigure I2C clock via IDF
      i2c_config_t conf = {};
      conf.mode            = I2C_MODE_MASTER;
      conf.sda_io_num      = kPinSda;
      conf.scl_io_num      = kPinScl;
      conf.sda_pullup_en   = GPIO_PULLUP_ENABLE;
      conf.scl_pullup_en   = GPIO_PULLUP_ENABLE;
      conf.master.clk_speed = clock_hz;
      i2c_param_config(ctx.i2c->port(), &conf);
      tx.sendTextResp(seq, "OK\n");
      return;
    }

    if (streqi(which, "tof")) {
      TofSettings s = ctx.tof->settings();
      for (;;) {
        char* kv = strtok_r(nullptr, " \t\r\n", &save);
        if (!kv) break;
        const char* v = kvValue(kv);
        if (!v) continue;
        uint32_t n = 0;
        if      (streqi(kv, "side"))       { if (parseU32(v, n)) s.side = (uint8_t)n; }
        else if (streqi(kv, "hz"))         { if (parseU32(v, n)) s.ranging_hz = (uint16_t)n; }
        else if (streqi(kv, "it_ms"))      { if (parseU32(v, n)) s.integration_time_ms = (uint16_t)n; }
        else if (streqi(kv, "continuous")) { if (parseU32(v, n)) s.continuous = (n != 0); }
      }
      bool ok = ctx.tof->applySettings(s);
      tx.sendTextResp(seq, ok ? "OK\n" : "ERR tof apply\n");
      return;
    }

    if (streqi(which, "mlx")) {
      MlxSettings s = ctx.mlx->settings();
      for (;;) {
        char* kv = strtok_r(nullptr, " \t\r\n", &save);
        if (!kv) break;
        const char* v = kvValue(kv);
        if (!v) continue;
        if      (streqi(kv, "mode"))   s.mode       = parseMlxMode(v, s.mode);
        else if (streqi(kv, "res"))    s.resolution = parseMlxRes(v, s.resolution);
        else if (streqi(kv, "refresh"))s.refresh    = parseMlxRefresh(v, s.refresh);
      }
      bool ok = ctx.mlx->applySettings(s);
      tx.sendTextResp(seq, ok ? "OK\n" : "ERR mlx apply\n");
      return;
    }

    if (streqi(which, "camsync")) {
      CamSyncSettings s = ctx.cam_sync->settings();
      for (;;) {
        char* kv = strtok_r(nullptr, " \t\r\n", &save);
        if (!kv) break;
        const char* v = kvValue(kv);
        if (!v) continue;
        uint32_t n = 0;
        if      (streqi(kv, "enabled"))   { if (parseU32(v, n)) s.enabled = (n != 0); }
        else if (streqi(kv, "period_us")) { if (parseU32(v, n)) s.trigger_period_us = n; }
        else if (streqi(kv, "pulse_us"))  { if (parseU32(v, n)) s.trigger_pulse_us  = n; }
      }
      ctx.cam_sync->applySettings(s);
      tx.sendTextResp(seq, "OK\n");
      return;
    }

#if HUB_USE_UVC_CAMERA
    if (streqi(which, "cam")) {
      UvcSettings s = ctx.cam->settings();
      bool want_enable  = false;
      bool want_disable = false;
      for (;;) {
        char* kv = strtok_r(nullptr, " \t\r\n", &save);
        if (!kv) break;
        const char* v = kvValue(kv);
        if (!v) continue;
        uint32_t n = 0;
        if      (streqi(kv, "enable"))     { if (parseU32(v, n)) { want_enable = (n != 0); want_disable = (n == 0); } }
        else if (streqi(kv, "w"))          { if (parseU32(v, n)) s.w = n; }
        else if (streqi(kv, "h"))          { if (parseU32(v, n)) s.h = n; }
        else if (streqi(kv, "interval_us")){ if (parseU32(v, n)) s.interval_us = n; }
      }
      bool ok = ctx.cam->applySettings(s);
      if (want_disable) {
        ctx.cam->stop();
        if (ctx.cam_started) *ctx.cam_started = false;
      }
      if (want_enable) {
        bool started = ctx.cam->begin();
        if (ctx.cam_started) *ctx.cam_started = started;
        ok = ok && started;
      }
      tx.sendTextResp(seq, ok ? "OK\n" : "ERR cam apply\n");
      return;
    }
#endif

    tx.sendTextResp(seq, "ERR unknown SET target\n");
    return;
  }

  // ── DIAG ──────────────────────────────────────────────────────────────────
  if (streqi(t0, "diag")) {
    char* what = strtok_r(nullptr, " \t\r\n", &save);
    if (what && streqi(what, "i2c")) {
      char* sub = strtok_r(nullptr, " \t\r\n", &save);
      if (sub && streqi(sub, "scan")) {
        I2CLock lk(*ctx.i2c, pdMS_TO_TICKS(200));
        if (!lk.ok()) { tx.sendTextResp(seq, "ERR i2c busy\n"); return; }
        char out[512];
        size_t n = snprintf(out, sizeof(out), "OK i2c_scan\n");
        for (uint8_t addr = 1; addr < 127; addr++) {
          if (ctx.i2c->probe(addr)) {
            n += snprintf(out + n, sizeof(out) - n, "0x%02X ", addr);
            if (n >= sizeof(out) - 8) break;
          }
        }
        snprintf(out + n, sizeof(out) - n, "\n");
        tx.sendTextResp(seq, out);
        return;
      }
    }
    tx.sendTextResp(seq, "ERR unknown DIAG\n");
    return;
  }

  tx.sendTextResp(seq, "ERR unknown cmd (try HELP)\n");
}

void handleSlipFrame(const uint8_t* data, size_t len, void* user) {
  auto* ctx = reinterpret_cast<ControlContext*>(user);
  if (!ctx || !data) return;
  if (!ctx->tx || !ctx->tof || !ctx->mlx || !ctx->cam_sync) return;
  if (len < kHeaderSize + kCrcSize) return;

  MsgHeader hdr;
  memcpy(&hdr, data, sizeof(hdr));
  if (hdr.magic != kMagic || hdr.version != kVersion) return;

  const size_t need = sizeof(MsgHeader) + (size_t)hdr.payload_len + sizeof(uint32_t);
  if (need != len) return;

  const uint8_t* payload = data + sizeof(MsgHeader);
  const uint8_t* crc_ptr = payload + hdr.payload_len;
  uint32_t crc_rx = 0;
  memcpy(&crc_rx, crc_ptr, sizeof(crc_rx));

  uint32_t crc = esp_crc32_le(0, (const uint8_t*)&hdr, sizeof(hdr));
  if (hdr.payload_len)
    crc = esp_crc32_le(crc, payload, hdr.payload_len);
  if (crc != crc_rx) return;
  if ((MsgType)hdr.type != MsgType::Cmd) return;

  static char cmd_buf[256];
  const size_t n = (hdr.payload_len < sizeof(cmd_buf) - 1)
                       ? hdr.payload_len : (sizeof(cmd_buf) - 1);
  memcpy(cmd_buf, payload, n);
  cmd_buf[n] = 0;

  handleCmdText(*ctx, *ctx->tx, hdr.seq, cmd_buf);
}

}  // namespace hub
