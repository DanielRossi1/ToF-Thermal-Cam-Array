#include "hub_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

static uint32_t simple_crc32(uint32_t crc, const uint8_t *buf, size_t len) {
    static const uint32_t table[256] = {
        0x00000000,0x77073096,0xEE0E612C,0x990951BA,0x076DC419,0x706AF48F,
        0xE963A535,0x9E6495A3,0x0EDB8832,0x79DCB8A4,0xE0D5E91E,0x97D2D988,
        0x09B64C2B,0x7EB17CBD,0xE7B82D07,0x90BF1D91,0x1DB71064,0x6AB020F2,
        0xF3B97148,0x84BE41DE,0x1ADAD47D,0x6DDDE4EB,0xF4D4B551,0x83D385C7,
        0x136C9856,0x646BA8C0,0xFD62F97A,0x8A65C9EC,0x14015C4F,0x63066CD9,
        0xFA0F3D63,0x8D080DF5,0x3B6E20C8,0x4C69105E,0xD56041E4,0xA2677172,
        0x3C03E4D1,0x4B04D447,0xD20D85FD,0xA50AB56B,0x35B5A8FA,0x42B2986C,
        0xDBBBC9D6,0xACBCF940,0x32D86CE3,0x45DF5C75,0xDCD60DCF,0xABD13D59,
        0x26D930AC,0x51DE003A,0xC8D75180,0xBFD06116,0x21B4F4B5,0x56B3C423,
        0xCFBA9599,0xB8BDA50F,0x2802B89E,0x5F058808,0xC60CD9B2,0xB10BE924,
        0x2F6F7C87,0x58684C11,0xC1611DAB,0xB6662D3D,0x76DC4190,0x01DB7106,
        0x98D220BC,0xEFD5102A,0x71B18589,0x06B6B51F,0x9FBFE4A5,0xE8B8D433,
        0x7807C9A2,0x0F00F934,0x9609A88E,0xE10E9818,0x7F6A0DBB,0x086D3D2D,
        0x91646C97,0xE6635C01,0x6B6B51F4,0x1C6C6162,0x856530D8,0xF262004E,
        0x6C0695ED,0x1B01A57B,0x8208F4C1,0xF50FC457,0x65B0D9C6,0x12B7E950,
        0x8BBEB8EA,0xFCB9887C,0x62DD1DDF,0x15DA2D49,0x8CD37CF3,0xFBD44C65,
        0x4DB26158,0x3AB551CE,0xA3BC0074,0xD4BB30E2,0x4ADFA541,0x3DD895D7,
        0xA4D1C46D,0xD3D6F4FB,0x4369E96A,0x346ED9FC,0xAD678846,0xDA60B8D0,
        0x44042D73,0x33031DE5,0xAA0A4C5F,0xDD0D7CC9,0x5005713C,0x270241AA,
        0xBE0B1010,0xC90C2086,0x5768B525,0x206F85B3,0xB966D409,0xCE61E49F,
        0x5EDEF90E,0x29D9C998,0xB0D09822,0xC7D7A8A8,0x59B33D17,0x2EB40D81,
        0xB7BD5C3B,0xC0BA6CAD,0xEDB88320,0x9ABFB3B6,0x03B6E20C,0x74B1D29A,
        0xEAD54739,0x9DD277AF,0x04DB2615,0x73DC1683,0xE3630B12,0x94643B84,
        0x0D6D6A3E,0x7A6A5AA8,0xE40ECF0B,0x9309FF9D,0x0A00AE27,0x7D079EB1,
        0xF00F9344,0x8708A3D2,0x1E01F268,0x6906C2FE,0xF762575D,0x806567CB,
        0x196C3671,0x6E6B06E7,0xFED41B76,0x89D32BE0,0x10DA7A5A,0x67DD4ACC,
        0xF9B9DF6F,0x8EBEEFF9,0x17B7BE43,0x60B08ED5,0xD6D6A3E8,0xA1D1937E,
        0x38D8C2C4,0x4FDFF252,0xD1BB67F1,0xA6BC5767,0x3FB506DD,0x48B2364B,
        0xD80D2BDA,0xAF0A1B4C,0x36034AF6,0x41047A60,0xDF60EFC3,0xA867DF55,
        0x316E8EEF,0x4669BE79,0xCB61B38C,0xBC66831A,0x256FD2A0,0x5268E236,
        0xCC0C7795,0xBB0B4703,0x220216B9,0x5505262F,0xC5BA3BBE,0xB2BD0B28,
        0x2BB45A92,0x5CB30A04,0xC2D7FFA7,0xB5D0CF31,0x2CD99E8B,0x5BDEAE1D,
        0x9B64C2B0,0xEC63F226,0x756AA39C,0x026D930A,0x9C0906A9,0xEB0E363F,
        0x72076785,0x05005713,0x95BF4A82,0xE2B87A14,0x7BB12BAE,0x0CB61B38,
        0x92D28E9B,0xE5D5BE0D,0x7CDCEFB7,0x0BDBDF21,0x86D3D2D4,0xF1D4E242,
        0x68DDB3F8,0x1FDA836E,0x81BE16CD,0xF6B9265B,0x6FB077E1,0x18B74777,
        0x88085AE6,0xFF0F6A70,0x66063BCA,0x11010B5C,0x8F659EFF,0xF862AE69,
        0x616BFFD3,0x166CCF45,0xA00AE278,0xD70DD2EE,0x4E048354,0x3903B3C2,
        0xA7672661,0xD06016F7,0x4969474D,0x3E6E77DB,0xAED16A4A,0xD9D65ADC,
        0x40DF0B66,0x37D83BF0,0xA9BCAE53,0xDEBB9EC5,0x47B2CF7F,0x30B5FFE9,
        0xBDBDF21C,0xCABAC28A,0x53B39330,0x24B4A3A6,0xBAD03605,0xCDD70693,
        0x54DE5729,0x23D967BF,0xB3667A2E,0xC4614AB8,0x5D681B02,0x2A6F2B94,
        0xB40BBE37,0xC30C8EA1,0x5A05DF1B,0x2D02EF8D
    };
    crc ^= 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++)
        crc = table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFF;
}

static ControlContext *g_ctx = NULL;

void hub_control_set_context(ControlContext *ctx) {
    g_ctx = ctx;
}

static int str_eq(const char *a, const char *b) {
    while (*a && *b) {
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return 0;
    }
    return *a == 0 && *b == 0;
}

static const char *mode_str(StreamMode m) {
    switch (m) {
        case STREAM_MODE_ALL:       return "all";
        case STREAM_MODE_TOF_ONLY:  return "tof";
        case STREAM_MODE_MLX_ONLY:  return "mlx";
        case STREAM_MODE_CAM_ONLY:  return "cam";
        case STREAM_MODE_NONE:      return "none";
        default:                     return "unknown";
    }
}

static int parse_mode(const char *v, StreamMode *out) {
    if (!v) return 0;
    if (str_eq(v, "all"))                        { *out = STREAM_MODE_ALL; return 1; }
    if (str_eq(v, "tof") || str_eq(v, "tofonly")){ *out = STREAM_MODE_TOF_ONLY; return 1; }
    if (str_eq(v, "mlx") || str_eq(v, "mlxonly")){ *out = STREAM_MODE_MLX_ONLY; return 1; }
    if (str_eq(v, "cam") || str_eq(v, "camonly")){ *out = STREAM_MODE_CAM_ONLY; return 1; }
    if (str_eq(v, "none") || str_eq(v, "off"))   { *out = STREAM_MODE_NONE; return 1; }
    return 0;
}

static void trim(char *p) {
    while (*p && isspace((unsigned char)*p)) p++;
}

static int parse_u32(const char *s, uint32_t *out) {
    if (!s || !*s) return 0;
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s) return 0;
    *out = (uint32_t)v;
    return 1;
}

static const char *kv_value(char *token) {
    char *eq = strchr(token, '=');
    if (!eq) return NULL;
    *eq = 0;
    return eq + 1;
}

static uint8_t parse_mlx_mode(const char *v, uint8_t fb) {
    if (!v) return fb;
    if (str_eq(v, "chess"))      return MLX90640_CHESS;
    if (str_eq(v, "interleaved"))return MLX90640_INTERLEAVED;
    return fb;
}

static uint8_t parse_mlx_res(const char *v, uint8_t fb) {
    if (!v) return fb;
    if (str_eq(v, "16")) return MLX90640_ADC_16BIT;
    if (str_eq(v, "17")) return MLX90640_ADC_17BIT;
    if (str_eq(v, "18")) return MLX90640_ADC_18BIT;
    if (str_eq(v, "19")) return MLX90640_ADC_19BIT;
    return fb;
}

static uint8_t parse_mlx_refresh(const char *v, uint8_t fb) {
    if (!v) return fb;
    if (str_eq(v, "0.5")) return MLX90640_0_5_HZ;
    if (str_eq(v, "1"))   return MLX90640_1_HZ;
    if (str_eq(v, "2"))   return MLX90640_2_HZ;
    if (str_eq(v, "4"))   return MLX90640_4_HZ;
    if (str_eq(v, "8"))   return MLX90640_8_HZ;
    if (str_eq(v, "16"))  return MLX90640_16_HZ;
    if (str_eq(v, "32"))  return MLX90640_32_HZ;
    if (str_eq(v, "64"))  return MLX90640_64_HZ;
    return fb;
}

static void cmd_help(Transport *tx, uint32_t seq) {
    transport_send_text_resp(tx, seq,
        "Commands:\n"
        "  PING\n"
        "  GET INFO\n"
        "  STREAM enable=0|1 mode=all|tof|mlx|cam|none\n"
        "  SET TOF side=4|8 hz=<n> it_ms=<n> continuous=0|1\n"
        "  SET MLX mode=chess|interleaved res=16|17|18|19 refresh=0.5|1|2|4|8|16|32|64\n"
        "  SET CAMSYNC enabled=0|1 period_us=<n> pulse_us=<n>\n"
        "  SET CAM enable=0|1 w=<n> h=<n> interval_us=<n>\n"
        "  DIAG I2C SCAN\n"
        "  HELP\n");
}

static void handle_cmd_text(Transport *tx, uint32_t seq, char *cmd) {
    trim(cmd);
    if (!*cmd) { transport_send_text_resp(tx, seq, "ERR empty\n"); return; }

    char *save = NULL;
    char *t0   = strtok_r(cmd, " \t\r\n", &save);
    if (!t0)   { transport_send_text_resp(tx, seq, "ERR empty\n"); return; }

    if (str_eq(t0, "help")) { cmd_help(tx, seq); return; }
    if (str_eq(t0, "ping")) { transport_send_text_resp(tx, seq, "PONG\n"); return; }

    if (str_eq(t0, "get")) {
        char *what = strtok_r(NULL, " \t\r\n", &save);
        if (what && str_eq(what, "info")) {
            TofSettings tofs;
            MlxSettings mlxs;
            CamSyncSettings css;
            tof_get_settings(g_ctx->tof, &tofs);
            mlx_get_settings(g_ctx->mlx, &mlxs);
            cam_sync_get_settings(g_ctx->cam_sync, &css);

            char buf[512];
            snprintf(buf, sizeof(buf),
                "OK info\n"
                "stream_enabled=%u stream_mode=%s\n"
                "tof_side=%u tof_hz=%u tof_it_ms=%u tof_cont=%u\n"
                "mlx_mode=%u mlx_res=%u mlx_refresh=%u\n"
                "camsync_enabled=%u camsync_period_us=%u camsync_pulse_us=%u\n",
                (unsigned)(g_ctx->stream_enabled ? *g_ctx->stream_enabled : 0),
                mode_str(g_ctx->mode ? *g_ctx->mode : STREAM_MODE_ALL),
                tofs.side, tofs.ranging_hz, tofs.integration_time_ms,
                (unsigned)tofs.continuous,
                mlxs.mode, mlxs.resolution, mlxs.refresh,
                (unsigned)css.enabled, (unsigned)css.trigger_period_us,
                (unsigned)css.trigger_pulse_us);
#if USE_UVC_CAMERA
            if (g_ctx->cam_started) {
                int cam_ok = g_ctx->cam ? v4l2_camera_is_ready(g_ctx->cam) : 0;
                snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
                    "cam_started=%u cam_ready=%u\n",
                    (unsigned)(*g_ctx->cam_started ? 1 : 0),
                    (unsigned)(cam_ok ? 1 : 0));
            }
#endif
            transport_send_text_resp(tx, seq, buf);
            return;
        }
        transport_send_text_resp(tx, seq, "ERR unknown GET\n");
        return;
    }

    if (str_eq(t0, "stream")) {
        int saw_any = 0;
        for (;;) {
            char *kv = strtok_r(NULL, " \t\r\n", &save);
            if (!kv) break;
            const char *v = kv_value(kv);
            if (!v) continue;
            saw_any = 1;
            if (str_eq(kv, "enable")) {
                uint32_t en = 0;
                if (!parse_u32(v, &en)) { transport_send_text_resp(tx, seq, "ERR invalid enable\n"); return; }
                if (g_ctx->stream_enabled) *g_ctx->stream_enabled = (en != 0);
            } else if (str_eq(kv, "mode")) {
                StreamMode m;
                if (!parse_mode(v, &m)) { transport_send_text_resp(tx, seq, "ERR invalid mode\n"); return; }
                if (g_ctx->mode) *g_ctx->mode = m;
            }
        }
        if (!saw_any) { transport_send_text_resp(tx, seq, "ERR stream expects enable= and/or mode=\n"); return; }
        transport_send_text_resp(tx, seq, "OK\n");
        return;
    }

    if (str_eq(t0, "set")) {
        char *which = strtok_r(NULL, " \t\r\n", &save);
        if (!which) { transport_send_text_resp(tx, seq, "ERR set expects TOF|MLX|CAM|CAMSYNC\n"); return; }

        if (str_eq(which, "tof")) {
            TofSettings s;
            tof_get_settings(g_ctx->tof, &s);
            for (;;) {
                char *kv = strtok_r(NULL, " \t\r\n", &save);
                if (!kv) break;
                const char *v = kv_value(kv);
                if (!v) continue;
                uint32_t n = 0;
                if      (str_eq(kv, "side"))       { if (parse_u32(v, &n)) s.side = (uint8_t)n; }
                else if (str_eq(kv, "hz"))         { if (parse_u32(v, &n)) s.ranging_hz = (uint16_t)n; }
                else if (str_eq(kv, "it_ms"))      { if (parse_u32(v, &n)) s.integration_time_ms = (uint16_t)n; }
                else if (str_eq(kv, "continuous")) { if (parse_u32(v, &n)) s.continuous = (n != 0); }
            }
            int ok = tof_apply_settings(g_ctx->tof, &s);
            transport_send_text_resp(tx, seq, ok ? "OK\n" : "ERR tof apply\n");
            return;
        }

        if (str_eq(which, "mlx")) {
            MlxSettings s;
            mlx_get_settings(g_ctx->mlx, &s);
            for (;;) {
                char *kv = strtok_r(NULL, " \t\r\n", &save);
                if (!kv) break;
                const char *v = kv_value(kv);
                if (!v) continue;
                if      (str_eq(kv, "mode"))   s.mode       = parse_mlx_mode(v, s.mode);
                else if (str_eq(kv, "res"))    s.resolution = parse_mlx_res(v, s.resolution);
                else if (str_eq(kv, "refresh"))s.refresh    = parse_mlx_refresh(v, s.refresh);
            }
            int ok = mlx_apply_settings(g_ctx->mlx, &s);
            transport_send_text_resp(tx, seq, ok ? "OK\n" : "ERR mlx apply\n");
            return;
        }

        if (str_eq(which, "camsync")) {
            CamSyncSettings s;
            cam_sync_get_settings(g_ctx->cam_sync, &s);
            for (;;) {
                char *kv = strtok_r(NULL, " \t\r\n", &save);
                if (!kv) break;
                const char *v = kv_value(kv);
                if (!v) continue;
                uint32_t n = 0;
                if      (str_eq(kv, "enabled"))   { if (parse_u32(v, &n)) s.enabled = (n != 0); }
                else if (str_eq(kv, "period_us")) { if (parse_u32(v, &n)) s.trigger_period_us = n; }
                else if (str_eq(kv, "pulse_us"))  { if (parse_u32(v, &n)) s.trigger_pulse_us  = n; }
            }
            cam_sync_apply_settings(g_ctx->cam_sync, &s);
            transport_send_text_resp(tx, seq, "OK\n");
            return;
        }

#if USE_UVC_CAMERA
        if (str_eq(which, "cam")) {
            UvcSettings s;
            v4l2_camera_get_settings(g_ctx->cam, &s);
            int want_enable  = 0;
            int want_disable = 0;
            for (;;) {
                char *kv = strtok_r(NULL, " \t\r\n", &save);
                if (!kv) break;
                const char *v = kv_value(kv);
                if (!v) continue;
                uint32_t n = 0;
                if      (str_eq(kv, "enable"))     { if (parse_u32(v, &n)) { want_enable = (n != 0); want_disable = (n == 0); } }
                else if (str_eq(kv, "w"))          { if (parse_u32(v, &n)) s.w = n; }
                else if (str_eq(kv, "h"))          { if (parse_u32(v, &n)) s.h = n; }
                else if (str_eq(kv, "interval_us")){ if (parse_u32(v, &n)) s.interval_us = n; }
            }
            if (want_disable) {
                v4l2_camera_stop(g_ctx->cam);
                if (g_ctx->cam_started) *g_ctx->cam_started = 0;
                transport_send_text_resp(tx, seq, "OK\n");
                return;
            }

            // Store/apply settings (restarts stream if already running)
            int apply_rc = v4l2_camera_apply_settings(g_ctx->cam, &s);

            if (want_enable) {
                int already = (g_ctx->cam_started && *g_ctx->cam_started);
                int ok = (apply_rc == 0) && (already ? 1 : (v4l2_camera_start(g_ctx->cam, s.w, s.h) == 0));
                if (g_ctx->cam_started) *g_ctx->cam_started = ok;
                transport_send_text_resp(tx, seq, ok ? "OK\n" : "ERR cam start\n");
                return;
            }

            transport_send_text_resp(tx, seq, "OK\n");
            return;
        }
#endif

        transport_send_text_resp(tx, seq, "ERR unknown SET target\n");
        return;
    }

    if (str_eq(t0, "diag")) {
        char *what = strtok_r(NULL, " \t\r\n", &save);
        if (what && str_eq(what, "i2c")) {
            char *sub = strtok_r(NULL, " \t\r\n", &save);
            if (sub && str_eq(sub, "scan")) {
                i2c_bus_lock(g_ctx->i2c);
                char out[512];
                size_t n = snprintf(out, sizeof(out), "OK i2c_scan\n");
                for (uint8_t addr = 1; addr < 127; addr++) {
                    if (i2c_bus_probe(g_ctx->i2c, addr)) {
                        n += snprintf(out + n, sizeof(out) - n, "0x%02X ", addr);
                        if (n >= sizeof(out) - 8) break;
                    }
                }
                snprintf(out + n, sizeof(out) - n, "\n");
                i2c_bus_unlock(g_ctx->i2c);
                transport_send_text_resp(tx, seq, out);
                return;
            }
        }
        transport_send_text_resp(tx, seq, "ERR unknown DIAG\n");
        return;
    }

    transport_send_text_resp(tx, seq, "ERR unknown cmd (try HELP)\n");
}

void hub_handle_slip_frame(const uint8_t *data, size_t len, void *user) {
    (void)user;
    if (!g_ctx || !data) return;
    if (len < MSG_HEADER_SIZE + MSG_CRC_SIZE) return;

    MsgHeader hdr;
    memcpy(&hdr, data, sizeof(hdr));
    if (hdr.magic != HUB_MAGIC || hdr.version != HUB_VERSION) return;

    size_t need = MSG_HEADER_SIZE + (size_t)hdr.payload_len + MSG_CRC_SIZE;
    if (need != len) return;

    const uint8_t *payload = data + MSG_HEADER_SIZE;
    const uint8_t *crc_ptr = payload + hdr.payload_len;
    uint32_t crc_rx = 0;
    memcpy(&crc_rx, crc_ptr, sizeof(crc_rx));

    uint32_t crc = simple_crc32(0, (const uint8_t *)&hdr, sizeof(hdr));
    if (hdr.payload_len)
        crc = simple_crc32(crc, (const uint8_t *)payload, hdr.payload_len);
    if (crc != crc_rx) return;
    if (hdr.type != MSG_CMD) return;

    static char cmd_buf[256];
    size_t n = (hdr.payload_len < sizeof(cmd_buf) - 1) ? hdr.payload_len : (sizeof(cmd_buf) - 1);
    memcpy(cmd_buf, payload, n);
    cmd_buf[n] = 0;

    handle_cmd_text(g_ctx->tx, hdr.seq, cmd_buf);
}