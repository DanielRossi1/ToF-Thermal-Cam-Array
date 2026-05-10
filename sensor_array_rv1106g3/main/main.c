#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "hub_config.h"
#include "hub_protocol.h"
#include "hub_frame.h"
#include "hub_runtime.h"
#include "hub_transport.h"
#include "hub_control.h"
#include "i2c_bus.h"
#include "tof_vl53l8ch.h"
#include "mlx90640_driver.h"
#include "camera_sync.h"
#if USE_UVC_CAMERA
#include "v4l2_camera.h"
#endif

// ── Globals ────────────────────────────────────────────────────────────────────

static I2CBus          g_i2c;
static Transport       g_tx;
static TofVl53l8ch     g_tof;
static Mlx90640Driver  g_mlx;
static CameraSync      g_cam_sync;
#if USE_UVC_CAMERA
static V4L2Camera      g_cam;
static int             g_cam_started = 0;
#endif

static volatile StreamMode g_mode = STREAM_MODE_ALL;
static volatile int        g_stream_enabled = 1;

static FrameBuffer    *g_frames[2] = {NULL, NULL};
static int             g_fill_idx = 0;
static uint32_t        g_frame_seq = 0;

static volatile int    g_running = 1;
static volatile int    g_tof_ready = 0;
static volatile uint64_t g_tof_irq_ts_us = 0;

// Queue for TX thread
static pthread_mutex_t g_tx_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_tx_cond  = PTHREAD_COND_INITIALIZER;
static FrameBuffer    *g_tx_fb = NULL;
static int             g_tx_has_frame = 0;

// Sensor init status
static int g_tof_ok = 0;
static int g_mlx_ok = 0;

// GPIO INT fd
static int g_gpio_fd = -1;

// Forward declarations
static uint64_t now_us(void);

// ── GPIO interrupt thread ──────────────────────────────────────────────────────

static int gpio_export(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    if (access(path, F_OK) == 0) return 0;

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    char buf[16];
    int n = snprintf(buf, sizeof(buf), "%d", pin);
    int ret = (write(fd, buf, (size_t)n) < 0) ? -1 : 0;
    close(fd);
    usleep(50000);
    return ret;
}

static int gpio_set_edge(int pin, const char *edge) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    if (write(fd, edge, strlen(edge)) < 0) { close(fd); return -1; }
    close(fd);
    return 0;
}

static int gpio_set_direction(int pin, const char *dir) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    if (write(fd, dir, strlen(dir)) < 0) { close(fd); return -1; }
    close(fd);
    return 0;
}

static int gpio_open_value(int pin) {
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    return open(path, O_RDONLY);
}

static void *gpio_int_thread(void *arg) {
    (void)arg;
    char buf[2];
    while (g_running) {
        lseek(g_gpio_fd, 0, SEEK_SET);

        struct pollfd pfd = { .fd = g_gpio_fd, .events = POLLPRI | POLLERR };
        int ret = poll(&pfd, 1, 100);
        if (ret < 0) continue;

        if (pfd.revents & POLLPRI) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            g_tof_irq_ts_us = (uint64_t)ts.tv_sec * 1000000ULL +
                              (uint64_t)ts.tv_nsec / 1000ULL;
            g_tof_ready = 1;
#if USE_CAM_SYNC
            // Record the edge timestamp for camera sync diagnostics
            g_cam_sync.last_edge_ts_us = g_tof_irq_ts_us;
#endif

            // Read to clear the interrupt
            lseek(g_gpio_fd, 0, SEEK_SET);
            read(g_gpio_fd, buf, sizeof(buf));
        }
    }
    return NULL;
}

// ── TX thread ──────────────────────────────────────────────────────────────────

static void *tx_thread(void *arg) {
    (void)arg;
    static uint64_t last_log_us = 0;
    uint64_t tx_count = 0;
    
    while (g_running) {
        pthread_mutex_lock(&g_tx_mutex);
        while (!g_tx_has_frame && g_running)
            pthread_cond_wait(&g_tx_cond, &g_tx_mutex);

        if (!g_running) { pthread_mutex_unlock(&g_tx_mutex); break; }

        FrameBuffer *fb = g_tx_fb;
        g_tx_fb = NULL;
        g_tx_has_frame = 0;
        pthread_mutex_unlock(&g_tx_mutex);

        if (!fb) continue;

        uint32_t plen = (uint32_t)(sizeof(FrameFixedV1) + fb->fixed.cam.len);
        transport_send(&g_tx, MSG_FRAME, fb->fixed.frame_seq,
                       fb->fixed.hub_ts_us, &fb->fixed, plen);
        
        tx_count++;
        uint64_t current_us = now_us();
        if (current_us - last_log_us > 1000000ULL) {  // Log every ~1 sec
            fprintf(stderr, "[TX] Sent %lu frames (last seq=%u, plen=%u)\n", 
                    tx_count, fb->fixed.frame_seq, plen);
            last_log_us = current_us;
            tx_count = 0;
        }
    }
    return NULL;
}

// ── RX thread ──────────────────────────────────────────────────────────────────

static SlipDecoder g_slip;

static void *rx_thread(void *arg) {
    (void)arg;
    uint8_t buf[128];
    while (g_running) {
        ssize_t n = read(g_tx.fd, buf, sizeof(buf));
        if (n > 0) {
            slip_decoder_feed(&g_slip, buf, (size_t)n);
        } else if (n == 0) {
            g_running = 0;
            usleep(1000);
        } else if (n < 0 && errno != EAGAIN) {
            usleep(1000);
        }
    }
    return NULL;
}

// ── Loop thread ────────────────────────────────────────────────────────────────

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void *loop_thread(void *arg) {
    (void)arg;
    uint64_t next_sample_us = 0;
    uint64_t frame_log_ts = 0;
    int      first_frame = 1;

    while (g_running) {
        if (!g_stream_enabled) { usleep(2000); continue; }

        const StreamMode mode   = g_mode;
        const uint64_t    nw_us = now_us();

        // Sampling interval management
        if (mode == STREAM_MODE_ALL || mode == STREAM_MODE_TOF_ONLY) {
            if (g_tof_ok) {
                // If GPIO interrupts available, wait for data ready signal
                // Otherwise use polling mode
                if (g_gpio_fd >= 0) {
                    if (!g_tof_ready && (nw_us - g_tof_irq_ts_us) < 200000ULL) {
                        usleep(1000);
                        continue;
                    }
                    g_tof_ready = 0;
                } else {
                    // No GPIO interrupts - use polling with ~67ms period (15 Hz)
                    if (next_sample_us == 0) next_sample_us = nw_us;
                    if (nw_us < next_sample_us) { usleep(2000); continue; }
                    next_sample_us = nw_us + 66667;
                }
            } else {
                if (next_sample_us == 0) next_sample_us = nw_us;
                if (nw_us < next_sample_us) { usleep(1000); continue; }
                next_sample_us = nw_us + 50000;
            }
        } else {
            if (next_sample_us == 0) next_sample_us = nw_us;
            if (nw_us < next_sample_us) { usleep(1000); continue; }
            next_sample_us = nw_us + 50000;
        }

        pthread_mutex_lock(&g_tx_mutex);
        if (g_tx_has_frame) {
            // The network hasn't finished sending the last frame. 
            // Drop this capture cycle to maintain real-time performance.
            pthread_mutex_unlock(&g_tx_mutex);
            usleep(2000);
            continue; 
        }
        pthread_mutex_unlock(&g_tx_mutex);
        // ----------------------

        FrameBuffer *fb = g_frames[g_fill_idx];
        if (!fb) continue;

        fb->fixed.frame_seq  = g_frame_seq++;
        fb->fixed.hub_ts_us  = now_us();
        fb->fixed.flags      = 0;
        fb->fixed.reserved   = 0;

        if ((mode == STREAM_MODE_ALL || mode == STREAM_MODE_TOF_ONLY) && g_tof_ok) {
            if (tof_read(&g_tof, &fb->fixed.tof)) {
                fb->fixed.tof.ts_us = g_tof_irq_ts_us;
                fb->fixed.flags |= FLAG_TOF_VALID;
            } else if (g_frame_seq % 100 == 0) {
                fprintf(stderr, "WARN: ToF read returned no data (frame %u)\n", g_frame_seq);
            }
        }

        if ((mode == STREAM_MODE_ALL || mode == STREAM_MODE_MLX_ONLY) && g_mlx_ok) {
            if (mlx_read_frame(&g_mlx, &fb->fixed.mlx))
                fb->fixed.flags |= FLAG_MLX_VALID;
            else if (g_frame_seq % 100 == 0)
                fprintf(stderr, "WARN: MLX read returned no data (frame %u)\n", g_frame_seq);
        }

        cam_sync_fill(&g_cam_sync, &fb->fixed.cam_sync);
#if USE_CAM_SYNC
        fb->fixed.flags |= FLAG_CAM_SYNC_VALID;
#endif

#if USE_UVC_CAMERA
        uint32_t cam_len   = 0;
        uint64_t cam_ts_us = 0;

        if ((mode == STREAM_MODE_ALL || mode == STREAM_MODE_CAM_ONLY) &&
            g_cam_started && v4l2_camera_is_ready(&g_cam)) {
            v4l2_camera_snapshot(&g_cam, fb->cam_bytes, CAM_JPEG_MAX,
                                 &cam_len, &cam_ts_us);
        }
        fb->fixed.cam.ts_us          = cam_ts_us;
        fb->fixed.cam.cfg.w          = g_cam.w;
        fb->fixed.cam.cfg.h          = g_cam.h;
        fb->fixed.cam.cfg.format_fourcc = FOURCC_MJPG;
        fb->fixed.cam.len            = cam_len;
        if (cam_len > 0) fb->fixed.flags |= FLAG_CAM_VALID;
#endif

        // Queue for TX
        pthread_mutex_lock(&g_tx_mutex);
        g_tx_fb = fb;
        g_tx_has_frame = 1;
        pthread_cond_signal(&g_tx_cond);
        pthread_mutex_unlock(&g_tx_mutex);
        
        if (g_frame_seq % 100 == 0) {
            fprintf(stderr, "[frame %u] flags=0x%02x tof=%s mlx=%s\n",
                    g_frame_seq,
                    fb->fixed.flags,
                    (fb->fixed.flags & FLAG_TOF_VALID) ? "OK" : "--",
                    (fb->fixed.flags & FLAG_MLX_VALID) ? "OK" : "--");
        }

        g_fill_idx ^= 1;
    }
    return NULL;
}

// ── Boot scan ──────────────────────────────────────────────────────────────────

typedef struct {
    int found;
    int has_mlx_33;
    int has_vl53_29;
} BootI2cScan;

static BootI2cScan scan_i2c(void) {
    BootI2cScan s = {0, 0, 0};
    i2c_bus_lock(&g_i2c);
    // Only probe known sensor addresses to avoid 280 ms I2C timeouts per address
    const uint8_t known[] = { 0x29, 0x33, 0x32, 0x3C };  // VL53, MLX primary, MLX alt, OLED/display
    for (size_t i = 0; i < sizeof(known); i++) {
        if (i2c_bus_probe(&g_i2c, known[i])) {
            s.found++;
            if (known[i] == 0x29) s.has_vl53_29 = 1;
            if (known[i] == 0x33) s.has_mlx_33 = 1;
            printf("I2C: found device at 0x%02X\n", known[i]);
        }
    }
    i2c_bus_unlock(&g_i2c);
    return s;
}

// ── Signal handler ─────────────────────────────────────────────────────────────

static void sig_handler(int sig) {
    (void)sig;
    g_running = 0;
}

// ── TCP transport setup ───────────────────────────────────────────────────────

#if USE_TCP_TRANSPORT
static int tcp_listen_and_accept(int port) {
    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { perror("tcp socket"); return -1; }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("tcp bind"); close(listen_fd); return -1;
    }
    if (listen(listen_fd, 1) < 0) {
        perror("tcp listen"); close(listen_fd); return -1;
    }

    printf("TCP server listening on port %d (waiting for PC client)...\n", port);
    fflush(stdout);

    struct sockaddr_in client;
    socklen_t clen = sizeof(client);
    int client_fd = accept(listen_fd, (struct sockaddr *)&client, &clen);
    close(listen_fd);

    if (client_fd < 0) { perror("tcp accept"); return -1; }

    // Disable Nagle for low-latency
    int nodelay = 1;
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    printf("TCP client connected: %s:%d\n",
           inet_ntoa(client.sin_addr), ntohs(client.sin_port));
    fflush(stdout);
    return client_fd;
}
#endif

// ── Main ───────────────────────────────────────────────────────────────────────

int main(void) {
    printf("Sensor Hub for RV1106G3 starting...\n");
    fflush(stdout);

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    // ── I2C ────────────────────────────────────────────────────────────────
    if (i2c_bus_open(&g_i2c, I2C_DEVICE_PATH) < 0) {
        fprintf(stderr, "FATAL: cannot open I2C bus\n");
        return 1;
    }

    // ── Transport (TCP or UART) ──────────────────────────────────────────
#if USE_TCP_TRANSPORT
    {
        int fd = tcp_listen_and_accept(TCP_LISTEN_PORT);
        if (fd < 0) {
            fprintf(stderr, "FATAL: cannot create TCP server\n");
            return 1;
        }
        g_tx.fd = fd;
    }
#else
    if (transport_open(&g_tx, UART_DEVICE_PATH, SERIAL_BAUD) < 0) {
        fprintf(stderr, "FATAL: cannot open serial port\n");
        return 1;
    }
#endif

    // ── Boot I2C scan ──────────────────────────────────────────────────────
    printf("Scanning I2C bus for sensors...\n");
    BootI2cScan i2c_scan = scan_i2c();

    // ── ToF ────────────────────────────────────────────────────────────────
    printf("Initializing ToF sensor...\n");
    g_tof.bus     = &g_i2c;
    g_tof.lpn_pin = PIN_TOF_LPN;
    g_tof_ok      = (tof_begin(&g_tof) == 0);
    printf("ToF sensor: %s\n", g_tof_ok ? "OK" : "FAILED");

    // ── MLX ────────────────────────────────────────────────────────────────
    printf("Initializing MLX sensor...\n");
    g_mlx.bus = &g_i2c;
    g_mlx_ok  = (mlx_begin(&g_mlx) == 0);
    printf("MLX sensor: %s\n", g_mlx_ok ? "OK" : "FAILED");

    // ── Camera sync ────────────────────────────────────────────────────────
    cam_sync_init(&g_cam_sync);

#if USE_UVC_CAMERA
    // ── V4L2 camera ────────────────────────────────────────────────────────
    v4l2_camera_init(&g_cam);
#if UVC_AUTOSTART
    g_cam_started = (v4l2_camera_start(&g_cam, 320, 240) == 0);
#else
    g_cam_started = 0;
#endif
#endif

    // ── Frame buffers ──────────────────────────────────────────────────────
    for (int i = 0; i < 2; i++) {
        g_frames[i] = (FrameBuffer *)malloc(sizeof(FrameBuffer));
        if (!g_frames[i]) { fprintf(stderr, "FATAL: frame alloc fail\n"); return 1; }
        memset(g_frames[i], 0, sizeof(FrameBuffer));
    }

    // ── GPIO interrupt for ToF INT pin (non-fatal if pin not available) ─────
    if (gpio_export(PIN_TOF_INT) == 0) {
        usleep(100000);
        gpio_set_direction(PIN_TOF_INT, "in");
        gpio_set_edge(PIN_TOF_INT, "falling");
        g_gpio_fd = gpio_open_value(PIN_TOF_INT);
        fprintf(stderr, "GPIO: INT pin %d ready for interrupts\n", PIN_TOF_INT);
    } else {
        fprintf(stderr, "WARN: cannot export GPIO %d (INT pin) - using polling mode\n", PIN_TOF_INT);
        g_gpio_fd = -1;
    }

    // ── Control context ────────────────────────────────────────────────────
    ControlContext ctx;
    ctx.tx              = &g_tx;
    ctx.i2c             = &g_i2c;
    ctx.tof             = &g_tof;
    ctx.mlx             = &g_mlx;
    ctx.cam_sync        = &g_cam_sync;
#if USE_UVC_CAMERA
    ctx.cam             = &g_cam;
    ctx.cam_started     = &g_cam_started;
#endif
    ctx.stream_enabled  = &g_stream_enabled;
    ctx.mode            = &g_mode;
    hub_control_set_context(&ctx);

    // ── SLIP decoder ───────────────────────────────────────────────────────
    static uint8_t slip_buf[2048];
    slip_decoder_init(&g_slip, slip_buf, sizeof(slip_buf),
                      hub_handle_slip_frame, NULL);

    // ── Start threads ──────────────────────────────────────────────────────
    pthread_t th_tx, th_rx, th_gpio, th_loop;
    pthread_create(&th_tx,   NULL, tx_thread,       NULL);
    pthread_create(&th_rx,   NULL, rx_thread,       NULL);
    pthread_create(&th_loop, NULL, loop_thread,     NULL);
    if (g_gpio_fd >= 0)
        pthread_create(&th_gpio, NULL, gpio_int_thread, NULL);

    // ── Boot event ─────────────────────────────────────────────────────────
    char boot[512];
    snprintf(boot, sizeof(boot),
             "BOOT OK\n"
             "platform=rv1106g3 tof=%d mlx=%d\n"
             "i2c_found=%d mlx_0x33=%d vl53_0x29=%d\n"
             "link=slip+crc32 v=%d\n",
             g_tof_ok, g_mlx_ok,
             i2c_scan.found, i2c_scan.has_mlx_33, i2c_scan.has_vl53_29,
             HUB_VERSION);
    transport_send(&g_tx, MSG_EVENT, 0, now_us(), boot, (uint32_t)strlen(boot));
    
    // Small delay to ensure boot event is sent before streaming
    usleep(200000);

    printf("Hub running. Press Ctrl+C to stop.\n");
    fflush(stdout);

    // ── Wait ───────────────────────────────────────────────────────────────
    pthread_join(th_loop, NULL);
    pthread_join(th_tx,   NULL);
    pthread_join(th_rx,   NULL);
    if (g_gpio_fd >= 0) {
        g_running = 0;
        pthread_join(th_gpio, NULL);
    }

    // ── Cleanup ────────────────────────────────────────────────────────────
#if USE_UVC_CAMERA
    v4l2_camera_deinit(&g_cam);
#endif
    transport_close(&g_tx);
    i2c_bus_close(&g_i2c);

    for (int i = 0; i < 2; i++) free(g_frames[i]);
    if (g_gpio_fd >= 0) close(g_gpio_fd);

    printf("Hub stopped.\n");
    return 0;
}