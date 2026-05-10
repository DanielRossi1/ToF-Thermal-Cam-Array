# RV1106G3 (Luckfox) Synchronized Sensor Hub

This project ports the ESP32-S3 Sensor Hub to the **Luckfox Pico** board with a **Rockchip RV1106G3** SoC, providing significantly more CPU headroom for real-time multi-modal sensor acquisition and on-device inference.

By fusing high-resolution RGB imagery with spatial depth data from a Time-of-Flight (ToF) array, this system enables robust depth estimation and object detection. The thermal camera adds an additional sensing dimension.

---

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    Luckfox RV1106G3                       │
│  ┌─────────┐  ┌──────────┐  ┌──────────────────────────┐ │
│  │ VL53L8CH│  │ MLX90640 │  │   USB UVC Camera         │ │
│  │  (ToF)  │  │ (Thermal)│  │   (Global Shutter)       │ │
│  │  I2C    │  │  I2C     │  │   V4L2 / MJPEG           │ │
│  └────┬────┘  └────┬─────┘  └───────────┬──────────────┘ │
│       │            │                     │                │
│  ┌────┴────────────┴─────────────────────┴──────────────┐ │
│  │               sensor_hub (C + pthreads)              │ │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐  │ │
│  │  │ Loop Thd │ │  TX Thd  │ │  RX Thd  │ │GPIO Thd │  │ │
│  │  │ acquire  │ │  SLIP    │ │  SLIP    │ │ ToF INT │  │ │
│  │  │ sensors  │ │  encode  │ │  decode  │ │  poll() │  │ │
│  │  └──────────┘ └──────────┘ └──────────┘ └────────┘  │ │
│  └────────────────────────┬─────────────────────────────┘ │
│                           │ UART (2M baud)               │
└───────────────────────────┼──────────────────────────────┘
                            │
                    ┌───────┴───────┐
                    │  PC (USB-UART)│
                    │  pc_visualizer│
                    │  (Python/PyQt)│
                    └───────────────┘
```

---

## Directory Structure

```
sensor_array_rv1106g3/
├── main/               # Core application source
│   ├── main.c          # Entry point, thread orchestration
│   ├── hub_protocol.h  # Binary message protocol + SLIP framing
│   ├── hub_frame.h     # Sensor data packet layout
│   ├── hub_config.h    # Pin mappings and tuning constants
│   ├── hub_runtime.h   # Stream mode enum
│   ├── hub_transport.c # UART TX/RX + SLIP encoder/decoder
│   ├── hub_transport.h
│   ├── hub_control.c   # Text command parser/dispatcher
│   ├── hub_control.h
│   ├── i2c_bus.c       # Linux I2C abstraction (/dev/i2c-N)
│   ├── i2c_bus.h
│   ├── tof_vl53l8ch.c  # VL53L8CH ToF sensor driver
│   ├── tof_vl53l8ch.h
│   ├── mlx90640_driver.c # MLX90640 thermal camera driver
│   ├── mlx90640_driver.h
│   ├── v4l2_camera.c   # USB UVC camera via V4L2 + mmap
│   ├── v4l2_camera.h
│   ├── camera_sync.c   # External camera sync (optional)
│   └── camera_sync.h
├── components/
│   ├── STM32duino_VL53L8CH/src/
│   │   ├── platform.c  # Linux I2C platform layer for VL53LMZ API
│   │   ├── platform.h
│   │   ├── platform_config.h
│   │   ├── vl53lmz_api.c/h     # ST VL53LMZ API (firmware+config)
│   │   ├── vl53lmz_buffers.h
│   │   └── vl53lmz_plugin_*.c/h # Xtalk, motion, CNH, thresholds
│   └── Adafruit_MLX90640/
│       ├── MLX90640_API.c/h   # Melexis temperature calculation
│       ├── MLX90640_I2C_Driver.h
│       └── mlx90640_linux_hal.c # Linux I2C HAL for Melexis driver
├── pc_visualizer/     # Python PC-side GUI (copied from ESP project)
│   ├── main.py
│   ├── protocol.py
│   ├── sensor_test.py
│   └── requirements.txt
├── Makefile           # Cross-compilation
├── build.sh           # Build + optional deploy via scp
└── README.md          # This file
```

---

## Thread Architecture

The application uses four POSIX threads (pthreads) mirroring the FreeRTOS task topology of the ESP32 version:

| Thread      | Core | Role                                                    |
|:----------- |:---- |:------------------------------------------------------- |
| `tx_thread` | —    | Dequeues `FrameBuffer` and sends SLIP-encoded frames     |
| `rx_thread` | —    | Reads raw bytes from UART, feeds SLIP decoder            |
| `loop_thread`| —   | Acquires ToF + MLX + Camera, assembles frame             |
| `gpio_thread`| —   | Polls ToF INT GPIO edge via sysfs, sets `g_tof_ready`    |

### Synchronisation
- `g_tx_mutex` / `g_tx_cond`: condition variable to signal TX thread when a new frame is ready
- `g_tof_ready` flag: atomic volatile flag set by GPIO thread, consumed by loop thread
- All I2C access serialised via `i2c_bus_lock()`/`unlock()` (pthread mutex)
- V4L2 frame buffer protected by `cam->mutex`

---

## Protocol (Wire Format)

The protocol is **identical** to the ESP32-S3 implementation. See `CONTROL_PROTOCOL.md` in the ESP32 project for full details.

### Framing
- **SLIP** byte-stuffed frames: `END (0xC0)` ... data ... `END (0xC0)`
- Escape bytes: `0xDB 0xDC` for END, `0xDB 0xDD` for ESC

### Message Layout
```
[MsgHeader (24 bytes)] [payload] [CRC32 (4 bytes, LE)]
```

**MsgHeader**: (`packed`, little-endian)
```
uint32_t magic       = 0x53454E53   // 'SENS'
uint16_t version     = 1
uint16_t type        = 1=Frame, 2=Cmd, 3=Resp, 4=Event
uint32_t seq         // sequence number
uint64_t ts_us       // sender timestamp in microseconds
uint32_t payload_len // bytes of payload
```

### Frame Payload (type=1)

The payload is `FrameFixedV1` followed by `cam.len` bytes of JPEG:
```
[frame_seq:4] [hub_ts_us:8] [flags:4] [reserved:4]
[TofDataV1  (variable)]
[MlxDataV1  (1556 bytes)]
[CamSyncV1  (24 bytes)]
[CamDataV1  (24 bytes)]
[JPEG bytes (cam.len bytes)]
```

### Control Commands (type=2)

Plain text commands sent as UTF-8, e.g.:
- `PING` — responds with "PONG"
- `GET INFO` — returns sensor status
- `STREAM enable=0|1 mode=all|tof|mlx|cam|none`
- `SET TOF side=4|8 hz=15 it_ms=50 continuous=1`
- `SET MLX mode=chess res=18 refresh=16`
- `SET CAM enable=1 w=320 h=240 interval_us=83333`
- `DIAG I2C SCAN` — lists I2C devices

---

## Electrical Schematics

### I2C Shared Bus
*Requires 4.7kΩ pull-up resistors on SDA and SCL to 3.3V.*

```
                    ┌──────────────────────┐
                    │    Luckfox Pico       │
                    │    RV1106G3           │
                    │                      │
                    │  Pin 3 (GPIO3_C1) ───┼── SDA ──┬── VL53L8CH SDA
                    │             I2C0     │         │
                    │  Pin 5 (GPIO3_C2) ───┼── SCL ──┼── VL53L8CH SCL
                    │                      │         │
                    │           3.3V ──────┼── VCC ──┼── VL53L8CH VCC
                    │                      │         │   MLX90640 VCC
                    │           GND  ──────┼── GND ──┼── VL53L8CH GND
                    │                      │         │   MLX90640 GND
                    │                      │         │
                    │                      │   MLX90640 SDA ──┤
                    │                      │   MLX90640 SCL ──┤
                    └──────────────────────┘
```

### VL53L8CH (ToF Sensor)

| VL53L8CH Pin | Luckfox Pin  | Description              |
|:------------ |:------------ |:------------------------ |
| SDA          | GPIO3_C1 (3) | I2C0 SDA                 |
| SCL          | GPIO3_C2 (5) | I2C0 SCL                 |
| VCC          | 3.3V         | Power                    |
| GND          | GND          | Ground                   |
| INT (GPIO1)  | GPIO1_D2 (55)| Data-ready interrupt (falling edge) |
| LPn (XSHUT)  | GPIO1_D3 (56)| Hardware reset (optional) |

### MLX90640 (Thermal Camera)

| MLX90640 Pin | Luckfox Pin  | Description              |
|:------------ |:------------ |:------------------------ |
| SDA          | GPIO3_C1 (3) | I2C0 SDA (shared)        |
| SCL          | GPIO3_C2 (5) | I2C0 SCL (shared)        |
| VCC          | 3.3V         | Power                    |
| GND          | GND          | Ground                   |

### USB UVC Camera

| USB Signal   | Luckfox Pin  | Description              |
|:------------ |:------------ |:------------------------ |
| D+           | USB_DP       | USB 2.0 host data+       |
| D-           | USB_DM       | USB 2.0 host data-       |
| VBUS         | 5V           | USB bus power            |
| GND          | GND          | Ground                   |

### Serial Communication (to PC)

| Signal       | Luckfox Pin  | Description              |
|:------------ |:------------ |:------------------------ |
| TX           | UART2_TX (8) | 2M baud data to PC       |
| RX           | UART2_RX (10)| 2M baud data from PC      |
| GND          | GND          | Common ground            |

> **Note**: The pin numbers above are for the Luckfox Pico Pro Max. Adjust for your specific Luckfox board variant. The GPIO numbers in `hub_config.h` must match the Linux GPIO numbering (e.g., GPIO1_D2 = chip 1, offset 26 = number 58 on some kernels). Check `/sys/kernel/debug/gpio` on the target.

### External Camera Sync (Optional)

| Signal       | Luckfox Pin  | Description              |
|:------------ |:------------ |:------------------------ |
| TRIGGER_OUT  | GPIO1_D4 (57)| Pulse output to camera    |
| TRIGGER_IN   | GPIO1_D5 (58)| Edge input from camera    |

---

## Prerequisites

### Cross-Compilation Toolchain
The Luckfox SDK provides the Rockchip ARM toolchain:

```bash
git clone https://github.com/LuckfoxTECH/luckfox-pico.git
cd luckfox-pico/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf/
source env_install_toolchain.sh
```

Verify:
```bash
arm-rockchip830-linux-uclibcgnueabihf-gcc --version
```

### Build Dependencies (host)
- `make`, `gcc`, `zlib1g-dev` (for host CRC testing if needed)

### Target Dependencies (on RV1106)
The Luckfox Buildroot image should include:
- Kernel with I2C, UART, USB UVC, GPIO sysfs support
- `/dev/i2c-0` — I2C bus device
- `/dev/ttyS0` (or `/dev/ttyS2`) — UART device
- `/dev/video0` — V4L2 camera device
- GPIO sysfs (enabled in kernel config)

---

## Building

```bash
cd sensor_array_rv1106g3/

# Build
./build.sh

# The binary is at build/sensor_hub
```

### Manual cross-compilation

```bash
export CROSS_COMPILE=arm-rockchip830-linux-uclibcgnueabihf-
make -j$(nproc)
```

### Building with the Luckfox SDK

To integrate into the Luckfox SDK's build system, copy this project into:
```
luckfox-pico/project/app/sensor_array_rv1106g3/
```
Then:
```bash
cd luckfox-pico
./build.sh app
```

---

## Flashing and Running

### 1. Copy binary to board
```bash
# Via scp (adjust IP):
scp build/sensor_hub root@192.168.1.100:/root/

# Or via SD card
```

### 2. Set up GPIO permissions
```bash
# On the Luckfox (as root):
echo 55 > /sys/class/gpio/export    # ToF INT pin
echo in > /sys/class/gpio/gpio55/direction
echo falling > /sys/class/gpio/gpio55/edge

echo 56 > /sys/class/gpio/export    # ToF LPn pin
echo out > /sys/class/gpio/gpio56/direction
echo 1 > /sys/class/gpio/gpio56/value
```

### 3. Run
```bash
chmod +x /root/sensor_hub
/root/sensor_hub
```

Expected output:
```
Sensor Hub for RV1106G3 starting...
Hub running. Press Ctrl+C to stop.
```

### 4. Connect PC Visualizer
```bash
cd pc_visualizer/
pip install -r requirements.txt
python main.py --port /dev/ttyUSB0 --baud 2000000
```

---

## Configuration

All pin assignments and tuning constants are in `main/hub_config.h`:

| Constant            | Default          | Description                   |
|:------------------- |:---------------- |:---------------------------- |
| `I2C_DEVICE_PATH`  | `/dev/i2c-0`     | I2C bus device               |
| `PIN_TOF_INT`      | `55`             | ToF interrupt GPIO number    |
| `PIN_TOF_LPN`      | `56`             | ToF reset/low-power GPIO     |
| `UART_DEVICE_PATH` | `/dev/ttyS0`     | Serial port for PC comm      |
| `SERIAL_BAUD`      | `2000000`        | UART baud rate               |
| `USE_UVC_CAMERA`   | `1`              | Enable V4L2 camera           |
| `UVC_AUTOSTART`    | `0`              | Auto-start camera on boot    |
| `CAM_JPEG_MAX`     | `55 * 1024`      | Max JPEG frame size          |

---

## Key Differences from ESP32-S3 Version

| Aspect           | ESP32-S3           | RV1106G3 (this port)         |
|:---------------- |:------------------ |:---------------------------- |
| **OS**           | FreeRTOS (ESP-IDF) | Linux 5.10 (Buildroot)       |
| **Threads**      | FreeRTOS tasks     | POSIX pthreads               |
| **I2C**          | ESP-IDF driver API | Linux `/dev/i2c-N` + ioctl   |
| **UART**         | ESP-IDF UART       | Linux termios / termios2     |
| **USB Camera**   | usb_stream library | V4L2 + mmap + ioctl          |
| **CRC**          | esp_crc32_le()     | zlib `crc32()`               |
| **Timestamp**    | esp_timer_get_time | `clock_gettime(MONOTONIC)`   |
| **GPIO INT**     | ESP-IDF GPIO ISR   | Linux sysfs + poll(POLLPRI)  |
| **Memory**       | heap_caps_malloc   | malloc()                     |
| **Protocol**     | **Identical**      | **Identical**                |
| **Frame format** | **Identical**      | **Identical**                |

---

## Performance Notes

- The RV1106G3 Cortex-A7 @ 1.2GHz is significantly faster than ESP32-S3 @ 240MHz
- V4L2 mmap + dedicated capture thread eliminates USB host enumeration delays
- The SLIP protocol supports streaming all three sensors at their maximum rates simultaneously
- Baud rate of 2Mbaud over UART provides ~200KB/s throughput
- Frame rate is governed by the slowest sensor (typically MLX90640 at 16-32Hz)

---

## Troubleshooting

### I2C devices not found
```bash
i2cdetect -y 0          # scan I2C bus
cat /sys/kernel/debug/gpio  # check GPIO states
```

### V4L2 camera not detected
```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### Serial port issues
```bash
stty -F /dev/ttyS0 2000000 cs8 -parenb -cstopb raw
```

### Permission denied on GPIO
```bash
echo 55 > /sys/class/gpio/export    # must be root
```

---

## License

This project follows the same license as the parent ESP32-S3 Sensor Hub project. The ST VL53LMZ API and Melexis MLX90640 API retain their original vendor licenses.

---

## Credits

- **STMicroelectronics** — VL53LMZ ULD (Ultra Lite Driver)
- **Melexis** — MLX90640 API
- **Luckfox** — Pico SDK and toolchain
- **Original ESP32-S3 implementation** — `sensor_array_idf/` in this repository