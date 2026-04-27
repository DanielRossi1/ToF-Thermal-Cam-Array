# ESP32-S3 Synchronized Sensor Hub

## Hardware

| Component       | Part            | Interface |
|-----------------|-----------------|-----------|
| MCU             | ESP32-S3 DevKitC-1 | —      |
| ToF             | VL53L8CH (RTrobot board) | I2C 0x52 |
| Thermal camera  | MLX90640        | I2C 0x33  |
| RGB camera      | USB UVC module  | USB-OTG   |
| PC connection   | —               | USB-UART  |

---

## Wiring table

### I2C bus (shared, 4.7kΩ pull-ups SDA→3V3 and SCL→3V3)

| Signal  | ESP32-S3 pin | VL53L8CH pad | MLX90640 pad |
|---------|-------------|--------------|--------------|
| SDA     | GPIO 8      | SDA          | SDA          |
| SCL     | GPIO 9      | SCL          | SCL          |
| VCC     | 3V3         | VCC          | VCC          |
| GND     | GND         | GND          | GND          |

### VL53L8CH extra pins

| Signal | ESP32-S3 pin | Notes                          |
|--------|-------------|--------------------------------|
| INT    | GPIO 4      | Active-low interrupt, pull-up  |
| XSHUT  | GPIO 5      | Pull low to reset (optional)   |

### USB camera (UVC module)

| Camera pad | ESP32-S3 pin       | Notes                         |
|------------|--------------------|-------------------------------|
| DM (D+)    | USB-OTG D+         | Direct connection             |
| DP (D−)    | USB-OTG D−         | Direct connection             |
| 5V         | USB-OTG VBUS (5V)  | From OTG port                 |
| GND        | USB-OTG GND        | Common ground                 |

### PC connection
Connect the **USB-UART** port of the DevKitC-1 to your PC.  
The USB-OTG port is fully occupied by the camera.

---

## Arduino libraries (install via Library Manager)

1. **VL53L8CX** — search "VL53L8CX" → install Pololu or ST version
2. **SparkFun MLX90640** — search "SparkFun MLX90640"
3. **esp32-camera** — bundled with ESP32 Arduino core ≥ 2.0.14
4. **usb_stream** (Espressif) — for UVC host:
   - IDF component: `idf.py add-dependency "espressif/usb_stream"`
   - Or via Arduino component manager

### Arduino IDE board settings

| Setting            | Value                  |
|--------------------|------------------------|
| Board              | ESP32S3 Dev Module     |
| USB CDC On Boot    | **Disabled**           |
| USB Mode           | **USB-OTG**            |
| PSRAM              | OPI PSRAM              |
| Flash Size         | 8MB (or match your board) |
| Partition Scheme   | Huge APP (3MB No OTA)  |

> ⚠️  "USB CDC On Boot: Disabled" is critical. If enabled, Serial goes to CDC
> (OTG port) instead of UART, and your camera won't work.

---

## Python setup

```bash
pip install pyserial numpy opencv-python matplotlib
```

### Run with live visualization
```bash
python pc_reader.py --port /dev/ttyUSB0
```

### Run headless (print stats only)
```bash
python pc_reader.py --port /dev/ttyUSB0 --headless
```

### Save all frames to disk
```bash
python pc_reader.py --port /dev/ttyUSB0 --save ./frames
```

Each synchronized frame saves as:
- `00000001_tof_dist.npy`   — uint16 (8×8) distances in mm
- `00000001_tof_sigma.npy`  — uint16 (8×8) sigma estimates
- `00000001_tof_status.npy` — uint8  (8×8) target status (5 = valid)
- `00000001_thermal.npy`    — float32 (24×32) temperatures °C
- `00000001_camera.jpg`     — JPEG from USB camera
- `00000001_meta.txt`       — one-line metadata

---

## Synchronization strategy

```
VL53L8CH INT ──► GPIO4 ISR ──► g_tof_ready = true
                                     │
                              loop() detects flag
                                     │
                         ┌───────────┼───────────┐
                         ▼           ▼           ▼
                     read_tof()  read_mlx()  uvc_capture()
                         │           │           │
                         └───────────┴───────────┘
                                     │
                              send_packet() over UART
```

The VL53L8CH fires INT at 30Hz. The MLX90640 is set to 32Hz (slightly faster),
so its data is always fresh when the ISR triggers. The camera grab is triggered
at the same instant. Total jitter between sensors is < 1ms.

---

## UVC camera note

The `camera_init()` / `uvc_capture()` / `uvc_release()` functions in the `.ino`
are **stubs**. Replace them with the actual Espressif `usb_stream` API calls:

```cpp
// Typical usb_stream init
usb_host_config_t host_cfg = {
    .skip_phy_setup = false,
    .intr_flags = ESP_INTR_FLAG_LEVEL1
};
usb_host_install(&host_cfg);
uvc_host_driver_install(NULL);

uvc_host_stream_config_t stream_cfg = {
    .callback = frame_callback,
    .frame_width = 320,
    .frame_height = 240,
    .frame_interval = 333333,  // 30fps = 1/30s in 100ns units
    .interface_num = 1,
    .interface_alt_num = 1,
};
uvc_host_stream_open(0x0000, 0x0000, &stream_cfg, portMAX_DELAY, &uvc_handle);
```

Adjust `frame_width`, `frame_height`, and VID/PID to match your camera module.
