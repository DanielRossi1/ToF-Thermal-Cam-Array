# ESP32-S3 Sensor Hub — Control + Data Protocol (v1)

This firmware streams synchronized sensor frames to the PC **and** accepts runtime configuration commands over the same physical link.

- Physical link (default): **Right USB‑C (CH343p USB‑UART)** @ `2000000` baud
- USB camera: **native USB (GPIO19/20 = D-/D+)** used in host mode via `USB_STREAM`
- Framing: **SLIP** (robust packet boundaries)
- Integrity: **CRC32** (little‑endian, same as ESP-IDF `esp_crc32_le()`)

Important:

- GPIO19/20 are reserved for USB D-/D+. They must not be used as normal GPIO while USB is active.
- External camera sync (trigger/strobe) is only supported if you wire it to different GPIOs and build with `HUB_USE_CAM_SYNC=1`.

## 1) Framing (SLIP)

Each message is sent as a SLIP frame:

- `END = 0xC0`
- `ESC = 0xDB`
- `ESC_END = 0xDC` (represents `END`)
- `ESC_ESC = 0xDD` (represents `ESC`)

On the wire:

- `END` + *encoded bytes* + `END`

Encoding rules:

- byte `0xC0` → `0xDB 0xDC`
- byte `0xDB` → `0xDB 0xDD`
- all other bytes unchanged

## 2) Message layout

Inside the SLIP payload:

```
[MsgHeader][payload...][crc32]
```

### 2.1 Header (`hub::MsgHeader`)

Little-endian, packed:

- `uint32 magic` = `0x53454E53` (`'SENS'`)
- `uint16 version` = `1`
- `uint16 type`
  - `1` = Frame
  - `2` = Cmd
  - `3` = Resp
  - `4` = Event
- `uint32 seq` (sender sequence)
- `uint64 ts_us` (sender timestamp; 0 if not applicable)
- `uint32 payload_len` (bytes)

### 2.2 CRC32

- `uint32 crc32_le` computed over **header + payload** (not including the CRC field)
- Uses ESP-IDF `esp_crc32_le(0, data, len)` semantics

Python note: in many setups `binascii.crc32(data) & 0xffffffff` matches, but if you see CRC mismatches, verify against the device by sending a known packet and comparing.

## 3) Frame messages (`type=1`)

Payload = `hub::FrameFixedV1` followed by `cam.len` bytes of MJPEG.

Key fields:

- `frame_seq`: frame counter
- `hub_ts_us`: timestamp when the hub assembled the frame
- `flags`: bitmask
  - bit0: ToF valid
  - bit1: MLX valid
  - bit2: Camera valid
  - bit3: Camera sync valid

### 3.1 ToF (`VL53L8CH`) data

- Acquisition is **synchronized by the ToF INT pin**.
- Timestamp `tof.ts_us` is taken at the **INT edge** (best timing anchor).
- Provides per-zone **multi-target** data (up to 4 targets/zone).

### 3.2 Thermal (`MLX90640`) data

- Uses Adafruit MLX90640 library.
- Payload contains:
  - ambient temperature `ta_cC` (centi‑°C)
  - 768 pixel temperatures `frame_cC[]` (centi‑°C)

### 3.3 Camera sync (external GPIO, optional)

- In this project, GPIO19/20 are reserved for USB D-/D+ (UVC host).
- Camera sync over extra GPIO is optional and disabled by default (`HUB_USE_CAM_SYNC=0`).
- If you enable cam-sync, use dedicated non-USB pins in `hub_config.h`.

### 3.4 Camera bytes

If UVC support is enabled (`HUB_USE_UVC_CAMERA=1`):

- `cam.cfg.format_fourcc` = `MJPG`
- `cam.len` bytes of JPEG follow immediately after the fixed payload

## 4) Command messages (`type=2`) and responses (`type=3`)

- CMD payload is a UTF‑8 text line (max ~255 bytes recommended).
- RESP payload is UTF‑8 text.

### Supported commands

- `PING`
- `HELP`
- `GET INFO`
- `STREAM enable=0|1 mode=all|tof|mlx|cam|none`
- `SET I2C clock_hz=<n>`
Diagnostics:

- `DIAG I2C SCAN`

ToF:

- `SET TOF side=4|8 hz=<n> it_ms=<n> continuous=0|1`

Thermal:

- `SET MLX mode=chess|interleaved res=16|17|18|19 refresh=0.5|1|2|4|8|16|32|64`

Camera sync:

- `SET CAMSYNC enabled=0|1 period_us=<n> pulse_us=<n>`

UVC camera (only if enabled):

- `SET CAM enable=0|1 w=<n> h=<n> interval_us=<n>`

Notes:

- Camera autostart is disabled by default (`HUB_UVC_AUTOSTART=0`) so ToF/MLX can always boot even if UVC enum is unstable.
- Enable camera at runtime with `SET CAM enable=1`.

## 5) Minimal Python: send a command

This snippet sends one `Cmd` message and prints `Resp/Event` texts.

```python
import serial, struct, binascii

END=0xC0; ESC=0xDB; ESC_END=0xDC; ESC_ESC=0xDD
MAGIC=0x53454E53
VERSION=1
TYPE_CMD=2

def slip_encode(data: bytes) -> bytes:
    out = bytearray([END])
    for b in data:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)

def build_cmd(seq: int, text: str) -> bytes:
    payload = text.encode('utf-8')
    hdr = struct.pack('<IHHIQI', MAGIC, VERSION, TYPE_CMD, seq, 0, len(payload))
    crc = binascii.crc32(hdr + payload) & 0xffffffff
    return slip_encode(hdr + payload + struct.pack('<I', crc))

ser = serial.Serial('/dev/ttyUSB0', 2000000, timeout=0.2)
ser.write(build_cmd(1, 'GET INFO'))
ser.flush()

# Note: to fully decode responses, implement SLIP decode + header parse.
print('sent')
```

## 6) Practical notes

## 7) Arduino IDE "Tools" configuration (ESP32S3 Dev Module)

Set these exactly for this firmware:

- USB CDC On Boot: `Disabled`
- CPU Frequency: `240MHz (WiFi/BT)`
- USB DFU On Boot: `Disabled`
- Events Run On: `Core 1`
- Flash Mode: `QIO 80MHz`
- Flash Size: `16MB (128Mb)`
- JTAG Adapter: `Disabled`
- Arduino Runs On: `Core 1`
- USB Firmware MSC On Boot: `Disabled`
- Partition Scheme: `Huge APP (3MB No OTA/1MB SPIFFS)`
- PSRAM: `OPI PSRAM`
- Upload Mode: `UART0 / Hardware CDC`
- USB Mode: `USB-OTG (TinyUSB)`

Notes:

- Data streaming to PC uses the CH343p UART path (right USB-C) at `2000000` baud.
- UVC camera uses native USB host on GPIO19/20, so those pins must not be used as generic GPIO.
- If `USB Mode` is set incorrectly, camera host enum may panic in USB HCD (`_uvc_uac_device_enum` / `hcd_urb_enqueue`).

- Once the hub starts, it avoids raw `Serial.println()` to prevent corrupting the binary stream.
- If you change settings, expect 1–2 frames of transient behavior while sensors reconfigure.
