#!/usr/bin/env python3
"""\
serial_cmd.py — send a one-shot command to an ESP32 over serial.

Usage:
  python3 serial_cmd.py /dev/ttyACM0 p
  python3 serial_cmd.py /dev/serial/by-id/... r

Notes:
  - Use 'p' to pause streaming, 'r' to resume (supported by updated sketches).
"""

import sys
import time

import serial

BAUD = 460800


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 serial_cmd.py <port> <cmd>")
        sys.exit(2)

    port = sys.argv[1]
    cmd = sys.argv[2]
    if len(cmd) != 1:
        print("cmd must be a single character (e.g. p or r)")
        sys.exit(2)

    with serial.Serial(port, BAUD, timeout=1, write_timeout=1) as ser:
        # Avoid toggling DTR/RTS
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass

        time.sleep(0.1)
        ser.write(cmd.encode("ascii"))
        ser.flush()
        time.sleep(0.1)

    print(f"Sent '{cmd}' to {port}")


if __name__ == "__main__":
    main()
