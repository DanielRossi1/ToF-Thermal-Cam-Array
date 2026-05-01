#!/usr/bin/env python3
"""hub_cmd.py — send a single command to the ESP32-S3 hub.

Usage:
  python3 hub_cmd.py /dev/ttyUSB0 "GET INFO"
  python3 hub_cmd.py /dev/ttyUSB0 "SET TOF side=8 hz=15 it_ms=50 continuous=1"

Requirements:
  pip install pyserial
"""

from __future__ import annotations

import sys
import struct
import binascii

import serial

END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD

MAGIC = 0x53454E53
VERSION = 1
TYPE_CMD = 2

HEADER_FMT = "<IHHIQI"


def slip_encode(payload: bytes) -> bytes:
    out = bytearray([END])
    for b in payload:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)


def build_cmd(seq: int, text: str) -> bytes:
    p = text.encode("utf-8")
    hdr = struct.pack(HEADER_FMT, MAGIC, VERSION, TYPE_CMD, seq, 0, len(p))
    crc = binascii.crc32(hdr + p) & 0xFFFFFFFF
    return slip_encode(hdr + p + struct.pack("<I", crc))


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 hub_cmd.py <port> <command>")
        return 2

    port = sys.argv[1]
    cmd = " ".join(sys.argv[2:]).strip()
    if not cmd:
        print("Empty command")
        return 2

    baud = 2_000_000
    with serial.Serial(port, baud, timeout=0.2, write_timeout=0.2) as ser:
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        ser.write(build_cmd(1, cmd))
        ser.flush()

    print("sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
