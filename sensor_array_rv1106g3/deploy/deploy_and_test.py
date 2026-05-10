#!/usr/bin/env python3
"""
Full deploy and test script for sensor_hub on Luckfox Pico Max.
1. Uploads the binary to /root/sensor_hub
2. Installs /etc/init.d/S99sensorhub for boot auto-start
3. Sets up GPIOs, starts the service
4. Runs TCP protocol test (PING, GET INFO, DIAG I2C SCAN, STREAM)
"""
import paramiko
import base64
import os
import sys
import time
import struct
import socket
import zlib

# ── Config ──────────────────────────────────────────────────────────────────
HOST = "192.168.1.67"
PORT = 22
USER = "root"
PASSWORD = "luckfox"
TCP_PORT = 9000

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BIN_PATH = os.path.join(PROJECT_DIR, "build", "sensor_hub")
INIT_SCRIPT = os.path.join(PROJECT_DIR, "deploy", "S99sensorhub")

# ── SSH helpers ─────────────────────────────────────────────────────────────

def ssh_connect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, port=PORT, username=USER, password=PASSWORD, timeout=15)
    return ssh

def ssh_exec(ssh, cmd):
    stdin, stdout, stderr = ssh.exec_command(cmd)
    return stdout.read().decode().strip(), stderr.read().decode().strip()

# ── Protocol (TCP client for sensor_hub) ────────────────────────────────────

SLIP_END = 0xC0; SLIP_ESC = 0xDB; SLIP_ESC_END = 0xDC; SLIP_ESC_ESC = 0xDD
MAGIC = 0x53454E53; VERSION = 1
MSG_FRAME = 1; MSG_CMD = 2; MSG_RESP = 3; MSG_EVENT = 4
HDR_FMT = '<IHHIQI'; HDR_SIZE = 24; CRC_SIZE = 4
_cmd_seq = 0
TN = {1:'FRAME', 2:'CMD', 3:'RESP', 4:'EVENT'}

def slip_encode(data):
    out = bytearray([SLIP_END])
    for b in data:
        if b == SLIP_END: out+=bytes([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out+=bytes([SLIP_ESC, SLIP_ESC_ESC])
        else: out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def build_cmd(text):
    global _cmd_seq
    _cmd_seq += 1
    pl = text.encode()
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, MSG_CMD, _cmd_seq, 0, len(pl))
    crc = zlib.crc32(hdr + pl) & 0xFFFFFFFF
    return slip_encode(hdr + pl + struct.pack('<I', crc))

def parse_message(raw):
    if len(raw) < HDR_SIZE + CRC_SIZE: return None
    magic, ver, mtype, seq, ts_us, plen = struct.unpack_from(HDR_FMT, raw, 0)
    if magic != MAGIC or ver != VERSION: return None
    if len(raw) != HDR_SIZE + plen + CRC_SIZE: return None
    payload = raw[HDR_SIZE:HDR_SIZE + plen]
    crc_rx, = struct.unpack_from('<I', raw, HDR_SIZE + plen)
    if zlib.crc32(raw[:HDR_SIZE + plen]) & 0xFFFFFFFF != crc_rx: return None
    return mtype, seq, ts_us, payload

def get_packets(data):
    result, pkt, esc = [], bytearray(), False
    for b in data:
        if b == SLIP_END:
            if pkt: result.append(bytes(pkt)); pkt.clear()
            esc = False
        elif esc:
            esc = False
            if b == SLIP_ESC_END: pkt.append(SLIP_END)
            elif b == SLIP_ESC_ESC: pkt.append(SLIP_ESC)
        elif b == SLIP_ESC: esc = True
        else: pkt.append(b)
    return result

def tcp_test():
    """Run full TCP protocol test against sensor_hub."""
    print("\n" + "="*60)
    print("TCP Protocol Test against sensor_hub (port 9000)")
    print("="*60)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    sock.connect((HOST, TCP_PORT))
    print(f"Connected to {HOST}:{TCP_PORT}")

    buf = bytearray()
    def recv_msgs(timeout=3):
        nonlocal buf
        sock.settimeout(timeout)
        try:
            while True:
                data = sock.recv(4096)
                if not data: break
                buf.extend(data)
        except socket.timeout: pass
        sock.settimeout(10)
        pkts = get_packets(bytes(buf))
        buf.clear()
        msgs = []
        for p in pkts:
            m = parse_message(p)
            if m: msgs.append(m)
        return msgs

    def send_and_recv(cmd, wait=1):
        print(f"\n>>> {cmd}")
        sock.sendall(build_cmd(cmd))
        time.sleep(wait)
        msgs = recv_msgs(2)
        for mt, seq, ts, pl in msgs:
            print(f"  [{TN.get(mt,'?')}]")
            for line in pl.decode(errors='replace').strip().split('\n'):
                print(f"    {line}")
        return msgs

    # Drain boot event
    print("Draining BOOT event...")
    time.sleep(1)
    msgs = recv_msgs(3)
    for mt, seq, ts, pl in msgs:
        print(f"  [{TN.get(mt,'?')}] {pl.decode(errors='replace').strip()[:120]}")

    # Test commands
    send_and_recv("PING", 0.3)
    send_and_recv("GET INFO", 0.3)
    send_and_recv("DIAG I2C SCAN", 0.3)

    # Stream test
    print("\n>>> STREAM enable=1 mode=all")
    sock.sendall(build_cmd("STREAM enable=1 mode=all"))
    time.sleep(0.3)
    recv_msgs(2)

    print("Collecting frames for 3 seconds...")
    time.sleep(3)
    msgs = recv_msgs(3)
    frames = [m for m in msgs if m[0] == MSG_FRAME]
    print(f"  Received {len(frames)} frames in 3s")

    for i, (mt, seq, ts, pl) in enumerate(frames[:3]):
        flags, = struct.unpack_from('<I', pl, 8)
        flags_str = []
        if flags & 1: flags_str.append("TOF")
        if flags & 2: flags_str.append("MLX")
        if flags & 4: flags_str.append("CAM")
        if flags & 8: flags_str.append("SYNC")
        print(f"  Frame seq={seq} flags=0x{flags:08X} [{','.join(flags_str) if flags_str else 'NONE'}] len={len(pl)}")

    if len(frames) > 3:
        print(f"  ... and {len(frames)-3} more frames")

    # STREAM disable
    send_and_recv("STREAM enable=0 mode=none", 0.3)

    sock.close()
    print("\nTCP test complete!")
    return True

# ── Deploy ──────────────────────────────────────────────────────────────────

def deploy():
    print("="*60)
    print("Sensor Hub Deploy to Luckfox Pico Max")
    print(f"Target: {USER}@{HOST}")
    print("="*60)

    # ── 1. Connect ──
    print("\n[1/5] Connecting via SSH...")
    ssh = ssh_connect()
    print("  Connected OK")

    # ── 2. Upload binary ──
    print("\n[2/5] Uploading sensor_hub binary...")
    with open(BIN_PATH, "rb") as f:
        data = f.read()
    print(f"  Binary size: {len(data)} bytes ({len(data)/1024:.1f} KB)")
    
    ssh.exec_command("killall -9 sensor_hub 2>/dev/null; sleep 1; rm -f /root/sensor_hub")
    
    encoded = base64.b64encode(data)
    stdin, stdout, stderr = ssh.exec_command("base64 -d > /root/sensor_hub && chmod +x /root/sensor_hub")
    stdin.write(encoded)
    stdin.channel.shutdown_write()
    out = stdout.read().decode().strip()
    if out:
        print(f"  {out}")
    print("  Binary uploaded")

    # ── 3. Install init.d auto-start script ──
    print("\n[3/5] Installing /etc/init.d/S99sensorhub...")
    with open(INIT_SCRIPT, "rb") as f:
        init_data = f.read()
    enc_init = base64.b64encode(init_data)
    stdin, stdout, stderr = ssh.exec_command("base64 -d > /etc/init.d/S99sensorhub && chmod +x /etc/init.d/S99sensorhub")
    stdin.write(enc_init)
    stdin.channel.shutdown_write()
    out, err = ssh_exec(ssh, "ls -la /etc/init.d/S99sensorhub")
    print(f"  {out}")

    # ── 4. Start sensor_hub ──
    print("\n[4/5] Starting sensor_hub...")
    out, err = ssh_exec(ssh, "/etc/init.d/S99sensorhub start")
    print(f"  {out}")
    if err:
        print(f"  stderr: {err}")

    time.sleep(4)
    out, err = ssh_exec(ssh, "cat /tmp/sensor_hub.log")
    print(f"  Log: {out[:500]}")
    
    out, err = ssh_exec(ssh, "ps | grep sensor | grep -v grep")
    if out:
        print(f"  Process: {out}")
    else:
        print("  WARNING: sensor_hub not running!")

    ssh.close()
    print("\n[5/5] Deploy complete!")

# ── Main ────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "test":
        tcp_test()
    else:
        deploy()
        tcp_test()