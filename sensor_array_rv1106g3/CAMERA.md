# USB Camera on Luckfox Pico Max — Complete Guide
### Platform: Ubuntu 22.04 (SD card), RV1106G3

---

## What This Guide Covers

Getting a USB UVC camera working on the Luckfox Pico Max, including:
- Flashing Ubuntu to SD card correctly
- Booting from SD instead of NAND
- Enabling USB host mode
- The physical wiring fix that actually solves error -71

---

## Root Cause Summary

> **The Luckfox Pico Max only has a USB-C port. Attaching camera D+ and D- to board's USB pads and powering it via VBUS (board powered via USB-C) does not work, causing `error -71` on every device, including USB drives. This is not a kernel or driver problem.**

The fix is to connect a PD-Trigger to VBUS and power both the board and the camera from the same source. Nothing should be connected then to the USB-C port. Data will be sent via Ethernet.

---

## Prerequisites

- Luckfox Pico Max (RV1106G3)
- MicroSD card (≥8GB)
- Ubuntu host PC
- Bench power supply or stable 5V source
- Jumper wires
- Ubuntu SD image: `Ubuntu_Luckfox_Pico_Max_MicroSD_250313` (or newer)
- Luckfox SDK: `luckfox-pico` cloned from GitHub

---

## Step 1 — Flash Ubuntu to SD Card

The Ubuntu MicroSD image **cannot** be flashed as a single `dd` of `update.img`. Each partition must be written at a specific sector offset defined in `sd_update.txt`.

```bash
cd ~/path/to/Ubuntu_Luckfox_Pico_Max_MicroSD_250313/

sudo umount /dev/sdX* 2>/dev/null

# Wipe first
sudo dd if=/dev/zero of=/dev/sdX bs=1M count=10

# Flash each partition at correct offset
sudo dd if=env.img      of=/dev/sdX seek=$((0x0))      bs=512 conv=notrunc
sudo dd if=idblock.img  of=/dev/sdX seek=$((0x40))     bs=512 conv=notrunc
sudo dd if=uboot.img    of=/dev/sdX seek=$((0x440))    bs=512 conv=notrunc
sudo dd if=boot.img     of=/dev/sdX seek=$((0x640))    bs=512 conv=notrunc
sudo dd if=oem.img      of=/dev/sdX seek=$((0x10640))  bs=512 conv=notrunc
sudo dd if=userdata.img of=/dev/sdX seek=$((0x110640)) bs=512 conv=notrunc
sudo dd if=rootfs.img   of=/dev/sdX seek=$((0x190640)) bs=512 conv=notrunc

sync
sudo eject /dev/sdX
```

> Replace `/dev/sdX` with your actual SD card device (check with `lsblk`).
> The offsets above are from `sd_update.txt` — verify them against your image version if it differs.

---

## Step 2 — Erase NAND Flash

The board boots from NAND by default. You must erase it so the board falls through to SD.

SSH into your existing system first:
```bash
ssh root@<board-ip>
```

Then erase all NAND partitions:
```bash
flash_eraseall /dev/mtd0
flash_eraseall /dev/mtd1
flash_eraseall /dev/mtd2
flash_eraseall /dev/mtd3
flash_eraseall /dev/mtd4
flash_eraseall /dev/mtd5
flash_eraseall /dev/mtd6
```

> **Warning:** This erases the bootloader too. The board will enter MaskRom mode if no SD card is present. Always have the SD card ready before doing this.

Do **not** reboot from SSH — just power off.

---

## Step 3 — Boot from SD

1. Insert the SD card into the Luckfox Pico Max
2. Power on — no button holding required
3. Wait ~15 seconds for Ubuntu to boot

Find the board on your network:
```bash
nmap -sn 192.168.1.0/24 | grep -B1 -A1 -i "luckfox\|rock"
```

SSH in:
```bash
ssh pico@<board-ip>
# default password: luckfox
```

---

## Step 4 — Enable USB Host Mode

The USB port defaults to device/gadget mode (https://wiki.luckfox.com/Luckfox-Pico-Pro-Max/USB). Switch it to host:

```bash
sudo su
echo host > /sys/devices/platform/ff3e0000.usb2-phy/otg_mode
```

Verify:
```bash
cat /sys/devices/platform/ff3e0000.usb2-phy/otg_mode
# Should output: host
```

> This setting does not persist across reboots. To make it permanent, add it to `/etc/rc.local` or a systemd service.

---

## Step 5 — Physical Wiring (The Actual Fix)

**Do not use a USB-C to USB-A adapter.** Even adapters that appear to work for charging will corrupt USB data signals on the RV1106, producing `error -71` on every device.

### Wire the camera directly to the USB-C pads:

| Camera USB-A Pin | Signal | Connect To |
|---|---|---|
| Pin 1 | VBUS (5V) | External 5V supply (bench PSU or PD-Trigger) |
| Pin 2 | D- | USB-C pad D- on board |
| Pin 3 | D+ | USB-C pad D+ on board |
| Pin 4 | GND | Common GND |

**Notes:**
- Power the camera from an external source, not just from the board's VBUS
- Keep wires short (under 10cm), no need to twist them at this length
- USB-C has two orientation sets of D+/D- pads (D1+/D1- and D2+/D2-), try one pair, if it doesn't work try the other or use the multimeter to infer the correct order.
- Check the Luckfox Pico Max schematic at [wiki.luckfox.com](https://wiki.luckfox.com) for exact pad locations

---

## Step 6 — Verify Camera

Watch dmesg live while connecting:
```bash
dmesg -w
```

You should see:
```
usb 1-1: new high-speed USB device number 2 using xhci-hcd
uvcvideo: Found UVC 1.00 device USB Camera (xxxx:xxxx)
```

Check device nodes:
```bash
lsusb          # camera should appear with its ID
ls /dev/video* # multiple nodes will exist
```

Capture a test frame:
```bash
sudo fswebcam -d /dev/video0 test.jpg
```

If `/dev/video0` gives permission denied, run as root or add your user to the `video` group:
```bash
sudo usermod -aG video pico
# then log out and back in
```

If `/dev/video0` is the wrong node (ISP internal), try higher numbers:
```bash
sudo fswebcam -d /dev/video19 test.jpg
sudo fswebcam -d /dev/video20 test.jpg
```

Copy the image to your PC:
```bash
# On your PC:
scp pico@<board-ip>:/home/pico/test.jpg ~/Desktop/test.jpg
```

---

## Recovery: If You Erased NAND Without SD Ready

If the board has no bootable media and enters MaskRom mode:

1. Plug USB-C into PC — no button holding needed in MaskRom
2. Verify detection:
```bash
lsusb | grep "2207"
# Should show: Fuzhou Rockchip Electronics Company
```
3. Reflash NAND using the SDK:
```bash
cd ~/path/to/luckfox-pico
sudo ./rkflash.sh update
```

---

## Making Host Mode Persistent

Create a systemd service:
```bash
sudo nano /etc/systemd/system/usb-host-mode.service
```

```ini
[Unit]
Description=Set USB OTG to host mode
After=network.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo host > /sys/devices/platform/ff3e0000.usb2-phy/otg_mode'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable usb-host-mode
sudo systemctl start usb-host-mode
```

---

## Fixing MAC Address (Stable DHCP IP)

By default the MAC address is random on each boot. Fix it:

```bash
sudo nano /etc/systemd/network/10-eth0.network
```

```ini
[Match]
Name=eth0

[Network]
DHCP=yes

[Link]
MACAddress=xx:xx:xx:xx:xx:xx
```

Replace `xx:xx:xx:xx:xx:xx` with your board's MAC from `cat /sys/class/net/eth0/address`.

```bash
sudo systemctl enable systemd-networkd
sudo reboot
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `error -71` on all USB devices | Bad USB-C adapter | Wire D+/D- directly |
| `error -71` only on camera | Power starvation | Power camera from external supply |
| No `/dev/video*` after camera detected | UVC module not loaded | `sudo modprobe uvcvideo` |
| `lsusb` shows nothing | Host mode not set | Repeat Step 4 |
| Board doesn't boot from SD | NAND not erased | Repeat Step 2 |
| Board enters MaskRom after NAND erase | No SD card present | Insert SD first, see Recovery section |

---

*Tested on: Luckfox Pico Max (RV1106G3), Ubuntu 22.04 SD image 250313, kernel 5.10.160*