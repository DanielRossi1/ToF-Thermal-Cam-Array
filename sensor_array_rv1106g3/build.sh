#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

CROSS_COMPILE="${CROSS_COMPILE:-}"

if [ -z "$CROSS_COMPILE" ]; then
    LUCKFOX_SDK="${LUCKFOX_SDK:-${HOME}/luckfox-pico}"
    [ ! -d "$LUCKFOX_SDK" ] && LUCKFOX_SDK="$PROJECT_DIR/../luckfox-pico"

    if [ -d "$LUCKFOX_SDK" ]; then
        TOOLCHAIN_DIR="$LUCKFOX_SDK/tools/linux/toolchain/arm-rockchip830-linux-uclibcgnueabihf"
        TOOLCHAIN_BIN="$TOOLCHAIN_DIR/bin"
        if [ -d "$TOOLCHAIN_BIN" ]; then
            [ -f "$TOOLCHAIN_DIR/env_install_toolchain.sh" ] && \
                source "$TOOLCHAIN_DIR/env_install_toolchain.sh"
            export PATH="$TOOLCHAIN_BIN:$PATH"
            CROSS_COMPILE="arm-rockchip830-linux-uclibcgnueabihf-"
            echo "Using Luckfox SDK toolchain"
        fi
    fi
fi

if [ -z "$CROSS_COMPILE" ]; then
    for candidate in arm-rockchip830-linux-uclibcgnueabihf- arm-none-linux-gnueabihf- arm-linux-gnueabihf-; do
        if command -v "${candidate}gcc" &>/dev/null; then
            CROSS_COMPILE="$candidate"
            echo "Using toolchain on PATH: $candidate"
            break
        fi
    done
fi

if [ -z "$CROSS_COMPILE" ]; then
    echo "ERROR: No ARM cross-compiler found." >&2
    echo "  Set: CROSS_COMPILE=/path/to/arm-none-linux-gnueabihf- ./build.sh" >&2
    exit 1
fi

if ! command -v "${CROSS_COMPILE}gcc" &>/dev/null; then
    echo "ERROR: ${CROSS_COMPILE}gcc not found" >&2
    exit 1
fi
echo "Compiler: $(${CROSS_COMPILE}gcc --version | head -1)"

USE_TCP="${USE_TCP:-0}"
USE_CAM="${USE_CAM:-0}"
USE_CAM_SYNC="${USE_CAM_SYNC:-0}"
UVC_AUTOSTART="${UVC_AUTOSTART:-0}"

cd "$PROJECT_DIR"
make clean 2>/dev/null || true
make -j"$(nproc)" \
    CROSS_COMPILE="$CROSS_COMPILE" \
    USE_TCP="$USE_TCP" \
    USE_CAM="$USE_CAM" \
    USE_CAM_SYNC="$USE_CAM_SYNC" \
    UVC_AUTOSTART="$UVC_AUTOSTART"

echo ""
echo "Build complete: $PROJECT_DIR/build/sensor_hub"
file "$PROJECT_DIR/build/sensor_hub"

if [ "${1:-}" = "deploy" ]; then
    TARGET_IP="${TARGET_IP:-192.168.42.1}"
    echo "Deploying to root@${TARGET_IP}..."
    scp -o StrictHostKeyChecking=no \
        "$PROJECT_DIR/build/sensor_hub" \
        "root@${TARGET_IP}:/userdata/sensor_hub"
    echo "Run: ssh root@${TARGET_IP} /userdata/sensor_hub"
fi
