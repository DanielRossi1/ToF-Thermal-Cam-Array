#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"

CROSS_COMPILE="${CROSS_COMPILE:-}"
TOOLCHAIN_CACHE_DIR="${TOOLCHAIN_CACHE_DIR:-$PROJECT_DIR/.toolchain}"
TOOLCHAIN_URL="${TOOLCHAIN_URL:-https://developer.arm.com/-/media/Files/downloads/gnu/12.3.rel1/binrel/arm-gnu-toolchain-12.3.rel1-x86_64-arm-none-linux-gnueabihf.tar.xz}"
TOOLCHAIN_PREFIX="${TOOLCHAIN_PREFIX:-arm-none-linux-gnueabihf-}"

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
    mkdir -p "$TOOLCHAIN_CACHE_DIR"

    find_toolchain_bin() {
        find "$TOOLCHAIN_CACHE_DIR" -type f -name "${TOOLCHAIN_PREFIX}gcc" -print -quit 2>/dev/null || true
    }

    TOOLCHAIN_GCC="$(find_toolchain_bin)"
    if [ -z "$TOOLCHAIN_GCC" ]; then
        echo "No toolchain found. Downloading to $TOOLCHAIN_CACHE_DIR ..."
        ARCHIVE_NAME="${TOOLCHAIN_URL##*/}"
        ARCHIVE_PATH="$TOOLCHAIN_CACHE_DIR/$ARCHIVE_NAME"

        if command -v curl &>/dev/null; then
            curl -fL --retry 3 --retry-all-errors -o "$ARCHIVE_PATH" "$TOOLCHAIN_URL"
        elif command -v wget &>/dev/null; then
            wget -O "$ARCHIVE_PATH" "$TOOLCHAIN_URL"
        else
            echo "ERROR: Neither curl nor wget found to download toolchain." >&2
            exit 1
        fi

        if ! tar -tf "$ARCHIVE_PATH" >/dev/null 2>&1; then
            echo "ERROR: Downloaded file is not a valid tar archive." >&2
            echo "  URL: $TOOLCHAIN_URL" >&2
            rm -f "$ARCHIVE_PATH"
            exit 1
        fi

        tar -xf "$ARCHIVE_PATH" -C "$TOOLCHAIN_CACHE_DIR"
        TOOLCHAIN_GCC="$(find_toolchain_bin)"
    fi

    if [ -n "$TOOLCHAIN_GCC" ]; then
        TOOLCHAIN_BIN="$(dirname "$TOOLCHAIN_GCC")"
        export PATH="$TOOLCHAIN_BIN:$PATH"
        CROSS_COMPILE="$TOOLCHAIN_PREFIX"
        echo "Using downloaded toolchain: $TOOLCHAIN_BIN"
    fi
fi

if [ -z "$CROSS_COMPILE" ]; then
    echo "ERROR: No ARM cross-compiler found." >&2
    echo "  Set: CROSS_COMPILE=/path/to/arm-none-linux-gnueabihf- ./build.sh" >&2
    echo "  Or override: TOOLCHAIN_URL=... TOOLCHAIN_PREFIX=... ./build.sh" >&2
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
    echo "Deploying to pico@${TARGET_IP}..."
    scp -o StrictHostKeyChecking=no \
        "$PROJECT_DIR/build/sensor_hub" \
        "pico@${TARGET_IP}:/home/pico/sensor_hub"
    echo "Run: ssh pico@${TARGET_IP} /home/pico/sensor_hub"
fi
