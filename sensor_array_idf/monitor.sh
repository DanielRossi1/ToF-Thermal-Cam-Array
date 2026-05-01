#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
IDF_BASE="$SCRIPT_DIR/../esp/esp-idf"

. "$IDF_BASE/export.sh"

cd "$PROJECT_DIR"
idf.py -p /dev/ttyACM0 --baud 115200 monitor