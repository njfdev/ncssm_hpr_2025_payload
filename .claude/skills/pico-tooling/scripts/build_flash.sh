#!/bin/bash
# Build and flash pico_logger firmware
# Automatically resets device to BOOTSEL if running, or waits for manual BOOTSEL
#
# Usage:
#   build_flash.sh              # Build release and flash
#   build_flash.sh --debug      # Build debug and flash

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../../../../pico_logger" && pwd)"

BUILD_TYPE="--release"
if [ "$1" = "--debug" ]; then
    BUILD_TYPE=""
fi

cd "$PROJECT_DIR"

echo "=== Building pico_logger ==="
cargo build $BUILD_TYPE

echo ""
echo "=== Preparing device for flash ==="

# Try to reset via USB command first
if python3 "$SCRIPT_DIR/pico_control.py" bootsel --no-wait 2>/dev/null; then
    echo "Sent BOOTSEL command, waiting for device..."
    sleep 2
fi

# Check if device is in BOOTSEL mode
if ! picotool info >/dev/null 2>&1; then
    echo "Device not in BOOTSEL mode."
    echo "Please hold BOOTSEL button and press reset..."

    # Wait for device
    for i in {1..30}; do
        if picotool info >/dev/null 2>&1; then
            echo "Device found!"
            break
        fi
        printf "."
        sleep 1
    done
    echo ""
fi

# Verify device is ready
if ! picotool info >/dev/null 2>&1; then
    echo "Error: Device not found in BOOTSEL mode after 30s"
    exit 1
fi

echo ""
echo "=== Flashing firmware ==="
cargo run $BUILD_TYPE

echo ""
echo "=== Done ==="
