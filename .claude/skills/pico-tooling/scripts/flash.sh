#!/bin/bash
# Flash Pico Logger firmware using picotool
# Usage: ./flash.sh [--release]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../../pico_logger" && pwd)"

BUILD_TYPE="debug"
if [[ "$1" == "--release" || "$1" == "-r" ]]; then
    BUILD_TYPE="release"
fi

TARGET="thumbv8m.main-none-eabihf"
BINARY="$PROJECT_ROOT/target/$TARGET/$BUILD_TYPE/pico_logger"

echo "=== Pico Logger Flash Script ==="
echo "Project: $PROJECT_ROOT"
echo "Build type: $BUILD_TYPE"
echo ""

# Build
echo "Building..."
cd "$PROJECT_ROOT"
if [[ "$BUILD_TYPE" == "release" ]]; then
    cargo build --release
else
    cargo build
fi

# Check if binary exists
if [[ ! -f "$BINARY" ]]; then
    echo "Error: Binary not found at $BINARY"
    exit 1
fi

# Flash
echo ""
echo "Flashing..."
echo "  If the Pico is not in BOOTSEL mode, hold BOOTSEL and reconnect USB."
echo ""

picotool load -u -v -x -t elf "$BINARY"

echo ""
echo "Done!"
