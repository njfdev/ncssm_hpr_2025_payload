#!/bin/bash
# Display information about connected Pico device
# Usage: ./pico_info.sh

echo "=== Pico Device Information ==="
echo ""

# Check if picotool is installed
if ! command -v picotool &> /dev/null; then
    echo "Error: picotool not found. Install it first."
    echo "  macOS: brew install picotool"
    echo "  Linux: Build from https://github.com/raspberrypi/picotool"
    exit 1
fi

# Get device info
echo "Device Info:"
picotool info -a 2>/dev/null || echo "  No device found (or not in BOOTSEL mode)"

echo ""
echo "=== Serial Devices ==="

# List potential serial devices
if [[ "$(uname)" == "Darwin" ]]; then
    # macOS
    ls -la /dev/tty.usbmodem* /dev/cu.usbmodem* 2>/dev/null || echo "  No USB serial devices found"
else
    # Linux
    ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "  No USB serial devices found"
fi

echo ""
echo "=== USB Devices ==="

# List USB devices (look for Raspberry Pi or custom VID)
if command -v lsusb &> /dev/null; then
    lsusb | grep -i "raspberry\|2e8a\|1209" || echo "  No Pico USB devices found"
elif [[ "$(uname)" == "Darwin" ]]; then
    system_profiler SPUSBDataType 2>/dev/null | grep -A5 -i "pico\|raspberry" || echo "  No Pico USB devices found"
fi
