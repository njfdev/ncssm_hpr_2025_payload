#!/bin/bash
# Wait for RP2040/RP2350 device to enter BOOTSEL mode
# Usage: wait_bootsel.sh [timeout_seconds]

TIMEOUT=${1:-60}
echo "Waiting for device in BOOTSEL mode (timeout: ${TIMEOUT}s)..."
echo "Hold BOOTSEL button and press reset (or replug USB while holding BOOTSEL)"

for ((i=1; i<=TIMEOUT; i++)); do
    if picotool info 2>/dev/null | grep -q "RP2"; then
        echo ""
        echo "Device found in BOOTSEL mode!"
        picotool info 2>/dev/null | head -20
        exit 0
    fi
    printf "."
    sleep 1
done

echo ""
echo "Timeout: No device found in BOOTSEL mode after ${TIMEOUT}s"
exit 1
