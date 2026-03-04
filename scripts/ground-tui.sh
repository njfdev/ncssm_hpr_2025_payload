#!/usr/bin/env bash
# Launch the ground station TUI.
# Usage: ./scripts/ground-tui.sh [serial|udp] [DEVICE_OR_ADDR]
#
# Examples:
#   ./scripts/ground-tui.sh                          # FTDI radio (auto-detect)
#   ./scripts/ground-tui.sh serial /dev/cu.usbserial-BG00KCTM
#   ./scripts/ground-tui.sh udp                      # UDP loopback on :14550
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
MODE="${1:-serial}"
BAUD="${BAUD:-57600}"

case "$MODE" in
  serial)
    if [ -n "${2:-}" ]; then
      DEV="$2"
    else
      # Auto-detect FTDI device
      DEV=$(ls /dev/cu.usbserial-* 2>/dev/null | head -1)
      if [ -z "$DEV" ]; then
        echo "No FTDI serial device found. Plug in the radio or specify a device."
        echo "Usage: $0 serial /dev/cu.usbserial-XXXX"
        exit 1
      fi
      echo "Auto-detected: $DEV"
    fi
    ADDR="serial:${DEV}:${BAUD}"
    ;;
  udp)
    ADDR="${2:-udpin:0.0.0.0:14550}"
    ;;
  *)
    echo "Usage: $0 [serial|udp] [device_or_addr]"
    exit 1
    ;;
esac

echo "Connecting to $ADDR"
cd "$PROJECT_DIR/ground_tui"
cargo run --release -- --addr "$ADDR"
