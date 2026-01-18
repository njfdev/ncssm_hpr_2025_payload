#!/bin/bash
# Pico Logger Command Script
# Usage: ./pico.sh <command> [options]
#
# Commands:
#   ping      - Test connection (returns PONG)
#   sensors   - Check sensor status
#   sdstatus  - Check SD card status
#   read      - Read sensors once
#   start     - Start streaming (USB only)
#   record    - Start streaming with SD card logging
#   stop      - Stop streaming/recording
#   log       - Create new log file on SD card
#   stoplog   - Stop SD card logging
#   bootsel   - Reboot to BOOTSEL mode
#   monitor   - Monitor serial output continuously
#   stream    - Start streaming and monitor output

set -e

# Find the serial port
find_port() {
    local port=$(ls /dev/tty.usbmodem* 2>/dev/null | head -1)
    if [ -z "$port" ]; then
        echo "Error: No Pico found. Is it connected?" >&2
        exit 1
    fi
    echo "$port"
}

# Send a command and read response
send_cmd() {
    local port=$(find_port)
    local cmd="$1"
    local timeout="${2:-1}"

    # Use Python for reliable serial communication
    python3 -c "
import serial
import time
import sys

try:
    ser = serial.Serial('$port', 115200, timeout=$timeout)
    time.sleep(0.3)  # Wait for connection
    ser.reset_input_buffer()
    ser.write(b'$cmd\n')
    time.sleep(0.2)
    response = ser.read(ser.in_waiting or 256).decode('utf-8', errors='replace')
    print(response.strip())
    ser.close()
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
    sys.exit(1)
"
}

# Monitor serial output continuously
monitor() {
    local port=$(find_port)
    local duration="${1:-0}"  # 0 = infinite

    echo "Monitoring $port (Ctrl+C to stop)..."
    python3 -c "
import serial
import time
import sys

try:
    ser = serial.Serial('$port', 115200, timeout=0.5)
    start = time.time()
    duration = $duration
    while True:
        if duration > 0 and (time.time() - start) > duration:
            break
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(line)
except KeyboardInterrupt:
    pass
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
finally:
    ser.close()
"
}

# Start streaming and monitor
stream_monitor() {
    local port=$(find_port)
    local cmd="${1:-START}"

    echo "Starting $cmd and monitoring (Ctrl+C to stop)..."
    python3 -c "
import serial
import time
import sys

try:
    ser = serial.Serial('$port', 115200, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b'$cmd\n')

    while True:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(line)
except KeyboardInterrupt:
    print('\nSending STOP...')
    ser.write(b'STOP\n')
    time.sleep(0.3)
    response = ser.read(ser.in_waiting or 256).decode('utf-8', errors='replace')
    print(response.strip())
except Exception as e:
    print(f'Error: {e}', file=sys.stderr)
finally:
    ser.close()
"
}

# Main command handling
case "${1:-help}" in
    ping)
        send_cmd "PING"
        ;;
    sensors)
        send_cmd "SENSORS"
        ;;
    sdstatus|sd)
        send_cmd "SDSTATUS"
        ;;
    read)
        send_cmd "READ"
        ;;
    start)
        send_cmd "START"
        ;;
    record|rec)
        send_cmd "RECORD"
        ;;
    stop)
        send_cmd "STOP"
        ;;
    log)
        send_cmd "LOG"
        ;;
    stoplog)
        send_cmd "STOPLOG"
        ;;
    bootsel|boot)
        send_cmd "BOOTSEL"
        echo "Device should now be in BOOTSEL mode"
        ;;
    monitor|mon)
        monitor "${2:-0}"
        ;;
    stream)
        stream_monitor "START"
        ;;
    recording)
        stream_monitor "RECORD"
        ;;
    help|--help|-h|"")
        echo "Pico Logger Command Script"
        echo ""
        echo "Usage: ./pico.sh <command>"
        echo ""
        echo "Commands:"
        echo "  ping        Test connection (returns PONG)"
        echo "  sensors     Check sensor status"
        echo "  sdstatus    Check SD card status"
        echo "  read        Read sensors once"
        echo "  start       Start USB streaming"
        echo "  record      Start streaming + SD logging"
        echo "  stop        Stop streaming/recording"
        echo "  log         Create new log file on SD"
        echo "  stoplog     Stop SD card logging"
        echo "  bootsel     Reboot to BOOTSEL mode"
        echo "  monitor     Monitor serial output (Ctrl+C to stop)"
        echo "  stream      Start streaming and monitor"
        echo "  recording   Start recording and monitor"
        echo ""
        echo "Examples:"
        echo "  ./pico.sh ping"
        echo "  ./pico.sh sensors"
        echo "  ./pico.sh stream      # Start streaming, Ctrl+C sends STOP"
        echo "  ./pico.sh recording   # Record to SD + stream, Ctrl+C sends STOP"
        ;;
    *)
        # Send arbitrary command
        send_cmd "$1"
        ;;
esac
