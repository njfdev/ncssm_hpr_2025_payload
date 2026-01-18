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
#   download  - Download latest log file from SD card
#   scan      - Scan I2C bus for devices
#   calibrate - Run IMU calibration
#   bootsel   - Reboot to BOOTSEL mode
#   monitor   - Monitor serial output continuously
#   stream    - Start streaming and monitor output
#   recording - Start recording and monitor output

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
    download|dl)
        # Download latest log file from SD card
        port=$(find_port)
        outfile="${2:-}"

        python3 -c "
import serial
import time
import sys
import os

port = '$port'
outfile = '$outfile'

ser = serial.Serial(port, 115200, timeout=10)
time.sleep(0.3)

# Clear any buffered data and stop streaming
ser.reset_input_buffer()
ser.write(b'STOP\n')
time.sleep(0.3)
ser.read(ser.in_waiting or 4096)  # Drain buffer
ser.reset_input_buffer()

print('Requesting file download...', file=sys.stderr)
ser.write(b'DOWNLOAD\n')

# Read until we get the FILE: header
data = b''
start = time.time()
while time.time() - start < 10:
    chunk = ser.read(ser.in_waiting or 1)
    if chunk:
        data += chunk
        if b'FILE:' in data and b'\n' in data[data.find(b'FILE:'):]:
            break
    time.sleep(0.01)

# Check for errors
if b'ERROR:' in data:
    err_start = data.find(b'ERROR:')
    err_end = data.find(b'\n', err_start)
    print(f'Error: {data[err_start:err_end].decode()}', file=sys.stderr)
    ser.close()
    sys.exit(1)

if b'FILE:' not in data:
    print(f'No file header received: {data[:100]}', file=sys.stderr)
    ser.close()
    sys.exit(1)

# Parse header
file_start = data.find(b'FILE:')
header_end = data.find(b'\n', file_start)
header = data[file_start:header_end].decode().strip()
parts = header.split(',')
filename = parts[0].split(':')[1]
filesize = int(parts[1])

print(f'Downloading {filename} ({filesize} bytes)...', file=sys.stderr)

# Get data after header
file_data = data[header_end+1:]

# Continue reading file data
last_progress = 0
while len(file_data) < filesize + 20:
    chunk = ser.read(4096)
    if chunk:
        file_data += chunk
        # Progress indicator
        progress = len(file_data) * 100 // filesize
        if progress >= last_progress + 10:
            print(f'  {min(progress, 100)}%...', file=sys.stderr)
            last_progress = progress
    else:
        print(f'Timeout at {len(file_data)}/{filesize} bytes', file=sys.stderr)
        break

    if b'EOF' in file_data or b'ERROR' in file_data:
        break

ser.close()

# Check result
if b'ERROR' in file_data:
    err_idx = file_data.find(b'ERROR')
    print(f'Transfer error: {file_data[err_idx:err_idx+50].decode()}', file=sys.stderr)
    sys.exit(1)

if b'EOF' not in file_data:
    print(f'Incomplete transfer: {len(file_data)}/{filesize} bytes', file=sys.stderr)
    sys.exit(1)

# Extract actual content (before EOF marker)
eof_idx = file_data.find(b'\r\nEOF')
if eof_idx == -1:
    eof_idx = file_data.find(b'EOF')
content = file_data[:eof_idx] if eof_idx > 0 else file_data

# Save or output
if outfile:
    with open(outfile, 'wb') as f:
        f.write(content)
    print(f'Saved to {outfile} ({len(content)} bytes)', file=sys.stderr)
else:
    # Auto-generate filename
    outfile = filename
    with open(outfile, 'wb') as f:
        f.write(content)
    print(f'Saved to {outfile} ({len(content)} bytes)', file=sys.stderr)
"
        ;;
    scan)
        send_cmd "SCAN" 5
        ;;
    calibrate|cal)
        send_cmd "CALIBRATE" 5
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
        echo "  download    Download latest log file from SD card"
        echo "  scan        Scan I2C bus for devices"
        echo "  calibrate   Run IMU calibration"
        echo "  bootsel     Reboot to BOOTSEL mode"
        echo "  monitor     Monitor serial output (Ctrl+C to stop)"
        echo "  stream      Start streaming and monitor"
        echo "  recording   Start recording and monitor"
        echo ""
        echo "Examples:"
        echo "  ./pico.sh ping"
        echo "  ./pico.sh sensors"
        echo "  ./pico.sh stream       # Start streaming, Ctrl+C sends STOP"
        echo "  ./pico.sh recording    # Record to SD + stream, Ctrl+C sends STOP"
        echo "  ./pico.sh download     # Download latest log to FLTxxx.CSV"
        echo "  ./pico.sh download out.csv  # Download to specific file"
        ;;
    *)
        # Send arbitrary command
        send_cmd "$1"
        ;;
esac
