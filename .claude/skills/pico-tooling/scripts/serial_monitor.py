#!/usr/bin/env python3
"""
Serial monitor for Pico USB CDC communication.
Reads data from the Pico and optionally logs to file.

Usage:
    python serial_monitor.py                        # Auto-detect device
    python serial_monitor.py /dev/ttyACM0          # Specify device
    python serial_monitor.py --log flight.log     # Log to file
    python serial_monitor.py --duration 10        # Run for 10 seconds
    python serial_monitor.py --lines 100          # Read 100 lines then exit
"""

import argparse
import glob
import sys
import time
from datetime import datetime

try:
    import serial
    # Verify it's pyserial, not the conflicting 'serial' package
    if not hasattr(serial, 'Serial'):
        raise ImportError("Wrong serial package")
except (ImportError, AttributeError):
    print("Error: pyserial not installed (or wrong 'serial' package installed).")
    print("Fix with: pip uninstall serial && pip install pyserial")
    sys.exit(1)


def find_pico_device():
    """Auto-detect Pico serial device."""
    patterns = [
        "/dev/ttyACM*",       # Linux
        "/dev/tty.usbmodem*", # macOS
        "/dev/cu.usbmodem*",  # macOS alternate
    ]

    for pattern in patterns:
        devices = glob.glob(pattern)
        if devices:
            return devices[0]

    return None


def monitor(
    device: str,
    baud: int,
    log_file: str | None,
    timestamp: bool,
    duration: float | None,
    max_lines: int | None,
):
    """Monitor serial port and optionally log data."""
    print(f"Connecting to {device} at {baud} baud...", file=sys.stderr)

    try:
        ser = serial.Serial(device, baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening {device}: {e}", file=sys.stderr)
        sys.exit(1)

    if duration:
        print(f"Running for {duration} seconds...", file=sys.stderr)
    elif max_lines:
        print(f"Reading {max_lines} lines...", file=sys.stderr)
    else:
        print("Connected. Press Ctrl+C to exit.", file=sys.stderr)

    log_handle = None
    if log_file:
        log_handle = open(log_file, "a")
        print(f"Logging to {log_file}", file=sys.stderr)

    print("", file=sys.stderr)

    start_time = time.time()
    line_count = 0

    try:
        while True:
            # Check duration limit
            if duration and (time.time() - start_time) >= duration:
                print(f"\nDuration limit reached ({duration}s)", file=sys.stderr)
                break

            # Check line limit
            if max_lines and line_count >= max_lines:
                print(f"\nLine limit reached ({max_lines})", file=sys.stderr)
                break

            if ser.in_waiting:
                try:
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                    if line:
                        if timestamp:
                            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            output = f"[{ts}] {line}"
                        else:
                            output = line

                        print(output)
                        line_count += 1

                        if log_handle:
                            log_handle.write(output + "\n")
                            log_handle.flush()
                except Exception as e:
                    print(f"Read error: {e}", file=sys.stderr)
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        ser.close()
        if log_handle:
            log_handle.close()
        print(f"Read {line_count} lines.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Serial monitor for Pico")
    parser.add_argument("device", nargs="?", help="Serial device path")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--log", "-l", help="Log file path")
    parser.add_argument("--timestamp", "-t", action="store_true", help="Add timestamps")
    parser.add_argument("--duration", "-d", type=float, help="Run for N seconds then exit")
    parser.add_argument("--lines", "-n", type=int, help="Read N lines then exit")

    args = parser.parse_args()

    device = args.device or find_pico_device()
    if not device:
        print("Error: No Pico device found. Specify device path manually.")
        sys.exit(1)

    monitor(device, args.baud, args.log, args.timestamp, args.duration, args.lines)


if __name__ == "__main__":
    main()
