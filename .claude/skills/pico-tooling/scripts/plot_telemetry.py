#!/usr/bin/env python3
"""
Real-time telemetry plotting for Pico Logger.
Displays altitude, acceleration, and gyroscope data.

Usage:
    python plot_telemetry.py /dev/ttyACM0      # Plot from serial
    python plot_telemetry.py --file data.csv   # Plot from CSV file

Requires: pip install matplotlib pyserial
"""

import argparse
import sys
import glob
from collections import deque
from typing import Callable

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError:
    print("Error: matplotlib not installed. Run: pip install matplotlib")
    sys.exit(1)

try:
    import serial
    # Verify it's pyserial, not the conflicting 'serial' package
    if not hasattr(serial, 'Serial'):
        serial = None
except ImportError:
    serial = None


# Data buffers (circular)
MAX_POINTS = 500
timestamps = deque(maxlen=MAX_POINTS)
altitudes = deque(maxlen=MAX_POINTS)
accel_x = deque(maxlen=MAX_POINTS)
accel_y = deque(maxlen=MAX_POINTS)
accel_z = deque(maxlen=MAX_POINTS)
gyro_x = deque(maxlen=MAX_POINTS)
gyro_y = deque(maxlen=MAX_POINTS)
gyro_z = deque(maxlen=MAX_POINTS)


def parse_csv_line(line: str) -> dict | None:
    """Parse a CSV line into telemetry data."""
    try:
        parts = line.strip().split(",")
        if len(parts) < 10 or parts[0] == "timestamp_ms":
            return None
        return {
            "timestamp_ms": int(parts[0]),
            "pressure_pa": int(parts[1]),
            "altitude_m": float(parts[2]),
            "temp_c": int(parts[3]),
            "accel_x": int(parts[4]),
            "accel_y": int(parts[5]),
            "accel_z": int(parts[6]),
            "gyro_x": int(parts[7]),
            "gyro_y": int(parts[8]),
            "gyro_z": int(parts[9]),
        }
    except (ValueError, IndexError):
        return None


def create_serial_reader(device: str, baud: int) -> Callable:
    """Create a data reader from serial port."""
    if serial is None:
        print("Error: pyserial not installed")
        sys.exit(1)

    ser = serial.Serial(device, baud, timeout=0.1)

    def read_data():
        if ser.in_waiting:
            try:
                line = ser.readline().decode("utf-8", errors="replace")
                return parse_csv_line(line)
            except Exception:
                pass
        return None

    return read_data


def create_file_reader(filepath: str) -> Callable:
    """Create a data reader from CSV file."""
    with open(filepath, "r") as f:
        lines = f.readlines()

    index = [0]  # Mutable container for closure

    def read_data():
        if index[0] < len(lines):
            data = parse_csv_line(lines[index[0]])
            index[0] += 1
            return data
        return None

    return read_data


def update_plot(frame, read_data, lines, axes):
    """Animation update function."""
    # Read multiple data points per frame for smoother updates
    for _ in range(10):
        data = read_data()
        if data:
            timestamps.append(data["timestamp_ms"] / 1000.0)  # Convert to seconds
            altitudes.append(data["altitude_m"])
            accel_x.append(data["accel_x"])
            accel_y.append(data["accel_y"])
            accel_z.append(data["accel_z"])
            gyro_x.append(data["gyro_x"])
            gyro_y.append(data["gyro_y"])
            gyro_z.append(data["gyro_z"])

    if len(timestamps) > 1:
        t = list(timestamps)

        # Update altitude plot
        lines[0].set_data(t, list(altitudes))
        axes[0].relim()
        axes[0].autoscale_view()

        # Update accelerometer plot
        lines[1].set_data(t, list(accel_x))
        lines[2].set_data(t, list(accel_y))
        lines[3].set_data(t, list(accel_z))
        axes[1].relim()
        axes[1].autoscale_view()

        # Update gyroscope plot
        lines[4].set_data(t, list(gyro_x))
        lines[5].set_data(t, list(gyro_y))
        lines[6].set_data(t, list(gyro_z))
        axes[2].relim()
        axes[2].autoscale_view()

    return lines


def find_device() -> str | None:
    """Auto-detect Pico serial device."""
    for pattern in ["/dev/ttyACM*", "/dev/tty.usbmodem*", "/dev/cu.usbmodem*"]:
        devices = glob.glob(pattern)
        if devices:
            return devices[0]
    return None


def main():
    parser = argparse.ArgumentParser(description="Plot Pico telemetry")
    parser.add_argument("device", nargs="?", help="Serial device path")
    parser.add_argument("--file", "-f", help="CSV file to plot")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")

    args = parser.parse_args()

    # Create data reader
    if args.file:
        read_data = create_file_reader(args.file)
        title_suffix = f" - {args.file}"
    else:
        device = args.device or find_device()
        if not device:
            print("Error: No device found")
            sys.exit(1)
        read_data = create_serial_reader(device, args.baud)
        title_suffix = f" - {device}"

    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"Pico Logger Telemetry{title_suffix}")

    # Altitude plot
    axes[0].set_ylabel("Altitude (m)")
    axes[0].set_title("Altitude")
    axes[0].grid(True)
    line_alt, = axes[0].plot([], [], "b-", label="Altitude")

    # Accelerometer plot
    axes[1].set_ylabel("Acceleration (raw)")
    axes[1].set_title("Accelerometer")
    axes[1].grid(True)
    line_ax, = axes[1].plot([], [], "r-", label="X")
    line_ay, = axes[1].plot([], [], "g-", label="Y")
    line_az, = axes[1].plot([], [], "b-", label="Z")
    axes[1].legend(loc="upper right")

    # Gyroscope plot
    axes[2].set_xlabel("Time (s)")
    axes[2].set_ylabel("Angular rate (raw)")
    axes[2].set_title("Gyroscope")
    axes[2].grid(True)
    line_gx, = axes[2].plot([], [], "r-", label="X")
    line_gy, = axes[2].plot([], [], "g-", label="Y")
    line_gz, = axes[2].plot([], [], "b-", label="Z")
    axes[2].legend(loc="upper right")

    lines = [line_alt, line_ax, line_ay, line_az, line_gx, line_gy, line_gz]

    # Animation
    ani = animation.FuncAnimation(
        fig, update_plot,
        fargs=(read_data, lines, axes),
        interval=50,  # 20 FPS
        blit=False,
        cache_frame_data=False,
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
