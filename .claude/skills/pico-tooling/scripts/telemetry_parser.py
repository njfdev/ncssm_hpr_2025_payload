#!/usr/bin/env python3
"""
Parse binary telemetry packets from Pico Logger.
Supports both real-time serial input and log file parsing.

Packet format (example - adjust to match your struct):
    timestamp_ms: u32 (4 bytes)
    pressure_pa:  u32 (4 bytes)
    temp_c:       i16 (2 bytes)
    accel_x:      i16 (2 bytes)
    accel_y:      i16 (2 bytes)
    accel_z:      i16 (2 bytes)
    gyro_x:       i16 (2 bytes)
    gyro_y:       i16 (2 bytes)
    gyro_z:       i16 (2 bytes)
    Total: 22 bytes

Usage:
    python telemetry_parser.py /dev/ttyACM0           # Real-time from serial
    python telemetry_parser.py --file telemetry.bin  # Parse binary file
    python telemetry_parser.py --csv output.csv      # Export to CSV
    python telemetry_parser.py --duration 10         # Run for 10 seconds
    python telemetry_parser.py --packets 100         # Read 100 packets then exit
"""

import argparse
import struct
import sys
import glob
import time
from dataclasses import dataclass
from typing import Iterator

try:
    import serial
    # Verify it's pyserial, not the conflicting 'serial' package
    if not hasattr(serial, 'Serial'):
        serial = None
except ImportError:
    serial = None


# Packet sync byte (optional, for framing)
SYNC_BYTE = 0xAA
PACKET_SIZE = 22  # Adjust to match your telemetry struct


@dataclass
class TelemetryPacket:
    timestamp_ms: int
    pressure_pa: int
    temp_c: int
    accel_x: int
    accel_y: int
    accel_z: int
    gyro_x: int
    gyro_y: int
    gyro_z: int

    @classmethod
    def from_bytes(cls, data: bytes) -> "TelemetryPacket":
        """Parse packet from raw bytes."""
        if len(data) < PACKET_SIZE:
            raise ValueError(f"Packet too short: {len(data)} < {PACKET_SIZE}")

        # Unpack little-endian (matches ARM default)
        # Adjust format string to match your struct!
        values = struct.unpack("<IIhhhhhhhh", data[:PACKET_SIZE])

        return cls(
            timestamp_ms=values[0],
            pressure_pa=values[1],
            temp_c=values[2],
            accel_x=values[3],
            accel_y=values[4],
            accel_z=values[5],
            gyro_x=values[6],
            gyro_y=values[7],
            gyro_z=values[8],
        )

    def altitude_m(self) -> float:
        """Calculate altitude from pressure using barometric formula."""
        sea_level_pa = 101325.0
        if self.pressure_pa <= 0:
            return 0.0
        return 44330.0 * (1.0 - (self.pressure_pa / sea_level_pa) ** 0.1903)

    def to_csv_row(self) -> str:
        """Convert to CSV row."""
        return (
            f"{self.timestamp_ms},{self.pressure_pa},{self.altitude_m():.2f},"
            f"{self.temp_c},{self.accel_x},{self.accel_y},{self.accel_z},"
            f"{self.gyro_x},{self.gyro_y},{self.gyro_z}"
        )

    @staticmethod
    def csv_header() -> str:
        return "timestamp_ms,pressure_pa,altitude_m,temp_c,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z"


def parse_file(filepath: str) -> Iterator[TelemetryPacket]:
    """Parse packets from binary file."""
    with open(filepath, "rb") as f:
        while True:
            data = f.read(PACKET_SIZE)
            if len(data) < PACKET_SIZE:
                break
            try:
                yield TelemetryPacket.from_bytes(data)
            except Exception as e:
                print(f"Parse error: {e}", file=sys.stderr)


def parse_serial(device: str, baud: int) -> Iterator[TelemetryPacket]:
    """Parse packets from serial port."""
    if serial is None:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    ser = serial.Serial(device, baud, timeout=1)
    buffer = bytearray()

    print(f"Reading from {device}...", file=sys.stderr)

    try:
        while True:
            data = ser.read(64)
            if data:
                buffer.extend(data)

                # Extract complete packets
                while len(buffer) >= PACKET_SIZE:
                    try:
                        packet = TelemetryPacket.from_bytes(bytes(buffer[:PACKET_SIZE]))
                        yield packet
                        buffer = buffer[PACKET_SIZE:]
                    except Exception:
                        # Skip byte and try again (resync)
                        buffer = buffer[1:]
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def find_device() -> str | None:
    """Auto-detect Pico serial device."""
    for pattern in ["/dev/ttyACM*", "/dev/tty.usbmodem*", "/dev/cu.usbmodem*"]:
        devices = glob.glob(pattern)
        if devices:
            return devices[0]
    return None


def main():
    parser = argparse.ArgumentParser(description="Parse Pico telemetry")
    parser.add_argument("device", nargs="?", help="Serial device path")
    parser.add_argument("--file", "-f", help="Binary file to parse")
    parser.add_argument("--csv", "-c", help="Output CSV file")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--duration", "-d", type=float, help="Run for N seconds then exit")
    parser.add_argument("--packets", "-n", type=int, help="Read N packets then exit")

    args = parser.parse_args()

    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "w")
        csv_file.write(TelemetryPacket.csv_header() + "\n")

    start_time = time.time()
    packet_count = 0

    try:
        if args.file:
            packets = parse_file(args.file)
        else:
            device = args.device or find_device()
            if not device:
                print("Error: No device found")
                sys.exit(1)
            packets = parse_serial(device, args.baud)

        for packet in packets:
            # Check duration limit
            if args.duration and (time.time() - start_time) >= args.duration:
                print(f"\nDuration limit reached ({args.duration}s)", file=sys.stderr)
                break

            # Check packet limit
            if args.packets and packet_count >= args.packets:
                print(f"\nPacket limit reached ({args.packets})", file=sys.stderr)
                break

            print(f"t={packet.timestamp_ms:8d}ms  "
                  f"P={packet.pressure_pa:6d}Pa  "
                  f"alt={packet.altitude_m():7.1f}m  "
                  f"T={packet.temp_c:4d}°C  "
                  f"acc=[{packet.accel_x:6d},{packet.accel_y:6d},{packet.accel_z:6d}]  "
                  f"gyr=[{packet.gyro_x:6d},{packet.gyro_y:6d},{packet.gyro_z:6d}]")

            packet_count += 1

            if csv_file:
                csv_file.write(packet.to_csv_row() + "\n")
                csv_file.flush()

    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
    finally:
        if csv_file:
            csv_file.close()
        print(f"Processed {packet_count} packets.", file=sys.stderr)


if __name__ == "__main__":
    main()
