#!/usr/bin/env python3
"""Listen for MAVLink messages on a serial port and print them."""
import sys
import time
from pymavlink import mavutil

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem0021"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 57600

print(f"Listening on {port} at {baud} baud...")
conn = mavutil.mavlink_connection(port, baud=baud)

while True:
    msg = conn.recv_match(blocking=True, timeout=5)
    if msg is None:
        print("  (no message received in 5s)")
        continue
    msg_type = msg.get_type()
    if msg_type == "HEARTBEAT":
        print(f"[HEARTBEAT] type={msg.type} autopilot={msg.autopilot} status={msg.system_status}")
    elif msg_type == "GLOBAL_POSITION_INT":
        alt_m = msg.alt / 1000.0
        rel_alt_m = msg.relative_alt / 1000.0
        print(f"[POSITION]  lat={msg.lat/1e7:.4f} lon={msg.lon/1e7:.4f} alt={alt_m:.0f}m rel_alt={rel_alt_m:.0f}m t={msg.time_boot_ms}ms")
    else:
        print(f"[{msg_type}] {msg}")
