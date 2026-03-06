#!/usr/bin/env python3
"""Configure RFD 900x radio baud rate via AT commands.

Usage:
    python3 scripts/rfd_config.py [port] [new_baud]

Defaults:
    port:     /dev/cu.usbserial-BG00KCTM
    new_baud: 115200

This script:
  1. Opens the serial port at the current baud rate (tries 57600, then 115200)
  2. Enters AT command mode (+++  with 1s guard times)
  3. Queries current settings
  4. Sets local serial speed to new_baud
  5. Sets remote serial speed to new_baud (if remote radio is reachable)
  6. Saves and reboots both radios
"""

import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-BG00KCTM"
NEW_BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

BAUD_CANDIDATES = [57600, 115200, 230400]

# SiK firmware uses abbreviated baud values (baud / 1000)
BAUD_TO_SIK = {
    1200: 1, 2400: 2, 4800: 4, 9600: 9, 19200: 19,
    38400: 38, 57600: 57, 115200: 115, 230400: 230,
}


def try_at_mode(ser):
    """Try to enter AT command mode. Returns True if successful."""
    ser.reset_input_buffer()
    time.sleep(1.1)  # guard time
    ser.write(b"+++")
    time.sleep(1.1)  # guard time
    # Read response — expect "OK\r\n"
    resp = ser.read(32).decode(errors="replace").strip()
    return "OK" in resp


def at_cmd(ser, cmd, timeout=3):
    """Send an AT command and return the response."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(0.1)
    # Read until timeout
    end = time.time() + timeout
    resp = b""
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            resp += chunk
            if b"\r\n" in resp:
                break
        else:
            time.sleep(0.05)
    return resp.decode(errors="replace").strip()


def main():
    # Try each candidate baud rate to find current radio setting
    ser = None
    current_baud = None

    for baud in BAUD_CANDIDATES:
        print(f"Trying {baud} baud on {PORT}...")
        try:
            s = serial.Serial(PORT, baud, timeout=2)
            s.dtr = False
            s.rts = False
            if try_at_mode(s):
                print(f"  -> Entered AT mode at {baud} baud")
                ser = s
                current_baud = baud
                break
            else:
                print(f"  -> No response at {baud}")
                s.close()
        except Exception as e:
            print(f"  -> Error: {e}")

    if ser is None:
        print("\nFailed to enter AT command mode at any baud rate.")
        print("Make sure the radio is connected and no other program has the port open.")
        sys.exit(1)

    # Query current settings
    print(f"\nCurrent settings (connected at {current_baud}):")
    resp = at_cmd(ser, "ATI")
    print(f"  ATI: {resp}")

    resp = at_cmd(ser, "ATI5")
    print(f"  ATI5 (all params):")
    for line in resp.split("\n"):
        line = line.strip()
        if line:
            print(f"    {line}")

    sik_val = BAUD_TO_SIK.get(NEW_BAUD)
    if sik_val is None:
        print(f"\nUnsupported baud rate: {NEW_BAUD}")
        print(f"Supported: {list(BAUD_TO_SIK.keys())}")
        at_cmd(ser, "ATO")
        ser.close()
        sys.exit(1)

    if current_baud == NEW_BAUD:
        print(f"\nRadio already at {NEW_BAUD} baud. No changes needed.")
        at_cmd(ser, "ATO")  # exit AT mode
        ser.close()
        return

    # Set local serial speed (SiK uses abbreviated value, e.g. 115 for 115200)
    print(f"\nSetting local serial speed to {NEW_BAUD} (SiK value: {sik_val})...")
    resp = at_cmd(ser, f"ATS1={sik_val}")
    print(f"  ATS1={sik_val}: {resp}")
    if "ERROR" in resp:
        print("  Failed to set local baud rate!")
        at_cmd(ser, "ATO")
        ser.close()
        sys.exit(1)

    # Save local settings
    resp = at_cmd(ser, "AT&W")
    print(f"  AT&W (save): {resp}")

    # Try to configure remote radio
    print(f"\nConfiguring remote radio...")
    resp = at_cmd(ser, f"RTS1={sik_val}", timeout=5)
    if "OK" in resp:
        print(f"  RTS1={sik_val}: {resp}")
        resp = at_cmd(ser, "RT&W", timeout=5)
        print(f"  RT&W (save remote): {resp}")
    else:
        print(f"  Remote radio not reachable: {resp}")
        print("  You'll need to configure the air-side radio separately.")

    # Reboot
    print("\nRebooting local radio...")
    at_cmd(ser, "ATZ", timeout=1)
    ser.close()

    print(f"\nDone! Local radio should now be at {NEW_BAUD} baud.")
    print(f"Update your connection string to use :{NEW_BAUD}")


if __name__ == "__main__":
    main()
