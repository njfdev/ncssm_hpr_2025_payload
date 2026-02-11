#!/usr/bin/env python3
"""
Interactive serial interface for the Ground Station RFD 900x bridge.

Usage:
    python ground_station.py              # Auto-detect device
    python ground_station.py /dev/cu.usbmodem0021  # Specify device

Commands (in COMMAND mode):
    PING          - Check connection (responds PONG)
    STATUS        - Show current mode and UART config
    HELP          - List available commands
    SEND <data>   - Send data through the radio to the paired device
    PASSTHROUGH   - Enter transparent USB <-> UART bridge mode
    ATMODE        - Enter RFD 900x AT configuration mode
    BOOTSEL       - Reboot into BOOTSEL (firmware update) mode

In PASSTHROUGH mode:
    All input is forwarded to the radio and all radio output is displayed.
    Press Ctrl-A three times (0x01 0x01 0x01) to exit back to COMMAND mode.

In ATMODE:
    Type AT commands (e.g. ATI, ATS1?, ATS1=57) to configure the radio.
    Type EXITAT or ATO to return to COMMAND mode.
"""

import argparse
import glob
import os
import select
import sys
import termios
import time
import tty

try:
    import serial
    if not hasattr(serial, "Serial"):
        raise ImportError("Wrong serial package")
except (ImportError, AttributeError):
    print("Error: pyserial not installed.")
    print("Fix with: pip install pyserial")
    sys.exit(1)


def find_device():
    """Auto-detect the ground station USB serial device."""
    patterns = [
        "/dev/cu.usbmodem*",
        "/dev/ttyACM*",
    ]
    for pattern in patterns:
        devices = glob.glob(pattern)
        if devices:
            return devices[0]
    return None


def read_response(ser, timeout=1.0):
    """Read all available response data, waiting up to timeout seconds.

    Waits for the first byte to arrive, then keeps reading until
    no more data arrives for 50ms (end of message).
    """
    deadline = time.time() + timeout
    result = b""

    # Wait for first byte
    while time.time() < deadline:
        if ser.in_waiting > 0:
            result += ser.read(ser.in_waiting)
            break
        time.sleep(0.01)
    else:
        return ""

    # Keep reading until 50ms silence (message complete)
    while True:
        time.sleep(0.05)
        if ser.in_waiting > 0:
            result += ser.read(ser.in_waiting)
        else:
            break

    return result.decode("utf-8", errors="replace")


def interactive_mode(port: str):
    """Run an interactive serial terminal."""
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)

    time.sleep(1.5)
    ser.reset_input_buffer()

    print(f"Ground Station connected on {port}")
    print("Type commands and press Enter. Ctrl-C to quit.")
    print("─" * 50)

    # Send initial PING to verify connection
    ser.write(b"PING")
    ser.flush()
    resp = read_response(ser)
    if resp:
        print(f"< {resp.strip()}")
    else:
        print("Warning: no response to PING")
    print()

    old_settings = termios.tcgetattr(sys.stdin)

    try:
        line_mode(ser, old_settings)
    except KeyboardInterrupt:
        print("\nDisconnected.")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ser.close()


def line_mode(ser: serial.Serial, old_settings):
    """Line-buffered command mode - type commands and press Enter."""
    while True:
        # Restore cooked mode for line editing
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Print any data that arrived while waiting for input
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
            sys.stdout.write(data)
            sys.stdout.flush()

        try:
            cmd = input("> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        if cmd.lower() in ("quit", "exit", "q"):
            break

        # Send command
        ser.write(cmd.encode("utf-8"))
        ser.flush()

        # Special handling for mode-switching commands
        if cmd.upper() == "PASSTHROUGH":
            resp = read_response(ser)
            if resp:
                sys.stdout.write(resp)
                sys.stdout.flush()
            raw_mode(ser, old_settings)
            continue

        if cmd.upper() == "ATMODE":
            # Read the "ENTERING AT MODE..." response
            resp = read_response(ser, timeout=1)
            if resp:
                sys.stdout.write(resp)
                sys.stdout.flush()
            # Wait for guard times (1s + 1s) + "AT MODE READY"
            resp = read_response(ser, timeout=3)
            if resp:
                sys.stdout.write(resp)
                sys.stdout.flush()
            at_mode(ser, old_settings)
            continue

        if cmd.upper() == "BOOTSEL":
            print("Device entering BOOTSEL mode...")
            time.sleep(0.5)
            break

        # Read response
        resp = read_response(ser)
        if resp:
            sys.stdout.write(f"< {resp}")
            if not resp.endswith("\n"):
                sys.stdout.write("\n")
            sys.stdout.flush()


def raw_mode(ser: serial.Serial, old_settings):
    """Raw passthrough mode - all keystrokes forwarded to radio."""
    print("[PASSTHROUGH - press Ctrl-A 3x to exit]")
    tty.setraw(sys.stdin)
    stdin_fd = sys.stdin.fileno()

    try:
        while True:
            readable, _, _ = select.select([stdin_fd, ser], [], [], 0.05)

            for source in readable:
                if source == stdin_fd:
                    ch = os.read(stdin_fd, 1)
                    ser.write(ch)
                    ser.flush()
                elif source == ser:
                    waiting = ser.in_waiting
                    if waiting > 0:
                        data = ser.read(waiting)
                        if b"EXITING PASSTHROUGH" in data:
                            termios.tcsetattr(
                                sys.stdin, termios.TCSADRAIN, old_settings
                            )
                            sys.stdout.write(
                                data.decode("utf-8", errors="replace")
                            )
                            sys.stdout.flush()
                            print("[Back to COMMAND mode]")
                            return
                        os.write(sys.stdout.fileno(), data)
    except Exception:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n[Back to COMMAND mode]")


def at_mode(ser: serial.Serial, old_settings):
    """AT command mode - line-buffered, sends to radio for configuration."""
    print("[AT MODE - type AT commands, EXITAT to exit]")

    while True:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Print any data that arrived
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
            sys.stdout.write(data)
            sys.stdout.flush()

        try:
            cmd = input("AT> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        ser.write(cmd.encode("utf-8"))
        ser.flush()

        if cmd.upper() in ("EXITAT", "ATO"):
            resp = read_response(ser)
            if resp:
                sys.stdout.write(resp)
                sys.stdout.flush()
            print("[Back to COMMAND mode]")
            return

        # Wait for AT response
        resp = read_response(ser)
        if resp:
            sys.stdout.write(resp)
            if not resp.endswith("\n"):
                sys.stdout.write("\n")
            sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(
        description="Ground Station RFD 900x serial interface"
    )
    parser.add_argument(
        "device",
        nargs="?",
        help="Serial device path (auto-detected if omitted)",
    )
    args = parser.parse_args()

    device = args.device or find_device()
    if not device:
        print("No ground station device found.")
        print("Connect the Feather RP2040 via USB.")
        sys.exit(1)

    interactive_mode(device)


if __name__ == "__main__":
    main()
