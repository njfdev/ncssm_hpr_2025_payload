#!/usr/bin/env python3
"""
Control interface for Pico Logger via USB serial commands.
Send commands to control device state, trigger actions, and query status.

Usage:
    python pico_control.py reboot              # Reboot to application
    python pico_control.py bootsel             # Reboot to BOOTSEL mode
    python pico_control.py status              # Query device status
    python pico_control.py start-logging       # Start SD card logging
    python pico_control.py stop-logging        # Stop SD card logging
    python pico_control.py send "CUSTOM_CMD"   # Send custom command

Commands are sent as newline-terminated strings. The firmware must implement
a command handler to process these. See the SKILL.md for Embassy integration.
"""

import argparse
import glob
import sys
import time

try:
    import serial
    # Verify it's pyserial, not the conflicting 'serial' package
    if not hasattr(serial, 'Serial'):
        raise ImportError("Wrong serial package")
except (ImportError, AttributeError):
    print("Error: pyserial not installed (or wrong 'serial' package installed).")
    print("Fix with: pip uninstall serial && pip install pyserial")
    sys.exit(1)


# Command protocol - simple text commands recognized by firmware
# Firmware responds with appropriate output or enters requested state
COMMANDS = {
    "reboot": "REBOOT",
    "bootsel": "BOOTSEL",
    "status": "STATUS",
    "start-logging": "LOG_START",
    "stop-logging": "LOG_STOP",
    "ping": "PING",
    "version": "VERSION",
    "sensors": "SENSORS",
    "calibrate": "CALIBRATE",
}


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


def send_command(
    device: str,
    command: str,
    baud: int = 115200,
    timeout: float = 2.0,
    wait_response: bool = True,
) -> str | None:
    """Send a command and optionally wait for response."""
    try:
        ser = serial.Serial(device, baud, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error opening {device}: {e}", file=sys.stderr)
        return None

    try:
        # Clear any pending input
        ser.reset_input_buffer()

        # Send command with newline terminator
        cmd_bytes = (command + "\n").encode("utf-8")
        ser.write(cmd_bytes)
        ser.flush()

        if not wait_response:
            return "SENT"

        # Wait for response
        response_lines = []
        start = time.time()

        while (time.time() - start) < timeout:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    response_lines.append(line)
                    # Check for terminal responses
                    if line.startswith("OK") or line.startswith("ERR:"):
                        break
            else:
                time.sleep(0.01)

        return "\n".join(response_lines) if response_lines else None

    finally:
        ser.close()


def interactive_mode(device: str, baud: int):
    """Interactive command mode."""
    print(f"Interactive mode on {device}")
    print("Type commands, 'help' for list, 'quit' to exit\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if not cmd:
            continue

        if cmd == "quit" or cmd == "exit":
            break

        if cmd == "help":
            print("Available commands:")
            for name, raw in COMMANDS.items():
                print(f"  {name:20s} -> {raw}")
            print(f"  {'send <text>':20s} -> Send custom command")
            print(f"  {'quit':20s} -> Exit interactive mode")
            continue

        # Resolve command alias or use raw
        if cmd in COMMANDS:
            raw_cmd = COMMANDS[cmd]
        elif cmd.startswith("send "):
            raw_cmd = cmd[5:]
        else:
            raw_cmd = cmd

        response = send_command(device, raw_cmd, baud)
        if response:
            print(response)
        else:
            print("(no response)")


def main():
    parser = argparse.ArgumentParser(
        description="Control Pico Logger via USB serial",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available commands:
  reboot          Reboot device to application
  bootsel         Reboot to BOOTSEL mode for flashing
  status          Query device status
  start-logging   Start SD card logging
  stop-logging    Stop SD card logging
  ping            Check if device is responsive
  version         Get firmware version
  sensors         Get current sensor readings
  calibrate       Trigger sensor calibration
  send <text>     Send custom command
  interactive     Enter interactive command mode
""",
    )
    parser.add_argument("command", nargs="?", help="Command to send")
    parser.add_argument("args", nargs="*", help="Command arguments")
    parser.add_argument("--device", "-D", help="Serial device path")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--timeout", "-t", type=float, default=2.0, help="Response timeout")
    parser.add_argument("--no-wait", action="store_true", help="Don't wait for response")

    args = parser.parse_args()

    device = args.device or find_pico_device()
    if not device:
        print("Error: No Pico device found. Specify with --device")
        sys.exit(1)

    if not args.command:
        parser.print_help()
        sys.exit(1)

    # Handle special modes
    if args.command == "interactive":
        interactive_mode(device, args.baud)
        return

    # Resolve command
    if args.command == "send":
        if not args.args:
            print("Error: 'send' requires command text")
            sys.exit(1)
        raw_cmd = " ".join(args.args)
    elif args.command in COMMANDS:
        raw_cmd = COMMANDS[args.command]
    else:
        print(f"Unknown command: {args.command}")
        print("Use 'send <text>' for custom commands")
        sys.exit(1)

    # Send and print response
    print(f"Sending: {raw_cmd}", file=sys.stderr)
    response = send_command(
        device,
        raw_cmd,
        args.baud,
        args.timeout,
        wait_response=not args.no_wait,
    )

    if response:
        print(response)
    else:
        print("(no response)", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
