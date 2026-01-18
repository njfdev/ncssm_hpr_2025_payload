# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NCSSM Rocketry 2025 Payload - A multi-component Rust embedded systems project for a high-altitude rocket. The system consists of three packages that work together: a flight computer for high-level processing and camera capture, a Pico-based data logger for sensor telemetry, and a ground station for mission control.

## Build Commands

### Flight Computer (Linux host, camera/video processing)
```bash
cd flight_computer
cargo build --release
cargo run --release
```
**Prerequisites**: Requires `ffmpeg-rk` (not regular ffmpeg) for hardware encoding and PipeWire audio system.

### Pico Logger (RP2040 embedded firmware)
```bash
cd pico_logger
cargo build --release
cargo run --release  # Uses picotool to flash (device must be in BOOTSEL mode)
```
**Prerequisites**: Requires `picotool` installed for flashing. Target is `thumbv6m-none-eabi` (ARMv6-M Cortex-M0+).

### Ground Station
```bash
cd ground_station
cargo build --release
cargo run --release
```

## Architecture

### Three-Package Structure
- **flight_computer/**: High-level processing unit running on a Linux SBC. Handles camera capture using `nokhwa` with callback-based frame processing. Uses camera index 3.
- **pico_logger/**: Bare-metal `#![no_std]` firmware for RP2040 (Adafruit Feather RP2040). Uses Embassy async runtime for concurrent sensor reading. Planned sensors: BMP390 (barometer), ENS160 (gas), LSM6DSOX+LIS3MDL (IMU), HGLRC Mini M100 (GPS), SPI SD card logging.
- **ground_station/**: Telemetry receiver and mission control interface (stub implementation).

### Embedded Framework
The pico_logger uses the Embassy async embedded framework:
- `embassy-executor` for async task scheduling
- `embassy-rp` for RP2040 peripheral access (feature: `rp2040`)
- Memory layout: 2MB flash at 0x10000000, 256K main RAM
- Binary info entries for picotool identification
- USB CDC serial for communication

### Inter-Component Communication
Flight computer and pico_logger are intended to communicate sensor data between boards (protocol TBD).

## Hardware Configuration

- **Flight Computer**: Linux SBC with camera on index 3
- **Pico Logger**: Adafruit Feather RP2040 with I2C sensors and SPI SD card
- **Calibration**: DaytonAudio mic calibration data in `flight_computer/src/2581417.txt`

## Skill Maintenance Guidelines

Claude Code skills are located in `.claude/skills/`. Follow these practices:

### Session Reflection (REQUIRED)
At the end of each working session, **you MUST update the relevant skills** with learnings. Reflect on:
1. What approaches were tried and which succeeded
2. What information would have allowed solving problems faster on the first try
3. **Update the relevant skill files with this critical context** - don't just think about it, actually edit the files

This is not optional. Skills are the project's institutional memory. Future sessions will waste time rediscovering solutions if learnings aren't captured.

### Updating Skills
- **Always update skills, not CLAUDE.md** - Skills contain domain-specific knowledge; CLAUDE.md is for project-wide overview only
- **Split large skills into sub-files** - When a skill grows beyond ~200 lines, split into logical sub-files (e.g., `SKILL.md`, `setup.md`, `troubleshooting.md`, `examples.md`)
- **Include concrete examples** - Code snippets, commands, and configurations that work
- **Document gotchas prominently** - Hardware variants, version mismatches, common errors

### Skill Structure
```
.claude/skills/<skill-name>/
├── SKILL.md           # Main entry point with overview and triggers
├── setup.md           # Installation and configuration (optional)
├── troubleshooting.md # Common issues and solutions (optional)
├── examples/          # Working code examples (optional)
└── scripts/           # Helper scripts for automation
```

### Critical Information to Capture
- Hardware variants (RP2040 vs RP2350, different boards)
- Cargo feature flags and their effects
- Target triples and toolchain requirements
- Memory layouts and linker script differences
- Version compatibility between crates

## Script Usage

**Always use existing scripts** in `.claude/skills/*/scripts/` rather than running commands directly. Scripts provide:
- Consistent behavior and error handling
- Duration limits to prevent blocking
- Auto-detection of devices and ports
- Proper cleanup

### Key Scripts

| Script | Location | Purpose |
|--------|----------|---------|
| `build_flash.sh` | pico-tooling/scripts/ | Build and flash pico_logger (auto-resets to BOOTSEL) |
| `pico_control.py` | pico-tooling/scripts/ | Send commands to Pico (bootsel, ping, etc.) |
| `serial_monitor.py` | pico-tooling/scripts/ | Read serial output with duration limit |
| `wait_bootsel.sh` | pico-tooling/scripts/ | Wait for device to enter BOOTSEL mode |

Example workflow:
```bash
# Build and flash (handles BOOTSEL automatically)
.claude/skills/pico-tooling/scripts/build_flash.sh

# Monitor output for 10 seconds
python3 .claude/skills/pico-tooling/scripts/serial_monitor.py --duration 10

# Reset to BOOTSEL via USB command
python3 .claude/skills/pico-tooling/scripts/pico_control.py bootsel
```

## User Notifications

**When asking the user a question**, always play an audible sound so they can be alerted from another app:
```bash
afplay /System/Library/Sounds/Ping.aiff
```

## Code Structure Guidelines

**Keep modules small and focused**. For embedded projects like pico_logger:
- Split large files into logical modules (sensors, usb, telemetry, etc.)
- Each sensor driver should be in its own file under `src/sensors/`
- USB/communication code in `src/usb.rs` or `src/comms/`
- Main should only contain initialization and the main loop
- Use `mod.rs` files to re-export public interfaces

Example structure for pico_logger:
```
pico_logger/src/
├── main.rs           # Entry point, initialization, main loop
├── sensors/
│   ├── mod.rs        # Re-exports sensor modules
│   ├── bmp390.rs     # Barometer driver
│   ├── lsm6dsox.rs   # Accelerometer/gyro driver
│   ├── lis3mdl.rs    # Magnetometer driver
│   └── types.rs      # Shared sensor data types
├── usb.rs            # USB CDC setup and handling
└── telemetry.rs      # Data formatting and transmission
```

## Context Management

**Compact context at good stopping points** - When reaching natural stopping points (feature complete, tests passing, ready for review), proactively compact the conversation to preserve context window for future work. Do not wait for automatic compaction which may occur at inconvenient times.

Good times to compact:
- After completing a feature or fix
- After updating documentation/skills
- Before starting a new unrelated task
- When context is getting long but work is at a stable point
