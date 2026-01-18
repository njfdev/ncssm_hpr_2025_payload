---
name: pico-logger
description: Pico Logger subproject context. Use when working in the pico_logger package, RP2040 firmware, sensor data collection, SD card logging, or embedded telemetry.
---

# Pico Logger Subproject

## Hardware

**Board**: Adafruit Feather RP2040
**Chip**: RP2040 (Cortex-M0+, ARMv6-M)
**Target**: `thumbv6m-none-eabi`

> **CRITICAL**: This is an RP2040, NOT RP2350. Use `rp2040` feature in embassy-rp.

## Role in System

The Pico Logger is the **low-level data acquisition unit** of the payload system, running bare-metal on an Adafruit Feather RP2040.

### Responsibilities
- Collect data from multiple sensors at high rates
- Log all sensor data to SD card
- Send telemetry to flight computer via USB serial
- Operate reliably during high-G flight conditions

### Sensors
| Sensor | Type | Interface |
|--------|------|-----------|
| BMP390 | Barometer/Altitude | I2C |
| ENS160 | Air Quality/Gas | I2C |
| LSM6DSOX | 6-axis IMU | I2C |
| LIS3MDL | Magnetometer | I2C |
| HGLRC Mini M100 | GPS | UART |

### Storage
- SPI SD card for flight data logging

## Build & Flash

```bash
cd pico_logger

# Build
cargo build --release

# Flash (device must be in BOOTSEL mode)
cargo run --release

# Or use the helper script
.claude/skills/pico-tooling/scripts/wait_bootsel.sh && cargo run --release
```

### Entering BOOTSEL Mode
1. Hold BOOTSEL button on the Feather
2. Press reset button (or replug USB while holding BOOTSEL)
3. Release BOOTSEL
4. Device appears as USB mass storage "RPI-RP2"

## Memory Layout (RP2040)

```
BOOT2: 256B  @ 0x10000000 (second-stage bootloader)
FLASH: 2048K @ 0x10000100
RAM:   256K  @ 0x20000000
```

The RP2040 has 264KB SRAM total (6 banks), but 256K is the usable contiguous region.

## Configuration Files

### .cargo/config.toml
```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "picotool load -u -v -x -t elf"

[build]
target = "thumbv6m-none-eabi"
```

### Cargo.toml Dependencies
```toml
embassy-rp = { version = "0.9.0", features = ["rp2040", "binary-info", "critical-section-impl", "time-driver", "unstable-pac"] }
portable-atomic = { version = "1", features = ["critical-section"] }  # Required for RP2040 atomics
```

### memory.x (RP2040)
```
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS {
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2
} INSERT BEFORE .text;

SECTIONS {
    .bi_entries : ALIGN(4)
    {
        __bi_entries_start = .;
        KEEP(*(.bi_entries));
        . = ALIGN(4);
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;
```

## USB Serial Communication

The firmware uses USB CDC ACM for serial communication:
- Device appears as `/dev/cu.usbmodem*` on macOS
- Baud rate doesn't matter (USB CDC)
- Use `serial_monitor.py` from pico-tooling scripts

## Key Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| embassy-executor | 0.9.1 | Async task runtime |
| embassy-rp | 0.9.0 | RP2040 HAL (use `rp2040` feature) |
| embassy-usb | 0.5 | USB device stack |
| embassy-sync | 0.7.2 | Channels, mutexes |
| embassy-time | 0.4 | Async timers |
| static_cell | 2.1 | Static buffer allocation |
| portable-atomic | 1 | Atomics for Cortex-M0+ |
| heapless | 0.8 | Stack-allocated collections |

## TODO / Planned Features

- [ ] Log data to SPI SD card
- [ ] Log data from BMP390 (Barometer)
- [ ] Log data from ENS160 (Gas)
- [ ] Log data from LSM6DSOX + LIS3MDL (IMU)
- [ ] Log data from HGLRC Mini M100 (GPS)
- [ ] Send sensor data to flight_computer board
- [ ] Add reset-to-bootsel command over USB

## Related Skills

- `embassy-rp`: HAL and peripheral usage
- `embassy`: Async framework patterns
- `sensor-drivers`: Individual sensor details
- `sd-card-logging`: Data storage patterns
- `pico-tooling`: Flashing, serial, and debug tools
- `flight-computer`: Telemetry destination
