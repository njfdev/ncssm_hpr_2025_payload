---
name: sd-card-logging
description: SD card storage and data logging on embedded systems. Use when working with embedded-sdmmc, FAT filesystems, SPI SD cards, flight data logging, or the talc allocator.
keywords: [sd card, sdcard, embedded-sdmmc, fat32, fat16, spi, logging, csv, data logging, storage, file, adalogger, talc, allocator, ExclusiveDevice]
---

# SD Card Logging for Embedded Systems

## Overview

Flight data logging to SD card via SPI interface using `embedded-sdmmc` crate.

## Cargo.toml

```toml
[dependencies]
embedded-sdmmc = "0.8"
embedded-hal = "1.0"
embedded-hal-bus = "0.2"  # For ExclusiveDevice SpiDevice wrapper
# Optional: for dynamic allocation if needed
talc = "4.4"
```

## Hardware Setup

### Adafruit Feather RP2040 Adalogger

The Adalogger has a built-in SD card slot using SPI0:

| SD Card Pin | Function | GPIO |
|-------------|----------|------|
| CLK | Clock | GP18 |
| MOSI | Data In | GP19 |
| MISO | Data Out | GP20 |
| CS | Chip Select | GP23 |

### Generic SPI SD Card

| SD Card Pin | Function | Pico Pin |
|-------------|----------|----------|
| CS | Chip Select | GPIO17 |
| CLK | Clock | GPIO18 |
| MOSI | Data In | GPIO19 |
| MISO | Data Out | GPIO16 |
| VCC | 3.3V | 3V3 |
| GND | Ground | GND |

## CRITICAL: embedded-sdmmc 0.8 API Changes

**Version 0.8+ requires `SpiDevice` trait, not raw SPI bus + CS pin.**

You must use `ExclusiveDevice` from `embedded-hal-bus` to wrap your SPI bus:

```rust
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx};
use embassy_rp::spi::{Spi, Config as SpiConfig, Blocking};
use embassy_rp::gpio::{Output, Level};

// Delay implementation for SD card init
pub struct Delay;
impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns / 100; // Rough approximation
        for _ in 0..cycles {
            cortex_m::asm::nop();
        }
    }
}

// Configure SPI at 400kHz for SD card init (required by spec)
let mut spi_config = SpiConfig::default();
spi_config.frequency = 400_000;  // MUST be 400kHz for init

let spi = Spi::new_blocking(spi0, clk, mosi, miso, spi_config);
let cs = Output::new(cs_pin, Level::High);

// CRITICAL: Wrap with ExclusiveDevice to get SpiDevice trait
let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

// Now create SD card instance
let sdcard = SdCard::new(spi_dev, Delay);

// Verify card communication (this completes initialization)
let _size = sdcard.num_bytes()?;

// Create volume manager with a time source
let mut volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);

// CRITICAL: Increase SPI speed after initialization!
// SD cards support high speeds after init. 16MHz is safe for most cards.
// This provides 40x faster writes compared to staying at 400kHz.
volume_mgr.device().spi(|spi_dev| {
    spi_dev.bus_mut().set_frequency(16_000_000);
});
```

## CRITICAL: SPI Speed After Initialization

**SD cards require 400kHz for initialization, but support MUCH higher speeds afterward.**

Staying at 400kHz causes severe performance issues:
- At 400kHz: Each 2KB batch write takes ~48ms
- At 16MHz: Same write takes ~1ms (40x faster!)

**Symptom of slow SPI**: Timing gaps that start small but grow over time as the FAT table operations become slower with file growth.

**Solution**: Use `volume_mgr.device().spi()` to access the underlying SPI and increase frequency after card init:

```rust
volume_mgr.device().spi(|spi_dev| {
    spi_dev.bus_mut().set_frequency(16_000_000);  // 16MHz
});
```

## Embassy-RP 0.9+ Peripheral Types

Embassy 0.9 uses `Peri<'static, T>` wrapper types. Your function signatures should use generics with pin traits:

```rust
use embassy_rp::spi::{ClkPin, MosiPin, MisoPin};
use embassy_rp::Peri;

pub fn new<C, MO, MI, CS>(
    spi0: Peri<'static, SPI0>,
    clk: Peri<'static, C>,
    mosi: Peri<'static, MO>,
    miso: Peri<'static, MI>,
    cs_pin: Peri<'static, CS>,
) -> Result<Self, &'static str>
where
    C: ClkPin<SPI0> + 'static,
    MO: MosiPin<SPI0> + 'static,
    MI: MisoPin<SPI0> + 'static,
    CS: embassy_rp::gpio::Pin + 'static,
{
    // ...
}
```

## Basic File Operations

```rust
// Open first partition
let mut volume = volume_mgr.open_volume(VolumeIdx(0))?;
let mut root_dir = volume.open_root_dir()?;

// Create/open file for writing
let mut file = root_dir.open_file_in_dir(
    "DATA.LOG",
    Mode::ReadWriteCreateOrAppend,
)?;

// Write data
file.write(b"timestamp,pressure,temp,ax,ay,az\n")?;
file.flush()?;
```

## Time Source Implementation

```rust
use embedded_sdmmc::{TimeSource, Timestamp};

struct DummyTimesource;

impl TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> Timestamp {
        // Return a fixed timestamp or implement RTC
        Timestamp {
            year_since_1970: 55,  // 2025
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
```

## Data Logging Patterns

### Binary Format (Compact, Fast)
```rust
#[repr(C, packed)]
struct LogEntry {
    timestamp_ms: u32,
    pressure_pa: u32,
    temp_c: i16,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
}

// Write binary
let bytes = unsafe {
    core::slice::from_raw_parts(
        &entry as *const LogEntry as *const u8,
        core::mem::size_of::<LogEntry>(),
    )
};
file.write(bytes)?;
```

### Text/CSV Format (Human Readable)
```rust
use core::fmt::Write;
use heapless::String;

let mut line: String<128> = String::new();
write!(&mut line, "{},{},{},{},{},{}\n",
    timestamp, pressure, temp, ax, ay, az)?;
file.write(line.as_bytes())?;
```

## Power-Safe Writing

```rust
// Flush after critical data
file.write(data)?;
file.flush()?;  // Ensures data is written to card

// Consider periodic flushes
static WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
let count = WRITE_COUNT.fetch_add(1, Ordering::Relaxed);
if count % 100 == 0 {
    file.flush()?;
}
```

## File Naming for Multiple Flights

```rust
// Find next available filename
for i in 0..999 {
    let mut name: String<12> = String::new();
    write!(&mut name, "FLT{:03}.LOG", i)?;

    match root_dir.find_directory_entry(&name) {
        Err(embedded_sdmmc::Error::NotFound) => {
            // This filename is available
            return root_dir.open_file_in_dir(&name, Mode::ReadWriteCreateOrAppend);
        }
        Ok(_) => continue,  // File exists, try next
        Err(e) => return Err(e),
    }
}
```

## Performance Tips

1. **Buffer writes**: Accumulate data before writing to reduce SD card operations
2. **Use binary format**: ~50% smaller than CSV, faster to write
3. **Avoid frequent opens/closes**: Keep file handle open during flight
4. **Pre-allocate file**: Some cards are faster with pre-sized files

## Filesystem Notes

- **FAT16/FAT32**: Supported by embedded-sdmmc
- **exFAT**: Not directly supported; use separate crate if needed
- **8.3 filenames**: FAT16/32 limitation without LFN support

## talc Allocator (Optional)

For dynamic allocation in no_std:

```rust
use talc::{Talc, Span};

static mut HEAP: [u8; 16384] = [0; 16384];

#[global_allocator]
static ALLOCATOR: Talc<spin::Mutex<()>> = Talc::new(unsafe {
    Span::from_array(&mut HEAP)
}).lock();
```

---

## Troubleshooting

### "SD card not responding"
1. Check SPI wiring (CLK, MOSI, MISO, CS)
2. Ensure SD card is FAT32 formatted (not exFAT)
3. Verify SPI frequency is 400kHz for init (some cards are picky)
4. Check power supply - SD cards need stable 3.3V

### ExclusiveDevice/SpiDevice errors
embedded-sdmmc 0.8+ requires `SpiDevice` trait, not raw `SpiBus`. Use `ExclusiveDevice::new(spi, cs, delay)` to wrap your SPI bus.

### Embassy Peri<> type errors
Embassy 0.9+ uses `Peri<'static, T>` wrapper types. Don't use raw peripheral types like `SPI0` or `PIN_18` in function signatures - use generics with trait bounds or accept `Peri<'static, T>` directly.

### File operations fail silently
Always check return values. `file.write()` returns `Result` - errors may indicate card full, filesystem corruption, or card ejected.

### Rust 2024 match ergonomics
In Rust 2024 edition, don't use `ref` or `ref mut` when matching on references:
```rust
// Wrong in Rust 2024:
if let (Some(ref mut logger), Some(ref filename)) = (&mut sd_logger, &log_filename) { ... }

// Correct:
if let (Some(logger), Some(filename)) = (&mut sd_logger, &log_filename) { ... }
```

---

## Working Code Reference

The pico_logger implementation in `pico_logger/src/sdcard.rs` provides a complete working example including:
- SdLogger struct with VolumeManager
- Automatic file naming (FLT000.CSV, FLT001.CSV, etc.)
- CSV logging with sensor data
- Proper Embassy 0.9 peripheral handling
