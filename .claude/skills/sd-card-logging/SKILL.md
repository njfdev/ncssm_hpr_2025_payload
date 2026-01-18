---
name: sd-card-logging
description: SD card storage and data logging on embedded systems. Use when working with embedded-sdmmc, FAT filesystems, SPI SD cards, flight data logging, or the talc allocator.
---

# SD Card Logging for Embedded Systems

## Overview

Flight data logging to SD card via SPI interface using `embedded-sdmmc` crate.

## Cargo.toml

```toml
[dependencies]
embedded-sdmmc = "0.8"
# Optional: for dynamic allocation if needed
talc = "4.4"
```

## Hardware Setup

**SPI Connections** (typical pinout):
| SD Card Pin | Function | Pico Pin |
|-------------|----------|----------|
| CS | Chip Select | GPIO17 |
| CLK | Clock | GPIO18 |
| MOSI | Data In | GPIO19 |
| MISO | Data Out | GPIO16 |
| VCC | 3.3V | 3V3 |
| GND | Ground | GND |

## Basic Usage with embedded-sdmmc

```rust
use embedded_sdmmc::{SdCard, VolumeManager, Mode, VolumeIdx};
use embassy_rp::spi::{Spi, Config};
use embassy_rp::gpio::{Output, Level};
use embassy_time::Delay;

// Setup SPI
let spi = Spi::new_blocking(
    p.SPI0,
    p.PIN_18,  // CLK
    p.PIN_19,  // MOSI
    p.PIN_16,  // MISO
    Config::default(),
);

let cs = Output::new(p.PIN_17, Level::High);

// Create SD card instance
let sdcard = SdCard::new(spi, cs, Delay);

// Create volume manager with a time source
let mut volume_mgr = VolumeManager::new(sdcard, DummyTimesource);

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
