---
name: pico-tooling
description: Interfacing with Raspberry Pi Pico using picotool, serial communication, and defmt logging. Use when flashing firmware, debugging, setting up serial telemetry, or configuring defmt/probe-rs logging.
---

# Pico Tooling & Serial Communication

## Picotool

### Installation

```bash
# macOS
brew install picotool

# Linux (build from source)
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build && cd build
cmake ..
make
sudo make install
```

### Flashing Firmware

```bash
# Flash and execute ELF file
picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/pico_logger

# Flags:
# -u  Reboot into BOOTSEL if device not found
# -v  Verbose output
# -x  Execute after loading
# -t  Specify file type (elf, uf2, bin)
```

### .cargo/config.toml Setup

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "picotool load -u -v -x -t elf"

[build]
target = "thumbv8m.main-none-eabihf"  # RP2350
# target = "thumbv6m-none-eabi"       # RP2040
```

### Other Picotool Commands

```bash
# Get device info
picotool info

# Reboot device
picotool reboot

# Save firmware from device
picotool save -t uf2 backup.uf2
```

## Serial Communication (USB CDC)

### CRITICAL: 64-Byte Packet Size Limit

**USB CDC `write_packet()` has a 64-byte maximum packet size.** Messages larger than 64 bytes will be silently dropped or truncated. This is a common source of bugs where data appears to send successfully but never arrives.

**Symptoms:**
- `write_packet()` returns Ok but host never receives data
- Short messages work, long messages don't
- Streaming works initially then "stops" when data grows

**Solutions:**
1. Use compact formats (CSV instead of verbose labels)
2. Split large messages across multiple `write_packet()` calls
3. Count characters before sending - stay under 64 bytes

**Example of problematic vs working format:**
```rust
// BAD: 68 bytes - will be dropped!
"#1 P:6593600 T:8615424 A:-204,87,2161 G:5,5,-11 M:-1234,5678,-9012\r\n"

// GOOD: 58 bytes - works reliably
"#1,6593600,8615424,-204,87,2161,5,5,-11,-1234,5678,-9012\r\n"
```

### Cargo.toml Dependencies

```toml
[dependencies]
embassy-usb = "0.5"
embassy-usb-logger = "0.4"  # Optional: for USB logging
```

### USB Serial Setup

```rust
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB config
    let mut config = Config::new(0x1209, 0x0001);  // Use your VID/PID
    config.manufacturer = Some("NCSSM Rocketry");
    config.product = Some("Pico Logger");
    config.serial_number = Some("001");

    // Create USB device builder
    let mut builder = Builder::new(
        driver,
        config,
        &mut make_static!([0u8; 256])[..],
        &mut make_static!([0u8; 256])[..],
        &mut make_static!([0u8; 256])[..],
        &mut make_static!([0u8; 64])[..],
    );

    // Create CDC ACM class
    let mut class = CdcAcmClass::new(&mut builder, &mut make_static!(State::new())[..], 64);

    // Build USB device
    let mut usb = builder.build();

    // Spawn USB task
    spawner.spawn(usb_task(usb)).unwrap();

    // Use class for serial I/O
    loop {
        class.wait_connection().await;

        let mut buf = [0u8; 64];
        match class.read_packet(&mut buf).await {
            Ok(n) => {
                // Echo back or process data
                let _ = class.write_packet(&buf[..n]).await;
            }
            Err(_) => continue,
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

// Helper macro for static buffers
macro_rules! make_static {
    ($val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<_> = static_cell::StaticCell::new();
        STATIC_CELL.init($val)
    }};
}
```

### Reading Serial on Host

```bash
# Linux/macOS - find device
ls /dev/tty.usb*  # macOS
ls /dev/ttyACM*   # Linux

# Read with screen
screen /dev/ttyACM0 115200

# Read with minicom
minicom -D /dev/ttyACM0 -b 115200

# Read with picocom
picocom -b 115200 /dev/ttyACM0
```

## defmt Logging (Recommended for Development)

defmt provides efficient, lightweight logging that transmits formatted strings over RTT (Real-Time Transfer) or probe-rs.

### Cargo.toml

```toml
[dependencies]
defmt = "1.0"
defmt-rtt = "1.0"
panic-probe = { version = "1.0", features = ["print-defmt"] }

[features]
default = ["defmt"]
defmt = ["dep:defmt", "embassy-rp/defmt"]
```

### Usage

```rust
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Pico Logger starting...");
    debug!("Peripherals initialized");

    let pressure = read_pressure().await;
    info!("Pressure: {} Pa", pressure);

    // Log structs
    #[derive(defmt::Format)]
    struct SensorData {
        pressure: u32,
        temp: i16,
    }

    let data = SensorData { pressure: 101325, temp: 25 };
    info!("Sensor data: {:?}", data);
}
```

### Reading defmt Output

```bash
# Using probe-rs (recommended)
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/pico_logger

# Or with cargo
DEFMT_LOG=info cargo run --release
```

### Log Levels

```rust
defmt::trace!("Very verbose");
defmt::debug!("Debug info");
defmt::info!("General info");
defmt::warn!("Warnings");
defmt::error!("Errors");
```

Filter with environment variable:
```bash
DEFMT_LOG=debug cargo run --release
DEFMT_LOG=pico_logger=trace,embassy_rp=warn cargo run --release
```

## Telemetry Logging Pattern

Combining SD card logging with serial telemetry:

```rust
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

// Telemetry data structure
#[derive(Clone, defmt::Format)]
struct TelemetryPacket {
    timestamp_ms: u32,
    pressure_pa: u32,
    altitude_m: f32,
    accel: [i16; 3],
    gyro: [i16; 3],
}

// Channel for passing telemetry between tasks
static TELEMETRY: Channel<CriticalSectionRawMutex, TelemetryPacket, 8> = Channel::new();

#[embassy_executor::task]
async fn sensor_task() {
    loop {
        let packet = TelemetryPacket {
            timestamp_ms: now_ms(),
            pressure_pa: read_pressure().await,
            altitude_m: calculate_altitude(),
            accel: read_accel().await,
            gyro: read_gyro().await,
        };

        // Send to telemetry channel
        TELEMETRY.send(packet).await;

        Timer::after_millis(10).await;  // 100 Hz
    }
}

#[embassy_executor::task]
async fn telemetry_task(mut serial: CdcAcmClass<'static, Driver<'static, USB>>) {
    loop {
        let packet = TELEMETRY.receive().await;

        // Log to defmt for debugging
        defmt::info!("{:?}", packet);

        // Send over USB serial
        let mut buf = [0u8; 64];
        let len = encode_packet(&packet, &mut buf);
        let _ = serial.write_packet(&buf[..len]).await;
    }
}
```

## probe-rs (Alternative to picotool)

probe-rs provides debugging and RTT support but has limited RP2350 support currently.

```bash
# Install
cargo install probe-rs-tools

# Flash and run with RTT
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/pico_logger

# Attach to running device
probe-rs attach --chip RP2350
```

## USB Command Protocol

The firmware can implement a command handler to receive control commands over USB serial. This enables remote control without reflashing.

### Command Format

Commands are newline-terminated strings: `CMD:<ACTION>\n`

| Command | Description |
|---------|-------------|
| `CMD:PING` | Check if device is responsive |
| `CMD:STATUS` | Query device status |
| `CMD:VERSION` | Get firmware version |
| `CMD:REBOOT` | Reboot to application |
| `CMD:BOOTSEL` | Reboot to BOOTSEL mode for flashing |
| `CMD:LOG_START` | Start SD card logging |
| `CMD:LOG_STOP` | Stop SD card logging |
| `CMD:SENSORS` | Get current sensor readings |
| `CMD:CALIBRATE` | Trigger sensor calibration |

Responses: `OK\n`, `OK:<data>\n`, or `ERR:<reason>\n`

### Embassy Command Handler Implementation

```rust
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use heapless::String;

// Signals for cross-task communication
static LOG_START: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static LOG_STOP: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Process a command string and return response
fn handle_command(cmd: &str) -> String<64> {
    let cmd = cmd.trim();
    let mut response: String<64> = String::new();

    match cmd {
        "CMD:PING" => {
            let _ = response.push_str("OK:PONG");
        }
        "CMD:VERSION" => {
            let _ = response.push_str("OK:");
            let _ = response.push_str(env!("CARGO_PKG_VERSION"));
        }
        "CMD:STATUS" => {
            // Return current device status
            let _ = response.push_str("OK:RUNNING");
        }
        "CMD:REBOOT" => {
            let _ = response.push_str("OK:REBOOTING");
            // Schedule reboot after response is sent
            cortex_m::peripheral::SCB::sys_reset();
        }
        "CMD:BOOTSEL" => {
            let _ = response.push_str("OK:ENTERING_BOOTSEL");
            // Reboot to BOOTSEL mode (RP2040/RP2350)
            reset_to_usb_boot(0, 0);
        }
        "CMD:LOG_START" => {
            LOG_START.signal(());
            let _ = response.push_str("OK:LOGGING_STARTED");
        }
        "CMD:LOG_STOP" => {
            LOG_STOP.signal(());
            let _ = response.push_str("OK:LOGGING_STOPPED");
        }
        "CMD:SENSORS" => {
            // Read current sensor values
            let _ = response.push_str("OK:P=101325,T=25,A=0,0,16384");
        }
        "CMD:CALIBRATE" => {
            // Trigger calibration routine
            let _ = response.push_str("OK:CALIBRATING");
        }
        _ => {
            let _ = response.push_str("ERR:UNKNOWN_CMD");
        }
    }

    response
}

#[embassy_executor::task]
async fn command_task(mut serial: CdcAcmClass<'static, Driver<'static, USB>>) {
    let mut buf = [0u8; 64];
    let mut cmd_buf: String<64> = String::new();

    loop {
        serial.wait_connection().await;

        match serial.read_packet(&mut buf).await {
            Ok(n) => {
                // Accumulate bytes into command buffer
                for &byte in &buf[..n] {
                    if byte == b'\n' {
                        // Process complete command
                        let response = handle_command(&cmd_buf);
                        let mut out: String<66> = String::new();
                        let _ = out.push_str(&response);
                        let _ = out.push('\n');
                        let _ = serial.write_packet(out.as_bytes()).await;
                        cmd_buf.clear();
                    } else if byte != b'\r' {
                        let _ = cmd_buf.push(byte as char);
                    }
                }
            }
            Err(_) => {
                cmd_buf.clear();
            }
        }
    }
}
```

### Using Signals for Cross-Task Control

```rust
#[embassy_executor::task]
async fn logging_task() {
    loop {
        // Wait for start signal
        LOG_START.wait().await;
        defmt::info!("Logging started");

        // Open file, start logging loop
        loop {
            // Check for stop signal (non-blocking)
            if LOG_STOP.signaled() {
                LOG_STOP.reset();
                defmt::info!("Logging stopped");
                break;
            }

            // Log data...
            Timer::after_millis(10).await;
        }
    }
}
```

## Utility Scripts

This skill includes helper scripts in the `scripts/` directory.

**Install dependencies:**
```bash
pip install pyserial matplotlib
```

### pico_control.py
Send commands to control the Pico remotely.

```bash
# Query device status
python scripts/pico_control.py status

# Reboot to BOOTSEL mode for flashing
python scripts/pico_control.py bootsel

# Start/stop logging
python scripts/pico_control.py start-logging
python scripts/pico_control.py stop-logging

# Send custom command
python scripts/pico_control.py send "CMD:CUSTOM"

# Interactive mode
python scripts/pico_control.py interactive
```

### serial_monitor.py
Monitor serial output with duration/line limits.

```bash
# Run for 10 seconds
python scripts/serial_monitor.py --duration 10

# Read 100 lines then exit
python scripts/serial_monitor.py --lines 100

# Log to file with timestamps
python scripts/serial_monitor.py --log flight.log --timestamp --duration 60
```

### telemetry_parser.py
Parse binary telemetry packets with limits.

```bash
# Read for 10 seconds
python scripts/telemetry_parser.py --duration 10

# Read 100 packets then exit
python scripts/telemetry_parser.py --packets 100 --csv output.csv
```

**Note**: Edit the `TelemetryPacket` struct and `PACKET_SIZE` to match your firmware.

### plot_telemetry.py
Real-time plotting of telemetry data.

```bash
python scripts/plot_telemetry.py /dev/tty.usbmodem2101
python scripts/plot_telemetry.py --file data.csv
```

### flash.sh
Build and flash firmware.

```bash
./scripts/flash.sh           # Debug build
./scripts/flash.sh --release # Release build
```

### pico_info.sh
Display connected device information.

```bash
./scripts/pico_info.sh
```

## Debugging USB CDC Issues

### Data Not Being Received

When `write_packet()` appears to succeed but host doesn't receive data:

1. **Check packet size** - Must be ≤64 bytes (see above)
2. **Add breadcrumb messages** - Send short debug markers before/after suspect code:
   ```rust
   let _ = class.write_packet(b"BEFORE_OP\r\n").await;
   // ... suspect operation ...
   let _ = class.write_packet(b"AFTER_OP\r\n").await;
   ```
3. **Test with simple data first** - Replace complex telemetry with `"TEST\r\n"` to isolate issue
4. **Check for blocking operations** - I2C reads, timers, or other async calls might hang

### Streaming Data Pattern

When implementing continuous data streaming:

```rust
// Working pattern: fixed iterations, no select
for _ in 0..50 {
    // Read sensors
    let _ = sensor::read(&mut i2c, &mut data).await;

    // Format (keep under 64 bytes!)
    let msg = format_telemetry(counter, &data);
    let _ = class.write_packet(msg.as_bytes()).await;

    Timer::after_millis(100).await;
}
```

**Note:** `embassy_futures::select` with `read_packet()` and `Timer` may not work as expected for interruptible streaming - the timer branch may never fire. Use fixed iteration counts or dedicated tasks instead.

### Quick Serial Test Script

```python
import serial
import time

ser = serial.Serial('/dev/tty.usbmodem0011', 115200, timeout=3)
time.sleep(0.5)

ser.write(b'PING\n')
time.sleep(0.2)
print('Response:', repr(ser.read(ser.in_waiting or 100)))

ser.close()
```

## Resources

- [picotool GitHub](https://github.com/raspberrypi/picotool)
- [defmt Book](https://defmt.ferrous-systems.com/)
- [probe-rs](https://probe.rs/)
- [embassy-usb Docs](https://docs.embassy.dev/embassy-usb/)
