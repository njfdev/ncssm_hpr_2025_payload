---
name: embassy-rp
description: Embassy HAL for Raspberry Pi RP2040 and RP2350 microcontrollers. Use when working with Pico/Pico 2 peripherals, GPIO, SPI, I2C, UART, PIO, or flashing with picotool.
---

# Embassy-RP HAL

embassy-rp provides safe, idiomatic Rust APIs for RP2040 and RP2350 microcontrollers with both blocking and async support.

## Cargo.toml Setup

### For RP2350 (Pico 2)
```toml
[dependencies]
embassy-executor = { version = "0.9", features = ["arch-cortex-m", "executor-thread"] }
embassy-rp = { version = "0.9", features = ["rp235xa", "time-driver", "critical-section-impl"] }
embassy-time = "0.5"
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-probe = "1.0"

[profile.release]
opt-level = "s"
lto = true
```

### For RP2040 (Original Pico)
```toml
embassy-rp = { version = "0.9", features = ["rp2040", "time-driver", "critical-section-impl"] }
```

## .cargo/config.toml

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "picotool load -u -v -x -t elf"

[build]
target = "thumbv8m.main-none-eabihf"  # RP2350
# target = "thumbv6m-none-eabi"       # RP2040
```

## Memory Layout (memory.x)

```ld
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM : ORIGIN = 0x20000000, LENGTH = 512K
    SRAM8 : ORIGIN = 0x20080000, LENGTH = 4K  /* Core 0 dedicated */
    SRAM9 : ORIGIN = 0x20081000, LENGTH = 4K  /* Core 1 dedicated */
}
```

## Basic Template

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::Timer;
use panic_probe as _;

// Binary info for picotool
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"My Program"),
    embassy_rp::binary_info::rp_program_description!(c"Description"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut led = Output::new(p.PIN_25, Level::Low);

    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}
```

## Peripheral Examples

### GPIO
```rust
use embassy_rp::gpio::{Input, Output, Level, Pull};

let mut led = Output::new(p.PIN_25, Level::Low);
let button = Input::new(p.PIN_15, Pull::Up);

if button.is_low() {
    led.set_high();
}
```

### I2C (Async)
```rust
use embassy_rp::i2c::{I2c, Config};

let i2c = I2c::new_async(
    p.I2C0,
    p.PIN_1,  // SCL
    p.PIN_0,  // SDA
    Irqs,
    Config::default(),
);

let mut buf = [0u8; 2];
i2c.write_read(0x76, &[0xD0], &mut buf).await.unwrap();
```

### SPI (Async)
```rust
use embassy_rp::spi::{Spi, Config};

let mut spi = Spi::new(
    p.SPI0,
    p.PIN_18,  // CLK
    p.PIN_19,  // MOSI
    p.PIN_16,  // MISO
    p.DMA_CH0,
    p.DMA_CH1,
    Config::default(),
);

let mut cs = Output::new(p.PIN_17, Level::High);
cs.set_low();
spi.transfer(&mut buf).await.unwrap();
cs.set_high();
```

### UART (Async)
```rust
use embassy_rp::uart::{Uart, Config};

let uart = Uart::new(
    p.UART0,
    p.PIN_0,  // TX
    p.PIN_1,  // RX
    Irqs,
    p.DMA_CH0,
    p.DMA_CH1,
    Config::default(),
);

let (mut tx, mut rx) = uart.split();
tx.write(b"Hello\n").await.unwrap();
```

## Interrupt Binding

```rust
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{I2C0, UART0};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    UART0_IRQ => embassy_rp::uart::InterruptHandler<UART0>;
});
```

## Flashing

```bash
# Build and flash
cargo run --release

# Build only
cargo build --release

# Manual flash
picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/pico_logger
```

## Resources

- [embassy-rp Docs](https://docs.embassy.dev/embassy-rp)
- [Pico Pico Book](https://pico.implrust.com/)
- [RP2350 Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/silicon.html)
