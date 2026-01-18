---
name: embassy
description: Embassy async embedded framework for Rust. Use when working with async/await in no_std embedded contexts, task executors, or Embassy crates like embassy-executor, embassy-time, embassy-sync.
keywords: [embassy, async, await, no_std, embedded, executor, embassy-time, embassy-sync, embassy-executor, timer, channel, mutex, signal, spawner, interrupt]
---

# Embassy Async Embedded Framework

Embassy is a modern embedded framework using Rust's async/await for efficient, safe embedded programming.

## Core Concepts

### Async Executor
- **No alloc, no heap**: Tasks are statically allocated
- **Single stack**: No per-task stack sizing needed
- **Compile-time transformation**: Tasks become state machines
- **No RTOS needed**: Replaces kernel context switching

### Low Power
- Executor automatically sleeps when idle
- Tasks wake via interrupts (no busy-loop polling)
- Ideal for battery-powered devices

## Key Crates

### embassy-executor
Async task runtime for embedded systems.

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    spawner.spawn(background_task()).unwrap();
    // Main task continues...
}

#[embassy_executor::task]
async fn background_task() {
    loop {
        // Do work...
        Timer::after_secs(1).await;
    }
}
```

### embassy-time
Timers, delays, and timeouts.

```rust
use embassy_time::{Timer, Duration, Instant};

// Delay
Timer::after(Duration::from_millis(100)).await;
Timer::after_secs(1).await;

// Timeout
let result = with_timeout(Duration::from_secs(5), async_operation()).await;

// Measuring time
let start = Instant::now();
do_work().await;
let elapsed = start.elapsed();
```

### embassy-sync
Synchronization primitives for async contexts.

```rust
use embassy_sync::{
    mutex::Mutex,
    channel::Channel,
    signal::Signal,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

// Shared state
static SHARED: Mutex<CriticalSectionRawMutex, u32> = Mutex::new(0);

// Channel for message passing
static CHANNEL: Channel<CriticalSectionRawMutex, Message, 4> = Channel::new();

// Signal for notifications
static SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
```

### embassy-embedded-hal
Async traits for embedded-hal compatibility.

## Additional Crates

| Crate | Purpose |
|-------|---------|
| embassy-net | TCP, UDP, DHCP, ICMP networking |
| embassy-usb | Device-side USB (CDC, HID, custom) |
| embassy-boot | Power-fail-safe firmware updates |
| trouble | Bluetooth Low Energy 4.x/5.x |

## HAL Support

- **embassy-stm32**: All STM32 families
- **embassy-nrf**: Nordic nRF52, nRF53, nRF54, nRF91
- **embassy-rp**: Raspberry Pi RP2040, RP2350
- **embassy-mspm0**: Texas Instruments MSPM0
- **esp-hal**: Espressif ESP32 series
- **ch32-hal**: WCH RISC-V chips

## Resources

- [Embassy Website](https://embassy.dev/)
- [Embassy Book](https://embassy.dev/book/)
- [GitHub Repository](https://github.com/embassy-rs/embassy)
- [API Documentation](https://docs.embassy.dev/)
