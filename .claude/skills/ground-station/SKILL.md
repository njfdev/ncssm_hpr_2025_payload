---
name: ground-station
description: Ground station subproject context. Use when working in the ground_station package, telemetry reception, mission control interface, or real-time flight monitoring.
keywords: [ground station, ground_station, telemetry, mission control, monitoring, radio, RFD 900x, receiver, UART bridge, USB CDC]
---

# Ground Station - RFD 900x USB-UART Bridge

## Role in System

The ground station is an **RP2040 Embassy firmware** (Adafruit Feather RP2040, same board as pico_logger) that bridges USB CDC serial and UART to interface with an RFD 900x-US radio modem. Primary use case is receiving telemetry from the rocket during flight.

**Data flow**: Computer <-> USB CDC <-> RP2040 <-> UART (57600 8N1, TX=GP0, RX=GP1) <-> RFD 900x

## Project Structure

```
ground_station/
├── .cargo/config.toml  # thumbv6m-none-eabi target, picotool runner
├── build.rs            # Copies memory.x, sets linker flags
├── memory.x            # RP2040 memory layout (2MB flash, 256K RAM)
├── Cargo.toml          # Embassy dependencies
└── src/
    ├── main.rs          # Entry point, USB CDC + UART setup, command loop
    ├── protocol.rs      # Mode enum, DataPacket, EscapeDetector
    └── uart_bridge.rs   # UART rx/tx async tasks, inter-task channels
```

## Build & Flash

```bash
cd ground_station
cargo build --release
cargo run --release  # Uses picotool to flash (device must be in BOOTSEL mode)
```

**Target**: `thumbv6m-none-eabi` (RP2040 Cortex-M0+)

## Operating Modes

### Command Mode (default)
- UART RX data forwarded to USB
- USB input interpreted as local commands

| Command | Action |
|---------|--------|
| `PING` | Responds `PONG\r\n` |
| `BOOTSEL` | Reboot to BOOTSEL mode |
| `STATUS` | Show current mode and UART config |
| `PASSTHROUGH` | Enter transparent bridge mode |
| `ATMODE` | Enter AT config mode (handles +++ guard times) |
| `SEND <data>` | Send payload bytes through radio |
| `HELP` | List available commands |

### Passthrough Mode
- Fully transparent bidirectional USB <-> UART bridge
- All USB data forwarded to UART and vice versa
- **Exit**: 3x Ctrl-A (0x01 0x01 0x01) returns to Command mode

### AT Config Mode
- For RFD 900x AT command configuration
- Entry sends `+++` with 1-second guard times
- USB input forwarded to UART as AT commands
- UART responses forwarded to USB
- **Exit**: `EXITAT` (local) or `ATO` (also sent to radio)

## Architecture

### Async Tasks (Embassy)
- `main` - Init, USB CDC command loop, mode switching
- `usb_task` - Drives USB device stack
- `uart_rx_task` - Reads UART with 2ms inter-byte timeout batching into DataPackets
- `uart_tx_task` - Reads DataPackets from channel, writes to UART

### Inter-Task Communication
Two `Channel<CriticalSectionRawMutex, DataPacket, 8>` statics:
- `UART_TO_USB` - Radio data → computer
- `USB_TO_UART` - Computer data → radio

### DataPacket
64-byte buffer + length, Copy type for use in channels.

## Hardware Configuration

- **Board**: Adafruit Feather RP2040
- **USB**: CDC ACM, VID=0x2e8a PID=0x000b, "Ground Station RFD Bridge"
- **UART0**: 57600 baud, 8N1, TX=GP0, RX=GP1, DMA channels 0/1
- **Radio**: RFD 900x-US (connected to UART0)

## Key Gotchas

### embassy-rp 0.9.0 UART Types
- `UartTx<'d, M: Mode>` and `UartRx<'d, M: Mode>` use **Mode** (Async/Blocking) as generic, NOT the peripheral type
- `Uart::new()` takes 7 args: `(uart, tx_pin, rx_pin, irq_binding, tx_dma, rx_dma, config)`
- UART requires interrupt binding even for DMA mode: `UART0_IRQ => uart::InterruptHandler<UART0>`
- Task function signatures use `UartRx<'static, Async>`, NOT `UartRx<'static, UART0>`

### USB PID Allocation
- Pico Logger: 0x000a
- Ground Station: 0x000b

## Related Skills

- `pico-logger`: Same board, similar Embassy patterns
- `embassy-rp`: Embassy HAL for RP2040
- `pico-tooling`: Flashing and serial monitoring
