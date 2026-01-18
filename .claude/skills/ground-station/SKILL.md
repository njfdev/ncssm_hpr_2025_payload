---
name: ground-station
description: Ground station subproject context. Use when working in the ground_station package, telemetry reception, mission control interface, or real-time flight monitoring.
keywords: [ground station, ground_station, telemetry, mission control, monitoring, radio, lora, receiver, display, dashboard]
---

# Ground Station Subproject

## Role in System

The ground station is the **mission control and telemetry reception** component, running on a laptop or desktop computer on the ground.

### Responsibilities
- Receive real-time telemetry from rocket during flight
- Display flight data (altitude, velocity, GPS position, sensor readings)
- Provide mission control interface
- Log received data for post-flight analysis
- Alert on anomalies or mission events

### Communication
- Receives telemetry from rocket (radio link TBD)
- Protocol and hardware to be determined

## Project Structure

```
ground_station/
├── Cargo.toml
├── Cargo.lock
└── src/
    └── main.rs    # Entry point (stub)
```

## Build & Run

```bash
cd ground_station
cargo build --release
cargo run --release
```

**Target**: Standard desktop (Linux, macOS, Windows)

## Current Implementation

Minimal stub:

```rust
fn main() {
    println!("Hello, world!");
}
```

## Planned Features

- [ ] Radio receiver integration (hardware TBD)
- [ ] Telemetry packet parsing
- [ ] Real-time data display (TUI or GUI)
- [ ] GPS tracking visualization
- [ ] Flight phase detection
- [ ] Data logging
- [ ] Alerts and notifications

## Potential Dependencies

| Crate | Purpose |
|-------|---------|
| serialport | Serial/radio communication |
| egui/iced | GUI framework |
| ratatui | Terminal UI |
| plotters | Data visualization |
| tokio | Async runtime |

## Design Considerations

### Telemetry Protocol
- Packet format matching Pico Logger output
- Error detection (CRC)
- Handling packet loss

### Display Requirements
- Real-time altitude graph
- GPS map view
- Sensor data panels
- Flight phase indicator
- Signal strength

### Radio Options
- LoRa modules (long range, low bandwidth)
- 900MHz/433MHz ISM band
- Consider amateur radio regulations

## Related Skills

- `pico-logger`: Telemetry source
- `flight-computer`: Intermediate processor
