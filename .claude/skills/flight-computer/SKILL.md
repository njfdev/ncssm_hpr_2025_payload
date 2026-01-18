---
name: flight-computer
description: Flight computer subproject context. Use when working in the flight_computer package, camera capture, video processing, audio recording, or high-level mission logic.
keywords: [flight computer, flight_computer, camera, video, audio, nokhwa, recording, mission logic, orange pi, linux, dayton audio, calibration]
---

# Flight Computer Subproject

## Role in System

The flight computer is the **high-level processing unit** of the payload system, running on an Orange Pi 4A single-board computer with Linux.

### Responsibilities
- Camera capture and video/image processing
- Audio recording with calibrated microphone
- Receiving telemetry from Pico Logger
- High-level mission decision logic
- Data storage to onboard storage

### Communication
- Receives sensor data from Pico Logger (protocol TBD)
- May transmit to ground station (TBD)

## Project Structure

```
flight_computer/
├── Cargo.toml
├── Cargo.lock
├── README.md
├── src/
│   ├── main.rs              # Entry point, camera capture
│   └── 2581417.txt          # DaytonAudio mic calibration
└── target/
```

## Build & Run

```bash
cd flight_computer
cargo build --release
cargo run --release
```

**Target**: Standard Linux (aarch64 or x86_64 depending on dev machine)

## Prerequisites

Before running on Orange Pi 4A:

1. **Remove standard ffmpeg, install ffmpeg-rk**:
   ```bash
   sudo apt remove ffmpeg
   # Install ffmpeg-rk for hardware encoding
   ```

2. **Install PipeWire audio system**:
   ```bash
   # Replace PulseAudio with PipeWire
   ```

## Current Implementation

Camera capture using nokhwa with callback-based frame processing:

```rust
use nokhwa::{CallbackCamera, pixel_format::LumaFormat, utils::CameraIndex};

let index = CameraIndex::Index(3);  // Camera at index 3
let mut camera = CallbackCamera::new(index, requested, |buffer| {
    let decoded = buffer.decode_image::<LumaFormat>().unwrap();
    decoded.save("./test.png").unwrap();
}).unwrap();

camera.open_stream().unwrap();
```

## Key Dependencies

| Crate | Version | Purpose |
|-------|---------|---------|
| nokhwa | 0.10.10 | Camera/webcam access |
| image | 0.25.9 | Image processing/encoding |

## Calibration Data

`src/2581417.txt` contains DaytonAudio microphone frequency response calibration data for accurate audio recording.

## TODO / Planned Features

- [ ] Video recording (not just single frame capture)
- [ ] Audio recording integration
- [ ] Telemetry reception from Pico Logger
- [ ] Mission state machine
- [ ] Data fusion and logging

## Related Skills

- `orange-pi-4a`: Hardware platform details
- `pico-logger`: Telemetry source
