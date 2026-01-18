---
name: orange-pi-4a
description: Development guidance for Orange Pi 4A SBC. Use when working with camera capture, microphone/audio, video encoding, GPIO, or the flight computer hardware platform.
---

# Orange Pi 4A Development

## Hardware Overview

**SoC**: Allwinner T527
- Octa-core Cortex-A55 (4x 1.8GHz + 4x 1.42GHz)
- XuanTie E906 RISC-V core @ 200MHz
- Mali-G57 GPU
- 2 TOPS NPU for AI acceleration
- HiFi4 DSP for audio processing

**Memory**: 2GB or 4GB LPDDR4/4X

## Camera Interfaces

- **2-lane MIPI-CSI**: Lower bandwidth cameras
- **4-lane MIPI-CSI**: Higher resolution/framerate cameras

### Camera in Rust with nokhwa

```rust
use nokhwa::{
    CallbackCamera,
    pixel_format::LumaFormat,
    utils::{CameraIndex, RequestedFormat, RequestedFormatType},
};

let index = CameraIndex::Index(3);  // Check dmesg for actual index
let requested = RequestedFormat::new::<LumaFormat>(
    RequestedFormatType::AbsoluteHighestFrameRate,
);

let mut camera = CallbackCamera::new(index, requested, |buffer| {
    let decoded = buffer.decode_image::<LumaFormat>().unwrap();
    // Process frame...
}).unwrap();

camera.open_stream().unwrap();
```

**Available pixel formats**: `LumaFormat`, `RgbFormat`, `YuyvFormat`

## Audio

- **Output**: 3.5mm headphone jack
- **DSP**: HiFi4 for audio/voice/speech processing
- No built-in microphone; use external via 3.5mm or USB

### Required Setup

1. Install PipeWire (replaces PulseAudio):
   ```bash
   # Follow distribution-specific instructions
   ```

2. Install ffmpeg-rk (hardware encoding):
   ```bash
   # Remove standard ffmpeg first
   sudo apt remove ffmpeg
   # Install ffmpeg-rk for Rockchip/Allwinner hardware encoding
   ```

## GPIO

40-pin header compatible with Raspberry Pi pinout:
- GPIO (directly controllable pins)
- I2C (for sensors)
- SPI (for SD cards, displays)
- UART (for GPS, serial devices)
- PWM (for servos, motors)

## Supported Operating Systems

- Ubuntu
- Debian
- Android 13

## Resources

- [Official Wiki](http://www.orangepi.org/orangepiwiki/index.php/Orange_Pi_4A)
- [Hardware Specs](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-4A.html)
