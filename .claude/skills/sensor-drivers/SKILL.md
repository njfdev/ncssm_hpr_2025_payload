---
name: sensor-drivers
description: Guidance for embedded sensor drivers used in the payload. Use when working with BMP390, ENS160, LSM6DSOX, LIS3MDL, GPS, IMU, barometer, or any sensor integration.
keywords: [bmp390, ens160, lsm6dsox, lis3mdl, gps, imu, barometer, magnetometer, accelerometer, gyroscope, pressure, temperature, altitude, i2c, sensor, embedded-hal]
---

# Payload Sensor Drivers

## Hardware Configuration

| Sensor | Type | Bus | SDA | SCL | I2C Address |
|--------|------|-----|-----|-----|-------------|
| BMP390 | Barometer | I2C1 | GP2 | GP3 | 0x77 (SDO high) |
| LSM6DSOX | 6-axis IMU | I2C0 | GP28 | GP29 | 0x6A (SA0 low) |
| LIS3MDL | Magnetometer | I2C0 | GP28 | GP29 | 0x1C (SA1 low) |

## BMP390 - Barometric Pressure Sensor

**Datasheet**: [Bosch BMP390](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf)

### Specifications
- Pressure range: 300-1250 hPa
- Relative accuracy: ±3 Pa (~0.25m altitude)
- Absolute accuracy: ±0.40 hPa
- Temperature: -40 to +85°C (±0.5°C accuracy)
- Power: 3.2 µA @ 1Hz

### I2C Address
- **0x76**: SDO pin connected to GND
- **0x77**: SDO pin connected to VDDIO (3.3V) ← **Our configuration**

### Key Registers

| Register | Address | Description |
|----------|---------|-------------|
| CHIP_ID | 0x00 | Chip ID (reads 0x60) |
| ERR_REG | 0x02 | Error register |
| STATUS | 0x03 | Sensor status |
| DATA_0-5 | 0x04-0x09 | Pressure and temperature data |
| INT_STATUS | 0x11 | Interrupt status |
| PWR_CTRL | 0x1B | Power control (enable press/temp) |
| OSR | 0x1C | Oversampling settings |
| ODR | 0x1D | Output data rate |
| CONFIG | 0x1F | IIR filter config |
| CALIB_DATA | 0x31-0x45 | Calibration coefficients (21 bytes) |
| CMD | 0x7E | Command register |

### Power Modes
- **Sleep**: No measurements (default after reset)
- **Forced**: Single measurement, then sleep
- **Normal**: Continuous measurements with standby

### Initialization Sequence
```rust
const BMP390_ADDR: u8 = 0x77;
const CHIP_ID: u8 = 0x00;
const PWR_CTRL: u8 = 0x1B;
const OSR: u8 = 0x1C;
const ODR: u8 = 0x1D;

// 1. Read and verify chip ID (should be 0x60)
let mut id = [0u8];
i2c.write_read(BMP390_ADDR, &[CHIP_ID], &mut id).await?;
assert_eq!(id[0], 0x60);

// 2. Read calibration data (21 bytes from 0x31)
let mut calib = [0u8; 21];
i2c.write_read(BMP390_ADDR, &[0x31], &mut calib).await?;

// 3. Configure oversampling (x8 pressure, x1 temp)
i2c.write(BMP390_ADDR, &[OSR, 0b00000011]).await?;

// 4. Configure ODR (50 Hz = 0x02)
i2c.write(BMP390_ADDR, &[ODR, 0x02]).await?;

// 5. Enable pressure and temp, normal mode
// PWR_CTRL: press_en=1, temp_en=1, mode=normal(11)
i2c.write(BMP390_ADDR, &[PWR_CTRL, 0b00110011]).await?;
```

### Reading Data
```rust
const DATA_0: u8 = 0x04;

// Read 6 bytes: pressure (3) + temperature (3)
let mut data = [0u8; 6];
i2c.write_read(BMP390_ADDR, &[DATA_0], &mut data).await?;

// Pressure: 24-bit unsigned (XLSB, LSB, MSB)
let press_raw = (data[2] as u32) << 16 | (data[1] as u32) << 8 | data[0] as u32;

// Temperature: 24-bit unsigned (XLSB, LSB, MSB)
let temp_raw = (data[5] as u32) << 16 | (data[4] as u32) << 8 | data[3] as u32;

// Apply compensation algorithm from datasheet using calibration data
```

---

## LSM6DSOX - 6-Axis IMU (Accelerometer + Gyroscope)

**Datasheet**: [ST LSM6DSOX](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf)

### Specifications
- Accelerometer: ±2/±4/±8/±16 g
- Gyroscope: ±125/±250/±500/±1000/±2000 dps
- ODR: up to 6.66 kHz
- Power: 0.55 mA in high-performance mode

### I2C Address
- **0x6A**: SA0/SDO pin connected to GND ← **Our configuration**
- **0x6B**: SA0/SDO pin connected to VDDIO

### Key Registers

| Register | Address | Description |
|----------|---------|-------------|
| FUNC_CFG_ACCESS | 0x01 | Embedded functions config |
| WHO_AM_I | 0x0F | Device ID (reads 0x6C) |
| CTRL1_XL | 0x10 | Accelerometer control |
| CTRL2_G | 0x11 | Gyroscope control |
| CTRL3_C | 0x12 | Control register 3 |
| CTRL6_C | 0x15 | Accelerometer power mode |
| CTRL7_G | 0x16 | Gyroscope power mode |
| STATUS_REG | 0x1E | Data ready status |
| OUTX_L_G | 0x22 | Gyro X low byte |
| OUTX_H_G | 0x23 | Gyro X high byte |
| OUTY_L_G | 0x24 | Gyro Y low byte |
| OUTY_H_G | 0x25 | Gyro Y high byte |
| OUTZ_L_G | 0x26 | Gyro Z low byte |
| OUTZ_H_G | 0x27 | Gyro Z high byte |
| OUTX_L_A | 0x28 | Accel X low byte |
| OUTX_H_A | 0x29 | Accel X high byte |
| OUTY_L_A | 0x2A | Accel Y low byte |
| OUTY_H_A | 0x2B | Accel Y high byte |
| OUTZ_L_A | 0x2C | Accel Z low byte |
| OUTZ_H_A | 0x2D | Accel Z high byte |

### CTRL1_XL (Accelerometer Control)
```
[7:4] ODR_XL: Output data rate
      0000 = Power-down
      0001 = 12.5 Hz
      0010 = 26 Hz
      0011 = 52 Hz
      0100 = 104 Hz
      0101 = 208 Hz
      0110 = 416 Hz
      0111 = 833 Hz
      1000 = 1.66 kHz
      1001 = 3.33 kHz
      1010 = 6.66 kHz

[3:2] FS_XL: Full-scale selection
      00 = ±2 g
      01 = ±16 g
      10 = ±4 g
      11 = ±8 g

[1] LPF2_XL_EN: Low-pass filter enable
[0] (unused)
```

### CTRL2_G (Gyroscope Control)
```
[7:4] ODR_G: Output data rate (same as accelerometer)

[3:2] FS_G: Full-scale selection
      00 = ±250 dps
      01 = ±500 dps
      10 = ±1000 dps
      11 = ±2000 dps

[1] FS_125: 125 dps full-scale (overrides FS_G if set)
[0] (unused)
```

### Initialization Sequence
```rust
const LSM6DSOX_ADDR: u8 = 0x6A;
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;

// 1. Read and verify WHO_AM_I (should be 0x6C)
let mut who = [0u8];
i2c.write_read(LSM6DSOX_ADDR, &[WHO_AM_I], &mut who).await?;
assert_eq!(who[0], 0x6C);

// 2. Software reset
i2c.write(LSM6DSOX_ADDR, &[CTRL3_C, 0x01]).await?;
Timer::after_millis(10).await;

// 3. Configure accelerometer: 104 Hz, ±16g
// ODR=0100, FS=01 (±16g) -> 0b0100_0100 = 0x44
i2c.write(LSM6DSOX_ADDR, &[CTRL1_XL, 0x44]).await?;

// 4. Configure gyroscope: 104 Hz, ±2000 dps
// ODR=0100, FS=11 (±2000 dps) -> 0b0100_1100 = 0x4C
i2c.write(LSM6DSOX_ADDR, &[CTRL2_G, 0x4C]).await?;
```

### Reading Data
```rust
const OUTX_L_G: u8 = 0x22;

// Read 12 bytes: gyro (6) + accel (6)
let mut data = [0u8; 12];
i2c.write_read(LSM6DSOX_ADDR, &[OUTX_L_G], &mut data).await?;

// Gyroscope (signed 16-bit, dps)
let gx = i16::from_le_bytes([data[0], data[1]]);
let gy = i16::from_le_bytes([data[2], data[3]]);
let gz = i16::from_le_bytes([data[4], data[5]]);

// Accelerometer (signed 16-bit, g)
let ax = i16::from_le_bytes([data[6], data[7]]);
let ay = i16::from_le_bytes([data[8], data[9]]);
let az = i16::from_le_bytes([data[10], data[11]]);

// Convert to physical units
// For ±16g: sensitivity = 0.488 mg/LSB
// For ±2000 dps: sensitivity = 70 mdps/LSB
```

---

## LIS3MDL - 3-Axis Magnetometer

**Datasheet**: [ST LIS3MDL](https://www.st.com/resource/en/datasheet/lis3mdl.pdf)

### Specifications
- Range: ±4/±8/±12/±16 gauss
- ODR: up to 1000 Hz (ultra-high performance)
- Power: 270 µA @ 80 Hz

### I2C Address
- **0x1C**: SA1/SDO pin connected to GND ← **Our configuration**
- **0x1E**: SA1/SDO pin connected to VDDIO

### Key Registers

| Register | Address | Description |
|----------|---------|-------------|
| WHO_AM_I | 0x0F | Device ID (reads 0x3D) |
| CTRL_REG1 | 0x20 | Control register 1 (ODR, mode) |
| CTRL_REG2 | 0x21 | Control register 2 (full-scale) |
| CTRL_REG3 | 0x22 | Control register 3 (power mode) |
| CTRL_REG4 | 0x23 | Control register 4 (Z-axis mode) |
| CTRL_REG5 | 0x24 | Control register 5 (block update) |
| STATUS_REG | 0x27 | Data status |
| OUT_X_L | 0x28 | X output low byte |
| OUT_X_H | 0x29 | X output high byte |
| OUT_Y_L | 0x2A | Y output low byte |
| OUT_Y_H | 0x2B | Y output high byte |
| OUT_Z_L | 0x2C | Z output low byte |
| OUT_Z_H | 0x2D | Z output high byte |
| TEMP_OUT_L | 0x2E | Temperature low byte |
| TEMP_OUT_H | 0x2F | Temperature high byte |

### CTRL_REG1 (ODR and XY Performance)
```
[7] TEMP_EN: Temperature sensor enable
[6:5] OM: X/Y-axis operative mode
      00 = Low-power
      01 = Medium-performance
      10 = High-performance
      11 = Ultra-high performance

[4:2] DO: Output data rate
      000 = 0.625 Hz
      001 = 1.25 Hz
      010 = 2.5 Hz
      011 = 5 Hz
      100 = 10 Hz
      101 = 20 Hz
      110 = 40 Hz
      111 = 80 Hz

[1] FAST_ODR: Enable rates >80 Hz
[0] ST: Self-test enable
```

### CTRL_REG2 (Full-Scale Selection)
```
[6:5] FS: Full-scale configuration
      00 = ±4 gauss
      01 = ±8 gauss
      10 = ±12 gauss
      11 = ±16 gauss
```

### CTRL_REG3 (Operating Mode)
```
[1:0] MD: Operating mode
      00 = Continuous-conversion
      01 = Single-conversion
      10 = Power-down (default)
      11 = Power-down
```

### Initialization Sequence
```rust
const LIS3MDL_ADDR: u8 = 0x1C;
const WHO_AM_I: u8 = 0x0F;
const CTRL_REG1: u8 = 0x20;
const CTRL_REG2: u8 = 0x21;
const CTRL_REG3: u8 = 0x22;
const CTRL_REG4: u8 = 0x23;

// 1. Read and verify WHO_AM_I (should be 0x3D)
let mut who = [0u8];
i2c.write_read(LIS3MDL_ADDR, &[WHO_AM_I], &mut who).await?;
assert_eq!(who[0], 0x3D);

// 2. Configure CTRL_REG1: temp enable, ultra-high perf XY, 80 Hz
// TEMP_EN=1, OM=11, DO=111 -> 0b1111_1100 = 0xFC
i2c.write(LIS3MDL_ADDR, &[CTRL_REG1, 0xFC]).await?;

// 3. Configure CTRL_REG2: ±4 gauss (default, 0x00)
i2c.write(LIS3MDL_ADDR, &[CTRL_REG2, 0x00]).await?;

// 4. Configure CTRL_REG3: continuous mode
i2c.write(LIS3MDL_ADDR, &[CTRL_REG3, 0x00]).await?;

// 5. Configure CTRL_REG4: ultra-high performance Z-axis
// OMZ=11 -> 0b0000_1100 = 0x0C
i2c.write(LIS3MDL_ADDR, &[CTRL_REG4, 0x0C]).await?;
```

### Reading Data
```rust
const OUT_X_L: u8 = 0x28;

// Read 6 bytes (can also read with auto-increment: 0x28 | 0x80)
let mut data = [0u8; 6];
i2c.write_read(LIS3MDL_ADDR, &[OUT_X_L | 0x80], &mut data).await?;

// Magnetometer (signed 16-bit, gauss)
let mx = i16::from_le_bytes([data[0], data[1]]);
let my = i16::from_le_bytes([data[2], data[3]]);
let mz = i16::from_le_bytes([data[4], data[5]]);

// Convert to physical units
// For ±4 gauss: sensitivity = 6842 LSB/gauss
```

---

## Embassy I2C Setup for RP2040

```rust
use embassy_rp::i2c::{I2c, Config, InterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{I2C0, I2C1};

bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<I2C0>;
    I2C1_IRQ => InterruptHandler<I2C1>;
});

// I2C0 for IMU (LSM6DSOX + LIS3MDL): SDA=GP28, SCL=GP29
let mut config = Config::default();
config.frequency = 400_000; // 400 kHz Fast mode
let i2c0 = I2c::new_async(p.I2C0, p.PIN_29, p.PIN_28, Irqs, config);

// I2C1 for Barometer (BMP390): SDA=GP2, SCL=GP3
let i2c1 = I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, config);
```

**Note**: Embassy I2C::new_async takes pins as (SCL, SDA) not (SDA, SCL).

---

## Sensitivity and Conversion

### BMP390
Requires calibration coefficients from registers 0x31-0x45. See datasheet section 8.4 for compensation formulas.

### LSM6DSOX

| Accel Range | Sensitivity |
|-------------|-------------|
| ±2 g | 0.061 mg/LSB |
| ±4 g | 0.122 mg/LSB |
| ±8 g | 0.244 mg/LSB |
| ±16 g | 0.488 mg/LSB |

| Gyro Range | Sensitivity |
|------------|-------------|
| ±125 dps | 4.375 mdps/LSB |
| ±250 dps | 8.75 mdps/LSB |
| ±500 dps | 17.5 mdps/LSB |
| ±1000 dps | 35 mdps/LSB |
| ±2000 dps | 70 mdps/LSB |

### LIS3MDL

| Mag Range | Sensitivity |
|-----------|-------------|
| ±4 gauss | 6842 LSB/gauss |
| ±8 gauss | 3421 LSB/gauss |
| ±12 gauss | 2281 LSB/gauss |
| ±16 gauss | 1711 LSB/gauss |

---

## Troubleshooting

### Sensor Not Responding
1. Check I2C address (measure SDO/SA0/SA1 pin voltage)
2. Verify pull-up resistors on SDA/SCL (2.2k-10k to 3.3V)
3. Check wiring: SDA to SDA, SCL to SCL
4. Reduce I2C frequency to 100kHz for debugging

### Wrong WHO_AM_I Value
- 0xFF: No device responding (check address, wiring)
- 0x00: Device in reset or powered off
- Other: Wrong device or address conflict

### No Data Updates
- Check power mode (devices start in power-down)
- Verify ODR is set (output data rate > 0)
- Check STATUS register for data ready flags

### I2C Reads Hanging
If sensor reads hang after successful initialization:
1. **Test sensors individually** - add debug output between each sensor read to identify which one hangs
2. **Check for I2C bus conflicts** - multiple sensors on same bus can cause issues if one holds SDA/SCL low
3. **Add small delays between operations** - some sensors need settling time between init and first read
4. **Verify async I2C trait is in scope** - must `use embedded_hal_async::i2c::I2c as I2cTrait;` in each module

### Debugging Pattern
When sensor reads aren't working, use this progressive debugging approach:
```rust
// Add debug output before/after each operation
let _ = class.write_packet(b"SENSOR_NAME...\r\n").await;
if sensor::read(&mut i2c, &mut data).await.is_err() {
    let _ = class.write_packet(b"SENSOR_NAME ERR\r\n").await;
}
let _ = class.write_packet(b"SENSOR_NAME DONE\r\n").await;
```

This reveals exactly which operation hangs - invaluable for I2C debugging.

---

## Working Code Reference

The current pico_logger implementation in `pico_logger/src/sensors/` provides working drivers for all three sensors. Key patterns:

1. **Each sensor in its own module** with `init()` and `read()` functions
2. **I2C trait imported in each module**: `use embedded_hal_async::i2c::I2c as I2cTrait;`
3. **Error handling via Result** - init returns error if WHO_AM_I doesn't match
4. **Shared SensorData struct** passed by mutable reference to reads
