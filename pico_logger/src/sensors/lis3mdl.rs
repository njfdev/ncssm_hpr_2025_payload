/// LIS3MDL 3-axis Magnetometer Driver
///
/// Connected to I2C0: SDA=GP28, SCL=GP29
/// Address: 0x1C

use embassy_rp::i2c::{self, I2c, Async};
use embassy_rp::peripherals::I2C0;
use embedded_hal_async::i2c::I2c as I2cTrait;
use super::types::SensorData;

pub const ADDR: u8 = 0x1C;

// Register addresses
const WHO_AM_I: u8 = 0x0F;
const CTRL_REG1: u8 = 0x20;
const CTRL_REG2: u8 = 0x21;
const CTRL_REG3: u8 = 0x22;
const CTRL_REG4: u8 = 0x23;
const OUT_X_L: u8 = 0x28;

/// Initialize LIS3MDL magnetometer
/// Configures at 155 Hz (FAST_ODR), ±4 gauss, ultra-high performance
pub async fn init(i2c: &mut I2c<'static, I2C0, Async>) -> Result<(), i2c::Error> {
    // Read WHO_AM_I
    let mut who = [0u8];
    i2c.write_read(ADDR, &[WHO_AM_I], &mut who).await?;

    if who[0] != 0x3D {
        return Err(i2c::Error::Abort(i2c::AbortReason::NoAcknowledge));
    }

    // Configure CTRL_REG1: temp enable, ultra-high perf XY, FAST_ODR (155 Hz)
    // Bits: TEMP_EN=1, OM=11 (ultra-high), DO=111 (80Hz base), FAST_ODR=1
    i2c.write(ADDR, &[CTRL_REG1, 0xFE]).await?;

    // Configure CTRL_REG2: ±4 gauss
    i2c.write(ADDR, &[CTRL_REG2, 0x00]).await?;

    // Configure CTRL_REG3: continuous mode
    i2c.write(ADDR, &[CTRL_REG3, 0x00]).await?;

    // Configure CTRL_REG4: ultra-high performance Z-axis
    i2c.write(ADDR, &[CTRL_REG4, 0x0C]).await?;

    Ok(())
}

/// Read LIS3MDL magnetometer data
pub async fn read(i2c: &mut I2c<'static, I2C0, Async>, data: &mut SensorData) -> Result<(), i2c::Error> {
    // Read with auto-increment (0x80 bit)
    let mut buf = [0u8; 6];
    i2c.write_read(ADDR, &[OUT_X_L | 0x80], &mut buf).await?;

    data.mag_x = i16::from_le_bytes([buf[0], buf[1]]);
    data.mag_y = i16::from_le_bytes([buf[2], buf[3]]);
    data.mag_z = i16::from_le_bytes([buf[4], buf[5]]);

    Ok(())
}
