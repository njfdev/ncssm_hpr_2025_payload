/// LSM6DSOX 6-axis IMU Driver (Accelerometer + Gyroscope)
///
/// Connected to I2C0: SDA=GP28, SCL=GP29
/// Address: 0x6A

use embassy_rp::i2c::{self, I2c, Async};
use embassy_rp::peripherals::I2C0;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as I2cTrait;
use super::types::SensorData;

pub const ADDR: u8 = 0x6A;

// Register addresses
const WHO_AM_I: u8 = 0x0F;
const CTRL1_XL: u8 = 0x10;
const CTRL2_G: u8 = 0x11;
const CTRL3_C: u8 = 0x12;
const OUTX_L_G: u8 = 0x22;

/// Initialize LSM6DSOX IMU
/// Configures accelerometer at 416 Hz, ±16g and gyroscope at 416 Hz, ±2000 dps
pub async fn init(i2c: &mut I2c<'static, I2C0, Async>) -> Result<(), i2c::Error> {
    // Read WHO_AM_I
    let mut who = [0u8];
    i2c.write_read(ADDR, &[WHO_AM_I], &mut who).await?;

    if who[0] != 0x6C {
        return Err(i2c::Error::Abort(i2c::AbortReason::NoAcknowledge));
    }

    // Software reset
    i2c.write(ADDR, &[CTRL3_C, 0x01]).await?;
    Timer::after_millis(10).await;

    // Configure accelerometer: 416 Hz (0x60), ±16g (0x04)
    i2c.write(ADDR, &[CTRL1_XL, 0x64]).await?;

    // Configure gyroscope: 416 Hz (0x60), ±2000 dps (0x0C)
    i2c.write(ADDR, &[CTRL2_G, 0x6C]).await?;

    Ok(())
}

/// Read LSM6DSOX accelerometer and gyroscope data
pub async fn read(i2c: &mut I2c<'static, I2C0, Async>, data: &mut SensorData) -> Result<(), i2c::Error> {
    let mut buf = [0u8; 12];
    i2c.write_read(ADDR, &[OUTX_L_G], &mut buf).await?;

    // Gyroscope (registers 0x22-0x27)
    data.gyro_x = i16::from_le_bytes([buf[0], buf[1]]);
    data.gyro_y = i16::from_le_bytes([buf[2], buf[3]]);
    data.gyro_z = i16::from_le_bytes([buf[4], buf[5]]);

    // Accelerometer (registers 0x28-0x2D)
    data.accel_x = i16::from_le_bytes([buf[6], buf[7]]);
    data.accel_y = i16::from_le_bytes([buf[8], buf[9]]);
    data.accel_z = i16::from_le_bytes([buf[10], buf[11]]);

    Ok(())
}
