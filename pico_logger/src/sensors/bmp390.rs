/// BMP390 Barometric Pressure Sensor Driver
///
/// Connected to I2C1: SDA=GP2, SCL=GP3
/// Address: 0x77

use embassy_rp::i2c::{self, I2c, Async};
use embassy_rp::peripherals::I2C1;
use embedded_hal_async::i2c::I2c as I2cTrait;
use super::types::SensorData;

pub const ADDR: u8 = 0x77;

// Register addresses
const CHIP_ID: u8 = 0x00;
const DATA: u8 = 0x04;
const PWR_CTRL: u8 = 0x1B;
const OSR: u8 = 0x1C;
const ODR: u8 = 0x1D;

/// Initialize BMP390 barometer
/// Returns Ok(()) if successful, Err if sensor not found or misconfigured
pub async fn init(i2c: &mut I2c<'static, I2C1, Async>) -> Result<(), i2c::Error> {
    // Read chip ID
    let mut id = [0u8];
    i2c.write_read(ADDR, &[CHIP_ID], &mut id).await?;

    if id[0] != 0x60 {
        // Wrong chip ID - sensor not responding correctly
        return Err(i2c::Error::Abort(i2c::AbortReason::NoAcknowledge));
    }

    // Configure oversampling: x8 pressure, x1 temp
    i2c.write(ADDR, &[OSR, 0b00000011]).await?;

    // Configure ODR: 50 Hz
    i2c.write(ADDR, &[ODR, 0x02]).await?;

    // Enable pressure and temp, normal mode
    i2c.write(ADDR, &[PWR_CTRL, 0b00110011]).await?;

    Ok(())
}

/// Read BMP390 pressure and temperature data
pub async fn read(i2c: &mut I2c<'static, I2C1, Async>, data: &mut SensorData) -> Result<(), i2c::Error> {
    let mut buf = [0u8; 6];
    i2c.write_read(ADDR, &[DATA], &mut buf).await?;

    // Pressure: 24-bit unsigned (XLSB, LSB, MSB)
    data.pressure_raw = (buf[2] as u32) << 16 | (buf[1] as u32) << 8 | buf[0] as u32;

    // Temperature: 24-bit unsigned (XLSB, LSB, MSB)
    data.temp_raw = (buf[5] as u32) << 16 | (buf[4] as u32) << 8 | buf[3] as u32;

    Ok(())
}
