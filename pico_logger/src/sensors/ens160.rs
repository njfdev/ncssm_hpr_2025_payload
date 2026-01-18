/// ENS160 Digital Metal-Oxide Multi-Gas Sensor Driver
///
/// Connected to I2C1: SDA=GP2, SCL=GP3 (same bus as BMP390)
/// Address: 0x53 (default)
///
/// Provides Air Quality Index (AQI), Total VOC (TVOC), and equivalent CO2 (eCO2) readings.

use embassy_rp::i2c::{self, I2c, Async};
use embassy_rp::peripherals::I2C1;
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as I2cTrait;
use super::types::SensorData;

pub const ADDR: u8 = 0x53;

// Register addresses
const PART_ID: u8 = 0x00;
const OPMODE: u8 = 0x10;
const CONFIG: u8 = 0x11;
const COMMAND: u8 = 0x12;
const TEMP_IN: u8 = 0x13;
const RH_IN: u8 = 0x15;
const DEVICE_STATUS: u8 = 0x20;
const DATA_AQI: u8 = 0x21;
const DATA_TVOC: u8 = 0x22;
const DATA_ECO2: u8 = 0x24;

// Operating modes
const OPMODE_DEEP_SLEEP: u8 = 0x00;
const OPMODE_IDLE: u8 = 0x01;
const OPMODE_STANDARD: u8 = 0x02;

// Commands
const COMMAND_NOP: u8 = 0x00;
const COMMAND_CLRGPR: u8 = 0xCC;

// Status flags
const STATUS_NEWDAT: u8 = 0x02;  // New data available
const STATUS_NEWGPR: u8 = 0x01;  // New GPR data available
const STATUS_VALIDITY_MASK: u8 = 0x0C;
const STATUS_VALIDITY_NORMAL: u8 = 0x00;

// Expected part ID
const EXPECTED_PART_ID: u16 = 0x0160;

/// Initialize ENS160 gas sensor
/// Resets the device and configures it for standard operation mode
pub async fn init(i2c: &mut I2c<'static, I2C1, Async>) -> Result<(), i2c::Error> {
    // Read part ID (2 bytes, little-endian)
    let mut id_buf = [0u8; 2];
    i2c.write_read(ADDR, &[PART_ID], &mut id_buf).await?;
    let part_id = u16::from_le_bytes(id_buf);

    if part_id != EXPECTED_PART_ID {
        return Err(i2c::Error::Abort(i2c::AbortReason::NoAcknowledge));
    }

    // Set to idle mode first (required for configuration)
    i2c.write(ADDR, &[OPMODE, OPMODE_IDLE]).await?;
    Timer::after_millis(10).await;

    // Clear GPR registers
    i2c.write(ADDR, &[COMMAND, COMMAND_CLRGPR]).await?;
    Timer::after_millis(10).await;

    // Set to standard operating mode (1 second measurement interval)
    i2c.write(ADDR, &[OPMODE, OPMODE_STANDARD]).await?;
    Timer::after_millis(50).await;

    Ok(())
}

/// Set ambient temperature for compensation
/// Temperature should be in centidegrees (25.5°C = 2550)
pub async fn set_temperature(i2c: &mut I2c<'static, I2C1, Async>, temp_centideg: u32) -> Result<(), i2c::Error> {
    // ENS160 expects temperature as: (T + 273.15) * 64
    // Input is in centidegrees, so: ((temp_centideg / 100) + 273.15) * 64
    // = (temp_centideg + 27315) * 64 / 100
    let temp_encoded = ((temp_centideg as u32 + 27315) * 64 / 100) as u16;
    let bytes = temp_encoded.to_le_bytes();
    i2c.write(ADDR, &[TEMP_IN, bytes[0], bytes[1]]).await?;
    Ok(())
}

/// Set ambient relative humidity for compensation
/// Humidity should be in percent * 100 (50.5% = 5050)
pub async fn set_humidity(i2c: &mut I2c<'static, I2C1, Async>, rh_centipercent: u16) -> Result<(), i2c::Error> {
    // ENS160 expects humidity as: RH * 512 / 100
    // Input is in centipercent, so: rh_centipercent * 512 / 10000
    let rh_encoded = (rh_centipercent as u32 * 512 / 10000) as u16;
    let bytes = rh_encoded.to_le_bytes();
    i2c.write(ADDR, &[RH_IN, bytes[0], bytes[1]]).await?;
    Ok(())
}

/// Check if new data is available
pub async fn data_ready(i2c: &mut I2c<'static, I2C1, Async>) -> Result<bool, i2c::Error> {
    let mut status = [0u8];
    i2c.write_read(ADDR, &[DEVICE_STATUS], &mut status).await?;
    Ok((status[0] & STATUS_NEWDAT) != 0)
}

/// Read ENS160 air quality data
/// Updates aqi, tvoc_ppb, and eco2_ppm fields in SensorData
pub async fn read(i2c: &mut I2c<'static, I2C1, Async>, data: &mut SensorData) -> Result<(), i2c::Error> {
    // Read status first
    let mut status = [0u8];
    i2c.write_read(ADDR, &[DEVICE_STATUS], &mut status).await?;

    // Check validity - only use data if status is normal operation
    let validity = (status[0] & STATUS_VALIDITY_MASK) >> 2;
    if validity > 1 {
        // 0 = normal, 1 = warm-up, 2 = initial start-up, 3 = invalid
        // During warm-up/start-up, return without updating (keep previous values)
        return Ok(());
    }

    // Read AQI (1 byte at 0x21)
    let mut aqi_buf = [0u8];
    i2c.write_read(ADDR, &[DATA_AQI], &mut aqi_buf).await?;
    data.aqi = aqi_buf[0];

    // Read TVOC (2 bytes, little-endian at 0x22)
    let mut tvoc_buf = [0u8; 2];
    i2c.write_read(ADDR, &[DATA_TVOC], &mut tvoc_buf).await?;
    data.tvoc_ppb = u16::from_le_bytes(tvoc_buf);

    // Read eCO2 (2 bytes, little-endian at 0x24)
    let mut eco2_buf = [0u8; 2];
    i2c.write_read(ADDR, &[DATA_ECO2], &mut eco2_buf).await?;
    data.eco2_ppm = u16::from_le_bytes(eco2_buf);

    Ok(())
}
