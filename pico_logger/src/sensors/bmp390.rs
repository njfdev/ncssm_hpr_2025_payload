/// BMP390 Barometric Pressure Sensor Driver
///
/// Connected to I2C1: SDA=GP2, SCL=GP3
/// Address: 0x77
///
/// This driver implements proper temperature and pressure compensation
/// using calibration coefficients from the sensor's NVM.

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
const CALIB_DATA: u8 = 0x31; // Start of calibration coefficients (21 bytes)

/// Calibration coefficients read from sensor NVM
/// These are the raw values before quantization
#[derive(Default, Clone, Copy)]
pub struct CalibrationData {
    // Temperature coefficients (raw)
    pub par_t1: u16,
    pub par_t2: u16,
    pub par_t3: i8,
    // Pressure coefficients (raw)
    pub par_p1: i16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i8,
    pub par_p5: u16,
    pub par_p6: u16,
    pub par_p7: i8,
    pub par_p8: i8,
    pub par_p9: i16,
    pub par_p10: i8,
    pub par_p11: i8,
}

/// Quantized calibration data for floating-point compensation
#[derive(Default, Clone, Copy)]
pub struct QuantizedCalibData {
    pub par_t1: f64,
    pub par_t2: f64,
    pub par_t3: f64,
    pub par_p1: f64,
    pub par_p2: f64,
    pub par_p3: f64,
    pub par_p4: f64,
    pub par_p5: f64,
    pub par_p6: f64,
    pub par_p7: f64,
    pub par_p8: f64,
    pub par_p9: f64,
    pub par_p10: f64,
    pub par_p11: f64,
    pub t_lin: f64, // Linearized temperature for pressure compensation
}

/// Global calibration data (initialized once)
static mut CALIB: Option<QuantizedCalibData> = None;

/// Parse calibration data from raw bytes
fn parse_calib_data(data: &[u8; 21]) -> CalibrationData {
    CalibrationData {
        par_t1: u16::from_le_bytes([data[0], data[1]]),
        par_t2: u16::from_le_bytes([data[2], data[3]]),
        par_t3: data[4] as i8,
        par_p1: i16::from_le_bytes([data[5], data[6]]),
        par_p2: i16::from_le_bytes([data[7], data[8]]),
        par_p3: data[9] as i8,
        par_p4: data[10] as i8,
        par_p5: u16::from_le_bytes([data[11], data[12]]),
        par_p6: u16::from_le_bytes([data[13], data[14]]),
        par_p7: data[15] as i8,
        par_p8: data[16] as i8,
        par_p9: i16::from_le_bytes([data[17], data[18]]),
        par_p10: data[19] as i8,
        par_p11: data[20] as i8,
    }
}

/// Quantize calibration coefficients for floating-point compensation
/// These scaling factors are from the Bosch BMP3 Sensor API
fn quantize_calib_data(raw: &CalibrationData) -> QuantizedCalibData {
    QuantizedCalibData {
        // Temperature coefficient scaling
        par_t1: (raw.par_t1 as f64) / 0.00390625,  // 2^-8
        par_t2: (raw.par_t2 as f64) / 1073741824.0, // 2^30
        par_t3: (raw.par_t3 as f64) / 281474976710656.0, // 2^48
        // Pressure coefficient scaling
        par_p1: ((raw.par_p1 as f64) - 16384.0) / 1048576.0, // (val - 2^14) / 2^20
        par_p2: ((raw.par_p2 as f64) - 16384.0) / 536870912.0, // (val - 2^14) / 2^29
        par_p3: (raw.par_p3 as f64) / 4294967296.0, // 2^32
        par_p4: (raw.par_p4 as f64) / 137438953472.0, // 2^37
        par_p5: (raw.par_p5 as f64) / 0.125, // 2^-3
        par_p6: (raw.par_p6 as f64) / 64.0, // 2^6
        par_p7: (raw.par_p7 as f64) / 256.0, // 2^8
        par_p8: (raw.par_p8 as f64) / 32768.0, // 2^15
        par_p9: (raw.par_p9 as f64) / 281474976710656.0, // 2^48
        par_p10: (raw.par_p10 as f64) / 281474976710656.0, // 2^48
        par_p11: (raw.par_p11 as f64) / 36893488147419103232.0, // 2^65
        t_lin: 0.0,
    }
}

/// Compensate raw temperature ADC value to Celsius
/// Returns temperature in degrees Celsius
pub fn compensate_temperature(uncomp_temp: u32, calib: &mut QuantizedCalibData) -> f64 {
    let partial_data1 = (uncomp_temp as f64) - calib.par_t1;
    let partial_data2 = partial_data1 * calib.par_t2;

    // Store linearized temperature for pressure compensation
    calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib.par_t3;

    // Clamp to valid range (-40 to 85°C)
    if calib.t_lin < -40.0 {
        -40.0
    } else if calib.t_lin > 85.0 {
        85.0
    } else {
        calib.t_lin
    }
}

/// Compensate raw pressure ADC value to Pascals
/// Must call compensate_temperature first to update t_lin
/// Returns pressure in Pascals
pub fn compensate_pressure(uncomp_press: u32, calib: &QuantizedCalibData) -> f64 {
    let t_lin = calib.t_lin;
    let t_lin2 = t_lin * t_lin;
    let t_lin3 = t_lin2 * t_lin;

    let partial_data1 = calib.par_p6 * t_lin;
    let partial_data2 = calib.par_p7 * t_lin2;
    let partial_data3 = calib.par_p8 * t_lin3;
    let partial_out1 = calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

    let partial_data1 = calib.par_p2 * t_lin;
    let partial_data2 = calib.par_p3 * t_lin2;
    let partial_data3 = calib.par_p4 * t_lin3;
    let partial_out2 = (uncomp_press as f64) * (calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

    let uncomp_press_f = uncomp_press as f64;
    let partial_data1 = uncomp_press_f * uncomp_press_f;
    let partial_data2 = calib.par_p9 + calib.par_p10 * t_lin;
    let partial_data3 = partial_data1 * partial_data2;
    let partial_data4 = partial_data3 + (uncomp_press_f * uncomp_press_f * uncomp_press_f) * calib.par_p11;

    let comp_press = partial_out1 + partial_out2 + partial_data4;

    // Clamp to valid range (30000 to 125000 Pa)
    if comp_press < 30000.0 {
        30000.0
    } else if comp_press > 125000.0 {
        125000.0
    } else {
        comp_press
    }
}

/// Initialize BMP390 barometer
/// Reads calibration coefficients and configures the sensor
pub async fn init(i2c: &mut I2c<'static, I2C1, Async>) -> Result<(), i2c::Error> {
    // Read chip ID
    let mut id = [0u8];
    i2c.write_read(ADDR, &[CHIP_ID], &mut id).await?;

    if id[0] != 0x60 {
        return Err(i2c::Error::Abort(i2c::AbortReason::NoAcknowledge));
    }

    // Read calibration data (21 bytes starting at 0x31)
    let mut calib_bytes = [0u8; 21];
    i2c.write_read(ADDR, &[CALIB_DATA], &mut calib_bytes).await?;

    // Parse and quantize calibration data
    let raw_calib = parse_calib_data(&calib_bytes);
    let quantized = quantize_calib_data(&raw_calib);

    // Store in global (safe because single-threaded init)
    unsafe {
        CALIB = Some(quantized);
    }

    // Configure oversampling: x8 pressure, x1 temp
    i2c.write(ADDR, &[OSR, 0b00000011]).await?;

    // Configure ODR: 50 Hz
    i2c.write(ADDR, &[ODR, 0x02]).await?;

    // Enable pressure and temp, normal mode
    i2c.write(ADDR, &[PWR_CTRL, 0b00110011]).await?;

    Ok(())
}

/// Read BMP390 pressure and temperature data with compensation
pub async fn read(i2c: &mut I2c<'static, I2C1, Async>, data: &mut SensorData) -> Result<(), i2c::Error> {
    let mut buf = [0u8; 6];
    i2c.write_read(ADDR, &[DATA], &mut buf).await?;

    // Pressure: 24-bit unsigned (XLSB, LSB, MSB)
    let uncomp_press = (buf[2] as u32) << 16 | (buf[1] as u32) << 8 | buf[0] as u32;

    // Temperature: 24-bit unsigned (XLSB, LSB, MSB)
    let uncomp_temp = (buf[5] as u32) << 16 | (buf[4] as u32) << 8 | buf[3] as u32;

    // Get calibration data and compensate
    unsafe {
        if let Some(ref mut calib) = *core::ptr::addr_of_mut!(CALIB) {
            // Must compensate temperature first (updates t_lin for pressure)
            let temp_c = compensate_temperature(uncomp_temp, calib);
            let press_pa = compensate_pressure(uncomp_press, calib);

            // Store compensated values (scaled for storage)
            // Temperature: store as centidegrees (25.5°C = 2550)
            data.temp_raw = (temp_c * 100.0) as u32;
            // Pressure: store as Pascals (already in Pa)
            data.pressure_raw = press_pa as u32;
        } else {
            // Fallback if not calibrated
            data.pressure_raw = uncomp_press;
            data.temp_raw = uncomp_temp;
        }
    }

    Ok(())
}
