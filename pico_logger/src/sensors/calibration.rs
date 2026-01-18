/// IMU Calibration Module
///
/// Provides functions to calibrate accelerometer and gyroscope bias offsets.
/// Calibration should be performed with the device stationary and level.

use embassy_rp::i2c::{I2c, Async};
use embassy_rp::peripherals::I2C0;
use embassy_time::Timer;
use super::types::{SensorData, ImuCalibration};
use super::lsm6dsox;

/// Number of samples to average during calibration
const CALIBRATION_SAMPLES: u32 = 100;

/// Delay between calibration samples in milliseconds
const CALIBRATION_DELAY_MS: u64 = 10;

/// Run IMU calibration procedure
///
/// Device must be stationary and level (Z axis pointing up) during calibration.
/// Takes ~1 second to complete (100 samples at 10ms intervals).
///
/// Returns calibration data with computed bias offsets.
pub async fn calibrate_imu(i2c: &mut I2c<'static, I2C0, Async>) -> Result<ImuCalibration, &'static str> {
    let mut accel_sum: [i32; 3] = [0, 0, 0];
    let mut gyro_sum: [i32; 3] = [0, 0, 0];
    let mut data = SensorData::default();

    // Collect samples
    for _ in 0..CALIBRATION_SAMPLES {
        if lsm6dsox::read(i2c, &mut data).await.is_err() {
            return Err("IMU read failed during calibration");
        }

        accel_sum[0] += data.accel_x as i32;
        accel_sum[1] += data.accel_y as i32;
        accel_sum[2] += data.accel_z as i32;

        gyro_sum[0] += data.gyro_x as i32;
        gyro_sum[1] += data.gyro_y as i32;
        gyro_sum[2] += data.gyro_z as i32;

        Timer::after_millis(CALIBRATION_DELAY_MS).await;
    }

    // Compute averages
    let n = CALIBRATION_SAMPLES as i32;
    let accel_avg_x = (accel_sum[0] / n) as i16;
    let accel_avg_y = (accel_sum[1] / n) as i16;
    let accel_avg_z = (accel_sum[2] / n) as i16;

    let gyro_avg_x = (gyro_sum[0] / n) as i16;
    let gyro_avg_y = (gyro_sum[1] / n) as i16;
    let gyro_avg_z = (gyro_sum[2] / n) as i16;

    // For accelerometer: X and Y should be 0, Z should be +1g (or -1g if upside down)
    // We compute bias as the offset from expected values
    // Assuming device is level with Z pointing up, expected: (0, 0, +2049)
    let calib = ImuCalibration {
        accel_bias_x: accel_avg_x,      // X should read 0
        accel_bias_y: accel_avg_y,      // Y should read 0
        accel_bias_z: accel_avg_z - ImuCalibration::ONE_G_LSB,  // Z should read +1g
        gyro_bias_x: gyro_avg_x,        // Gyro should read 0 when stationary
        gyro_bias_y: gyro_avg_y,
        gyro_bias_z: gyro_avg_z,
    };

    Ok(calib)
}

/// Quick gyro-only calibration (faster, for when only gyro calibration is needed)
/// Takes ~500ms (50 samples at 10ms intervals)
pub async fn calibrate_gyro_only(i2c: &mut I2c<'static, I2C0, Async>) -> Result<(i16, i16, i16), &'static str> {
    let mut gyro_sum: [i32; 3] = [0, 0, 0];
    let mut data = SensorData::default();
    const SAMPLES: u32 = 50;

    for _ in 0..SAMPLES {
        if lsm6dsox::read(i2c, &mut data).await.is_err() {
            return Err("IMU read failed during gyro calibration");
        }

        gyro_sum[0] += data.gyro_x as i32;
        gyro_sum[1] += data.gyro_y as i32;
        gyro_sum[2] += data.gyro_z as i32;

        Timer::after_millis(CALIBRATION_DELAY_MS).await;
    }

    let n = SAMPLES as i32;
    Ok((
        (gyro_sum[0] / n) as i16,
        (gyro_sum[1] / n) as i16,
        (gyro_sum[2] / n) as i16,
    ))
}
