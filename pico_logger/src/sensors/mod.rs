/// Sensor drivers for the Pico Logger
///
/// This module contains drivers for:
/// - BMP390: Barometric pressure sensor (I2C1)
/// - LSM6DSOX: 6-axis IMU - accelerometer + gyroscope (I2C0)
/// - LIS3MDL: 3-axis magnetometer (I2C0)

pub mod types;
pub mod bmp390;
pub mod lsm6dsox;
pub mod lis3mdl;
pub mod calibration;
pub mod orientation;

pub use types::{SensorData, ImuCalibration, Orientation, LogEntry};
