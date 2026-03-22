/// Sensor drivers for the Pico Logger
///
/// This module contains drivers for:
/// - BMP390: Barometric pressure sensor (I2C1)
/// - LSM6DSOX: 6-axis IMU - accelerometer + gyroscope (I2C0)
/// - LIS3MDL: 3-axis magnetometer (I2C0)
/// - ENS160: MOX gas sensor - AQI, TVOC, eCO2 (I2C0)
/// - GPS: UBlox NMEA GPS receiver (UART1 RX)

pub mod types;
pub mod bmp390;
pub mod lsm6dsox;
pub mod lis3mdl;
pub mod ens160;
pub mod calibration;
pub mod orientation;
pub mod gps;

pub use types::{SensorData, ImuCalibration, Orientation, LogEntry};
