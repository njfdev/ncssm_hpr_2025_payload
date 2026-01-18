/// Telemetry formatting utilities

use core::fmt::Write;
use heapless::String;
use crate::sensors::{SensorData, ImuCalibration, Orientation};

/// Format sensor status message
pub fn format_status(bmp390: bool, lsm6dsox: bool, lis3mdl: bool) -> String<128> {
    let mut s = String::new();
    let _ = write!(s, "BMP390:{} LSM6DSOX:{} LIS3MDL:{}\r\nOK\r\n",
        if bmp390 { "OK" } else { "ERR" },
        if lsm6dsox { "OK" } else { "ERR" },
        if lis3mdl { "OK" } else { "ERR" },
    );
    s
}

/// Format telemetry data packet - uses short format to fit in 64-byte USB packet
pub fn format_telemetry(counter: u32, data: &SensorData) -> String<64> {
    let mut s = String::new();
    // Compact format: #N,P,T,ax,ay,az,gx,gy,gz,mx,my,mz
    let _ = write!(s,
        "#{},{},{},{},{},{},{},{},{},{},{},{}\r\n",
        counter,
        data.pressure_raw,
        data.temp_raw,
        data.accel_x, data.accel_y, data.accel_z,
        data.gyro_x, data.gyro_y, data.gyro_z,
        data.mag_x, data.mag_y, data.mag_z,
    );
    s
}

/// Format calibration data message
pub fn format_calibration(calib: &ImuCalibration) -> String<128> {
    let mut s = String::new();
    let _ = write!(s,
        "CALIB:accel={},{},{} gyro={},{},{}\r\n",
        calib.accel_bias_x, calib.accel_bias_y, calib.accel_bias_z,
        calib.gyro_bias_x, calib.gyro_bias_y, calib.gyro_bias_z,
    );
    s
}

/// Format extended telemetry with orientation and linear acceleration
/// Format: $counter,P,T,roll,pitch,yaw,lin_x,lin_y,lin_z
pub fn format_telemetry_extended(
    counter: u32,
    data: &SensorData,
    orientation: &Orientation,
    calib: &ImuCalibration,
) -> String<96> {
    let mut s = String::new();
    // Use $ prefix to distinguish from basic telemetry
    let lin_x = data.linear_accel_x(calib);
    let lin_y = data.linear_accel_y(calib);
    let lin_z = data.linear_accel_z(calib);
    let _ = write!(s,
        "${},{},{},{:.1},{:.1},{:.1},{:.3},{:.3},{:.3}\r\n",
        counter,
        data.pressure_raw,
        data.temp_raw,
        orientation.roll, orientation.pitch, orientation.yaw,
        lin_x, lin_y, lin_z,
    );
    s
}
