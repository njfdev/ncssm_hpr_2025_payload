/// Telemetry formatting utilities

use core::fmt::Write;
use heapless::String;
use crate::sensors::SensorData;

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
