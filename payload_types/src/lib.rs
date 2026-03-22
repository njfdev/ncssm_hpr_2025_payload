#![no_std]

use serde::{Serialize, Deserialize};

/// Unit conversion constants matching pico_logger sensor configuration.
pub mod units {
    /// Accelerometer sensitivity: 0.488 mg/LSB at +/-16g (LSM6DSOX)
    pub const ACCEL_MG_PER_LSB: f32 = 0.488;
    /// Accelerometer sensitivity in g/LSB
    pub const ACCEL_G_PER_LSB: f32 = 0.000488;

    /// Gyroscope sensitivity: 70 mdps/LSB at +/-2000 dps (LSM6DSOX)
    pub const GYRO_MDPS_PER_LSB: f32 = 70.0;
    /// Gyroscope sensitivity in dps/LSB
    pub const GYRO_DPS_PER_LSB: f32 = 0.070;

    /// Magnetometer sensitivity: 6842 LSB/gauss = 68.42 LSB/uT at +/-4 gauss (LIS3MDL)
    pub const MAG_LSB_PER_UT: f32 = 68.42;

    /// BMP390 temperature: stored as centidegrees (divide by 100 for Celsius)
    pub const TEMP_CENTIDEG_DIVISOR: f32 = 100.0;

    /// BMP390 pressure: stored as Pa (divide by 100 for hPa/mbar)
    pub const PRESSURE_PA_TO_HPA: f32 = 100.0;

    /// Expected 1g raw value at +/-16g scale
    pub const ONE_G_LSB: i16 = 2049;
}

/// Compact telemetry packet for radio downlink.
///
/// Fields match `SensorData` from pico_logger, plus counter and timestamp.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct TelemetryPacket {
    pub counter: u32,
    pub timestamp_ms: u32,
    // BMP390
    pub pressure_pa: u32,
    pub temp_cdeg: u32,
    // LSM6DSOX
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    // LIS3MDL
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
    // ENS160
    pub aqi: u8,
    pub tvoc_ppb: u16,
    pub eco2_ppm: u16,
    // Orientation (integrated gyro, degrees)
    pub roll_cdeg: i16,    // roll * 100
    pub pitch_cdeg: i16,   // pitch * 100
    pub yaw_cdeg: i16,     // yaw * 100
    // GPS (UBlox)
    pub gps_lat_e7: i32,       // latitude * 1e7 (degrees, negative = South)
    pub gps_lon_e7: i32,       // longitude * 1e7 (degrees, negative = West)
    pub gps_alt_mm: i32,       // altitude in millimeters above MSL
    pub gps_fix: u8,           // 0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    pub gps_sats: u8,          // number of satellites used
    pub gps_hdop_e2: u16,      // HDOP * 100
    pub gps_speed_cms: u16,    // ground speed in cm/s
    pub gps_course_e2: u16,    // course over ground in degrees * 100
}

/// Maximum encoded packet size (raw struct + COBS overhead + sentinel).
/// postcard varint-encodes fields, so actual size varies, but 96 bytes is safe.
pub const MAX_ENCODED_SIZE: usize = 104;

impl TelemetryPacket {
    /// Encode this packet into a COBS-framed postcard buffer.
    ///
    /// Returns the number of bytes written to `buf`, including the trailing 0x00 sentinel.
    /// `buf` must be at least `MAX_ENCODED_SIZE` bytes.
    pub fn encode_cobs<'a>(&self, buf: &'a mut [u8]) -> Result<&'a [u8], postcard::Error> {
        postcard::to_slice_cobs(self, buf).map(|s| &*s)
    }

    /// Decode a COBS-framed postcard buffer into a TelemetryPacket.
    ///
    /// `buf` must include the trailing 0x00 sentinel. The buffer is modified in-place
    /// during decoding (COBS requirement).
    pub fn decode_cobs(buf: &mut [u8]) -> Result<Self, postcard::Error> {
        postcard::from_bytes_cobs(buf)
    }

    // --- Converted values ---

    pub fn pressure_hpa(&self) -> f32 {
        self.pressure_pa as f32 / units::PRESSURE_PA_TO_HPA
    }

    pub fn temp_celsius(&self) -> f32 {
        self.temp_cdeg as f32 / units::TEMP_CENTIDEG_DIVISOR
    }

    pub fn accel_x_g(&self) -> f32 {
        self.accel_x as f32 * units::ACCEL_G_PER_LSB
    }

    pub fn accel_y_g(&self) -> f32 {
        self.accel_y as f32 * units::ACCEL_G_PER_LSB
    }

    pub fn accel_z_g(&self) -> f32 {
        self.accel_z as f32 * units::ACCEL_G_PER_LSB
    }

    pub fn gyro_x_dps(&self) -> f32 {
        self.gyro_x as f32 * units::GYRO_DPS_PER_LSB
    }

    pub fn gyro_y_dps(&self) -> f32 {
        self.gyro_y as f32 * units::GYRO_DPS_PER_LSB
    }

    pub fn gyro_z_dps(&self) -> f32 {
        self.gyro_z as f32 * units::GYRO_DPS_PER_LSB
    }

    pub fn mag_x_ut(&self) -> f32 {
        self.mag_x as f32 / units::MAG_LSB_PER_UT
    }

    pub fn mag_y_ut(&self) -> f32 {
        self.mag_y as f32 / units::MAG_LSB_PER_UT
    }

    pub fn mag_z_ut(&self) -> f32 {
        self.mag_z as f32 / units::MAG_LSB_PER_UT
    }

    pub fn roll_deg(&self) -> f32 {
        self.roll_cdeg as f32 / 100.0
    }

    pub fn pitch_deg(&self) -> f32 {
        self.pitch_cdeg as f32 / 100.0
    }

    pub fn yaw_deg(&self) -> f32 {
        self.yaw_cdeg as f32 / 100.0
    }

    pub fn gps_lat_deg(&self) -> f64 {
        self.gps_lat_e7 as f64 / 1e7
    }

    pub fn gps_lon_deg(&self) -> f64 {
        self.gps_lon_e7 as f64 / 1e7
    }

    pub fn gps_alt_m(&self) -> f32 {
        self.gps_alt_mm as f32 / 1000.0
    }

    pub fn gps_hdop(&self) -> f32 {
        self.gps_hdop_e2 as f32 / 100.0
    }

    pub fn gps_speed_ms(&self) -> f32 {
        self.gps_speed_cms as f32 / 100.0
    }

    pub fn gps_course_deg(&self) -> f32 {
        self.gps_course_e2 as f32 / 100.0
    }

    /// Format timestamp_ms as HH:MM:SS.mmm
    pub fn timestamp_str<'a>(&self, buf: &'a mut [u8; 12]) -> &'a str {
        let ms = self.timestamp_ms;
        let total_s = ms / 1000;
        let millis = ms % 1000;
        let hours = total_s / 3600;
        let minutes = (total_s % 3600) / 60;
        let seconds = total_s % 60;

        // Format: HH:MM:SS.mmm
        buf[0] = b'0' + (hours / 10 % 10) as u8;
        buf[1] = b'0' + (hours % 10) as u8;
        buf[2] = b':';
        buf[3] = b'0' + (minutes / 10) as u8;
        buf[4] = b'0' + (minutes % 10) as u8;
        buf[5] = b':';
        buf[6] = b'0' + (seconds / 10) as u8;
        buf[7] = b'0' + (seconds % 10) as u8;
        buf[8] = b'.';
        buf[9] = b'0' + (millis / 100 % 10) as u8;
        buf[10] = b'0' + (millis / 10 % 10) as u8;
        buf[11] = b'0' + (millis % 10) as u8;

        // Safety: all bytes are ASCII
        unsafe { core::str::from_utf8_unchecked(&buf[..12]) }
    }
}

#[cfg(feature = "std")]
mod std_impl {
    extern crate std;
    use std::fmt;
    use super::TelemetryPacket;

    impl fmt::Display for TelemetryPacket {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let mut ts_buf = [0u8; 12];
            let ts = self.timestamp_str(&mut ts_buf);

            write!(
                f,
                "[{}] #{:<5} | P: {:.2} hPa  T: {:.1}\u{00b0}C | \
                 Accel: {:.3} {:.3} {:.3} g | \
                 Gyro: {:.1} {:.1} {:.1} dps | \
                 Mag: {:.1} {:.1} {:.1} \u{00b5}T | \
                 AQI: {} TVOC: {} CO2: {} | \
                 GPS: fix={} sats={} {:.6},{:.6} {:.1}m",
                ts,
                self.counter,
                self.pressure_hpa(),
                self.temp_celsius(),
                self.accel_x_g(),
                self.accel_y_g(),
                self.accel_z_g(),
                self.gyro_x_dps(),
                self.gyro_y_dps(),
                self.gyro_z_dps(),
                self.mag_x_ut(),
                self.mag_y_ut(),
                self.mag_z_ut(),
                self.aqi,
                self.tvoc_ppb,
                self.eco2_ppm,
                self.gps_fix,
                self.gps_sats,
                self.gps_lat_deg(),
                self.gps_lon_deg(),
                self.gps_alt_m(),
            )
        }
    }
}
