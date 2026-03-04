use std::time::Instant;

use mavlink::ardupilotmega::*;

pub struct TelemetryState {
    // GPS
    pub lat: f64,
    pub lon: f64,
    pub gps_alt: f64,
    pub fix_type: String,
    pub satellites: u8,
    pub hdop: f32,

    // Fused position
    pub fused_alt: f64,
    pub fused_relative_alt: f64,
    pub vz: f32,

    // Barometer
    pub pressure_hpa: f32,
    pub temperature_c: f32,
    pub baro_alt: f32,

    // IMU — Accelerometer
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,

    // IMU — Gyroscope
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,

    // IMU — Magnetometer
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,

    // System
    pub mav_type: String,
    pub system_status: String,
    pub connected: bool,
    pub msg_count: u64,
    pub last_heartbeat: Option<Instant>,
    pub msg_rate: f32,

    // Rate tracking
    rate_window_start: Instant,
    rate_window_count: u64,
}

impl TelemetryState {
    pub fn new() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            gps_alt: 0.0,
            fix_type: "No Fix".into(),
            satellites: 0,
            hdop: 0.0,

            fused_alt: 0.0,
            fused_relative_alt: 0.0,
            vz: 0.0,

            pressure_hpa: 0.0,
            temperature_c: 0.0,
            baro_alt: 0.0,

            accel_x: 0.0,
            accel_y: 0.0,
            accel_z: 0.0,

            gyro_x: 0.0,
            gyro_y: 0.0,
            gyro_z: 0.0,

            mag_x: 0.0,
            mag_y: 0.0,
            mag_z: 0.0,

            mav_type: "Unknown".into(),
            system_status: "Unknown".into(),
            connected: false,
            msg_count: 0,
            last_heartbeat: None,
            msg_rate: 0.0,

            rate_window_start: Instant::now(),
            rate_window_count: 0,
        }
    }

    pub fn update(&mut self, msg: &MavMessage) {
        self.msg_count += 1;
        self.connected = true;
        self.update_rate();

        match msg {
            MavMessage::HEARTBEAT(data) => {
                self.last_heartbeat = Some(Instant::now());
                self.mav_type = format!("{:?}", data.mavtype);
                self.system_status = format!("{:?}", data.system_status);
            }

            MavMessage::GLOBAL_POSITION_INT(data) => {
                self.lat = data.lat as f64 / 1e7;
                self.lon = data.lon as f64 / 1e7;
                self.fused_alt = data.alt as f64 / 1000.0;
                self.fused_relative_alt = data.relative_alt as f64 / 1000.0;
                self.vz = data.vz as f32 / 100.0;
            }

            MavMessage::SCALED_IMU(data) => {
                // mG -> g
                self.accel_x = data.xacc as f32 / 1000.0;
                self.accel_y = data.yacc as f32 / 1000.0;
                self.accel_z = data.zacc as f32 / 1000.0;
                // mrad/s -> deg/s
                self.gyro_x = data.xgyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                self.gyro_y = data.ygyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                self.gyro_z = data.zgyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                // mgauss -> uT (1 mgauss = 0.1 uT)
                self.mag_x = data.xmag as f32 * 0.1;
                self.mag_y = data.ymag as f32 * 0.1;
                self.mag_z = data.zmag as f32 * 0.1;
            }

            MavMessage::SCALED_PRESSURE(data) => {
                self.pressure_hpa = data.press_abs;
                self.temperature_c = data.temperature as f32 / 100.0;
                // ISA barometric formula
                if data.press_abs > 0.0 {
                    self.baro_alt =
                        44330.0 * (1.0 - (data.press_abs / 1013.25_f32).powf(0.1903));
                }
            }

            MavMessage::GPS_RAW_INT(data) => {
                self.lat = data.lat as f64 / 1e7;
                self.lon = data.lon as f64 / 1e7;
                self.gps_alt = data.alt as f64 / 1000.0;
                self.satellites = data.satellites_visible;
                self.hdop = data.eph as f32 / 100.0;
                self.fix_type = match data.fix_type {
                    GpsFixType::GPS_FIX_TYPE_NO_GPS => "No GPS",
                    GpsFixType::GPS_FIX_TYPE_NO_FIX => "No Fix",
                    GpsFixType::GPS_FIX_TYPE_2D_FIX => "2D Fix",
                    GpsFixType::GPS_FIX_TYPE_3D_FIX => "3D Fix",
                    GpsFixType::GPS_FIX_TYPE_DGPS => "DGPS",
                    GpsFixType::GPS_FIX_TYPE_RTK_FLOAT => "RTK Float",
                    GpsFixType::GPS_FIX_TYPE_RTK_FIXED => "RTK Fixed",
                    GpsFixType::GPS_FIX_TYPE_STATIC => "Static",
                    GpsFixType::GPS_FIX_TYPE_PPP => "PPP",
                }
                .into();
            }

            _ => {}
        }
    }

    fn update_rate(&mut self) {
        self.rate_window_count += 1;
        let elapsed = self.rate_window_start.elapsed().as_secs_f32();
        if elapsed >= 1.0 {
            self.msg_rate = self.rate_window_count as f32 / elapsed;
            self.rate_window_start = Instant::now();
            self.rate_window_count = 0;
        }
    }

    pub fn heartbeat_age_secs(&self) -> Option<f32> {
        self.last_heartbeat.map(|t| t.elapsed().as_secs_f32())
    }
}
