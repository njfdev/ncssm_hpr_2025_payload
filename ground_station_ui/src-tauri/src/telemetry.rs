use std::time::Instant;

use mavlink::ardupilotmega::*;
use mavlink::MavHeader;
use serde::{Deserialize, Serialize};

/// MAVLink v2 overhead: 10 header + 2 CRC = 12 bytes per message
const MAVLINK_V2_OVERHEAD: usize = 12;

/// Live telemetry state updated by the receiver thread.
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

    // Bandwidth
    pub bytes_per_sec: f32,
    pub max_bytes_per_sec: f32,

    // Packet loss
    pub packet_loss_pct: f32,
    pub total_expected: u64,
    pub total_lost: u64,

    // Internal rate tracking
    rate_window_start: Instant,
    rate_window_count: u64,
    rate_window_bytes: u64,

    // Sequence tracking
    last_seq: Option<u8>,
    seq_initialized: bool,

    // Session timing
    session_start: Instant,
}

/// Serializable snapshot emitted to the frontend at 10 Hz.
#[derive(Clone, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub timestamp_ms: u64,

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
    pub heartbeat_age_secs: Option<f32>,
    pub msg_rate: f32,
    pub bytes_per_sec: f32,
    pub max_bytes_per_sec: f32,
    pub bandwidth_pct: f32,
    pub packet_loss_pct: f32,
}

impl TelemetryState {
    pub fn new(max_baud: u32) -> Self {
        let max_bytes = if max_baud > 0 {
            max_baud as f32 / 10.0
        } else {
            0.0
        };

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

            bytes_per_sec: 0.0,
            max_bytes_per_sec: max_bytes,

            packet_loss_pct: 0.0,
            total_expected: 0,
            total_lost: 0,

            rate_window_start: Instant::now(),
            rate_window_count: 0,
            rate_window_bytes: 0,

            last_seq: None,
            seq_initialized: false,

            session_start: Instant::now(),
        }
    }

    pub fn update(&mut self, header: &MavHeader, msg: &MavMessage) {
        self.msg_count += 1;
        self.connected = true;

        let msg_bytes = payload_size(msg) + MAVLINK_V2_OVERHEAD;
        self.update_rate(msg_bytes as u64);
        self.update_packet_loss(header.sequence);

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
                self.accel_x = data.xacc as f32 / 1000.0;
                self.accel_y = data.yacc as f32 / 1000.0;
                self.accel_z = data.zacc as f32 / 1000.0;
                self.gyro_x = data.xgyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                self.gyro_y = data.ygyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                self.gyro_z = data.zgyro as f32 / 1000.0 * (180.0 / std::f32::consts::PI);
                self.mag_x = data.xmag as f32 * 0.1;
                self.mag_y = data.ymag as f32 * 0.1;
                self.mag_z = data.zmag as f32 * 0.1;
            }

            MavMessage::SCALED_PRESSURE(data) => {
                self.pressure_hpa = data.press_abs;
                self.temperature_c = data.temperature as f32 / 100.0;
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

    /// Create a serializable snapshot of the current state.
    pub fn snapshot(&self) -> TelemetrySnapshot {
        TelemetrySnapshot {
            timestamp_ms: self.session_start.elapsed().as_millis() as u64,

            lat: self.lat,
            lon: self.lon,
            gps_alt: self.gps_alt,
            fix_type: self.fix_type.clone(),
            satellites: self.satellites,
            hdop: self.hdop,

            fused_alt: self.fused_alt,
            fused_relative_alt: self.fused_relative_alt,
            vz: self.vz,

            pressure_hpa: self.pressure_hpa,
            temperature_c: self.temperature_c,
            baro_alt: self.baro_alt,

            accel_x: self.accel_x,
            accel_y: self.accel_y,
            accel_z: self.accel_z,

            gyro_x: self.gyro_x,
            gyro_y: self.gyro_y,
            gyro_z: self.gyro_z,

            mag_x: self.mag_x,
            mag_y: self.mag_y,
            mag_z: self.mag_z,

            mav_type: self.mav_type.clone(),
            system_status: self.system_status.clone(),
            connected: self.connected,
            msg_count: self.msg_count,
            heartbeat_age_secs: self.last_heartbeat.map(|t| t.elapsed().as_secs_f32()),
            msg_rate: self.msg_rate,
            bytes_per_sec: self.bytes_per_sec,
            max_bytes_per_sec: self.max_bytes_per_sec,
            bandwidth_pct: self.bandwidth_pct(),
            packet_loss_pct: self.packet_loss_pct,
        }
    }

    fn update_rate(&mut self, bytes: u64) {
        self.rate_window_count += 1;
        self.rate_window_bytes += bytes;
        let elapsed = self.rate_window_start.elapsed().as_secs_f32();
        if elapsed >= 1.0 {
            self.msg_rate = self.rate_window_count as f32 / elapsed;
            self.bytes_per_sec = self.rate_window_bytes as f32 / elapsed;
            self.rate_window_start = Instant::now();
            self.rate_window_count = 0;
            self.rate_window_bytes = 0;
        }
    }

    fn update_packet_loss(&mut self, seq: u8) {
        if !self.seq_initialized {
            self.last_seq = Some(seq);
            self.seq_initialized = true;
            return;
        }

        if let Some(last) = self.last_seq {
            let expected_next = last.wrapping_add(1);
            if seq != expected_next {
                let gap = seq.wrapping_sub(expected_next) as u64;
                self.total_lost += gap;
                self.total_expected += gap;
            }
            self.total_expected += 1;
        }

        self.last_seq = Some(seq);

        if self.total_expected > 0 {
            self.packet_loss_pct = (self.total_lost as f32 / self.total_expected as f32) * 100.0;
        }
    }

    fn bandwidth_pct(&self) -> f32 {
        if self.max_bytes_per_sec > 0.0 {
            (self.bytes_per_sec / self.max_bytes_per_sec) * 100.0
        } else {
            0.0
        }
    }
}

fn payload_size(msg: &MavMessage) -> usize {
    match msg {
        MavMessage::HEARTBEAT(_) => 9,
        MavMessage::GLOBAL_POSITION_INT(_) => 28,
        MavMessage::SCALED_IMU(_) => 22,
        MavMessage::SCALED_PRESSURE(_) => 14,
        MavMessage::GPS_RAW_INT(_) => 30,
        _ => 20,
    }
}
