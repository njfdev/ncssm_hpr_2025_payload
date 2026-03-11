mod camera;
mod config;
mod frame_sender;

use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use mavlink::ardupilotmega::*;
use mavlink::MavHeader;

use config::Config;

/// Token bucket rate limiter to prevent overdriving the radio link.
struct RateLimiter {
    max_bytes_per_sec: f64,
    tokens: f64,
    last_refill: Instant,
}

impl RateLimiter {
    fn new(max_bytes_per_sec: u32) -> Self {
        Self {
            max_bytes_per_sec: max_bytes_per_sec as f64,
            // Start with a small burst allowance (one full tick)
            tokens: max_bytes_per_sec as f64 * 0.1,
            last_refill: Instant::now(),
        }
    }

    /// Wait until we have enough tokens for `bytes` bytes, then consume them.
    fn consume(&mut self, bytes: usize) {
        let cost = bytes as f64;

        // Refill tokens based on elapsed time
        let now = Instant::now();
        let dt = now.duration_since(self.last_refill).as_secs_f64();
        self.last_refill = now;
        self.tokens += dt * self.max_bytes_per_sec;

        // Cap tokens to 1 second of burst
        if self.tokens > self.max_bytes_per_sec {
            self.tokens = self.max_bytes_per_sec;
        }

        // If not enough tokens, sleep until we have them
        if self.tokens < cost {
            let deficit = cost - self.tokens;
            let wait = Duration::from_secs_f64(deficit / self.max_bytes_per_sec);
            thread::sleep(wait);
            self.tokens = 0.0;
            self.last_refill = Instant::now();
        } else {
            self.tokens -= cost;
        }
    }
}

/// Shared serial port + sequence counter + rate limiter for MAVLink writes.
pub struct MavSerial {
    port: Mutex<Box<dyn serialport::SerialPort>>,
    sequence: AtomicU8,
    rate_limiter: Mutex<RateLimiter>,
    pub system_id: u8,
    pub component_id: u8,
}

impl MavSerial {
    /// Send a MAVLink v2 message with auto-incrementing sequence and rate limiting.
    pub fn send(&self, msg: &MavMessage) -> Result<(), mavlink::error::MessageWriteError> {
        // Estimate wire bytes: MAVLink v2 = 12 bytes overhead + payload
        // ENCAPSULATED_DATA is 255 bytes payload, others are 9-30 bytes
        let wire_bytes = match msg {
            MavMessage::ENCAPSULATED_DATA(_) => 267,
            _ => 50, // conservative average for telemetry messages
        };

        // Rate limit before acquiring the port lock
        self.rate_limiter.lock().unwrap().consume(wire_bytes);

        // Assign sequence INSIDE the port lock so wire order matches sequence order
        let mut lock = self.port.lock().unwrap();
        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
        };
        mavlink::write_v2_msg(&mut *lock, header, msg).map(|_| ())
    }
}

/// Open a serial port with a write timeout and rate limiter.
fn open_mav_serial(addr: &str, radio_bps: u32) -> Arc<MavSerial> {
    // Parse "serial:/dev/ttyXXX:BAUD"
    let parts: Vec<&str> = addr.split(':').collect();
    let (path, baud) = if parts.len() == 3 && parts[0] == "serial" {
        (parts[1], parts[2].parse::<u32>().expect("invalid baud rate"))
    } else {
        panic!("expected serial:/dev/ttyXXX:BAUD, got {addr}");
    };

    let port = serialport::new(path, baud)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(100))
        .open()
        .unwrap_or_else(|e| panic!("failed to open serial port {path}: {e}"));

    println!("Radio rate limiter: {} bytes/sec", radio_bps);

    Arc::new(MavSerial {
        port: Mutex::new(port),
        sequence: AtomicU8::new(0),
        rate_limiter: Mutex::new(RateLimiter::new(radio_bps)),
        system_id: 1,
        component_id: 1,
    })
}

fn main() {
    let config = Config::parse();

    println!("Connecting to {}...", config.mavlink_addr);

    let mav = open_mav_serial(&config.mavlink_addr, config.radio_bps);

    // Start camera if enabled
    if !config.no_camera {
        match camera::CameraManager::start(&config) {
            Ok((_cam, frame_rx)) => {
                println!(
                    "Camera started: {} -> {} ({}s segments)",
                    config.camera_device, config.recording_dir, config.segment_duration
                );
                println!(
                    "Radio stream: {}x{} {} q={} @ ~{}fps source",
                    config.stream_width, config.stream_height,
                    config.stream_format.to_uppercase(),
                    config.jpeg_quality, config.stream_fps
                );

                frame_sender::spawn(mav.clone(), frame_rx);

                // Leak the CameraManager so it lives until process exit.
                // Its Drop impl would kill ffmpeg, which we don't want.
                std::mem::forget(_cam);
            }
            Err(e) => {
                eprintln!("WARNING: Camera failed to start: {e}");
                eprintln!("Continuing with telemetry only.");
            }
        }
    } else {
        println!("Camera disabled (--no-camera)");
    }

    // Telemetry send loop
    let start = Instant::now();
    let mut seq: u32 = 0;
    let interval = Duration::from_millis(1000 / config.telemetry_hz as u64);

    println!(
        "Sending heartbeat + telemetry at {} Hz. Ctrl-C to stop.",
        config.telemetry_hz
    );

    loop {
        let elapsed = start.elapsed();
        let t = elapsed.as_secs_f32();
        let time_boot_ms = elapsed.as_millis() as u32;
        let alt_m = ((seq % 4000) as f32) * 10.0;

        macro_rules! send_or_log {
            ($msg:expr, $label:expr) => {
                if let Err(e) = mav.send($msg) {
                    eprintln!("[telemetry] {} send failed: {e}", $label);
                }
            };
        }

        send_or_log!(
            &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_ROCKET,
                autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
                base_mode: MavModeFlag::empty(),
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 0x3,
            }),
            "heartbeat"
        );

        send_or_log!(
            &MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
                time_boot_ms,
                lat: 357_796_000,
                lon: -789_320_000,
                alt: (alt_m * 1000.0) as i32,
                relative_alt: (alt_m * 1000.0) as i32,
                vx: 0,
                vy: 0,
                vz: -100,
                hdg: u16::MAX,
            }),
            "position"
        );

        send_or_log!(
            &MavMessage::SCALED_IMU(SCALED_IMU_DATA {
                time_boot_ms,
                xacc: (50.0 * t.sin()) as i16,
                yacc: (30.0 * (t * 1.3).cos()) as i16,
                zacc: -1000 + (20.0 * (t * 0.7).sin()) as i16,
                xgyro: (100.0 * (t * 2.0).sin()) as i16,
                ygyro: (80.0 * (t * 1.7).cos()) as i16,
                zgyro: (60.0 * (t * 0.9).sin()) as i16,
                xmag: (250.0 + 10.0 * (t * 0.1).sin()) as i16,
                ymag: (50.0 + 10.0 * (t * 0.1).cos()) as i16,
                zmag: (-400.0 + 5.0 * t.sin()) as i16,
            }),
            "IMU"
        );

        let press_hpa = 1013.25 * (1.0 - alt_m / 44330.0_f32).powf(5.255);
        let temp_cdeg = (2200.0 - alt_m * 0.65) as i16;
        send_or_log!(
            &MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA {
                time_boot_ms,
                press_abs: press_hpa,
                press_diff: 0.0,
                temperature: temp_cdeg,
            }),
            "pressure"
        );

        send_or_log!(
            &MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                time_usec: elapsed.as_micros() as u64,
                fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                lat: 357_796_000,
                lon: -789_320_000,
                alt: (alt_m * 1000.0) as i32,
                eph: 120,
                epv: 180,
                vel: 100,
                cog: u16::MAX,
                satellites_visible: 12,
            }),
            "GPS"
        );

        seq += 1;
        thread::sleep(interval);
    }
}
