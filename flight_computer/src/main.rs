mod audio;
mod camera;
mod command_rx;
mod config;
mod frame_sender;
mod pico_rx;

use std::sync::atomic::{AtomicU8, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;
use std::time::{Duration, Instant};

use clap::Parser;
use mavlink::ardupilotmega::*;
use mavlink::MavHeader;

use config::Config;
use command_rx::VehicleCommand;

/// Adaptive token bucket rate limiter with AIMD congestion control.
///
/// On write failures (congestion signal), the effective rate is halved.
/// On sustained success, it ramps back up toward the configured max.
struct RateLimiter {
    max_bps: f64,
    effective_bps: f64,
    min_bps: f64,
    tokens: f64,
    last_refill: Instant,
    success_streak: u32,
    /// Number of successes before increasing rate by one step
    increase_threshold: u32,
    /// Additive increase step (bytes/sec per threshold reached)
    increase_step: f64,
}

impl RateLimiter {
    fn new(max_bytes_per_sec: u32) -> Self {
        let max = max_bytes_per_sec as f64;
        // Reserve ~800 B/s for telemetry even at minimum
        let min = 800.0_f64.min(max * 0.2);
        Self {
            max_bps: max,
            effective_bps: max,
            min_bps: min,
            tokens: max * 0.1,
            last_refill: Instant::now(),
            success_streak: 0,
            increase_threshold: 50, // ~50 successful sends before bumping rate
            increase_step: max * 0.1, // recover 10% of max per threshold
        }
    }

    /// Wait until we have enough tokens for `bytes` bytes, then consume them.
    fn consume(&mut self, bytes: usize) {
        let cost = bytes as f64;

        let now = Instant::now();
        let dt = now.duration_since(self.last_refill).as_secs_f64();
        self.last_refill = now;
        self.tokens += dt * self.effective_bps;

        // Cap burst to 1 second of effective rate
        if self.tokens > self.effective_bps {
            self.tokens = self.effective_bps;
        }

        if self.tokens < cost {
            let deficit = cost - self.tokens;
            let wait = Duration::from_secs_f64(deficit / self.effective_bps);
            thread::sleep(wait);
            self.tokens = 0.0;
            self.last_refill = Instant::now();
        } else {
            self.tokens -= cost;
        }
    }

    /// Call after a successful send.
    fn notify_success(&mut self) {
        self.success_streak += 1;
        if self.success_streak >= self.increase_threshold && self.effective_bps < self.max_bps {
            self.effective_bps = (self.effective_bps + self.increase_step).min(self.max_bps);
            self.success_streak = 0;
        }
    }

    /// Call after a failed send — halve the effective rate.
    fn notify_failure(&mut self) {
        let prev = self.effective_bps;
        self.effective_bps = (self.effective_bps * 0.5).max(self.min_bps);
        self.success_streak = 0;
        if (prev - self.effective_bps).abs() > 1.0 {
            eprintln!(
                "[rate_limiter] congestion: {:.0} -> {:.0} B/s (max {:.0})",
                prev, self.effective_bps, self.max_bps
            );
        }
    }

    /// Current effective rate as a fraction of max (0.0 - 1.0).
    fn load_factor(&self) -> f32 {
        (self.effective_bps / self.max_bps) as f32
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
        let result = mavlink::write_v2_msg(&mut *lock, header, msg).map(|_| ());

        // Feed result back to adaptive rate limiter
        drop(lock); // release port lock before acquiring rate limiter lock
        let mut rl = self.rate_limiter.lock().unwrap();
        match &result {
            Ok(()) => rl.notify_success(),
            Err(_) => rl.notify_failure(),
        }

        result
    }

    /// Current load factor (0.0-1.0) — how much of the max rate we're using.
    /// Frame sender uses this to decide whether to skip frames.
    pub fn load_factor(&self) -> f32 {
        self.rate_limiter.lock().unwrap().load_factor()
    }
}

/// Open a serial port with a write timeout and rate limiter.
/// Returns MavSerial for writing + a cloned port for reading incoming commands.
fn open_mav_serial(addr: &str, radio_bps: u32) -> (Arc<MavSerial>, Box<dyn serialport::SerialPort>) {
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

    let read_port = port.try_clone().expect("failed to clone serial port");

    println!("Radio rate limiter: {} bytes/sec", radio_bps);

    let mav = Arc::new(MavSerial {
        port: Mutex::new(port),
        sequence: AtomicU8::new(0),
        rate_limiter: Mutex::new(RateLimiter::new(radio_bps)),
        system_id: 1,
        component_id: 1,
    });

    (mav, read_port)
}

/// Run a command with sudo, piping the password via stdin.
fn sudo_exec(args: &[&str]) {
    use std::io::Write;
    use std::process::{Command, Stdio};

    let mut child = match Command::new("sudo")
        .arg("-S")
        .args(args)
        .stdin(Stdio::piped())
        .spawn()
    {
        Ok(c) => c,
        Err(e) => {
            eprintln!("[main] Failed to spawn sudo: {e}");
            return;
        }
    };

    if let Some(mut stdin) = child.stdin.take() {
        let _ = writeln!(stdin, "orangepi");
    }

    match child.wait() {
        Ok(status) => println!("[main] sudo {:?} exited: {status}", args),
        Err(e) => eprintln!("[main] sudo wait error: {e}"),
    }
}

/// Start all configured cameras. The camera at `stream_camera_idx` index
/// produces the radio stream; all others are record-only.
fn start_cameras(
    config: &Config,
    devices: &[String],
    stream_camera_idx: usize,
    mav: &Arc<MavSerial>,
    mgrs: &Arc<Mutex<Vec<camera::CameraManager>>>,
) {
    // Clamp stream index: if out of range and multiple cameras, use last; if single, use 0
    let stream_idx = if stream_camera_idx < devices.len() {
        stream_camera_idx
    } else if devices.len() > 1 {
        devices.len() - 1
    } else {
        0
    };

    for (i, device) in devices.iter().enumerate() {
        let label = format!("cam{i}");
        let with_stream = i == stream_idx;

        match camera::CameraManager::start(config, device, &label, with_stream) {
            Ok((cam, frame_rx)) => {
                println!(
                    "{}: {} -> {} ({}s segments){}",
                    label, device, config.recording_dir, config.segment_duration,
                    if with_stream { " [STREAMING]" } else { "" }
                );
                if with_stream {
                    if let Some(rx) = frame_rx {
                        println!(
                            "Radio stream from {label}: {}x{} {} q={} @ ~{}fps",
                            config.stream_width, config.stream_height,
                            config.stream_format.to_uppercase(),
                            config.jpeg_quality, config.stream_fps
                        );
                        frame_sender::spawn(mav.clone(), rx);
                    }
                }
                mgrs.lock().unwrap().push(cam);
            }
            Err(e) => {
                eprintln!("WARNING: {label} ({device}) failed to start: {e}");
            }
        }
    }

    if mgrs.lock().unwrap().is_empty() {
        eprintln!("WARNING: No cameras started. Continuing with telemetry only.");
    }
}

fn main() {
    let config = Config::parse();

    // Ensure data directory exists
    std::fs::create_dir_all(&config.recording_dir).unwrap_or_else(|e| {
        eprintln!("WARNING: Failed to create data dir {}: {e}", config.recording_dir);
    });
    println!("Data directory: {}", config.recording_dir);

    println!("Connecting to {}...", config.mavlink_addr);

    let (mav, read_port) = open_mav_serial(&config.mavlink_addr, config.radio_bps);

    // Shared state for cameras and audio (can be stopped via command)
    let camera_mgrs: Arc<Mutex<Vec<camera::CameraManager>>> = Arc::new(Mutex::new(Vec::new()));
    let audio_mgr: Arc<Mutex<Option<audio::AudioRecorder>>> = Arc::new(Mutex::new(None));
    let stream_camera_idx = Arc::new(AtomicUsize::new(config.stream_camera));

    // Resolve camera device list (auto-detect if none specified)
    let camera_devices: Vec<String> = if config.camera_devices.is_empty() {
        let detected = camera::detect_cameras();
        if detected.is_empty() {
            eprintln!("WARNING: No USB cameras detected. Use --camera-device to specify manually.");
            vec![]
        } else {
            println!("Auto-detected {} camera(s)", detected.len());
            detected
        }
    } else {
        config.camera_devices.clone()
    };

    // Start cameras if enabled
    if !config.no_camera {
        start_cameras(&config, &camera_devices, stream_camera_idx.load(Ordering::Relaxed), &mav, &camera_mgrs);
    } else {
        println!("Cameras disabled (--no-camera)");
    }

    // Start audio recording if enabled
    if !config.no_audio {
        match audio::AudioRecorder::start(&config) {
            Ok(recorder) => {
                println!(
                    "Audio recording started: {} ({}) -> {} ({}s segments)",
                    config.audio_device, config.audio_format,
                    config.recording_dir, config.segment_duration
                );
                *audio_mgr.lock().unwrap() = Some(recorder);
            }
            Err(e) => {
                eprintln!("WARNING: Audio recording failed to start: {e}");
                eprintln!("Continuing without audio.");
            }
        }
    } else {
        println!("Audio recording disabled (--no-audio)");
    }

    // Start pico logger reader if enabled
    let pico_state = if !config.no_pico {
        match pico_rx::spawn_pico_reader(&config.pico_uart, config.pico_baud, &config.recording_dir) {
            Ok(state) => {
                println!("Pico reader started on {} @ {}", config.pico_uart, config.pico_baud);
                Some(state)
            }
            Err(e) => {
                eprintln!("WARNING: Failed to start pico reader: {e}");
                eprintln!("Continuing without pico logger.");
                None
            }
        }
    } else {
        println!("Pico reader disabled (--no-pico)");
        None
    };

    // Start command receiver on radio link (reads MAVLink from cloned serial port)
    let (cmd_tx, cmd_rx) = mpsc::channel::<VehicleCommand>();
    command_rx::spawn_command_receiver(read_port, cmd_tx);

    // Telemetry send loop
    let start = Instant::now();
    let mut _seq: u32 = 0;
    let interval = Duration::from_millis(1000 / config.telemetry_hz as u64);

    println!(
        "Sending heartbeat + telemetry at {} Hz. Ctrl-C to stop.",
        config.telemetry_hz
    );

    loop {
        // Check for ground station commands (non-blocking)
        while let Ok(cmd) = cmd_rx.try_recv() {
            match cmd {
                VehicleCommand::StopRecording => {
                    println!("[main] Stopping data collection...");
                    camera_mgrs.lock().unwrap().clear(); // Drop kills all ffmpeg processes
                    audio_mgr.lock().unwrap().take();
                    println!("[main] Data collection stopped. Safe to power off.");
                }
                VehicleCommand::SwitchCamera(idx) => {
                    let cam_count = camera_devices.len();
                    if idx < cam_count {
                        println!("[main] Switching stream to camera {idx}...");
                        stream_camera_idx.store(idx, Ordering::Relaxed);
                        // Restart all cameras with new stream index
                        camera_mgrs.lock().unwrap().clear();
                        if !config.no_camera {
                            start_cameras(&config, &camera_devices, idx, &mav, &camera_mgrs);
                        }
                        println!("[main] Stream switched to camera {idx}.");
                    } else {
                        eprintln!("[main] Invalid camera index {idx} (have {cam_count} cameras)");
                    }
                }
                VehicleCommand::StartRecording => {
                    println!("[main] Starting data collection...");
                    if camera_mgrs.lock().unwrap().is_empty() && !config.no_camera {
                        start_cameras(&config, &camera_devices, stream_camera_idx.load(Ordering::Relaxed), &mav, &camera_mgrs);
                    }
                    if audio_mgr.lock().unwrap().is_none() && !config.no_audio {
                        match audio::AudioRecorder::start(&config) {
                            Ok(recorder) => {
                                println!("[main] Audio restarted");
                                *audio_mgr.lock().unwrap() = Some(recorder);
                            }
                            Err(e) => eprintln!("[main] Audio restart failed: {e}"),
                        }
                    }
                    println!("[main] Data collection started.");
                }
                VehicleCommand::Shutdown => {
                    println!("[main] Shutdown requested. Stopping recording first...");
                    camera_mgrs.lock().unwrap().clear();
                    audio_mgr.lock().unwrap().take();
                    thread::sleep(Duration::from_secs(1));
                    println!("[main] Shutting down...");
                    sudo_exec(&["shutdown", "-h", "now"]);
                }
                VehicleCommand::Reboot => {
                    println!("[main] Reboot requested. Stopping recording first...");
                    camera_mgrs.lock().unwrap().clear();
                    audio_mgr.lock().unwrap().take();
                    thread::sleep(Duration::from_secs(1));
                    println!("[main] Rebooting...");
                    sudo_exec(&["reboot"]);
                }
            }
        }

        let elapsed = start.elapsed();
        let time_boot_ms = elapsed.as_millis() as u32;

        macro_rules! send_or_log {
            ($msg:expr, $label:expr) => {
                if let Err(e) = mav.send($msg) {
                    eprintln!("[telemetry] {} send failed: {e}", $label);
                }
            };
        }

        // Get latest pico data
        let (pico_connected, pico_data) = if let Some(ref ps) = pico_state {
            let mut st = ps.lock().unwrap();
            st.update_connected();
            (st.connected, st.latest)
        } else {
            (false, None)
        };

        // Heartbeat
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

        // IMU + Pressure from pico logger or zeros
        if let Some(ref pkt) = pico_data.filter(|_| pico_connected) {
            // Convert raw LSB values to MAVLink units
            send_or_log!(
                &MavMessage::SCALED_IMU(SCALED_IMU_DATA {
                    time_boot_ms,
                    xacc: (pkt.accel_x as f32 * 0.488).round() as i16,
                    yacc: (pkt.accel_y as f32 * 0.488).round() as i16,
                    zacc: (pkt.accel_z as f32 * 0.488).round() as i16,
                    xgyro: 0,
                    ygyro: 0,
                    zgyro: 0,
                    xmag: (pkt.mag_x as f32 / 6842.0 * 1000.0).round() as i16,
                    ymag: (pkt.mag_y as f32 / 6842.0 * 1000.0).round() as i16,
                    zmag: (pkt.mag_z as f32 / 6842.0 * 1000.0).round() as i16,
                }),
                "IMU"
            );

            send_or_log!(
                &MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA {
                    time_boot_ms,
                    press_abs: pkt.pressure_pa as f32 / 100.0,
                    press_diff: 0.0,
                    temperature: pkt.temp_cdeg as i16,
                }),
                "pressure"
            );

            // Orientation from integrated gyro
            send_or_log!(
                &MavMessage::ATTITUDE(ATTITUDE_DATA {
                    time_boot_ms,
                    roll: (pkt.roll_cdeg as f32 / 100.0).to_radians(),
                    pitch: (pkt.pitch_cdeg as f32 / 100.0).to_radians(),
                    yaw: (pkt.yaw_cdeg as f32 / 100.0).to_radians(),
                    rollspeed: 0.0,
                    pitchspeed: 0.0,
                    yawspeed: 0.0,
                }),
                "attitude"
            );
        } else {
            send_or_log!(
                &MavMessage::SCALED_IMU(SCALED_IMU_DATA {
                    time_boot_ms,
                    xacc: 0, yacc: 0, zacc: 0,
                    xgyro: 0, ygyro: 0, zgyro: 0,
                    xmag: 0, ymag: 0, zmag: 0,
                }),
                "IMU"
            );

            send_or_log!(
                &MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA {
                    time_boot_ms,
                    press_abs: 0.0,
                    press_diff: 0.0,
                    temperature: 0,
                }),
                "pressure"
            );

            send_or_log!(
                &MavMessage::ATTITUDE(ATTITUDE_DATA {
                    time_boot_ms,
                    roll: 0.0, pitch: 0.0, yaw: 0.0,
                    rollspeed: 0.0, pitchspeed: 0.0, yawspeed: 0.0,
                }),
                "attitude"
            );
        }

        // GPS from pico UBlox receiver
        if let Some(ref pkt) = pico_data.filter(|_| pico_connected) {
            let fix_type = match pkt.gps_fix {
                0 => GpsFixType::GPS_FIX_TYPE_NO_FIX,
                1 => GpsFixType::GPS_FIX_TYPE_3D_FIX,
                2 => GpsFixType::GPS_FIX_TYPE_DGPS,
                4 => GpsFixType::GPS_FIX_TYPE_RTK_FIXED,
                5 => GpsFixType::GPS_FIX_TYPE_RTK_FLOAT,
                _ => GpsFixType::GPS_FIX_TYPE_NO_FIX,
            };
            send_or_log!(
                &MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    time_usec: elapsed.as_micros() as u64,
                    fix_type,
                    lat: pkt.gps_lat_e7,
                    lon: pkt.gps_lon_e7,
                    alt: pkt.gps_alt_mm,
                    eph: if pkt.gps_hdop_e2 > 0 { pkt.gps_hdop_e2 } else { u16::MAX },
                    epv: u16::MAX,
                    vel: pkt.gps_speed_cms,
                    cog: if pkt.gps_fix > 0 { pkt.gps_course_e2 } else { u16::MAX },
                    satellites_visible: pkt.gps_sats,
                }),
                "GPS"
            );
        } else {
            send_or_log!(
                &MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    time_usec: elapsed.as_micros() as u64,
                    fix_type: GpsFixType::GPS_FIX_TYPE_NO_GPS,
                    lat: 0,
                    lon: 0,
                    alt: 0,
                    eph: u16::MAX,
                    epv: u16::MAX,
                    vel: 0,
                    cog: u16::MAX,
                    satellites_visible: 0,
                }),
                "GPS"
            );
        }

        // Pico logger connection status
        let mut pico_name = [0u8; 10];
        pico_name[..7].copy_from_slice(b"pico_ok");
        send_or_log!(
            &MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
                time_boot_ms,
                name: pico_name.into(),
                value: if pico_connected { 1 } else { 0 },
            }),
            "pico_status"
        );

        // Camera recording status
        let cam_recording = !camera_mgrs.lock().unwrap().is_empty();
        let mut cam_name = [0u8; 10];
        cam_name[..7].copy_from_slice(b"cam_rec");
        send_or_log!(
            &MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
                time_boot_ms,
                name: cam_name.into(),
                value: if cam_recording { 1 } else { 0 },
            }),
            "cam_rec"
        );

        // Audio recording status
        let aud_recording = audio_mgr.lock().unwrap().is_some();
        let mut aud_name = [0u8; 10];
        aud_name[..7].copy_from_slice(b"aud_rec");
        send_or_log!(
            &MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
                time_boot_ms,
                name: aud_name.into(),
                value: if aud_recording { 1 } else { 0 },
            }),
            "aud_rec"
        );

        // Camera count
        let mut cam_cnt_name = [0u8; 10];
        cam_cnt_name[..7].copy_from_slice(b"cam_cnt");
        send_or_log!(
            &MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
                time_boot_ms,
                name: cam_cnt_name.into(),
                value: camera_devices.len() as i32,
            }),
            "cam_cnt"
        );

        // Current stream camera index
        let mut cam_idx_name = [0u8; 10];
        cam_idx_name[..7].copy_from_slice(b"cam_idx");
        send_or_log!(
            &MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
                time_boot_ms,
                name: cam_idx_name.into(),
                value: stream_camera_idx.load(Ordering::Relaxed) as i32,
            }),
            "cam_idx"
        );

        _seq += 1;
        thread::sleep(interval);
    }
}
