use std::io::{Read, Write, BufWriter};
use std::fs::File;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use payload_types::TelemetryPacket;

pub struct PicoState {
    pub latest: Option<TelemetryPacket>,
    pub last_received: Instant,
    pub connected: bool,
}

impl PicoState {
    pub fn new() -> Self {
        Self {
            latest: None,
            last_received: Instant::now(),
            connected: false,
        }
    }

    pub fn update_connected(&mut self) {
        self.connected = self.last_received.elapsed().as_secs() < 3;
    }
}

pub fn spawn_pico_reader(
    uart_path: &str,
    baud: u32,
    data_dir: &str,
) -> Result<Arc<Mutex<PicoState>>, String> {
    let state = Arc::new(Mutex::new(PicoState::new()));

    let port = serialport::new(uart_path, baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|e| format!("Failed to open pico UART {uart_path}: {e}"))?;

    // Create CSV log file for redundant telemetry recording
    let log_path = format!(
        "{}/telemetry_{}.csv",
        data_dir,
        chrono::Local::now().format("%Y%m%d_%H%M%S")
    );
    let log_file = File::create(&log_path)
        .map_err(|e| format!("Failed to create telemetry log {log_path}: {e}"))?;
    println!("[pico_rx] Logging telemetry to {log_path}");

    let state_clone = state.clone();

    thread::spawn(move || {
        let mut port = port;
        let mut writer = BufWriter::new(log_file);

        // Write CSV header
        let _ = writeln!(
            writer,
            "counter,timestamp_ms,pressure_pa,temp_cdeg,\
             accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,\
             mag_x,mag_y,mag_z,aqi,tvoc_ppb,eco2_ppm,\
             gps_lat_e7,gps_lon_e7,gps_alt_mm,gps_fix,gps_sats,gps_hdop_e2,gps_speed_cms,gps_course_e2"
        );
        let _ = writer.flush();

        let mut buf = [0u8; 256];
        let mut frame_buf = Vec::with_capacity(128);
        let mut first_decode = true;
        let mut decode_count: u64 = 0;
        let mut last_stats = Instant::now();
        let mut last_flush = Instant::now();

        loop {
            match port.read(&mut buf) {
                Ok(n) => {
                    for &byte in &buf[..n] {
                        if byte == 0x00 {
                            if !frame_buf.is_empty() {
                                frame_buf.push(0x00);
                                if let Ok(packet) = TelemetryPacket::decode_cobs(&mut frame_buf) {
                                    // Write to CSV log
                                    let _ = writeln!(
                                        writer,
                                        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                                        packet.counter,
                                        packet.timestamp_ms,
                                        packet.pressure_pa,
                                        packet.temp_cdeg,
                                        packet.accel_x,
                                        packet.accel_y,
                                        packet.accel_z,
                                        packet.gyro_x,
                                        packet.gyro_y,
                                        packet.gyro_z,
                                        packet.mag_x,
                                        packet.mag_y,
                                        packet.mag_z,
                                        packet.aqi,
                                        packet.tvoc_ppb,
                                        packet.eco2_ppm,
                                        packet.gps_lat_e7,
                                        packet.gps_lon_e7,
                                        packet.gps_alt_mm,
                                        packet.gps_fix,
                                        packet.gps_sats,
                                        packet.gps_hdop_e2,
                                        packet.gps_speed_cms,
                                        packet.gps_course_e2,
                                    );

                                    let mut st = state_clone.lock().unwrap();
                                    st.latest = Some(packet);
                                    st.last_received = Instant::now();
                                    st.connected = true;

                                    decode_count += 1;
                                    if first_decode {
                                        println!("[pico_rx] First packet decoded: #{}", packet.counter);
                                        first_decode = false;
                                    }
                                }
                            }
                            frame_buf.clear();
                        } else {
                            frame_buf.push(byte);
                            if frame_buf.len() > 128 {
                                frame_buf.clear();
                            }
                        }
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => {
                    eprintln!("[pico_rx] Read error: {e}");
                    thread::sleep(Duration::from_millis(100));
                }
            }

            // Flush CSV every 5 seconds for power safety
            if last_flush.elapsed().as_secs() >= 5 {
                let _ = writer.flush();
                last_flush = Instant::now();
            }

            {
                let mut st = state_clone.lock().unwrap();
                st.update_connected();
            }

            if last_stats.elapsed().as_secs() >= 30 {
                println!("[pico_rx] {} packets decoded total", decode_count);
                last_stats = Instant::now();
            }
        }
    });

    Ok(state)
}
