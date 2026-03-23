use std::io::Write;
use std::time::{Duration, Instant};

use clap::Parser;
use payload_types::{TelemetryPacket, MAX_ENCODED_SIZE, units};
use rand::Rng;

#[derive(Parser)]
#[command(name = "telemetry_tester", about = "Generate simulated telemetry for radio testing")]
struct Args {
    /// Serial port (auto-detects /dev/cu.usbserial-* if omitted)
    #[arg(short, long)]
    port: Option<String>,

    /// Baud rate (must match RFD 900x air rate config)
    #[arg(short, long, default_value_t = 57600)]
    baud: u32,

    /// Packet rate in Hz
    #[arg(short, long, default_value_t = 10.0)]
    rate: f64,

    /// Duration in seconds (0 = run forever)
    #[arg(short, long, default_value_t = 0)]
    duration: u64,

    /// Simulate a launch profile (acceleration spike, altitude climb)
    #[arg(long)]
    launch: bool,
}

fn auto_detect_port() -> Option<String> {
    let ports = serialport::available_ports().ok()?;
    // Prefer /dev/cu.usbserial-* (FTDI adapter for RFD 900x)
    for p in &ports {
        if p.port_name.contains("cu.usbserial") {
            return Some(p.port_name.clone());
        }
    }
    None
}

struct SimState {
    counter: u32,
    pressure_pa: f64,
    temp_cdeg: f64,
    accel_z_bias: f64,
    gyro_drift: [f64; 3],
    mag_base: [f64; 3],
    phase: LaunchPhase,
    phase_start: f64,
}

#[derive(Clone, Copy, PartialEq)]
enum LaunchPhase {
    Pad,
    Boost,
    Coast,
    Apogee,
    Descent,
}

impl SimState {
    fn new(launch: bool) -> Self {
        Self {
            counter: 0,
            pressure_pa: 101325.0,
            temp_cdeg: 2550.0,
            accel_z_bias: units::ONE_G_LSB as f64, // 1g on Z at rest
            gyro_drift: [0.0; 3],
            mag_base: [200.0, -100.0, 300.0], // rough earth field in LSB
            phase: if launch { LaunchPhase::Pad } else { LaunchPhase::Pad },
            phase_start: 0.0,
        }
    }

    fn step(&mut self, t_sec: f64, dt: f64, launch: bool) -> TelemetryPacket {
        let mut rng = rand::rng();
        self.counter += 1;

        if launch {
            self.update_launch_phase(t_sec);
        }

        // Pressure: random walk simulating altitude change
        let pressure_drift = match self.phase {
            LaunchPhase::Pad => rng.random_range(-5.0..5.0),
            LaunchPhase::Boost => -200.0 * dt + rng.random_range(-10.0..10.0),
            LaunchPhase::Coast => -100.0 * dt + rng.random_range(-10.0..10.0),
            LaunchPhase::Apogee => rng.random_range(-5.0..5.0),
            LaunchPhase::Descent => 150.0 * dt + rng.random_range(-10.0..10.0),
        };
        self.pressure_pa = (self.pressure_pa + pressure_drift).clamp(20000.0, 110000.0);

        // Temperature: slow drift (gets colder with altitude)
        let temp_drift = match self.phase {
            LaunchPhase::Boost | LaunchPhase::Coast => -5.0 * dt,
            LaunchPhase::Descent => 3.0 * dt,
            _ => rng.random_range(-0.5..0.5) * dt,
        };
        self.temp_cdeg = (self.temp_cdeg + temp_drift).clamp(-4000.0, 6000.0);

        // Accelerometer
        let (ax, ay, az) = match self.phase {
            LaunchPhase::Pad => (
                rng.random_range(-20.0..20.0),
                rng.random_range(-20.0..20.0),
                self.accel_z_bias + rng.random_range(-10.0..10.0),
            ),
            LaunchPhase::Boost => {
                // ~10g on Z during boost
                let boost_g = 10.0;
                let boost_lsb = boost_g / units::ACCEL_G_PER_LSB as f64;
                (
                    rng.random_range(-50.0..50.0),
                    rng.random_range(-50.0..50.0),
                    self.accel_z_bias + boost_lsb + rng.random_range(-100.0..100.0),
                )
            }
            LaunchPhase::Coast => (
                rng.random_range(-30.0..30.0),
                rng.random_range(-30.0..30.0),
                rng.random_range(-30.0..30.0), // ~0g freefall
            ),
            LaunchPhase::Apogee => (
                rng.random_range(-10.0..10.0),
                rng.random_range(-10.0..10.0),
                rng.random_range(-10.0..10.0),
            ),
            LaunchPhase::Descent => (
                rng.random_range(-20.0..20.0),
                rng.random_range(-20.0..20.0),
                self.accel_z_bias * 0.3 + rng.random_range(-50.0..50.0), // drag
            ),
        };

        // Gyroscope: noise + drift
        for d in &mut self.gyro_drift {
            *d += rng.random_range(-0.5..0.5) * dt;
            *d = d.clamp(-50.0, 50.0);
        }
        let gyro_noise: f64 = match self.phase {
            LaunchPhase::Boost => 200.0,
            LaunchPhase::Coast => 50.0,
            _ => 20.0,
        };
        let gx = self.gyro_drift[0] + rng.random_range(-gyro_noise..gyro_noise);
        let gy = self.gyro_drift[1] + rng.random_range(-gyro_noise..gyro_noise);
        let gz = self.gyro_drift[2] + rng.random_range(-gyro_noise..gyro_noise);

        // Magnetometer: earth field + noise
        let mag_noise = 5.0;
        let mx = self.mag_base[0] + rng.random_range(-mag_noise..mag_noise);
        let my = self.mag_base[1] + rng.random_range(-mag_noise..mag_noise);
        let mz = self.mag_base[2] + rng.random_range(-mag_noise..mag_noise);

        // Air quality: mostly stable
        let aqi = if rng.random_range(0..100) < 5 { 2 } else { 1 };
        let tvoc = rng.random_range(20..100);
        let eco2 = rng.random_range(400..500);

        TelemetryPacket {
            counter: self.counter,
            timestamp_ms: (t_sec * 1000.0) as u32,
            pressure_pa: self.pressure_pa as u32,
            temp_cdeg: self.temp_cdeg as u32,
            accel_x: ax as i16,
            accel_y: ay as i16,
            accel_z: az as i16,
            gyro_x: gx as i16,
            gyro_y: gy as i16,
            gyro_z: gz as i16,
            mag_x: mx as i16,
            mag_y: my as i16,
            mag_z: mz as i16,
            aqi: aqi as u8,
            tvoc_ppb: tvoc as u16,
            eco2_ppm: eco2 as u16,
            gps_lat_e7: 0,
            gps_lon_e7: 0,
            gps_alt_mm: 0,
            gps_fix: 0,
            gps_sats: 0,
            gps_hdop_e2: 0,
            gps_speed_cms: 0,
            gps_course_e2: 0,
        }
    }

    fn update_launch_phase(&mut self, t_sec: f64) {
        let elapsed = t_sec - self.phase_start;
        match self.phase {
            LaunchPhase::Pad => {
                if t_sec >= 5.0 {
                    self.phase = LaunchPhase::Boost;
                    self.phase_start = t_sec;
                    eprintln!("[SIM] T+{:.1}s: BOOST", t_sec);
                }
            }
            LaunchPhase::Boost => {
                if elapsed >= 3.0 {
                    self.phase = LaunchPhase::Coast;
                    self.phase_start = t_sec;
                    eprintln!("[SIM] T+{:.1}s: COAST", t_sec);
                }
            }
            LaunchPhase::Coast => {
                if elapsed >= 10.0 {
                    self.phase = LaunchPhase::Apogee;
                    self.phase_start = t_sec;
                    eprintln!("[SIM] T+{:.1}s: APOGEE", t_sec);
                }
            }
            LaunchPhase::Apogee => {
                if elapsed >= 2.0 {
                    self.phase = LaunchPhase::Descent;
                    self.phase_start = t_sec;
                    eprintln!("[SIM] T+{:.1}s: DESCENT", t_sec);
                }
            }
            LaunchPhase::Descent => {} // continues until program ends
        }
    }
}

fn main() {
    let args = Args::parse();

    let port_name = args.port.unwrap_or_else(|| {
        auto_detect_port().unwrap_or_else(|| {
            eprintln!("No serial port specified and auto-detect failed.");
            eprintln!("Available ports:");
            if let Ok(ports) = serialport::available_ports() {
                for p in &ports {
                    eprintln!("  {}", p.port_name);
                }
            }
            std::process::exit(1);
        })
    });

    let interval = Duration::from_secs_f64(1.0 / args.rate);
    let deadline = if args.duration > 0 {
        Some(Instant::now() + Duration::from_secs(args.duration))
    } else {
        None
    };

    eprintln!(
        "Opening {} at {} baud, sending at {:.1} Hz{}",
        port_name,
        args.baud,
        args.rate,
        if args.launch { " (launch sim)" } else { "" }
    );

    let mut port = serialport::new(&port_name, args.baud)
        .timeout(Duration::from_millis(1000))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open {}: {}", port_name, e);
            std::process::exit(1);
        });

    let mut sim = SimState::new(args.launch);
    let start = Instant::now();
    let dt = 1.0 / args.rate;

    loop {
        if let Some(end) = deadline {
            if Instant::now() >= end {
                eprintln!("Duration elapsed, sent {} packets", sim.counter);
                break;
            }
        }

        let t_sec = start.elapsed().as_secs_f64();
        let pkt = sim.step(t_sec, dt, args.launch);

        let mut buf = [0u8; MAX_ENCODED_SIZE];
        match pkt.encode_cobs(&mut buf) {
            Ok(encoded) => {
                if let Err(e) = port.write_all(encoded) {
                    eprintln!("Write error: {}", e);
                    break;
                }
                println!("{}", pkt);
            }
            Err(e) => {
                eprintln!("Encode error: {:?}", e);
            }
        }

        std::thread::sleep(interval);
    }
}
