use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use mavlink::ardupilotmega::*;
use mavlink::MavConnection;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let addr = args
        .get(1)
        .map(|s| s.as_str())
        .unwrap_or("serial:/dev/ttyUSB0:57600");

    println!("Connecting to {addr}...");
    let conn = mavlink::connect::<MavMessage>(addr).expect("failed to connect");
    let conn = Arc::new(conn);

    // Receive thread — print anything that comes back
    let recv_conn = conn.clone();
    thread::spawn(move || loop {
        match recv_conn.recv() {
            Ok((header, msg)) => {
                println!("rx [sys={} comp={}]: {msg:?}", header.system_id, header.component_id);
            }
            Err(mavlink::error::MessageReadError::Io(e))
                if e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                thread::sleep(Duration::from_millis(10));
            }
            Err(e) => {
                eprintln!("recv error: {e:?}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    });

    // Send loop — heartbeat + telemetry at 4 Hz
    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };

    let start = Instant::now();
    let mut seq: u32 = 0;

    println!("Sending heartbeat + telemetry at 4 Hz. Ctrl-C to stop.");

    loop {
        let elapsed = start.elapsed();
        let t = elapsed.as_secs_f32();
        let time_boot_ms = elapsed.as_millis() as u32;
        let alt_m = (seq as f32) * 10.0; // climb 10m per tick

        // Heartbeat
        conn.send(&header, &MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_ROCKET,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 0x3,
        }))
        .expect("failed to send heartbeat");

        // Position with incrementing altitude
        conn.send(&header, &MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
            time_boot_ms,
            lat: 357_796_000,   // ~35.7796 N
            lon: -789_320_000,  // ~-78.932 W
            alt: (alt_m * 1000.0) as i32,
            relative_alt: (alt_m * 1000.0) as i32,
            vx: 0,
            vy: 0,
            vz: -100, // climbing at 1 m/s
            hdg: u16::MAX,
        }))
        .expect("failed to send position");

        // IMU — simulated vibration with slow drift
        conn.send(&header, &MavMessage::SCALED_IMU(SCALED_IMU_DATA {
            time_boot_ms,
            xacc: (50.0 * t.sin()) as i16,           // mG
            yacc: (30.0 * (t * 1.3).cos()) as i16,
            zacc: -1000 + (20.0 * (t * 0.7).sin()) as i16, // ~-1g with wobble
            xgyro: (100.0 * (t * 2.0).sin()) as i16, // mrad/s
            ygyro: (80.0 * (t * 1.7).cos()) as i16,
            zgyro: (60.0 * (t * 0.9).sin()) as i16,
            xmag: (250.0 + 10.0 * (t * 0.1).sin()) as i16, // mgauss
            ymag: (50.0 + 10.0 * (t * 0.1).cos()) as i16,
            zmag: (-400.0 + 5.0 * t.sin()) as i16,
        }))
        .expect("failed to send IMU");

        // Barometer — pressure decreases with altitude
        let press_hpa = 1013.25 * (1.0 - alt_m / 44330.0_f32).powf(5.255);
        let temp_cdeg = (2200.0 - alt_m * 0.65) as i16; // ~22°C, lapse rate 6.5°C/km
        conn.send(&header, &MavMessage::SCALED_PRESSURE(SCALED_PRESSURE_DATA {
            time_boot_ms,
            press_abs: press_hpa,
            press_diff: 0.0,
            temperature: temp_cdeg,
        }))
        .expect("failed to send pressure");

        // GPS raw
        conn.send(&header, &MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            time_usec: elapsed.as_micros() as u64,
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            lat: 357_796_000,
            lon: -789_320_000,
            alt: (alt_m * 1000.0) as i32,
            eph: 120, // HDOP * 100
            epv: 180, // VDOP * 100
            vel: 100, // cm/s
            cog: u16::MAX,
            satellites_visible: 12,
        }))
        .expect("failed to send GPS");

        println!(
            "[{:>6.1}s] seq={seq} alt={alt_m:.0}m press={press_hpa:.1}hPa",
            elapsed.as_secs_f64()
        );

        seq += 1;
        thread::sleep(Duration::from_millis(250));
    }
}
