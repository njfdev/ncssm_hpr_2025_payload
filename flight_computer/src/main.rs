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

    // Send loop — heartbeat + incrementing altitude every 250ms
    let header = mavlink::MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };

    let start = Instant::now();
    let mut seq: u32 = 0;

    println!("Sending heartbeat + altitude at 4 Hz. Ctrl-C to stop.");

    loop {
        let elapsed = start.elapsed();
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
            time_boot_ms: elapsed.as_millis() as u32,
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

        println!(
            "[{:>6.1}s] seq={seq} alt={alt_m:.0}m",
            elapsed.as_secs_f64()
        );

        seq += 1;
        thread::sleep(Duration::from_millis(250));
    }
}
