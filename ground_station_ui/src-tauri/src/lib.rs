use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use mavlink::ardupilotmega::*;
use mavlink::MavHeader;
use tauri::State;

mod image_reassembler;
mod mavlink_rx;
mod session;
mod telemetry;

use mavlink_rx::MavConn;
use session::SessionRecorder;
use telemetry::{TelemetrySnapshot, TelemetryState};

struct AppState {
    telemetry: Arc<Mutex<TelemetryState>>,
    session: Arc<Mutex<SessionRecorder>>,
    connected_addr: Mutex<Option<String>>,
    stop_flag: Arc<AtomicBool>,
    mav_conn: Mutex<Option<MavConn>>,
}

#[tauri::command]
fn connect(addr: String, state: State<'_, AppState>, app: tauri::AppHandle) -> Result<(), String> {
    {
        let conn = state.connected_addr.lock().unwrap();
        if conn.is_some() {
            return Err("Already connected. Disconnect first.".into());
        }
    }

    // Reset telemetry state
    {
        let mut st = state.telemetry.lock().unwrap();
        *st = TelemetryState::new(parse_baud(&addr));
    }

    state.stop_flag.store(false, Ordering::Relaxed);

    let conn = mavlink_rx::spawn_receiver(
        &addr,
        state.telemetry.clone(),
        state.session.clone(),
        state.stop_flag.clone(),
        app,
    )?;

    *state.mav_conn.lock().unwrap() = Some(conn);
    *state.connected_addr.lock().unwrap() = Some(addr);
    Ok(())
}

#[tauri::command]
fn disconnect(state: State<'_, AppState>) -> Result<(), String> {
    state.stop_flag.store(true, Ordering::Relaxed);
    state.session.lock().unwrap().stop();
    *state.mav_conn.lock().unwrap() = None;
    *state.connected_addr.lock().unwrap() = None;
    Ok(())
}

#[tauri::command]
fn get_connection_status(state: State<'_, AppState>) -> String {
    state
        .connected_addr
        .lock()
        .unwrap()
        .clone()
        .unwrap_or_else(|| "disconnected".into())
}

#[tauri::command]
fn list_serial_ports() -> Vec<String> {
    serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| p.port_name)
        .collect()
}

#[tauri::command]
fn start_logging(state: State<'_, AppState>) {
    state.session.lock().unwrap().start();
}

#[tauri::command]
fn stop_logging(state: State<'_, AppState>) {
    state.session.lock().unwrap().stop();
}

#[tauri::command]
fn is_logging(state: State<'_, AppState>) -> bool {
    state.session.lock().unwrap().is_logging()
}

#[tauri::command]
fn save_session(path: String, state: State<'_, AppState>) -> Result<(), String> {
    state.session.lock().unwrap().save(&path)
}

#[tauri::command]
fn load_session(path: String) -> Result<Vec<TelemetrySnapshot>, String> {
    session::load_from_file(&path)
}

#[tauri::command]
fn send_vehicle_command(cmd: String, state: State<'_, AppState>) -> Result<(), String> {
    let conn_guard = state.mav_conn.lock().unwrap();
    let conn = conn_guard.as_ref().ok_or("Not connected")?;

    let (name_str, value): (&str, i32) = match cmd.as_str() {
        "stop" => ("stop_rec", 1),
        "start" => ("stop_rec", 0),
        "shutdown" => ("shutdown", 1),
        "reboot" => ("reboot", 1),
        "calibrate_gyro" => ("cal_gyro", 1),
        "reset_orientation" => ("rst_ori", 1),
        s if s.starts_with("switch_cam:") => {
            let idx: i32 = s[11..].parse().map_err(|_| "Invalid camera index")?;
            ("sw_cam", idx)
        }
        _ => return Err(format!("Unknown command: {cmd}")),
    };

    let mut name_bytes = [0u8; 10];
    let src = name_str.as_bytes();
    name_bytes[..src.len()].copy_from_slice(src);

    let msg = MavMessage::NAMED_VALUE_INT(NAMED_VALUE_INT_DATA {
        time_boot_ms: 0,
        name: name_bytes.into(),
        value,
    });

    let header = MavHeader {
        system_id: 255,
        component_id: 190, // MAV_COMP_ID_MISSIONPLANNER
        sequence: 0,
    };

    // Send 3x for reliability over lossy radio link
    for _ in 0..3 {
        conn.send(&header, &msg)
            .map(|_| ())
            .map_err(|e| format!("Send failed: {e}"))?;
    }

    Ok(())
}

/// Extract baud rate from a serial address like "serial:/dev/cu.usbmodem0021:57600"
fn parse_baud(addr: &str) -> u32 {
    if addr.starts_with("serial:") {
        addr.rsplit(':').next().and_then(|s| s.parse().ok()).unwrap_or(0)
    } else {
        0
    }
}

pub fn run() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState {
            telemetry: Arc::new(Mutex::new(TelemetryState::new(0))),
            session: Arc::new(Mutex::new(SessionRecorder::new())),
            connected_addr: Mutex::new(None),
            stop_flag: Arc::new(AtomicBool::new(false)),
            mav_conn: Mutex::new(None),
        })
        .invoke_handler(tauri::generate_handler![
            connect,
            disconnect,
            get_connection_status,
            list_serial_ports,
            start_logging,
            stop_logging,
            is_logging,
            save_session,
            load_session,
            send_vehicle_command,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
