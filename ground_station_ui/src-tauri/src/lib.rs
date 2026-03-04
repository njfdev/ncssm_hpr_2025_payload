use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use tauri::State;

mod mavlink_rx;
mod session;
mod telemetry;

use session::SessionRecorder;
use telemetry::{TelemetrySnapshot, TelemetryState};

struct AppState {
    telemetry: Arc<Mutex<TelemetryState>>,
    session: Arc<Mutex<SessionRecorder>>,
    connected_addr: Mutex<Option<String>>,
    stop_flag: Arc<AtomicBool>,
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

    mavlink_rx::spawn_receiver(
        &addr,
        state.telemetry.clone(),
        state.session.clone(),
        state.stop_flag.clone(),
        app,
    )?;

    *state.connected_addr.lock().unwrap() = Some(addr);
    Ok(())
}

#[tauri::command]
fn disconnect(state: State<'_, AppState>) -> Result<(), String> {
    state.stop_flag.store(true, Ordering::Relaxed);
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
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
