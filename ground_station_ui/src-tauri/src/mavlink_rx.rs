use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;
use tauri::{AppHandle, Emitter};

use crate::session::SessionRecorder;
use crate::telemetry::TelemetryState;

pub fn spawn_receiver(
    addr: &str,
    state: Arc<Mutex<TelemetryState>>,
    session: Arc<Mutex<SessionRecorder>>,
    stop: Arc<AtomicBool>,
    app: AppHandle,
) -> Result<(), String> {
    let conn = mavlink::connect::<MavMessage>(addr)
        .map_err(|e| format!("Failed to connect: {e}"))?;
    let conn = Arc::new(conn);

    let recv_conn = conn.clone();
    thread::spawn(move || {
        let mut last_emit = Instant::now();

        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }

            match recv_conn.recv() {
                Ok((header, msg)) => {
                    // Update state and maybe take a snapshot (release lock before emitting)
                    let snapshot = {
                        let mut st = state.lock().unwrap();
                        st.update(&header, &msg);

                        if last_emit.elapsed() >= Duration::from_millis(100) {
                            Some(st.snapshot())
                        } else {
                            None
                        }
                    };

                    // Emit and record outside the state lock
                    if let Some(snapshot) = snapshot {
                        let _ = app.emit("telemetry", &snapshot);

                        let mut sess = session.lock().unwrap();
                        if sess.is_logging() {
                            sess.record(snapshot);
                        }

                        last_emit = Instant::now();
                    }
                }
                Err(mavlink::error::MessageReadError::Io(e))
                    if e.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    thread::sleep(Duration::from_millis(10));
                }
                Err(_) => {
                    if stop.load(Ordering::Relaxed) {
                        break;
                    }
                    thread::sleep(Duration::from_millis(100));
                }
            }
        }
    });

    // Keep connection alive (it's held by the Arc in the thread)
    drop(conn);

    Ok(())
}
