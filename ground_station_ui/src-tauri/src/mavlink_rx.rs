use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use mavlink::ardupilotmega::MavMessage;
use mavlink::MavConnection;
use tauri::{AppHandle, Emitter};

use crate::image_reassembler::ImageReassembler;
use crate::session::SessionRecorder;
use crate::telemetry::TelemetryState;

pub type MavConn = Arc<Box<dyn MavConnection<MavMessage> + Sync + Send>>;

pub fn spawn_receiver(
    addr: &str,
    state: Arc<Mutex<TelemetryState>>,
    session: Arc<Mutex<SessionRecorder>>,
    stop: Arc<AtomicBool>,
    app: AppHandle,
) -> Result<MavConn, String> {
    let conn = mavlink::connect::<MavMessage>(addr)
        .map_err(|e| format!("Failed to connect: {e}"))?;
    let conn: MavConn = Arc::new(Box::new(conn));

    let recv_conn = conn.clone();
    thread::spawn(move || {
        let mut last_emit = Instant::now();
        let mut reassembler = ImageReassembler::new();

        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }

            match recv_conn.recv() {
                Ok((header, msg)) => {
                    // Handle image messages
                    match &msg {
                        MavMessage::DATA_TRANSMISSION_HANDSHAKE(data) => {
                            reassembler.handle_handshake(
                                data.size,
                                data.width,
                                data.height,
                                data.packets,
                                data.jpg_quality,
                            );
                            // Still update telemetry for bandwidth tracking
                            state.lock().unwrap().update(&header, &msg);
                            continue;
                        }
                        MavMessage::ENCAPSULATED_DATA(data) => {
                            if let Some(jpeg) = reassembler.handle_data(data.seqnr, &data.data) {
                                let event = reassembler.make_event(jpeg);
                                let _ = app.emit("camera_frame", &event);
                            }
                            state.lock().unwrap().update(&header, &msg);
                            continue;
                        }
                        _ => {}
                    }

                    // Regular telemetry handling
                    let snapshot = {
                        let mut st = state.lock().unwrap();
                        st.update(&header, &msg);

                        if last_emit.elapsed() >= Duration::from_millis(50) {
                            Some(st.snapshot())
                        } else {
                            None
                        }
                    };

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

    Ok(conn)
}
