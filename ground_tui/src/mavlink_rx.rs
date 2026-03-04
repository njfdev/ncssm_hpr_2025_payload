use std::sync::mpsc;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use mavlink::ardupilotmega::MavMessage;
use mavlink::{MavConnection, MavHeader};

pub fn spawn_receiver(
    addr: &str,
    tx: mpsc::Sender<(MavHeader, MavMessage)>,
) -> Arc<dyn MavConnection<MavMessage> + Send + Sync> {
    let conn = mavlink::connect::<MavMessage>(addr).expect("failed to connect to MAVLink");
    let conn = Arc::new(conn);

    let recv_conn = conn.clone();
    thread::spawn(move || loop {
        match recv_conn.recv() {
            Ok((header, msg)) => {
                if tx.send((header, msg)).is_err() {
                    break; // main thread dropped the receiver
                }
            }
            Err(mavlink::error::MessageReadError::Io(e))
                if e.kind() == std::io::ErrorKind::WouldBlock =>
            {
                thread::sleep(Duration::from_millis(10));
            }
            Err(_) => {
                thread::sleep(Duration::from_millis(100));
            }
        }
    });

    conn
}
