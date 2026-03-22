use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use mavlink::ardupilotmega::MavMessage;
use mavlink::peek_reader::PeekReader;

#[derive(Debug)]
pub enum VehicleCommand {
    StopRecording,
    StartRecording,
    Shutdown,
    Reboot,
    SwitchCamera(usize),
}

pub fn spawn_command_receiver(
    read_port: Box<dyn serialport::SerialPort>,
    tx: mpsc::Sender<VehicleCommand>,
) {
    thread::spawn(move || {
        println!("[cmd_rx] Listening for commands on radio link...");
        let mut reader = PeekReader::new(read_port);
        let mut msg_count: u64 = 0;

        loop {
            match mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
                Ok((_header, msg)) => {
                    msg_count += 1;

                    if let MavMessage::NAMED_VALUE_INT(data) = &msg {
                        let name = std::str::from_utf8(&*data.name)
                            .unwrap_or("")
                            .trim_matches('\0');

                        println!("[cmd_rx] NAMED_VALUE_INT: name={name:?} value={}", data.value);

                        let cmd = match (name, data.value) {
                            ("stop_rec", 1) => Some(VehicleCommand::StopRecording),
                            ("stop_rec", 0) => Some(VehicleCommand::StartRecording),
                            ("shutdown", 1) => Some(VehicleCommand::Shutdown),
                            ("reboot", 1)   => Some(VehicleCommand::Reboot),
                            ("sw_cam", idx)  => Some(VehicleCommand::SwitchCamera(idx as usize)),
                            _ => None,
                        };

                        if let Some(cmd) = cmd {
                            println!("[cmd_rx] Executing: {cmd:?}");
                            let _ = tx.send(cmd);
                        }
                    } else if msg_count <= 5 || msg_count % 100 == 0 {
                        println!("[cmd_rx] msg #{msg_count}: {:?} (not a command)", msg_type_name(&msg));
                    }
                }
                Err(mavlink::error::MessageReadError::Io(ref e))
                    if e.kind() == std::io::ErrorKind::TimedOut
                        || e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(e) => {
                    // Only log non-parse errors (parse errors are normal when
                    // scanning past our own outbound traffic)
                    if !matches!(e, mavlink::error::MessageReadError::Parse(_)) {
                        eprintln!("[cmd_rx] Read error: {e:?}");
                    }
                    thread::sleep(Duration::from_millis(10));
                }
            }
        }
    });
}

fn msg_type_name(msg: &MavMessage) -> &'static str {
    match msg {
        MavMessage::HEARTBEAT(_) => "HEARTBEAT",
        MavMessage::GLOBAL_POSITION_INT(_) => "GLOBAL_POSITION_INT",
        MavMessage::SCALED_IMU(_) => "SCALED_IMU",
        MavMessage::SCALED_PRESSURE(_) => "SCALED_PRESSURE",
        MavMessage::GPS_RAW_INT(_) => "GPS_RAW_INT",
        MavMessage::NAMED_VALUE_INT(_) => "NAMED_VALUE_INT",
        MavMessage::DATA_TRANSMISSION_HANDSHAKE(_) => "DATA_TRANSMISSION_HANDSHAKE",
        MavMessage::ENCAPSULATED_DATA(_) => "ENCAPSULATED_DATA",
        _ => "OTHER",
    }
}
