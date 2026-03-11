use std::sync::{mpsc, Arc};
use std::thread;
use std::time::Duration;

use mavlink::ardupilotmega::*;

use crate::camera::CameraFrame;
use crate::MavSerial;

const CHUNK_SIZE: usize = 253; // ENCAPSULATED_DATA payload size

pub fn spawn(
    mav: Arc<MavSerial>,
    frame_rx: mpsc::Receiver<CameraFrame>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        loop {
            // Block until a frame arrives
            let frame = match frame_rx.recv() {
                Ok(f) => f,
                Err(_) => break, // channel closed
            };

            // Drain any stale queued frames — always send the freshest
            let frame = drain_to_latest(frame, &frame_rx);

            let data = &frame.image_data;
            let total_size = data.len() as u32;
            let num_packets = ((data.len() + CHUNK_SIZE - 1) / CHUNK_SIZE) as u16;

            // Send handshake announcing the frame
            if let Err(e) = mav.send(
                &MavMessage::DATA_TRANSMISSION_HANDSHAKE(DATA_TRANSMISSION_HANDSHAKE_DATA {
                    size: total_size,
                    width: frame.width,
                    height: frame.height,
                    packets: num_packets,
                    mavtype: MavlinkDataStreamType::MAVLINK_DATA_STREAM_IMG_JPEG,
                    payload: CHUNK_SIZE as u8,
                    jpg_quality: frame.quality,
                }),
            ) {
                eprintln!("[frame_sender] handshake failed: {e}");
                thread::sleep(Duration::from_millis(100));
                continue;
            }

            // Send data chunks with pacing
            let mut send_failed = false;
            for (seq, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
                let mut payload = [0u8; 253];
                payload[..chunk.len()].copy_from_slice(chunk);

                if let Err(e) = mav.send(
                    &MavMessage::ENCAPSULATED_DATA(ENCAPSULATED_DATA_DATA {
                        seqnr: seq as u16,
                        data: payload,
                    }),
                ) {
                    eprintln!("[frame_sender] chunk {seq} failed: {e}");
                    send_failed = true;
                    break;
                }

                // Rate limiter in MavSerial handles pacing automatically
            }

            if send_failed {
                thread::sleep(Duration::from_millis(200));
                continue;
            }

            println!(
                "[frame_sender] {}x{} {} {} bytes ({} pkts)",
                frame.width, frame.height, frame.mime_type, total_size, num_packets
            );
        }
    })
}

/// Drain the channel and return the most recent frame, dropping stale ones.
fn drain_to_latest(mut latest: CameraFrame, rx: &mpsc::Receiver<CameraFrame>) -> CameraFrame {
    loop {
        match rx.try_recv() {
            Ok(newer) => latest = newer,
            Err(_) => return latest,
        }
    }
}
