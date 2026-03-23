use std::io::Read;
use std::time::Duration;

use clap::Parser;
use payload_types::{TelemetryPacket, MAX_ENCODED_SIZE};

#[derive(Parser)]
#[command(name = "telemetry_decoder", about = "Decode telemetry from ground station USB")]
struct Args {
    /// Serial port (auto-detects /dev/cu.usbmodem* if omitted)
    #[arg(short, long)]
    port: Option<String>,

    /// Baud rate
    #[arg(short, long, default_value_t = 57600)]
    baud: u32,

    /// Show raw hex bytes for each frame
    #[arg(long)]
    raw: bool,

    /// Log raw bytes to file
    #[arg(long)]
    log: Option<String>,
}

fn auto_detect_port() -> Option<String> {
    let ports = serialport::available_ports().ok()?;
    // Prefer /dev/cu.usbmodem* (ground station CDC ACM)
    for p in &ports {
        if p.port_name.contains("cu.usbmodem") {
            return Some(p.port_name.clone());
        }
    }
    None
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

    eprintln!("Opening {} at {} baud...", port_name, args.baud);

    let mut port = serialport::new(&port_name, args.baud)
        .timeout(Duration::from_millis(100))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open {}: {}", port_name, e);
            std::process::exit(1);
        });

    // Set DTR/RTS so the Pico's CDC ACM wait_connection() returns
    if let Err(e) = port.write_data_terminal_ready(true) {
        eprintln!("Warning: could not set DTR: {}", e);
    }
    if let Err(e) = port.write_request_to_send(true) {
        eprintln!("Warning: could not set RTS: {}", e);
    }

    // Clear any stale buffers
    let _ = port.clear(serialport::ClearBuffer::All);

    eprintln!("Listening for telemetry packets...");
    eprintln!("  bytes_to_read: {:?}", port.bytes_to_read());

    let mut log_file = args.log.as_ref().map(|path| {
        std::fs::File::create(path).unwrap_or_else(|e| {
            eprintln!("Failed to create log file {}: {}", path, e);
            std::process::exit(1);
        })
    });

    // COBS framing: 0x00 is the delimiter between frames
    let mut frame_buf = [0u8; MAX_ENCODED_SIZE * 2];
    let mut frame_len = 0usize;
    let mut read_buf = [0u8; 256];
    let mut packets_decoded = 0u64;
    let mut errors = 0u64;
    let mut read_count = 0u64;

    loop {
        read_count += 1;
        if read_count % 50 == 0 {
            // Every ~5 seconds (50 * 100ms), print diagnostics
            eprintln!("[diag] reads: {}  bytes_to_read: {:?}  decoded: {}  errors: {}",
                read_count, port.bytes_to_read(), packets_decoded, errors);
        }

        let n = match port.read(&mut read_buf) {
            Ok(n) => n,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) => {
                eprintln!("Read error (kind={:?}): {}", e.kind(), e);
                break;
            }
        };

        // Log raw bytes if requested
        if let Some(ref mut f) = log_file {
            use std::io::Write;
            let _ = f.write_all(&read_buf[..n]);
        }

        // Print raw data as hex + ASCII
        let data = &read_buf[..n];
        let ascii: String = data.iter().map(|&b| {
            if b.is_ascii_graphic() || b == b' ' { b as char } else { '.' }
        }).collect();
        let hex: String = data.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(" ");
        eprintln!("RX {} bytes | {} | {}", n, hex, ascii);

        for &byte in &read_buf[..n] {
            if byte == 0x00 {
                // End of COBS frame
                if frame_len > 0 {
                    // Append the sentinel for postcard's from_bytes_cobs
                    if frame_len < frame_buf.len() {
                        frame_buf[frame_len] = 0x00;
                        frame_len += 1;
                    }

                    if args.raw {
                        eprint!("RAW [{}]: ", frame_len);
                        for b in &frame_buf[..frame_len] {
                            eprint!("{:02x} ", b);
                        }
                        eprintln!();
                    }

                    match TelemetryPacket::decode_cobs(&mut frame_buf[..frame_len]) {
                        Ok(pkt) => {
                            packets_decoded += 1;
                            println!("{}", pkt);
                        }
                        Err(e) => {
                            errors += 1;
                            eprintln!("Decode error (frame {} bytes): {:?}", frame_len, e);
                        }
                    }

                    frame_len = 0;
                }
                // else: skip leading/consecutive delimiters
            } else {
                if frame_len < frame_buf.len() {
                    frame_buf[frame_len] = byte;
                    frame_len += 1;
                } else {
                    // Frame too large, discard
                    eprintln!("Frame overflow, discarding {} bytes", frame_len);
                    frame_len = 0;
                    errors += 1;
                }
            }
        }

        // Print stats periodically on stderr (every 100 packets)
        if packets_decoded > 0 && packets_decoded % 100 == 0 {
            eprintln!("[stats] decoded: {}  errors: {}", packets_decoded, errors);
        }
    }
}
