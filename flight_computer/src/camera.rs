use std::collections::HashSet;
use std::io::Read;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc;
use std::thread;

use crate::config::Config;

/// Auto-detect USB video capture devices by scanning sysfs.
/// Skips non-USB devices (sunxi-vin) and metadata nodes (takes only the
/// lowest-numbered /dev/videoN per physical USB device).
pub fn detect_cameras() -> Vec<String> {
    let sysfs = "/sys/class/video4linux";
    let Ok(entries) = std::fs::read_dir(sysfs) else {
        eprintln!("[camera] Cannot read {sysfs}, falling back to manual config");
        return Vec::new();
    };

    // Collect (dev_path, name, physical_device_path) tuples
    let mut candidates: Vec<(String, String, String)> = Vec::new();

    for entry in entries.flatten() {
        let dev_name = entry.file_name().to_string_lossy().to_string();

        // Only consider /dev/videoN devices (skip v4l-subdev*, media*, etc.)
        if !dev_name.starts_with("video") {
            continue;
        }

        let name_file = entry.path().join("name");
        let Ok(name) = std::fs::read_to_string(&name_file) else { continue };
        let name = name.trim().to_string();

        // Skip non-USB devices (SoC camera pipelines)
        if name.contains("sunxi") || name.contains("Allwinner") || name.contains("vin_") {
            continue;
        }

        let dev_path = format!("/dev/{dev_name}");

        // Resolve physical device symlink to group capture + metadata nodes
        let device_link = entry.path().join("device");
        let phys = std::fs::read_link(&device_link)
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_default();

        candidates.push((dev_path, name, phys));
    }

    // Sort by path so /dev/video1 comes before /dev/video2, etc.
    candidates.sort_by(|a, b| a.0.cmp(&b.0));

    // Take only the first (capture) node per physical USB device
    let mut seen_phys = HashSet::new();
    let mut devices = Vec::new();
    for (dev_path, name, phys) in candidates {
        if seen_phys.insert(phys) {
            eprintln!("[camera] Detected: {dev_path} ({name})");
            devices.push(dev_path);
        }
    }

    devices
}

/// A single image frame ready for radio transmission.
pub struct CameraFrame {
    pub image_data: Vec<u8>,
    pub width: u16,
    pub height: u16,
    pub quality: u8,
    pub mime_type: &'static str,
}

pub struct CameraManager {
    ffmpeg_process: Child,
    /// Label for log messages (e.g. "cam0", "cam1")
    pub label: String,
}

impl CameraManager {
    /// Start a camera that records locally at full resolution.
    ///
    /// If `with_stream` is true, also produces a low-res image pipe for radio
    /// transmission and returns `Some(Receiver<CameraFrame>)`.
    pub fn start(
        config: &Config,
        device: &str,
        label: &str,
        with_stream: bool,
    ) -> std::io::Result<(Self, Option<mpsc::Receiver<CameraFrame>>)> {
        std::fs::create_dir_all(&config.recording_dir)?;

        let segment_pattern = format!(
            "{}/{}_seg_%Y%m%d_%H%M%S.ts",
            config.recording_dir, label
        );

        let use_webp = config.stream_format == "webp";

        let mut args: Vec<String> = Vec::new();

        // Overwrite output files without asking
        args.extend(["-y".into()]);

        // Input
        if config.test_source {
            args.extend([
                "-f".into(), "lavfi".into(),
                "-i".into(), format!("testsrc=size=640x480:rate={}", config.recording_fps),
            ]);
        } else {
            args.extend([
                "-f".into(), "v4l2".into(),
                "-framerate".into(), config.recording_fps.to_string(),
                "-i".into(), device.to_string(),
            ]);
        }

        // Output 1: local recording (power-safe segmented MPEG-TS)
        let encoder_args: Vec<String> = if config.video_encoder == "libx264" {
            vec!["-preset".into(), "ultrafast".into()]
        } else {
            vec![]
        };

        args.extend(["-map".into(), "0:v".into()]);
        args.extend(["-c:v".into(), config.video_encoder.clone()]);
        args.extend(encoder_args);
        args.extend([
            "-b:v".into(), "8M".into(),
            "-force_key_frames".into(), "expr:gte(t,n_forced*1)".into(),
            "-f".into(), "segment".into(),
            "-segment_time".into(), config.segment_duration.to_string(),
            "-segment_format".into(), "mpegts".into(),
            "-fflags".into(), "+flush_packets".into(),
            "-reset_timestamps".into(), "1".into(),
            "-strftime".into(), "1".into(),
            segment_pattern,
        ]);

        // Output 2 (optional): low-res grayscale image stream to pipe
        if with_stream {
            let vf = format!(
                "scale={}:{},format=gray",
                config.stream_width, config.stream_height
            );

            args.extend([
                "-map".into(), "0:v".into(),
                "-vf".into(), vf,
            ]);

            if use_webp {
                args.extend([
                    "-c:v".into(), "libwebp".into(),
                    "-quality".into(), config.jpeg_quality.to_string(),
                    "-r".into(), config.stream_fps.to_string(),
                    "-f".into(), "image2pipe".into(),
                    "pipe:1".into(),
                ]);
            } else {
                let qscale = (31 - (config.jpeg_quality as u32 * 30 / 100)).max(1).min(31);
                args.extend([
                    "-c:v".into(), "mjpeg".into(),
                    "-q:v".into(), qscale.to_string(),
                    "-r".into(), config.stream_fps.to_string(),
                    "-f".into(), "image2pipe".into(),
                    "-vcodec".into(), "mjpeg".into(),
                    "pipe:1".into(),
                ]);
            }
        }

        eprintln!("[{label}] launching: {} {}", config.ffmpeg_bin, args.join(" "));

        let mut child = Command::new(&config.ffmpeg_bin)
            .args(&args)
            .stdout(if with_stream { Stdio::piped() } else { Stdio::null() })
            .stderr(Stdio::piped())
            .spawn()?;

        let frame_rx = if with_stream {
            let stdout = child.stdout.take().ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::Other, "Failed to capture ffmpeg stdout")
            })?;

            let (tx, rx) = mpsc::sync_channel::<CameraFrame>(2);
            let width = config.stream_width as u16;
            let height = config.stream_height as u16;
            let quality = config.jpeg_quality;
            let log_label = label.to_string();

            thread::spawn(move || {
                if use_webp {
                    parse_webp_stream(stdout, tx, width, height, quality);
                } else {
                    parse_jpeg_stream(stdout, tx, width, height, quality);
                }
                eprintln!("[{log_label}] stdout reader exited");
            });

            Some(rx)
        } else {
            None
        };

        // Spawn stderr reader thread — log ffmpeg output
        if let Some(stderr) = child.stderr.take() {
            let log_label = label.to_string();
            thread::spawn(move || {
                let reader = std::io::BufReader::new(stderr);
                use std::io::BufRead;
                for line in reader.lines() {
                    match line {
                        Ok(line) => eprintln!("[{log_label}-ffmpeg] {line}"),
                        Err(_) => break,
                    }
                }
            });
        }

        Ok((
            Self {
                ffmpeg_process: child,
                label: label.to_string(),
            },
            frame_rx,
        ))
    }
}

impl Drop for CameraManager {
    fn drop(&mut self) {
        eprintln!("[{}] Stopping camera recording...", self.label);
        let _ = self.ffmpeg_process.kill();
        let _ = self.ffmpeg_process.wait();
        eprintln!("[{}] Camera recording stopped.", self.label);
    }
}

/// Parse a stream of WebP images by RIFF header.
/// Each WebP frame starts with "RIFF" + 4-byte LE file size.
/// Total frame size = 8 + file_size bytes.
fn parse_webp_stream(
    mut reader: impl Read,
    tx: mpsc::SyncSender<CameraFrame>,
    width: u16,
    height: u16,
    quality: u8,
) {
    let mut header = [0u8; 8];
    loop {
        // Read the 8-byte RIFF header
        if read_exact(&mut reader, &mut header).is_err() {
            break;
        }

        if &header[0..4] != b"RIFF" {
            eprintln!("[camera] expected RIFF header, got {:?}, resync", &header[0..4]);
            // Try to resync by scanning for RIFF
            if let Some(data) = resync_riff(&mut reader, &header) {
                let frame = CameraFrame {
                    image_data: data,
                    width,
                    height,
                    quality,
                    mime_type: "image/webp",
                };
                let _ = tx.try_send(frame);
            }
            continue;
        }

        let file_size = u32::from_le_bytes([header[4], header[5], header[6], header[7]]) as usize;

        if file_size > 512 * 1024 {
            eprintln!("[camera] WebP frame too large ({file_size} bytes), skipping");
            continue;
        }

        let mut frame_data = vec![0u8; 8 + file_size];
        frame_data[..8].copy_from_slice(&header);
        if read_exact(&mut reader, &mut frame_data[8..]).is_err() {
            break;
        }

        let frame = CameraFrame {
            image_data: frame_data,
            width,
            height,
            quality,
            mime_type: "image/webp",
        };
        let _ = tx.try_send(frame);
    }
}

/// Try to resync to next RIFF header after corruption.
fn resync_riff(reader: &mut impl Read, leftover: &[u8]) -> Option<Vec<u8>> {
    let mut window = [0u8; 4];
    window.copy_from_slice(&leftover[4..8]);
    let mut byte = [0u8; 1];
    for _ in 0..64 * 1024 {
        if reader.read_exact(&mut byte).is_err() {
            return None;
        }
        window[0] = window[1];
        window[1] = window[2];
        window[2] = window[3];
        window[3] = byte[0];
        if &window == b"RIFF" {
            // Found RIFF, read the rest
            let mut size_bytes = [0u8; 4];
            if reader.read_exact(&mut size_bytes).is_err() {
                return None;
            }
            let file_size = u32::from_le_bytes(size_bytes) as usize;
            if file_size > 512 * 1024 {
                return None;
            }
            let mut data = vec![0u8; 8 + file_size];
            data[0..4].copy_from_slice(b"RIFF");
            data[4..8].copy_from_slice(&size_bytes);
            if reader.read_exact(&mut data[8..]).is_err() {
                return None;
            }
            return Some(data);
        }
    }
    None
}

/// Helper: read exact bytes, return Err on EOF.
fn read_exact(reader: &mut impl Read, buf: &mut [u8]) -> Result<(), ()> {
    let mut filled = 0;
    while filled < buf.len() {
        match reader.read(&mut buf[filled..]) {
            Ok(0) => return Err(()),
            Ok(n) => filled += n,
            Err(e) => {
                eprintln!("[camera] read error: {e}");
                return Err(());
            }
        }
    }
    Ok(())
}

/// Parse a stream of concatenated JPEG images by SOI (0xFFD8) and EOI (0xFFD9) markers.
fn parse_jpeg_stream(
    mut reader: impl Read,
    tx: mpsc::SyncSender<CameraFrame>,
    width: u16,
    height: u16,
    quality: u8,
) {
    let mut buf = [0u8; 8192];
    let mut acc: Vec<u8> = Vec::with_capacity(32768);
    let mut in_frame = false;

    loop {
        let n = match reader.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => n,
            Err(e) => {
                eprintln!("[camera] read error: {e}");
                break;
            }
        };

        for &byte in &buf[..n] {
            acc.push(byte);

            if acc.len() < 2 {
                continue;
            }
            let len = acc.len();
            let prev = acc[len - 2];
            let curr = acc[len - 1];

            // Detect SOI marker (0xFF 0xD8) — start of new frame
            if !in_frame && prev == 0xFF && curr == 0xD8 {
                acc.clear();
                acc.push(0xFF);
                acc.push(0xD8);
                in_frame = true;
                continue;
            }

            // Detect EOI marker (0xFF 0xD9) — end of frame
            if in_frame && prev == 0xFF && curr == 0xD9 {
                let frame = CameraFrame {
                    image_data: std::mem::take(&mut acc),
                    width,
                    height,
                    quality,
                    mime_type: "image/jpeg",
                };
                let _ = tx.try_send(frame);
                in_frame = false;
            }
        }

        // Safety: reset if accumulator grows too large without finding EOI
        if acc.len() > 512 * 1024 {
            eprintln!("[camera] accumulator overflow ({} bytes), resetting", acc.len());
            acc.clear();
            in_frame = false;
        }
    }
}
