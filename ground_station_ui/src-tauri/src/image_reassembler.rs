use std::time::Instant;

use serde::Serialize;

/// Reassembles image frames from DATA_TRANSMISSION_HANDSHAKE + ENCAPSULATED_DATA.
pub struct ImageReassembler {
    expected_size: u32,
    expected_packets: u16,
    width: u16,
    height: u16,
    quality: u8,
    buffer: Vec<u8>,
    received: Vec<bool>,
    packets_received: u16,

    pub frames_completed: u64,
    pub frames_dropped: u64,
    last_frame_time: Option<Instant>,
    frame_intervals: Vec<f32>,
}

#[derive(Clone, Serialize)]
pub struct CameraFrameEvent {
    pub image_base64: String,
    pub mime_type: String,
    pub width: u16,
    pub height: u16,
    pub quality: u8,
    pub size_bytes: usize,
    pub fps: f32,
}

impl ImageReassembler {
    pub fn new() -> Self {
        Self {
            expected_size: 0,
            expected_packets: 0,
            width: 0,
            height: 0,
            quality: 0,
            buffer: Vec::new(),
            received: Vec::new(),
            packets_received: 0,
            frames_completed: 0,
            frames_dropped: 0,
            last_frame_time: None,
            frame_intervals: Vec::new(),
        }
    }

    /// Handle DATA_TRANSMISSION_HANDSHAKE. Drops any in-progress frame.
    pub fn handle_handshake(
        &mut self,
        size: u32,
        width: u16,
        height: u16,
        packets: u16,
        quality: u8,
    ) {
        if self.expected_packets > 0 && self.packets_received < self.expected_packets {
            self.frames_dropped += 1;
        }

        self.expected_size = size;
        self.expected_packets = packets;
        self.width = width;
        self.height = height;
        self.quality = quality;
        self.buffer = vec![0u8; size as usize];
        self.received = vec![false; packets as usize];
        self.packets_received = 0;
    }

    /// Handle ENCAPSULATED_DATA. Returns completed image bytes when all packets received.
    pub fn handle_data(&mut self, seqnr: u16, data: &[u8; 253]) -> Option<Vec<u8>> {
        if self.expected_packets == 0 {
            return None;
        }

        let idx = seqnr as usize;
        if idx >= self.received.len() || self.received[idx] {
            return None;
        }

        let offset = idx * 253;
        let remaining = self.expected_size as usize - offset;
        let copy_len = remaining.min(253);
        if offset + copy_len <= self.buffer.len() {
            self.buffer[offset..offset + copy_len].copy_from_slice(&data[..copy_len]);
        }

        self.received[idx] = true;
        self.packets_received += 1;

        if self.packets_received == self.expected_packets {
            self.frames_completed += 1;
            let image = std::mem::take(&mut self.buffer);
            self.expected_packets = 0;
            self.packets_received = 0;
            Some(image)
        } else {
            None
        }
    }

    /// Build a CameraFrameEvent from completed image data.
    /// Detects MIME type from magic bytes.
    pub fn make_event(&mut self, image: Vec<u8>) -> CameraFrameEvent {
        use base64::Engine;

        let mime_type = detect_mime_type(&image);

        let now = Instant::now();
        if let Some(last) = self.last_frame_time {
            let dt = now.duration_since(last).as_secs_f32();
            self.frame_intervals.push(dt);
            if self.frame_intervals.len() > 10 {
                self.frame_intervals.remove(0);
            }
        }
        self.last_frame_time = Some(now);

        let fps = if self.frame_intervals.is_empty() {
            0.0
        } else {
            let avg: f32 =
                self.frame_intervals.iter().sum::<f32>() / self.frame_intervals.len() as f32;
            if avg > 0.0 { 1.0 / avg } else { 0.0 }
        };

        let size_bytes = image.len();
        CameraFrameEvent {
            image_base64: base64::engine::general_purpose::STANDARD.encode(&image),
            mime_type: mime_type.to_string(),
            width: self.width,
            height: self.height,
            quality: self.quality,
            size_bytes,
            fps,
        }
    }
}

/// Detect image MIME type from magic bytes.
fn detect_mime_type(data: &[u8]) -> &'static str {
    if data.len() >= 2 && data[0] == 0xFF && data[1] == 0xD8 {
        return "image/jpeg";
    }
    if data.len() >= 12 && &data[0..4] == b"RIFF" && &data[8..12] == b"WEBP" {
        return "image/webp";
    }
    // fallback
    "image/jpeg"
}
