#[derive(Clone, Copy, PartialEq)]
pub enum Mode {
    Command,
    Passthrough,
    AtConfig,
}

#[derive(Clone, Copy)]
pub struct DataPacket {
    pub buf: [u8; 64],
    pub len: usize,
}

impl DataPacket {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; 64],
            len: 0,
        }
    }

    pub fn from_slice(data: &[u8]) -> Self {
        let mut pkt = Self::new();
        let n = data.len().min(64);
        pkt.buf[..n].copy_from_slice(&data[..n]);
        pkt.len = n;
        pkt
    }
}

/// Detects the passthrough escape sequence (3x Ctrl-A / 0x01).
pub struct EscapeDetector {
    count: usize,
}

impl EscapeDetector {
    const ESCAPE_BYTE: u8 = 0x01;
    const ESCAPE_COUNT: usize = 3;

    pub const fn new() -> Self {
        Self { count: 0 }
    }

    /// Feed a byte. Returns true if escape sequence is complete.
    pub fn feed(&mut self, byte: u8) -> bool {
        if byte == Self::ESCAPE_BYTE {
            self.count += 1;
            self.count >= Self::ESCAPE_COUNT
        } else {
            self.count = 0;
            false
        }
    }

    pub fn reset(&mut self) {
        self.count = 0;
    }
}
