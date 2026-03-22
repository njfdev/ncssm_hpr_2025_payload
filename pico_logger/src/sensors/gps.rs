/// UBlox GPS NMEA parser for UART1 RX
///
/// Parses GGA (position/fix) and RMC (speed/course) sentences from any
/// GNSS talker (GP, GN, GL, etc.). Shares the latest fix via a static
/// blocking mutex so the main telemetry loop can read it cheaply.

use core::cell::Cell;
use embassy_rp::uart::{Async, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Timer;

/// Parsed GPS data shared between the reader task and main loop.
#[derive(Default, Clone, Copy)]
pub struct GpsData {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
    pub fix_quality: u8,
    pub satellites: u8,
    pub hdop_e2: u16,
    pub speed_cms: u16,
    pub course_e2: u16,
}

/// Latest GPS fix, updated by the reader task.
pub static GPS_STATE: Mutex<CriticalSectionRawMutex, Cell<GpsData>> =
    Mutex::new(Cell::new(GpsData {
        lat_e7: 0,
        lon_e7: 0,
        alt_mm: 0,
        fix_quality: 0,
        satellites: 0,
        hdop_e2: 0,
        speed_cms: 0,
        course_e2: 0,
    }));

/// Read the latest GPS data from the shared state.
pub fn latest() -> GpsData {
    GPS_STATE.lock(|c| c.get())
}

/// Background task that continuously reads NMEA sentences from the GPS UART.
#[embassy_executor::task]
pub async fn gps_reader_task(mut rx: UartRx<'static, Async>) {
    let mut line_buf = [0u8; 128];
    let mut line_len: usize = 0;
    let mut byte = [0u8; 1];

    loop {
        match rx.read(&mut byte).await {
            Ok(()) => {
                if byte[0] == b'\n' {
                    if line_len > 6 {
                        parse_nmea_line(&line_buf[..line_len]);
                    }
                    line_len = 0;
                } else if byte[0] != b'\r' {
                    if line_len < line_buf.len() {
                        line_buf[line_len] = byte[0];
                        line_len += 1;
                    }
                }
            }
            Err(_) => {
                Timer::after_millis(10).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// NMEA parsing helpers
// ---------------------------------------------------------------------------

fn parse_nmea_line(line: &[u8]) {
    // Must start with '$' and have at least "$xxGGA," (7 chars)
    if line.len() < 7 || line[0] != b'$' {
        return;
    }
    // Sentence ID is at bytes 3..6 (skip '$' + 2-char talker ID)
    let id = &line[3..6];
    let fields = &line[7..]; // everything after "$xxGGA,"

    if id == b"GGA" {
        parse_gga(fields);
    } else if id == b"RMC" {
        parse_rmc(fields);
    }
}

/// Parse GGA sentence fields (after the sentence type comma).
///
/// Fields: time, lat, N/S, lon, E/W, fix, sats, hdop, alt, M, ...
fn parse_gga(data: &[u8]) {
    let mut f = Fields::new(data);
    let _time = f.next();       // 0: time
    let lat_raw = f.next();     // 1: latitude ddmm.mmmm
    let lat_hem = f.next();     // 2: N/S
    let lon_raw = f.next();     // 3: longitude dddmm.mmmm
    let lon_hem = f.next();     // 4: E/W
    let fix_raw = f.next();     // 5: fix quality
    let sats_raw = f.next();    // 6: satellites
    let hdop_raw = f.next();    // 7: HDOP
    let alt_raw = f.next();     // 8: altitude (meters)

    let fix = parse_u32(fix_raw) as u8;
    if fix == 0 {
        // No fix — clear position but keep fix=0
        GPS_STATE.lock(|c| {
            let mut d = c.get();
            d.fix_quality = 0;
            d.satellites = 0;
            c.set(d);
        });
        return;
    }

    let lat_e7 = parse_nmea_coord(lat_raw, first_byte(lat_hem), true);
    let lon_e7 = parse_nmea_coord(lon_raw, first_byte(lon_hem), false);
    let alt_mm = parse_fixed_i32(alt_raw, 3); // meters → mm (3 extra decimal places)
    let sats = parse_u32(sats_raw) as u8;
    let hdop_e2 = parse_fixed_u16(hdop_raw, 2);

    GPS_STATE.lock(|c| {
        let mut d = c.get();
        d.lat_e7 = lat_e7;
        d.lon_e7 = lon_e7;
        d.alt_mm = alt_mm;
        d.fix_quality = fix;
        d.satellites = sats;
        d.hdop_e2 = hdop_e2;
        c.set(d);
    });
}

/// Parse RMC sentence fields (after the sentence type comma).
///
/// Fields: time, status, lat, N/S, lon, E/W, speed(knots), course, date, ...
fn parse_rmc(data: &[u8]) {
    let mut f = Fields::new(data);
    let _time = f.next();       // 0: time
    let status = f.next();      // 1: A=active, V=void
    let _lat = f.next();        // 2: lat (already from GGA)
    let _lat_hem = f.next();    // 3: N/S
    let _lon = f.next();        // 4: lon
    let _lon_hem = f.next();    // 5: E/W
    let speed_raw = f.next();   // 6: speed in knots
    let course_raw = f.next();  // 7: course in degrees

    if first_byte(status) != b'A' {
        return; // void fix
    }

    // Speed: knots → cm/s.  1 knot ≈ 51.4444 cm/s
    // Parse speed as fixed-point with 2 decimals (hundredths of knots),
    // then multiply: speed_knots_e2 * 51444 / 100000 ≈ cm/s
    let speed_knots_e2 = parse_fixed_u32(speed_raw, 2) as u64;
    let speed_cms = (speed_knots_e2 * 51444 / 100_000) as u16;

    let course_e2 = parse_fixed_u16(course_raw, 2);

    GPS_STATE.lock(|c| {
        let mut d = c.get();
        d.speed_cms = speed_cms;
        d.course_e2 = course_e2;
        c.set(d);
    });
}

// ---------------------------------------------------------------------------
// Low-level parsing helpers (no_std, no alloc)
// ---------------------------------------------------------------------------

/// Simple comma-delimited field iterator over a byte slice.
struct Fields<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Fields<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    /// Return the next field (up to comma, '*', or end). Empty slice if exhausted.
    fn next(&mut self) -> &'a [u8] {
        let start = self.pos;
        while self.pos < self.data.len()
            && self.data[self.pos] != b','
            && self.data[self.pos] != b'*'
        {
            self.pos += 1;
        }
        let field = &self.data[start..self.pos];
        if self.pos < self.data.len() {
            self.pos += 1; // skip delimiter
        }
        field
    }
}

fn first_byte(s: &[u8]) -> u8 {
    if s.is_empty() { 0 } else { s[0] }
}

/// Parse an unsigned integer from ASCII digits.
fn parse_u32(s: &[u8]) -> u32 {
    let mut val: u32 = 0;
    for &b in s {
        if b >= b'0' && b <= b'9' {
            val = val.wrapping_mul(10).wrapping_add((b - b'0') as u32);
        }
    }
    val
}

/// Parse a decimal number and return it as a u32 scaled to `extra_frac` extra
/// fractional digits beyond what's in the string.
///
/// E.g., parse_fixed_u32(b"12.34", 1) → 12340  (one extra digit)
fn parse_fixed_u32(s: &[u8], target_frac: u32) -> u32 {
    if s.is_empty() {
        return 0;
    }
    let dot_pos = s.iter().position(|&b| b == b'.');
    let int_end = dot_pos.unwrap_or(s.len());
    let int_part = parse_u32(&s[..int_end]);

    let (frac_val, actual_frac) = if let Some(dp) = dot_pos {
        let frac_str = &s[dp + 1..];
        (parse_u32(frac_str), frac_str.len() as u32)
    } else {
        (0u32, 0u32)
    };

    let scale = pow10(target_frac);
    let frac_scaled = if actual_frac >= target_frac {
        frac_val / pow10(actual_frac - target_frac)
    } else {
        frac_val * pow10(target_frac - actual_frac)
    };

    int_part * scale + frac_scaled
}

fn parse_fixed_u16(s: &[u8], target_frac: u32) -> u16 {
    parse_fixed_u32(s, target_frac) as u16
}

fn parse_fixed_i32(s: &[u8], target_frac: u32) -> i32 {
    if s.is_empty() {
        return 0;
    }
    let negative = s[0] == b'-';
    let start = if negative { 1 } else { 0 };
    let val = parse_fixed_u32(&s[start..], target_frac) as i32;
    if negative { -val } else { val }
}

/// Convert NMEA coordinate (ddmm.mmmm or dddmm.mmmm) to degrees * 1e7.
///
/// `is_lat`: true for latitude (2-digit degrees), false for longitude (3-digit).
fn parse_nmea_coord(field: &[u8], hemisphere: u8, is_lat: bool) -> i32 {
    if field.is_empty() {
        return 0;
    }

    let dot_pos = match field.iter().position(|&b| b == b'.') {
        Some(p) => p,
        None => return 0,
    };

    // Degrees portion: 2 digits for lat, 3 for lon
    let deg_digits = if is_lat { 2 } else { 3 };
    if dot_pos < deg_digits + 2 {
        return 0; // malformed
    }
    let degrees = parse_u32(&field[..deg_digits]) as i64;

    // Minutes: from deg_digits to end, including decimal
    let min_whole = parse_u32(&field[deg_digits..dot_pos]) as i64;
    let frac_str = &field[dot_pos + 1..];
    let frac_val = parse_u32(frac_str) as i64;
    let frac_digits = frac_str.len() as u32;

    // Normalize fractional minutes to 5 decimal places
    let frac_e5 = if frac_digits >= 5 {
        frac_val / pow10(frac_digits - 5) as i64
    } else {
        frac_val * pow10(5 - frac_digits) as i64
    };

    // total_min_e5 = minutes * 100_000
    let total_min_e5 = min_whole * 100_000 + frac_e5;

    // degrees_e7 = degrees * 10_000_000 + total_min_e5 * 5 / 3
    // (because min_e5 * 1e7 / (60 * 1e5) = min_e5 * 100/60 = min_e5 * 5/3)
    let result = degrees * 10_000_000 + (total_min_e5 * 5 + 1) / 3;
    let result = result as i32;

    if hemisphere == b'S' || hemisphere == b'W' {
        -result
    } else {
        result
    }
}

fn pow10(n: u32) -> u32 {
    match n {
        0 => 1,
        1 => 10,
        2 => 100,
        3 => 1_000,
        4 => 10_000,
        5 => 100_000,
        6 => 1_000_000,
        7 => 10_000_000,
        _ => {
            let mut v = 1u32;
            for _ in 0..n {
                v *= 10;
            }
            v
        }
    }
}
