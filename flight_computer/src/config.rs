use clap::Parser;

#[derive(Parser, Debug, Clone)]
#[command(name = "flight_computer", about = "NCSSM HPR 2025 Flight Computer")]
pub struct Config {
    /// MAVLink connection address
    #[arg(long, default_value = "serial:/dev/ttyUSB0:115200")]
    pub mavlink_addr: String,

    /// Camera v4l2 device path
    #[arg(long, default_value = "/dev/video0")]
    pub camera_device: String,

    /// Local recording output directory
    #[arg(long, default_value = "./recordings")]
    pub recording_dir: String,

    /// Local recording FPS
    #[arg(long, default_value_t = 120)]
    pub recording_fps: u32,

    /// Radio stream resolution width
    #[arg(long, default_value_t = 320)]
    pub stream_width: u32,

    /// Radio stream resolution height
    #[arg(long, default_value_t = 240)]
    pub stream_height: u32,

    /// Image quality for radio stream (1-100)
    #[arg(long, default_value_t = 10)]
    pub jpeg_quality: u8,

    /// Radio stream image format: "webp" or "jpeg"
    #[arg(long, default_value = "webp")]
    pub stream_format: String,

    /// MPEG-TS segment duration in seconds (lower = more power-safe)
    #[arg(long, default_value_t = 2)]
    pub segment_duration: u32,

    /// Radio stream output FPS from ffmpeg (actual send rate is bandwidth-limited)
    #[arg(long, default_value_t = 5)]
    pub stream_fps: u32,

    /// Telemetry send rate in Hz
    #[arg(long, default_value_t = 10)]
    pub telemetry_hz: u32,

    /// Disable camera (telemetry only)
    #[arg(long)]
    pub no_camera: bool,

    /// Use ffmpeg test source instead of real camera (for development)
    #[arg(long)]
    pub test_source: bool,

    /// Max radio throughput in bytes/sec (rate limiter to avoid overdriving)
    #[arg(long, default_value_t = 4000)]
    pub radio_bps: u32,

    /// ffmpeg binary name (use "ffmpeg" for standard, "ffmpeg-rk" for hw accel)
    #[arg(long, default_value = "ffmpeg")]
    pub ffmpeg_bin: String,

    /// Video encoder for local recording (e.g. h264_v4l2m2m, libx264)
    #[arg(long, default_value = "libx264")]
    pub video_encoder: String,
}
