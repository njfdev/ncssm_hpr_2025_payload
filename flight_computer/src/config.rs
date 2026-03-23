use clap::Parser;

#[derive(Parser, Debug, Clone)]
#[command(name = "flight_computer", about = "NCSSM HPR 2025 Flight Computer")]
pub struct Config {
    /// MAVLink connection address
    #[arg(long, default_value = "serial:/dev/ttyUSB0:115200")]
    pub mavlink_addr: String,

    /// Camera v4l2 device paths (specify multiple --camera-device for multi-camera)
    #[arg(long = "camera-device")]
    pub camera_devices: Vec<String>,

    /// Which camera index to stream over radio (0-based)
    #[arg(long, default_value_t = 0)]
    pub stream_camera: usize,

    /// Data output directory (video recordings + telemetry logs)
    #[arg(long, default_value = "./flight_data")]
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
    #[arg(long, default_value_t = 30)]
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
    #[arg(long, default_value_t = 6000)]
    pub radio_bps: u32,

    /// ffmpeg binary name (use "ffmpeg" for standard, "ffmpeg-rk" for hw accel)
    #[arg(long, default_value = "ffmpeg")]
    pub ffmpeg_bin: String,

    /// Video encoder for local recording (e.g. h264_v4l2m2m, libx264)
    #[arg(long, default_value = "libx264")]
    pub video_encoder: String,

    /// UART device for pico logger connection
    #[arg(long, default_value = "/dev/ttyAS2")]
    pub pico_uart: String,

    /// Baud rate for pico logger UART
    #[arg(long, default_value_t = 115200)]
    pub pico_baud: u32,

    /// Disable pico logger reader (for testing without hardware)
    #[arg(long)]
    pub no_pico: bool,

    /// Disable audio recording
    #[arg(long)]
    pub no_audio: bool,

    /// Audio input format for ffmpeg (e.g. "pulse", "alsa")
    #[arg(long, default_value = "alsa")]
    pub audio_format: String,

    /// Audio input device for ffmpeg (e.g. "hw:4,0" for UMM-6 mic)
    #[arg(long, default_value = "hw:4,0")]
    pub audio_device: String,
}
