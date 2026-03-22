use std::process::{Child, Command, Stdio};
use std::thread;

use crate::config::Config;

pub struct AudioRecorder {
    ffmpeg_process: Child,
}

impl AudioRecorder {
    pub fn start(config: &Config) -> std::io::Result<Self> {
        std::fs::create_dir_all(&config.recording_dir)?;

        let segment_pattern = format!(
            "{}/audio_%Y%m%d_%H%M%S.wav",
            config.recording_dir
        );

        let mut args: Vec<String> = vec![
            "-y".into(),
            "-f".into(), config.audio_format.clone(),
            "-i".into(), config.audio_device.clone(),
            "-c:a".into(), "pcm_s16le".into(),
            "-ar".into(), "48000".into(),
            "-ac".into(), "1".into(),
            "-f".into(), "segment".into(),
            "-segment_time".into(), config.segment_duration.to_string(),
            "-segment_format".into(), "wav".into(),
            "-strftime".into(), "1".into(),
            segment_pattern,
        ];

        // Suppress banner noise
        args.insert(0, "-hide_banner".into());

        eprintln!("[audio] launching: {} {}", config.ffmpeg_bin, args.join(" "));

        let mut child = Command::new(&config.ffmpeg_bin)
            .args(&args)
            .stdout(Stdio::null())
            .stderr(Stdio::piped())
            .spawn()?;

        // Log ffmpeg stderr
        if let Some(stderr) = child.stderr.take() {
            thread::spawn(move || {
                use std::io::{BufRead, BufReader};
                let reader = BufReader::new(stderr);
                for line in reader.lines() {
                    match line {
                        Ok(line) => eprintln!("[audio-ffmpeg] {line}"),
                        Err(_) => break,
                    }
                }
            });
        }

        Ok(Self { ffmpeg_process: child })
    }
}

impl Drop for AudioRecorder {
    fn drop(&mut self) {
        eprintln!("[audio] Stopping audio recording...");
        let _ = self.ffmpeg_process.kill();
        let _ = self.ffmpeg_process.wait();
        eprintln!("[audio] Audio recording stopped.");
    }
}
