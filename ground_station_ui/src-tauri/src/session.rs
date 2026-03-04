use std::fs;

use crate::telemetry::TelemetrySnapshot;

pub struct SessionRecorder {
    logging: bool,
    records: Vec<TelemetrySnapshot>,
}

impl SessionRecorder {
    pub fn new() -> Self {
        Self {
            logging: false,
            records: Vec::new(),
        }
    }

    pub fn start(&mut self) {
        self.records.clear();
        self.logging = true;
    }

    pub fn stop(&mut self) {
        self.logging = false;
    }

    pub fn is_logging(&self) -> bool {
        self.logging
    }

    pub fn record(&mut self, snapshot: TelemetrySnapshot) {
        self.records.push(snapshot);
    }

    pub fn save(&self, path: &str) -> Result<(), String> {
        let json = serde_json::to_string_pretty(&self.records)
            .map_err(|e| format!("Serialize error: {e}"))?;
        fs::write(path, json).map_err(|e| format!("Write error: {e}"))?;
        Ok(())
    }
}

pub fn load_from_file(path: &str) -> Result<Vec<TelemetrySnapshot>, String> {
    let data = fs::read_to_string(path).map_err(|e| format!("Read error: {e}"))?;
    serde_json::from_str(&data).map_err(|e| format!("Parse error: {e}"))
}
