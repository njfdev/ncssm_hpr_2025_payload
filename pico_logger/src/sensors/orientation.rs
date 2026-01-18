/// Orientation Tracking Module
///
/// Tracks device orientation by integrating gyroscope readings over time.
/// Uses simple Euler angle integration (suitable for short-duration flights).
///
/// For long-duration or high-accuracy applications, consider implementing
/// a complementary filter or Madgwick/Mahony AHRS algorithm that fuses
/// accelerometer and magnetometer data.

use super::types::{SensorData, ImuCalibration, Orientation};

/// Orientation tracker that integrates gyroscope data
#[derive(Default)]
pub struct OrientationTracker {
    /// Current orientation (Euler angles in degrees)
    pub orientation: Orientation,
    /// Last update timestamp in milliseconds
    last_update_ms: u64,
    /// Whether the tracker has been initialized
    initialized: bool,
}

impl OrientationTracker {
    /// Create a new orientation tracker
    pub const fn new() -> Self {
        Self {
            orientation: Orientation {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            last_update_ms: 0,
            initialized: false,
        }
    }

    /// Reset orientation to zero (call when device is in known orientation)
    pub fn reset(&mut self) {
        self.orientation = Orientation::default();
        self.initialized = false;
    }

    /// Update orientation from sensor data
    ///
    /// `time_ms`: Current timestamp in milliseconds
    /// `data`: Sensor data with gyroscope readings
    /// `calib`: IMU calibration data for gyroscope bias correction
    ///
    /// Returns the current orientation
    pub fn update(&mut self, time_ms: u64, data: &SensorData, calib: &ImuCalibration) -> Orientation {
        if !self.initialized {
            self.last_update_ms = time_ms;
            self.initialized = true;
            return self.orientation;
        }

        // Calculate time delta in seconds
        let dt_ms = time_ms.saturating_sub(self.last_update_ms);
        if dt_ms == 0 {
            return self.orientation;
        }
        let dt = dt_ms as f32 / 1000.0;
        self.last_update_ms = time_ms;

        // Get calibrated gyroscope rates in degrees per second
        let gyro_x = data.gyro_x_calibrated(calib);
        let gyro_y = data.gyro_y_calibrated(calib);
        let gyro_z = data.gyro_z_calibrated(calib);

        // Simple Euler angle integration
        // Note: This works well for small angles and short durations
        // For large rotations, quaternions would be more accurate
        self.orientation.roll += gyro_x * dt;
        self.orientation.pitch += gyro_y * dt;
        self.orientation.yaw += gyro_z * dt;

        // Normalize yaw to [-180, 180]
        while self.orientation.yaw > 180.0 {
            self.orientation.yaw -= 360.0;
        }
        while self.orientation.yaw < -180.0 {
            self.orientation.yaw += 360.0;
        }

        self.orientation
    }

    /// Get current orientation without updating
    pub fn get(&self) -> Orientation {
        self.orientation
    }

    /// Initialize from accelerometer (estimates roll/pitch from gravity)
    ///
    /// Call this once at startup when device is relatively still.
    /// This gives a better initial orientation estimate than starting at zero.
    pub fn init_from_accel(&mut self, data: &SensorData, calib: &ImuCalibration) {
        let ax = data.accel_x_calibrated(calib);
        let ay = data.accel_y_calibrated(calib);
        let az = data.accel_z_calibrated(calib);

        // Calculate roll and pitch from accelerometer
        // Roll: rotation around X axis (tilt left/right)
        // Pitch: rotation around Y axis (tilt forward/backward)
        self.orientation.roll = libm::atan2f(ay, az) * 180.0 / core::f32::consts::PI;
        self.orientation.pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az)) * 180.0 / core::f32::consts::PI;
        // Yaw cannot be determined from accelerometer alone (needs magnetometer)
        self.orientation.yaw = 0.0;

        self.initialized = false; // Will be set true on first update() call
    }
}
