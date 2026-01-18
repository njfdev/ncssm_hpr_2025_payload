/// Shared sensor data types

/// IMU calibration data for accelerometer and gyroscope
#[derive(Clone, Copy)]
pub struct ImuCalibration {
    // Accelerometer bias (raw LSB values to subtract)
    pub accel_bias_x: i16,
    pub accel_bias_y: i16,
    pub accel_bias_z: i16,  // Note: Z should be offset so that 1g reads correctly

    // Gyroscope bias (raw LSB values to subtract)
    pub gyro_bias_x: i16,
    pub gyro_bias_y: i16,
    pub gyro_bias_z: i16,
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self {
            accel_bias_x: 0,
            accel_bias_y: 0,
            accel_bias_z: 0,
            gyro_bias_x: 0,
            gyro_bias_y: 0,
            gyro_bias_z: 0,
        }
    }
}

/// Orientation angles in degrees (Euler angles)
#[derive(Default, Clone, Copy)]
pub struct Orientation {
    pub roll: f32,   // Rotation around X axis
    pub pitch: f32,  // Rotation around Y axis
    pub yaw: f32,    // Rotation around Z axis
}

/// Sensor data structure containing readings from all sensors
#[derive(Default, Clone, Copy)]
pub struct SensorData {
    // BMP390
    pub pressure_raw: u32,
    pub temp_raw: u32,

    // LSM6DSOX (raw values)
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,

    // LIS3MDL
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
}

impl SensorData {
    /// Get pressure in Pascals (Pa)
    /// BMP390 driver stores compensated pressure directly in Pascals
    /// Divide by 100 for hPa/mbar
    pub fn pressure_pa(&self) -> f32 {
        self.pressure_raw as f32
    }

    /// Get temperature in degrees Celsius
    /// BMP390 driver stores compensated temperature as centidegrees (25.5°C = 2550)
    pub fn temp_celsius(&self) -> f32 {
        self.temp_raw as f32 / 100.0
    }

    /// Convert accelerometer X to g (gravitational acceleration)
    /// LSM6DSOX at ±16g: sensitivity = 0.488 mg/LSB
    pub fn accel_x_g(&self) -> f32 {
        self.accel_x as f32 * 0.000488
    }

    /// Convert accelerometer Y to g
    pub fn accel_y_g(&self) -> f32 {
        self.accel_y as f32 * 0.000488
    }

    /// Convert accelerometer Z to g
    pub fn accel_z_g(&self) -> f32 {
        self.accel_z as f32 * 0.000488
    }

    /// Convert gyroscope X to degrees per second (dps)
    /// LSM6DSOX at ±2000 dps: sensitivity = 70 mdps/LSB
    pub fn gyro_x_dps(&self) -> f32 {
        self.gyro_x as f32 * 0.070
    }

    /// Convert gyroscope Y to degrees per second
    pub fn gyro_y_dps(&self) -> f32 {
        self.gyro_y as f32 * 0.070
    }

    /// Convert gyroscope Z to degrees per second
    pub fn gyro_z_dps(&self) -> f32 {
        self.gyro_z as f32 * 0.070
    }

    /// Convert magnetometer X to microtesla (µT)
    /// LIS3MDL at ±4 gauss: sensitivity = 6842 LSB/gauss = 68.42 LSB/µT
    /// (1 gauss = 100 µT)
    pub fn mag_x_ut(&self) -> f32 {
        self.mag_x as f32 / 6.842
    }

    /// Convert magnetometer Y to microtesla
    pub fn mag_y_ut(&self) -> f32 {
        self.mag_y as f32 / 6.842
    }

    /// Convert magnetometer Z to microtesla
    pub fn mag_z_ut(&self) -> f32 {
        self.mag_z as f32 / 6.842
    }

    // === Calibrated IMU methods ===

    /// Get calibrated accelerometer X in g
    pub fn accel_x_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.accel_x - calib.accel_bias_x) as f32 * 0.000488
    }

    /// Get calibrated accelerometer Y in g
    pub fn accel_y_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.accel_y - calib.accel_bias_y) as f32 * 0.000488
    }

    /// Get calibrated accelerometer Z in g
    pub fn accel_z_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.accel_z - calib.accel_bias_z) as f32 * 0.000488
    }

    /// Get calibrated gyroscope X in dps
    pub fn gyro_x_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.gyro_x - calib.gyro_bias_x) as f32 * 0.070
    }

    /// Get calibrated gyroscope Y in dps
    pub fn gyro_y_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.gyro_y - calib.gyro_bias_y) as f32 * 0.070
    }

    /// Get calibrated gyroscope Z in dps
    pub fn gyro_z_calibrated(&self, calib: &ImuCalibration) -> f32 {
        (self.gyro_z - calib.gyro_bias_z) as f32 * 0.070
    }

    // === Derived measurements ===

    /// Get linear acceleration X (gravity removed) in g
    /// Assumes device is oriented with Z pointing up when stationary
    /// For arbitrary orientation, use with orientation estimation
    pub fn linear_accel_x(&self, calib: &ImuCalibration) -> f32 {
        self.accel_x_calibrated(calib) // X has no gravity component when level
    }

    /// Get linear acceleration Y (gravity removed) in g
    pub fn linear_accel_y(&self, calib: &ImuCalibration) -> f32 {
        self.accel_y_calibrated(calib) // Y has no gravity component when level
    }

    /// Get linear acceleration Z (gravity removed) in g
    /// When device is stationary and level, this should be ~0
    pub fn linear_accel_z(&self, calib: &ImuCalibration) -> f32 {
        self.accel_z_calibrated(calib) - 1.0 // Subtract 1g gravity
    }

    /// Get linear acceleration magnitude in g
    pub fn linear_accel_magnitude(&self, calib: &ImuCalibration) -> f32 {
        let x = self.linear_accel_x(calib);
        let y = self.linear_accel_y(calib);
        let z = self.linear_accel_z(calib);
        libm::sqrtf(x * x + y * y + z * z)
    }
}

impl ImuCalibration {
    /// Expected 1g value in raw LSB at ±16g scale
    /// 1g / 0.000488 g/LSB ≈ 2049 LSB
    pub const ONE_G_LSB: i16 = 2049;
}

/// Extended log entry with raw sensor data plus derived measurements
/// This is passed to core1 for SD card logging
#[derive(Default, Clone, Copy)]
pub struct LogEntry {
    pub sensor_data: SensorData,
    pub orientation: Orientation,
    pub linear_accel_x: f32,
    pub linear_accel_y: f32,
    pub linear_accel_z: f32,
}
