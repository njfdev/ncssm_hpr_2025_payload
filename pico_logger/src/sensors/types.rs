/// Shared sensor data types

/// Sensor data structure containing readings from all sensors
#[derive(Default, Clone, Copy)]
pub struct SensorData {
    // BMP390
    pub pressure_raw: u32,
    pub temp_raw: u32,

    // LSM6DSOX
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
    /// Convert pressure from raw ADC value to Pascals (Pa)
    /// BMP390 outputs 24-bit ADC values. This is a simplified linear conversion.
    /// For precise readings, use the factory calibration coefficients.
    /// Returns pressure in Pa (divide by 100 for hPa/mbar)
    pub fn pressure_pa(&self) -> f32 {
        // Simplified conversion: raw value is roughly in 1/64 Pa
        // Typical sea level: ~6,500,000 raw = ~101,325 Pa
        self.pressure_raw as f32 / 64.0
    }

    /// Convert temperature from raw ADC value to degrees Celsius
    /// Simplified linear conversion without factory calibration
    pub fn temp_celsius(&self) -> f32 {
        // Simplified conversion: raw value is roughly in 1/64 °C * 256
        // Typical room temp: ~8,600,000 raw = ~20-25°C
        self.temp_raw as f32 / 65536.0 - 40.0 + (self.temp_raw as f32 / 6553600.0 * 85.0)
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
}
