/// Shared sensor data types

/// Sensor data structure containing readings from all sensors
#[derive(Default, Clone)]
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
