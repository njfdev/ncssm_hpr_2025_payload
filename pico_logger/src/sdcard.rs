/// SD Card logging module for Adafruit Feather RP2040 Adalogger
///
/// SPI0 Pinout:
/// - CLK  = GP18
/// - MOSI = GP19
/// - MISO = GP20
/// - CS   = GP23

use core::fmt::Write;
use embassy_rp::gpio::{Output, Level};
use embassy_rp::spi::{Spi, Config as SpiConfig, Blocking, ClkPin, MosiPin, MisoPin};
use embassy_rp::peripherals::SPI0;
use embassy_rp::Peri;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeManager, VolumeIdx, Mode, TimeSource, Timestamp};
use heapless::String;

use crate::sensors::SensorData;

/// Dummy time source - returns fixed timestamp
struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 55, // 2025
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Embassy delay adapter for embedded-hal DelayNs
pub struct Delay;

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // Busy-wait loop - crude but works for SD card init
        let cycles = ns / 100; // Rough approximation at ~125MHz
        for _ in 0..cycles {
            cortex_m::asm::nop();
        }
    }
}

type SpiDev = ExclusiveDevice<Spi<'static, SPI0, Blocking>, Output<'static>, Delay>;

/// SD card logger state
pub struct SdLogger {
    volume_mgr: VolumeManager<SdCard<SpiDev, Delay>, DummyTimeSource>,
}

impl SdLogger {
    /// Initialize SD card and return logger
    /// Takes ownership of SPI0 and the SD card pins
    pub fn new<C, MO, MI, CS>(
        spi0: Peri<'static, SPI0>,
        clk: Peri<'static, C>,
        mosi: Peri<'static, MO>,
        miso: Peri<'static, MI>,
        cs_pin: Peri<'static, CS>,
    ) -> Result<Self, &'static str>
    where
        C: ClkPin<SPI0> + 'static,
        MO: MosiPin<SPI0> + 'static,
        MI: MisoPin<SPI0> + 'static,
        CS: embassy_rp::gpio::Pin + 'static,
    {
        // Configure SPI at 400kHz for SD card init (required by spec)
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 400_000;

        let spi = Spi::new_blocking(spi0, clk, mosi, miso, spi_config);
        let cs = Output::new(cs_pin, Level::High);

        // Create SpiDevice with CS management
        let spi_dev = ExclusiveDevice::new(spi, cs, Delay).map_err(|_| "SPI device error")?;

        // Create SD card instance
        let sdcard = SdCard::new(spi_dev, Delay);

        // Get card size to verify communication
        let _size = sdcard.num_bytes().map_err(|_| "SD card not responding")?;

        // Create volume manager
        let volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);

        Ok(Self { volume_mgr })
    }

    /// Create a new log file with incrementing name (FLT000.CSV, FLT001.CSV, etc.)
    /// Returns the filename that was created
    pub fn create_log_file(&mut self) -> Result<heapless::String<12>, &'static str> {
        let mut volume = self.volume_mgr
            .open_volume(VolumeIdx(0))
            .map_err(|_| "Failed to open volume")?;

        let mut root_dir = volume
            .open_root_dir()
            .map_err(|_| "Failed to open root dir")?;

        // Find next available filename
        for i in 0..1000u16 {
            let mut name: String<12> = String::new();
            let _ = write!(name, "FLT{:03}.CSV", i);

            match root_dir.find_directory_entry(name.as_str()) {
                Err(embedded_sdmmc::Error::NotFound) => {
                    // This filename is available - create the file
                    let mut file = root_dir
                        .open_file_in_dir(name.as_str(), Mode::ReadWriteCreateOrAppend)
                        .map_err(|_| "Failed to create file")?;

                    // Write CSV header
                    file.write(b"counter,pressure,temp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z\n")
                        .map_err(|_| "Failed to write header")?;

                    file.flush().map_err(|_| "Failed to flush")?;

                    return Ok(name);
                }
                Ok(_) => continue, // File exists, try next
                Err(_) => return Err("Directory error"),
            }
        }

        Err("No available filenames")
    }

    /// Append a sensor data row to the currently open file
    pub fn log_data(&mut self, filename: &str, counter: u32, data: &SensorData) -> Result<(), &'static str> {
        let mut volume = self.volume_mgr
            .open_volume(VolumeIdx(0))
            .map_err(|_| "Failed to open volume")?;

        let mut root_dir = volume
            .open_root_dir()
            .map_err(|_| "Failed to open root dir")?;

        let mut file = root_dir
            .open_file_in_dir(filename, Mode::ReadWriteAppend)
            .map_err(|_| "Failed to open file")?;

        // Format CSV line
        let mut line: String<128> = String::new();
        let _ = write!(
            line,
            "{},{},{},{},{},{},{},{},{},{},{},{}\n",
            counter,
            data.pressure_raw,
            data.temp_raw,
            data.accel_x, data.accel_y, data.accel_z,
            data.gyro_x, data.gyro_y, data.gyro_z,
            data.mag_x, data.mag_y, data.mag_z,
        );

        file.write(line.as_bytes()).map_err(|_| "Write failed")?;

        // Flush periodically (every 100 writes would be handled by caller)
        // For now, always flush for safety
        file.flush().map_err(|_| "Flush failed")?;

        Ok(())
    }
}

/// Format SD card status message
pub fn format_sd_status(ok: bool, filename: Option<&str>) -> String<64> {
    let mut s = String::new();
    if ok {
        if let Some(name) = filename {
            let _ = write!(s, "SD:OK FILE:{}\r\n", name);
        } else {
            let _ = write!(s, "SD:OK\r\n");
        }
    } else {
        let _ = write!(s, "SD:ERR\r\n");
    }
    s
}
