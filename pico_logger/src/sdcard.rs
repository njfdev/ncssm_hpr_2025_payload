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

use crate::sensors::LogEntry;

/// SPI frequency for SD card initialization (required by SD spec)
const SD_SPI_FREQ_INIT: u32 = 400_000;

/// SPI frequency for normal SD card operations (much faster after init)
const SD_SPI_FREQ_FAST: u32 = 16_000_000;

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
    /// Initializes at 400kHz (required by SD spec) then increases to 16MHz for fast operations
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
        // Configure SPI at 400kHz for SD card initialization (required by SD spec)
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = SD_SPI_FREQ_INIT;

        let spi = Spi::new_blocking(spi0, clk, mosi, miso, spi_config);
        let cs = Output::new(cs_pin, Level::High);

        // Create SpiDevice with CS management
        let spi_dev = ExclusiveDevice::new(spi, cs, Delay).map_err(|_| "SPI device error")?;

        // Create SD card instance
        let sdcard = SdCard::new(spi_dev, Delay);

        // Get card size to verify communication (this completes card initialization)
        let _size = sdcard.num_bytes().map_err(|_| "SD card not responding")?;

        // Create volume manager
        let mut volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);

        // After successful init, increase SPI speed to 16MHz for faster operations
        // This is safe because SD cards support high speeds after initialization
        volume_mgr.device().spi(|spi_dev| {
            spi_dev.bus_mut().set_frequency(SD_SPI_FREQ_FAST);
        });

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

                    // Write CSV header with units
                    file.write(b"time_s,pressure_Pa,temp_C,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mag_x_uT,mag_y_uT,mag_z_uT,roll_deg,pitch_deg,yaw_deg,lin_x_g,lin_y_g,lin_z_g\n")
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

    /// Append a batch of log entries to the file
    /// Opens the file once, writes all entries, then closes.
    /// entries: slice of (time_ms, LogEntry) tuples
    pub fn log_batch(&mut self, filename: &str, entries: &[(u64, LogEntry)]) -> Result<(), &'static str> {
        if entries.is_empty() {
            return Ok(());
        }

        let mut volume = self.volume_mgr
            .open_volume(VolumeIdx(0))
            .map_err(|_| "Failed to open volume")?;

        let mut root_dir = volume
            .open_root_dir()
            .map_err(|_| "Failed to open root dir")?;

        let mut file = root_dir
            .open_file_in_dir(filename, Mode::ReadWriteAppend)
            .map_err(|_| "Failed to open file")?;

        // Write all entries
        for (time_ms, entry) in entries {
            let time_s = *time_ms as f32 / 1000.0;
            let data = &entry.sensor_data;

            let mut line: String<256> = String::new();
            let _ = write!(
                line,
                "{:.3},{:.1},{:.2},{:.4},{:.4},{:.4},{:.2},{:.2},{:.2},{:.1},{:.1},{:.1},{:.1},{:.1},{:.1},{:.4},{:.4},{:.4}\n",
                time_s,
                data.pressure_pa(),
                data.temp_celsius(),
                data.accel_x_g(), data.accel_y_g(), data.accel_z_g(),
                data.gyro_x_dps(), data.gyro_y_dps(), data.gyro_z_dps(),
                data.mag_x_ut(), data.mag_y_ut(), data.mag_z_ut(),
                entry.orientation.roll, entry.orientation.pitch, entry.orientation.yaw,
                entry.linear_accel_x, entry.linear_accel_y, entry.linear_accel_z,
            );

            file.write(line.as_bytes()).map_err(|_| "Write failed")?;
        }

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
