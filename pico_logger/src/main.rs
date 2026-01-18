#![no_std]
#![no_main]

mod sensors;
mod sdcard;
mod telemetry;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c, Config as I2cConfig};
use embassy_rp::peripherals::{USB, I2C0, I2C1, SPI0, PIN_18, PIN_19, PIN_20, PIN_23};
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::UsbDevice;
use embassy_time::{Timer, Instant};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use panic_probe as _;

use sensors::{SensorData, ImuCalibration, LogEntry};
use sensors::orientation::OrientationTracker;
use sdcard::{SdLogger, format_sd_status};
use telemetry::{format_status, format_telemetry, format_calibration, format_telemetry_extended};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Pico Logger - 2025 NCSSM HPR Payload"),
    embassy_rp::binary_info::rp_program_description!(c"Payload data logger firmware for RP2040."),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

/// Commands sent to core1 for SD card operations
#[derive(Clone)]
pub enum SdCommand {
    /// Create a new log file
    CreateFile,
    /// Log a batch of sensor data entries
    LogBatch([(u64, LogEntry); 10], usize), // (buffer, count)
    /// Flush and stop logging
    StopLogging,
}

/// Responses from core1
#[derive(Clone)]
pub enum SdResponse {
    /// File created successfully with filename
    FileCreated(heapless::String<12>),
    /// File creation failed
    FileError,
    /// Batch written successfully
    BatchWritten,
    /// Write failed
    WriteError,
    /// Logging stopped
    Stopped,
    /// SD card not initialized
    NotInitialized,
}

// Channels for core0 <-> core1 communication
// Command channel: core0 sends commands to core1
static SD_CMD_CHANNEL: Channel<CriticalSectionRawMutex, SdCommand, 4> = Channel::new();
// Response channel: core1 sends responses to core0
static SD_RESP_CHANNEL: Channel<CriticalSectionRawMutex, SdResponse, 4> = Channel::new();
// Data channel: high-throughput channel for sensor data during recording
// Larger buffer (64 entries) to absorb SD write latency
static SD_DATA_CHANNEL: Channel<CriticalSectionRawMutex, (u64, LogEntry), 64> = Channel::new();

// Stack for core1
static mut CORE1_STACK: Stack<8192> = Stack::new();

// Peripherals for SD card - transferred from core0 to core1
use embassy_rp::Peri;
type SdPeripherals = (
    Peri<'static, SPI0>,
    Peri<'static, PIN_18>,
    Peri<'static, PIN_19>,
    Peri<'static, PIN_20>,
    Peri<'static, PIN_23>,
);
static mut SD_PERIPHERALS: Option<SdPeripherals> = None;

/// Core1 entry point - handles all SD card operations
/// This function never returns (returns !)
fn core1_main() -> ! {
    // Small delay for hardware to stabilize
    for _ in 0..1_000_000 {
        cortex_m::asm::nop();
    }

    // Get peripherals from the static cell (set by core0 before spawning)
    let (spi0, clk, mosi, miso, cs_pin) = unsafe {
        (*core::ptr::addr_of_mut!(SD_PERIPHERALS)).take().unwrap_unchecked()
    };

    // Initialize SD card on core1
    let sd_result = SdLogger::new(spi0, clk, mosi, miso, cs_pin);

    let mut sd_logger = match sd_result {
        Ok(logger) => Some(logger),
        Err(_) => None,
    };

    let mut current_filename: Option<heapless::String<12>> = None;
    let mut batch_buffer: [(u64, LogEntry); 10] = Default::default();
    let mut batch_idx: usize = 0;

    loop {
        // Check for commands (non-blocking)
        if let Ok(cmd) = SD_CMD_CHANNEL.try_receive() {
            match cmd {
                SdCommand::CreateFile => {
                    if let Some(ref mut logger) = sd_logger {
                        match logger.create_log_file() {
                            Ok(filename) => {
                                current_filename = Some(filename.clone());
                                let _ = SD_RESP_CHANNEL.try_send(SdResponse::FileCreated(filename));
                            }
                            Err(_) => {
                                let _ = SD_RESP_CHANNEL.try_send(SdResponse::FileError);
                            }
                        }
                    } else {
                        let _ = SD_RESP_CHANNEL.try_send(SdResponse::NotInitialized);
                    }
                }
                SdCommand::LogBatch(buffer, count) => {
                    if let (Some(logger), Some(filename)) = (&mut sd_logger, &current_filename) {
                        if logger.log_batch(filename.as_str(), &buffer[..count]).is_ok() {
                            let _ = SD_RESP_CHANNEL.try_send(SdResponse::BatchWritten);
                        } else {
                            let _ = SD_RESP_CHANNEL.try_send(SdResponse::WriteError);
                        }
                    }
                }
                SdCommand::StopLogging => {
                    // Flush any remaining data
                    if batch_idx > 0 {
                        if let (Some(logger), Some(filename)) = (&mut sd_logger, &current_filename) {
                            let _ = logger.log_batch(filename.as_str(), &batch_buffer[..batch_idx]);
                        }
                        batch_idx = 0;
                    }
                    current_filename = None;
                    let _ = SD_RESP_CHANNEL.try_send(SdResponse::Stopped);
                }
            }
        }

        // Check for data entries (non-blocking) - collect into batch
        while let Ok((time_ms, data)) = SD_DATA_CHANNEL.try_receive() {
            if current_filename.is_some() {
                batch_buffer[batch_idx] = (time_ms, data);
                batch_idx += 1;

                // Write batch when full
                if batch_idx >= 10 {
                    if let (Some(logger), Some(filename)) = (&mut sd_logger, &current_filename) {
                        let _ = logger.log_batch(filename.as_str(), &batch_buffer[..batch_idx]);
                    }
                    batch_idx = 0;
                }
            }
        }

        // Small delay to avoid busy-spinning
        for _ in 0..10_000 {
            cortex_m::asm::nop();
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Store SD card peripherals in static for core1 to access
    unsafe {
        *core::ptr::addr_of_mut!(SD_PERIPHERALS) = Some((p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, p.PIN_23));
    }

    // Spawn core1 - it will handle all SD card operations
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        core1_main,
    );

    // USB driver
    let driver = Driver::new(p.USB, Irqs);

    // USB config
    let mut config = embassy_usb::Config::new(0x2e8a, 0x000a);
    config.manufacturer = Some("NCSSM Rocketry");
    config.product = Some("Pico Logger");
    config.serial_number = Some("001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // USB buffers
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let state = STATE.init(State::new());

    // Build USB device
    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        config_desc,
        bos_desc,
        &mut [],
        control_buf,
    );

    // Create CDC ACM class
    let mut class = CdcAcmClass::new(&mut builder, state, 64);

    // Build and spawn USB device
    let usb = builder.build();
    spawner.spawn(usb_task(usb)).unwrap();

    // I2C config
    let i2c_config = I2cConfig::default();

    // I2C0 for IMU (LSM6DSOX + LIS3MDL): SCL=GP29, SDA=GP28
    let mut i2c0 = I2c::new_async(p.I2C0, p.PIN_29, p.PIN_28, Irqs, i2c_config);

    // I2C1 for Barometer (BMP390): SCL=GP3, SDA=GP2
    let mut i2c1 = I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, i2c_config);

    // Give core1 time to initialize SD card
    Timer::after_millis(500).await;

    // Give sensors time to power up
    Timer::after_millis(100).await;

    // Initialize sensors
    let bmp390_ok = sensors::bmp390::init(&mut i2c1).await.is_ok();
    let lsm6dsox_ok = sensors::lsm6dsox::init(&mut i2c0).await.is_ok();
    let lis3mdl_ok = sensors::lis3mdl::init(&mut i2c0).await.is_ok();

    let mut cmd_buf = [0u8; 64];
    let mut sensor_data = SensorData::default();
    let mut counter: u32 = 0;
    let mut logging_active = false;
    let mut log_filename: Option<heapless::String<12>> = None;
    // Track if SD is available (based on responses from core1)
    let mut sd_available = true;
    // IMU calibration data
    let mut imu_calib = ImuCalibration::default();
    // Orientation tracker
    let mut orientation_tracker = OrientationTracker::new();

    loop {
        class.wait_connection().await;

        loop {
            // Wait for a command
            match class.read_packet(&mut cmd_buf).await {
                Ok(n) if n > 0 => {
                    let cmd = &cmd_buf[..n];

                    if cmd.starts_with(b"BOOTSEL") || cmd.starts_with(b"bootsel") {
                        reset_to_usb_boot(0, 0);
                    } else if cmd.starts_with(b"PING") || cmd.starts_with(b"ping") {
                        let _ = class.write_packet(b"PONG\r\n").await;
                    } else if cmd.starts_with(b"SENSORS") || cmd.starts_with(b"sensors") {
                        let msg = format_status(bmp390_ok, lsm6dsox_ok, lis3mdl_ok);
                        let _ = class.write_packet(msg.as_bytes()).await;
                    } else if cmd.starts_with(b"SDSTATUS") || cmd.starts_with(b"sdstatus") {
                        let msg = format_sd_status(sd_available, log_filename.as_ref().map(|s| s.as_str()));
                        let _ = class.write_packet(msg.as_bytes()).await;
                    } else if cmd.starts_with(b"START") || cmd.starts_with(b"start") {
                        let _ = class.write_packet(b"STREAMING\r\n").await;

                        // Stream indefinitely until STOP received (no SD logging)
                        'streaming: loop {
                            if bmp390_ok {
                                let _ = sensors::bmp390::read(&mut i2c1, &mut sensor_data).await;
                            }
                            if lsm6dsox_ok {
                                let _ = sensors::lsm6dsox::read(&mut i2c0, &mut sensor_data).await;
                            }
                            if lis3mdl_ok {
                                let _ = sensors::lis3mdl::read(&mut i2c0, &mut sensor_data).await;
                            }

                            counter = counter.wrapping_add(1);
                            let msg = format_telemetry(counter, &sensor_data);
                            if class.write_packet(msg.as_bytes()).await.is_err() {
                                break 'streaming;
                            }

                            Timer::after_millis(100).await;

                            use embassy_time::with_timeout;
                            use embassy_time::Duration;
                            if let Ok(Ok(n)) = with_timeout(Duration::from_millis(1), class.read_packet(&mut cmd_buf)).await {
                                if n > 0 {
                                    let stop_cmd = &cmd_buf[..n];
                                    if stop_cmd.starts_with(b"STOP") || stop_cmd.starts_with(b"stop") {
                                        let _ = class.write_packet(b"STOPPED\r\n").await;
                                        break 'streaming;
                                    } else if stop_cmd.starts_with(b"BOOTSEL") || stop_cmd.starts_with(b"bootsel") {
                                        reset_to_usb_boot(0, 0);
                                    }
                                }
                            }
                        }
                    } else if cmd.starts_with(b"READ") || cmd.starts_with(b"read") {
                        if bmp390_ok {
                            let _ = sensors::bmp390::read(&mut i2c1, &mut sensor_data).await;
                        }
                        if lsm6dsox_ok {
                            let _ = sensors::lsm6dsox::read(&mut i2c0, &mut sensor_data).await;
                        }
                        if lis3mdl_ok {
                            let _ = sensors::lis3mdl::read(&mut i2c0, &mut sensor_data).await;
                        }
                        counter = counter.wrapping_add(1);
                        let msg = format_telemetry(counter, &sensor_data);
                        let _ = class.write_packet(msg.as_bytes()).await;
                    } else if cmd.starts_with(b"CALIBRATE") || cmd.starts_with(b"calibrate") {
                        // Run IMU calibration - device must be stationary and level
                        let _ = class.write_packet(b"CALIBRATING...\r\n").await;
                        match sensors::calibration::calibrate_imu(&mut i2c0).await {
                            Ok(calib) => {
                                imu_calib = calib;
                                // Also initialize orientation from accelerometer
                                if lsm6dsox_ok {
                                    let _ = sensors::lsm6dsox::read(&mut i2c0, &mut sensor_data).await;
                                    orientation_tracker.init_from_accel(&sensor_data, &imu_calib);
                                }
                                let msg = format_calibration(&imu_calib);
                                let _ = class.write_packet(msg.as_bytes()).await;
                            }
                            Err(e) => {
                                let _ = class.write_packet(b"CALIB:ERR ").await;
                                let _ = class.write_packet(e.as_bytes()).await;
                                let _ = class.write_packet(b"\r\n").await;
                            }
                        }
                    } else if cmd.starts_with(b"LOG") || cmd.starts_with(b"log") {
                        // Request core1 to create a log file
                        let _ = SD_CMD_CHANNEL.try_send(SdCommand::CreateFile);

                        // Wait for response (with timeout)
                        use embassy_time::with_timeout;
                        use embassy_time::Duration;
                        match with_timeout(Duration::from_millis(2000), SD_RESP_CHANNEL.receive()).await {
                            Ok(SdResponse::FileCreated(filename)) => {
                                log_filename = Some(filename.clone());
                                logging_active = true;
                                sd_available = true;
                                let msg = format_sd_status(true, Some(filename.as_str()));
                                let _ = class.write_packet(msg.as_bytes()).await;
                            }
                            Ok(SdResponse::NotInitialized) => {
                                sd_available = false;
                                let _ = class.write_packet(b"SD:NOT_INIT\r\n").await;
                            }
                            _ => {
                                let _ = class.write_packet(b"SD:FILE_ERR\r\n").await;
                            }
                        }
                    } else if cmd.starts_with(b"STOPLOG") || cmd.starts_with(b"stoplog") {
                        let _ = SD_CMD_CHANNEL.try_send(SdCommand::StopLogging);
                        logging_active = false;
                        let _ = class.write_packet(b"SD:LOGGING_STOPPED\r\n").await;
                    } else if cmd.starts_with(b"RECORD") || cmd.starts_with(b"record") {
                        // Create log file if not already logging
                        if !logging_active {
                            let _ = SD_CMD_CHANNEL.try_send(SdCommand::CreateFile);

                            use embassy_time::with_timeout;
                            use embassy_time::Duration;
                            match with_timeout(Duration::from_millis(2000), SD_RESP_CHANNEL.receive()).await {
                                Ok(SdResponse::FileCreated(filename)) => {
                                    log_filename = Some(filename);
                                    logging_active = true;
                                    sd_available = true;
                                }
                                Ok(SdResponse::NotInitialized) => {
                                    sd_available = false;
                                }
                                _ => {}
                            }
                        }

                        if logging_active {
                            let _ = class.write_packet(b"RECORDING@50Hz\r\n").await;
                        } else {
                            let _ = class.write_packet(b"STREAMING(NO_SD)\r\n").await;
                        }

                        let start_time = Instant::now();
                        let mut bmp_counter: u8 = 0;

                        // Reset orientation tracker for this recording session
                        orientation_tracker.reset();

                        // Stream and log until STOP received
                        'recording: loop {
                            // Read IMU every iteration (50Hz)
                            if lsm6dsox_ok {
                                let _ = sensors::lsm6dsox::read(&mut i2c0, &mut sensor_data).await;
                            }
                            if lis3mdl_ok {
                                let _ = sensors::lis3mdl::read(&mut i2c0, &mut sensor_data).await;
                            }

                            // Read BMP390 every 5th iteration (10Hz)
                            bmp_counter = bmp_counter.wrapping_add(1);
                            if bmp_counter >= 5 {
                                bmp_counter = 0;
                                if bmp390_ok {
                                    let _ = sensors::bmp390::read(&mut i2c1, &mut sensor_data).await;
                                }
                            }

                            counter = counter.wrapping_add(1);
                            let elapsed_ms = start_time.elapsed().as_millis();

                            // Update orientation from gyroscope data
                            let orientation = orientation_tracker.update(elapsed_ms, &sensor_data, &imu_calib);

                            // Send data to core1 for logging (non-blocking)
                            if logging_active {
                                // Create log entry with derived measurements
                                let log_entry = LogEntry {
                                    sensor_data,
                                    orientation,
                                    linear_accel_x: sensor_data.linear_accel_x(&imu_calib),
                                    linear_accel_y: sensor_data.linear_accel_y(&imu_calib),
                                    linear_accel_z: sensor_data.linear_accel_z(&imu_calib),
                                };
                                // Use try_send to avoid blocking if channel is full
                                // Data will be dropped if channel is full (better than blocking)
                                let _ = SD_DATA_CHANNEL.try_send((elapsed_ms, log_entry));
                            }

                            // Send extended telemetry over USB every 5th iteration (10Hz)
                            if counter % 5 == 0 {
                                let msg = format_telemetry_extended(counter, &sensor_data, &orientation, &imu_calib);
                                if class.write_packet(msg.as_bytes()).await.is_err() {
                                    break 'recording;
                                }
                            }

                            // Wait 20ms between readings (50 Hz)
                            Timer::after_millis(20).await;

                            // Check for incoming command
                            use embassy_time::with_timeout;
                            use embassy_time::Duration;
                            if let Ok(Ok(n)) = with_timeout(Duration::from_millis(1), class.read_packet(&mut cmd_buf)).await {
                                if n > 0 {
                                    let stop_cmd = &cmd_buf[..n];
                                    if stop_cmd.starts_with(b"STOP") || stop_cmd.starts_with(b"stop") {
                                        // Tell core1 to stop and flush
                                        let _ = SD_CMD_CHANNEL.try_send(SdCommand::StopLogging);
                                        logging_active = false;
                                        let _ = class.write_packet(b"STOPPED\r\n").await;
                                        break 'recording;
                                    } else if stop_cmd.starts_with(b"BOOTSEL") || stop_cmd.starts_with(b"bootsel") {
                                        reset_to_usb_boot(0, 0);
                                    }
                                }
                            }
                        }
                    }
                }
                Ok(_) => {}
                Err(_) => break,
            }
        }
    }
}
