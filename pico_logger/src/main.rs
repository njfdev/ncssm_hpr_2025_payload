#![no_std]
#![no_main]

mod sensors;
mod sdcard;
mod telemetry;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, I2c, Config as I2cConfig};
use embassy_rp::peripherals::{USB, I2C0, I2C1};
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::UsbDevice;
use embassy_time::Timer;
use static_cell::StaticCell;
use panic_probe as _;

use sensors::SensorData;
use sdcard::{SdLogger, format_sd_status};
use telemetry::{format_status, format_telemetry};

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

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) {
    usb.run().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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

    // Initialize SD card (SPI0: CLK=GP18, MOSI=GP19, MISO=GP20, CS=GP23)
    let sd_result = SdLogger::new(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, p.PIN_23);
    let (mut sd_logger, sd_ok) = match sd_result {
        Ok(logger) => (Some(logger), true),
        Err(_) => (None, false),
    };
    let mut log_filename: Option<heapless::String<12>> = None;

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
                    } else if cmd.starts_with(b"START") || cmd.starts_with(b"start") {
                        let _ = class.write_packet(b"STREAMING\r\n").await;

                        // Stream indefinitely until STOP received
                        'streaming: loop {
                            // Read all sensors
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
                                break 'streaming; // USB disconnected
                            }

                            // Wait 100ms between readings (10 Hz)
                            Timer::after_millis(100).await;

                            // Check for incoming command (poll with short timeout)
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
                        // Read all sensors
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
                    } else if cmd.starts_with(b"SDSTATUS") || cmd.starts_with(b"sdstatus") {
                        // Report SD card status
                        let msg = format_sd_status(sd_ok, log_filename.as_ref().map(|s| s.as_str()));
                        let _ = class.write_packet(msg.as_bytes()).await;
                    } else if cmd.starts_with(b"LOG") || cmd.starts_with(b"log") {
                        // Start logging to SD card (creates new file)
                        if let Some(ref mut logger) = sd_logger {
                            match logger.create_log_file() {
                                Ok(filename) => {
                                    log_filename = Some(filename.clone());
                                    logging_active = true;
                                    let msg = format_sd_status(true, Some(filename.as_str()));
                                    let _ = class.write_packet(msg.as_bytes()).await;
                                }
                                Err(_) => {
                                    let _ = class.write_packet(b"SD:FILE_ERR\r\n").await;
                                }
                            }
                        } else {
                            let _ = class.write_packet(b"SD:NOT_INIT\r\n").await;
                        }
                    } else if cmd.starts_with(b"STOPLOG") || cmd.starts_with(b"stoplog") {
                        // Stop logging to SD card
                        logging_active = false;
                        let _ = class.write_packet(b"SD:LOGGING_STOPPED\r\n").await;
                    } else if cmd.starts_with(b"RECORD") || cmd.starts_with(b"record") {
                        // Start streaming with SD card logging
                        // First, create log file if not already logging
                        if !logging_active {
                            if let Some(ref mut logger) = sd_logger {
                                if let Ok(filename) = logger.create_log_file() {
                                    log_filename = Some(filename.clone());
                                    logging_active = true;
                                }
                            }
                        }

                        if logging_active {
                            let _ = class.write_packet(b"RECORDING\r\n").await;
                        } else {
                            let _ = class.write_packet(b"STREAMING(NO_SD)\r\n").await;
                        }

                        // Stream and log until STOP received
                        'recording: loop {
                            // Read all sensors
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

                            // Log to SD card if active
                            if logging_active {
                                if let (Some(logger), Some(filename)) = (&mut sd_logger, &log_filename) {
                                    let _ = logger.log_data(filename.as_str(), counter, &sensor_data);
                                }
                            }

                            // Send telemetry over USB
                            let msg = format_telemetry(counter, &sensor_data);
                            if class.write_packet(msg.as_bytes()).await.is_err() {
                                break 'recording; // USB disconnected
                            }

                            // Wait 100ms between readings (10 Hz)
                            Timer::after_millis(100).await;

                            // Check for incoming command (poll with short timeout)
                            use embassy_time::with_timeout;
                            use embassy_time::Duration;
                            if let Ok(Ok(n)) = with_timeout(Duration::from_millis(1), class.read_packet(&mut cmd_buf)).await {
                                if n > 0 {
                                    let stop_cmd = &cmd_buf[..n];
                                    if stop_cmd.starts_with(b"STOP") || stop_cmd.starts_with(b"stop") {
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
                Ok(_) => {} // Empty packet, keep waiting
                Err(_) => break, // USB disconnected
            }
        }
    }
}
