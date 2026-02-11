#![no_std]
#![no_main]

mod protocol;
mod uart_bridge;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{USB, UART0};
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_rp::uart::{self, Uart, Config as UartConfig};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::UsbDevice;
use embassy_time::Timer;
use embassy_futures::select::{select, Either};
use static_cell::StaticCell;
use panic_probe as _;

use protocol::{Mode, DataPacket, EscapeDetector};
use uart_bridge::{UART_TO_USB, USB_TO_UART, uart_rx_task, uart_tx_task};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    UART0_IRQ => uart::InterruptHandler<UART0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Ground Station RFD Bridge"),
    embassy_rp::binary_info::rp_program_description!(c"USB-UART bridge for RFD 900x radio modem."),
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

    // --- USB CDC setup ---
    let driver = Driver::new(p.USB, Irqs);

    let mut config = embassy_usb::Config::new(0x2e8a, 0x000b);
    config.manufacturer = Some("NCSSM Rocketry");
    config.product = Some("Ground Station RFD Bridge");
    config.serial_number = Some("002");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let state = STATE.init(State::new());

    let mut builder = embassy_usb::Builder::new(
        driver, config, config_desc, bos_desc, &mut [], control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, state, 64);
    let usb = builder.build();
    spawner.spawn(usb_task(usb)).unwrap();

    // --- UART setup for RFD 900x (57600 8N1, TX=GP0, RX=GP1) ---
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 57600;

    let uart = Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, uart_config,
    );
    let (uart_tx, uart_rx) = uart.split();

    spawner.spawn(uart_rx_task(uart_rx)).unwrap();
    spawner.spawn(uart_tx_task(uart_tx)).unwrap();

    // --- Main command loop ---
    let mut usb_buf = [0u8; 64];
    let mut escape = EscapeDetector::new();

    loop {
        class.wait_connection().await;
        let mut mode = Mode::Command;
        escape.reset();

        // Drain any stale UART data that arrived while disconnected
        while UART_TO_USB.try_receive().is_ok() {}

        'connected: loop {
            match mode {
                Mode::Command => {
                    match select(
                        class.read_packet(&mut usb_buf),
                        UART_TO_USB.receive(),
                    )
                    .await
                    {
                        // USB data received - interpret as command
                        Either::First(Ok(n)) if n > 0 => {
                            let cmd = &usb_buf[..n];

                            if cmd.starts_with(b"PING") || cmd.starts_with(b"ping") {
                                let _ = class.write_packet(b"PONG\r\n").await;
                            } else if cmd.starts_with(b"BOOTSEL")
                                || cmd.starts_with(b"bootsel")
                            {
                                reset_to_usb_boot(0, 0);
                            } else if cmd.starts_with(b"STATUS")
                                || cmd.starts_with(b"status")
                            {
                                let mode_name = match mode {
                                    Mode::Command => b"COMMAND" as &[u8],
                                    Mode::Passthrough => b"PASSTHROUGH",
                                    Mode::AtConfig => b"ATCONFIG",
                                };
                                let _ = class.write_packet(b"MODE:").await;
                                let _ = class.write_packet(mode_name).await;
                                let _ = class
                                    .write_packet(b" UART:57600,8N1 TX=GP0 RX=GP1\r\n")
                                    .await;
                            } else if cmd.starts_with(b"PASSTHROUGH")
                                || cmd.starts_with(b"passthrough")
                            {
                                mode = Mode::Passthrough;
                                escape.reset();
                                let _ = class
                                    .write_packet(
                                        b"PASSTHROUGH MODE (3x Ctrl-A to exit)\r\n",
                                    )
                                    .await;
                            } else if cmd.starts_with(b"ATMODE")
                                || cmd.starts_with(b"atmode")
                            {
                                let _ =
                                    class.write_packet(b"ENTERING AT MODE...\r\n").await;
                                // Guard time before +++
                                Timer::after_millis(1000).await;
                                let _ = USB_TO_UART
                                    .try_send(DataPacket::from_slice(b"+++"));
                                // Guard time after +++
                                Timer::after_millis(1000).await;
                                mode = Mode::AtConfig;
                                let _ = class.write_packet(b"AT MODE READY\r\n").await;
                            } else if cmd.starts_with(b"SEND ")
                                || cmd.starts_with(b"send ")
                            {
                                if n > 5 {
                                    let _ = USB_TO_UART.try_send(
                                        DataPacket::from_slice(&usb_buf[5..n]),
                                    );
                                    let _ = class.write_packet(b"SENT\r\n").await;
                                } else {
                                    let _ =
                                        class.write_packet(b"ERR:NO_DATA\r\n").await;
                                }
                            } else if cmd.starts_with(b"HELP")
                                || cmd.starts_with(b"help")
                            {
                                let _ = class
                                    .write_packet(
                                        b"Commands: PING BOOTSEL STATUS PASSTHROUGH ATMODE SEND HELP\r\n",
                                    )
                                    .await;
                            } else {
                                let _ =
                                    class.write_packet(b"ERR:UNKNOWN_CMD\r\n").await;
                            }
                        }
                        Either::First(Ok(_)) => {} // empty packet
                        Either::First(Err(_)) => break 'connected,
                        // UART data received - forward to USB
                        Either::Second(packet) => {
                            if class.write_packet(&packet.buf[..packet.len]).await.is_err() {
                                break 'connected;
                            }
                        }
                    }
                }

                Mode::Passthrough => {
                    match select(
                        class.read_packet(&mut usb_buf),
                        UART_TO_USB.receive(),
                    )
                    .await
                    {
                        Either::First(Ok(n)) if n > 0 => {
                            // Check for escape sequence
                            let mut escaped = false;
                            for &byte in &usb_buf[..n] {
                                if escape.feed(byte) {
                                    escaped = true;
                                    break;
                                }
                            }
                            if escaped {
                                mode = Mode::Command;
                                escape.reset();
                                let _ = class
                                    .write_packet(b"\r\nEXITING PASSTHROUGH\r\n")
                                    .await;
                            } else {
                                // Forward USB -> UART
                                let _ = USB_TO_UART.try_send(
                                    DataPacket::from_slice(&usb_buf[..n]),
                                );
                            }
                        }
                        Either::First(Ok(_)) => {}
                        Either::First(Err(_)) => break 'connected,
                        Either::Second(packet) => {
                            // Forward UART -> USB
                            if class.write_packet(&packet.buf[..packet.len]).await.is_err() {
                                break 'connected;
                            }
                        }
                    }
                }

                Mode::AtConfig => {
                    match select(
                        class.read_packet(&mut usb_buf),
                        UART_TO_USB.receive(),
                    )
                    .await
                    {
                        Either::First(Ok(n)) if n > 0 => {
                            let cmd = &usb_buf[..n];

                            // Local exit command
                            if cmd.starts_with(b"EXITAT")
                                || cmd.starts_with(b"exitat")
                            {
                                mode = Mode::Command;
                                let _ = class
                                    .write_packet(b"EXITING AT MODE\r\n")
                                    .await;
                            } else {
                                // Forward to radio (AT commands)
                                let _ = USB_TO_UART.try_send(
                                    DataPacket::from_slice(&usb_buf[..n]),
                                );
                                // Check if user sent ATO (return to data mode)
                                if cmd.starts_with(b"ATO\r")
                                    || cmd.starts_with(b"ATO\n")
                                    || cmd == b"ATO"
                                    || cmd.starts_with(b"ato\r")
                                    || cmd.starts_with(b"ato\n")
                                    || cmd == b"ato"
                                {
                                    mode = Mode::Command;
                                    let _ = class
                                        .write_packet(b"EXITING AT MODE\r\n")
                                        .await;
                                }
                            }
                        }
                        Either::First(Ok(_)) => {}
                        Either::First(Err(_)) => break 'connected,
                        Either::Second(packet) => {
                            // Forward AT responses to USB
                            if class.write_packet(&packet.buf[..packet.len]).await.is_err() {
                                break 'connected;
                            }
                        }
                    }
                }
            }
        }
    }
}
