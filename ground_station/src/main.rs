#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{UART0, USB};
use embassy_rp::rom_data::reset_to_usb_boot;
use embassy_rp::uart::{self, Async, Uart, Config as UartConfig};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::UsbDevice;
use embassy_time::{Duration, with_timeout};
use embassy_futures::select::{select, Either};
use static_cell::StaticCell;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    UART0_IRQ => uart::InterruptHandler<UART0>;
});

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Ground Station RFD Bridge"),
    embassy_rp::binary_info::rp_program_description!(c"Transparent USB-UART bridge for RFD 900x"),
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

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let mut class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let usb = builder.build();
    spawner.spawn(usb_task(usb)).unwrap();

    // --- UART setup for RFD 900x (57600 8N1, TX=GP0, RX=GP1) ---
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 57600;

    let uart = Uart::new(
        p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, uart_config,
    );
    let (mut uart_tx, mut uart_rx) = uart.split();

    // --- Simple passthrough: no channels, no tasks, just select in main ---
    let mut usb_buf = [0u8; 64];
    let mut uart_byte = [0u8; 1]; // 1-byte reads so DMA returns immediately
    let mut escape_count: u8 = 0;

    loop {
        class.wait_connection().await;
        escape_count = 0;

        'connected: loop {
            match select(
                class.read_packet(&mut usb_buf),
                uart_rx.read(&mut uart_byte),
            )
            .await
            {
                // USB → UART
                Either::First(Ok(n)) if n > 0 => {
                    for &b in &usb_buf[..n] {
                        if b == 0x01 {
                            escape_count += 1;
                            if escape_count >= 3 {
                                reset_to_usb_boot(0, 0);
                            }
                        } else {
                            escape_count = 0;
                        }
                    }
                    let _ = uart_tx.write(&usb_buf[..n]).await;
                }
                Either::First(Ok(_)) => {}
                Either::First(Err(_)) => break 'connected,

                // UART → USB: got 1 byte, greedily batch more with short timeout
                Either::Second(Ok(())) => {
                    let mut batch = [0u8; 64];
                    batch[0] = uart_byte[0];
                    let mut count: usize = 1;

                    while count < 64 {
                        match with_timeout(
                            Duration::from_millis(2),
                            uart_rx.read(&mut uart_byte),
                        )
                        .await
                        {
                            Ok(Ok(())) => {
                                batch[count] = uart_byte[0];
                                count += 1;
                            }
                            _ => break,
                        }
                    }

                    match with_timeout(
                        Duration::from_millis(500),
                        class.write_packet(&batch[..count]),
                    )
                    .await
                    {
                        Ok(Ok(())) => {}
                        _ => break 'connected,
                    }
                }
                Either::Second(Err(_)) => {
                    // UART error, continue
                }
            }
        }
    }
}
