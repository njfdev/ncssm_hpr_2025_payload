use embassy_rp::uart::{Async, UartRx, UartTx};
use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Timer, with_timeout, Duration};

use crate::protocol::DataPacket;

/// UART data destined for USB (radio -> computer)
pub static UART_TO_USB: Channel<CriticalSectionRawMutex, DataPacket, 8> = Channel::new();

/// USB data destined for UART (computer -> radio)
pub static USB_TO_UART: Channel<CriticalSectionRawMutex, DataPacket, 8> = Channel::new();

/// Reads bytes from UART, batches them into DataPackets with a 2ms
/// inter-byte timeout, and sends them to the UART_TO_USB channel.
#[embassy_executor::task]
pub async fn uart_rx_task(mut rx: UartRx<'static, Async>) {
    loop {
        let mut packet = DataPacket::new();
        let mut byte = [0u8; 1];

        // Wait for first byte (blocks until data arrives)
        if rx.read(&mut byte).await.is_err() {
            Timer::after_millis(10).await;
            continue;
        }
        packet.buf[0] = byte[0];
        packet.len = 1;

        // Batch more bytes with short timeout
        while packet.len < 64 {
            match with_timeout(Duration::from_millis(2), rx.read(&mut byte)).await {
                Ok(Ok(())) => {
                    packet.buf[packet.len] = byte[0];
                    packet.len += 1;
                }
                _ => break,
            }
        }

        let _ = UART_TO_USB.try_send(packet);
    }
}

/// Receives DataPackets from the USB_TO_UART channel and writes them
/// to the UART TX line.
#[embassy_executor::task]
pub async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    loop {
        let packet = USB_TO_UART.receive().await;
        let _ = tx.write(&packet.buf[..packet.len]).await;
    }
}
