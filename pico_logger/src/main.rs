#![no_std]
#![no_main]

use embassy_executor::Spawner;
use panic_probe as _;

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Pico Logger - 2025 NCSSM HPR Payload"),
    embassy_rp::binary_info::rp_program_description!(
        c"This is the payload code for the onboard rp2350 chip."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    /*
    TODO for the Pico Logger

    [ ] - Log data to connected SPI SD Card (explore exfat, talc, and embedded-sdmmc-rs crates)
    [ ] - Log data from BMP390 (Barometer)
    [ ] - Log data from ENS160 (Gas)
    [ ] - Log data from LSM6DSOX + LIS3MDL (IMU)
    [ ] - Log data from HGLRC Mini M100 (GPS)
    [ ] - Send sensor data to flight_computer board

     */
}
