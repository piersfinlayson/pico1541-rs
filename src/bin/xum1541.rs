#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    pico1541_rs::common_main(spawner, env!("CARGO_BIN_NAME")).await
}
