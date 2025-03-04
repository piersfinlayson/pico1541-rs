#![no_std]
#![no_main]

use defmt_rtt as _;
use pico1541_rs::{defmt_panic_handler, panic_handler};

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    pico1541_rs::common_main(spawner, env!("CARGO_BIN_NAME")).await
}

// Custom defmt panic handler
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    defmt_panic_handler()
}

// Custom core panic handler
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    panic_handler(info)
}
