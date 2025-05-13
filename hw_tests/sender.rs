//! test_sender
//!
//! Tests the pico1541 hardware, by "sending" data - toggles each IEC line in
//! turn.  This is intended to be used in conjunction with the `test_receiver`
//! program, which will detect the toggling lines and output logs using RTT.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use pico1541_rs::test::create_pins;
use {defmt_rtt as _, panic_probe as _};

pub const DELAY_MS: u64 = 2500;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("pico1541 test sender");

    // Create the pin objects
    let p = embassy_rp::init(Default::default());

    let (_, pins) = create_pins(p, false, true);
    let mut pins = pins.unwrap();

    loop {
        for pin in pins.iter_mut() {
            let _ = Timer::after_millis(DELAY_MS).await;
            if pin.is_high() {
                info!("Set pin {} {} low", pin.name, pin.num);
                pin.set_low();
            } else {
                info!("Set pin {} {} high", pin.name, pin.num);
                pin.set_high();
            }
        }
    }
}
