//! test_sender
//!
//! Tests the pico1541 hardware, by "receiving" data - detects when each IEC
//! line is toggled, and outputs a log message using RTT.  This is intended to
//! be used in conjunction with the `test_sender` program, which will drive the
//! lines high and low in turn.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::gpio::Level;
use pico1541_rs::test::create_pins;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("pico1541 test sender");

    let p = embassy_rp::init(Default::default());

    let (pins, _) = create_pins(p, true, false);
    let mut pins = pins.unwrap();

    loop {
        for pin in pins.iter_mut() {
            if pin.has_changed() {
                if pin.get_level() == Level::High {
                    info!("Pin {} {} changed to high", pin.name, pin.num);
                } else {
                    info!("Pin {} {} changed to low", pin.name, pin.num);
                }
            }
        }
    }
}
