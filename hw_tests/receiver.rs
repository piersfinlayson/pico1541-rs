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

use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use embassy_executor::Spawner;
use pico1541_rs::test::{IEC_PINS_IN, InputPin};

pub const DELAY_MS: u64 = 100;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("pico1541 test sender");

    // Create the pin objects
    let p = embassy_rp::init(Default::default());
    let clock = InputPin::new("clock", p.PIN_11.into());
    assert_eq!(clock.num, IEC_PINS_IN.clock);
    let data = InputPin::new("data", p.PIN_13.into());
    assert_eq!(data.num, IEC_PINS_IN.data);
    let atn = InputPin::new("atn", p.PIN_12.into());
    assert_eq!(atn.num, IEC_PINS_IN.atn);
    let reset = InputPin::new("reset", p.PIN_10.into());
    assert_eq!(reset.num, IEC_PINS_IN.reset);
    let srq = InputPin::new("srq", p.PIN_14.into());
    assert_eq!(srq.num, IEC_PINS_IN.srq);

    let mut pins = [
        clock,
        data,
        atn,
        reset,
        srq,
    ];

    loop {
        for pin in pins.iter_mut() {
            if pin.has_changed() {
                info!("Pin {} changed to {}", pin.name, pin.get_level());
            }
        }
    }
}
