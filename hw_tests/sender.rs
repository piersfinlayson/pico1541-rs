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

use {defmt_rtt as _, panic_probe as _};
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use pico1541_rs::test::{IEC_PINS_OUT, OutputPin};

pub const DELAY_MS: u64 = 2500;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    info!("pico1541 test sender");

    // Create the pin objects
    let p = embassy_rp::init(Default::default());
    let clock = OutputPin::new("clock", p.PIN_11.into());
    assert_eq!(clock.num, IEC_PINS_OUT.clock);
    let data = OutputPin::new("data", p.PIN_13.into());
    assert_eq!(data.num, IEC_PINS_OUT.data);
    let atn = OutputPin::new("atn", p.PIN_12.into());
    assert_eq!(atn.num, IEC_PINS_OUT.atn);
    let reset = OutputPin::new("reset", p.PIN_10.into());
    assert_eq!(reset.num, IEC_PINS_OUT.reset);
    let srq = OutputPin::new("srq", p.PIN_14.into());
    assert_eq!(srq.num, IEC_PINS_OUT.srq);

    let mut pins = [
        clock,
        data,
        atn,
        reset,
        srq,
    ];

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
