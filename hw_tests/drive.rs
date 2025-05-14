//! test_drive
//!
//! Simulates an IEC drive, to allow the pico1541 to be tested.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use pico1541_rs::test::device::{IecDev, run_iec_device};
use {defmt_rtt as _, panic_probe as _};

// Device ID
const DEVICE_ID: u8 = 0x08;

// Static mutable reference to the IEC device
static mut DEVICE: Option<IecDev> = None;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    info!("pico1541 test drive");

    let p = embassy_rp::init(Default::default());

    // This is test code, so we aren't too worried about taking a shortcut here
    // with a static mutable ref.
    unsafe {
        DEVICE = Some(IecDev::new(p, DEVICE_ID));

        if let Some(ref mut device) = DEVICE {
            spawner.spawn(run_iec_device(device)).unwrap();
        }
    }

    info!("IEC Device running");

    #[allow(clippy::empty_loop)]
    loop {}
}
