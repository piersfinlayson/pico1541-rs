//! pico1541
//!
//! This implements a USB device that allows Commodore disk drives to be
//! connected to a PC.
//!
//! The device can either emulate an xum1541, and is therefore supported
//! directly by [`OpenCBM`](https://github.com/OpenCBM/OpenCBM), or provides
//! extended capabilities and is detected as a pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html
#![no_std]
#![no_main]

// Provide some feature guidance when compiling the library.
#[cfg(not(any(feature = "compatibility", feature = "extended")))]
compile_error!("Either 'xum1541/compatibility' or 'extended' feature must be enabled");
#[cfg(all(feature = "compatibility", feature = "extended"))]
compile_error!("Features 'xum1541/compatibility' and 'extended' cannot be enabled simultaneously");
#[cfg(not(any(feature = "pico", feature = "pico2")))]
compile_error!("Either 'pico' or 'pico2' feature must be enabled");
#[cfg(all(feature = "pico", feature = "pico2"))]
compile_error!("Features 'pico' and 'pico2' cannot be enabled simultaneously");

// Declare all of this library's modules.
mod built;
mod bulk;
mod constants;
mod control;
mod dev_info;
mod display;
mod driver;
mod gpio;
mod iec;
mod ieee;
mod protocol;
mod tape;
mod task;
mod transfer;
mod types;
mod usb;
mod watchdog;
mod wifi;

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_time::Ticker;

use constants::LOOP_LOG_INTERVAL;
use dev_info::get_serial;
use display::status_task;
use gpio::Gpio;
use task::{core1_spawn, spawn_or_reboot};
use usb::{usb_task, UsbStack};
use watchdog::{watchdog_task, Watchdog};
use wifi::{spawn_wifi, WiFi};

// Statics
//
// We set up statics primarily to avoid lifetime issues, and to allow us to
// spawn tasks (accessing these statics), and to split our code into
// separate modules.
//
// These are a bit tricksy to get right, so here is some general guidance:
//
// - Use StaticCell for statics that cannot be initialized at compile time.
//
// - Use ConstStaticCell for statics that can be initialized at compile time.
//   Note that initialization is different than mutability.  A ConstStaticCell
//   can be mutable, when used with RefCell, but it must be initialized at
//   compile time.  It also cannot be take()n and then modified and then
//   take()n again.
//
// - If your static will be immutable, that is all that is required.
//
// - If your static will be mutable, but you will be passing ownership of it
//   to another object, then no Mutex is required either.
//
// - If you need mutable access, you need to use a Mutex.  If you are using a
//   embassy_sync::mutex::Mutex (which is async) you do not need a RefCell for
//   interior mutability.  If you use a blocking_mutexx::Mutex you do.
//   - Generally use CriticalSectionRawMutex, as these work on multi-core
//     systems.
//   - ThreadModeRawMutex is, as it sounds, so single threaded usage.
//   - NoopRawMutex is for when you don't need a mutex.
//
//   Our implementation is not sufficiently dependent on performance to
//   require optimization here, so we tend to use CriticalSectionRawMutex.
//
// The statics tend to be stored in the module that creates them.  So, for
// example, the GPIO static is in gpio.

// Extra binary information that picotool can read.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"pico1541 by piers.rocks"),
    embassy_rp::binary_info::rp_program_description!(c"A USB-based Commodore disk drive adapter for PCs, compatible with OpenCBM, a drop-in replacement for the xum1541, and provides additional, functionality, like WiFi support."),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Our main function.  This is the entry point for our application and like
// most embedded implementations, we do not want it to exit as that would mean
// the device has halted.
pub async fn common_main(spawner: Spawner, bin_name: &str) -> ! {
    // Get device peripherals.  This gives us access to the hardware - like
    // the USB and Watchdog.  We extract the ones we need to avoid having to
    // pass the entire object around, partially moving it.
    let p = embassy_rp::init(Default::default());
    let embassy_rp::Peripherals {
        PIN_0: pin_0,
        PIN_1: pin_1,
        PIN_2: pin_2,
        PIN_3: pin_3,
        PIN_4: pin_4,
        PIN_5: pin_5,
        PIN_6: pin_6,
        PIN_7: pin_7,
        PIN_8: pin_8,
        PIN_9: pin_9,
        PIN_10: pin_10,
        PIN_11: pin_11,
        PIN_12: pin_12,
        PIN_13: pin_13,
        PIN_14: pin_14,
        PIN_15: pin_15,
        PIN_16: pin_16,
        PIN_17: pin_17,
        PIN_18: pin_18,
        PIN_19: pin_19,
        PIN_20: pin_20,
        PIN_21: pin_21,
        PIN_22: pin_22,
        PIN_23: pin_23,
        PIN_24: pin_24,
        PIN_25: pin_25,
        PIN_26: pin_26,
        PIN_27: pin_27,
        PIN_28: pin_28,
        PIN_29: pin_29,
        WATCHDOG: p_watchdog,
        USB: p_usb,
        FLASH: p_flash,
        DMA_CH0: p_dma_ch0,
        CORE1: p_core1,
        ADC: p_adc,
        PIO0: p_pio0,
        ..
    } = p;

    // Load the serial number.  This is done early, so we can log it.
    let mut p_flash = p_flash;
    let mut p_dma_ch0 = p_dma_ch0;
    let (log_serial, usb_serial) = get_serial(&mut p_flash, &mut p_dma_ch0).await;

    // Create the Gpio object, which manages the pins
    Gpio::create_static(
        pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, pin_10, pin_11,
        pin_12, pin_13, pin_14, pin_15, pin_16, pin_17, pin_18, pin_19, pin_20, pin_21, pin_22,
        pin_23, pin_24, pin_25, pin_26, pin_27, pin_28, pin_29, None,
    )
    .await;

    // Initialize WiFi.  At this stage, we don't know if we're running on a
    // Pico or Pico W, so we create the IS_WIFI static, which our code can
    // then test whether we are running on a WiFi device.  We do that in the
    // log_fw_info() function, and also in StatusDisplay (to decide whether to
    // use pin 25 or WiFi GPIO 0 to access the status LED).
    //
    // Note that we have to create WiFi after the Gpio object, as WiFi needs
    // ADC pin (29) to do its capability detection.
    let wifi = WiFi::create_static(p_adc, p_pio0, p_dma_ch0).await;

    // Log the information about this firmware build.  We do this before
    // initializing WiFi so it's done as early as possible (and initializing
    // WiFi involves writing some firmware to the WiFi chip).
    built::log_fw_info(bin_name, log_serial);

    // Set up the watchdog - stores it in the WATCHDOG static.  We do this
    // before we create any tasks (as they might try and access the static
    // once that's happened, in order to feed it).
    Watchdog::create_static(p_watchdog);

    // Now attempt to initialize WiFi and spawm the WiFi tasks.  If this
    // hardware doesn't support WiFi, WiFi::init() will return None, so we
    // won't spawn the WiFi tasks.
    if let Some(wifi_args) = wifi.init().await {
        spawn_wifi(&spawner, wifi_args).await;
    }

    // Spawn the status display task.  We do this before the other tasks, as
    // once they are created they may try to update the status display.
    // However, it must be done after the GPIO static is created, as it needs
    // to retrieve the appropriate pin.
    spawn_or_reboot(spawner.spawn(status_task()), "Status Display");

    // Create the USB stack and the Bulk object.  We do this at the same time
    // because Bulk needs the endpoints from the USB Stack creation.
    let usb = UsbStack::create_static(p_usb, usb_serial).await;

    // Spawn the Status Display and USB tasks on core 0.
    //
    // It's fine to spawn the watchdog before the other tasks, because the
    // other tasks themselves register with the watchdog when starting up.
    // Therefore, they should be running and feeding the watchdog before it
    // starts checking.
    //
    // See [`task.rs`] for an explanation of the multi-core strategy.
    spawn_or_reboot(spawner.spawn(watchdog_task()), "Watchdog");

    spawn_or_reboot(spawner.spawn(usb_task(usb)), "USB");

    // Spawn the core1 task to start the Bulk task and the core Commodore
    // protocol handling.
    core1_spawn(p_core1);

    // Now just loop forever, logging every so often.
    let core = embassy_rp::pac::SIO.cpuid().read();
    let mut ticker = Ticker::every(LOOP_LOG_INTERVAL);
    loop {
        info!("Core{}: Main loop", core);
        ticker.next().await;
    }
}

// Defmt panic handler
pub fn defmt_panic_handler() -> ! {
    error!("Hit defmt panic handler");

    // Panic (and reboot) in release mode
    #[cfg(not(debug_assertions))]
    core::panic!();

    // Abort in debug mode
    #[cfg(debug_assertions)]
    cortex_m::asm::udf();
}

// Core panic handler
pub fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    // Debug build
    #[cfg(debug_assertions)]
    {
        // If you have a logging mechanism set up:
        if let Some(location) = info.location() {
            error!("Panic at {}:{}", location.file(), location.line());
        }
        error!("Message: {}", info.message().as_str());
    }

    #[cfg(not(debug_assertions))]
    {
        let _ = info;
    }

    // Reboot if in release mode
    #[cfg(not(debug_assertions))]
    force_reboot();

    // Abort in debug mode
    #[cfg(debug_assertions)]
    cortex_m::asm::udf();
}

// A reliable rebooting function
#[cfg(not(debug_assertions))]
fn force_reboot() -> ! {
    use embassy_time::Delay;
    use embedded_hal::delay::DelayNs;

    // cortex_m::peripheral::SCB::sys_reset();

    // sys_reset() doesn't work on core 1 so use the watchdog - get it.
    let mut watchdog =
        unsafe { embassy_rp::watchdog::Watchdog::new(embassy_rp::peripherals::WATCHDOG::steal()) };

    // Pause, or if there's a panic early on the probe may not be able to
    // connect to the device as it'll be restarting too quickly.
    Delay.delay_us(1000000); // Pause or the probe

    // Now reset
    watchdog.trigger_reset();

    // Loop forever to ensure this function doesn't return
    loop {
        cortex_m::asm::nop();
    }
}
