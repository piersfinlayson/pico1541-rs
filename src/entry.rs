//! This module provides the main entry points to the application, which are
//! used by the binaries in the `bin` directory.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_time::Ticker;

use crate::built::log_fw_info;
use crate::constants::LOOP_LOG_INTERVAL;
use crate::dev_info::get_serial;
use crate::display::status_task;
use crate::gpio::Gpio;
use crate::task::{core1_spawn, spawn_or_reboot_yield};
use crate::usb::{usb_task, UsbStack};
use crate::watchdog::{watchdog_task, Watchdog};
use crate::wifi::{spawn_wifi, WiFi};


/// The main function.  This is the entry point for our application and like
/// most embedded implementations, we do not want it to exit as that would
/// mean the device has halted.
///
/// See [`task.rs`] for an explanation of the multi-core strategy.
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

    // Set up the watchdog - stores it in the WATCHDOG static.  We do this
    // before we create any tasks (as they might try and access the static
    // once that's happened, in order to feed it).
    Watchdog::create_static(p_watchdog);

    // Spawn the watchdog task.  We do this very early, in case there's some
    // kind on hang on core 0, and we're unable to feed it - this will cause
    // the hardware watchdog to reset the device.
    //
    // It's fine to spawn the watchdog before the other tasks, because the
    // other tasks themselves register with the watchdog when starting up.
    // Therefore, they should be running and feeding the watchdog before it
    // starts checking.
    spawn_or_reboot_yield(spawner.spawn(watchdog_task()), "Watchdog").await;

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
    log_fw_info(bin_name, log_serial);

    // Now attempt to initialize WiFi and spawm the WiFi tasks.  If this
    // hardware doesn't support WiFi, WiFi::init() will return None, so we
    // won't spawn the WiFi tasks.  We want this done before we spawn Status
    // Display, as that will use the WiFi GPIO for the status LED (if WiFI
    // is supported).
    if let Some(wifi_args) = wifi.init().await {
        spawn_wifi(&spawner, wifi_args).await;
    }

    // Spawn the status display task.  We do this before any other tasks, as
    // once they are created they may try to update the status display.
    // However, it must be done after the GPIO static is created, as it needs
    // to retrieve the appropriate pin.
    spawn_or_reboot_yield(spawner.spawn(status_task()), "Status Display").await;

    // Create the USB stack and the Bulk object.  We do this at the same time
    // because Bulk needs the endpoints from the USB Stack creation.
    let usb = UsbStack::create_static(p_usb, usb_serial).await;

    // Spawn the USB stack.  The Bulk task will be spawned as part of
    // launching core 1.
    spawn_or_reboot_yield(spawner.spawn(usb_task(usb)), "USB").await;

    // Spawn the core1 task to start the Bulk task and the core Commodore
    // protocol handling.
    core1_spawn(p_core1);

    // Now just loop forever, logging every so often.
    let core = embassy_rp::pac::SIO.cpuid().read();
    let mut ticker = Ticker::every(LOOP_LOG_INTERVAL);
    loop {
        trace!("Core{}: Main loop", core);
        ticker.next().await;
    }
}

// Defmt panic handler
pub fn defmt_panic_handler() -> ! {
    error!("Hit defmt panic handler");

    // Pause for a bit to give any logs time to be received - althought its
    // not clear this will help, as one core is stuck in this panic handler.s
    let wait_until = embassy_time::Instant::now() + embassy_time::Duration::from_millis(1000);
    while embassy_time::Instant::now() < wait_until {
        cortex_m::asm::nop();
    }

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
