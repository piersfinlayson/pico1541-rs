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

// Declare all of this library's modules.
mod usb;
mod bulk;
mod constants;
mod control;
mod protocol;
mod types;
mod dev_info;
mod built;
mod display;
mod driver;
mod iec;
mod gpio;
mod watchdog;

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;

use bulk::bulk_task;
use constants::{LOOP_LOG_INTERVAL, BULK_WATCHDOG_TIMER, STATUS_DISPLAY_WATCHDOG_TIMER, };
use dev_info::get_serial;
use display::status_task;
use gpio::Gpio;
use watchdog::{Watchdog, TaskId, reboot_normal, watchdog_task, register_task};
use usb::{UsbStack, usb_task};

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
//   compile time.
//
// - If your static will be immutable, that is all that is required.
//
// - If your static will be mutable, but you will be passing ownership of it
//   to another object, then no Mutex is required either. 
//
// - If you need mutable access, you need to use a Mutex _and_ a RefCell.
//   - Generally use CriticalSectionRawMutex, as these work on multi-core
//     systems.
//   - ThreadModeRawMutex is, as it sounds, so single threaded usage.
//   - NoopRawMutex is for when you don't need a mutex.
//
//   Our implementation is not sufficiently dependent on performance to
//   require optimization here, so we always use CriticalSectionRawMutex.
//
// The statics are stored in the module that creates them.

// Our main function.  This is the entry point for our application and like
// most embedded implementations, we do not want it to exit as that would mean
// the device has halted.
pub async fn common_main(spawner: Spawner, bin_name: &str) -> ! {
    // Get device peripherals.  This gives us access to the hardware - like
    // the USB and Watchdog.  We extract the ones we need to avoid having to
    // pass the entire object around, partially moving it.
    let p = embassy_rp::init(Default::default());
    let embassy_rp::Peripherals {
        PIN_0: pin_0, PIN_1: pin_1, PIN_2: pin_2, PIN_3: pin_3, 
        PIN_4: pin_4, PIN_5: pin_5, PIN_6: pin_6, PIN_7: pin_7,
        PIN_8: pin_8, PIN_9: pin_9, PIN_10: pin_10, PIN_11: pin_11, 
        PIN_12: pin_12, PIN_13: pin_13, PIN_14: pin_14, PIN_15: pin_15,
        PIN_16: pin_16, PIN_17: pin_17, PIN_18: pin_18, PIN_19: pin_19, 
        PIN_20: pin_20, PIN_21: pin_21, PIN_22: pin_22, PIN_23: pin_23,
        PIN_24: pin_24, PIN_25: pin_25, PIN_26: pin_26, PIN_27: pin_27, 
        PIN_28: pin_28, PIN_29: pin_29,
        WATCHDOG: p_watchdog,
        USB: p_usb,
        FLASH: p_flash,
        DMA_CH0: p_dma_ch0,
        ..
    } = p;

    // Load the serial number.  This is done early, so we can log it.
    let mut p_flash = p_flash;
    let mut p_dma_ch0 = p_dma_ch0;
    let (log_serial, usb_serial) = get_serial(&mut p_flash, &mut p_dma_ch0);

    // Log the information about this firmware build.
    built::log_fw_info(bin_name, &log_serial);

    // Create the objects that need GPIOs.  We do this all at once so there is
    // a single place in the code that allocates GPIOs - to make it easier to
    // see what is being used.
    let (iec_bus, ) = Gpio::create_pin_objects(
        pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, 
        pin_10, pin_11, pin_12, pin_13, pin_14, pin_15, pin_16, pin_17, pin_18, pin_19, 
        pin_20, pin_21, pin_22, pin_23, pin_24, pin_25, pin_26, pin_27, pin_28, pin_29
    );

    // Create the USB stack and the Bulk object.  We do this at the same time
    // because Bulk needs the endpoints from the USB Stack creation.
    let (usb, bulk) = UsbStack::create_static(p_usb, usb_serial, iec_bus);

    // Set up the watchdog - stores it in the WATCHDOG static.  We do this
    // before we create any tasks (as they might try and access the static
    // once that's happened, in order to feed it).
    Watchdog::create_static(p_watchdog);

    // Spawn the Status Display, USB and Bulk tasks.  We do this before we
    // start the watchdog, as we want to make sure the tasks are running
    // and reading to feed it.  (Feeding can happen before the watchdog is
    // started);
    spawn_or_reboot(spawner.spawn(status_task()), "Status Display");
    spawn_or_reboot(spawner.spawn(usb_task(usb)), "USB");
    spawn_or_reboot(spawner.spawn(bulk_task(bulk)), "Bulk");

    // Register the tasks with the watchdog.  We do this after we have created
    // the tasks, so that the watchdog doesn't expect to be fed until they are#
    // running.
    register_task(TaskId::Bulk, BULK_WATCHDOG_TIMER);
    register_task(TaskId::Display, STATUS_DISPLAY_WATCHDOG_TIMER);

    // Now spawn the watchdog task.
    spawn_or_reboot(spawner.spawn(watchdog_task()), "Watchdog");

    // Log every so often and avoid letting main() return.
    loop {
        info!("Main loop");
        Timer::after(LOOP_LOG_INTERVAL).await;
    }
}

// Helper method to spawn tasks
fn spawn_or_reboot<T, E: defmt::Format>(spawn_result: Result<T, E>, task_name: &str) {
    if let Err(e) = spawn_result {
        error!("Failed to spawn task: {}, error: {}", task_name, e);
        reboot_normal();
    }
}

