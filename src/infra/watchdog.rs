//! Contains a multi-task capable watchdog, that ensures all expected threads
//! are running, and resets the device if not.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::peripherals::WATCHDOG as p_WATCHDOG;
use static_cell::StaticCell;
use task_watchdog::embassy_rp::{WatchdogRunner, watchdog_run};
use task_watchdog::{Id, WatchdogConfig};

use crate::constants::{WATCHDOG_CHECK_INTERVAL, WATCHDOG_HW_TIMEOUT};

// Create a type alias for the WatchdogRunner to make it easier to use.
pub type WatchdogType = WatchdogRunner<TaskId, NUM_TASK_IDS>;

// We use the WATCHDOG static to store the Watchdog object, so we can feed it
// from all of our tasks and objects.
pub static WATCHDOG: StaticCell<WatchdogType> = StaticCell::new();

/// The tasks which are policed by the watchdog.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TaskId {
    /// The main [`ProtcolHandler`] task,
    ProtocolHandler = 0,

    /// The [`StatusDisplay`] task.
    Display,

    /// The Wi-Fi Control task.  The cyw43 Wi-Fi task can't be policed by a
    /// watchdog, as the Wi-Fi `runner()` doesn't return
    WiFiControl,

    /// A spawned drive operation task (read or write)
    DriveOperation,

    // We cannot police USB as it needs to run permanently.  The same goes
    // for the Bulk task - it calls the IN endpoint read().  Technically
    // we probably could cancel it to feed the watchdog, but we will trust
    // it and the USB stack not to lock up the device.

    // Add any other tasks here
    // ...
    //
    /// The is the number of tasks which are policed by the watchdog.
    Num,
}
impl Id for TaskId {}
const NUM_TASK_IDS: usize = TaskId::Num as usize;

/// A helper function to create the watchdog.
pub fn create_watchdog(p_watchdog: p_WATCHDOG) -> &'static mut WatchdogType {
    // Create watchdog configuration
    let config = WatchdogConfig {
        hardware_timeout: WATCHDOG_HW_TIMEOUT,
        check_interval: WATCHDOG_CHECK_INTERVAL,
    };

    // Create and configure the watchdog runner
    let watchdog = WatchdogRunner::new(p_watchdog, config);

    // Make watchdog static so it can be shared with tasks
    WATCHDOG.init(watchdog)
}

/// A task to run the watchdog.
#[embassy_executor::task]
pub async fn watchdog_task(watchdog: &'static WatchdogType) -> ! {
    watchdog_run(watchdog.create_task()).await
}
