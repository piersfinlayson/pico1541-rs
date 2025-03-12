//! Contains a multi-task capable watchdog, that ensures all expected threads
//! are running, and resets the device if not.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use core::cell::RefCell;
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::peripherals::WATCHDOG as P_RpWatchdog;
use embassy_rp::watchdog::ResetReason;
use embassy_rp::watchdog::Watchdog as RpWatchdog;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::{Duration, Instant, Timer};
use rp2040_rom::ROM;

use crate::constants::{WATCHDOG_LOOP_TIMER, WATCHDOG_TIMER};
use crate::time::block_ms;

// We use the WATCHDOG static to store the Watchdog object, so we can feed it
// from all of our tasks and objects.  This is shared and mutable.  We provide
// helper function to allow tasks to register with, feed and use the watchdog:
// - register_task()
// - feed_watchdog()
// - reboot_normal()
// - reboot_dfu()
pub static WATCHDOG: Mutex<CriticalSectionRawMutex, RefCell<Option<Watchdog>>> =
    Mutex::new(RefCell::new(None));

// The Watchdog object implements a multi-task capable watchdog, that ensures
// that the hardware watchdog is only thread is all of our threads remain
// running, and feeding this watchdog.
pub struct Watchdog {
    /// THe hardware watchdog object
    hw_watchdog: RpWatchdog,

    /// The tasks which are policed by the watchdog
    tasks: [Option<Task>; TaskId::Num as usize],
}

impl Watchdog {
    /// Creates a new Watchdog object and stores it in the WATCHDOG static.
    pub fn create_static(p_watchdog: P_RpWatchdog) {
        // Set up the hardware watchdog
        let hw_watchdog = RpWatchdog::new(p_watchdog);

        // Log the last reset reason
        let rr_str = match hw_watchdog.reset_reason() {
            Some(ResetReason::Forced) => "forced",
            Some(ResetReason::TimedOut) => "watchdog timer",
            None => "unknown",
        };
        info!("Last reset reason: {}", rr_str);

        // Create our watchdog object
        let watchdog = Watchdog {
            hw_watchdog,
            tasks: [const { None }; TaskId::Num as usize],
        };

        // Store our watchdog object in the static
        WATCHDOG.lock(|w| {
            *w.borrow_mut() = Some(watchdog);
        });
    }

    /// Registers a task with this watchdog.
    pub fn register_task(&mut self, task: Task) {
        let task_id = task.id;
        let idx = task_id as usize;
        if idx < TaskId::Num as usize {
            self.tasks[idx] = Some(task);
        }
        debug!("Watchdog - registered task: {}", task_id);
    }

    /// De-registers a task with this watchdog.  The watchdog will not require
    /// that task to feed it until it is re-registered.
    #[allow(dead_code)]
    pub fn deregister_task(&mut self, task_id: TaskId) {
        let idx = task_id as usize;
        if idx < TaskId::Num as usize {
            self.tasks[idx] = None;
        }
        debug!("Watchdog - deregistered task: {}", task_id);
    }

    /// Feed the watchdog from a specific task
    pub fn feed(&mut self, task_id: TaskId) {
        let idx = task_id as usize;
        if idx < TaskId::Num as usize {
            if let Some(task) = &mut self.tasks[idx] {
                task.feed();
            } else {
                info!("Attempt to feed unregistered task: {:?}", task_id);
            }
        }
    }

    pub fn start(&mut self) {
        // Feed all of the registered tasks
        for task in self.tasks.iter_mut().flatten() {
            task.feed();
        }

        // Start the hardware watchdog
        self.hw_watchdog.start(WATCHDOG_TIMER);

        debug!("Watchdog started");
    }

    fn trigger_reset(&mut self) -> ! {
        warn!("Trigger reset");

        // Brief pause to allow log to be produced
        block_ms!(10);

        // Reboot the device
        self.hw_watchdog.trigger_reset();

        // Failed to reset.  Try again, differently.
        cortex_m::peripheral::SCB::sys_reset();
    }

    fn feed_hw_watchdog(&mut self) {
        self.hw_watchdog.feed();
    }
}

/// The tasks which are policed by the watchdog.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum TaskId {
    /// The ProtcolHandler task,
    ProtocolHandler = 0,

    /// The StatusDisplay
    Display,

    /// The WiFi Control task.  The cyw43 WiFi task can't be policed by a
    /// watchdog, as the WiFI runner() doesn't return
    WiFiControl,

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

#[derive(Debug, Clone)]
pub struct Task {
    /// The task name
    id: TaskId,

    /// The last time the task was fed
    last_feed: Instant,

    /// Maximum duration between feeds
    max_duration: Duration,
}

impl Task {
    /// Creates a new Task object for registration with the watchdog.
    pub fn new(id: TaskId, max_duration: Duration) -> Self {
        Self {
            id,
            last_feed: Instant::now(),
            max_duration,
        }
    }

    // Feed the task - this is driven by the task itself via the static
    // WATCHDOG
    fn feed(&mut self) {
        self.last_feed = Instant::now();
    }

    // Test whether this task has starved the watchdog.
    fn starved(&self) -> bool {
        (Instant::now() - self.last_feed) > self.max_duration
    }
}

/// The watchdog runner task.  This periodically checks that all tasks have
/// feed the watchdog in the appropriate timeframe, and if not causes a reset.
#[embassy_executor::task]
pub async fn watchdog_task() -> ! {
    // Start the watchdog
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist - can't start the watchdog")
            .start();
    });

    loop {
        // Get the watchdog object and see if any tasks have starved the watchdog
        WATCHDOG.lock(|w| {
            let mut watchdog = w.borrow_mut();
            let watchdog = watchdog
                .as_mut()
                .expect("Watchdog doesn't exist - can't run the watchdog");

            // See if any tasks have starved us
            let mut starved = false;
            for task in watchdog.tasks.iter().flatten() {
                if task.starved() {
                    error!("Task {:?} has starved the watchdog", task.id);
                    starved = true;
                }
            }

            // Either feed the hardware watchdog, or trigger a reset
            if !starved {
                watchdog.feed_hw_watchdog();
            } else {
                // Trigger a reset
                watchdog.trigger_reset();
            }
        });

        // Pause to allow tasks to feed the watchdog.
        Timer::after(WATCHDOG_LOOP_TIMER).await;
    }
}

// Helper functions to prevent the task code from having to worry about locking
// the mutex

/// Called to perform a standard device reboot.  (Normally as in not entering
/// BOOTSEL/DFU mode.)
pub fn reboot_normal() -> ! {
    warn!("Rebooting");

    // Try rebooting using the watchdog
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist  - will try and reset another way")
            .trigger_reset()
    })
}

// Called to perform a reboot into BOOTSEL/DFU mode.
pub fn reboot_dfu() -> ! {
    unsafe {
        ROM::reset_usb_boot(0, 0);
    }
}

// Called to feed the watchdog
pub fn feed_watchdog(task_id: TaskId) {
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist - can't feed it")
            .feed(task_id)
    });
}

// Helper function called to regiser tasks
pub fn register_task(task_id: TaskId, max_duration: Duration) {
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist - can't register task")
            .register_task(Task::new(task_id, max_duration));
    });
}
