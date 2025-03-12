//! Handles displaying status of the device on the LED.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

use crate::constants::{
    STATUS_DISPLAY_BLINK_TIMER, STATUS_DISPLAY_TIMER, STATUS_DISPLAY_WATCHDOG_TIMER,
};
use crate::gpio::GPIO;
use crate::watchdog::{feed_watchdog, register_task, TaskId};
use crate::wifi::{is_wifi_supported, WIFI_GPIO_0};

// The STATUS_DISPLAY static is used to signal to the StatusDisplay object
// that it needs to update the LED state.
pub static STATUS_DISPLAY: Signal<CriticalSectionRawMutex, DisplayType> = Signal::new();

/// Status display types
/// Corresponds to the different operating states of the device
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum DisplayType {
    /// Initialization state, LED is fully on
    Init,
    /// Ready state, LED is fully off
    Ready,
    /// Active state, LED toggles at regular intervals
    Active,
    /// Error state, LED toggles at regular intervals (same as Active)
    Error,
}

/// Status display management
/// Handles the operation of the status LED based on the current device state
pub struct StatusDisplay {
    /// The Pico GPIO pin connected to the LED.  If None, we are using the
    /// WiFi status GPIO instead.  
    led: Option<Output<'static>>,

    /// Current state of the device
    current_status: DisplayType,

    /// Last time the LED was toggled, used for blinking patterns
    last_toggle: Instant,

    /// Current LED state (on or off)
    led_state: bool,
}

impl StatusDisplay {
    /// Creates a new StatusDisplay with the specified LED pin..
    ///
    /// The LED is initially turned on (Init state)
    pub async fn new() -> Self {
        // Does this board support WiFi?
        let wifi = is_wifi_supported();

        let led = if !wifi {
            debug!("WiFi not supported, getting Pico status display pin");
            Some(Self::get_local_display_pin().await)
        } else {
            // led == None indicates we are using the WiFI status LED
            None
        };

        // Create the display with the LED off
        Self {
            led,
            current_status: DisplayType::Init,
            last_toggle: Instant::now(),
            led_state: false,
        }
    }

    // If this board doesn't supply WiFi, this function is called to get the
    // appropriate Output pin to use.
    async fn get_local_display_pin() -> Output<'static> {
        let pin = GPIO
            .lock()
            .await
            .as_mut()
            .expect("GPIO not created")
            .get_status_display_pin();

        Output::new(pin, Level::High)
    }

    // Turns the appropriate LED on
    async fn led_on(&mut self) {
        if let Some(led) = self.led.as_mut() {
            led.set_high();
        } else {
            WIFI_GPIO_0.signal(true);
        }
        self.led_state = true;
    }

    // Turns the appropriate LED off
    async fn led_off(&mut self) {
        if let Some(led) = self.led.as_mut() {
            led.set_low();
        } else {
            WIFI_GPIO_0.signal(false);
        }
        self.led_state = false;
    }

    /// Update the current status
    ///
    /// This is calld by other tasks to change what the LED displays.
    pub async fn update(&mut self, status: DisplayType) {
        debug!("Update status to {}", status);

        // Only update LED immediately if changing to/from states with
        // different LED behaviors
        if self.current_status != status {
            self.current_status = status;

            // Apply immediate LED state change based on new status
            match status {
                DisplayType::Init => {
                    self.led_on().await;
                }
                DisplayType::Ready => {
                    self.led_off().await;
                }
                // For Active and Error states, we'll handle the toggling
                // in do_work()
                _ => {}
            }
        }
    }

    /// Perform an action on the status display if one is required.
    /// This function does not block - but performs an action if it's
    /// outstanding, queues up the next action (for example sets a time by
    /// which the next action should be performed).
    /// Returns the maximum Duration until the next time this function should
    /// be called.
    async fn do_work(&mut self) -> Duration {
        // Handle LED updates based on current status
        match self.current_status {
            DisplayType::Init => self.do_on().await,
            DisplayType::Ready => self.do_off().await,
            DisplayType::Active | DisplayType::Error => self.do_blink().await,
        }
    }

    // Handles turning the LED off from within the do_work() function.
    async fn do_on(&mut self) -> Duration {
        // LED is always on in Init state, nothing to do
        if !self.led_state {
            self.led_on().await;
        }

        // No urgent updates needed, can wait the default time
        STATUS_DISPLAY_BLINK_TIMER
    }

    // Handles turning the LED off from within the do_work() function.
    async fn do_off(&mut self) -> Duration {
        // LED is always off in Ready state, nothing to do
        if self.led_state {
            self.led_off().await;
        }

        // No urgent updates needed, can wait the default time
        STATUS_DISPLAY_BLINK_TIMER
    }

    // Handles LED blinking, returns the time until the next update is needed.
    // Works by toggling the LED every STATUS_DISPLAY_BLINK_TIMER regardless
    // of how often we're called.
    async fn do_blink(&mut self) -> Duration {
        // Find out how long it is since we last toggled the LED
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_toggle);

        // Figure out if we need to toggle it.
        if elapsed.as_millis() >= STATUS_DISPLAY_BLINK_TIMER.as_millis() {
            // Time to toggle
            self.toggle_led().await;

            // Time until next toggle
            STATUS_DISPLAY_BLINK_TIMER
        } else {
            // Return the remaining time until we need to toggle
            STATUS_DISPLAY_BLINK_TIMER - elapsed
        }
    }

    /// Toggle the LED state.
    async fn toggle_led(&mut self) {
        if self.led_state {
            self.led_off().await;
        } else {
            self.led_on().await;
        }
        self.last_toggle = Instant::now();
    }
}

/// Runs the status display task.
/// This is an embassy executor task that periodically calls do_work()
#[embassy_executor::task]
pub async fn status_task() -> ! {
    // Get the core number, in order to log it.
    let core = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Status display task started", core);

    // Create the status display object
    let mut display = StatusDisplay::new().await;

    // Register with the watchdog
    register_task(TaskId::Display, STATUS_DISPLAY_WATCHDOG_TIMER);

    loop {
        // Feed the watchdog
        feed_watchdog(TaskId::Display);

        // Update the status if necessary
        if let Some(new_status) = STATUS_DISPLAY.try_take() {
            display.update(new_status).await
        }

        // Let the status display do some work and get the time until next
        // update.
        let next_update = display.do_work().await;

        // Determine how long to wait - use the minimum of:
        // 1. The time until the next LED update is needed (from do_work)
        // 2. The maximum time we want to wait before checking for status
        //    changes (STATUS_DISPLAY_TIMER)
        let wait_time = Duration::min(next_update, STATUS_DISPLAY_TIMER);

        // Now pause for the calculated time. This allows other tasks to grab
        // the StatusDisplay object and do their work, while also ensuring we
        // wake up in time for the next required LED update.
        Timer::after(wait_time).await;
    }
}

/// Helper function to update the status, which handles locking the mutex.
pub fn update_status(display: DisplayType) {
    // It's OK to overwrite the last status, if it hasn't been applied yet, as
    // this new status should supercede the old one.
    debug!("Updating status to {}", display);
    STATUS_DISPLAY.signal(display);
}
