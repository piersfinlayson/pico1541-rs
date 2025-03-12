//! This module handles WiFi support for the pico1541.
//!
//! As well as actually running the WiFi stack, this module also detects
//! whether the board we are running on supports WiFi and, if so, provides
//! a capability to control the WiFi GPIO 0 (which on a Pico W/Pico 2 W is
//! connected to the onboard LED).  That is done via the WIFI_GPIO_0 Signal
//! static.
//!
//! StatusDisplay also needs to know whether to use the standard LED GPIO (25)
//! or the WiFi GPIO.  It does this, by querying the IS_WIFI AtomicBool
//! static.
//!
//! Both these statics are owned and initialized by this module:
//! - IS_WIFI is set up by WiFi::create_static()
//! - WIFI_GPIO_0 is set up at start of day, but will only be acted upon once
//!   spawn_wifi() has been called.
//!
//! There is no issue w
//!
//!

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![allow(unused_imports)]
use core::sync::atomic::{AtomicBool, Ordering};
use cyw43::{Control, Runner, State};
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::{debug, error, info, trace, warn};
use embassy_executor::{Executor, Spawner};
use embassy_rp::adc::{Adc, Async, Channel, Config, InterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{ADC, DMA_CH0, PIN_29, PIO0};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::with_timeout;
use static_assertions::const_assert;
use static_cell::StaticCell;

use crate::constants::{
    WIFI_CLK_PIN, WIFI_CONTROL_WAIT_TIMER, WIFI_CONTROL_WATCHDOG_TIMER, WIFI_DETECT_ADC_PIN,
    WIFI_DIO_PIN,
};
use crate::gpio::GPIO;
use crate::task::spawn_or_reboot_yield;
use crate::watchdog::{feed_watchdog, register_task, TaskId};

//
// Statics
//

// Static boolean indicating whether this board has WiFi support.  We could
// just provide a method on WiFi to allow this to be queried, but this is
// easier and quicker (in case WiFi is locked by something else).
pub static IS_WIFI: AtomicBool = AtomicBool::new(false);

// Signal for StatusDisplay to update the WiFi GPIO 0.
pub static WIFI_GPIO_0: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// Static WiFI object.  Cannot wrap in a Mutex because the cyw43 WiFi objects
// are not Send.
static WIFI: StaticCell<WiFi> = StaticCell::new();

// Static for WiFi state which needs to be static so it outlives cyw43.
static WIFI_STATE: StaticCell<cyw43::State> = StaticCell::new();

// Required to use the ADC.
bind_interrupts!(struct AdcIrqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

// Required by WiFi implementation.
bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// The WiFi object is used to control and handle WiFi support for the
/// pico1541.  This must be created, whether running on a Pico or Pico W, as it
/// it detects whether WiFi is actually supported by the board we are running
/// on.
///
/// ```rust
/// import core::sync::atomic::Ordering;
///
/// let mut wifi = WiFi::new(p_adc, pin_29);
/// wifi.init().await;
///
/// info!("WiFi supported: {}", IS_WIFI.load(Ordering::Relaxed));
/// ```
pub struct WiFi {
    p_adc: Option<ADC>,
    adc_pin: Option<PIN_29>,
    p_pio0: Option<PIO0>,
    dma_ch0: Option<DMA_CH0>,
    adc: Option<Adc<'static, Async>>,
    wifi_supported: bool,
}

impl WiFi {
    /// Creates a new instance of the WiFi object.
    pub async fn create_static(p_adc: ADC, p_pio0: PIO0, p_dma_ch0: DMA_CH0) -> &'static mut Self {
        // Get the ADC pin required to detect whether WiFi is supported.
        const_assert!(WIFI_DETECT_ADC_PIN == 29);
        let adc_pin = GPIO
            .lock()
            .await
            .as_mut()
            .expect("GPIO object not initialized")
            .take_pin29()
            .expect("Failed to take ADC pin 3");

        // Create the WiFi object.
        let mut wifi = Self {
            p_adc: Some(p_adc),
            adc_pin: Some(adc_pin),
            p_pio0: Some(p_pio0),
            dma_ch0: Some(p_dma_ch0),
            adc: None,
            wifi_supported: false,
        };

        // Detect whether WiFi is supported.
        wifi.detect().await;

        // Store the WiFi object in the static and return mutable reference to
        // it.
        WIFI.init(wifi)
    }

    // Detect whether WiFi is supported by this board.  Must be called before
    // initializing the WiFi object.
    //
    // Detects whether this is a Pico or Pico W, based on the voltage read from
    // the ADC3 pin.
    // ADC3 is around 0V for Pico W, and around 1.3 of VSYS (itself 3.3V) for Pico.
    //
    // This is documented in "Conecting to the Internet with Pico W", section 2.4
    // "Which hardware am I running on?".
    // https://datasheets.raspberrypi.com/picow/connecting-to-the-internet-with-pico-w.pdf
    async fn detect(&mut self) {
        // Create the ADC object, which we need to test the voltage on pin 29.
        self.adc = Some(Adc::new(
            self.p_adc.take().expect("ADC peripheral not present"),
            AdcIrqs,
            Config::default(),
        ));

        // Figure out if WiFi supported.  This code reads the voltage on pin
        // 29 and sets the IS_WIFI static accordingly.
        let mut adc_pin = self.adc_pin.take().expect("Pin 29/ADC pin 3 not present");
        let mut channel = Channel::new_pin(&mut adc_pin, Pull::None);
        let raw_value: u16 = self
            .adc
            .as_mut()
            .expect("WiFi object not initialized")
            .read(&mut channel)
            .await
            .unwrap();
        let voltage = (raw_value as f32) * (3.3 / (1 << 12) as f32);
        debug!("ADC3 voltage: {}V", voltage);
        self.wifi_supported = voltage < 0.2;
        IS_WIFI.store(self.wifi_supported, Ordering::Release);
        drop(channel);

        // Replace the ADC pin back in WiFi, as we'll need it again later to
        // initialize the WiFi stack.
        self.adc_pin.replace(adc_pin);
    }

    /// Initialize the WiFi object.  Must be called before calling
    /// spawn_wifi() to start the WiFi tasks.
    ///
    /// This function returns an Option<tuple>, with the tuple contaning
    /// arguments required by the spawn_wifi() function.  None is returned
    /// is this device doesn't support WiFi (i.e. a non-W variant of the
    /// Pico).
    pub async fn init(
        &mut self,
    ) -> Option<(
        Control<'static>,
        Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
    )> {
        // Check if WiFi isn't supported.  This is set up in
        // WiFi::create_static() so must be valid here.
        if !self.wifi_supported {
            return None;
        }

        // Locking section
        let (pwr_pin, cs_pin, dio_pin, clk_pin) = {
            // Lock GPIO, so we can retrieve the pins we need.
            let mut guard = GPIO.lock().await;
            let guard = guard.as_mut().expect("GPIO object not initialized");

            // Get WiFi config - if we've detected WiFi is supported, we should
            // have the pins, but we will fail gracefully just in case.
            let wifi_config = match guard.get_wifi_pins() {
                Some(wifi_config) => wifi_config,
                None => {
                    error!(
                        "WiFi initialization failed - pins not found, but we detected WiFi support"
                    );
                    return None;
                }
            };

            // Set up the power and chip select pins as output with the
            // correct initial values.
            let pwr_pin = guard
                .take_output(wifi_config.pwr, Level::Low)
                .expect("Failed to take WiFi power pin");

            let cs_pin = guard
                .take_output(wifi_config.cs, Level::High)
                .expect("Failed to take WiFi CS pin");

            // Get the DIO and CLK pins.  These must be passed into the WiFi
            // object as the pin types themselves, rather than generic Flex,
            // AnyPin or other types, so the WiFi code knows these support the
            // required SPI capabilities.
            const_assert!(WIFI_DIO_PIN == 24);
            assert!(wifi_config.dio == WIFI_DIO_PIN);
            let doi_pin = guard.take_pin24().expect("Failed to get DIO pin");

            const_assert!(WIFI_CLK_PIN == WIFI_DETECT_ADC_PIN);
            assert!(wifi_config.clk == WIFI_CLK_PIN);
            let clk_pin = self.adc_pin.take().expect("Failed to get CLK pin");

            (pwr_pin, cs_pin, doi_pin, clk_pin)
        };

        // Set up the PIO required by the WiFi stack.
        let mut pio = Pio::new(
            self.p_pio0.take().expect("PIO0 peripheral not present"),
            PioIrqs,
        );

        // Create the SPI object required by the WiFi stack.  This is used to
        // communicate with the cyw43 IC.
        let spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            DEFAULT_CLOCK_DIVIDER,
            pio.irq0,
            cs_pin,
            dio_pin,
            clk_pin,
            self.dma_ch0.take().expect("DMA_CH0 peripheral not present"),
        );

        // Get the WiFi firmware.  There are other options here - the firmware
        // doesn't not need to built into our application binary, but it does
        // need to be written somewhere to the Pico flash, and then loaded so
        // it can be passed into the cy43 IC.
        let fw = include_bytes!("../firmware/cyw43/43439A0.bin");

        // Create the WiFi stack.
        let state = WIFI_STATE.init(State::new());
        let (_net_device, control, runner) = cyw43::new(state, pwr_pin, spi, fw).await;

        // Store the control and runner objects.
        Some((control, runner))
    }
}

/// Helper to spawn all required WiFi tasks, on the same core as this
/// function is called from:
/// - The cyw43 WiFi stack
/// - The WiFi control task
pub async fn spawn_wifi(
    spawner: &Spawner,
    wifi_args: (
        Control<'static>,
        Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
    ),
) {
    // Break out the arguments
    let (mut control, runner) = wifi_args;

    // Spawn the WiFi task.
    spawn_or_reboot_yield(spawner.spawn(cyw43_task(runner)), "cyw43 WiFi").await;

    // Now the WiFi task is spwaned, initialize the WiFi control object..
    init_control(&mut control).await;

    // Spawn the control task.
    spawn_or_reboot_yield(spawner.spawn(wifi_control_task(control)), "WiFi Control").await;
}

/// Method to run the WiFi stack.
#[embassy_executor::task]
pub async fn cyw43_task(
    runner: Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    // Run the cyw43 runner.  I believe this isn't cancel safe, so we can't
    // run it in a loop, with periodic watchdog feedings - hence this task
    // isn't watchdog policed.
    runner.run().await
}

/// Method to run the WiFi control task
#[embassy_executor::task]
pub async fn wifi_control_task(mut control: Control<'static>) -> ! {
    // Register with the watchdog
    register_task(TaskId::WiFiControl, WIFI_CONTROL_WATCHDOG_TIMER);

    loop {
        // Feed the watchdog.
        feed_watchdog(TaskId::WiFiControl);

        // Wait for the signal to update the WiFi GPIO 0.
        if let Ok(state) = with_timeout(WIFI_CONTROL_WAIT_TIMER, WIFI_GPIO_0.wait()).await {
            // wait() returned, so the Signal was signalled.
            trace!("Set WiFi GPIO 0 state to {}", state);
            control.gpio_set(0, state).await;
        }

        // Go around the loop again, immediately feeding the watchdog,
        // whatever response we got from with_timeout().
    }
}

/// Helper function to retrieve whether WiFi is supported.
///
/// The response is valid once WiFi::create_static() has returned.
pub fn is_wifi_supported() -> bool {
    IS_WIFI.load(Ordering::Relaxed)
}

// Initializes WiFI control, we can subsequently use it to control the WiFi
// stack.
async fn init_control(control: &mut Control<'_>) {
    let clm = include_bytes!("../firmware/cyw43/43439A0_clm.bin");
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
}
