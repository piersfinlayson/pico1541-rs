//! This module handles WiFi support for the pico1541.

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
use static_assertions::const_assert;
use static_cell::StaticCell;

use crate::gpio::GPIO;
use crate::task::spawn_or_reboot;

//
// Statics
//

// Static boolean indicating whether this board has WiFi support.  We could
// just provide a method on WiFi to allow this to be queried, but this is
// easier and quicker (in case WiFi is locked by something else).
pub static IS_WIFI: AtomicBool = AtomicBool::new(false);

// Static WiFI object.  Cannot wrap in a Mutex because the cyw43 WiFi objects
// are not Send.
pub static WIFI: StaticCell<WiFi> = StaticCell::new();

// Static for WiFi state which needs to be static for liftime reasons.
static WIFI_STATE: StaticCell<cyw43::State> = StaticCell::new();

// Static for WiFi control
static WIFI_CONTROL: StaticCell<Control> = StaticCell::new();

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
    pub async fn detect(&mut self) {
        // Create the ADC object.
        self.adc = Some(Adc::new(
            self.p_adc.take().expect("ADC peripheral not present"),
            AdcIrqs,
            Config::default(),
        ));

        // Figure out if WiFi supported.
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

        // Replace the ADC pin - we'll need it again later to start the WiFi
        // stack.
        self.adc_pin.replace(adc_pin);
    }

    /// Initialize the WiFi object.
    pub async fn init(
        &mut self,
    ) -> Option<(
        &'static mut Control<'static>,
        Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
    )> {
        // No-op if WiFi isn't supported.
        if !self.wifi_supported {
            return None;
        }

        // Locking section
        let (pwr_pin, cs_pin, dio_pin, clk_pin) = {
            // Lock GPIO
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

            // Set up the power and chip select pins
            let pwr_pin = guard
                .take_output(wifi_config.pwr, Level::Low)
                .expect("Failed to take WiFi power pin");

            let cs_pin = guard
                .take_output(wifi_config.cs, Level::High)
                .expect("Failed to take WiFi CS pin");

            // Get the DIO and CLK pins
            assert!(wifi_config.dio == 24);
            assert!(wifi_config.clk == 29);
            let doi_pin = guard.take_pin24().expect("Failed to get DIO pin");
            let clk_pin = self.adc_pin.take().expect("Failed to get CLK pin");

            (pwr_pin, cs_pin, doi_pin, clk_pin)
        };

        // Set up the required PIO
        let mut pio = Pio::new(
            self.p_pio0.take().expect("PIO0 peripheral not present"),
            PioIrqs,
        );

        // Create the SPI object
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

        // Get the WiFi firmware
        let fw = include_bytes!("../firmware/cyw43/43439A0.bin");

        // Create the WiFi stack.
        let state = WIFI_STATE.init(State::new());
        let (_net_device, control, runner) = cyw43::new(state, pwr_pin, spi, fw).await;

        // Store the control and runner objects.
        Some((WIFI_CONTROL.init(control), runner))
    }
}

pub async fn init_control(control: &mut Control<'_>) {
    let clm = include_bytes!("../firmware/cyw43/43439A0_clm.bin");
    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;
}

/// Method to run the WiFi stack
#[embassy_executor::task]
pub async fn wifi_task(
    runner: Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    // Run the cyw43 runner
    runner.run().await
}

/// Helper function to easily retrieve whether WiFi is supported.
pub fn is_wifi_supported() -> bool {
    IS_WIFI.load(Ordering::Relaxed)
}
