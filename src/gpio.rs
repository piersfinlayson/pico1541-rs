//! This file handles GPIO pin allocation.
//!
//! The source code serves as the master list of pin assignments for the
//! hardware.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19,
    PIN_2, PIN_20, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27, PIN_28, PIN_29, PIN_3,
    PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use crate::iec::IecBus;

//
// Statics
//

pub static GPIO: Mutex<CriticalSectionRawMutex, Option<Gpio>> = Mutex::new(None);

/// GPIO configurations for different device types
pub mod config {
    #![allow(dead_code)]
    use super::*;

    /// Standard configuration for original hardware
    pub fn standard() -> PinConfig {
        PinConfig {
            status_display_pin: 25,
            iec_pins: IecPinConfig {
                clock_in: 2,
                clock_out: 3,
                data_in: 4,
                data_out: 5,
                atn_in: 6,
                atn_out: 7,
                reset_in: 8,
                reset_out: 9,
                srq_in: 10,
                srq_out: 11,
            },
        }
    }

    /// Configuration for pico1541w variant
    pub fn pico1541w() -> PinConfig {
        PinConfig {
            status_display_pin: 0,
            iec_pins: IecPinConfig {
                clock_in: 2,
                clock_out: 3,
                data_in: 4,
                data_out: 5,
                atn_in: 6,
                atn_out: 7,
                reset_in: 8,
                reset_out: 9,
                srq_in: 10,
                srq_out: 11,
            },
        }
    }
}

/// Pin configuration for different device types
pub struct PinConfig {
    pub status_display_pin: usize,
    pub iec_pins: IecPinConfig,
}

/// IEC Bus pin configuration
pub struct IecPinConfig {
    pub clock_in: usize,
    pub clock_out: usize,
    pub data_in: usize,
    pub data_out: usize,
    pub atn_in: usize,
    pub atn_out: usize,
    pub reset_in: usize,
    pub reset_out: usize,
    pub srq_in: usize,
    pub srq_out: usize,
}

/// Default pin configuration
impl Default for PinConfig {
    fn default() -> Self {
        #[cfg(not(feature = "pico1541w"))]
        {
            config::standard()
        }

        #[cfg(feature = "pico1541w")]
        {
            config::pico1541w()
        }
    }
}

/// Object which provides methods to create objects that require GPIO pins.
pub struct Gpio {
    pins: [Option<AnyPin>; 30],
    config: PinConfig,
}

impl Gpio {
    // Create a new instance of the Gpio object and create the GPIO static.
    #[allow(clippy::too_many_arguments)]
    pub async fn create_static(
        pin0: PIN_0,
        pin1: PIN_1,
        pin2: PIN_2,
        pin3: PIN_3,
        pin4: PIN_4,
        pin5: PIN_5,
        pin6: PIN_6,
        pin7: PIN_7,
        pin8: PIN_8,
        pin9: PIN_9,
        pin10: PIN_10,
        pin11: PIN_11,
        pin12: PIN_12,
        pin13: PIN_13,
        pin14: PIN_14,
        pin15: PIN_15,
        pin16: PIN_16,
        pin17: PIN_17,
        pin18: PIN_18,
        pin19: PIN_19,
        pin20: PIN_20,
        pin21: PIN_21,
        pin22: PIN_22,
        pin23: PIN_23,
        pin24: PIN_24,
        pin25: PIN_25,
        pin26: PIN_26,
        pin27: PIN_27,
        pin28: PIN_28,
        pin29: PIN_29,
        config: Option<PinConfig>,
    ) {
        let config = config.unwrap_or_default();

        // Convert all pins to AnyPin and place in array
        let pins_array = [
            Some(pin0.degrade()),
            Some(pin1.degrade()),
            Some(pin2.degrade()),
            Some(pin3.degrade()),
            Some(pin4.degrade()),
            Some(pin5.degrade()),
            Some(pin6.degrade()),
            Some(pin7.degrade()),
            Some(pin8.degrade()),
            Some(pin9.degrade()),
            Some(pin10.degrade()),
            Some(pin11.degrade()),
            Some(pin12.degrade()),
            Some(pin13.degrade()),
            Some(pin14.degrade()),
            Some(pin15.degrade()),
            Some(pin16.degrade()),
            Some(pin17.degrade()),
            Some(pin18.degrade()),
            Some(pin19.degrade()),
            Some(pin20.degrade()),
            Some(pin21.degrade()),
            Some(pin22.degrade()),
            Some(pin23.degrade()),
            Some(pin24.degrade()),
            Some(pin25.degrade()),
            Some(pin26.degrade()),
            Some(pin27.degrade()),
            Some(pin28.degrade()),
            Some(pin29.degrade()),
        ];

        let gpio = Self {
            pins: pins_array,
            config,
        };

        // Locking section
        {
            // Put the gpio object into the GPIO static.
            let mut g = GPIO.lock().await;
            if g.is_some() {
                panic!("GPIO static already set");
            }
            let _ = g.insert(gpio);
        }
    }

    /// Get the pin used for the status display.
    pub fn get_status_display_pin(&mut self) -> AnyPin {
        self.take_pin(self.config.status_display_pin)
    }

    /// Creates IecBus object.
    pub fn create_iec_bus(&mut self) -> IecBus {
        // Create the IEC bus object from the configured pins
        IecBus::new(
            self.take_pin(self.config.iec_pins.clock_in),
            self.take_pin(self.config.iec_pins.clock_out),
            self.take_pin(self.config.iec_pins.data_in),
            self.take_pin(self.config.iec_pins.data_out),
            self.take_pin(self.config.iec_pins.atn_in),
            self.take_pin(self.config.iec_pins.atn_out),
            self.take_pin(self.config.iec_pins.reset_in),
            self.take_pin(self.config.iec_pins.reset_out),
            self.take_pin(self.config.iec_pins.srq_in),
            self.take_pin(self.config.iec_pins.srq_out),
        )
    }

    /// Helper to take a pin by index
    fn take_pin(&mut self, index: usize) -> AnyPin {
        match self.pins[index].take() {
            Some(pin) => pin,
            None => {
                error!("Pin {} already taken", index);
                panic!()
            }
        }
    }
}
