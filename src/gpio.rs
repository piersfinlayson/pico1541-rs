//! This file handles GPIO pin allocation.
//!
//! The source code serves as the master list of pin assignments for the
//! hardware.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use crate::constants::{INVALID_GPIO_PINS, MAX_GPIO_PINS};
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{AnyPin, Flex, Pin};
use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19,
    PIN_2, PIN_20, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27, PIN_28, PIN_29, PIN_3,
    PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

//
// Statics
//

pub static GPIO: Mutex<CriticalSectionRawMutex, Option<Gpio>> = Mutex::new(None);

/// GPIO configurations for different device types
pub mod config {
    // We allow dead code in this module because some of the configuration
    // settings wil not be included, depending on the device the firmware is
    // being built for.
    #![allow(dead_code)]
    use super::*;

    /// Standard configuration for original hardware
    pub fn standard_protoype() -> PinConfig {
        PinConfig {
            status_display_pin: 25,
            iec_pins: iec_prototype(),
            ieee_pins: ieee_prototype(),
        }
    }

    /// Configuration for v0.1 board
    pub fn standard_v0_1() -> PinConfig {
        PinConfig {
            status_display_pin: 25,
            iec_pins: iec_pico1541_v0_1(),
            ieee_pins: ieee_pico1541_v0_1(),
        }
    }

    /// Configuration for pico1541w variant
    pub fn pico1541w_protoype() -> PinConfig {
        PinConfig {
            status_display_pin: 0,
            iec_pins: iec_prototype(),
            ieee_pins: ieee_prototype(),
        }
    }

    /// Configuration for pico1541w v0.1 board
    pub fn pico1541w_v0_1() -> PinConfig {
        PinConfig {
            status_display_pin: 0,
            iec_pins: iec_pico1541_v0_1(),
            ieee_pins: ieee_pico1541_v0_1(),
        }
    }

    // Pinout used for initial prototyping
    fn iec_prototype() -> IecPinConfig {
        IecPinConfig {
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
        }
    }

    // Pinout used for pico1541 v0.1 board
    fn iec_pico1541_v0_1() -> IecPinConfig {
        IecPinConfig {
            clock_in: 21,
            clock_out: 11,
            data_in: 20,
            data_out: 13,
            atn_in: 17,
            atn_out: 12,
            reset_in: 18,
            reset_out: 10,
            srq_in: 16,
            srq_out: 14,
        }
    }

    // Pinout used for initial prototyping
    fn ieee_prototype() -> IeeePinConfig {
        IeeePinConfig {
            nrfd_in: 2,
            nrfd_out: 3,
            ndac_in: 4,
            ndac_out: 5,
            atn_in: 6,
            atn_out: 7,
            ifc_in: 8,
            ifc_out: 9,
            srq_in: 10,
            srq_out: 11,
            d_io: [12, 13, 14, 15, 16, 17, 18, 19],
        }
    }

    // Pinout used for pico1541 v0.1 board
    fn ieee_pico1541_v0_1() -> IeeePinConfig {
        IeeePinConfig {
            nrfd_in: 21,
            nrfd_out: 11,
            ndac_in: 20,
            ndac_out: 13,
            atn_in: 17,
            atn_out: 12,
            ifc_in: 18,
            ifc_out: 10,
            srq_in: 16,
            srq_out: 14,
            d_io: [2, 3, 4, 5, 9, 8, 7, 6],
        }
    }
}

/// Pin configuration for different device types
pub struct PinConfig {
    pub status_display_pin: u8,
    pub iec_pins: IecPinConfig,
    pub ieee_pins: IeeePinConfig,
}

/// IEC Bus pin configuration
#[derive(Clone)]
pub struct IecPinConfig {
    pub clock_in: u8,
    pub clock_out: u8,
    pub data_in: u8,
    pub data_out: u8,
    pub atn_in: u8,
    pub atn_out: u8,
    pub reset_in: u8,
    pub reset_out: u8,
    pub srq_in: u8,
    pub srq_out: u8,
}

impl IntoIterator for IecPinConfig {
    type Item = u8;
    type IntoIter = core::array::IntoIter<u8, 10>;

    fn into_iter(self) -> Self::IntoIter {
        [
            self.clock_in,
            self.clock_out,
            self.data_in,
            self.data_out,
            self.atn_in,
            self.atn_out,
            self.reset_in,
            self.reset_out,
            self.srq_in,
            self.srq_out,
        ]
        .into_iter()
    }
}

/// IEEE Bus pin configuration
#[derive(Clone)]
pub struct IeeePinConfig {
    pub nrfd_in: u8,
    pub nrfd_out: u8,
    pub ndac_in: u8,
    pub ndac_out: u8,
    pub atn_in: u8,
    pub atn_out: u8,
    pub ifc_in: u8,
    pub ifc_out: u8,
    pub srq_in: u8,
    pub srq_out: u8,
    pub d_io: [u8; 8],
}

impl IntoIterator for IeeePinConfig {
    type Item = u8;
    type IntoIter = core::array::IntoIter<u8, 18>;

    fn into_iter(self) -> Self::IntoIter {
        [
            self.nrfd_in,
            self.nrfd_out,
            self.ndac_in,
            self.ndac_out,
            self.atn_in,
            self.atn_out,
            self.ifc_in,
            self.ifc_out,
            self.srq_in,
            self.srq_out,
            self.d_io[0],
            self.d_io[1],
            self.d_io[2],
            self.d_io[3],
            self.d_io[4],
            self.d_io[5],
            self.d_io[6],
            self.d_io[7],
        ]
        .into_iter()
    }
}

/// Default pin configuration
impl Default for PinConfig {
    fn default() -> Self {
        #[cfg(not(feature = "pico1541w"))]
        {
            #[cfg(feature = "prototype")]
            {
                config::standard_protoype()
            }
            #[cfg(not(feature = "prototype"))]
            {
                config::standard_v0_1()
            }
        }

        #[cfg(feature = "pico1541w")]
        {
            #[cfg(feature = "prototype")]
            {
                config::pico1541w_protoype()
            }
            #[cfg(not(feature = "prototype"))]
            {
                config::pico1541w_v0_1()
            }
        }
    }
}

/// Object which provides methods to create objects that require GPIO pins.
pub struct Gpio {
    pins: [Option<AnyPin>; MAX_GPIO_PINS as usize],
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
        let mut pins_array = [
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

        // Go through the pins and throw away any that are invalid
        // (unassignable).
        for pin in INVALID_GPIO_PINS.iter() {
            pins_array[*pin as usize] = None;
        }

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
        match self.take_pin(self.config.status_display_pin) {
            Some(pin) => pin,
            None => panic!(
                "Status display pin {} already taken",
                self.config.status_display_pin
            ),
        }
    }

    /// Get the IEC pins
    pub fn get_iec_pins(&self) -> IecPinConfig {
        self.config.iec_pins.clone()
    }

    /// Get the IEEE pins
    pub fn get_ieee_pins(&self) -> IeeePinConfig {
        self.config.ieee_pins.clone()
    }

    pub fn take_flex_pin(&mut self, index: u8) -> Option<Flex<'static>> {
        self.take_pin(index).map(Flex::new)
    }

    /// Helper to take a pin by index
    fn take_pin(&mut self, index: u8) -> Option<AnyPin> {
        self.pins[index as usize].take()
    }
}
