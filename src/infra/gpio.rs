//! This file handles GPIO pin allocation.
//!
//! The source code serves as the master list of pin assignments for the
//! hardware.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{AnyPin, Flex};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, PIN_12,
    PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_23, PIN_24,
    PIN_25, PIN_26, PIN_27, PIN_28, PIN_29,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

//
// Statics
//

// Static Gpio object
pub static GPIO: Mutex<CriticalSectionRawMutex, Option<Gpio>> = Mutex::new(None);

/// GPIO configurations for different device types
pub mod config {
    // We allow dead code in this module because some of the configuration
    // settings wil not be included, depending on the device the firmware is
    // being built for.
    #![allow(dead_code)]
    use crate::constants::{WIFI_CLK_PIN, WIFI_CS_PIN, WIFI_DIO_PIN, WIFI_PWR_PIN};

    use super::{IecPinConfig, IeeePinConfig, PinConfig, WiFiPinConfig};

    /// Configuration for v0.1 board
    pub fn standard_v0_1() -> PinConfig {
        PinConfig {
            status_display_pin: 25,
            iec_pins: iec_pico1541_v0_1(),
            ieee_pins: ieee_pico1541_v0_1(),
            wifi_pins: Some(wifi_config()),
        }
    }

    // Pinout used for pico1541 v0.1 board
    fn iec_pico1541_v0_1() -> IecPinConfig {
        IecPinConfig {
            clock_in: 19,
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

    // Pinout used for pico1541 v0.1 board
    fn ieee_pico1541_v0_1() -> IeeePinConfig {
        IeeePinConfig {
            nrfd_in: 19,
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

    // WiFi pins used by the Pico W.
    fn wifi_config() -> WiFiPinConfig {
        WiFiPinConfig {
            pwr: WIFI_PWR_PIN,
            cs: WIFI_CS_PIN,
            dio: WIFI_DIO_PIN,
            clk: WIFI_CLK_PIN,
        }
    }
}

/// Pin configuration for different device types
pub struct PinConfig {
    pub status_display_pin: u8,
    pub iec_pins: IecPinConfig,
    pub ieee_pins: IeeePinConfig,
    pub wifi_pins: Option<WiFiPinConfig>,
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

#[derive(Clone)]
pub struct WiFiPinConfig {
    pub pwr: u8,
    pub cs: u8,
    pub dio: u8,
    pub clk: u8,
}

/// Default pin configuration
impl Default for PinConfig {
    fn default() -> Self {
        config::standard_v0_1()
    }
}

/// Object which provides methods to create objects that require GPIO pins.
pub struct Gpio {
    pin0: Option<PIN_0>,
    pin1: Option<PIN_1>,
    pin2: Option<PIN_2>,
    pin3: Option<PIN_3>,
    pin4: Option<PIN_4>,
    pin5: Option<PIN_5>,
    pin6: Option<PIN_6>,
    pin7: Option<PIN_7>,
    pin8: Option<PIN_8>,
    pin9: Option<PIN_9>,
    pin10: Option<PIN_10>,
    pin11: Option<PIN_11>,
    pin12: Option<PIN_12>,
    pin13: Option<PIN_13>,
    pin14: Option<PIN_14>,
    pin15: Option<PIN_15>,
    pin16: Option<PIN_16>,
    pin17: Option<PIN_17>,
    pin18: Option<PIN_18>,
    pin19: Option<PIN_19>,
    pin20: Option<PIN_20>,
    pin21: Option<PIN_21>,
    pin22: Option<PIN_22>,
    pin23: Option<PIN_23>,
    pin24: Option<PIN_24>,
    pin25: Option<PIN_25>,
    pin26: Option<PIN_26>,
    pin27: Option<PIN_27>,
    pin28: Option<PIN_28>,
    pin29: Option<PIN_29>,
    config: PinConfig,
}

impl Gpio {
    // Create a new instance of the Gpio object and create the GPIO static.
    #[allow(clippy::too_many_arguments)]
    #[allow(clippy::similar_names)]
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

        let gpio = Self {
            pin0: Some(pin0),
            pin1: Some(pin1),
            pin2: Some(pin2),
            pin3: Some(pin3),
            pin4: Some(pin4),
            pin5: Some(pin5),
            pin6: Some(pin6),
            pin7: Some(pin7),
            pin8: Some(pin8),
            pin9: Some(pin9),
            pin10: Some(pin10),
            pin11: Some(pin11),
            pin12: Some(pin12),
            pin13: Some(pin13),
            pin14: Some(pin14),
            pin15: Some(pin15),
            pin16: Some(pin16),
            pin17: Some(pin17),
            pin18: Some(pin18),
            pin19: Some(pin19),
            pin20: Some(pin20),
            pin21: Some(pin21),
            pin22: Some(pin22),
            pin23: Some(pin23),
            pin24: Some(pin24),
            pin25: Some(pin25),
            pin26: Some(pin26),
            pin27: Some(pin27),
            pin28: Some(pin28),
            pin29: Some(pin29),
            config,
        };

        // Locking section
        {
            // Put the gpio object into the GPIO static.
            let mut g = GPIO.lock().await;
            #[allow(clippy::manual_assert)]
            if g.is_some() {
                panic!("GPIO static already set");
            }
            let _ = g.insert(gpio);
        }
    }

    /// Get the pin used for the status display.
    pub fn get_status_display_pin(&mut self) -> AnyPin {
        match self.take_pin_as_any(self.config.status_display_pin) {
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

    /// Get the Wi-Fi pins
    pub fn get_wifi_pins(&self) -> Option<WiFiPinConfig> {
        self.config.wifi_pins.clone()
    }

    pub fn take_flex_pin(&mut self, index: u8) -> Option<Flex<'static>> {
        self.take_pin_as_any(index).map(Flex::new)
    }

    pub fn take_output(&mut self, index: u8, level: Level) -> Option<Output<'static>> {
        self.take_pin_as_any(index)
            .map(|pin| Output::new(pin, level))
    }

    /// Helper to take a pin by index
    fn take_pin_as_any(&mut self, index: u8) -> Option<AnyPin> {
        match index {
            0 => self.pin0.take().map(AnyPin::from),
            1 => self.pin1.take().map(AnyPin::from),
            2 => self.pin2.take().map(AnyPin::from),
            3 => self.pin3.take().map(AnyPin::from),
            4 => self.pin4.take().map(AnyPin::from),
            5 => self.pin5.take().map(AnyPin::from),
            6 => self.pin6.take().map(AnyPin::from),
            7 => self.pin7.take().map(AnyPin::from),
            8 => self.pin8.take().map(AnyPin::from),
            9 => self.pin9.take().map(AnyPin::from),
            10 => self.pin10.take().map(AnyPin::from),
            11 => self.pin11.take().map(AnyPin::from),
            12 => self.pin12.take().map(AnyPin::from),
            13 => self.pin13.take().map(AnyPin::from),
            14 => self.pin14.take().map(AnyPin::from),
            15 => self.pin15.take().map(AnyPin::from),
            16 => self.pin16.take().map(AnyPin::from),
            17 => self.pin17.take().map(AnyPin::from),
            18 => self.pin18.take().map(AnyPin::from),
            19 => self.pin19.take().map(AnyPin::from),
            20 => self.pin20.take().map(AnyPin::from),
            21 => self.pin21.take().map(AnyPin::from),
            22 => self.pin22.take().map(AnyPin::from),
            23 => self.pin23.take().map(AnyPin::from),
            24 => self.pin24.take().map(AnyPin::from),
            25 => self.pin25.take().map(AnyPin::from),
            26 => self.pin26.take().map(AnyPin::from),
            27 => self.pin27.take().map(AnyPin::from),
            28 => self.pin28.take().map(AnyPin::from),
            29 => self.pin29.take().map(AnyPin::from),
            _ => {
                warn!("Attempt to take non-existant pin");
                None
            }
        }
    }

    #[allow(dead_code)]
    pub fn take_pin0(&mut self) -> Option<PIN_0> {
        self.pin0.take()
    }

    #[allow(dead_code)]
    pub fn take_pin1(&mut self) -> Option<PIN_1> {
        self.pin1.take()
    }

    #[allow(dead_code)]
    pub fn take_pin2(&mut self) -> Option<PIN_2> {
        self.pin2.take()
    }

    #[allow(dead_code)]
    pub fn take_pin3(&mut self) -> Option<PIN_3> {
        self.pin3.take()
    }

    #[allow(dead_code)]
    pub fn take_pin4(&mut self) -> Option<PIN_4> {
        self.pin4.take()
    }

    #[allow(dead_code)]
    pub fn take_pin5(&mut self) -> Option<PIN_5> {
        self.pin5.take()
    }

    #[allow(dead_code)]
    pub fn take_pin6(&mut self) -> Option<PIN_6> {
        self.pin6.take()
    }

    #[allow(dead_code)]
    pub fn take_pin7(&mut self) -> Option<PIN_7> {
        self.pin7.take()
    }

    #[allow(dead_code)]
    pub fn take_pin8(&mut self) -> Option<PIN_8> {
        self.pin8.take()
    }

    #[allow(dead_code)]
    pub fn take_pin9(&mut self) -> Option<PIN_9> {
        self.pin9.take()
    }

    #[allow(dead_code)]
    pub fn take_pin10(&mut self) -> Option<PIN_10> {
        self.pin10.take()
    }

    #[allow(dead_code)]
    pub fn take_pin11(&mut self) -> Option<PIN_11> {
        self.pin11.take()
    }

    #[allow(dead_code)]
    pub fn take_pin12(&mut self) -> Option<PIN_12> {
        self.pin12.take()
    }

    #[allow(dead_code)]
    pub fn take_pin13(&mut self) -> Option<PIN_13> {
        self.pin13.take()
    }

    #[allow(dead_code)]
    pub fn take_pin14(&mut self) -> Option<PIN_14> {
        self.pin14.take()
    }

    #[allow(dead_code)]
    pub fn take_pin15(&mut self) -> Option<PIN_15> {
        self.pin15.take()
    }

    #[allow(dead_code)]
    pub fn take_pin16(&mut self) -> Option<PIN_16> {
        self.pin16.take()
    }

    #[allow(dead_code)]
    pub fn take_pin17(&mut self) -> Option<PIN_17> {
        self.pin17.take()
    }

    #[allow(dead_code)]
    pub fn take_pin18(&mut self) -> Option<PIN_18> {
        self.pin18.take()
    }

    #[allow(dead_code)]
    pub fn take_pin19(&mut self) -> Option<PIN_19> {
        self.pin19.take()
    }

    #[allow(dead_code)]
    pub fn take_pin20(&mut self) -> Option<PIN_20> {
        self.pin20.take()
    }

    #[allow(dead_code)]
    pub fn take_pin21(&mut self) -> Option<PIN_21> {
        self.pin21.take()
    }

    #[allow(dead_code)]
    pub fn take_pin22(&mut self) -> Option<PIN_22> {
        self.pin22.take()
    }

    #[allow(dead_code)]
    pub fn take_pin23(&mut self) -> Option<PIN_23> {
        self.pin23.take()
    }

    #[allow(dead_code)]
    pub fn take_pin24(&mut self) -> Option<PIN_24> {
        self.pin24.take()
    }

    #[allow(dead_code)]
    pub fn take_pin25(&mut self) -> Option<PIN_25> {
        self.pin25.take()
    }

    #[allow(dead_code)]
    pub fn take_pin26(&mut self) -> Option<PIN_26> {
        self.pin26.take()
    }

    #[allow(dead_code)]
    pub fn take_pin27(&mut self) -> Option<PIN_27> {
        self.pin27.take()
    }

    #[allow(dead_code)]
    pub fn take_pin28(&mut self) -> Option<PIN_28> {
        self.pin28.take()
    }

    #[allow(dead_code)]
    pub fn take_pin29(&mut self) -> Option<PIN_29> {
        self.pin29.take()
    }
}
