//! This file handles GPIO pin allocation.
//! 
//! The source code serves as the master list of pin assignments for the
//! hardware. 

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_rp::peripherals::{
    PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9,
    PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19,
    PIN_20, PIN_21, PIN_22, PIN_23, PIN_24, PIN_25, PIN_26, PIN_27, PIN_28, PIN_29,
};
use crate::display::StatusDisplay;
use crate::iec::IecBus;

/// Object which provides methods to create objects that require GPIO pins.
pub struct Gpio {}

impl Gpio {
    /// Creates all objects which require GPIO pins.
    /// Creates:
    /// - IEC bus object
    /// - Status display object
    /// 
    /// Returns a tuple containing the created objects back to the caller,
    /// with the except of the StatusDisplay object, which is stored in a
    /// static variable.
    pub fn create_pin_objects(
        pin_0: PIN_0, _pin_1: PIN_1, pin_2: PIN_2, pin_3: PIN_3,
        pin_4: PIN_4, pin_5: PIN_5, pin_6: PIN_6, pin_7: PIN_7,
        pin_8: PIN_8, pin_9: PIN_9, pin_10: PIN_10, pin_11: PIN_11,
        _pin_12: PIN_12, _pin_13: PIN_13, _pin_14: PIN_14, _pin_15: PIN_15,
        _pin_16: PIN_16, _pin_17: PIN_17, _pin_18: PIN_18, _pin_19: PIN_19,
        _pin_20: PIN_20, _pin_21: PIN_21, _pin_22: PIN_22, _pin_23: PIN_23,
        _pin_24: PIN_24, pin_25: PIN_25, _pin_26: PIN_26, _pin_27: PIN_27,
        _pin_28: PIN_28, _pin_29: PIN_29,
    ) -> (IecBus, ) {
        // Create the status display object.  This initializes STATIC_DISPLAY.
        // We do this first, because the IEC object will try and update the
        // status when it's created.
        // Use different pins depending on the build variant.
        #[cfg(not(feature = "pico1541w"))]
        {
            // Not a WiFi board - so use pin 25 for status
            StatusDisplay::create_static(pin_25);
            let _ = pin_0;
        }
        #[cfg(feature = "pico1541w")]
        {
            // A WiFi board - so use pin 25 for status
            StatusDisplay::create_static(pin_0);
            let _ = _pin_25;
        }
        
        // Create the IEC bus object.  This will be passed to the Bulk to
        // pass onto ProtocolHandler.
        let iec_bus = IecBus::new(
            // Clock in/out
            pin_2, pin_3,
            // Data in/out
            pin_4, pin_5,
            // SRQ in/out
            pin_6, pin_7,
            // ATN in/out
            pin_8, pin_9,
            // RESET in/out
            pin_10, pin_11,
        );

        (iec_bus, )
    }
}