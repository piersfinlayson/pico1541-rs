//! Various test objects for pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_rp::gpio::{AnyPin, Drive, Input, Level, Output, Pin, Pull};

pub struct IecPins {
    pub clock: u8,
    pub data: u8,
    pub atn: u8,
    pub reset: u8,
    pub srq: u8,
}

pub const IEC_PINS_OUT: IecPins  = IecPins {
    clock: 11,
    data: 13,
    atn: 12,
    reset: 10,
    srq: 14,
};

pub const IEC_PINS_IN: IecPins = IecPins {
    clock: 19,
    data: 20,
    atn: 17,
    reset: 18,
    srq: 16,
};

pub struct OutputPin {
    pub name: &'static str,
    pub num: u8,
    pin: Output<'static>, 
}

impl OutputPin {
    #[must_use]
    pub fn new(name: &'static str, pin: AnyPin) -> Self {
        let num = pin.pin();
        let mut output = Output::new(pin, Level::High);
        output.set_drive_strength(Drive::_4mA);
        OutputPin { name, num, pin: output }
    }

    pub fn set_low(&mut self) {
        self.pin.set_low();
    }

    pub fn set_high(&mut self) {
        self.pin.set_high();
    }

    #[must_use]
    pub fn is_high(&self) -> bool {
        self.pin.is_set_high()
    }
}

pub struct InputPin {
    pub name: &'static str,
    pub num: u8,
    pin: Input<'static>,
    last_level: Level,
}

impl InputPin {
    #[must_use]
    pub fn new(name: &'static str, pin: AnyPin) -> Self {
        let num = pin.pin();
        let input = Input::new(pin, Pull::None);
        let last_level = input.get_level();
        InputPin { name, num, pin: input, last_level }
    }

    pub fn has_changed(&mut self) -> bool {
        let level = self.pin.get_level();
        if level != self.last_level {
            self.last_level = level;
            return true;
        }
        false
    }

    #[must_use]
    pub fn get_level(&self) -> Level {
	self.last_level
    }
}
