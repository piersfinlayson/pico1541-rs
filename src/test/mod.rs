//! Various test objects for pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_rp::gpio::{AnyPin, Drive, Input, Level, Output, Pin, Pull};
use embassy_rp::Peripherals;

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
        output.set_drive_strength(Drive::_12mA);
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
    pub fn new(name: &'static str, pin: AnyPin, pull: Pull) -> Self {
        let num = pin.pin();
        let input = Input::new(pin, pull);
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

pub fn create_pins(p: Peripherals, input: bool, output: bool) -> (Option<[InputPin; 5]>, Option<[OutputPin; 5]>) {
    let output_pins = if output {
        // Create the pin objects
        let clock = OutputPin::new("clock", p.PIN_11.into());
        assert_eq!(clock.num, IEC_PINS_OUT.clock);
        let data = OutputPin::new("data", p.PIN_13.into());
        assert_eq!(data.num, IEC_PINS_OUT.data);
        let atn = OutputPin::new("atn", p.PIN_12.into());
        assert_eq!(atn.num, IEC_PINS_OUT.atn);
        let reset = OutputPin::new("reset", p.PIN_10.into());
        assert_eq!(reset.num, IEC_PINS_OUT.reset);
        let srq = OutputPin::new("srq", p.PIN_14.into());
        assert_eq!(srq.num, IEC_PINS_OUT.srq);

        let output_pins = [
            clock,
            data,
            atn,
            reset,
            srq,
        ];

        Some(output_pins)
    } else {
        let clock = InputPin::new("clock", p.PIN_11.into(), Pull::None);
        assert!(clock.num == IEC_PINS_OUT.clock);
        let data = InputPin::new("data", p.PIN_13.into(), Pull::None);
        assert!(data.num == IEC_PINS_OUT.data);
        let atn = InputPin::new("atn", p.PIN_12.into(), Pull::None);
        assert!(atn.num == IEC_PINS_OUT.atn);
        let reset = InputPin::new("reset", p.PIN_10.into(), Pull::None);
        assert!(reset.num == IEC_PINS_OUT.reset);
        let srq = InputPin::new("srq", p.PIN_14.into(), Pull::None);
        assert!(srq.num == IEC_PINS_OUT.srq);

        None
    };

    let input_pins = if input {
        let clock = InputPin::new("clock", p.PIN_19.into(), Pull::None);
        assert_eq!(clock.num, IEC_PINS_IN.clock);
        let data = InputPin::new("data", p.PIN_20.into(), Pull::None);
        assert_eq!(data.num, IEC_PINS_IN.data);
        let atn = InputPin::new("atn", p.PIN_17.into(), Pull::None);
        assert_eq!(atn.num, IEC_PINS_IN.atn);
        let reset = InputPin::new("reset", p.PIN_18.into(), Pull::None);
        assert_eq!(reset.num, IEC_PINS_IN.reset);
        let srq = InputPin::new("srq", p.PIN_16.into(), Pull::None);
        assert_eq!(srq.num, IEC_PINS_IN.srq);
    
        let input_pins = [
            clock,
            data,
            atn,
            reset,
            srq,
        ];

        Some(input_pins)
    } else {
        None
    };

    (input_pins, output_pins)
}