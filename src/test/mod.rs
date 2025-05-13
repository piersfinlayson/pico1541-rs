//! Various test objects for pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use defmt::{debug, trace};
use embassy_rp::Peripherals;
use embassy_rp::gpio::{AnyPin, Drive, Flex, Input, Level, Output, Pin, Pull};
use embassy_time::{Duration, Timer, with_timeout};

pub struct IecPins {
    pub clock: u8,
    pub data: u8,
    pub atn: u8,
    pub reset: u8,
    pub srq: u8,
}

pub const IEC_PINS_OUT: IecPins = IecPins {
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
        OutputPin {
            name,
            num,
            pin: output,
        }
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
        InputPin {
            name,
            num,
            pin: input,
            last_level,
        }
    }

    pub fn has_changed(&mut self) -> bool {
        let level = self.pin.get_level();
        if level == self.last_level {
            false
        } else {
            self.last_level = level;
            true
        }
    }

    #[must_use]
    pub fn get_level(&self) -> Level {
        self.last_level
    }
}

const NUM_PINS: usize = 5;
#[allow(clippy::missing_panics_doc)]
#[must_use]
pub fn create_pins(
    p: Peripherals,
    input: bool,
    output: bool,
) -> (Option<[InputPin; NUM_PINS]>, Option<[OutputPin; NUM_PINS]>) {
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

        let output_pins = [clock, data, atn, reset, srq];

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

    // Configure receiving pins with Pull::Down because when two TXS0108E level
    // shifters are connected back-to-back, their internal pull-ups create a
    // voltage divider with the driving 74LS04. This results in a ~1.2V signal
    // (instead of 0V) which is in the indeterminate range for the RP2040.
    // The pull-downs bias the input detection circuit to interpret this
    // borderline voltage as a LOW signal. This configuration is only necessary
    // for back-to-back testing scenarios and isn't needed when interfacing
    // with actual Commodore devices.
    let input_pins = if input {
        let clock = InputPin::new("clock", p.PIN_19.into(), Pull::Down);
        assert_eq!(clock.num, IEC_PINS_IN.clock);
        let data = InputPin::new("data", p.PIN_20.into(), Pull::Down);
        assert_eq!(data.num, IEC_PINS_IN.data);
        let atn = InputPin::new("atn", p.PIN_17.into(), Pull::Down);
        assert_eq!(atn.num, IEC_PINS_IN.atn);
        let reset = InputPin::new("reset", p.PIN_18.into(), Pull::Down);
        assert_eq!(reset.num, IEC_PINS_IN.reset);
        let srq = InputPin::new("srq", p.PIN_16.into(), Pull::Down);
        assert_eq!(srq.num, IEC_PINS_IN.srq);

        let input_pins = [clock, data, atn, reset, srq];

        Some(input_pins)
    } else {
        None
    };

    (input_pins, output_pins)
}

pub struct IecLine {
    pub name: &'static str,
    pub input: Input<'static>,
    pub output: Flex<'static>,
    is_output: bool,
}

impl IecLine {
    #[must_use]
    pub fn new(name: &'static str, input_pin: AnyPin, output_pin: AnyPin) -> Self {
        let input = Input::new(input_pin, Pull::Down);
        let mut output = Flex::new(output_pin);
        output.set_as_input();
        output.set_pull(Pull::None);
        Self {
            name,
            input,
            output,
            is_output: false,
        }
    }

    pub fn set_as_output(&mut self) {
        self.output.set_as_output();
        self.output.set_drive_strength(Drive::_4mA);
        self.is_output = true;
    }

    #[allow(clippy::missing_panics_doc)]
    pub fn set(&mut self, high: bool) {
        assert!(self.is_output, "set() called on input pin");
        // Invert
        if high {
            self.output.set_low();
        } else {
            self.output.set_high();
        }
    }

    pub fn set_as_input(&mut self) {
        self.output.set_as_input();
        self.output.set_pull(Pull::None);
        self.is_output = false;
    }

    #[allow(clippy::missing_panics_doc)]
    pub fn get(&mut self) -> bool {
        assert!(!self.is_output, "get() called on output pin");
        !self.input.is_high() // Inverg
    }

    #[allow(clippy::missing_panics_doc)]
    pub async fn wait_for_low(&mut self) {
        assert!(!self.is_output, "wait_for_low() called on output pin");
        self.output.wait_for_high().await;
    }

    #[allow(clippy::missing_panics_doc)]
    pub async fn wait_for_high(&mut self) {
        assert!(!self.is_output, "wait_for_high() called on output pin");
        self.output.wait_for_low().await;
    }
}

/// A device (as opposed to a controller) on the IEC bus
pub struct IecDevice {
    pub clock: IecLine,
    pub data: IecLine,
    pub atn: IecLine,
}

impl IecDevice {
    const MAX_RECV_BYTES: usize = 64;

    #[allow(clippy::missing_panics_doc)]
    #[must_use]
    pub fn new(p: Peripherals) -> Self {
        let clock = IecLine::new("clock", p.PIN_19.into(), p.PIN_11.into());
        assert_eq!(IEC_PINS_IN.clock, 19);
        assert_eq!(IEC_PINS_OUT.clock, 11);

        let data = IecLine::new("data", p.PIN_20.into(), p.PIN_13.into());
        assert_eq!(IEC_PINS_IN.data, 20);
        assert_eq!(IEC_PINS_OUT.data, 13);

        let atn = IecLine::new("atn", p.PIN_17.into(), p.PIN_12.into());
        assert_eq!(IEC_PINS_IN.atn, 17);
        assert_eq!(IEC_PINS_OUT.atn, 12);

        IecDevice { clock, data, atn }
    }

    async fn receive_bytes(&mut self, bytes: &mut [u8; Self::MAX_RECV_BYTES]) -> Result<usize, ()> {
        // Wait for CLK to go low
        trace!("Wait for CLK");
        self.clock.wait_for_low().await;

        // Pull DATA low to signifiy we're ready to receive
        trace!("Assert DATA");
        self.data.set_as_output();
        self.data.set(false);

        // Wait for CLK to go high and release DATA
        trace!("Wait for CLK deasserted");
        self.clock.wait_for_high().await;
        self.data.set_as_input();

        // Now the sender will pull CLK low within 200us, or it won't.  If
        // it doesn't, that's EOI.
        let mut eoi = false;
        if with_timeout(Duration::from_micros(200), self.clock.wait_for_low())
            .await
            .is_err()
        {
            trace!("EOI");
            eoi = true;

            // Pull DATA low for 60us to show we've spotted EOI
            self.data.set_as_output();
            self.data.set(false);
            Timer::after_micros(60).await;

            // Wait for CLK to go high and release DATA
            trace!("(EOI) Wait for CLK deasserted");
            self.clock.wait_for_high().await;
            self.data.set_as_input();
        }

        let mut byte_num = 0;
        loop {
            trace!("Read byte {}", byte_num);
            // Read the byte
            let mut byte: u8 = 0;
            let mut bit_num: u8 = 0;
            loop {
                // Wait for CLK to go low to signify a bit is waiting
                trace!("(Bit {}) Wait for CLK", bit_num);
                self.clock.wait_for_low().await;

                // Read data bit
                trace!("Read data bit");
                let rbit = self.data.get();
                if !rbit {
                    byte |= 1 << bit_num;
                }

                // Wait for CLK to go high
                trace!("Wait for CLK deasserted");
                self.clock.wait_for_high().await;

                bit_num += 1;
                if bit_num >= 8 {
                    break;
                }
            }

            // Show we've received the byte
            trace!("Assert DATA");
            self.data.set_as_output();
            self.data.set(false);

            bytes[byte_num] = byte;

            if eoi {
                // Give the controller a bit of time to notice we pulled DATA
                // low
                Timer::after_micros(100).await;
                self.data.set_as_input();
                break;
            }

            byte_num += 1;
        }

        Ok(byte_num + 1)
    }

    #[allow(clippy::missing_panics_doc)]
    pub async fn run(&mut self) -> ! {
        // Set all lines to inputs
        trace!("Releasing CLK, DATA, ATN");
        self.clock.set_as_input();
        self.data.set_as_input();
        self.atn.set_as_input();

        loop {
            // Wait for ATN to be asserted
            trace!("Wait for ATN");
            self.atn.wait_for_low().await;
            trace!("ATN asserted");

            // Read bytes
            let mut bytes = [0; Self::MAX_RECV_BYTES];
            let num_bytes = self.receive_bytes(&mut bytes).await.unwrap();

            debug!("Received {} bytes", num_bytes);
            let mut byte_num = 0;
            loop {
                debug!("Byte {}: 0x{:02x}", byte_num, bytes[byte_num]);
                byte_num += 1;
                if byte_num >= num_bytes {
                    break;
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run_iec_device(device: &'static mut IecDevice) -> ! {
    device.run().await
}
