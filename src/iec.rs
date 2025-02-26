//! This file implements the Commodore IEC protocol driver.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![allow(dead_code)]

use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};

//use crate::driver::{DriverError, ProtocolDriver};
use crate::display::{update_status, DisplayType};

// The IEC protocol driver implementation
pub struct IecDriver {
    bus: IecBus,
    eoi: bool,
}

impl IecDriver {
    pub fn new(bus: IecBus) -> Self {
        Self { bus, eoi: false }
    }

    // Check if host signaled an abort
    async fn check_abort(&self) -> bool {
        // Access your USB state through whatever mechanism you have
        // e.g.: USB_STATE.with(|state| state.borrow().check_abort())
        false // Placeholder - replace with actual implementation
    }
}

/*
impl ProtocolDriver for IecDriver {
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        defmt::debug!("reset");
        self.bus.release_lines(IO_DATA | IO_ATN | IO_CLK | IO_SRQ);

        // Reset EOI state
        self.eoi = false;

        // Hold reset line active
        self.bus.set_reset();
        Timer::after_millis(100).await;
        self.bus.release_reset();

        self.wait_for_free_bus(forever).await
    }

    async fn raw_read(&mut self, len: u16) -> Result<u16, DriverError> {
        defmt::info!("crd {}", len);

        // Notify USB system we're starting an I/O operation
        // Handle this through your existing USB mechanism

        self.eoi = false;
        let mut count = 0;

        while count < len {
            // Read logic here that will update self.eoi

            if self.eoi {
                break;
            }

            count += 1;
        }

        // Notify USB system we're done with I/O

        Ok(count)
    }

    // Other implementations...
}
*/

// An object representing the physical IEC bus.  Each Line is a pair of
// pins, one input and one output.
pub struct IecBus {
    data: Line,
    clock: Line,
    atn: Line,
    reset: Line,
    srq: Line,
}

// IEC pin bit masks (direct port mapping)
pub const IO_DATA: u8 = 0x01;
pub const IO_CLK: u8 = 0x02;
pub const IO_ATN: u8 = 0x04;
pub const IO_RESET: u8 = 0x08;
pub const IO_SRQ: u8 = 0x10;

// IEC protocol bit masks (logical representation)
pub const IEC_DATA: u8 = 0x01;
pub const IEC_CLOCK: u8 = 0x02;
pub const IEC_ATN: u8 = 0x04;
pub const IEC_SRQ: u8 = 0x08;

// Write flags
pub const XUM_WRITE_ATN: u8 = 0x01;
pub const XUM_WRITE_TALK: u8 = 0x02;

// LED status constants
pub const STATUS_INIT: u8 = 0;
pub const STATUS_READY: u8 = 1;
pub const STATUS_ACTIVE: u8 = 2;
pub const STATUS_ERROR: u8 = 3;

/// Represents a single bidirectional IEC bus line using separate input/output pins
pub struct Line {
    input_pin: Input<'static>,
    output_pin: Output<'static>,
}

impl Line {
    /// Create a new Line with the specified input and output pins
    pub fn new(input_pin: impl Pin, output_pin: impl Pin) -> Self {
        // Initialize with input having pull-up and output high (released state)
        Self {
            input_pin: Input::new(input_pin, Pull::Up),
            output_pin: Output::new(output_pin, Level::High),
        }
    }

    /// Drive the line low (active)
    pub fn set(&mut self) {
        self.output_pin.set_low();
    }

    /// Release the line (inactive)
    pub fn release(&mut self) {
        self.output_pin.set_high();
    }

    /// Read the current state of the line
    pub fn get(&self) -> bool {
        self.input_pin.is_high()
    }

    /// Check if line is currently being driven low
    pub fn is_set(&self) -> bool {
        self.output_pin.is_set_low()
    }
}

impl IecBus {
    /// Create a new IEC bus with the specified pins
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        clock_in: impl Pin,
        clock_out: impl Pin,
        data_in: impl Pin,
        data_out: impl Pin,
        srq_in: impl Pin,
        srq_out: impl Pin,
        atn_in: impl Pin,
        atn_out: impl Pin,
        reset_in: impl Pin,
        reset_out: impl Pin,
    ) -> Self {
        let mut bus = Self {
            data: Line::new(data_in, data_out),
            clock: Line::new(clock_in, clock_out),
            atn: Line::new(atn_in, atn_out),
            reset: Line::new(reset_in, reset_out),
            srq: Line::new(srq_in, srq_out),
        };

        // Initialize all pins to released state (similar to board_init_iec)
        bus.release_lines(IO_DATA | IO_CLK | IO_ATN | IO_RESET | IO_SRQ);

        // Set initial status - LED on
        update_status(DisplayType::Init);

        bus
    }

    // DATA line control
    pub fn set_data(&mut self) {
        self.data.set();
    }

    pub fn release_data(&mut self) {
        self.data.release();
    }

    pub fn get_data(&self) -> bool {
        self.data.get()
    }

    // CLOCK line control
    pub fn set_clock(&mut self) {
        self.clock.set();
    }

    pub fn release_clock(&mut self) {
        self.clock.release();
    }

    pub fn get_clock(&self) -> bool {
        self.clock.get()
    }

    // ATN line control
    pub fn set_atn(&mut self) {
        self.atn.set();
    }

    pub fn release_atn(&mut self) {
        self.atn.release();
    }

    pub fn get_atn(&self) -> bool {
        self.atn.get()
    }

    // RESET line control
    pub fn set_reset(&mut self) {
        self.reset.set();
    }

    pub fn release_reset(&mut self) {
        self.reset.release();
    }

    pub fn get_reset(&self) -> bool {
        self.reset.get()
    }

    // SRQ line control (if available)
    pub fn set_srq(&mut self) {
        self.srq.set();
    }

    pub fn release_srq(&mut self) {
        self.srq.release();
    }

    pub fn get_srq(&self) -> bool {
        self.srq.get()
    }

    /// Set multiple lines at once based on a bit mask
    pub fn set_lines(&mut self, mask: u8) {
        if (mask & IO_DATA) != 0 {
            self.set_data();
        }
        if (mask & IO_CLK) != 0 {
            self.set_clock();
        }
        if (mask & IO_ATN) != 0 {
            self.set_atn();
        }
        if (mask & IO_RESET) != 0 {
            self.set_reset();
        }
        if (mask & IO_SRQ) != 0 {
            self.set_srq();
        }
    }

    /// Release multiple lines at once based on a bit mask
    pub fn release_lines(&mut self, mask: u8) {
        if (mask & IO_DATA) != 0 {
            self.release_data();
        }
        if (mask & IO_CLK) != 0 {
            self.release_clock();
        }
        if (mask & IO_ATN) != 0 {
            self.release_atn();
        }
        if (mask & IO_RESET) != 0 {
            self.release_reset();
        }
        if (mask & IO_SRQ) != 0 {
            self.release_srq();
        }
    }

    /// Combined set and release operation
    pub fn set_release(&mut self, set: u8, release: u8) {
        self.set_lines(set);
        self.release_lines(release);
    }

    /// Poll all pins - returns a bit mask of active (low) lines
    /// This mimics the iec_poll_pins function in the original code
    pub fn poll_pins(&self) -> u8 {
        let mut result = 0;
        if !self.get_data() {
            result |= IO_DATA;
        }
        if !self.get_clock() {
            result |= IO_CLK;
        }
        if !self.get_atn() {
            result |= IO_ATN;
        }
        if !self.get_reset() {
            result |= IO_RESET;
        }
        if !self.get_srq() {
            result |= IO_SRQ;
        }
        result
    }

    /// Convert between logical and physical IEC line representations
    pub fn iec_to_hw(&self, iec: u8) -> u8 {
        let mut hw = 0;
        if (iec & IEC_DATA) != 0 {
            hw |= IO_DATA;
        }
        if (iec & IEC_CLOCK) != 0 {
            hw |= IO_CLK;
        }
        if (iec & IEC_ATN) != 0 {
            hw |= IO_ATN;
        }
        if (iec & IEC_SRQ) != 0 {
            hw |= IO_SRQ;
        }
        hw
    }

    /// Convert from IEC logical representation to physical, then set lines
    pub fn iec_set(&mut self, iec: u8) {
        self.set_lines(self.iec_to_hw(iec));
    }

    /// Convert from IEC logical representation to physical, then release lines
    pub fn iec_release(&mut self, iec: u8) {
        self.release_lines(self.iec_to_hw(iec));
    }

    /// Convert from IEC logical representation to physical, then perform set/release
    pub fn iec_set_release(&mut self, set: u8, release: u8) {
        self.set_release(self.iec_to_hw(set), self.iec_to_hw(release));
    }

    /// Poll pins and convert to IEC logical representation
    pub fn iec_poll(&self) -> u8 {
        let pins = self.poll_pins();
        let mut iec = 0;

        if (pins & IO_DATA) != 0 {
            iec |= IEC_DATA;
        }
        if (pins & IO_CLK) != 0 {
            iec |= IEC_CLOCK;
        }
        if (pins & IO_ATN) != 0 {
            iec |= IEC_ATN;
        }
        if (pins & IO_SRQ) != 0 {
            iec |= IEC_SRQ;
        }

        iec
    }
}
