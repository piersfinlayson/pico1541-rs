//! This file implements the Commodore IEC protocol driver.  Based on
//! the xum1541 source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{Flex, Pull};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use super::driver::{DriverError, ProtocolDriver};
use super::{Dio, ProtocolFlags, ProtocolType};

use crate::constants::{
    PROTOCOL_YIELD_TIMER, PROTOCOL_YIELD_TIMER_US, USB_DATA_TRANSFER_WAIT_TIMER,
};
use crate::infra::watchdog::{TaskId, WatchdogType};
use crate::usb::transfer::UsbDataTransfer;
use crate::util::time::iec::{
    BUS_FREE_CHECK_YIELD, BUS_FREE_TIMEOUT, FOREVER_TIMEOUT, LISTENER_WAIT_INTERVAL,
};
use crate::util::time::{block_us, yield_for, yield_ms, yield_us};

// IEC protocol bit masks - these are used by external applications
pub const IEC_DATA: u8 = 0x01;
pub const IEC_CLOCK: u8 = 0x02;
pub const IEC_ATN: u8 = 0x04;
#[allow(dead_code)]
pub const IEC_RESET: u8 = 0x08;
pub const IEC_SRQ: u8 = 0x10;

// IEC pin bit masks - these are used internally.  We use the same values as
// the external masks to reduce the possibility of bugs.
pub const IO_DATA: u8 = IEC_DATA;
pub const IO_CLK: u8 = IEC_CLOCK;
pub const IO_ATN: u8 = IEC_ATN;
pub const IO_RESET: u8 = IEC_RESET;
pub const IO_SRQ: u8 = IEC_SRQ;

// Timer macros, used for simple inlines.
//
// We have two choices for a delay:
// * Delay.delay_*() - blocks the current core for the duration indicated.
//   This is useful if we want precise timing.
// * Timer::after_*() - yields to the executor for the duration indicated.
//   Because the executor will schedule something else during the yield, it is
//   possible that the pause will be longer than required.
//
// So, where the precise timing is critical, we use Delay.delay*(), using the
// block_us!() macro.  Where we can tolerate a longer delay if required,
// we use the yield_*!() macros.
//
// Irrespective of which macro is used, this module must ensure watchdog is
// fed, should it be delaying for a long time.

/// Represents a single bidirectional IEC bus line using separate input/output pins
pub struct Line {
    input_pin_num: u8,
    input_pin: Option<Flex<'static>>,
    output_pin_num: u8,
    output_pin: Option<Flex<'static>>,
}

impl defmt::Format for Line {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Input: {}, Outupt: {}",
            self.input_pin_num,
            self.output_pin_num
        );
    }
}

impl Line {
    /// Create a new Line with the specified input and output pins
    pub fn new(
        input_pin_num: u8,
        input_pin: Flex<'static>,
        output_pin_num: u8,
        output_pin: Flex<'static>,
    ) -> Self {
        // Initialize with input having no pull-ups/downs (TXS0108E has pull-
        // ups on its outputs) and output low (inverted released state).
        let mut input = input_pin;
        input.set_as_input();
        input.set_pull(Pull::None);

        // Initialize the output pin as an output with low level (inverted
        // released state - which means it will be physically high on the bus)
        let mut output = output_pin;
        output.set_as_output();
        output.set_low();

        // Create the Line.
        Self {
            input_pin_num,
            output_pin_num,
            input_pin: Some(input),
            output_pin: Some(output),
        }
    }

    // Called before this Line object is dropped, in order to retrieve the
    // pins and reassign them.
    //
    // Great care must be taken when this function is called - if the Line
    // object is still used after the pins are taken, it will cause a panic
    // because the set(), release(), get() and is_set() functions all use
    // unwrap().
    pub fn take_pins(&mut self) -> (u8, Option<Flex<'static>>, u8, Option<Flex<'static>>) {
        (
            self.input_pin_num,
            self.input_pin.take(),
            self.output_pin_num,
            self.output_pin.take(),
        )
    }

    /// Drive the output line low (active) - ouptut is inverted pin so high
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set(&mut self) {
        self.output_pin.as_mut().unwrap().set_high();
    }

    /// Release the output line (inactive) - output is inverted pin so low
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release(&mut self) {
        self.output_pin.as_mut().unwrap().set_low();
    }

    /// Read the current state of the line.  This returns true is the line
    /// is active - as it is not inverted, this means true if low.
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get(&self) -> bool {
        self.input_pin.as_ref().unwrap().is_low()
    }

    /// Check if line is currently being driven low - inverted pin so high
    #[allow(clippy::inline_always)]
    #[allow(dead_code)]
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        self.output_pin.as_ref().unwrap().is_set_high()
    }
}

// An object representing the physical IEC bus.  Each Line is a pair of
// pins, one input and one output.  Pin assignments are in `gpio.rs`.
pub struct IecBus {
    clock: Line,
    data: Line,
    atn: Line,
    reset: Line,
    srq: Line,
    pub dio: [Dio; 8],
}

impl IecBus {
    /// Create a new IEC bus with the specified pins
    #[allow(clippy::too_many_arguments)]
    pub fn new(clock: Line, data: Line, atn: Line, reset: Line, srq: Line, dio: [Dio; 8]) -> Self {
        let mut bus = Self {
            clock,
            data,
            atn,
            reset,
            srq,
            dio,
        };

        // Initialize all pins to released state (similar to board_init_iec)
        bus.release_lines(IO_DATA | IO_CLK | IO_ATN | IO_RESET | IO_SRQ);

        bus
    }

    pub fn retrieve_pins(&mut self) -> [(u8, Option<Flex<'static>>, u8, Option<Flex<'static>>); 5] {
        [
            self.clock.take_pins(),
            self.data.take_pins(),
            self.atn.take_pins(),
            self.reset.take_pins(),
            self.srq.take_pins(),
        ]
    }

    pub fn retrieve_dios(&mut self) -> [Dio; 8] {
        core::array::from_fn(|ii| Dio {
            pin_num: self.dio[ii].pin_num,
            pin: self.dio[ii].pin.take(),
        })
    }

    // DATA line control
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_data(&mut self) {
        self.data.set();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release_data(&mut self) {
        self.data.release();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get_data(&self) -> bool {
        self.data.get()
    }

    // CLOCK line control
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_clock(&mut self) {
        self.clock.set();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release_clock(&mut self) {
        self.clock.release();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get_clock(&self) -> bool {
        self.clock.get()
    }

    // ATN line control
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_atn(&mut self) {
        self.atn.set();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release_atn(&mut self) {
        self.atn.release();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get_atn(&self) -> bool {
        self.atn.get()
    }

    // RESET line control
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_reset(&mut self) {
        self.reset.set();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release_reset(&mut self) {
        self.reset.release();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get_reset(&self) -> bool {
        self.reset.get()
    }

    // SRQ line control (if available)
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_srq(&mut self) {
        self.srq.set();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn release_srq(&mut self) {
        self.srq.release();
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn get_srq(&self) -> bool {
        self.srq.get()
    }

    /// Set multiple lines at once based on a bit mask
    #[allow(clippy::inline_always)]
    #[inline(always)]
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
    #[allow(clippy::inline_always)]
    #[inline(always)]
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

    /// Poll all pins - returns a bit mask of _inactive_, i.e. high input
    /// lines.  This mimics the `iec_poll_pins` function in the original code.
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
    // TODO may need to be converted to array implementation for speed
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn iec_to_hw(iec: u8) -> u8 {
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
        self.set_lines(Self::iec_to_hw(iec));
    }

    /// Convert from IEC logical representation to physical, then release lines
    pub fn iec_release(&mut self, iec: u8) {
        self.release_lines(Self::iec_to_hw(iec));
    }

    /// Convert from IEC logical representation to physical, then perform set/release
    #[allow(dead_code)]
    pub fn setrelease(&mut self, set: u8, release: u8) {
        self.iec_set(set);
        self.iec_release(release);
    }
}

// The IEC protocol driver implementation
pub struct IecDriver {
    pub bus: IecBus,
    eoi: bool,
    suppress_nib_command: bool,
    pub saved_nib_bytes: [u8; 4],
    pub current_nib_write: usize,
    pub cmd_idx: usize,
    watchdog: &'static WatchdogType,
}

impl ProtocolDriver for IecDriver {
    /// Timing is not critical here, so we use yield.
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        debug!("reset");
        self.bus.release_lines(IO_DATA | IO_ATN | IO_CLK | IO_SRQ);

        // Reset EOI state
        self.eoi = false;

        debug!("Reset: set_reset()");
        // Hold reset line active
        self.bus.set_reset();
        debug!("Reset: set_reset() done");
        yield_ms!(100);
        debug!("Reset: release_reset()");
        self.bus.release_reset();

        debug!("Reset: wait_for_free_bus()");
        let result = self.wait_for_free_bus(forever).await;
        debug!("Reset: done");

        result
    }

    /// Read data from the IEC bus
    async fn raw_read(
        &mut self,
        len: u16,
        protocol: ProtocolType,
    ) -> Result<u16, (DriverError, u16)> {
        info!("Raw Read: Protocol: {}, {} bytes requested", protocol, len);

        // Call the main read() function, which handles all supported
        // protocols.
        self.read(len, protocol).await
    }

    /// Send data to the drive
    async fn raw_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        trace!(
            "Raw Write: Protocol: {}, {}, {} bytes requested",
            protocol, flags, len
        );

        // Call the main read() function, which handles all supported
        // protocols.
        self.write(len, protocol, flags).await
    }

    // As this is wait forever, timing with the loop isn't critical, so we use
    // yield.
    async fn wait(
        &mut self,
        line: u8,
        state: u8,
        timeout: Option<Duration>,
    ) -> Result<(), DriverError> {
        let hw_mask = Self::iec2hw(line);
        let hw_state = if state != 0 { hw_mask } else { 0 };

        let timeout = timeout.unwrap_or(FOREVER_TIMEOUT);
        self.wait_timeout_yield(hw_mask, hw_state, timeout, true, true)
            .await
    }

    // This is an externally exposed function, which returns IEC_* lines, not
    // IO_* lines (the latter being used internally)
    fn poll(&mut self) -> u8 {
        let iec_state = self.bus.poll_pins();
        let mut rv = 0;

        if (iec_state & IO_DATA) == 0 {
            rv |= IEC_DATA;
        }
        if (iec_state & IO_CLK) == 0 {
            rv |= IEC_CLOCK;
        }
        if (iec_state & IO_ATN) == 0 {
            rv |= IEC_ATN;
        }
        if (iec_state & IO_SRQ) == 0 {
            rv |= IEC_SRQ;
        }

        rv
    }

    // This is an externally exposed function, which takes and handles IEC_*
    // lines, not IO_* lines (the latter being used internally)
    fn setrelease(&mut self, set: u8, release: u8) {
        self.bus.setrelease(set, release);
    }

    /// Get the End Of Indication (EOI) status
    fn get_eoi(&self) -> bool {
        self.eoi
    }

    /// Clear the End of Indiciation (EOI) status
    fn clear_eoi(&mut self) {
        self.eoi = false;
    }

    // Writes a byte to the parallel port.
    // LSB is dio[0], and the MSB is dio[7]
    fn write_pp_byte(&mut self, val: u8) {
        // Iterate through all 8 bits
        for ii in 0..8 {
            // Set pin high or low based on the corresponding bit in val
            self.bus.dio[ii].set_output((val >> ii) & 1 == 1);
        }
    }

    // Reads a byte from the parallel port
    // LSB is dio[0] and MSB is dio[7]
    fn read_pp_byte(&mut self) -> u8 {
        let mut val = 0;

        // Iterate through all 8 bits
        for ii in 0..8 {
            // Read the bit and set the corresponding bit in val
            if self.bus.dio[ii].read_pin() {
                val |= (u8::from(self.bus.dio[ii].read_pin())) << ii;
            }
        }

        val
    }
}

// Various other functions
impl IecDriver {
    pub fn new(bus: IecBus, watchdog: &'static WatchdogType) -> Self {
        debug!("Reset line {:?}", bus.data);
        debug!("ATN line {:?}", bus.atn);
        debug!("Clock line {:?}", bus.clock);
        debug!("Data line {:?}", bus.data);
        debug!("Srq line {:?}", bus.srq);

        Self {
            bus,
            eoi: false,
            suppress_nib_command: false,
            saved_nib_bytes: [0; 4],
            current_nib_write: 0,
            cmd_idx: 0,
            watchdog,
        }
    }

    /// Get `supress_nib_command` status
    pub fn get_suppress_nib_command(&self) -> bool {
        self.suppress_nib_command
    }

    /// Set `supress_nib_command` status
    pub fn set_suppress_nib_command(&mut self, suppress: bool) {
        self.suppress_nib_command = suppress;
    }

    /// Set the End of Indiciation (EOI) status
    pub fn set_eoi(&mut self) {
        self.eoi = true;
    }

    /// Combined set and release operation
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_release(&mut self, set: u8, release: u8) {
        self.bus.set_lines(set);
        self.bus.release_lines(release);
    }

    pub fn retrieve_pins(&mut self) -> [(u8, Option<Flex<'static>>, u8, Option<Flex<'static>>); 5] {
        self.bus.retrieve_pins()
    }

    pub fn retrieve_dios(&mut self) -> [Dio; 8] {
        self.bus.retrieve_dios()
    }

    pub async fn feed_watchdog(&self) {
        self.watchdog.feed(&TaskId::DriveOperation).await;
    }

    pub async fn feed_watchdog_protocol_handler(&self) {
        self.watchdog.feed(&TaskId::ProtocolHandler).await;
    }

    /// Wait for listener to release DATA line
    ///
    /// Timing is not critical here so use yield.
    pub async fn wait_for_listener(&mut self) -> bool {
        // Release CLK to indicate we're ready to send
        self.bus.release_clock();

        // On Piers's 1541 it took around 270us for the listener to release
        // data

        // Wait for device to release DATA
        while self.bus.get_data() {
            if self.check_abort() {
                info!("Aborted waiting for listener");
                return false;
            }
            yield_for!(LISTENER_WAIT_INTERVAL);
            self.feed_watchdog().await;
        }

        true
    }

    /// Wait up to 2ms for lines to reach specified state.
    ///
    /// `poll_pins()` returns the mask if the pin is high (inactive).
    ///
    /// If state is the mask being tested for (i.e. `IO_DATA`, `IO_DATA`), then
    /// we wait until the line is low/active.
    ///
    /// If the state is 0 (i.e. `IO_DATA`, 0), then we wait until the line is
    /// high/inactive.
    ///
    /// If mask is two lines (i.e `IO_ATN` | `IO_RESET`) and state is 0, then we
    /// wait until at least one line is high/inactive.  If both lines remain
    /// low/active in this case, we return false.
    ///
    /// We return true if the line got to the state and false otherwise.
    ///
    /// We wait for up to 2ms.
    ///
    /// Timing is considered critical here to wait for around 2ms.  Hence we
    /// use `block_us`.
    pub fn wait_timeout_2ms(&mut self, mask: u8, state: u8) -> bool {
        self.wait_timeout_block(mask, state, Duration::from_millis(2))
    }

    pub fn wait_timeout_block(&mut self, mask: u8, state: u8, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            // Hard loop, as we want to block anyway
            if (self.bus.poll_pins() & mask) != state {
                break;
            }
        }
        (self.bus.poll_pins() & mask) != state
    }

    /// Waits for a defined time, feeding the watchdog and checking for abort
    /// until the lines are _not_ in the specified state.  Returns an error if
    /// it timed out.
    pub async fn wait_timeout_yield(
        &mut self,
        mask: u8,
        state: u8,
        timeout: Duration,
        feed: bool,
        called_from_wait: bool,
    ) -> Result<(), DriverError> {
        let deadline = Instant::now() + timeout;
        loop {
            // Feed the watchdog first, just in case we don't go around this
            // loop again.
            if feed {
                if called_from_wait {
                    // We were called from wait(), which is called from the
                    // ProtocolHandler context, so feed the watchdog
                    // appropriately.
                    self.feed_watchdog_protocol_handler().await;
                } else {
                    // We were called from the DriveOperation context.
                    self.feed_watchdog().await;
                }
            }

            // Next, check if we should abort
            if self.check_abort() {
                break Err(DriverError::Abort);
            }

            // Thirdy check if the mask is in the opposite to the passed in
            // state
            if (self.bus.poll_pins() & mask) != state {
                break Ok(());
            }

            // Check if we hit the timeout
            if Instant::now() >= deadline {
                break Err(DriverError::Timeout);
            }

            // Yield, briefly.
            yield_us!(PROTOCOL_YIELD_TIMER_US);
        }
    }

    async fn check_bus_until_free(&mut self) -> Result<(), DriverError> {
        loop {
            // Check if bus is free
            if self.check_if_bus_free().await {
                return Ok(());
            }

            // Check if we should cancel
            if self.check_abort() {
                info!("Aborted waiting for free bus");
                return Err(DriverError::Abort);
            }

            // Wait so we don't tight loop
            yield_for!(BUS_FREE_CHECK_YIELD);

            // Feed the bus
            self.feed_watchdog().await;
        }
    }

    /// Wait for the bus to be free
    ///
    /// Timing is not criticial here, so we use yield.
    async fn wait_for_free_bus(&mut self, forever: bool) -> Result<(), DriverError> {
        // Figure out the time to wait
        let timeout = if forever {
            FOREVER_TIMEOUT
        } else {
            BUS_FREE_TIMEOUT
        };

        // Wait for the the bus to be free with a timeout
        match with_timeout(timeout, self.check_bus_until_free()).await {
            Ok(result) => result,
            Err(_timeout_error) => {
                debug!("Timed out waiting for the bus to be free (expected if no drive");
                Err(DriverError::Timeout)
            }
        }
    }

    /// Check if the bus is free
    /// We aim for every exit path to take 200us.
    /// Timing is important, to achieve this 200us, so we use `delay_block`.
    async fn check_if_bus_free(&mut self) -> bool {
        // Release all lines and wait for drive reaction time
        self.bus.release_lines(IO_ATN | IO_CLK | IO_DATA | IO_RESET);
        block_us!(50);

        // If DATA is held, drive is not yet ready
        if self.bus.get_data() {
            block_us!(150);
            return false;
        }

        // Ensure DATA is stable
        yield_us!(50);
        if self.bus.get_data() {
            block_us!(100);
            return false;
        }

        // Assert ATN and wait for drive reaction
        self.bus.set_lines(IO_ATN);
        block_us!(100);

        // If DATA is still unset, no drive answered
        if !self.bus.get_data() {
            self.bus.release_atn();
            return false;
        }

        // Test releasing ATN
        self.bus.release_atn();
        block_us!(100);

        // Check if drive released DATA
        !self.bus.get_data()
    }

    /// Convert logical IEC lines to hardware-specific representation
    fn iec2hw(iec: u8) -> u8 {
        // This would implement the conversion based on the C code's lookup table
        // Simplified implementation for demonstration
        let mut result = 0;
        if iec & IEC_DATA != 0 {
            result |= IO_DATA;
        }
        if iec & IEC_CLOCK != 0 {
            result |= IO_CLK;
        }
        if iec & IEC_ATN != 0 {
            result |= IO_ATN;
        }
        if iec & IEC_SRQ != 0 {
            result |= IO_SRQ;
        }
        result
    }

    // Send data using the USB data transfer object. This queues it up for
    // ProtocolHandler to send it.
    pub async fn send_data_to_host(buffer: &[u8], buf_count: usize) -> Result<(), DriverError> {
        // Send the data to the host
        let mut sent_bytes = 0;
        loop {
            let result = UsbDataTransfer::lock_try_add_bytes(&buffer[..buf_count]).await;

            if let Ok(size) = result {
                sent_bytes += size;
            }

            if sent_bytes >= buf_count {
                break;
            }

            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }

        Ok(())
    }

    pub async fn loop_and_yield_while<F>(&self, mut condition: F) -> Result<(), DriverError>
    where
        F: FnMut() -> bool,
    {
        loop {
            if !condition() {
                break Ok(());
            }

            self.feed_watchdog().await;

            Timer::after(PROTOCOL_YIELD_TIMER).await;

            if self.check_abort() {
                info!("Aborted in loop_and_yield");
                break Err(DriverError::Abort);
            }
        }
    }
}
