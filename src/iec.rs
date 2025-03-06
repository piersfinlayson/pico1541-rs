//! This file implements the Commodore IEC protocol driver.  Based on
//! the xum1541 source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{Flex, Pull};
use embassy_time::{with_timeout, Delay, Duration, Instant, Timer};
use embedded_hal::delay::DelayNs;

use crate::constants::{MAX_EP_PACKET_SIZE_USIZE, USB_DATA_TRANSFER_WAIT_TIMER};
use crate::display::{update_status, DisplayType};
use crate::driver::{DriverError, ProtocolDriver};
use crate::protocol::{ProtocolFlags, ProtocolType};
use crate::transfer::UsbDataTransfer;
use crate::watchdog::{feed_watchdog, TaskId};

//
// IEC protocol timers
//
#[allow(dead_code)]
const IEC_T_AT: u32 = 1000; // Max ATN response required time (us)
                            //      IEC_T_H     inf  // Max listener hold-off time
const IEC_T_NE: u32 = 40; // Typical non-EOI response to RFD time (us)
const IEC_T_S: u32 = 20; // Min talker bit setup time (us, 70 typical)
const IEC_T_V: u32 = 20; // Min data valid time (us, 20 typical)
#[allow(dead_code)]
const IEC_T_F: u32 = 1000; // Max frame handshake time (us, 20 typical)
const IEC_T_R: u32 = 20; // Min frame to release of ATN time (us)
const IEC_T_BB: u32 = 100; // Min time between bytes (us)
#[allow(dead_code)]
const IEC_T_YE: u32 = 200; // Min EOI response time (us, 250 typical)
#[allow(dead_code)]
const IEC_T_EI: u32 = 60; // Min EOI response hold time (us)
#[allow(dead_code)]
const IEC_T_RY: u32 = 60; // Max talker response limit (us, 30 typical)
#[allow(dead_code)]
const IEC_T_PR: u32 = 20; // Min byte acknowledge hold time (us, 30 typical)
                          //const IEC_T_TK: u32 = -1;   // 20/30/100 talk-attention release time (us)
                          //      IEC_T_DC    inf  // Talk-attention acknowledge time, 0 - inf
#[allow(dead_code)]
const IEC_T_DA: u32 = 80; // Min talk-attention ack hold time (us)
#[allow(dead_code)]
const IEC_T_FR: u32 = 60; // Min EOI acknowledge time (us)

// An object representing the physical IEC bus.  Each Line is a pair of
// pins, one input and one output.  Pin assignments are in `gpio.rs`.
pub struct IecBus {
    clock: Line,
    data: Line,
    atn: Line,
    reset: Line,
    srq: Line,
}

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

// Timer contants
const BUS_FREE_CHECK_INTERVAL: Duration = Duration::from_millis(1);
const BUS_FREE_TIMEOUT: Duration = Duration::from_millis(1500);
const LISTENER_WAIT_INTERVAL: Duration = Duration::from_micros(1);
const WAIT_INTERVAL: Duration = Duration::from_micros(1);

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
// delay_block_us!() macro.  Where we can tolerate a longer delay if required,
// we use the delay_yield_*!() macros.
//
// Irrespective of which macro is used, this module must ensure watchdog is
// fed, should it be delaying for a long time.

// Brief delay to allow lines to settle
macro_rules! iec_delay {
    () => {
        Delay.delay_us(2)
    };
}

// Blocking delay for a specified time
macro_rules! delay_block_us {
    ($us:expr) => {
        Delay.delay_us($us)
    };
}

// Yielding delay for a specified time
macro_rules! delay_yield_us {
    ($us:expr) => {
        Timer::after_micros($us).await
    };
}
macro_rules! delay_yield_ms {
    ($ms:expr) => {
        Timer::after_millis($ms).await
    };
}
macro_rules! delay_yield {
    ($dur:expr) => {
        Timer::after($dur).await
    };
}

/// Represents a single bidirectional IEC bus line using separate input/output pins
pub struct Line {
    input_pin_num: u8,
    input_pin: Option<Flex<'static>>,
    output_pin_num: u8,
    output_pin: Option<Flex<'static>>,
}

impl Line {
    /// Create a new Line with the specified input and output pins
    pub fn new(
        input_pin_num: u8,
        input_pin: Flex<'static>,
        output_pin_num: u8,
        output_pin: Flex<'static>,
    ) -> Self {
        // Initialize with input having pull-up and output low (inverted released state)
        // Initialize the input pin as an input with pull-up
        let mut input = input_pin;
        input.set_as_input();
        input.set_pull(Pull::Up);

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
    pub fn set(&mut self) {
        self.output_pin.as_mut().unwrap().set_high();
    }

    /// Release the output line (inactive) - output is inverted pin so low
    pub fn release(&mut self) {
        self.output_pin.as_mut().unwrap().set_low();
    }

    /// Read the current state of the line.  This returns true is the line
    /// is active - as it is not inverted, this means true if low.
    pub fn get(&self) -> bool {
        self.input_pin.as_ref().unwrap().is_low()
    }

    /// Check if line is currently being driven low - inverted pin so high
    #[allow(dead_code)]
    pub fn is_set(&self) -> bool {
        self.output_pin.as_ref().unwrap().is_set_high()
    }
}

impl IecBus {
    /// Create a new IEC bus with the specified pins
    #[allow(clippy::too_many_arguments)]
    pub fn new(clock: Line, data: Line, atn: Line, reset: Line, srq: Line) -> Self {
        let mut bus = Self {
            clock,
            data,
            atn,
            reset,
            srq,
        };

        // Initialize all pins to released state (similar to board_init_iec)
        bus.release_lines(IO_DATA | IO_CLK | IO_ATN | IO_RESET | IO_SRQ);

        // Set initial status - LED on
        update_status(DisplayType::Init);

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
    /// lines.  This mimics the iec_poll_pins function in the original code.
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
    #[inline(always)]
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
    #[allow(dead_code)]
    pub fn setrelease(&mut self, set: u8, release: u8) {
        self.iec_set(set);
        self.iec_release(release);
    }
}

// The IEC protocol driver implementation
pub struct IecDriver {
    bus: IecBus,
    eoi: bool,
}

impl ProtocolDriver for IecDriver {
    /// Timing is not critical here, so we use delay_yield.
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        debug!("reset");
        self.bus.release_lines(IO_DATA | IO_ATN | IO_CLK | IO_SRQ);

        // Reset EOI state
        self.eoi = false;

        debug!("Reset: set_reset()");
        // Hold reset line active
        self.bus.set_reset();
        debug!("Reset: set_reset() done");
        delay_yield_ms!(100);
        debug!("Reset: release_reset()");
        self.bus.release_reset();

        debug!("Reset: wait_for_free_bus()");
        let result = self.wait_for_free_bus(forever).await;
        debug!("Reset: done");

        result
    }

    /// Read data from the IEC bus
    ///
    /// Timing is critical in this function, so we use delay_block_us.
    async fn raw_read(&mut self, len: u16) -> Result<u16, DriverError> {
        info!("Raw Read: {} bytes requested", len);

        let mut buffer = [0u8; MAX_EP_PACKET_SIZE_USIZE];

        let mut count: u16 = 0;
        let mut buf_count: usize = 0;
        self.eoi = false;

        // Check we can send bytes before we get started, so we don't have to
        // pause in the middle of a transfer.  We could just wait for one byte
        // of space, but we'll wait for enough space for an entire USB packet
        // as the buffer should be twice that size.
        UsbDataTransfer::lock_wait_space_available(MAX_EP_PACKET_SIZE_USIZE).await;

        // Main read loop
        let result = loop {
            debug!(
                "raw_read Loop start: count {}, buf_count {}",
                count, buf_count
            );
            // Wait for clock to be released, with 1s timeout.  Timing isn't
            // particularly critical here, so we can yield.
            let timeout = Instant::now() + Duration::from_secs(1);
            let result = loop {
                let clock_released = !self.bus.get_clock();
                match clock_released {
                    true => break Ok(()),
                    false => {
                        if Instant::now() > timeout {
                            error!("Raw read: timeout waiting for clock");
                            break Err(DriverError::Timeout);
                        }
                        delay_yield!(WAIT_INTERVAL);

                        if self.check_abort() {
                            break Err(DriverError::Abort);
                        }
                    }
                }
            };
            if let Err(e) = result {
                // Exit the loop rather than return in case there's some
                // cleaning up later in the function.
                break Err(e);
            }

            // Break if we've already seen EOI
            if self.eoi {
                error!("Raw read: EOI detected");
                break Err(DriverError::Io);
            }

            // Release DATA line to signal we're ready for data
            self.bus.release_data();

            // Wait up to 400us for CLK to be pulled by the drive
            let timeout = Instant::now() + Duration::from_micros(400);
            while !self.bus.get_clock() && Instant::now() < timeout {
                delay_block_us!(2);
            }

            // Check for EOI signalling from talker
            if !self.bus.get_clock() {
                debug!("Clock high - so EOI signalled");
                self.eoi = true;
                self.bus.set_data();
                delay_block_us!(70);
                self.bus.release_data();
            }

            // Read the byte
            match self.receive_byte().await {
                Ok(byte) => {
                    // Acknowledge byte received by pulling DATA
                    self.bus.set_data();

                    // Store the byte in our buffer
                    buffer[buf_count] = byte;
                    buf_count += 1;
                    count += 1;

                    delay_block_us!(50);
                }
                Err(e) => {
                    error!("Raw read: error receiving byte");
                    break Err(e);
                }
            }

            // Send the data if we've filled up a USB packet, or we're
            // finished reading.
            if buf_count >= MAX_EP_PACKET_SIZE_USIZE || count >= len {
                Self::send_data_to_host(&buffer, buf_count).await?;
                buf_count = 0;
            }

            feed_watchdog(TaskId::ProtocolHandler);

            // Check if we're finished
            if count >= len {
                info!("Raw read: received {} bytes and forwarded to host", count);
                break Ok(count);
            }
        };

        if result.as_ref().err().is_some() {
            if buf_count > 0 {
                // Send any remaining data.  There should be fewer than 64
                // bytes of data to send, as we should have sent the data
                // immediately after hitting 64 bytes.
                debug!("Hit error reading, sending outstanding {} bytes", buf_count);
                assert!(buf_count < MAX_EP_PACKET_SIZE_USIZE);
                Self::send_data_to_host(&buffer, buf_count).await?;
            }
            error!("Raw read: error {}", result.as_ref().err().unwrap());
        }

        result
    }

    /// Send data to the drive
    ///
    /// Timing is critical in this function, so we use delay_block_us.
    ///
    /// When the read has finished it is important that the last USB packet
    /// was either not the maximum size allowed, or, if it was, it is followed
    /// by a zero length packet (ZLP).  This informs the host (via its USB
    /// stack), that the bulk transfer has completed.  In our implementation,
    /// this is handled by ProtocolHandler when it received the data we sent
    /// it, using USB_DATA_TRANSFER, once we're done.  And done is signalled
    /// by the task function that calls this one, so again, we don't need to
    /// worry about it.
    async fn raw_write(
        &mut self,
        len: u16,
        _protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, DriverError> {
        let atn = flags.is_atn();
        let talk = flags.is_talk();

        info!("Raw Write: len {} bytes, atn {}, talk {}", len, atn, talk);

        if len == 0 {
            return Ok(len);
        }

        // Wait until there's some bytes to send before we start so we don't
        // have to pause in the middle of a transfer.
        UsbDataTransfer::lock_wait_outstanding().await;
        feed_watchdog(TaskId::ProtocolHandler);

        // Check if any device is present on the bus.  If both lines stay low
        // then this check fails - and corresponds to a device present but
        // powered off.  If we stayed, we'd get stuck in wait_for_listener().
        if !self.wait_timeout_2ms(IO_ATN | IO_RESET, 0) {
            error!("Raw Write: No devices present on the bus");
            debug!("Poll pins returns 0x{:02x}", self.bus.poll_pins());
            // TO DO - async read handling
            return Err(DriverError::NoDevices);
        }

        // Let data go high.
        self.bus.release_data();

        // Either pull CLOCK low, or both ATN and CLOCK.
        match atn {
            true => self.bus.set_lines(IO_CLK | IO_ATN),
            false => self.bus.set_lines(IO_CLK),
        }

        // Short delay to let lines settle
        iec_delay!();

        // Wait for any device to pull data after we set CLK.
        // This should be IEC_T_AT (1ms) but allow a bit longer.
        if !self.wait_timeout_2ms(IO_DATA, IO_DATA) {
            error!("Raw Write: No devices detected");
            debug!("Poll pins returns 0x{:02x}", self.bus.poll_pins() & IO_DATA);
            self.bus.release_lines(IO_CLK | IO_ATN);
            return Err(DriverError::NoDevices);
        }

        // Wait for drive to be ready for us to release CLK.  The tranfer
        // starts to be unreliable below 10 us.
        delay_block_us!(IEC_T_NE);

        // Send bytes as soon as the device is ready
        let mut count = 0;
        let result = loop {
            // Need to signal EOI for the last byte
            let is_last_byte = count == len - 1;

            // Get the next byte to send
            debug!("Lock get next byte");
            let byte = UsbDataTransfer::lock_get_next_byte().await;
            debug!("Lock get next byte done");

            // Be sure DATA line has been pulled by device
            if !self.bus.get_data() {
                error!("Raw write: Device not present");
                break Err(DriverError::NoDevice);
            }

            debug!("Data line low");

            // Release CLK and wait for listener to release data
            if !self.wait_for_listener().await {
                error!("Raw write: No listener");
                break Err(DriverError::Timeout);
            }

            debug!("Listener released DATA");

            // Signal EOI for last byte
            if is_last_byte && !atn {
                self.wait_timeout_2ms(IO_DATA, IO_DATA);
                self.wait_timeout_2ms(IO_DATA, 0);
            }

            // Pull CLOCK low
            self.bus.set_lines(IO_CLK);

            // Send the byte
            debug!("send byte");
            if self.send_byte(byte).await {
                count += 1;
                delay_block_us!(IEC_T_BB);
            } else {
                error!("Raw write: io err");
                // TODO - async read handling
                break Err(DriverError::Io);
            }

            feed_watchdog(TaskId::ProtocolHandler);

            // Break if we've sent all the bytes
            if count >= len {
                break Ok(());
            }
        };

        debug!("Sent data: {}", result);

        match result {
            Ok(_) => {
                // Talk-ATN turn around if requested
                if talk {
                    // Hold DATA and release ATN
                    self.set_release(IO_DATA, IO_ATN).await;
                    // IEC_T_TK was defined to be -1 in the original C code which
                    // presumably meant to make the delay sub micro-second.  We'll try
                    // to oblige.  The embassy-rp documentation indicates that when
                    // time-driver feature is enabled (which it is), the RP2040 TIMER
                    // device is used for embassy-time time at a tick rate of 1MHz.  So,
                    // timers less than 1us are not possible.
                    delay_block_us!(0); // try a similar trick
                                        //delay_block_us!(IEC_T_TK).await;

                    // Release CLK and wait for device to grab it
                    self.bus.release_clock();
                    iec_delay!();

                    // Wait for device to pull CLOCK low before returning.  As
                    // self.wait() is an external API, it takes IEC_* lines
                    // not IO_lines*
                    if self.wait(IEC_CLOCK, 1).await.is_err() {
                        // Abort was signalled.
                        Err(DriverError::Abort)
                    } else {
                        debug!("Raw write: wrote {} bytes - talk", count);
                        Ok(count)
                    }
                } else {
                    self.bus.release_atn();
                    debug!("Raw write: wrote {} bytes", count);
                    Ok(count)
                }
            }
            Err(e) => {
                delay_block_us!(IEC_T_R);
                self.bus.iec_release(IO_CLK | IO_ATN);
                Err(e)
            }
        }
    }

    // As this is wait forever, timing witht he loop isn't critical, so we use
    // delay_yield.
    async fn wait(&mut self, line: u8, state: u8) -> Result<(), DriverError> {
        let hw_mask = self.iec2hw(line);
        let hw_state = if state != 0 { hw_mask } else { 0 };

        // Continuously poll lines until we get the expected state or timeout
        while (self.bus.poll_pins() & hw_mask) == hw_state {
            // Check if we should cancel
            if self.check_abort() {
                return Err(DriverError::Abort);
            }

            delay_yield!(WAIT_INTERVAL);
        }

        Ok(())
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
}

impl IecDriver {
    pub fn new(bus: IecBus) -> Self {
        Self { bus, eoi: false }
    }

    /// Combined set and release operation
    #[inline(always)]
    async fn set_release(&mut self, set: u8, release: u8) {
        self.bus.set_lines(set);
        self.bus.release_lines(release);
    }

    pub fn retrieve_pins(&mut self) -> [(u8, Option<Flex<'static>>, u8, Option<Flex<'static>>); 5] {
        self.bus.retrieve_pins()
    }

    /// Wait for listener to release DATA line
    ///
    /// Timing is not critical here so use delay_yield.
    async fn wait_for_listener(&mut self) -> bool {
        // Release CLK to indicate we're ready to send
        self.bus.release_clock();

        // On Piers's 1541 it took around 270us for the listener to release
        // data

        // Wait for device to release DATA
        while self.bus.get_data() {
            if self.check_abort() {
                return false;
            }
            delay_yield!(LISTENER_WAIT_INTERVAL);
            feed_watchdog(TaskId::ProtocolHandler);
        }

        true
    }

    /// Send a byte, one bit at a time via the IEC protocol
    ///
    /// Timing is critical here, so we use delay_block_us.
    async fn send_byte(&mut self, b: u8) -> bool {
        let mut data = b;

        for _ in 0..8 {
            // Wait for setup time
            delay_block_us!(IEC_T_S + 55);

            // Set the bit value on the DATA line
            if (data & 1) == 0 {
                self.bus.set_lines(IO_DATA);
                iec_delay!();
            }

            // Trigger clock edge and hold valid for specified time
            self.bus.release_clock();
            delay_block_us!(IEC_T_V);

            // Prepare for next bit
            self.set_release(IO_CLK, IO_DATA).await;
            data >>= 1;
        }

        // Wait for acknowledgement
        let ack = self.wait_timeout_2ms(IO_DATA, IO_DATA);
        if !ack {
            error!("send_byte: no ack");
        }
        ack
    }

    /// Receive a single byte from the IEC bus
    async fn receive_byte(&mut self) -> Result<u8, DriverError> {
        // Wait for CLK to be asserted (pulled low)
        if !self.wait_timeout_2ms(IO_CLK, IO_CLK) {
            error!("Receive byte: no clock");
            return Err(DriverError::Timeout);
        }

        let mut byte: u8 = 0;

        // Read 8 bits
        for _bit in 0..8 {
            // Wait for CLK to be released (high)
            if !self.wait_timeout_2ms(IO_CLK, 0) {
                error!("Receive byte: clock not released");
                return Err(DriverError::Timeout);
            }

            // Read the bit
            byte >>= 1;
            if !self.bus.get_data() {
                byte |= 0x80;
            }

            // Wait for CLK to be asserted again
            if !self.wait_timeout_2ms(IO_CLK, IO_CLK) {
                error!("Receive byte: no clock in loop");
                return Err(DriverError::Timeout);
            }
        }

        Ok(byte)
    }

    /// Wait up to 2ms for lines to reach specified state.
    ///
    /// poll_pins() returns the mask if the pin is high (inactive).
    ///
    /// If state is the mask being tested for (i.e. IO_DATA, IO_DATA), then
    /// we wait until the line is low/active.
    ///
    /// If the state is 0 (i.e. IO_DATA, 0), then we wait until the line is
    /// high/inactive.
    ///
    /// If mask is two lines (i.e IO_ATN | IO_RESET) and state is 0, then we
    /// wait until at least one line is high/inactive.  If both lines remain
    /// low/active in this case, we return false.
    ///
    /// We return true if the line got to the state and false otherwise.
    ///
    /// We wait for up to 2ms.
    ///
    /// Timing is considered critical here to wait for around 2ms.  Hence we
    /// use delay_block_us.
    fn wait_timeout_2ms(&mut self, mask: u8, state: u8) -> bool {
        for _ in 0..200 {
            if (self.bus.poll_pins() & mask) != state {
                return true;
            }
            delay_block_us!(10);
        }
        self.bus.poll_pins() & mask != state
    }

    /// Wait for the bus to be free
    ///
    /// Timing is not criticial here, so we use delay_yield.
    async fn wait_for_free_bus(&mut self, forever: bool) -> Result<(), DriverError> {
        async fn check_bus_until_free(device: &mut IecDriver) -> Result<(), DriverError> {
            loop {
                // Check if bus is free
                if device.check_if_bus_free().await {
                    return Ok(());
                }

                // Check if we should cancel
                if device.check_abort() {
                    return Err(DriverError::Abort);
                }

                // Wait so we don't tight loop
                delay_yield!(BUS_FREE_CHECK_INTERVAL);

                // Feed the bus
                feed_watchdog(TaskId::ProtocolHandler);
            }
        }

        if forever {
            // Simply wait for the bus to be free
            check_bus_until_free(self).await
        } else {
            // Wait for the the bus to be free with a timeout
            match with_timeout(BUS_FREE_TIMEOUT, check_bus_until_free(self)).await {
                Ok(result) => result,
                Err(_timeout_error) => {
                    warn!("Timed out waiting for the bus to be free (expected if no drive");
                    Err(DriverError::Timeout)
                }
            }
        }
    }

    /// Check if the bus is free
    /// We aim for every exit path to take 200us.
    /// Timing is important, to achieve this 200us, so we use delay_block.
    async fn check_if_bus_free(&mut self) -> bool {
        // Release all lines and wait for drive reaction time
        self.bus.release_lines(IO_ATN | IO_CLK | IO_DATA | IO_RESET);
        delay_block_us!(50);

        // If DATA is held, drive is not yet ready
        if self.bus.get_data() {
            delay_block_us!(150);
            return false;
        }

        // Ensure DATA is stable
        delay_yield_us!(50);
        if self.bus.get_data() {
            delay_block_us!(100);
            return false;
        }

        // Assert ATN and wait for drive reaction
        self.bus.set_lines(IO_ATN);
        delay_block_us!(100);

        // If DATA is still unset, no drive answered
        if !self.bus.get_data() {
            self.bus.release_atn();
            return false;
        }

        // Test releasing ATN
        self.bus.release_atn();
        delay_block_us!(100);

        // Check if drive released DATA
        !self.bus.get_data()
    }

    // Check if host signaled an abort
    fn check_abort(&self) -> bool {
        // TO DO
        false // Placeholder
    }

    /// Convert logical IEC lines to hardware-specific representation
    fn iec2hw(&self, iec: u8) -> u8 {
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
    async fn send_data_to_host(buffer: &[u8], buf_count: usize) -> Result<(), DriverError> {
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
}
