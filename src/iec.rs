//! This file implements the Commodore IEC protocol driver.  Based on
//! the xum1541 source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::gpio::{Flex, Pull};
use embassy_time::{with_timeout, Duration, Instant, Timer};

use crate::constants::{
    MAX_EP_PACKET_SIZE_USIZE, PROTOCOL_YIELD_TIMER, PROTOCOL_YIELD_TIMER_MS,
    USB_DATA_TRANSFER_WAIT_TIMER,
};
use crate::driver::{DriverError, ProtocolDriver};
use crate::protocol::{Dio, ProtocolFlags, ProtocolType};
use crate::time::iec::{
    BUS_FREE_CHECK_YIELD, BUS_FREE_TIMEOUT, FOREVER_TIMEOUT, IEC_T_BB, IEC_T_NE, IEC_T_R, IEC_T_S,
    IEC_T_V, LISTENER_WAIT_INTERVAL, READ_CLK_TIMEOUT, WRITE_TALK_CLK_TIMEOUT,
};
use crate::time::{block_ns, block_us, iec_delay, yield_for, yield_ms, yield_us};
use crate::transfer::UsbDataTransfer;
use crate::watchdog::{feed_watchdog, TaskId};

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

// An object representing the physical IEC bus.  Each Line is a pair of
// pins, one input and one output.  Pin assignments are in `gpio.rs`.
pub struct IecBus {
    clock: Line,
    data: Line,
    atn: Line,
    reset: Line,
    srq: Line,
    dio: [Dio; 8],
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

        // Check protocol type
        match protocol {
            // Standard Commodore IEC read
            ProtocolType::Cbm => self.cbm_read(len).await,

            // Non Commodore IEC read
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::PP | ProtocolType::P2 => {
                self.non_cbm_read(len, protocol).await
            }

            // TODO NibCommand, NibSrq, NibSrqCommand, Tap and TapConfig
            _ => {
                error!("Raw read: Unsupported protocol type");
                Err((DriverError::Unsupported, 0))
            }
        }
    }

    /// Send data to the drive
    ///
    /// Timing is critical in this function, so we use block_us.
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
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        trace!(
            "Raw Write: Protocol: {}, {}, {} bytes requested",
            protocol,
            flags,
            len
        );

        let result = match protocol {
            // Standard Commodore IEC write
            ProtocolType::Cbm => self.cbm_write(len, protocol, flags).await,

            // Non Commodore IEC write
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::PP | ProtocolType::P2 => {
                self.non_cbm_write(len, protocol, flags).await
            }

            // TODO NibCommand, NibSrq, NibSrqCommand, Tap and TapConfig
            _ => {
                error!("Raw write: Unsupported protocol type");
                Err((DriverError::Unsupported, 0))
            }
        };

        debug!("Raw write completed with result {:?}", result);
        result
    }

    // As this is wait forever, timing with the loop isn't critical, so we use
    // yield.
    async fn wait(
        &mut self,
        line: u8,
        state: u8,
        timeout: Option<Duration>,
    ) -> Result<(), DriverError> {
        let hw_mask = self.iec2hw(line);
        let hw_state = if state != 0 { hw_mask } else { 0 };

        let timeout = timeout.unwrap_or(FOREVER_TIMEOUT);
        self.wait_timeout_yield(hw_mask, hw_state, timeout, true)
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
                val |= (self.bus.dio[ii].read_pin() as u8) << ii;
            }
        }

        val
    }
}

// Implementations of write for the various supported protocols
impl IecDriver {
    /// Implements standard Commodore IEC write support
    async fn cbm_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        assert!(protocol == ProtocolType::Cbm);

        let atn = flags.is_atn();
        let talk = flags.is_talk();

        trace!(
            "CBM Write: {} bytes, ATN: {}, ATN: {}",
            len, atn, talk
        );

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
            debug!("Raw Write: No devices present on the bus");
            debug!("Poll pins returns 0x{:02x}", self.bus.poll_pins());
            // TO DO - async read handling
            return Err((DriverError::NoDevices, 0));
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
            debug!("Raw Write: No devices detected");
            debug!("Poll pins returns 0x{:02x}", self.bus.poll_pins() & IO_DATA);
            self.bus.release_lines(IO_CLK | IO_ATN);
            return Err((DriverError::NoDevices, 0));
        }

        // Wait for drive to be ready for us to release CLK.  The tranfer
        // starts to be unreliable below 10 us.
        block_us!(IEC_T_NE);

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
                debug!("Raw write: Device not present");
                break Err(DriverError::NoDevice);
            }

            debug!("Data line low");

            // Release CLK and wait for listener to release data
            if !self.wait_for_listener().await {
                debug!("Raw write: No listener");
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
            if self.send_byte(byte).await {
                count += 1;
                block_us!(IEC_T_BB);
            } else {
                debug!("Raw write: io err");
                // TODO - async read handling
                break Err(DriverError::Io);
            }

            trace!("Written byte #{} to drive", count);

            feed_watchdog(TaskId::ProtocolHandler);

            // Break if we've sent all the bytes
            if count >= len {
                break Ok(());
            }
        };

        debug!("Finished writing data: {}", result);

        match result {
            Ok(_) => {
                // Talk-ATN turn around if requested
                if talk {
                    debug!("Put drive into Talk mode");

                    // Hold DATA and release ATN
                    self.set_release(IO_DATA, IO_ATN).await;
                    // IEC_T_TK was defined to be -1 in the original C code
                    // which presumably meant to make the delay sub
                    // micro-second.
                    block_ns!(500);

                    // Release CLK and wait for device to grab it
                    self.bus.release_clock();
                    iec_delay!();

                    // Wait for device to pull CLOCK low before returning.  As
                    // self.wait() is an external API, it takes IEC_* lines
                    // not IO_lines*
                    self.wait(IEC_CLOCK, 1, Some(WRITE_TALK_CLK_TIMEOUT))
                        .await
                        .map(|_| count)
                        .map_err(|e| (e, count))
                } else {
                    self.bus.release_atn();
                    Ok(count)
                }
            }
            Err(e) => {
                block_us!(IEC_T_R);
                self.bus.iec_release(IO_CLK | IO_ATN);
                Err((e, count))
            }
        }
    }
}

// Implementations of read for the various supported protocols
impl IecDriver {
    /// Implements standard Commodore IEC read support
    ///
    /// Timing is critical in this function, so we use block_us.
    async fn cbm_read(&mut self, len: u16) -> Result<u16, (DriverError, u16)> {
        let mut buffer = [0u8; MAX_EP_PACKET_SIZE_USIZE];

        let mut count: u16 = 0;
        let mut buf_count: usize = 0;
        self.eoi = false;

        trace!("CBM Read: Requested {} bytes", len);

        // Check we can send bytes before we get started, so we don't have to
        // pause in the middle of a transfer.  We could just wait for one byte
        // of space, but we'll wait for enough space for an entire USB packet
        // as the buffer should be twice that size.
        UsbDataTransfer::lock_wait_space_available(MAX_EP_PACKET_SIZE_USIZE).await;

        // Main read loop
        let result = loop {
            trace!(
                "raw_read Loop start: count {}, buf_count {}",
                count,
                buf_count
            );

            // Wait for clock to be released, with 1s timeout.  Timing isn't
            // particularly critical here, so we can yield.
            if let Err(e) = self
                .wait_timeout_yield(IO_CLK, 0, READ_CLK_TIMEOUT, true)
                .await
            {
                break Err((e, count));
            }

            // Break if we've already seen EOI
            if self.eoi {
                debug!("Raw read: EOI detected at {} bytes", count);
                break Ok(count);
            }

            // Release DATA line to signal we're ready for data
            self.bus.release_data();

            // Wait up to 400us for CLK to be pulled by the drive
            let clock = self.wait_timeout_block(IO_CLK, IO_CLK, Duration::from_micros(400));

            // Check for EOI signalling from talker
            if !clock {
                debug!("Clock high - so EOI signalled");
                self.eoi = true;
                self.bus.set_data();
                block_us!(70);
                self.bus.release_data();
            }

            // Read the byte
            match self.receive_byte().await {
                Ok(byte) => {
                    // Acknowledge byte received by pulling DATA
                    self.bus.set_data();
                    trace!("Read byte #{}", count);

                    // Store the byte in our buffer
                    buffer[buf_count] = byte;
                    buf_count += 1;
                    count += 1;

                    block_us!(50);
                }
                Err(e) => {
                    info!("Raw read: error receiving byte");
                    break Err((e, count));
                }
            }

            // Send the data if we've filled up a USB packet, or we're
            // finished reading.
            if buf_count >= MAX_EP_PACKET_SIZE_USIZE || count >= len {
                match Self::send_data_to_host(&buffer, buf_count).await {
                    Ok(_) => {}
                    Err(e) => break Err((e, count)),
                }
                buf_count = 0;
            }

            feed_watchdog(TaskId::ProtocolHandler);

            // Check if we're finished
            if count >= len {
                info!("Raw read: received {} bytes and forwarded to host", count);
                break Ok(count);
            }
        };

        if buf_count > 0 {
            // Send any remaining data.  There should be fewer than 64
            // bytes of data to send, as we should have sent the data
            // immediately after hitting 64 bytes.
            debug!("Hit error reading, sending outstanding {} bytes", buf_count);
            assert!(buf_count < MAX_EP_PACKET_SIZE_USIZE);
            Self::send_data_to_host(&buffer, buf_count)
                .await
                .map_err(|e| (e, count))?;
        }

        result
    }

    async fn non_cbm_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        _flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        // Wait until there's some bytes to send before we start so we don't
        // have to pause in the middle of a transfer.
        UsbDataTransfer::lock_wait_outstanding().await;
        feed_watchdog(TaskId::ProtocolHandler);

        let num_bytes = match protocol {
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::P2 => 1,
            ProtocolType::PP => 2,
            _ => unreachable!("Non-CBM write: Unsupported protocol type"),
        };

        let mut count = 0;

        while count < len as usize {
            // Get the byte(s) to write
            let mut buf = [0u8; 2];
            for item in buf.iter_mut().take(num_bytes) {
                *item = UsbDataTransfer::lock_get_next_byte().await;
            }

            // Call the appropriate write function

            let result = match protocol {
                ProtocolType::S1 => self.write_s1(buf[0]).await,
                ProtocolType::S2 => self.write_s2(buf[0]).await,
                ProtocolType::PP => self.write_pp(buf[0], buf[1]).await,
                ProtocolType::P2 => self.write_p2(buf[0]).await,
                _ => unreachable!("Non-CBM write: Unsupported protocol type"),
            };

            match result {
                Ok(size) => count += size,
                Err(e) => {
                    return Err((e, count as u16));
                }
            }
        }

        Ok(count as u16)
    }

    // To write a byte using S1:
    // - Send the byte bit by bit, starting from the MSB
    // - For each bit, set DATA to represent the bit (1 is pulled low, 0 high)
    // - Wait for DATA line to stabilize
    // - Release CLK
    // - Wait for it to settle
    // - Wait for CLK to be pulled low
    // - Send the bit a second time, using DATA in the opposite direction
    // - Wait for drive to release CLK
    // - Set CLK and release DATA
    // - Pause briefly to allow lines to settle
    // - Wait for device to release DATA
    // - Move to the next most significant bit
    async fn write_s1(&mut self, byte: u8) -> Result<usize, DriverError> {
        let mut byte = byte;

        // Process the byte bit by bit, starting from MSB
        for _ in 0..8 {
            // Set DATA to represent the bit - 1 is pulled low, 0 is high.
            if byte & 0x80 == 0 {
                self.bus.release_data();
            } else {
                self.bus.set_data();
            }

            // Wait for DATA line to stabilize
            iec_delay!();

            // Release CLK
            self.bus.release_clock();

            // Wait for it to settle
            iec_delay!();

            // Wait CLK to be pulled low
            self.loop_and_yield_while(|| !self.bus.get_clock()).await?;

            // Send bit a second time - using DATA in the opposite direction
            if (byte & 0x80) == 0 {
                self.bus.set_data();
            } else {
                self.bus.release_data();
            }

            // Wait for drive to release CLK
            self.loop_and_yield_while(|| self.bus.get_clock()).await?;

            // Set CLK and release DATA
            self.set_release(IO_CLK, IO_DATA).await;

            // Pause briefly to allow lines to settle
            iec_delay!();

            // Wait for device to acknowledge by setting DATA high
            self.loop_and_yield_while(|| !self.bus.get_data()).await?;

            // Shift to next most significant bit
            byte <<= 1;
        }

        Ok(1)
    }

    // To write a byte using S2:
    // - Send the byte 2 bits for each pass through the loop, starting from
    //   the MSB
    // - For the first bit of the pair, set DATA to represent the bit (1 is
    //   pulled low, 0 high)
    // - Wait for DATA line to stabilize
    // - Release ATN
    // - Wait for CLK to be released
    // - Send the next bit, using DATA in the opposite direction
    // - Wait for CLK to be set
    // - Set ATN
    // - Wait for CLK to be pulled low
    async fn write_s2(&mut self, byte: u8) -> Result<usize, DriverError> {
        let mut byte = byte;

        // Process 4 iterations of 2 bits each
        for _ in 0..4 {
            // Send the first bit, with DATA low for 1, high for 0
            if (byte & 1) == 0 {
                self.bus.release_data();
            } else {
                self.bus.set_data();
            }

            // Delay briefly to allow line to settle
            iec_delay!();

            // Move to the next bit
            byte >>= 1;

            // Release ATN
            self.bus.release_atn();

            // Wait for CLK to be released
            self.loop_and_yield_while(|| self.bus.get_clock()).await?;

            // Send second bit, again with DATA low for 1 and high for 0
            if (byte & 1) == 0 {
                self.bus.release_data();
            } else {
                self.bus.set_data();
            }

            // Delay briefly to allow line to settle
            iec_delay!();

            // Move to next bit (which will be sent at the start of the loop)
            byte >>= 1;

            // Set ATN
            self.bus.set_atn();

            // Wait for CLK to be set
            self.loop_and_yield_while(|| !self.bus.get_clock()).await?;
        }

        // Release DATA and pause for it to settle
        self.bus.release_data();
        iec_delay!();

        Ok(1)
    }

    // To write a byte using P2:
    // - Send the byte bit by bit, starting from the MSB
    // - Set the parallel port pins for this byte
    // - Pause for 0.5us
    // - Release CLK
    // - Wait for DATA to be released
    // - Set CLK
    // - Wait for DATA to be pulled low
    async fn write_p2(&mut self, byte: u8) -> Result<usize, DriverError> {
        self.write_pp_byte(byte);

        block_ns!(500);

        self.bus.release_clock();

        // Wait for DATA to be released
        self.loop_and_yield_while(|| self.bus.get_data()).await?;

        self.bus.set_clock();

        // Wait for DATA to be pulled low
        self.loop_and_yield_while(|| !self.bus.get_data()).await?;

        Ok(1)
    }

    // PP writes 2 bytes one after the other:
    // - Wait for DATA to be pulled low
    // - Set the parallel port pins for this first byte
    // - Pause for 0.5us
    // - Release CLK
    // - Wait for DATA to be released
    // - Write second byte using the parallel port pins
    // - Pause for 0.5us
    // - Set CLK
    async fn write_pp(&mut self, byte_0: u8, byte_1: u8) -> Result<usize, DriverError> {
        // Wait for DATA to be pulled low
        self.loop_and_yield_while(|| !self.bus.get_data()).await?;

        // Write first byte
        self.write_pp_byte(byte_0);

        block_ns!(500);

        self.bus.release_clock();

        // Wait for DATA to be released
        self.loop_and_yield_while(|| self.bus.get_data()).await?;

        // Write second byte
        self.write_pp_byte(byte_1);

        block_ns!(500);

        self.bus.set_clock();

        Ok(2)
    }

    async fn non_cbm_read(
        &mut self,
        len: u16,
        protocol: ProtocolType,
    ) -> Result<u16, (DriverError, u16)> {
        // Check we can send bytes before we get started, so we don't have to
        // pause in the middle of a transfer.  We could just wait for one byte
        // of space, but we'll wait for enough space for an entire USB packet
        // as the buffer should be twice that size.
        UsbDataTransfer::lock_wait_space_available(MAX_EP_PACKET_SIZE_USIZE).await;

        let num_bytes = match protocol {
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::P2 => 1,
            ProtocolType::PP => 2,
            _ => unreachable!("Non-CBM read: Unsupported protocol type"),
        };

        let mut count = 0;

        while count < len as usize {
            // TODO - need some way of knowing when the transfer is being
            // aborted
            let mut buf = [0u8; 2];

            // Get the byte(s) to read
            let size = match protocol {
                ProtocolType::S1 => self.read_s1(&mut buf).await,
                ProtocolType::S2 => self.read_s2(&mut buf).await,
                ProtocolType::PP => self.read_pp(&mut buf).await,
                ProtocolType::P2 => self.read_p2(&mut buf).await,
                _ => unreachable!("Non-CBM read: Unsupported protocol type"),
            }
            .map_err(|e| (e, count as u16))?;

            // There should be no way for the read functions to return an
            // unexpected number of bytes.
            assert!(size == num_bytes);

            // Send the data (immediately - don't wait for a full packet)
            Self::send_data_to_host(&buf[..size], size)
                .await
                .map_err(|e| (e, count as u16))?;

            count += size;
        }

        Ok(count as u16)
    }

    // S1 involves reading a bit at a time, starting with the MSB, as follows:
    // - Wait for DATA line to be released
    // - Release CLK
    // - Pause briefly to allow it to stabilise
    // - Read CLK - this is our next bit, with MSB first
    // - Pull DATA low to acknowledge the bit
    // - Wait for CLK to change
    // - Release DATA
    // - Pause briefly to allow DATA to stabilize
    // - Wait for DATA to be pulled low
    // - Pull CLK low
    async fn read_s1(&mut self, data: &mut [u8]) -> Result<usize, DriverError> {
        assert!(!data.is_empty());

        let mut byte = 0;

        for _ in 0..8 {
            // Wait for DATA to be released
            self.loop_and_yield_while(|| self.bus.get_data()).await?;

            // Release the clock
            self.bus.release_clock();
            iec_delay!();

            // Get the next bit - we read from MSB to LSB
            let bit = self.bus.get_clock();
            byte = (byte >> 1) | ((bit as u8) << 7);

            // Pull data low
            self.bus.set_data();

            // Wait for CLOCK to change
            self.loop_and_yield_while(|| bit == self.bus.get_clock())
                .await?;

            // Release the data line
            self.bus.release_data();
            iec_delay!();

            // Wait for it to be pulled low
            self.loop_and_yield_while(|| !self.bus.get_data()).await?;

            // Pull CLOCK low
            self.bus.set_clock();
        }

        data[0] = byte;
        Ok(1)
    }

    // S2 involves reading a bit every time the CLOCK line changes, with the
    // with first byte coming when the CLOCK line is pulled low, and hence the
    // second with it high.
    //
    // After we detect the CLOCK changing, we pause to make sure DATA has time
    // to stabilise.
    //
    // We flip ATN once we've read the bit.
    async fn read_s2(&mut self, data: &mut [u8]) -> Result<usize, DriverError> {
        assert!(!data.is_empty());

        let mut byte = 0;

        for _ in 0..4 {
            // Wait for CLK to be released
            self.loop_and_yield_while(|| self.bus.get_clock()).await?;

            // Brief pause
            iec_delay!();

            // Read the bit from DATA
            byte = (byte >> 1) | ((self.bus.get_data() as u8) << 7);

            // Release ATN now we've read the bit
            self.bus.release_atn();

            // Wait for CLOCK to be pulled low
            self.loop_and_yield_while(|| !self.bus.get_clock()).await?;

            // Brief pause
            iec_delay!();

            // Read the bit
            byte = (byte >> 1) | ((self.bus.get_data() as u8) << 7);

            // Set ATN now we've read the bit
            self.bus.set_atn();
        }

        data[0] = byte;
        Ok(1)
    }

    // P2 involves reading a single byte from the paralle port as follows:
    // - Release CLK
    // - Wait for DATA to be released
    // - Read the byte
    // - Set CLK
    // - Wait for DATA to be pulled low
    async fn read_p2(&mut self, data: &mut [u8]) -> Result<usize, DriverError> {
        self.bus.release_clock();

        self.loop_and_yield_while(|| self.bus.get_data()).await?;

        data[0] = self.read_pp_byte();

        self.bus.set_clock();

        self.loop_and_yield_while(|| !self.bus.get_data()).await?;

        Ok(1)
    }

    // PP involves reading 2 bytes:
    // - Wait for DATA to be pulled low
    // - Read the byte from the parallel bus
    // - Relase CLK
    // - Wait for DATA to be released
    // - Read the second byte from the parallel bus
    // - Set CLK
    async fn read_pp(&mut self, data: &mut [u8]) -> Result<usize, DriverError> {
        assert!(data.len() >= 2);

        // Wait for DATA to be pulled low
        self.loop_and_yield_while(|| !self.bus.get_data()).await?;

        // Read the first byte
        data[0] = self.read_pp_byte();

        // Release CLK
        self.bus.release_clock();

        // Wait for DATA to be released
        self.loop_and_yield_while(|| self.bus.get_data()).await?;

        // Read the second byte
        data[1] = self.read_pp_byte();

        // Set CLK
        self.bus.set_clock();

        Ok(2)
    }
}

// Other private functions
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

    pub fn retrieve_dios(&mut self) -> [Dio; 8] {
        self.bus.retrieve_dios()
    }

    /// Wait for listener to release DATA line
    ///
    /// Timing is not critical here so use yield.
    async fn wait_for_listener(&mut self) -> bool {
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
            feed_watchdog(TaskId::ProtocolHandler);
        }

        true
    }

    /// Send a byte, one bit at a time via the IEC protocol
    ///
    /// Timing is critical here, so we use block_us.
    async fn send_byte(&mut self, b: u8) -> bool {
        let mut data = b;

        for _ in 0..8 {
            // Wait for setup time
            block_us!(IEC_T_S + 55);

            // Set the bit value on the DATA line
            if (data & 1) == 0 {
                self.bus.set_lines(IO_DATA);
                iec_delay!();
            }

            // Trigger clock edge and hold valid for specified time
            self.bus.release_clock();
            block_us!(IEC_T_V);

            // Prepare for next bit
            self.set_release(IO_CLK, IO_DATA).await;
            data >>= 1;
        }

        // Wait for acknowledgement
        let ack = self.wait_timeout_2ms(IO_DATA, IO_DATA);
        if !ack {
            debug!("send_byte: no ack");
        }
        ack
    }

    /// Receive a single byte from the IEC bus
    async fn receive_byte(&mut self) -> Result<u8, DriverError> {
        // Wait for CLK to be asserted (pulled low)
        if !self.wait_timeout_2ms(IO_CLK, IO_CLK) {
            debug!("Receive byte: no clock");
            return Err(DriverError::Timeout);
        }

        let mut byte: u8 = 0;

        // Read 8 bits
        for _bit in 0..8 {
            // Wait for CLK to be released (high)
            if !self.wait_timeout_2ms(IO_CLK, 0) {
                debug!("Receive byte: clock not released");
                return Err(DriverError::Timeout);
            }

            // Read the bit
            byte >>= 1;
            if !self.bus.get_data() {
                byte |= 0x80;
            }

            // Wait for CLK to be asserted again
            if !self.wait_timeout_2ms(IO_CLK, IO_CLK) {
                debug!("Receive byte: no clock in loop");
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
    /// use block_us.
    fn wait_timeout_2ms(&mut self, mask: u8, state: u8) -> bool {
        self.wait_timeout_block(mask, state, Duration::from_millis(2))
    }

    fn wait_timeout_block(&mut self, mask: u8, state: u8, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            // Hard loop, as we want to block anyway
            if (self.bus.poll_pins() & mask) != state {
                break;
            }
        }
        (self.bus.poll_pins() & mask) != state
    }

    async fn wait_timeout_yield(
        &mut self,
        mask: u8,
        state: u8,
        timeout: Duration,
        feed: bool,
    ) -> Result<(), DriverError> {
        let deadline = Instant::now() + timeout;
        loop {
            if (self.bus.poll_pins() & mask) != state {
                break Ok(());
            }

            if Instant::now() >= deadline {
                break Err(DriverError::Timeout);
            }

            if feed {
                feed_watchdog(TaskId::ProtocolHandler);
            }

            if self.check_abort() {
                break Err(DriverError::Abort);
            }

            yield_us!(PROTOCOL_YIELD_TIMER_MS)
        }
    }

    /// Wait for the bus to be free
    ///
    /// Timing is not criticial here, so we use yield.
    async fn wait_for_free_bus(&mut self, forever: bool) -> Result<(), DriverError> {
        async fn check_bus_until_free(device: &mut IecDriver) -> Result<(), DriverError> {
            loop {
                // Check if bus is free
                if device.check_if_bus_free().await {
                    return Ok(());
                }

                // Check if we should cancel
                if device.check_abort() {
                    info!("Aborted waiting for free bus");
                    return Err(DriverError::Abort);
                }

                // Wait so we don't tight loop
                yield_for!(BUS_FREE_CHECK_YIELD);

                // Feed the bus
                feed_watchdog(TaskId::ProtocolHandler);
            }
        }

        // Figure out the time to wait
        let timeout = if forever {
            FOREVER_TIMEOUT
        } else {
            BUS_FREE_TIMEOUT
        };

        // Wait for the the bus to be free with a timeout
        match with_timeout(timeout, check_bus_until_free(self)).await {
            Ok(result) => result,
            Err(_timeout_error) => {
                debug!("Timed out waiting for the bus to be free (expected if no drive");
                Err(DriverError::Timeout)
            }
        }
    }

    /// Check if the bus is free
    /// We aim for every exit path to take 200us.
    /// Timing is important, to achieve this 200us, so we use delay_block.
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

    async fn loop_and_yield_while<F>(&self, mut condition: F) -> Result<(), DriverError>
    where
        F: FnMut() -> bool,
    {
        loop {
            if !condition() {
                break Ok(());
            }

            feed_watchdog(TaskId::ProtocolHandler);

            Timer::after(PROTOCOL_YIELD_TIMER).await;

            if self.check_abort() {
                info!("Aborted in loop_and_yield");
                break Err(DriverError::Abort);
            }
        }
    }
}
