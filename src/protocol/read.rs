//! This file implements the reading of bytes from the drive, for all
//! supported protocols.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use static_assertions::const_assert;

use super::driver::{DriverError, ProtocolDriver};
use super::iec::{IecDriver, IO_ATN, IO_CLK, IO_DATA, IO_SRQ};
use super::ProtocolType;

use crate::constants::MAX_EP_PACKET_SIZE_USIZE;
use crate::usb::transfer::UsbDataTransfer;
use crate::util::time::iec::{READ_CLK_START_TIMEOUT, READ_CLK_TIMEOUT};
use crate::util::time::{block_ns, block_us, iec_delay, yield_ms};

/// Some protocols may support specific options.  This is used to define and
/// control those options.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ReadOption {
    // No specific option
    None,

    // NIB parallel read mode should not exit early on receipt of special byte
    // (0x55)
    NibParReadNoEarlyExit,

    // NIB parallel read mode should exit early on receipt of special byte
    // (0x55)
    NibParReadEarlyExit,
}

// Parallel NIB protocol has a special case ead character where it can be
// caused to exit early. This special byte is 0x55, and it is signalled with
// the top bit of length.
const NIB_EARLY_EXIT_BIT: u16 = 0x8000;
const NIB_EARLY_EXIT_BYTE: u8 = 0x55;

// Constants covering number of bytes read per read loop iteration
const READ_BUF_SIZE: usize = 2;
const DEFAULT_READ_ITER_BYTES: usize = 1;
const PP_READ_ITER_BYTES: usize = 2;
const_assert!(READ_BUF_SIZE >= DEFAULT_READ_ITER_BYTES);
const_assert!(READ_BUF_SIZE >= PP_READ_ITER_BYTES);

impl IecDriver {
    /// Main read routine which handles all protocols.
    ///
    /// As well as the standard CBM IEC read protocol, this also supports:
    /// - S1 Faster serial protocol developed for use with 1541 type drives.
    /// - S2 Stock Commodore faster serial than S1, used by 1570, 1571 and
    ///   1581.
    /// - PP Parallel protocol, offers improve rates above serial, and
    ///   requires a parallel port to be installed.
    /// - P2 Parallel protocol 2, faster parallel protocol, again requiring a
    ///   parallel port.
    ///
    /// These are all supported by uploading custom firmware routines to the
    /// drive.  That is outside the scope of the pico1541 - it is performed
    /// by the host software, such as OpenCBM, using the pico1541 as a data
    /// transmission vehicle.
    ///
    /// Implements the following steps:
    /// - Makes sure there's space to write bytes via USB
    /// - Gets any options for this operation
    /// - Calls the protocol specific startup routine to prep for reading
    /// - Runs the main read loop, sending data via USB to the host.
    /// - Terminates the read routine
    /// - Returns the number of bytes read, or error.
    ///
    /// Returns
    /// - Ok(u16) - Number of bytes read
    /// - Err(DriverError, usize) - Error and number of bytes read and sent
    ///   via USB, before the error occurred.
    ///
    /// Note that EarlyExit and Eoi are not considered errors - they are
    /// converted before this function returns to Ok(u16).  The fact that
    /// fewer bytes than requested were returned demonstrates to the host that
    /// the read was terminated early.
    pub async fn read(
        &mut self,
        len: u16,
        protocol: ProtocolType,
    ) -> Result<u16, (DriverError, u16)> {
        // Check we can send bytes before we get started, so we don't have to
        // pause in the middle of a transfer.  We could just wait for one byte
        // of space, but we'll wait for enough space for an entire USB packet
        // as the buffer should be twice that size.
        UsbDataTransfer::lock_wait_space_available(MAX_EP_PACKET_SIZE_USIZE).await;
        Self::feed_watchdog();

        // Get any options for this operation.
        let option = Self::get_read_option(len, protocol);

        // Do protocol specific startup routine
        self.startup_read(protocol, option).await;

        // Call the main read loop
        let loop_result = self.read_main_loop(len, protocol, option).await;

        // Protocol specific termination - we terminate whether we hit an error
        // or not.
        self.terminate_read(protocol, option);

        // Feed the watchdog, as belt and braces in case it's been a while.
        Self::feed_watchdog();

        loop_result
    }

    // Main read loop, which receives bytes, and sends them out to the USB
    // host:
    // - Gets number of bytes read in each loop iteration by this protocol
    // - Loops through reading bytes, using the protocol specific read
    //   function, sending them to the host as they're received.  Also exits
    //   early from the loop if told to
    // - Sends any remaining bytes to the host
    // - Returns the number of bytes read
    async fn read_main_loop(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        option: ReadOption,
    ) -> Result<u16, (DriverError, u16)> {
        // Get number of bytes each read loop iteration below.
        let num_bytes = Self::get_read_num_iter_bytes(protocol);

        // Tracking variables for read and USB send operations
        const SIZE: usize = MAX_EP_PACKET_SIZE_USIZE; // Size of the buffer
        let mut count = 0; // Number of bytes read so far
        let mut usb_buf = [0u8; SIZE]; // Buffer for data to be sent
        let mut usb_buf_count = 0; // Count of bytes in the USB buffer
        let mut usb_sent = 0; // Count of bytes sent to USB host

        // Main read loop
        let result = loop {
            // Temporary buffer to store this iteration's read in
            let mut buf = [0u8; READ_BUF_SIZE];

            // Read the next chunk of bytes
            let (early_exit, size) = match self
                .read_next_chunk(protocol, option, count, len, num_bytes, &mut buf)
                .await
            {
                Ok((exit, size)) => (exit, size),
                Err(e) => break Err(e),
            };

            // Add data to the USB buffer and send it if it's full
            let usb_result = self
                .process_and_send_usb_data(
                    &mut usb_buf,
                    &buf[0..size],
                    &mut usb_buf_count,
                    &mut usb_sent,
                )
                .await;

            if let Err(e) = usb_result {
                // Propagate the error, breaking out of the main read loop.
                break Err(e);
            }

            // Increment the count of bytes read
            count += size;

            // See if we're done
            if count >= len as usize {
                trace!("Read requested number of bytes {}", count);
                break Ok(());
            } else if early_exit {
                debug!("Exiting read loop early as rquested");
                break Err(DriverError::EarlyExit);
            }
        };

        // Send any remaining data in the buffer
        self.flush_usb_buffer(&usb_buf, usb_buf_count, &mut usb_sent)
            .await;

        // Now return either the number of bytes successfully read (which means
        // the number actually sent via USB to the host), or the error and the
        // number of bytes actuallt sent.
        result
            .map(|_| usb_sent as u16)
            .map_err(|e| (e, usb_sent as u16))
    }

    /// Read the next chunk of data
    ///
    /// Attempts to read the next chunk of data using the specified protocol
    /// and handles any early exit conditions.
    ///
    /// # Arguments
    /// * `protocol` - Protocol type to use for reading
    /// * `option` - Options for the read operation
    /// * `count` - Current count of bytes read
    /// * `len` - Maximum number of bytes to read
    /// * `num_bytes` - Expected number of bytes to read in this iteration
    /// * `usb_sent` - Current count of bytes sent to USB host
    /// * `buf` - Buffer to store the read bytes
    ///
    /// # Returns
    /// * `Ok((bool, usize))` - (early_exit flag, number of bytes read)
    /// * `Err(DriverError)` - Error encountered during read
    async fn read_next_chunk(
        &mut self,
        protocol: ProtocolType,
        option: ReadOption,
        count: usize,
        len: u16,
        num_bytes: usize,
        buf: &mut [u8],
    ) -> Result<(bool, usize), DriverError> {
        assert!(!buf.is_empty());

        // Read the byte(s)
        let result = match protocol {
            ProtocolType::Cbm => self.read_cbm(buf).await,
            ProtocolType::S1 => self.read_s1(buf).await,
            ProtocolType::S2 => self.read_s2(buf).await,
            ProtocolType::PP => self.read_pp(buf).await,
            ProtocolType::P2 => self.read_p2(buf).await,
            ProtocolType::Nib => {
                buf[0] = self.read_nib_handshaked(count & 1 == 1);
                if option == ReadOption::NibParReadEarlyExit && buf[0] == NIB_EARLY_EXIT_BYTE {
                    Err(DriverError::EarlyExit)
                } else {
                    Ok(1)
                }
            }
            ProtocolType::NibSrq => {
                buf[0] = self.read_nib_srq_byte();
                if count == (len as usize - 2) {
                    // We indicate to the drive to stop by releasing CLK
                    // before the last byte is read.
                    self.bus.release_clock();
                }
                Ok(1)
            }

            _ => unreachable!("Read: Unsupported protocol type"),
        };

        // Handle early_exit signal
        match result {
            Ok(n) => {
                // Check the read function returned the right number of bytes
                assert!(n == num_bytes);
                Ok((false, n))
            }

            // Handle both (NIB) EarlyExit and EOI as early exit conditions.
            // In both cases, we didn't read a (valid) byte.
            Err(DriverError::EarlyExit) | Err(DriverError::Eoi) => Ok((true, 0)),
            Err(e) => Err(e),
        }
    }

    /// Process and send data over USB
    ///
    /// Takes the data in `read_buf` and adds it to the `usb_buf`. When the USB buffer
    /// is full, sends the data to the host.
    ///
    /// # Arguments
    /// * `usb_buf` - Buffer used to store bytes before sending to USB host
    /// * `read_buf` - Buffer containing newly read bytes to process
    /// * `usb_buf_count` - Reference to count of bytes in the USB buffer
    /// * `usb_sent` - Reference to count of bytes sent to the USB host so far
    ///
    /// # Returns
    /// * `Ok(())` if successful
    /// * `Err(DriverError)` if there was an error sending data
    async fn process_and_send_usb_data(
        &self,
        usb_buf: &mut [u8; MAX_EP_PACKET_SIZE_USIZE],
        read_buf: &[u8],
        usb_buf_count: &mut usize,
        usb_sent: &mut usize,
    ) -> Result<(), DriverError> {
        // Add data to the USB buffer and send it if it's full.
        // This processes each byte in the read buffer one at a time.
        for &byte in read_buf {
            usb_buf[*usb_buf_count] = byte;
            *usb_buf_count += 1;

            if *usb_buf_count >= MAX_EP_PACKET_SIZE_USIZE {
                // Send the data
                Self::send_data_to_host(usb_buf, *usb_buf_count).await?;

                // Update USB sent and buffer count variables
                *usb_sent += *usb_buf_count;
                *usb_buf_count = 0;
            }
        }

        Ok(())
    }

    /// Flush any remaining data in the USB buffer
    ///
    /// Sends any remaining data in the USB buffer to the host, ignoring errors
    /// to ensure the original error is propagated.
    ///
    /// # Arguments
    /// * `usb_buf` - Buffer containing data to send
    /// * `usb_buf_count` - Count of bytes in the USB buffer
    /// * `usb_sent` - Reference to count of bytes sent to the USB host
    async fn flush_usb_buffer(
        &self,
        usb_buf: &[u8; MAX_EP_PACKET_SIZE_USIZE],
        usb_buf_count: usize,
        usb_sent: &mut usize,
    ) {
        if usb_buf_count > 0 {
            // Send any remaining data, whether or not there was an error.  It
            // is possible we'll be trying to send again after a USB send
            // error, but that shouldn't hurt and saves us having more complex
            // logic.  We will, however, ignore any errors from this send so we
            // propagate the original error.
            if Self::send_data_to_host(usb_buf, usb_buf_count)
                .await
                .is_ok()
            {
                *usb_sent += usb_buf_count;
            }
        }
    }

    // Gets the number of bytes read in each iteration through the main read
    // loop.
    fn get_read_num_iter_bytes(protocol: ProtocolType) -> usize {
        match protocol {
            ProtocolType::Cbm => DEFAULT_READ_ITER_BYTES,
            ProtocolType::S1 => DEFAULT_READ_ITER_BYTES,
            ProtocolType::S2 => DEFAULT_READ_ITER_BYTES,
            ProtocolType::PP => PP_READ_ITER_BYTES,
            ProtocolType::P2 => DEFAULT_READ_ITER_BYTES,
            ProtocolType::Nib => DEFAULT_READ_ITER_BYTES,
            ProtocolType::NibSrq => DEFAULT_READ_ITER_BYTES,
            _ => unreachable!("Read: Unsupported protocol type"),
        }
    }

    // Gets any options for this operation.
    fn get_read_option(len: u16, protocol: ProtocolType) -> ReadOption {
        match protocol {
            ProtocolType::Nib => {
                // Check whether early exit should be supported.
                if len & NIB_EARLY_EXIT_BIT == NIB_EARLY_EXIT_BIT {
                    ReadOption::NibParReadEarlyExit
                } else {
                    ReadOption::NibParReadNoEarlyExit
                }
            }
            _ => ReadOption::None,
        }
    }

    /// Performs the specific protocol read startup routine.
    async fn startup_read(&mut self, protocol: ProtocolType, _option: ReadOption) {
        match protocol {
            ProtocolType::Cbm => {}
            ProtocolType::S1 => {}
            ProtocolType::S2 => {}
            ProtocolType::PP => {}
            ProtocolType::P2 => {}
            ProtocolType::Nib => {
                // Wait for drive to be ready to send - 5ms is too short.
                // We yield here, because this is a long time.
                yield_ms!(10);

                // Read the first, dummy, byte and discard it
                let _ = self.read_nib_parburst().await;
            }
            ProtocolType::NibSrq => {
                // Release all lines
                self.bus.release_lines(IO_SRQ | IO_CLK | IO_DATA | IO_ATN);

                // Wait for drive to be ready to send - 5ms is too short.
                // We yield here, because this is a long time.
                yield_ms!(10);

                // Read the first, dummy, byte and discard it
                let _ = self.read_nib_srqburst().await;

                // Pulling CLK low signals the drive to start sending
                self.bus.set_clock();
            }
            _ => unreachable!("Read: Unsupported protocol type"),
        }
    }

    /// Performs specific protocol read termination.
    pub fn terminate_read(&mut self, protocol: ProtocolType, _option: ReadOption) {
        match protocol {
            ProtocolType::Cbm => {}
            ProtocolType::S1 => {}
            ProtocolType::S2 => {}
            ProtocolType::PP => {}
            ProtocolType::P2 => {}
            ProtocolType::Nib => {}
            ProtocolType::NibSrq => {
                // Release all lines
                self.bus.release_lines(IO_SRQ | IO_CLK | IO_DATA | IO_ATN);
            }
            _ => unreachable!("Read: Unsupported protocol type"),
        }
    }

    // Standard IEC read support
    async fn read_cbm(&mut self, data: &mut [u8]) -> Result<usize, DriverError> {
        // Wait for clock to be released, with 1s timeout.  Timing isn't
        // particularly critical here, so we can yield.
        self.wait_timeout_yield(IO_CLK, 0, READ_CLK_START_TIMEOUT, true, false)
            .await?;

        // Break if we've already seen EOI
        if self.get_eoi() {
            debug!("Raw read: EOI detected");
            return Err(DriverError::Eoi);
        }

        // Release DATA line to signal we're ready for data
        self.bus.release_data();

        // Wait up to 400us for CLK to be pulled low by the drive
        let clock = self.wait_timeout_block(IO_CLK, IO_CLK, READ_CLK_TIMEOUT);

        // Check for EOI signalling from talker
        if !clock {
            debug!("Clock high - so EOI signalled");
            self.set_eoi();
            self.bus.set_data();
            block_us!(70);
            self.bus.release_data();
        }

        // Read the byte
        match self.iec_receive_byte().await {
            Ok(byte) => {
                // Acknowledge byte received by pulling DATA
                self.bus.set_data();

                // Store the byte in our buffer
                data[0] = byte;

                // Brief pause
                block_us!(50);

                Ok(1)
            }
            Err(e) => {
                info!("Raw read: error receiving byte");
                Err(e)
            }
        }
    }

    /// Receive a single byte from the IEC bus
    async fn iec_receive_byte(&mut self) -> Result<u8, DriverError> {
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

    async fn read_nib_parburst(&mut self) -> u8 {
        // Set ATN and wait for drive to release DATA
        self.set_release(IO_ATN, IO_DATA | IO_CLK).await;
        block_us!(5);
        while self.bus.get_data() {}

        // Byte ready -- read it and release ATN
        block_us!(1);
        let byte = self.read_pp_byte();
        self.bus.release_atn();

        // Wait for the drive to pull DATA again. Delay for a bit afterwards
        // to keep the next read from being too close together.
        while !self.bus.get_data() {}
        block_us!(5);

        byte
    }

    // Caller must release DATAs before calling this function
    #[inline(always)]
    fn read_nib_handshaked(&mut self, toggle: bool) -> u8 {
        // Wait for a byte to be ready (data toggle matches expected value).
        while self.bus.get_data() != toggle {}

        // Read it directly from the port without debouncing.
        self.read_pp_byte()
    }

    async fn read_nib_srqburst(&mut self) -> u8 {
        // Set ATN, and wait for drive to start SRQ transfer
        self.set_release(IO_ATN, IO_SRQ | IO_CLK | IO_DATA).await;
        block_us!(1);

        // Read 8 bits via fast serial
        let byte = self.read_nib_srq_byte();
        self.bus.release_atn();

        // Wait for the drive to release CLK.
        while self.bus.get_clock() {}

        byte
    }

    #[inline(always)]
    fn read_nib_srq_byte(&mut self) -> u8 {
        let mut byte = 0;

        for _ in 0..8 {
            // Wait for the drive to pull IO_SRQ.
            while !self.bus.get_srq() {}

            // Wait for drive to release SRQ, then delay another 375 ns for
            // DATA to stabilize before reading it.
            while self.bus.get_srq() {}
            block_ns!(375);

            // Read data bit
            byte = (byte << 1) | (self.bus.get_data() as u8);
        }

        byte
    }
}
