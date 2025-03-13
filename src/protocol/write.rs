//! This file implements the writing of bytes to the drive, for all supported
//! protocols.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use static_assertions::const_assert;

use super::driver::{DriverError, ProtocolDriver};
use super::iec::{IecDriver, IO_ATN, IO_CLK, IO_DATA, IO_RESET, IO_SRQ};
use super::ProtocolFlags;
use super::ProtocolType;

use crate::constants::MAX_EP_PACKET_SIZE_USIZE;
use crate::usb::transfer::UsbDataTransfer;
use crate::util::time::iec::{
    IEC_T_BB, IEC_T_NE, IEC_T_R, IEC_T_S, IEC_T_V, WRITE_TALK_CLK_TIMEOUT,
};
use crate::util::time::{block_ns, block_us, iec_delay};

// Constants covering number of bytes written per read loop iteration
const WRITE_BUF_SIZE: usize = 2;
const DEFAULT_WRITE_ITER_BYTES: usize = 1;
const PP_WRITE_ITER_BYTES: usize = 2;
const_assert!(WRITE_BUF_SIZE >= DEFAULT_WRITE_ITER_BYTES);
const_assert!(WRITE_BUF_SIZE >= PP_WRITE_ITER_BYTES);

impl IecDriver {
    /// Main write routine which handles all protocols.
    ///
    /// See [`ProtocolDriver::read`] for more information about the specific
    /// protocols.
    ///
    /// Implements the following steps:
    /// - Makes sure there are bytes waiting to be written before starting (to
    ///   reduce delays)
    /// - (Unlike read, does not get options for this operation, as there are
    ///   none for writes)
    /// - Calls the protcol specific startup routine to prep for writing
    /// - Runs the main write loop, reading data from USB and sending data to
    ///   the drive.
    /// - Terminates the write routine
    /// - Returns the nuber of bytes written, or an error.
    ///
    /// Returns
    /// - Ok(u16) - Number of bytes written
    /// - Err((DriverError, u16)) - Error and number of bytes written to the
    ///   drive, before the error occurred.
    pub async fn write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        // Wait for there to be bytes waiting to be written before starting.
        UsbDataTransfer::lock_wait_outstanding().await;
        Self::feed_watchdog();

        // Do protocol specific startup routine.  Propogate any error, as we
        // don't need to terminate (startup_write() must do any clean-up).
        self.startup_write(protocol, flags)
            .await
            .map_err(|e| (e, 0))?;

        // Call the main write loop
        let loop_result = self.write_main_loop(len, protocol, flags).await;

        // Protocol specific termination routine - we terminate whether or not
        // we hit an error.  We need to know the result of the main loop in
        // here, as the CBM protocol has to clear some lines if it fails.  It
        // may also fail, hence may modify it and will return any updated
        // value.
        let term_result = self.terminate_write(loop_result, protocol, flags).await;

        // Feed the watchdog, as belt and braces in case it's been a while.
        Self::feed_watchdog();

        term_result
    }

    /// Protocol specific startup routine for writing.
    async fn startup_write(
        &mut self,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<(), DriverError> {
        match protocol {
            ProtocolType::Cbm => self.cbm_startup_write(flags).await,
            ProtocolType::Nib => {
                // Release data line
                self.bus.release_data();
                Ok(())
            }
            ProtocolType::NibSrq => {
                // Release all lines
                self.bus.iec_release(IO_SRQ | IO_CLK | IO_DATA | IO_ATN);
                Ok(())
            }
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::PP | ProtocolType::P2 => Ok(()),
            // TODO NibCommand
            _ => unreachable!("Non-CBM read: Unsupported protocol type"),
        }
    }

    // A dedicated function to the startup routine for a CBM protocol write,
    // as it's quite involved, and can fail.
    async fn cbm_startup_write(&mut self, flags: ProtocolFlags) -> Result<(), DriverError> {
        // Check if any device is present on the bus.  If both lines stay low
        // then this check fails - and corresponds to a device present but
        // powered off.  If we stayed, we'd get stuck in wait_for_listener().
        if !self.wait_timeout_2ms(IO_ATN | IO_RESET, 0) {
            debug!("Raw Write: No devices present on the bus");
            debug!("Poll pins returns 0x{:02x}", self.bus.poll_pins());
            // TO DO - async read handling
            return Err(DriverError::NoDevices);
        }

        // Release DATA.
        self.bus.release_data();

        // Either pull CLOCK low, or both ATN and CLOCK.
        match flags.is_atn() {
            true => self.bus.set_lines(IO_CLK | IO_ATN),
            false => self.bus.set_lines(IO_CLK),
        }

        // Short delay to let lines settle
        iec_delay!();

        // Wait for any device to pull data after we set CLK.
        // This should be IEC_T_AT (1ms) but allow a bit longer.
        if !self.wait_timeout_2ms(IO_DATA, IO_DATA) {
            debug!("Raw Write: No devices detected");
            trace!("Poll pins returns 0x{:02x}", self.bus.poll_pins() & IO_DATA);
            self.bus.release_lines(IO_CLK | IO_ATN);
            return Err(DriverError::NoDevices);
        }

        // Wait for drive to be ready for us to release CLK.  The transfer
        // starts to be unreliable below 10 us.
        block_us!(IEC_T_NE);

        Ok(())
    }

    // Terminate the write routine
    async fn terminate_write(
        &mut self,
        loop_result: Result<u16, (DriverError, u16)>,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        match protocol {
            ProtocolType::Cbm => self.cbm_terminate_write(loop_result, flags).await,
            ProtocolType::Nib => {
                // TODO
                // Release data line
                //nib_write_handshaked(0, i & 1);
                //nib_parburst_read();
                loop_result
            }
            ProtocolType::NibSrq => {
                // TODO
                // Release all lines
                //nib_srqburst_read();
                loop_result
            }
            ProtocolType::S1 | ProtocolType::S2 | ProtocolType::PP | ProtocolType::P2 => {
                loop_result
            }
            // TODO NibCommand
            _ => unreachable!("Non-CBM read: Unsupported protocol type"),
        }
    }

    async fn cbm_terminate_write(
        &mut self,
        loop_result: Result<u16, (DriverError, u16)>,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        match loop_result {
            Ok(count) => {
                if flags.is_talk() {
                    // Now put the drive into TALK mode.
                    trace!("Put drive into Talk mode");

                    // Hold DATA and release ATN
                    self.set_release(IO_DATA, IO_ATN).await;

                    // IEC_T_TK was defined to be -1 in the original
                    // C code which presumably meant to make the
                    // delay sub micro-second.
                    block_ns!(500);

                    // Release CLK and wait for device to grab it
                    self.bus.release_clock();
                    iec_delay!();

                    // Wait for device to pull CLOCK low before
                    // returning.  As self.wait() is an external API,
                    // it takes IEC_* lines not IO_lines*
                    self.wait_timeout_yield(IO_CLK, IO_CLK, WRITE_TALK_CLK_TIMEOUT, true, false)
                        .await
                        .map(|_| count)
                        .map_err(|e| (e, count))
                } else {
                    self.bus.release_atn();
                    Ok(count)
                }
            }
            Err((e, count)) => {
                block_us!(IEC_T_R);
                self.bus.iec_release(IO_CLK | IO_ATN);
                Err((e, count))
            }
        }
    }

    async fn write_main_loop(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)> {
        // Get number of bytes each write loop iteration below.
        let num_bytes = Self::get_write_num_iter_bytes(protocol);

        // Tracking variables for write and USB receive operations
        const SIZE: usize = MAX_EP_PACKET_SIZE_USIZE; // Size of the buffer
        let mut count = 0; // Number of bytes written so far
        let mut usb_buf = [0u8; SIZE]; // Buffer for received USB data
        let mut usb_buf_count = 0; // Number of bytes in USB buffer
        let mut usb_written = 0; // Count of bytes from USB buffer written so far

        let result = loop {
            // Temporary buffer to store this iteration's write data in
            let mut buf = [0u8; WRITE_BUF_SIZE];

            // Fill the temporary buffer with the data chunk to be written
            // next
            for byte in buf.iter_mut().take(num_bytes) {
                // Read data from USB if we need some data.  We attempt to
                // fill our USB buffer, so we don't need to read as often.
                while usb_buf_count == 0 || usb_written == usb_buf_count {
                    // Read data from USB
                    usb_written = 0;
                    usb_buf_count = UsbDataTransfer::lock_get_next_bytes(&mut usb_buf).await;
                }
                *byte = usb_buf[usb_written];
                usb_written += 1;
            }

            // Write the next chunk off bytes of data to the drive
            let size = match self
                .write_next_chunk(protocol, flags, count, len, &mut buf)
                .await
            {
                Ok(size) => size,
                Err(e) => break Err(e),
            };

            // Increment the count of bytes written
            count += size;

            // See if we're done
            if count >= len as usize {
                trace!("Wrote requested number of bytes {}", count);
                break Ok(());
            }
        };

        // Now return the number of bytes successfully written
        result.map(|_| count as u16).map_err(|e| (e, count as u16))
    }

    // Gets the number of bytes read in each iteration through the main read
    // loop.
    fn get_write_num_iter_bytes(protocol: ProtocolType) -> usize {
        match protocol {
            ProtocolType::Cbm => DEFAULT_WRITE_ITER_BYTES,
            ProtocolType::S1 => DEFAULT_WRITE_ITER_BYTES,
            ProtocolType::S2 => DEFAULT_WRITE_ITER_BYTES,
            ProtocolType::PP => PP_WRITE_ITER_BYTES,
            ProtocolType::P2 => DEFAULT_WRITE_ITER_BYTES,
            ProtocolType::Nib => DEFAULT_WRITE_ITER_BYTES,
            ProtocolType::NibSrq => DEFAULT_WRITE_ITER_BYTES,
            _ => unreachable!("Write: Unsupported protocol type"),
        }
    }

    async fn write_next_chunk(
        &mut self,
        protocol: ProtocolType,
        flags: ProtocolFlags,
        count: usize,
        len: u16,
        buf: &mut [u8],
    ) -> Result<usize, DriverError> {
        assert!(!buf.is_empty());

        // Write the byte(s)
        match protocol {
            ProtocolType::Cbm => self.write_cbm(flags, count, len, buf[0]).await,
            ProtocolType::S1 => self.write_s1(buf[0]).await,
            ProtocolType::S2 => self.write_s2(buf[0]).await,
            ProtocolType::PP => {
                assert!(buf.len() >= 2);
                self.write_pp(buf[0], buf[1]).await
            }
            ProtocolType::P2 => self.write_p2(buf[0]).await,
            ProtocolType::Nib => {
                // TODO
                Ok(1)
            }
            ProtocolType::NibSrq => {
                // TODO
                Ok(1)
            }

            _ => unreachable!("Write: Unsupported protocol type"),
        }
    }

    async fn write_cbm(
        &mut self,
        flags: ProtocolFlags,
        count: usize,
        len: u16,
        byte: u8,
    ) -> Result<usize, DriverError> {
        // Need to signal EOI for the last byte
        let is_last_byte = count == len as usize - 1;

        // Be sure DATA line has been pulled by device
        if !self.bus.get_data() {
            debug!("Raw write: Device not present");
            return Err(DriverError::NoDevice);
        }

        debug!("Data line low");

        // Release CLK and wait for listener to release data
        if !self.wait_for_listener().await {
            debug!("Raw write: No listener");
            return Err(DriverError::Timeout);
        }

        debug!("Listener released DATA");

        // Signal EOI for last byte
        if is_last_byte && !flags.is_atn() {
            self.wait_timeout_2ms(IO_DATA, IO_DATA);
            self.wait_timeout_2ms(IO_DATA, 0);
        }

        // Pull CLOCK low
        self.bus.set_lines(IO_CLK);

        // Send the byte
        if self.iec_send_byte(byte).await {
            block_us!(IEC_T_BB);
        } else {
            debug!("Raw write: io err");
            return Err(DriverError::Io);
        }

        trace!("Written byte #{} to drive", count + 1);

        Ok(1)
    }

    /// Send a byte, one bit at a time via the IEC protocol
    ///
    /// Timing is critical here, so we use block_us.
    async fn iec_send_byte(&mut self, b: u8) -> bool {
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
}
