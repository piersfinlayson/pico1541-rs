//! Implements USB data transfer logic.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

//
// Statics
//

#![allow(dead_code)]

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

use crate::constants::{TRANSFER_DATA_BUFFER_SIZE, USB_DATA_TRANSFER_WAIT_TIMER};
use crate::protocol::types::Direction;
use crate::protocol::ProtocolType;

/// A static for IN/Read and OUT/Write USB bulk data transfers between the
/// host and device.  We use a ThreadModeRawMutex, as the Bulk task (which
/// sends/receives the USB data) and the Protocol Handler both run on the
/// same core, core 1.
pub static USB_DATA_TRANSFER: Mutex<ThreadModeRawMutex, UsbDataTransfer> =
    Mutex::new(UsbDataTransfer::new_default());

/// Used to handle In and Out data transfers between host and device.
///
/// This struct is set up when a WRITE or READ bulk transfer command is
/// received, and is used to manage the transfer of data to or from the host.
///
/// Users can either lock the USB_DATA_TRANSFER static and call the instance
/// methods directly (for example, if they want to perform multiple
/// operations with relinquishing the lock) or call the associated functions,
/// which will handle locking.
pub struct UsbDataTransfer {
    /// Which direction the transfer is in.  In means from device to host, Out
    /// means from host to device.  None means the transfer is inactive.
    direction: Option<Direction>,

    // The protocol for this transfer.  This is used for ProtocolHandler to
    // device whether to send a status response once the transfer is complete.
    protocol: Option<ProtocolType>,

    /// Total expected bytes in this operation
    expected_bytes: u16,

    /// Total bytes received so far
    received_bytes: u16,

    /// The data buffer.  We set this to twice the maximum endpoint packet
    /// size, so we can handle the maximum received packet size in one go.
    data: [u8; TRANSFER_DATA_BUFFER_SIZE],

    /// The current read position from this buffer - i.e the next byte to
    /// read.
    read_pos: usize,

    /// The number of valid bytes in the buffer.
    valid_bytes: usize,

    /// Response.  Whether the command succeeded or failed.
    response: UsbTransferResponse,
}

// Public instance methods
impl UsbDataTransfer {
    /// Implement a default.  Needs to be a const fn so it can be used in the
    /// static.
    const fn new_default() -> Self {
        Self {
            direction: None,
            protocol: None,
            expected_bytes: 0,
            received_bytes: 0,
            data: [0; TRANSFER_DATA_BUFFER_SIZE],
            read_pos: 0,
            valid_bytes: 0,
            response: UsbTransferResponse::None,
        }
    }

    /// Initialize the transfer
    pub fn init(&mut self, direction: Direction, protocol: ProtocolType, expected_bytes: u16) {
        // TODO consider optimising out the clearing of the data buffer
        self.direction = Some(direction);
        self.protocol = Some(protocol);
        self.expected_bytes = expected_bytes;
        self.received_bytes = 0;
        self.data = [0; TRANSFER_DATA_BUFFER_SIZE];
        self.read_pos = 0;
        self.valid_bytes = 0;
        self.response = UsbTransferResponse::None;
    }

    /// Clear the transfer
    pub fn clear(&mut self) {
        self.direction = None;
        self.protocol = None;
        self.expected_bytes = 0;
        self.received_bytes = 0;
        self.read_pos = 0;
        self.valid_bytes = 0;
        self.response = UsbTransferResponse::None;
    }

    /// Get the direction of the transfer
    pub fn direction(&self) -> Option<Direction> {
        self.direction
    }

    /// Get the protcol of the transfer
    pub fn protocol(&self) -> Option<ProtocolType> {
        self.protocol
    }

    /// Get the response
    pub fn get_response(&self) -> UsbTransferResponse {
        self.response.clone()
    }

    /// Set the response
    pub fn set_response(&mut self, response: UsbTransferResponse) {
        self.response = response;
    }

    /// Query expected bytes
    pub fn expected_bytes(&self) -> u16 {
        self.expected_bytes
    }

    /// Query remaining bytes
    pub fn remaining_bytes(&self) -> u16 {
        self.expected_bytes - self.received_bytes
    }

    /// Query outstanding bytes
    pub fn outstanding_bytes(&self) -> usize {
        self.valid_bytes
    }

    /// Check if there are outstanding bytes
    pub fn outstanding(&self) -> bool {
        self.valid_bytes > 0
    }

    /// Internal helper to get a single byte from the buffer
    /// Assumes all checks have been performed and there is at least one byte available
    #[inline]
    fn read_byte_unchecked(&mut self) -> u8 {
        let byte = self.data[self.read_pos];
        self.read_pos += 1;
        self.valid_bytes -= 1;
        if self.read_pos >= TRANSFER_DATA_BUFFER_SIZE {
            self.read_pos = 0;
        }
        byte
    }

    /// Try to get next byte
    pub fn try_get_next_byte(&mut self) -> Option<u8> {
        if self.direction.is_none() {
            defmt::panic!("Tried to get a byte from an inactive transfer");
        }

        if self.valid_bytes == 0 {
            return None;
        }

        Some(self.read_byte_unchecked())
    }

    /// Try to get multiple bytes from the buffer at once
    ///
    /// This function will attempt to fill the provided buffer with bytes from the data
    /// transfer. It will return the number of bytes actually read, which may be less than
    /// the buffer length if there aren't enough bytes available.
    ///
    /// Returns None if the transfer isn't active or if there are no bytes available.
    pub fn try_get_next_bytes(&mut self, buffer: &mut [u8]) -> Option<usize> {
        if self.direction.is_none() {
            warn!("Tried to get bytes from an inactive transfer");
            return None;
        }

        if self.valid_bytes == 0 {
            return None;
        }

        // Determine how many bytes we can read
        let bytes_to_read = buffer.len().min(self.valid_bytes);
        if bytes_to_read == 0 {
            return None;
        }

        // Read bytes into the buffer
        for byte in &mut buffer[..bytes_to_read] {
            *byte = self.read_byte_unchecked();
        }

        Some(bytes_to_read)
    }

    /// Check if the buffer is full
    pub fn is_full(&self) -> bool {
        self.valid_bytes >= TRANSFER_DATA_BUFFER_SIZE
    }

    /// Query how much space is available in the buffer
    pub fn available_space(&self) -> usize {
        (TRANSFER_DATA_BUFFER_SIZE) - self.valid_bytes
    }

    /// Internal helper to add a byte to the buffer
    /// Assumes all checks have been performed and there is space
    #[inline]
    fn write_byte_unchecked(&mut self, byte: u8) {
        let write_pos = (self.read_pos + self.valid_bytes) % (TRANSFER_DATA_BUFFER_SIZE);
        self.data[write_pos] = byte;
        self.valid_bytes += 1;
        self.received_bytes += 1;
    }

    /// Internal helper to check if we can add more bytes
    /// Returns Err if buffer is full or transfer is complete
    #[inline]
    fn check_can_write(&self) -> Result<(), UsbTransferError> {
        if self.is_full() {
            return Err(UsbTransferError::BufferFull);
        }

        if self.received_bytes >= self.expected_bytes {
            return Err(UsbTransferError::TransferComplete);
        }

        Ok(())
    }

    /// Attempt to add a single byte to the buffer
    pub fn try_add_byte(&mut self, byte: u8) -> Result<(), UsbTransferError> {
        self.check_can_write()?;
        self.write_byte_unchecked(byte);
        Ok(())
    }

    /// Attempt to add multiple bytes to the buffer
    pub fn try_add_bytes(&mut self, bytes: &[u8]) -> Result<usize, UsbTransferError> {
        self.check_can_write()?;

        let space = self.available_space();
        let remaining_transfer = (self.expected_bytes - self.received_bytes) as usize;
        let max_bytes = space.min(remaining_transfer).min(bytes.len());

        for &byte in bytes.iter().take(max_bytes) {
            self.write_byte_unchecked(byte);
        }

        // Always return Ok with the number of bytes actually written
        // (which could be 0 if we somehow got here with no space, though check_can_write should prevent that)
        Ok(max_bytes)
    }
}

// Associated functions that wrap instance functions by locking the
// USB_DATA_TRANSFER static.
impl UsbDataTransfer {
    /// Initialize the transfer
    pub async fn lock_init(direction: Direction, protocol: ProtocolType, expected_bytes: u16) {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.init(direction, protocol, expected_bytes)
    }

    /// Clear the transfer
    pub async fn lock_clear() {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.clear()
    }

    /// Get the direction of the transfer
    pub async fn lock_direction() -> Option<Direction> {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.direction()
    }

    /// Get the response
    pub async fn lock_get_response() -> UsbTransferResponse {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.get_response()
    }

    /// Set the response
    pub async fn lock_set_response(response: UsbTransferResponse) {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.set_response(response)
    }

    /// Query expected bytes
    pub async fn lock_expected_bytes() -> u16 {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.expected_bytes()
    }

    /// Query how many bytes are remaining in this transfer.
    pub async fn lock_remaining_bytes() -> u16 {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.remaining_bytes()
    }

    /// Query how many bytes are sitting in the buffer, waiting to be read.
    pub async fn lock_outstanding_bytes() -> usize {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.outstanding_bytes()
    }

    /// Query whether there are any bytes outstanding in the buffer.
    pub async fn lock_outstanding() -> bool {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.outstanding()
    }

    /// Wait until there are outstanding bytes
    pub async fn lock_wait_outstanding() {
        loop {
            // Lock the mutex
            let guard = USB_DATA_TRANSFER.lock().await;

            // Check if there are outstanding bytes
            if guard.outstanding() {
                break;
            }

            // Unlock the mutex and wait, briefly, to allow bytes to appear
            drop(guard);
            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }
    }

    /// Try to get the next byte from the buffer, but do not wait for one to
    /// become available.
    pub async fn lock_try_get_next_byte() -> Option<u8> {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.try_get_next_byte()
    }

    /// Try to get multiple bytes from the buffer at once, with locking
    ///
    /// This function will attempt to fill the provided buffer with bytes from the data
    /// transfer. It will return the number of bytes actually read, which may be less than
    /// the buffer length if there aren't enough bytes available.
    ///
    /// Returns None if the transfer isn't active or if there are no bytes available.
    pub async fn lock_try_get_next_bytes(buffer: &mut [u8]) -> Option<usize> {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.try_get_next_bytes(buffer)
    }

    /// Get multiple bytes from the buffer, waiting for them to become available
    ///
    /// This function will wait until at least one byte is available and then attempt to fill
    /// the provided buffer. It returns the number of bytes actually read.
    pub async fn lock_get_next_bytes(buffer: &mut [u8]) -> usize {
        loop {
            {
                let mut guard = USB_DATA_TRANSFER.lock().await;
                if let Some(bytes_read) = guard.try_get_next_bytes(buffer) {
                    return bytes_read;
                }
            }
            // Release the lock during delay
            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }
    }

    /// Get the next byte from the buffer, waiting for one to become available.
    /// There is no instance method for this, as it has to release the mutex,
    /// wait and then relock the mutex.
    pub async fn lock_get_next_byte() -> u8 {
        loop {
            {
                let mut guard = USB_DATA_TRANSFER.lock().await;
                if let Some(byte) = guard.try_get_next_byte() {
                    return byte;
                }
            }
            // Release the lock during delay
            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }
    }

    /// Check if the buffer is full
    pub async fn lock_is_full() -> bool {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.is_full()
    }

    /// Query how much space is available in the buffer
    pub async fn lock_available_space() -> usize {
        let guard = USB_DATA_TRANSFER.lock().await;
        guard.available_space()
    }

    /// Wait until at least the specified amount of space is available in the
    /// buffer.
    /// Returns the actual amount of space available (which may be more than
    /// requested).
    /// There is no instance method for this, as it has to release the mutex,
    /// wait and then relock the mutex.
    pub async fn lock_wait_space_available(required_space: usize) -> usize {
        loop {
            {
                let guard = USB_DATA_TRANSFER.lock().await;
                let space = guard.available_space();
                if space >= required_space {
                    return space;
                }
            }
            // Release the lock during delay
            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }
    }

    /// Attempt to add a single byte to the buffer
    pub async fn lock_try_add_byte(byte: u8) -> Result<(), UsbTransferError> {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.try_add_byte(byte)
    }

    /// Attempt to add multiple bytes to the buffer
    pub async fn lock_try_add_bytes(bytes: &[u8]) -> Result<usize, UsbTransferError> {
        let mut guard = USB_DATA_TRANSFER.lock().await;
        guard.try_add_bytes(bytes)
    }
}

#[derive(Debug)]
pub enum UsbTransferError {
    BufferFull,
    TransferComplete,
}

/// Response to a USB transfer
#[derive(Debug, Clone, PartialEq)]
pub enum UsbTransferResponse {
    /// USB transfer is still acive
    None,

    /// USB transfer is complete, and how many bytes
    Ok(u16),

    /// USB transfer failed
    Error,
}

impl defmt::Format for UsbTransferResponse {
    fn format(&self, f: defmt::Formatter) {
        match self {
            UsbTransferResponse::None => defmt::write!(f, "None"),
            UsbTransferResponse::Ok(bytes) => defmt::write!(f, "Ok {} bytes", bytes),
            UsbTransferResponse::Error => defmt::write!(f, "Error"),
        }
    }
}
