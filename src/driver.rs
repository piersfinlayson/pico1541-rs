//! This file defines the ProtocolDriver trait, which is implemented by the
//! various Commodore protocol implementations.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use crate::protocol::ProtocolFlags;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Endpoint, In};

/// Defines errors for ProtocolDriver implementations.
#[allow(dead_code)]
#[derive(defmt::Format, Debug)]
pub enum DriverError {
    /// Device timeout - no response within expected time
    Timeout,
    /// A reset cancelled this operation
    Resetting,
    /// Bus was occupied
    BusOccupied,
    /// No devices detected
    NoDevices,
    /// No device detected
    NoDevice,
    /// IoError
    Io,
    /// Unsupported protocol or operation
    Unsupported,
    /// Internal error
    Internal,
}

/// Defines the interface for Commodore protocol implementations.
/// Each supported protocol (IEC, parallel, etc.) must implement this trait
/// to provide the core communication functionality needed by the xum1541.
#[allow(dead_code)]
pub trait ProtocolDriver {
    /// Reset the bus, optionally holding the reset line indefinitely.
    ///
    /// # Arguments
    /// * `forever` - Whether to wait forever for the bus to free after
    ///   resetting
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError>;

    /// Write raw bytes to the bus with specified flags.
    ///
    /// # Arguments
    /// * `data` - The bytes to write
    /// * `flags` - Control flags affecting the write operation
    ///
    /// # Returns
    /// The number of bytes actually written or an error
    async fn raw_write(&mut self, data: &[u8], flags: ProtocolFlags) -> Result<u16, DriverError>;

    /// Read raw bytes from the bus.
    ///
    /// # Arguments
    /// * `read` - The Read objects
    /// * `write_ep` - The endpoint to write data to
    ///
    /// # Returns
    /// The number of bytes actually read or an error
    async fn raw_read(
        &mut self,
        len: u16,
        write_ep: &mut Endpoint<'static, USB, In>,
    ) -> Result<u16, DriverError>;

    /// Wait for a specific bus line to reach the desired state.
    ///
    /// # Arguments
    /// * `line` - The bus line to monitor
    /// * `state` - The state to wait for (typically high/low)
    ///
    /// # Returns
    /// Ok(()) if the line reached the desired state otherwise an error
    async fn wait(&mut self, line: u8, state: u8) -> Result<(), DriverError>;

    /// Poll the current state of the bus lines.
    ///
    /// # Returns
    /// A bitfield representing the current state of all bus lines
    async fn poll(&mut self) -> u8;

    /// Configure which bus lines should be set or released.
    ///
    /// # Arguments
    /// * `set` - Bitfield of lines to set (drive active)
    /// * `release` - Bitfield of lines to release (float/inactive)
    async fn set_release(&mut self, set: u8, release: u8);
}
