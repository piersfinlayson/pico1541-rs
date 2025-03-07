//! This file defines the ProtocolDriver trait, which is implemented by the
//! various Commodore protocol implementations.  This is based on the xum1541
//! source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

use crate::iec::IecDriver;
use crate::ieee::IeeeDriver;
use crate::protocol::{ProtocolFlags, ProtocolType};
use crate::tape::TapeDriver;
use crate::transfer::{UsbDataTransfer, UsbTransferResponse};

// A static for the current driver in use.
pub static DRIVER: Mutex<ThreadModeRawMutex, Option<Driver>> = Mutex::new(None);

/// Defines errors for ProtocolDriver implementations.
#[allow(dead_code)]
#[derive(defmt::Format, Debug)]
pub enum DriverError {
    /// Device timeout - no response within expected time
    Timeout,
    /// The operation was aborted
    Abort,
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
    /// Try lock error
    TryLock,
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
    /// * `len` - The number of bytes to write
    /// * `protocol` - The protocol type
    /// * `flags` - Control flags affecting the write operation
    ///
    /// # Returns
    /// The number of bytes actually written or an error
    async fn raw_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, DriverError>;

    /// Read raw bytes from the bus.
    ///
    /// # Arguments
    /// * `read` - The Read objects
    /// * `protocol` - The protocol type
    /// * `write_ep` - The endpoint to write data to
    ///
    /// # Returns
    /// The number of bytes actually read or an error
    async fn raw_read(&mut self, protocol: ProtocolType, len: u16) -> Result<u16, DriverError>;

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
    fn poll(&mut self) -> u8;

    /// Configure which bus lines should be set or released.
    ///
    /// # Arguments
    /// * `set` - Bitfield of lines to set (drive active)
    /// * `release` - Bitfield of lines to release (float/inactive)
    fn setrelease(&mut self, set: u8, release: u8);

    /// Get the End Of Indication (EOI) status
    fn get_eoi(&self) -> bool;

    /// Clear the End of Indiciation (EOI) status
    fn clear_eoi(&mut self);
}

pub enum Driver {
    Iec(IecDriver),
    #[allow(dead_code)]
    Ieee(IeeeDriver),
    #[allow(dead_code)]
    Tape(TapeDriver),
}

impl ProtocolDriver for Driver {
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        match self {
            Driver::Iec(driver) => driver.reset(forever).await,
            Driver::Ieee(driver) => driver.reset(forever).await,
            Driver::Tape(driver) => driver.reset(forever).await,
        }
    }

    async fn raw_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, DriverError> {
        match self {
            Driver::Iec(driver) => driver.raw_write(len, protocol, flags).await,
            Driver::Ieee(driver) => driver.raw_write(len, protocol, flags).await,
            Driver::Tape(driver) => driver.raw_write(len, protocol, flags).await,
        }
    }

    async fn raw_read(&mut self, protocol: ProtocolType, len: u16) -> Result<u16, DriverError> {
        match self {
            Driver::Iec(driver) => driver.raw_read(protocol, len).await,
            Driver::Ieee(driver) => driver.raw_read(protocol, len).await,
            Driver::Tape(driver) => driver.raw_read(protocol, len).await,
        }
    }

    async fn wait(&mut self, line: u8, state: u8) -> Result<(), DriverError> {
        match self {
            Driver::Iec(driver) => driver.wait(line, state).await,
            Driver::Ieee(driver) => driver.wait(line, state).await,
            Driver::Tape(driver) => driver.wait(line, state).await,
        }
    }

    fn poll(&mut self) -> u8 {
        match self {
            Driver::Iec(driver) => driver.poll(),
            Driver::Ieee(driver) => driver.poll(),
            Driver::Tape(driver) => driver.poll(),
        }
    }

    fn setrelease(&mut self, set: u8, release: u8) {
        match self {
            Driver::Iec(driver) => driver.setrelease(set, release),
            Driver::Ieee(driver) => driver.setrelease(set, release),
            Driver::Tape(driver) => driver.setrelease(set, release),
        }
    }

    fn get_eoi(&self) -> bool {
        match self {
            Driver::Iec(driver) => driver.get_eoi(),
            Driver::Ieee(driver) => driver.get_eoi(),
            Driver::Tape(driver) => driver.get_eoi(),
        }
    }

    fn clear_eoi(&mut self) {
        match self {
            Driver::Iec(driver) => driver.clear_eoi(),
            Driver::Ieee(driver) => driver.clear_eoi(),
            Driver::Tape(driver) => driver.clear_eoi(),
        }
    }
}

#[allow(dead_code)]
impl Driver {
    pub async fn is_none() -> bool {
        DRIVER.lock().await.is_none()
    }

    pub fn try_is_none() -> Result<bool, DriverError> {
        DRIVER
            .try_lock()
            .map(|driver| driver.is_none())
            .map_err(|_| DriverError::TryLock)
    }
}

/// Task to perform a raw_write() operation, using the active driver.
///
/// Important - DRIVER will be locked for the timetime of this task.  If
/// the driver is not set, this task will immediately exits, setting error.
///
/// We deliberately set the response type in this function, as the task exits
/// so that the spawner can be sure, once response is set, that this task is
/// done, and hence the driver is unlocked.
#[embassy_executor::task]
pub async fn raw_write_task(len: u16, protocol: ProtocolType, flags: ProtocolFlags) {
    debug!("Starting raw_write_task");

    let response = {
        let mut guard = DRIVER.lock().await;
        let guard = guard.as_mut();

        match guard {
            Some(guard) => {
                // Call raw_write and set the response appropriately.
                guard
                    .raw_write(len, protocol, flags)
                    .await
                    .map(|_| UsbTransferResponse::Ok)
                    .unwrap_or(UsbTransferResponse::Error)
            }
            None => {
                warn!("No driver set for raw_write_task");
                UsbTransferResponse::Error
            }
        }
    };
    // Driver is unlocked here.

    UsbDataTransfer::lock_set_response(response).await;

    debug!("Finished raw_write_task");
}

/// Task to perform a raw_read() operation, using the active driver.
///
/// Important - DRIVER will be locked for the timetime of this task.  If
/// the driver is not set, this task will immediately exits, setting error.
///
/// We deliberately set the response type in this function, as the task exits
/// so that the spawner can be sure, once response is set, that this task is
/// done, and hence the driver is unlocked.
#[embassy_executor::task]
pub async fn raw_read_task(protocol: ProtocolType, len: u16) {
    debug!("Starting raw_read_task");

    let response = {
        let mut guard = DRIVER.lock().await;
        let guard = guard.as_mut();

        match guard {
            Some(guard) => {
                // Call raw_read and set the response appropriately.
                guard
                    .raw_read(protocol, len)
                    .await
                    .map(|_| UsbTransferResponse::Ok)
                    .unwrap_or(UsbTransferResponse::Error)
            }
            None => {
                warn!("No driver set for raw_read_task");
                UsbTransferResponse::Error
            }
        }
    };
    // Driver is unlocked here.

    UsbDataTransfer::lock_set_response(response).await;

    debug!("Finished raw_read_task");
}
