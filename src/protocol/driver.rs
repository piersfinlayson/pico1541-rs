//! This file defines the ProtocolDriver trait, which is implemented by the
//! various Commodore protocol implementations.  This is based on the xum1541
//! source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use core::sync::atomic::{AtomicBool, Ordering};
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;

use crate::protocol::iec::IecDriver;
use crate::protocol::ieee::IeeeDriver;
use crate::protocol::tape::TapeDriver;
use crate::protocol::{ProtocolFlags, ProtocolType};
use crate::usb::transfer::{UsbDataTransfer, UsbTransferResponse};

//
// Statics
//

// A static for the current driver in use.
pub static DRIVER: Mutex<ThreadModeRawMutex, Option<Driver>> = Mutex::new(None);

// Static used to signal to a driver task whether it should abort.
pub static ABORT_DRIVER_TASK: AtomicBool = AtomicBool::new(false);

/// Defines errors for ProtocolDriver implementations.
#[allow(dead_code)]
#[derive(defmt::Format, Debug)]
pub enum DriverError {
    // Errors which are signalled back to the host with a successful status
    /// IoError
    Io,
    /// Device timeout - no response within expected time
    Timeout,
    /// Early exit - on a Nib parallel read
    EarlyExit,
    /// No devices detected
    NoDevices,
    /// No device detected
    NoDevice,

    // Erors which are signalled back to the host as errors.
    /// The operation was aborted
    Abort,
    /// Bus was occupied
    BusOccupied,
    /// Unsupported protocol or operation
    Unsupported,
    /// Internal error
    Internal,
    /// Try lock error
    TryLock,
}

// Provide a method to convert DriverError to UsbTransferResponse.  The
// protocol handler will use the UsbTransferResponse to device what to send
// back to the host on a write.  The original xum1541 code uses OK for all
// errors it detects, but uses the count, on the OK, to signal if the data
// couldn't be written.  Hence the Ok response includes the count.
// In the read case we still pass back the count, so the protocol handler gets
// it, but it won't be used.  Instead the number of bytes sent back via the
// bulk IN transfer will signify the number of bytes read, and when it's done.
impl DriverError {
    fn into_usb_transfer_response(self, count: u16) -> UsbTransferResponse {
        match self {
            DriverError::Io
            | DriverError::Timeout
            | DriverError::EarlyExit
            | DriverError::NoDevices
            | DriverError::NoDevice => UsbTransferResponse::Ok(count),
            _ => UsbTransferResponse::Error,
        }
    }
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
    /// The number of bytes actually written or an error.  If an error, the
    /// response includes the error and the number of bytes written
    async fn raw_write(
        &mut self,
        len: u16,
        protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, (DriverError, u16)>;

    /// Read raw bytes from the bus.
    ///
    /// # Arguments
    /// * `read` - The Read objects
    /// * `protocol` - The protocol type
    /// * `write_ep` - The endpoint to write data to
    ///
    /// # Returns
    /// The number of bytes actually read or an error.  If an error, the
    /// response includes the error and the number of bytes read
    async fn raw_read(
        &mut self,
        len: u16,
        protocol: ProtocolType,
    ) -> Result<u16, (DriverError, u16)>;

    /// Wait for a specific bus line to reach the desired state.
    ///
    /// # Arguments
    /// * `line` - The bus line to monitor
    /// * `state` - The state to wait for (typically high/low)
    ///
    /// # Returns
    /// Ok(()) if the line reached the desired state otherwise an error
    async fn wait(
        &mut self,
        line: u8,
        state: u8,
        timeout: Option<Duration>,
    ) -> Result<(), DriverError>;

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

    /// Checks whether the task should abort
    #[inline(always)]
    fn check_abort(&self) -> bool {
        let abort = check_abort();
        if abort {
            warn!("Aborting driver task");
        }
        abort
    }

    /// Write a byte to the parallel port.
    fn write_pp_byte(&mut self, byte: u8);

    /// Read a byte from the parallel port.
    fn read_pp_byte(&mut self) -> u8;
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
    ) -> Result<u16, (DriverError, u16)> {
        match self {
            Driver::Iec(driver) => driver.raw_write(len, protocol, flags).await,
            Driver::Ieee(driver) => driver.raw_write(len, protocol, flags).await,
            Driver::Tape(driver) => driver.raw_write(len, protocol, flags).await,
        }
    }

    async fn raw_read(
        &mut self,
        len: u16,
        protocol: ProtocolType,
    ) -> Result<u16, (DriverError, u16)> {
        match self {
            Driver::Iec(driver) => driver.raw_read(len, protocol).await,
            Driver::Ieee(driver) => driver.raw_read(len, protocol).await,
            Driver::Tape(driver) => driver.raw_read(len, protocol).await,
        }
    }

    async fn wait(
        &mut self,
        line: u8,
        state: u8,
        timeout: Option<Duration>,
    ) -> Result<(), DriverError> {
        match self {
            Driver::Iec(driver) => driver.wait(line, state, timeout).await,
            Driver::Ieee(driver) => driver.wait(line, state, timeout).await,
            Driver::Tape(driver) => driver.wait(line, state, timeout).await,
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

    fn write_pp_byte(&mut self, byte: u8) {
        match self {
            Driver::Iec(driver) => driver.write_pp_byte(byte),
            Driver::Ieee(driver) => driver.write_pp_byte(byte),
            Driver::Tape(driver) => driver.write_pp_byte(byte),
        }
    }
    fn read_pp_byte(&mut self) -> u8 {
        match self {
            Driver::Iec(driver) => driver.read_pp_byte(),
            Driver::Ieee(driver) => driver.read_pp_byte(),
            Driver::Tape(driver) => driver.read_pp_byte(),
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
    trace!("Driver write task started");

    let response = {
        abort();
        let mut guard = DRIVER.lock().await;
        clear_abort();
        let guard = guard.as_mut();

        match guard {
            Some(guard) => {
                // Call raw_write and set the response appropriately.
                let result = guard.raw_write(len, protocol, flags).await;
                match result {
                    Ok(count) => {
                        trace!("Driver write task completed OK: wrote {} bytes", count);
                        UsbTransferResponse::Ok(count)
                    }
                    Err((e, count)) => {
                        info!(
                            "Driver write task completed with error: {}, wrote {} bytes",
                            e, count
                        );
                        e.into_usb_transfer_response(count)
                    }
                }
            }
            None => {
                warn!("No driver available for write task");
                UsbTransferResponse::Error
            }
        }
    };
    // Driver is unlocked here.

    UsbDataTransfer::lock_set_response(response).await;

    if check_abort() {
        warn!("Driver write task was aborted");
        clear_abort();
    }

    trace!("Driver write task exiting");
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
    trace!("Driver read task started");

    let response = {
        abort();
        let mut guard = DRIVER.lock().await;
        clear_abort();
        let guard = guard.as_mut();

        match guard {
            Some(guard) => {
                // Call raw_read and set the response appropriately.
                let result = guard.raw_read(len, protocol).await;

                match result {
                    Ok(count) => {
                        trace!("Driver read task completed OK: read {} bytes", count);
                        UsbTransferResponse::Ok(count)
                    }
                    Err((e, count)) => {
                        info!(
                            "Driver read task completed with error: {}, wrote {} bytes",
                            e, count
                        );
                        e.into_usb_transfer_response(count)
                    }
                }
            }
            None => {
                warn!("No driver available for read task");
                UsbTransferResponse::Error
            }
        }
    };
    // Driver is unlocked here.

    UsbDataTransfer::lock_set_response(response).await;

    if check_abort() {
        warn!("Driver read task was aborted");
        clear_abort();
    }

    trace!("Driver read task exiting");
}

/// Helper function to abort the driver task.
#[inline(always)]
pub fn abort() {
    trace!("Aborting driver task");
    ABORT_DRIVER_TASK.store(true, Ordering::SeqCst);
}

/// Helper function to clear the driver task abort.
#[inline(always)]
pub fn clear_abort() {
    trace!("Deasserting driver task abort");
    ABORT_DRIVER_TASK.store(false, Ordering::SeqCst);
}

/// Helper function to check if the driver task should abort.
#[inline(always)]
pub fn check_abort() -> bool {
    ABORT_DRIVER_TASK.load(Ordering::Relaxed)
}

/// Helper function to check if the driver is currently in use.
#[inline(always)]
pub fn driver_in_use() -> bool {
    DRIVER.try_lock().map_or(true, |d| {
        drop(d);
        false
    })
}
