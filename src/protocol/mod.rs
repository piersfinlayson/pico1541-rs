//! This module implements the Protocol handler, which decoodes and actions
//! Bulk transfers on the OUT endpoint, and returns any requested data on the
//! IN endpoint.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

mod driver;
mod iec;
mod ieee;
mod read;
mod tape;
pub(crate) mod types;
mod write;

use bitflags::bitflags;
use core::sync::atomic::{AtomicBool, Ordering};
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::Spawner;
use embassy_rp::gpio::{Flex, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Endpoint, In};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Instant, Timer};
use embassy_usb::driver::EndpointIn;

use crate::constants::DRIVE_OPERATION_WATCHDOG_TIMER;
use driver::{
    DRIVER, Driver, DriverError, ProtocolDriver, abort, clear_abort, driver_in_use, raw_read_task,
    raw_write_task,
};
use iec::{IecBus, IecDriver, Line};
use types::Direction;

use crate::constants::{
    LOOP_LOG_INTERVAL, MAX_EP_PACKET_SIZE_USIZE, MAX_WRITE_SIZE_USIZE, PROTOCOL_LOOP_TIMER,
    PROTOCOL_WATCHDOG_TIMER, READ_DATA_CHANNEL_SIZE, TOTAL_GENERIC_GPIOS,
    USB_DATA_TRANSFER_WAIT_TIMER,
};
use crate::infra::display::{DisplayType, update_status};
use crate::infra::gpio::{GPIO, IecPinConfig, IeeePinConfig};
use crate::infra::watchdog::{TaskId, WatchdogType};
use crate::usb::WRITE_EP;
use crate::usb::transfer::{USB_DATA_TRANSFER, UsbDataTransfer, UsbTransferResponse};

/// The `PROTOCOL_ACTION` static is a Signal that is used to communicate to the
/// Protocol Handler that a protocol state change is requrested.  We use a
/// `CriticalSectionRawMutex` Signal because the Protocol Handler and Control
/// Handler (which sends signals) run on different cores.
pub static PROTOCOL_ACTION: Signal<CriticalSectionRawMutex, ProtocolAction> = Signal::new();

// Static indicating whether the ProtocolHandler has been initialized yet, or
// not.  Accessed via helper functions
static PROTOCOL_INIT: AtomicBool = AtomicBool::new(false);

// Static indicating whether an IEEE-488 device is present on the bus.  This
// is accessed via helper functions.
static IEEE_PRESENT: AtomicBool = AtomicBool::new(false);

// Static indicating whether a tape device is present.  This is accessed via
// helper functions.
static TAPE_PRESENT: AtomicBool = AtomicBool::new(false);

// A channel for Bulk to send read data from the OUT endpoint into, and then
// send it to the ProtocolHandler.
pub static WRITE_DATA_CHANNEL: Channel<
    ThreadModeRawMutex,
    (usize, [u8; MAX_EP_PACKET_SIZE_USIZE]),
    READ_DATA_CHANNEL_SIZE,
> = Channel::new();

/// `ProtocolAction` is used to flag to the `ProtocolHandler` that there is a
/// state change requested.  This is driven by the Control handler based on
/// Control transfers.
#[derive(PartialEq, Clone)]
pub enum ProtocolAction {
    Initialize,
    Uninitialize,
    Reset,
}

// Implement Format for ProtocolAction so it can be printed with defmt.
impl defmt::Format for ProtocolAction {
    fn format(&self, f: defmt::Formatter) {
        match self {
            ProtocolAction::Initialize => defmt::write!(f, "Initialize"),
            ProtocolAction::Uninitialize => defmt::write!(f, "Uninitialize"),
            ProtocolAction::Reset => defmt::write!(f, "Reset"),
        }
    }
}

/// `ProtocolHandler` handles requests that come in via bulk transfers on the
/// OUT endpoint.
///
/// It is initialized, uninitialized and reset via control requests, and these
/// are signalled via `ProtocolAction`.
#[allow(dead_code)]
pub struct ProtocolHandler {
    // The IN endpoint to use to send status responses and READ data.
    write_ep: Endpoint<'static, USB, In>,

    // Core we are running on
    core: u32,

    // IEC bus pins,
    iec_pins: IecPinConfig,

    // IEEE bus pins
    ieee_pins: IeeePinConfig,

    // The GPIOs.
    gpios: [Option<Flex<'static>>; TOTAL_GENERIC_GPIOS],

    // Watchdog
    watchdog: &'static WatchdogType,
}

impl ProtocolHandler {
    /// Create a new `ProtocolHandler` instance.  Takes an IN (write) endpoint
    /// as argument, so it can send responses directly to the IN endpoint.
    /// The Bulk object retains the OUT (read) endpoint as this must be read
    /// from within the main Bulk future runner (which hands bulk OUT
    /// transfers).
    pub async fn new(
        core: u32,
        write_ep: Endpoint<'static, USB, In>,
        watchdog: &'static WatchdogType,
    ) -> Self {
        // Get the GPIOs for all bus types
        let (gpios, iec_pins, ieee_pins) = Self::get_gpios().await;

        // Set the protocol init static to false
        set_protocol_init(false);

        Self {
            write_ep,
            core,
            iec_pins,
            ieee_pins,
            gpios,
            watchdog,
        }
    }

    // Get the GPIOs that all bus types require from the GPIO object.
    async fn get_gpios() -> (
        [Option<Flex<'static>>; TOTAL_GENERIC_GPIOS],
        IecPinConfig,
        IeeePinConfig,
    ) {
        // Lock the GPIO object
        let mut guard = GPIO.lock().await;
        let gpio = guard.as_mut().expect("GPIO not created");

        // Get the pin numbers for the IEC and IEEE buses.
        let iec_pins = gpio.get_iec_pins();
        let ieee_pins = gpio.get_ieee_pins();

        // Create an array of Option<AnyPin> to hold the GPIOs we get.
        let mut gpios: [Option<Flex<'static>>; TOTAL_GENERIC_GPIOS] = Default::default();

        // Get the pins, accepting that if we've already got the pin, we don't
        // need to (and can't) get it again.
        for pin in iec_pins
            .clone()
            .into_iter()
            .chain(ieee_pins.clone().into_iter())
        {
            if let Some(taken_pin) = gpio.take_flex_pin(pin) {
                gpios[pin as usize] = Some(taken_pin);
            } else if gpios[pin as usize].is_none() {
                defmt::panic!("Pin {} not available", pin);
            }
        }

        (gpios, iec_pins, ieee_pins)
    }

    fn create_iec_driver(&mut self) -> Driver {
        debug!("Creating IEC driver");

        // Create the lines
        let clock_line = Line::new(
            self.iec_pins.clock_in,
            self.gpios[self.iec_pins.clock_in as usize].take().unwrap(),
            self.iec_pins.clock_out,
            self.gpios[self.iec_pins.clock_out as usize].take().unwrap(),
        );
        let data_line = Line::new(
            self.iec_pins.data_in,
            self.gpios[self.iec_pins.data_in as usize].take().unwrap(),
            self.iec_pins.data_out,
            self.gpios[self.iec_pins.data_out as usize].take().unwrap(),
        );
        let atn_line = Line::new(
            self.iec_pins.atn_in,
            self.gpios[self.iec_pins.atn_in as usize].take().unwrap(),
            self.iec_pins.atn_out,
            self.gpios[self.iec_pins.atn_out as usize].take().unwrap(),
        );
        let reset_line = Line::new(
            self.iec_pins.reset_in,
            self.gpios[self.iec_pins.reset_in as usize].take().unwrap(),
            self.iec_pins.reset_out,
            self.gpios[self.iec_pins.reset_out as usize].take().unwrap(),
        );
        let srq_line = Line::new(
            self.iec_pins.srq_in,
            self.gpios[self.iec_pins.srq_in as usize].take().unwrap(),
            self.iec_pins.srq_out,
            self.gpios[self.iec_pins.srq_out as usize].take().unwrap(),
        );

        // Create array of Digital IO pins
        let dios = core::array::from_fn(|ii| Dio {
            pin_num: self.ieee_pins.d_io[ii],
            pin: Some(self.gpios[self.ieee_pins.d_io[ii] as usize].take().unwrap()),
        });

        // Create the bus
        let iec_bus = IecBus::new(clock_line, data_line, atn_line, reset_line, srq_line, dios);

        // Create the IEC driver
        let iec_driver = IecDriver::new(iec_bus, self.watchdog);

        // Test for the IEEE-488 device and tape
        // TODO
        set_ieee_present(false);
        set_tape_present(false);

        // Create the generic driver object
        Driver::Iec(iec_driver)
    }

    fn retrieve_driver_pins(&mut self, driver: &mut Driver) {
        match driver {
            Driver::Iec(iec) => {
                let pins = iec.retrieve_pins();
                for pin in pins {
                    let (input_pin_num, input_pin, output_pin_num, output_pin) = pin;
                    self.gpios[input_pin_num as usize] = input_pin;
                    self.gpios[output_pin_num as usize] = output_pin;
                }

                for dio in &mut iec.retrieve_dios() {
                    let pin_num = dio.pin_num;
                    let pin = dio.pin.take().expect("Missing DIO pin");
                    self.gpios[pin_num as usize] = Some(pin);
                }
            }
            Driver::Ieee(_) => unimplemented!("IEEE driver not implemented"),
            Driver::Tape(_) => unimplemented!("Tape driver not implemented"),
        }
    }

    async fn drop_driver(&mut self) {
        // Lock the driver - this will succeed once any ongoing task has
        // aborted.
        abort();
        let mut guard = DRIVER.lock().await;
        clear_abort();

        // Take the driver, retrieve the pins, then drop it (when it goes out
        // of scope)
        if let Some(mut driver) = guard.take() {
            self.retrieve_driver_pins(&mut driver);
        }
    }

    // Initialize the correct driver type.
    async fn init_driver(&mut self) {
        // Lock the driver - this will succeed once any ongoing task has
        // aborted.
        abort();
        let mut guard = DRIVER.lock().await;
        clear_abort();

        // If we have a driver already we need to remove the pins and drop it.
        if let Some(mut driver) = guard.take() {
            self.retrieve_driver_pins(&mut driver);
        }
        // Existing driver gets dropped here

        // Check which driver type we want to create.
        // TODO

        // Create the appropriate driver
        let driver = self.create_iec_driver();
        *guard = Some(driver);
    }

    // Called from the main runner to perform an action.  This is called every
    // spin of the main runner, and will perform an action if one is
    // waiting.
    //
    // These actions primarily come from the Control object, which sets them
    // in response to Control requests from the host.
    pub async fn perform_action(&mut self) {
        // Check whether there's an action from Control to perform.  This also
        // removes it from the PROTOCOL_ACTION signal.
        let Some(action) = PROTOCOL_ACTION.try_take() else {
            return;
        };

        // If there is one, perform it.
        match action {
            // We can initialize the ProtocolHandler when it's in any state.
            ProtocolAction::Initialize => self.initialize().await,

            // We can unintialize the ProtocolHandler when it's in any state.
            ProtocolAction::Uninitialize => self.uninitialize().await,

            // We can only reset the ProtocolHandler when it's initialized.
            ProtocolAction::Reset => {
                if get_protocol_init() {
                    self.reset().await;
                } else {
                    info!("Received Reset action when not initialized - ignoring");
                }
            }
        }
    }

    /// Called when data has been received on the OUT endpoint.  This function
    /// is the main high-level protocol handler, deciding whether it is the
    /// beginning of a new command, or the continuation of a previous command,
    /// and performing the appropriate actions, specifically:
    ///
    /// * If no command (existing transfer) is underway, treat as a new
    ///   command.  A new command must be 4 bytes long.
    ///   * Byte 0 is the command (0x08 for READ, 0x09 for WRITE)
    ///   * Byte 1 is the protocol (0x10 only supported)
    ///   * Bytes 2 nd 3 contain the data length (for READ and WRITE
    ///     commands), in little endian format.
    ///
    ///   Based on the command, a new transfer of the appropriate (R/W) type
    ///   is created.
    ///  
    /// * If command (transfer) is underway, handles the incoming data as part
    ///   of that transfer.
    ///   * For a WRITE transfer, the data is read into a buffer.  Once all of
    ///     the expected data has been received, a 3 byte Status is returned
    ///     using the IN endpoint, and the current comamnd is cleared.
    ///   * For a READ transfer, the appropriate number of bytes is sent.  No
    ///     status is sent after the successful completion of a READ.
    pub async fn received_data(&mut self, data: &[u8], len: u16) {
        // Check we have a driver.  If not, create one.  This makes it safe to
        // unwrap() self.driver later in this function and sub-functions.
        // We have to "try" this because it is possible there is a transfer
        // outstanding and the driver is locked because of that.)
        if let Ok(true) = Driver::try_is_none() {
            info!("No driver - create it");
            self.init_driver().await;
        }

        // Find out whether we have a transfer underway
        match UsbDataTransfer::lock_direction().await {
            // No transfer is underway - this is the start of a new command.
            None => self.handle_new_command(data, len).await,

            // An Out (write) transfer is underway - handle the incoming data.
            Some(Direction::Out) => self.handle_write_data(data, len).await,

            // An In (read) transfer is underway - ignore the incoming data
            // because only a WRITE or READ can be underway at once.  Because
            // we're here a read is underway, and hence we should be sending
            // data out, not receiving it.
            Some(Direction::In) => {
                warn!("Received data during READ - ignoring");
            }
        }
    }

    // Handles a new command, by creating the appropriate transfer and, in
    // the case of an OUT/write command, sending a response if it fails
    #[allow(clippy::too_many_lines)]
    async fn handle_new_command(&mut self, data: &[u8], len: u16) {
        update_status(DisplayType::Active);

        // Parse the command and get a new Command object.  We handle errors
        // by dropping the request.
        let Ok(command) = Command::new(data, len) else {
            warn!("Failed to parse command - drop it");
            update_status(DisplayType::Error);
            return;
        };

        info!(
            "Received Bulk OUT request: {} 0x{:02x} 0x{:02x} 0x{:02x}",
            command.command, command.bytes[1], command.bytes[2], command.bytes[3]
        );

        // See if the driver is currently locked - this means that a command
        // is in progress, so we can't handle this one.
        if driver_in_use() {
            warn!("Driver in use - drop command {}", command.command);
            update_status(DisplayType::Error);
            return;
        }

        match command.command {
            // Create a new OUT/write transfer.
            CommandType::Write => {
                trace!(
                    "Host to WRITE {} bytes, protocol {}, {}",
                    command.len, command.protocol, command.flags
                );

                if !command.protocol.supported() {
                    // Try to send an error
                    info!("Unsupported protocol {}", command.protocol);
                    self.send_status(Status {
                        code: StatusCode::Error,
                        value: command.len,
                    })
                    .await;
                    update_status(DisplayType::Error);
                    return;
                }

                // TODO Should check command.len < 32KB and send error resonse
                // but that would make this code more complex, and we don't
                // actually care if the len is longer than that.

                UsbDataTransfer::lock_init(Direction::Out, command.protocol, command.len).await;

                // Spawn the transfer
                let spawner = Spawner::for_current_executor().await;
                let result = spawner.spawn(raw_write_task(
                    command.len,
                    command.protocol,
                    command.flags,
                    self.watchdog,
                ));

                if let Err(e) = result {
                    // Failed to spawn the raw_write task - clear down
                    warn!("Failed to spawn raw_write task {}", e);
                    UsbDataTransfer::lock_clear().await;
                    self.send_status(Status {
                        code: StatusCode::Error,
                        value: 0,
                    })
                    .await;
                    update_status(DisplayType::Error);
                }
            }

            // Create a new IN/read transfer.
            CommandType::Read => {
                trace!(
                    "Host to READ {} bytes, protocol {}",
                    command.len, command.protocol
                );

                if !command.protocol.supported() {
                    // Send a zero length packet (we don't send status in
                    // response to a read error).
                    info!("Unsupported protocol {}", command.protocol);
                    let _ = self.write_ep.write(&[]).await;
                    update_status(DisplayType::Error);
                }

                // TODO again, strictly we should check len < 32KB

                UsbDataTransfer::lock_init(Direction::In, command.protocol, command.len).await;

                // Spawn the transfer
                let spawner = Spawner::for_current_executor().await;
                let result =
                    spawner.spawn(raw_read_task(command.protocol, command.len, self.watchdog));

                if let Err(e) = result {
                    // Failed to spawn the raw_read task - clear down.  Not
                    // allowed to send a response.
                    warn!("Failed to spawn raw_read task {}", e);
                    UsbDataTransfer::lock_clear().await;
                    update_status(DisplayType::Error);
                }
            }

            CommandType::GetEoi => {
                trace!("Get EOI");
                let value = DRIVER.lock().await.as_ref().unwrap().get_eoi();
                debug!("Get EOI response: {}", value);
                let value = u16::from(value);
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value,
                })
                .await;
            }
            CommandType::ClearEoi => {
                trace!("Clear EOI");
                DRIVER.lock().await.as_mut().unwrap().clear_eoi();
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value: 0,
                })
                .await;
            }
            CommandType::IecWait => {
                let line = command.bytes[1];
                let state = command.bytes[2];
                trace!("IEC Wait - line(s): 0x{:02x}, state: 0x{:02x}", line, state);
                match DRIVER
                    .lock()
                    .await
                    .as_mut()
                    .unwrap()
                    .wait(line, state, None)
                    .await
                {
                    Ok(()) => {
                        debug!("IEC Wait - success");
                        self.send_status(Status {
                            code: StatusCode::Ok,
                            value: 0,
                        })
                        .await;
                    }
                    Err(e) => {
                        info!("IEC Wait - error: {}", e);
                    }
                }
            }
            CommandType::IecPoll => {
                trace!("IEC Poll");
                let value = DRIVER.lock().await.as_mut().unwrap().poll();
                debug!("IEC Poll response: 0x{:02x}", value);
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value: u16::from(value),
                })
                .await;
            }
            CommandType::IecSetRelease => {
                let set = command.bytes[1];
                let release = command.bytes[2];
                trace!(
                    "IEC Set/Release - set: 0x{:02x}, release: 0x{:02x}",
                    set, release
                );
                DRIVER
                    .lock()
                    .await
                    .as_mut()
                    .unwrap()
                    .setrelease(set, release);
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value: 0,
                })
                .await;
            }
            CommandType::PpRead => {
                trace!("PP Read");
                let byte = DRIVER.lock().await.as_mut().unwrap().read_pp_byte();
                debug!("PP Read byte: 0x{:02x}", byte);
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value: u16::from(byte),
                })
                .await;
            }
            CommandType::PpWrite => {
                let byte = command.bytes[1];
                trace!("PP Write byte: 0x{:02x}", byte);
                DRIVER.lock().await.as_mut().unwrap().write_pp_byte(byte);
                self.send_status(Status {
                    code: StatusCode::Ok,
                    value: 0,
                })
                .await;
            }

            // The rest are unsupported - just drop
            _ => {
                warn!("Unsupported command: {}", command.command);
                update_status(DisplayType::Error);
            }
        }
    }

    // Handles receipt of a chunk of data from the host for a WRITE transfer.
    async fn handle_write_data(&mut self, data: &[u8], len: u16) {
        let mut written: usize = 0;
        loop {
            let result = UsbDataTransfer::lock_try_add_bytes(&data[..len as usize]).await;

            if let Ok(size) = result {
                written += size;
            }

            if written >= (len as usize) {
                break;
            }
            Timer::after(USB_DATA_TRANSFER_WAIT_TIMER).await;
        }
    }

    // (Re-)Initialize the ProtocolHandler.  Any oustanding transfer is
    // cleared.
    async fn initialize(&mut self) {
        debug!("Protocol Handler - initialized");

        if get_protocol_init() {
            // If we receive an initialization request when we're already
            // initialized, that means the previous communication from a host
            // terminated uncleanly (no uninitialize was issued).  Hence we
            // reset the bus in order to clear its state.
            info!("Received initialize request without being shutdown first - reset bus");
            self.reset().await;

            // No need to clear data transfers - reset() does that
        } else {
            set_protocol_init(true);
            UsbDataTransfer::lock_clear().await;
        }

        // (Re-)initialize the driver
        self.init_driver().await;

        // Set the display status
        update_status(DisplayType::Active);
    }

    // Uninitialize the ProtocolHandler.  Any outstanding transfer is cleared.
    async fn uninitialize(&mut self) {
        debug!("Protocol Handler - uninitialized");
        set_protocol_init(false);

        UsbDataTransfer::lock_clear().await;

        self.drop_driver().await;

        update_status(DisplayType::Ready);
    }

    // Reset the ProtocolHandler.  Any outstanding transfer is cleared.
    async fn reset(&mut self) {
        debug!("Protocol Handler - reset");

        // Initialize the driver if not already initialized
        let mut guard = DRIVER.lock().await;
        if guard.is_none() {
            guard.replace(self.create_iec_driver());
        }

        // Resetting involves actual protocol handling, which will feed the
        // Drive Operation watchdog.  Hence deregister the ProtocolHandler, and
        // instead register the Drive Operation.
        self.watchdog
            .deregister_task(&TaskId::ProtocolHandler)
            .await;
        self.watchdog
            .register_task(&TaskId::DriveOperation, DRIVE_OPERATION_WATCHDOG_TIMER)
            .await;

        // Reset the bus
        if let Err(e) = guard.as_mut().unwrap().reset(false).await {
            if e == DriverError::Timeout {
                debug!("Timeout reseting the bus - expected if no devices present");
            } else {
                warn!("Hit error reseting the bus {}", e);
                update_status(DisplayType::Error);
            }
        }

        // Now switch around the watchdog tasks
        self.watchdog.deregister_task(&TaskId::DriveOperation).await;
        self.watchdog
            .register_task(&TaskId::ProtocolHandler, PROTOCOL_WATCHDOG_TIMER)
            .await;

        // Clear out any existing transfers
        UsbDataTransfer::lock_clear().await;
    }

    // Get the appropriate Status for a transfer.  Neither None (no transfer)
    // nor IN (read) transfers return a Status, so those return None.  OUT
    // (write) transfers do, until it isn't finished.
    fn get_status_from_transfer(transfer: &UsbDataTransfer) -> Option<Status> {
        match transfer.direction() {
            // IN/Read doesn't return a Status
            None | Some(Direction::In) => None,

            // OUT/Write
            Some(Direction::Out) => match transfer.get_response() {
                UsbTransferResponse::Ok(bytes) => Some(Status {
                    code: StatusCode::Ok,
                    value: bytes,
                }),
                UsbTransferResponse::Error => Some(Status {
                    code: StatusCode::Error,
                    value: 0,
                }),
                UsbTransferResponse::None => None,
            },
        }
    }

    // See if we have an in-progress transfer, and if so, see if it needs any
    // actioning.
    //
    // Either:
    // - There is no outstanding transfer, in which case we do nothing.
    // - There is an outstanding transfer but it's complete, in which case,
    //   in the OUT case we send a Status back to the host, and in both cases
    //   clear down the transfer
    // - There is an outstanding, incomplete IN transfer, in which case we see
    //   if we need to send any data
    async fn handle_transfer(&mut self) {
        // Lock the data transfer for this function
        let mut guard = USB_DATA_TRANSFER.lock().await;

        match guard.direction() {
            // No transfer is active
            None => (),

            // OUT/write transfer
            Some(Direction::Out) => {
                // An OUT/write transfer - see if it's done.  This is a way
                // of doing so and getting the Status response at the same
                // time.  It's a little wasteful if we end up not sending the
                // status, if not all the write bytes have been received yet,
                // but we don't care because that's an edge case.
                let status = Self::get_status_from_transfer(&guard);

                if let Some(status) = status {
                    // Check to see if there's more bytes to be received
                    // This happens if the device hits an error before
                    // writing all of the bytes.  In which case, we need to
                    // receive the rest of the bytes before we can clear the
                    // transfer.
                    if guard.remaining_bytes() > 0 {
                        // We have more bytes to receive - do nothing
                        return;
                    }

                    // It is done
                    debug!("OUT transfer complete ({})", status.code);

                    // We only send a response on Cbm and Tape protocols
                    let send_response = guard
                        .protocol()
                        .is_some_and(ProtocolType::write_send_status);

                    if send_response {
                        // Send the response
                        debug!("Send OUT transfer status {:?}", status);
                        let result = self.write_ep.write(&status.to_bytes()).await;
                        if let Err(e) = result {
                            warn!("Failed to send status {}", e);
                        }
                    }

                    // Clear the transfer
                    guard.clear();
                }
            }

            // IN/Read transfer
            Some(Direction::In) => {
                // Get the transfer status - we'll use in both steps below
                let response = guard.get_response();

                // An IN/read transfer - first see if there's bytes to send
                let mut outstanding_bytes = guard.outstanding_bytes();

                // If there's as much as an endpoint packet size to send, we
                // will send.  Or, if we're done and there are bytes to send,
                // send them.  If we're done with the transfer and our final
                // buffer was a 64-byte one, or there's no further bytes to
                // send we need to send a zero length packet, so the host
                // knows the transfer is complete.
                let sent = if outstanding_bytes >= MAX_EP_PACKET_SIZE_USIZE
                    || (response != UsbTransferResponse::None && outstanding_bytes > 0)
                {
                    // We either have a full 64 bytes to send, or we're done
                    // so need to send anyway.  We only send a max of 64 bytes
                    // from this function at a time.  We will be rescheduled
                    // to send more, if they are outstanding, soon enough.

                    // Use a temporary buffer to get the bytes to send into.
                    let mut buffer = [0u8; MAX_EP_PACKET_SIZE_USIZE];

                    // Figure out how many bytes to send
                    let bytes_to_send = if outstanding_bytes > MAX_EP_PACKET_SIZE_USIZE {
                        MAX_EP_PACKET_SIZE_USIZE
                    } else {
                        outstanding_bytes
                    };

                    let result = guard.try_get_next_bytes(&mut buffer[..bytes_to_send]);

                    let size = if let Some(size) = result {
                        // Send the bytes
                        let result = self.write_ep.write(&buffer[..size]).await;
                        match result {
                            Ok(()) => {
                                // We sent the bytes
                                debug!("Successfully sent {} bytes", size);

                                // update the outstanding bytes count
                                outstanding_bytes -= size;

                                // We sent this many bytes.
                                size
                            }
                            Err(e) => {
                                // Drop any remaining data
                                warn!("Failed to send IN data {}", e);
                                outstanding_bytes = 0;

                                // Say we sent 0 bytes, in order to get
                                // ZLP sent.
                                0
                            }
                        }
                    } else {
                        // Drop any remaining data
                        warn!("Failed to get bytes to send");
                        outstanding_bytes = 0;

                        // Say we sent 0 bytes in order to get ZLP sent.
                        0
                    };

                    if outstanding_bytes > 0 {
                        // There are still bytes to send - we'll be
                        // rescheduled to send them soon.
                        return;
                    }

                    // Indicate how many bytes we sent
                    size
                } else {
                    // No bytes to send, so indicate we didn't send any.
                    0
                };

                // Now see if the IN/read transfer is done
                if response != UsbTransferResponse::None {
                    // It's done - see how it went
                    debug!("IN transfer complete ({})", response);

                    // Do not send a response in the IN/read case, as one is
                    // not supported by the protocol.  Instead, if the last
                    // data sent wasn't 1-63 bytes, send a zero length packet.
                    if sent == 0 || sent == MAX_EP_PACKET_SIZE_USIZE {
                        let _ = self
                            .write_ep
                            .write(&[])
                            .await
                            .inspect(|()| debug!("ZLP sent"))
                            .inspect_err(|e| warn!("Failed to complete read with ZLP: {}", e));
                    }

                    // Transfer complete - clear it
                    guard.clear();
                }
            }
        }
    }

    // See if there's any data in WRITE_DATA_CHANNEL from Bulk for us to
    // process.
    async fn handle_out_data(&mut self) {
        // See if there's any data on Channel
        if let Ok((size, data)) = WRITE_DATA_CHANNEL.try_receive() {
            trace!("Received data from channel: {:?}", data);
            if size <= MAX_WRITE_SIZE_USIZE {
                // We got bulk data.  Handle it.
                #[allow(clippy::cast_possible_truncation)]
                self.received_data(&data, size as u16).await;
            } else {
                info!(
                    "Received more data than we can handle {} bytes - dropping",
                    size
                );
            }
        } else {
            // No data
        }
    }

    async fn send_status(&mut self, status: Status) {
        if let Err(e) = self.write_ep.write(&status.to_bytes()).await {
            warn!("Failed to send status {}", e);
        }
    }
}

#[embassy_executor::task]
pub async fn protocol_handler_task(watchdog: &'static WatchdogType) -> ! {
    // Read the core ID.  Tasks are allocated to cores at compile time with
    // embassy, so we only need to do this once and store in ProtocolHandler.
    let core = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Protocol Handler task started", core);

    // Create and spawn the ProtocolHandler task.
    let write_ep = WRITE_EP
        .lock()
        .await
        .take()
        .expect("Write endpoint not created");
    let mut protocol_handler = ProtocolHandler::new(core, write_ep, watchdog).await;

    // Register with the watchdog
    let id = TaskId::ProtocolHandler;
    watchdog.register_task(&id, PROTOCOL_WATCHDOG_TIMER).await;

    // We don't want to perform a full ProtocolHandler initialize at this
    // stage, as otherwise, when the user sends an Init control request we'll
    // always reset the bus.  However, we _do_ want to initialize the driver
    // here, as that will detect whether there's an IEEE-488 or tape device
    // present, so that information will be in hand for when an Init control
    // request comes in.
    protocol_handler.init_driver().await;

    // Start the protocol handling loop
    let mut next_log_instant = Instant::now();
    loop {
        let now = Instant::now();
        if now >= next_log_instant {
            trace!("Core{}: Protocol loop", protocol_handler.core);
            next_log_instant += LOOP_LOG_INTERVAL;
        }

        // Feed the watchdog
        watchdog.feed(&id).await;

        // Perform any actions that are waiting.
        protocol_handler.perform_action().await;

        // Handle the in progress transfer if it needs actioning
        protocol_handler.handle_transfer().await;

        // Handle any data read by the USB driver and passed to us.
        protocol_handler.handle_out_data().await;

        // Pause to allow other tasks to run
        Timer::after(PROTOCOL_LOOP_TIMER).await;
    }
}

/// Helper function to get `ProtocolHandler` state - required by Control
/// Handler to determine what to indicate in resonse to an Init command.
pub fn get_protocol_init() -> bool {
    PROTOCOL_INIT.load(Ordering::Relaxed)
}

// Helper function to set ProtocolHandler state - used internally to set the
// initialized status.
fn set_protocol_init(value: bool) {
    PROTOCOL_INIT.store(value, Ordering::Relaxed);
}

/// Helper function to determine if an IEEE-488 device is present.  This is
/// used by the Control Handler indicate in response to an Init command.
pub fn get_ieee_present() -> bool {
    IEEE_PRESENT.load(Ordering::Relaxed)
}

// Helper function to set the IEEE-488 device present status.  Used internally
// to set the IEEE-488 device present status.
fn set_ieee_present(value: bool) {
    IEEE_PRESENT.store(value, Ordering::Relaxed);
}

/// Helper function to determine if a tape device is present.  This is used by
/// the Control Handler indicate in response to an Init command.
pub fn get_tape_present() -> bool {
    TAPE_PRESENT.load(Ordering::Relaxed)
}

// Helper function to set the tape device present status.  Used internally to
// set the tape device present status.
fn set_tape_present(value: bool) {
    TAPE_PRESENT.store(value, Ordering::Relaxed);
}

/// Helper function to signal to the `ProtocolHandler` that an action is
/// required.  This is used by the Control Handler.
pub fn set_protocol_action(action: ProtocolAction) {
    PROTOCOL_ACTION.signal(action);
}

/// Helper function to retrieve a signal for the `ProtocolHandler`.  This is
/// used internally, and by the Control Handler to retrieve an action.
///
/// This takes the current signal, so a new signal must be set if required.
pub fn take_protocol_action() -> Option<ProtocolAction> {
    PROTOCOL_ACTION.try_take()
}

// The type of Command received from the host.
#[derive(Debug, PartialEq)]
pub enum CommandType {
    // The host is requesting data from the device.
    Read = 0x08,

    // The host is sending data to the device.
    Write = 0x09,

    GetEoi = 23,
    ClearEoi = 24,
    PpRead = 25,
    PpWrite = 26,
    IecPoll = 27,
    IecWait = 28,
    IecSetRelease = 29,
    ParburstRead = 30,
    ParburstWrite = 31,
    SrqburstRead = 32,
    SrqburstWrite = 33,
    TapMotorOn = 66,
    TapGetVer = 67,
    TapPrepareCapture = 68,
    TapPrepareWrite = 69,
    TapGetSense = 70,
    TapWaitForStopSense = 71,
    TapWaitForPlaySense = 72,
    TapMotorOff = 73,
}

// Implement TryFrom for CommandType so we can parse an imcoming byte into a
// CommandType.
impl TryFrom<u8> for CommandType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x08 => Ok(Self::Read),
            0x09 => Ok(Self::Write),
            23 => Ok(Self::GetEoi),
            24 => Ok(Self::ClearEoi),
            25 => Ok(Self::PpRead),
            26 => Ok(Self::PpWrite),
            27 => Ok(Self::IecPoll),
            28 => Ok(Self::IecWait),
            29 => Ok(Self::IecSetRelease),
            30 => Ok(Self::ParburstRead),
            31 => Ok(Self::ParburstWrite),
            32 => Ok(Self::SrqburstRead),
            33 => Ok(Self::SrqburstWrite),
            66 => Ok(Self::TapMotorOn),
            67 => Ok(Self::TapGetVer),
            68 => Ok(Self::TapPrepareCapture),
            69 => Ok(Self::TapPrepareWrite),
            70 => Ok(Self::TapGetSense),
            71 => Ok(Self::TapWaitForStopSense),
            72 => Ok(Self::TapWaitForPlaySense),
            73 => Ok(Self::TapMotorOff),
            _ => Err(()),
        }
    }
}

impl CommandType {
    // Whether the command is asynchronous.
    #[allow(dead_code)]
    fn is_async(&self) -> bool {
        matches!(self, Self::IecWait)
    }
}

impl defmt::Format for CommandType {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            CommandType::Read => defmt::write!(fmt, "Read"),
            CommandType::Write => defmt::write!(fmt, "Write"),
            CommandType::GetEoi => defmt::write!(fmt, "GetEoi"),
            CommandType::ClearEoi => defmt::write!(fmt, "ClearEoi"),
            CommandType::PpRead => defmt::write!(fmt, "PpRead"),
            CommandType::PpWrite => defmt::write!(fmt, "PpWrite"),
            CommandType::IecPoll => defmt::write!(fmt, "IecPoll"),
            CommandType::IecWait => defmt::write!(fmt, "IecWait"),
            CommandType::IecSetRelease => defmt::write!(fmt, "IecSetRelease"),
            CommandType::ParburstRead => defmt::write!(fmt, "ParburstRead"),
            CommandType::ParburstWrite => defmt::write!(fmt, "ParburstWrite"),
            CommandType::SrqburstRead => defmt::write!(fmt, "SrqburstRead"),
            CommandType::SrqburstWrite => defmt::write!(fmt, "SrqburstWrite"),
            CommandType::TapMotorOn => defmt::write!(fmt, "TapMotorOn"),
            CommandType::TapGetVer => defmt::write!(fmt, "TapGetVer"),
            CommandType::TapPrepareCapture => defmt::write!(fmt, "TapPrepareCapture"),
            CommandType::TapPrepareWrite => defmt::write!(fmt, "TapPrepareWrite"),
            CommandType::TapGetSense => defmt::write!(fmt, "TapGetSense"),
            CommandType::TapWaitForStopSense => defmt::write!(fmt, "TapWaitForStopSense"),
            CommandType::TapWaitForPlaySense => defmt::write!(fmt, "TapWaitForPlaySense"),
            CommandType::TapMotorOff => defmt::write!(fmt, "TapMotorOff"),
        }
    }
}

/// A Command object is created from the incoming Bulk data, and used to
/// determine the type of command, the protocol, and the length of any data
/// associated with the command.
pub struct Command {
    // The type of command.  This is the first byte of the incoming command
    // packet.
    #[allow(clippy::struct_field_names)]
    command: CommandType,

    // The protocol used by the command.  This is the second byte of the
    // command packet.
    protocol: ProtocolType,

    // Any protocol flags - these are only used when ProtocolType is Cbm
    flags: ProtocolFlags,

    // The length of any data associated with the command.  This is encoded
    // in the third and fourth bytes of the command packet as a little endian
    // u16.
    len: u16,

    // Store the third byte off the actual bytes
    bytes: [u8; Command::LEN as usize],
}

impl Command {
    /// The length
    const LEN: u16 = 4;

    /// Create a new Command object from an incoming data packet.
    ///
    /// This function checks the data length and contents are valid, and, if
    /// so, return a Command object.
    pub fn new(bytes: &[u8], size: u16) -> Result<Self, ()> {
        // Check the data size is the length of a command.
        if size != Self::LEN {
            info!("Unexpected data length: {}", size);
            return Err(());
        }

        // Check the buffer has enough bytes in it.
        if bytes.len() < size as usize {
            info!("Too few bytes to parse command: {} < {}", bytes.len(), size);
            return Err(());
        }

        // Get the command itself.
        let Ok(command) = CommandType::try_from(bytes[0]) else {
            info!("Unsupported command: 0x{:02x}", bytes[0]);
            return Err(());
        };

        // Check the protocol is supported (for Read and Write commands only).
        let protocol = if command == CommandType::Read || command == CommandType::Write {
            ProtocolType::try_from(bytes[1]).map_err(|()| {
                info!("Unsupported protocol: 0x{:02x}", bytes[1]);
            })?
        } else {
            ProtocolType::None
        };

        // Check the flags are supported (for Read and Write Commands only).
        let flags = if command == CommandType::Read || command == CommandType::Write {
            let flags = ProtocolFlags::try_from(bytes[1]).map_err(|()| {
                info!("Unsupported flags: 0x{:02x}", bytes[1]);
            })?;

            if flags.is_any() && protocol != ProtocolType::Cbm {
                return Err(());
            }

            flags
        } else {
            ProtocolFlags::NONE
        };

        // Get the length of any data associated with this command.
        let len = u16::from_le_bytes([bytes[2], bytes[3]]);

        // Return the new Command object.
        Ok(Self {
            command,
            protocol,
            flags,
            len,
            bytes: [bytes[0], bytes[1], bytes[2], bytes[3]],
        })
    }
}

/// Supported Bulk command protocols.
#[repr(u8)]
#[derive(Clone, Copy, defmt::Format, PartialEq)]
pub enum ProtocolType {
    None = 0x00,
    Cbm = 0x10,
    S1 = 0x20,
    S2 = 0x30,
    PP = 0x40,
    P2 = 0x50,
    Nib = 0x60,
    NibCommand = 0x70,
    NibSrq = 0x80,
    NibSrqCommand = 0x90,
    Tap = 0xA0,
    TapConfig = 0xB0,
}

impl ProtocolType {
    /// Whether the Protocol is supported
    fn supported(self) -> bool {
        matches!(self, Self::Cbm | Self::S1 | Self::S2 | Self::PP | Self::P2)
    }

    fn write_send_status(self) -> bool {
        self == Self::Cbm || self == Self::Tap || self == Self::TapConfig
    }
}

/// Implement `TryFrom` for `ProtocolType` in order to parse an incoming byte
/// into a `ProtocolType`.
impl TryFrom<u8> for ProtocolType {
    type Error = ();

    // Protocol is the high order nibble of the second byte of the command
    // packet.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let check_value = value & 0xF0;
        match check_value {
            0x10 => Ok(Self::Cbm),
            0x20 => Ok(Self::S1),
            0x30 => Ok(Self::S2),
            0x40 => Ok(Self::PP),
            0x50 => Ok(Self::P2),
            0x60 => Ok(Self::Nib),
            0x70 => Ok(Self::NibCommand),
            0x80 => Ok(Self::NibSrq),
            0x90 => Ok(Self::NibSrqCommand),
            0xA0 => Ok(Self::Tap),
            0xB0 => Ok(Self::TapConfig),
            _ => Err(()),
        }
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Clone, Copy)]
    pub struct ProtocolFlags: u8 {
        const NONE = 0x00;
        const CBM_TALK = 0x01;
        const CBM_ATN = 0x02;
    }
}

// Implement defmt::Format manually for ProtocolFlags
impl defmt::Format for ProtocolFlags {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ProtocolFlags({})", self.bits());
    }
}

impl ProtocolFlags {
    pub fn is_none(self) -> bool {
        self.is_empty()
    }

    pub fn is_any(self) -> bool {
        !self.is_none()
    }

    pub fn is_talk(self) -> bool {
        self.contains(Self::CBM_TALK)
    }

    pub fn is_atn(self) -> bool {
        self.contains(Self::CBM_ATN)
    }
}

impl TryFrom<u8> for ProtocolFlags {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        let check_value = value & 0x0F;
        ProtocolFlags::from_bits(check_value).ok_or(())
    }
}

/// The status code used in the response to certain terns of Bulk Commands.
#[repr(u8)]
#[derive(Clone, defmt::Format)]
#[allow(dead_code)]
enum StatusCode {
    /// The device is busy.  The host may retry later.
    Busy = 0x01,

    /// The command was successful.
    Ok = 0x02,

    /// The command failed, or was invalid.
    Error = 0x03,
}

/// A Status object is used to return a status response to the host after
/// certin commands have been received and handled.  It is returned as
/// stream of bytes on the IN endpoint.
struct Status {
    /// The status code to return.
    code: StatusCode,

    /// The value to return.  This is typically the number of bytes
    /// processed, or an error code.
    value: u16,
}

impl Status {
    /// Convert the Status object into a stream of bytes to be sent on the IN
    /// endpoint.
    fn to_bytes(&self) -> [u8; 3] {
        let mut bytes = [0; 3];
        bytes[0] = self.code.clone() as u8;
        bytes[1..3].copy_from_slice(&self.value.to_le_bytes());
        bytes
    }
}

impl defmt::Format for Status {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "Status({:?}, 0x{:04x})", self.code, self.value);
    }
}

pub struct Dio {
    pub pin_num: u8,
    pub pin: Option<Flex<'static>>,
}

impl Dio {
    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn set_output(&mut self, level: bool) {
        if let Some(pin) = &mut self.pin {
            pin.set_as_output();
            if level {
                pin.set_high();
            } else {
                pin.set_low();
            }
        } else {
            warn!("Pin {} not set", self.pin_num);
        }
    }

    #[allow(clippy::inline_always)]
    #[inline(always)]
    pub fn read_pin(&mut self) -> bool {
        if let Some(pin) = &mut self.pin {
            pin.set_as_input();
            pin.set_pull(Pull::None);
            pin.is_high()
        } else {
            warn!("Pin {} not set", self.pin_num);
            false
        }
    }
}
