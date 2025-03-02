//! This module implements the Protocol handler, which decoodes and actions
//! Bulk transfers on the OUT endpoint, and returns any requested data on the
//! IN endpoint.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

use bitflags::bitflags;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Endpoint, In};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embassy_usb::driver::EndpointIn;
use heapless::Vec;

use crate::constants::{
    LOOP_LOG_INTERVAL, MAX_EP_PACKET_SIZE_USIZE, MAX_READ_SIZE, MAX_WRITE_SIZE,
    MAX_WRITE_SIZE_USIZE, PROTOCOL_WATCHDOG_TIMER, READ_DATA_CHANNEL_SIZE,
};
use crate::display::{update_status, DisplayType};
use crate::driver::{DriverError, ProtocolDriver};
use crate::iec::{IecBus, IecDriver};
use crate::watchdog::{feed_watchdog, register_task, TaskId};

// The PROTOCOL_ACTION static is a Signal that is used to communicate to the
// Protocol Handler that a protocol state change is requrested.  We use a
// CriticalSectionRawMutex Signal because the Protocol Handler and Control
// Handler (which sends signals) run on different cores.
pub static PROTOCOL_ACTION: Signal<CriticalSectionRawMutex, ProtocolAction> = Signal::new();

// We need a static for the write data we may be asked to read in.  The
// maximum size of a data transfer supported by the xum1541 is 32768 bytes,
// which is the MAX_WRITE_SIZE_USIZE.  So, we can't possibly allocate this on
// the stack, or even on the heap, so we have a static, which should be OK as
// the Pico has 264KB RAM.  However, we need to be careful about accessing
// this.
//
// TODO - should probably move to using much, much smaller buffers and then
// it's up to the Protocol handling code to read it out of the buffer in a
// timely enough fashion.
static mut WRITE_DATA: Vec<u8, MAX_WRITE_SIZE_USIZE> = Vec::new();

// A channel for Bulk to send read data from the OUT endpoint into, and then
// send it to the ProtocolHandler.
pub static WRITE_DATA_CHANNEL: Channel<
    ThreadModeRawMutex,
    (usize, [u8; MAX_EP_PACKET_SIZE_USIZE]),
    READ_DATA_CHANNEL_SIZE,
> = Channel::new();

// ProtocolState is used by ProtocolHandler to store its state.
#[derive(PartialEq)]
enum ProtocolState {
    // The protcocol handler is initialized, and will accept Bulk transfers.
    Initialized,

    // The protocol handler is uninitialized, and will not accept Bulk
    // transfers.
    Uninitialized,
}

// Implement Format for ProtocolState so it can be printed with defmt.
impl defmt::Format for ProtocolState {
    fn format(&self, f: defmt::Formatter) {
        match self {
            ProtocolState::Initialized => defmt::write!(f, "Initialized"),
            ProtocolState::Uninitialized => defmt::write!(f, "Uninitialized"),
        }
    }
}

/// ProtocolAction is used to flag to the ProtocolHandler that there is a
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

// A Transfer operation.  This is a read or write of data using a Bulk
// transfer and is triggerd by a Bulk command from the Host on the OUT
// endoint.  Writes also take take on the OUT endpoint (as they are host to
// device).  Reads are sent on the IN endpoint (as they are device to host).
#[allow(clippy::large_enum_variant)]
enum Transfer {
    #[allow(dead_code)]
    Read(Read),
    Write(Write),
}

// A Write operation.  This is created when a WRITE command is received via
// the bulk OUT endpoint, used to manage the reception of the data to be
// written, and create the Status to be returned after data reception.
struct Write {
    // Number of bytes of data we are expecting the host to write to us,
    // (and we will not process any other protocol commands until that comes
    // in).
    expected_len: u16,

    // Number of bytes of write data we've received so far, and are stored
    // in data.
    received_bytes: u16,

    // Protocol to use.
    protocol: ProtocolType,

    // Protocol flags.
    flags: ProtocolFlags,

    // A buffer to hold all of the data to be written.  As we are running on
    // an embedded device, and we are no_std, we are using heapless::Vec.
    // So long as Write is within Transfer, which is within ProtocolHandler,
    // which is within Bulk, and so long as Bulk is in a StaticCell, then
    // this Vec is allocated statically.
    data: &'static mut Vec<u8, MAX_WRITE_SIZE_USIZE>,
}

impl Write {
    // Create a new Write transfer, with a buffer large enough to store all
    // possible incoming data.
    fn new(command: Command) -> Result<Self, Status> {
        if command.len < MAX_WRITE_SIZE {
            // This should be safe, as Bulk and Protocol are the only objects
            // that access data, and they run within a single thread, on core
            // 1.  Nothing else can access it.  The only functionality that
            // operates in parallel, on core 1, is an attempt to read data
            // from USB into a separate buffer.
            unsafe {
                #[allow(static_mut_refs)]
                Ok(Self {
                    expected_len: command.len,
                    received_bytes: 0,
                    protocol: command.protocol,
                    flags: command.flags,
                    data: &mut WRITE_DATA,
                })
            }
        } else {
            info!("Received WRITE command for too many bytes: {}", command.len);
            Err(Status {
                code: StatusCode::Error,
                value: command.len,
            })
        }
    }

    // Called when data has been received from the OUT endpoint for this
    // transfer.  Stores the data in the buffer.
    fn receive_data(&mut self, data: &[u8], len: u16) -> Result<(), ()> {
        if self.received_bytes + len > self.expected_len {
            return Err(());
        }

        // We know this can't fail as expected_len <= MAX_WRITE_SIZE
        let _ = self.data.extend_from_slice(&data[..(len as usize)]);
        self.received_bytes += len;
        Ok(())
    }

    // Called to find out if the WRITE operation has completed yet.  If so a
    // Some(Status) is returned, which can then be used to construct and send
    // a status response.
    fn complete(&self) -> Option<Status> {
        if self.received_bytes >= self.expected_len {
            Some(Status {
                code: StatusCode::Ok,
                value: self.received_bytes,
            })
        } else {
            None
        }
    }
}

// A Read operation.  This is created when a READ command is received via
// the bulk OUT endpoint, used to manage the sending of the data to the IN
// endpoint.
pub struct Read {
    // Number of bytes the host is expecting us to send.  The transfer is not
    // complete until we have sent this number of bytes (or the device is
    // reset, uninitialized, or initialized.
    pub len: u16,

    // Protocol to use
    protocol: ProtocolType,
}

impl Read {
    // Create a new Read transfer.  As this example implementation sends dummy
    // data we refill the buffer with all 'x's.  In a real implementation this
    // would need to be enhanced.
    fn new(command: Command) -> Result<Self, ()> {
        if command.len > MAX_READ_SIZE {
            info!("Received READ command for too many bytes: {}", command.len);
            return Err(());
        }

        if command.flags.is_any() {
            info!("Unsupported flags on read: {:?}", command.flags);
            return Err(());
        }

        Ok(Self {
            len: command.len,
            protocol: command.protocol,
        })
    }
}

/// ProtocolHandler handles requests that come in via bulk transfers on the
/// OUT endpoint.
///
/// It is initialized, uninitialized and reset via control requests, and these
/// are signalled via ProtocolAction.
#[allow(dead_code)]
pub struct ProtocolHandler {
    // The state of the ProtocolHandler.
    state: ProtocolState,

    // The IN endpoint to use to send status responses and READ data.
    write_ep: Endpoint<'static, USB, In>,

    // The current transfer operation.  This is either a Read or Write
    // operation, and is set when a new command is received, and cleared when
    // the operation is complete, or `Self::state` is changed.
    transfer: Option<Transfer>,

    // The IEC Driver.
    iec_driver: IecDriver,

    // Core we are running on
    core: u32,
}

impl ProtocolHandler {
    /// Create a new ProtocolHandler instance.  Takes an IN (write) endpoint
    /// as argument, so it can send responses directly to the IN endpoint.
    /// The Bulk object retains the OUT (read) endpoint as this must be read
    /// from within the main Bulk future runner (which hands bulk OUT
    /// transfers).
    pub fn new(core: u32, write_ep: Endpoint<'static, USB, In>, iec_bus: IecBus) -> Self {
        Self {
            state: ProtocolState::Uninitialized,
            write_ep,
            transfer: None,
            iec_driver: IecDriver::new(iec_bus),
            core,
        }
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
        let action = match PROTOCOL_ACTION.try_take() {
            Some(action) => action,
            None => {
                // No action to perform
                return;
            }
        };

        // If there is one, perform it.
        match action {
            // We can initialize the ProtocolHandler when it's in any state.
            ProtocolAction::Initialize => self.initialize(),

            // We can unintialize the ProtocolHandler when it's in any state.
            ProtocolAction::Uninitialize => self.uninitialize(),

            // We can only reset the ProtocolHandler when it's initialized.
            ProtocolAction::Reset => {
                if self.state == ProtocolState::Initialized {
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
        // Take the transfer out here, so we don't borrow self mutably in
        // the match.  We have to put it back again, if it was Some(Transfer)
        // and it hasn't been dropped, but we will let the match arm function
        // handle that.
        let transfer = self.transfer.take();
        match transfer {
            // No transfer is underway - this is the start of a new command.
            None => {
                // Parse and handle the new command.
                self.handle_new_command(data, len).await;
            }

            // A WRITE transfer is underway - handle the incoming data.
            Some(Transfer::Write(write)) => {
                // Put the transfer back in self before calling the function
                // to handle the write data, so the function can access it
                // mutably.
                self.transfer = Some(Transfer::Write(write));
                self.handle_write_data(data, len).await;
            }

            // A READ transfer is underway - ignore the incoming data because
            // only a WRITE or READ can be underway at once.  Because we're
            // here a READ is outstanding, and hence we should be sending data
            // out, not receiving it.
            Some(Transfer::Read(_)) => {
                // We're just ignoring it when we receive (WRITE) data during
                // a READ transfer, and we consider the READ transfer ongoing.
                // Hence we put tranfer back in self.
                info!("Received data during READ - ignoring");
                self.transfer = transfer;
            }
        }
    }

    // Handles a new command, by creating the appropriate transfer and, in
    // the case of a WRITE command, sending a response if it fails
    async fn handle_new_command(&mut self, data: &[u8], len: u16) {
        update_status(DisplayType::Active);

        // Parse the command and get a new Command object.  We handle errors
        // by dropping the request.
        let command = match Command::new(data, len) {
            Ok(command) => command,
            Err(_) => {
                info!("Failed to parse command - drop it");
                update_status(DisplayType::Error);
                return;
            }
        };

        if !command.protocol.supported() {
            info!("Unsupported protocol: {:?}", command.protocol);
            match command.command {
                CommandType::Read => {
                    // Drop the request as we aren't allow to send a status
                }
                CommandType::Write => {
                    // Try to send an error - although we can't be explicit
                    // about what error we hit as the protocol doesn't allow
                    // it
                    match self
                        .write_ep
                        .write(
                            &Status {
                                code: StatusCode::Error,
                                value: command.len,
                            }
                            .to_bytes(),
                        )
                        .await
                    {
                        Ok(_) => info!("Failed to handle WRITE - sent error response"),
                        Err(_) => info!("Failed to send WRITE error response"),
                    }
                }
                _ => {
                    // The rest are unsupported - just drop
                }
            };
            update_status(DisplayType::Error);
            return;
        }

        match command.command {
            // Create a new Write transfer.  If the creation fails, we will
            // attempt to send a status response.
            CommandType::Write => {
                info!("Host to WRITE {} bytes", command.len);
                match Write::new(command) {
                    Ok(write) => self.transfer = Some(Transfer::Write(write)),
                    Err(status) => {
                        update_status(DisplayType::Error);
                        match self.write_ep.write(&status.to_bytes()).await {
                            Ok(_) => info!("Failed to handle WRITE - sent error response"),
                            Err(_) => info!("Failed to send WRITE error response"),
                        }
                    }
                }
            }
            // Create a new Read transfer.  If the creation fails, we silently
            // drop the request, as Read commands do no support a status
            // response.
            CommandType::Read => {
                info!("Host to READ {} bytes", command.len);
                match Read::new(command) {
                    Ok(read) => {
                        let proto = read.protocol.clone();
                        let len = read.len;
                        self.transfer = Some(Transfer::Read(read));

                        let result = match proto {
                            ProtocolType::Cbm => {
                                self.iec_driver.raw_read(len, &mut self.write_ep).await
                            }
                            _ => {
                                error!("Unsupported read protocol: {:?}", proto);
                                update_status(DisplayType::Error);
                                Err(DriverError::Unsupported)
                            }
                        };

                        match result {
                            Ok(_) => info!("Sent READ response"),
                            Err(_) => {
                                info!("Failed to send READ response");
                                update_status(DisplayType::Error);
                            }
                        }

                        self.transfer = None;
                    }
                    Err(_) => {
                        info!("Failed to handle READ - drop it");
                        update_status(DisplayType::Error);
                    }
                }
            }
            // The rest are unsupported - just drop
            _ => (),
        }
    }

    // Handles receipt of a chunk of data from the host for a WRITE transfer.
    // This function demands that self.transfer is of type Transfer::Write.
    async fn handle_write_data(&mut self, data: &[u8], len: u16) {
        let transfer = self
            .transfer
            .as_mut()
            .expect("Internal error - expected WRITE transfer");
        let write = match transfer {
            Transfer::Write(write) => write,
            _ => unreachable!(),
        };

        // Handle the incoming data using the Transfer object.
        match write.receive_data(data, len) {
            Ok(_) => {
                if let Some(status) = write.complete() {
                    // Write complete
                    info!("WRITE complete");

                    // Execute the write using the correct protocol
                    let result = match write.protocol {
                        ProtocolType::Cbm => {
                            self.iec_driver.raw_write(write.data, write.flags).await
                        }
                        _ => {
                            error!("Unsupported protocol: {}", write.protocol);
                            update_status(DisplayType::Error);
                            Err(DriverError::Unsupported)
                        }
                    };

                    // Set up the correct status response
                    let status = match result {
                        Ok(bytes) => {
                            debug!("Sent WRITE response: {} bytes", bytes);
                            status
                        }
                        Err(e) => {
                            info!("Failed to handle WRITE {}", e);
                            update_status(DisplayType::Error);
                            Status {
                                code: StatusCode::Error,
                                value: 0,
                            }
                        }
                    };

                    // Send the status response.
                    match self.write_ep.write(&status.to_bytes()).await {
                        Ok(_) => info!("Sent WRITE response"),
                        Err(e) => error!("Failed to send WRITE response {}", e),
                    }

                    self.transfer = None
                } else {
                    // Incomplete - leave transfer in place.
                }
            }
            Err(_) => {
                // The only error from write.receive() is if too much data
                // was received.
                info!("Too much data received - dropping WRITE");
                self.transfer = None;
            }
        }
    }

    // (Re-)Initialize the ProtocolHandler.  Any oustanding transfer is
    // cleared.
    fn initialize(&mut self) {
        info!("Protocol Handler - initialized");
        self.state = ProtocolState::Initialized;
        self.transfer = None
    }

    // Uninitialize the ProtocolHandler.  Any outstanding transfer is cleared.
    fn uninitialize(&mut self) {
        info!("Protocol Handler - uninitialized");
        self.state = ProtocolState::Uninitialized;
        self.transfer = None
    }

    // Reset the ProtocolHandler.  Any outstanding transfer is cleared.
    async fn reset(&mut self) {
        info!("Protocol Handler - reset");
        if let Err(e) = self.iec_driver.reset(false).await {
            info!("Hit error reseting the bus {}", e);
        }
        self.transfer = None
    }

    // See if there's any data in WRITE_DATA_CHANNEL from Bulk for us to
    // process.
    // TO DO - don't just read all written data into a massive WRITE buffer.
    // Instead keep it close by ready to feed to IecDriver when it asks for
    // it. 
    async fn handle_data(&mut self) {
        // See if there's any data on Channel
        match WRITE_DATA_CHANNEL.try_receive() {
            Ok((size, data)) => {
                info!("Received data from channel: {:?}", data);
                if size <= MAX_WRITE_SIZE_USIZE {
                    // We got bulk data.  Handle it.
                    self.received_data(&data, size as u16).await;
                } else {
                    info!(
                        "Received more data than we can handle {} bytes - dropping",
                        size
                    );
                }
            }
            Err(_) => {
                // No data
            }
        }
    }
}

#[embassy_executor::task]
pub async fn protocol_handler_task(iec_bus: IecBus, write_ep: Endpoint<'static, USB, In>) -> ! {
    // Read the core ID.  Tasks are allocated to cores at compile time with
    // embassy, so we only need to do this once and store in ProtocolHandler.
    let core = embassy_rp::pac::SIO.cpuid().read();

    // Create and spawn the ProtocolHandler task.
    let mut protocol_handler = ProtocolHandler::new(core, write_ep, iec_bus);

    // Register with the watchdog
    register_task(TaskId::ProtocolHandler, PROTOCOL_WATCHDOG_TIMER);

    let mut next_log_instant = Instant::now();

    loop {
        let now = Instant::now();
        if now >= next_log_instant {
            info!("Core{}: Protocol loop", protocol_handler.core);
            next_log_instant += LOOP_LOG_INTERVAL;
        }

        // Feed the watchdog
        feed_watchdog(TaskId::ProtocolHandler);

        // Perform any actions that are waiting.
        protocol_handler.perform_action().await;

        // Handle any data read in by the USB driver.
        protocol_handler.handle_data().await;
    }
}

// The type of Command received from the host.
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

/// A Command object is created from the incoming Bulk data, and used to
/// determine the type of command, the protocol, and the length of any data
/// associated with the command.
pub struct Command {
    // The type of command.  This is the first byte of the incoming command
    // packet.
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
        let command = match CommandType::try_from(bytes[0]) {
            Ok(cmd) => cmd,
            Err(_) => {
                info!("Unsupported command: 0x{:02x}", bytes[0]);
                return Err(());
            }
        };

        // Check the protocol is supported.
        let protocol = match ProtocolType::try_from(bytes[1]) {
            Ok(proto) => proto,
            Err(_) => {
                info!("Unsupported protocol: 0x{:02x}", bytes[1]);
                return Err(());
            }
        };

        // Check the flags are supported.
        let flags = match ProtocolFlags::try_from(bytes[1]) {
            Ok(flags) => {
                if flags.is_any() && protocol != ProtocolType::Cbm {
                    return Err(());
                }
                flags
            }
            Err(_) => {
                info!("Unsupported flags: 0x{:02x}", bytes[1]);
                return Err(());
            }
        };

        // Get the length of any data associated with this command.
        let len = u16::from_le_bytes([bytes[2], bytes[3]]);

        // Return the new Command object.
        Ok(Self {
            command,
            protocol,
            flags,
            len,
        })
    }
}

/// Supported Bulk command protocols.
#[repr(u8)]
#[derive(Clone, defmt::Format, PartialEq)]
enum ProtocolType {
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
    fn supported(&self) -> bool {
        matches!(self, Self::Cbm)
    }
}

/// Implement TryFrom for ProtocolType in order to parse an incoming byte into
/// a ProtocolType.
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
        defmt::write!(f, "ProtocolFlags({})", self.bits())
    }
}

impl ProtocolFlags {
    pub fn is_none(&self) -> bool {
        self.is_empty()
    }

    pub fn is_any(&self) -> bool {
        !self.is_none()
    }

    pub fn is_talk(&self) -> bool {
        self.contains(Self::CBM_TALK)
    }

    pub fn is_atn(&self) -> bool {
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
