//! This module implements the USB Control handler.
//!
//! This includes
//! * USB lifecycle events (enabled, disabled, reset, addressed, configured,
//!   deconfigured, suspended and resumed)
//! * Control requests (IN and OUT) from the host
//!
//! IN Control requests are those where the host is expecting data from th
//! device.  OUT Control requests are those where the host may send data to
//! the device.  In all cases, the requests is initiated by the host.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

use embassy_usb::Handler;
use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::types::InterfaceNumber;
use static_cell::StaticCell;

use crate::constants::FIRMWARE_VERSION;
use crate::constants::{
    ECHO_CONTROL_RESPONSE_LEN, INIT_CONTROL_RESPONSE_LEN, MAX_XUM_DEVINFO_SIZE_USIZE,
};
use crate::infra::display::{DisplayType, update_status};
use crate::infra::watchdog::WatchdogType;
use crate::protocol::types::{Capabilities, Direction, InitStatus};
use crate::protocol::{
    ProtocolAction, get_ieee_present, get_protocol_init, get_tape_present, set_protocol_action,
    take_protocol_action,
};
use crate::util::built::{GIT_VERSION, PKG_VERSION, RUSTC_VERSION};

// Our Control Handler handles Control requests that come in on the Control
// endpoint, and the USB stack calls control_in() and control_out() for us
// to handle them.  It also handles various USB device lifecycles events, such
// as enabled, reset, address, configured, etc.  We pass ownership of this to
// our UsbDevice object, so we don't need anything other than a StaticCell.
static CONTROL: StaticCell<Control> = StaticCell::new();

/// Handle USB events.
pub struct Control {
    // The interface number of this control handler.  Used to check that we
    // only handle requests for this interface.
    if_num: InterfaceNumber,

    // Watchdog
    _watchdog: &'static WatchdogType,
}

// Error type used internally to decide how to respond to a Control message.
enum ControlError {
    // Ignore means we return None from the control handler function.
    Ignore,

    // Invalid means we return Rejected from the control handler function.
    Invalid,
}

impl Handler for Control {
    /// Called when the USB device has been enabled or disabled.
    fn enabled(&mut self, enabled: bool) {
        #[allow(clippy::if_same_then_else)]
        if enabled {
            trace!("USB device enabled");
        } else {
            trace!("USB device disabled");
        }
        Self::set_action(ProtocolAction::Uninitialize);
    }

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        trace!("USB device reset complete");
        Self::set_action(ProtocolAction::Uninitialize);
    }

    /// Called when the host has set the address of the device to `addr`.
    fn addressed(&mut self, addr: u8) {
        trace!("USB device addressed: {}", addr);
    }

    /// Called when the host has enabled or disabled the configuration of the device.
    fn configured(&mut self, configured: bool) {
        #[allow(clippy::if_same_then_else)]
        if configured {
            trace!("USB device configured");
        } else {
            trace!("USB device deconfigured");
        }
        Self::set_action(ProtocolAction::Uninitialize);
    }

    /// Called when the bus has entered or exited the suspend state.
    fn suspended(&mut self, suspended: bool) {
        if suspended {
            info!("USB device suspended");
            update_status(DisplayType::Init);
        } else {
            info!("USB device resumed");
        }

        // It could be argued that we shoud store off the current state and
        // reapply it after resume.  But we're not going to do that.
        Self::set_action(ProtocolAction::Uninitialize);
    }

    /// Respond to OUT control messages, where the host sends us a command.
    /// In our implementation, none of the OUT control messages include the
    /// host sending data, so buf is unused.
    fn control_out<'a>(&'a mut self, req: Request, _buf: &'a [u8]) -> Option<OutResponse> {
        // Get the request type, and check for errors
        let result = self.check_request(req, Direction::Out);

        // Handle the request
        if result.is_err() {
            update_status(DisplayType::Error);
        }
        match result {
            Err(ControlError::Ignore) => None,
            Err(ControlError::Invalid) => Some(OutResponse::Rejected),
            Ok(request) => {
                // Handle the request
                match Self::handle_out(&request) {
                    Err(ControlError::Ignore) => None,
                    Err(ControlError::Invalid) => Some(OutResponse::Rejected),
                    Ok(()) => Some(OutResponse::Accepted),
                }
            }
        }
    }

    /// Respond to IN control messages, where the host requests some data from
    /// the device.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        // Get the request type, and check for errors
        let result = self.check_request(req, Direction::In);

        // Handle the request
        if result.is_err() {
            update_status(DisplayType::Error);
        }
        match result {
            Err(ControlError::Ignore) => None,
            Err(ControlError::Invalid) => Some(InResponse::Rejected),
            Ok(request) => {
                // Handle the request and build the response
                match Self::handle_in(&request, buf, req.length as usize) {
                    Err(ControlError::Ignore) => None,
                    Err(ControlError::Invalid) | Ok(0) => Some(InResponse::Rejected),
                    Ok(len) => Some(InResponse::Accepted(&buf[..len])),
                }
            }
        }
    }
}

// Our own Control functions to help deal with the USB control requests.
impl Control {
    // Create a new instance of this control handler.  Stores the created
    // instance in the static.  Will panic if this is called more than once.
    pub fn create_static(
        if_num: InterfaceNumber,
        watchdog: &'static WatchdogType,
    ) -> &'static mut Self {
        let control = Self {
            if_num,
            _watchdog: watchdog,
        };
        CONTROL.init(control)
    }

    // Check the request is valid and supported.
    fn check_request(&self, req: Request, dir: Direction) -> Result<ControlRequest, ControlError> {
        // Trace the request.
        trace!(
            "Control request to interface: 0x{:02x}, request: {:#x}, request type: {}, recipient: {}, direction: {}",
            req.index, req.request, req.request_type, req.recipient, dir
        );

        // Only handle Class request types to an Interface.
        if req.request_type != RequestType::Class || req.recipient != Recipient::Device {
            info!(
                "Ignoring Control request type: {}, recipient: {}",
                req.request_type, req.recipient
            );
            return Err(ControlError::Ignore);
        }

        // Ignore requests to other interfaces.
        if req.index != u16::from(self.if_num.0) {
            info!("Ignoring Control request to interface: 0x{:02x}", req.index);
            return Err(ControlError::Ignore);
        }

        // Check we got a supported request - reject if not.
        let request = ControlRequest::try_from(req.request).map_err(|_| ControlError::Invalid)?;

        // Check we got a request in the correct direction - reject if not.
        if request.direction() != dir {
            info!(
                "Ignoring request {} with direction {}",
                request,
                request.direction()
            );
            return Err(ControlError::Invalid);
        }

        Ok(request)
    }

    // Handler for OUT rquests.
    #[allow(clippy::unnecessary_wraps)]
    fn handle_out(request: &ControlRequest) -> Result<(), ControlError> {
        info!("Received Control OUT request: {}", request);
        match request {
            ControlRequest::Reset => Self::set_action(ProtocolAction::Reset),
            ControlRequest::Shutdown => Self::set_action(ProtocolAction::Uninitialize),
            ControlRequest::EnterBootloader => {
                info!("Entering DFU bootloader");
                reboot_dfu(); // Does not return
            }
            ControlRequest::TapBreak => {
                // TODO - implement
                info!("{} - unsupported, ignoring", request);
            }
            _ => unreachable!(),
        }
        Ok(())
    }

    // Handler for IN requests.
    fn handle_in(
        request: &ControlRequest,
        buf: &mut [u8],
        len: usize,
    ) -> Result<usize, ControlError> {
        info!("Received Control IN request: {}", request);

        // Check the expected length is correct for this request.
        let response_len = request.response_len();
        if len < response_len {
            info!(
                "Invalid response length for request: 0x{:02x}, {} vs {}",
                request, len, response_len
            );
            return Err(ControlError::Invalid);
        }

        // Zero out the number of bytes for this request's response
        buf[..len].fill(0);

        // Perform any required actions, and fill in the response
        match request {
            ControlRequest::Echo => buf[0] = ControlRequest::Echo as u8,
            ControlRequest::Init => {
                Self::handle_init(buf);
            }
            ControlRequest::GitRev => {
                let version = GIT_VERSION.unwrap_or("unknown");
                Self::copy_string_to_buffer(version, buf);
            }
            ControlRequest::RustcVer => {
                // Extract the version number (e.g. 1.83.0) from the full rustc
                // version string
                let version = RUSTC_VERSION.split_whitespace().nth(1).unwrap_or("unknown");
                Self::copy_string_to_buffer(version, buf);
            }
            ControlRequest::PkgVer => {
                // There is no libc version or SDK version within this
                // rust implementation, so we interpret this as our Cargo.toml
                // version (i.e. the version of this crate).
                let version = PKG_VERSION;
                Self::copy_string_to_buffer(version, buf);
            }
            _ => unreachable!(),
        }
        Ok(response_len)
    }

    // Copies a string to a buffer, ensuring it is null-terminated.  Used for
    // returning device information on control requests.
    fn copy_string_to_buffer(source: &str, buf: &mut [u8]) {
        let copy_len = core::cmp::min(buf.len().saturating_sub(1), source.len());
        buf[..copy_len].copy_from_slice(&source.as_bytes()[..copy_len]);
        buf[copy_len] = 0;
    }

    // Sets the shared PROTOCOL_ACTION to the given action to signal to the
    // ProtocolHandler object to take the appropriate action.
    fn set_action(action: ProtocolAction) {
        // Try to take the current action - if there is one we need to
        // decide how to modify it and then resignal it.
        let new_action = match take_protocol_action() {
            Some(existing) => {
                match action {
                    // Initialize takes precedence over any other outstanding
                    // action
                    ProtocolAction::Initialize => ProtocolAction::Initialize,

                    // Unitialize takes precedence over any other outstanding
                    // action
                    ProtocolAction::Uninitialize => ProtocolAction::Uninitialize,

                    // Only set the action to reset if there's no other action
                    // outstanding.
                    ProtocolAction::Reset => existing,
                };
                action
            }
            None => action,
        };

        // signal() always succeeds as it overwrites any other value - but
        // in any case there shouldn't be one because we just took it above.
        set_protocol_action(new_action);
    }

    // Dedicated function for handling ControlRequest::Init.
    //
    // Will assume that buf is long enough for the response, and it has been
    // zeroed out.  The length was checked in control_in() and the buf zeroed
    // in handle_in().
    fn handle_init(buf: &mut [u8]) {
        assert!(buf.len() >= INIT_CONTROL_RESPONSE_LEN);

        // Check whether the Protocol Handler is already initialized.  If so
        // we will set the InitStatus appropriately.  We do this before we
        // signal to the Protocol Handler to initialize.
        let initialized = get_protocol_init();

        // Indicate the Protocol Handler that it should initialize.
        Self::set_action(ProtocolAction::Initialize);

        // Build the response.

        // Byte 0 is the xum1541 (or pico1541) firmware version.
        buf[0] = FIRMWARE_VERSION;

        // Byte 1 contains any capabilities of this device.
        buf[1] = Capabilities::CBM.bits();

        // Byte 2 contains any initialization status flags.
        buf[2] = InitStatus::NONE.bits();
        if initialized {
            trace!("Device was not shutdown cleanly");
            buf[2] |= InitStatus::NOT_SHUTDOWN.bits();
        }
        if get_ieee_present() {
            trace!("IEEE-488 device is present");
            buf[2] |= InitStatus::IEEE488_PRESENT.bits();
        }
        if get_tape_present() {
            trace!("Tape device is present");
            buf[2] |= InitStatus::TAPE_PRESENT.bits();
        }
    }
}

// List of Control requests that the device accepts.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ControlRequest {
    Echo = 0x00,
    Init = 0x01,
    Reset = 0x02,
    Shutdown = 0x03,
    EnterBootloader = 0x04,
    TapBreak = 0x05,
    GitRev = 0x06,
    RustcVer = 0x07,
    PkgVer = 0x08,
}

// Control request functions.
impl ControlRequest {
    // Returns the supported direction of the request.
    pub fn direction(self) -> Direction {
        match self {
            ControlRequest::Echo
            | ControlRequest::Init
            | ControlRequest::GitRev
            | ControlRequest::RustcVer
            | ControlRequest::PkgVer => Direction::In,
            ControlRequest::Reset
            | ControlRequest::Shutdown
            | ControlRequest::EnterBootloader
            | ControlRequest::TapBreak => Direction::Out,
        }
    }

    // Returns the response (data) length of the request, for OUT requests.
    fn response_len(self) -> usize {
        match self {
            ControlRequest::Echo => ECHO_CONTROL_RESPONSE_LEN,
            ControlRequest::Init => INIT_CONTROL_RESPONSE_LEN,
            ControlRequest::GitRev | ControlRequest::RustcVer | ControlRequest::PkgVer => {
                MAX_XUM_DEVINFO_SIZE_USIZE
            }
            _ => 0,
        }
    }
}

// Implements try_from to create a ControlRequest from the USB control request
// byte
impl TryFrom<u8> for ControlRequest {
    type Error = ControlError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Echo),
            0x01 => Ok(Self::Init),
            0x02 => Ok(Self::Reset),
            0x03 => Ok(Self::Shutdown),
            0x04 => Ok(Self::EnterBootloader),
            0x05 => Ok(Self::TapBreak),
            0x06 => Ok(Self::GitRev),
            0x07 => Ok(Self::RustcVer),
            0x08 => Ok(Self::PkgVer),
            _ => Err(ControlError::Invalid),
        }
    }
}

// Implements Format to allow defmt to print the Control request type in human
// readable form.
impl defmt::Format for ControlRequest {
    fn format(&self, f: defmt::Formatter) {
        match self {
            ControlRequest::Echo => defmt::write!(f, "Echo"),
            ControlRequest::Init => defmt::write!(f, "Init"),
            ControlRequest::Reset => defmt::write!(f, "Reset"),
            ControlRequest::Shutdown => defmt::write!(f, "Shutdown"),
            ControlRequest::EnterBootloader => defmt::write!(f, "EnterBootloader"),
            ControlRequest::TapBreak => defmt::write!(f, "TapBreak"),
            ControlRequest::GitRev => defmt::write!(f, "GitRev"),
            ControlRequest::RustcVer => defmt::write!(f, "RustcVer"),
            ControlRequest::PkgVer => defmt::write!(f, "PkgVer"),
        }
    }
}

// Called to perform a reboot into BOOTSEL/DFU mode.
fn reboot_dfu() -> ! {
    #[cfg(feature = "pico")]
    {
        embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        #[allow(clippy::empty_loop)]
        loop {}
    }
    #[cfg(feature = "pico2")]
    {
        // Doesn't appear to reboot the device in BOOTSEL mode, not sure why
        const REBOOT_TYPE_BOOTSEL: u32 = 0x0002;
        const NO_RETURN_ON_SUCCESS: u32 = 0x0100;
        let _ = embassy_rp::rom_data::reboot(REBOOT_TYPE_BOOTSEL | NO_RETURN_ON_SUCCESS, 0, 0, 0);
        #[allow(clippy::empty_loop)]
        loop {}
    }
}
