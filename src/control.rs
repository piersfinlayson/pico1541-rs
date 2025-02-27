//! This module implements the USB Control handler.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

use embassy_usb::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use embassy_usb::types::InterfaceNumber;
use embassy_usb::Handler;
use static_cell::StaticCell;

use crate::built::{GIT_VERSION, PKG_VERSION, RUSTC_VERSION};
use crate::constants::{
    ECHO_CONTROL_RESPONSE_LEN, INIT_CONTROL_RESPONSE_LEN, MAX_XUM_DEVINFO_SIZE_USIZE,
};
use crate::display::{update_status, DisplayType};
use crate::protocol::ProtocolAction;
use crate::protocol::PROTOCOL_ACTION;
use crate::types::Direction;
use crate::watchdog::reboot_dfu;

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
}

// Error type used internally to device how to respond to a Control message.

enum ControlError {
    // Ignore means we return None from the control handler function.
    Ignore,

    // Invalid means we return Rejected from the control handler function.
    Invalid,
}

impl Handler for Control {
    /// Called when the USB device has been enabled or disabled.
    fn enabled(&mut self, enabled: bool) {
        match enabled {
            true => {
                info!("USB device enabled");
            }
            false => {
                info!("USB device disabled");
                update_status(DisplayType::Init);
            }
        }
        self.set_action(ProtocolAction::Uninitialize);
    }

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self) {
        info!("USB device reset complete");
        self.set_action(ProtocolAction::Uninitialize);
    }

    /// Called when the host has set the address of the device to `addr`.
    fn addressed(&mut self, addr: u8) {
        info!("USB device addressed: {}", addr);
    }

    /// Called when the host has enabled or disabled the configuration of the device.
    fn configured(&mut self, configured: bool) {
        match configured {
            true => info!("USB device configuration enabled"),
            false => info!("USB device configuration disabled"),
        }
        self.set_action(ProtocolAction::Uninitialize);
    }

    /// Called when the bus has entered or exited the suspend state.
    fn suspended(&mut self, suspended: bool) {
        match suspended {
            true => {
                info!("USB device suspended");
                update_status(DisplayType::Init);
            }
            false => {
                info!("USB device resumed");
            }
        }

        // It could be argued that we shoud store off the current state and
        // reapply it after resume.  But we're not going to do that.
        self.set_action(ProtocolAction::Uninitialize);
    }

    /// Respond to HostToDevice control messages, where the host sends us a command and
    /// optionally some data, and we can only acknowledge or reject it.
    fn control_out<'a>(&'a mut self, req: Request, buf: &'a [u8]) -> Option<OutResponse> {
        // Log the request before filtering to help with debugging.
        info!("Got control_out, request={}, buf={:a}", req, buf);

        // Get the request type, and check for errors
        let result = self.check_request(req, Direction::Out);
        if result.is_err() {
            update_status(DisplayType::Error);
        }
        let request = match result {
            Err(ControlError::Ignore) => return None,
            Err(ControlError::Invalid) => return Some(OutResponse::Rejected),
            Ok(r) => r,
        };

        // Handle the request
        match self.handle_out(&request) {
            Err(ControlError::Ignore) => None,
            Err(ControlError::Invalid) => Some(OutResponse::Rejected),
            Ok(_) => Some(OutResponse::Accepted),
        }
    }

    /// Respond to DeviceToHost control messages, where the host requests some data from us.
    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        info!("Got control_in, request={}", req);

        // Get the request type, and check for errors
        let result = self.check_request(req, Direction::In);
        if result.is_err() {
            update_status(DisplayType::Error);
        }
        let request = match result {
            Err(ControlError::Ignore) => return None,
            Err(ControlError::Invalid) => return Some(InResponse::Rejected),
            Ok(r) => r,
        };

        // Handle the request and build the response
        match self.handle_in(&request, buf, req.length as usize) {
            Err(ControlError::Ignore) => None,
            Err(ControlError::Invalid) | Ok(0) => Some(InResponse::Rejected),
            Ok(len) => Some(InResponse::Accepted(&buf[..len])),
        }
    }
}

// Our own Control functions to help deal with the USB control requests.
impl Control {
    // Create a new instance of this control handler.
    pub fn create_static(if_num: InterfaceNumber) -> &'static mut Self {
        let control = Self { if_num };
        CONTROL.init(control)
    }

    // Check the request is valid and supported.
    fn check_request(&self, req: Request, dir: Direction) -> Result<ControlRequest, ControlError> {
        // Only handle Class request types to an Interface.
        if req.request_type != RequestType::Class || req.recipient != Recipient::Interface {
            info!(
                "Ignoring request type: {:?}, recipient: {:?}",
                req.request_type, req.recipient
            );
            return Err(ControlError::Ignore);
        }

        // Ignore requests to other interfaces.
        if req.index != self.if_num.0 as u16 {
            info!("Ignoring request to interface: {}", req.index);
            return Err(ControlError::Ignore);
        }

        // Check we got a supported request
        let request = match ControlRequest::try_from(req.request) {
            Ok(r) => r,
            Err(_) => {
                info!("Invalid control request: {}", req.request);
                return Err(ControlError::Invalid);
            }
        };

        // Check we got an In request
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
    fn handle_out(&mut self, request: &ControlRequest) -> Result<(), ControlError> {
        match request {
            ControlRequest::Reset => {
                // We only want to set the action to RESET if there's no action
                // outstanding.  If there's a reset outstanding, it's a no-op.
                // If we have initialize or uninitialize, they superceed.
                self.set_action(ProtocolAction::Reset);
            }
            ControlRequest::Shutdown => {
                // Overwrite any existing action as this takes precedence
                self.set_action(ProtocolAction::Uninitialize);
                update_status(DisplayType::Ready);
            }
            ControlRequest::EnterBootloader => {
                info!("Entering DFU bootloader");
                reboot_dfu(); // Does not return
            }
            ControlRequest::TapBreak => {
                // TODO - implement
                info!("{}", request);
            }
            _ => unreachable!(),
        };
        Ok(())
    }

    fn handle_in(
        &self,
        request: &ControlRequest,
        buf: &mut [u8],
        len: usize,
    ) -> Result<usize, ControlError> {
        // Check the response length is correct for this request
        let response_len = request.response_len();
        if len < response_len {
            return Err(ControlError::Invalid);
        }

        // Zero out the number of bytes for this request's response
        buf[..len].fill(0);

        // Perform any required actions, and fill in the response
        match request {
            ControlRequest::Echo => buf[0] = &ControlRequest::Echo as *const _ as u8,
            ControlRequest::Init => {
                // Overwrite any existing action as initialize will also
                // deinitialize first (and also reset if required)
                self.set_action(ProtocolAction::Initialize);
                update_status(DisplayType::Active);

                // Build the response
                buf[0] = 0x08;
                buf[1] = 0x03;
            }
            ControlRequest::GitRev => {
                if let Some(version) = GIT_VERSION {
                    // Copy the version string to the buffer with null
                    // termination
                    let bytes = version.as_bytes();
                    let len = core::cmp::min(bytes.len(), len);
                    buf[..len].copy_from_slice(&bytes[..len]);
                    buf[len] = 0;
                } else {
                    buf[..8].copy_from_slice(b"unknown\0");
                }
            }
            ControlRequest::RustcVer => {
                // Extract the version number (e.g. 1.83.0) from the full rustc
                // version string
                let version = RUSTC_VERSION.split_whitespace().nth(1).unwrap_or("unknown");

                // Copy the version string to the buffer with null termination
                let copy_len = core::cmp::min(len - 1, version.len());
                buf[..copy_len].copy_from_slice(&version.as_bytes()[..copy_len]);
                buf[copy_len] = 0;
            }
            ControlRequest::PkgVer => {
                // There is no libc version or SDK version within this
                // rust implementation, so we interpret this as our Cargo.toml
                // version (i.e. the version of this crate).
                let version = PKG_VERSION;
                let copy_len = core::cmp::min(len, version.len());
                buf[..copy_len].copy_from_slice(&version.as_bytes()[..copy_len]);
                if copy_len < len {
                    // null terminate
                    buf[copy_len] = 0;
                }
            }
            _ => unreachable!(),
        };

        Ok(response_len)
    }

    // Sets the shared PROTOCOL_ACTION to the given action to signal to the
    // ProtocolHandler object to take the appropriate action.
    fn set_action(&self, action: ProtocolAction) {
        // Try to take the current action - if there is one we need to
        // decide how to modify it and then resignal it.
        let action = match PROTOCOL_ACTION.try_take() {
            Some(existing) => {
                match action {
                    // Initialize takes precedence over any other outstanding
                    // action
                    ProtocolAction::Initialize => ProtocolAction::Initialize,

                    // Unitialize takes precedence over any other outstanding
                    // action
                    ProtocolAction::Uninitialize => ProtocolAction::Uninitialize,

                    // Only set the action to reset if there's no action
                    // outstanding (which there is if we get here)
                    ProtocolAction::Reset => existing,
                };
                action
            }
            None => action,
        };

        // signal() always succeeds as it overwrites any other value - but
        // there shouldn't be one because we just took it.
        PROTOCOL_ACTION.signal(action);
    }
}

// List of Control requests that we accept.
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
    pub fn direction(&self) -> Direction {
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

    // Returns the response length of the request, for OUT requests.
    fn response_len(&self) -> usize {
        match self {
            ControlRequest::Echo => ECHO_CONTROL_RESPONSE_LEN,
            ControlRequest::Init => INIT_CONTROL_RESPONSE_LEN,
            ControlRequest::GitRev => MAX_XUM_DEVINFO_SIZE_USIZE,
            ControlRequest::RustcVer => MAX_XUM_DEVINFO_SIZE_USIZE,
            ControlRequest::PkgVer => MAX_XUM_DEVINFO_SIZE_USIZE,
            _ => 0,
        }
    }
}

// Implements try_from to create a ControlRequest from the USB control request
// byte
impl TryFrom<u8> for ControlRequest {
    type Error = ();

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
            _ => Err(()),
        }
    }
}

// Implements Format to allow defmt to print the ControlRequest.
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
