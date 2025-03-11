//! This module contains general types used by the embassy-rs Vendor Example.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use bitflags::bitflags;

/// Direction - used to apply to Control transfers, and other operations/
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    /// In is from the device to the host.
    In,

    /// Out is from the host to the device.
    Out,
}

// Implement Format so Direction can be formatted by defmt.
impl defmt::Format for Direction {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Direction::In => defmt::write!(f, "In"),
            Direction::Out => defmt::write!(f, "Out"),
        }
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Clone, Copy)]
    /// The capabilities of the device are returned on ControlRequest::Init
    /// response (byte 1).
    pub struct Capabilities: u8 {
        const NONE = 0x00;
        const CBM = 0x01;
        const NIB = 0x02;
        const NIB_SRQ = 0x04;
        const IEEE488 = 0x08;
        const TAP = 0x10;
    }
}

bitflags! {
    #[repr(transparent)]
    #[derive(Clone, Copy)]
    /// Initialization status flags for the device, returned on
    /// ControlRequest::Init response (byte 2).
    pub struct InitStatus: u8 {
        const NONE = 0x00;
        const DOING_RESET = 0x01;
        const IEEE488_PRESENT = 0x10;
        const TAPE_PRESENT = 0x20;
    }
}
