//! This file implements the Commodore IEEE protocol driver.  Based on
//! the xum1541 source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Endpoint, In};

use crate::driver::{DriverError, ProtocolDriver};
use crate::protocol::ProtocolFlags;

pub struct IeeeDriver {}

#[allow(unused_variables)]
impl ProtocolDriver for IeeeDriver {
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        unimplemented!()
    }

    async fn raw_write(&mut self, data: &[u8], flags: ProtocolFlags) -> Result<u16, DriverError> {
        unimplemented!()
    }

    async fn raw_read(
        &mut self,
        len: u16,
        write_ep: &mut Endpoint<'static, USB, In>,
    ) -> Result<u16, DriverError> {
        unimplemented!()
    }

    async fn wait(&mut self, line: u8, state: u8) -> Result<(), DriverError> {
        unimplemented!()
    }

    async fn poll(&mut self) -> u8 {
        unimplemented!()
    }

    async fn set_release(&mut self, set: u8, release: u8) {
        unimplemented!()
    }
}
