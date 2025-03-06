//! This file implements the Commodore IEEE protocol driver.  Based on
//! the xum1541 source code.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

use crate::driver::{DriverError, ProtocolDriver};
use crate::protocol::{ProtocolFlags, ProtocolType};

pub struct IeeeDriver {}

#[allow(unused_variables)]
impl ProtocolDriver for IeeeDriver {
    async fn reset(&mut self, forever: bool) -> Result<(), DriverError> {
        unimplemented!()
    }

    async fn raw_write(
        &mut self,
        len: u16,
        _protocol: ProtocolType,
        flags: ProtocolFlags,
    ) -> Result<u16, DriverError> {
        unimplemented!()
    }

    async fn raw_read(&mut self, len: u16) -> Result<u16, DriverError> {
        unimplemented!()
    }

    async fn wait(&mut self, line: u8, state: u8) -> Result<(), DriverError> {
        unimplemented!()
    }

    fn poll(&mut self) -> u8 {
        unimplemented!()
    }

    fn setrelease(&mut self, set: u8, release: u8) {
        unimplemented!()
    }

    fn get_eoi(&self) -> bool {
        unimplemented!()
    }

    fn clear_eoi(&mut self) {
        unimplemented!()
    }
}
