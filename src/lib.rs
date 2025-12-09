//! pico1541
//!
//! This implements a USB device that allows Commodore disk drives to be
//! connected to a PC.
//!
//! The device can either emulate an xum1541, and is therefore supported
//! directly by [`OpenCBM`](https://github.com/OpenCBM/OpenCBM), or provides
//! extended capabilities and is detected as a pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![no_std]
#![no_main]

// Provide some feature guidance when compiling the library.
#[cfg(not(any(feature = "compatibility", feature = "extended")))]
compile_error!("Either 'xum1541/compatibility' or 'extended' feature must be enabled");
#[cfg(all(feature = "compatibility", feature = "extended"))]
compile_error!("Features 'xum1541/compatibility' and 'extended' cannot be enabled simultaneously");
#[cfg(not(any(feature = "pico", feature = "pico2")))]
compile_error!("Either 'pico' or 'pico2' feature must be enabled");
#[cfg(all(feature = "pico", feature = "pico2"))]
compile_error!("Features 'pico' and 'pico2' cannot be enabled simultaneously");

// Declare all of this library's modules.
mod constants;
pub mod entry;
mod infra;
mod protocol;
mod task;
pub mod test;
mod usb;
mod util;
mod wifi;

// Extra binary information that picotool can read.
//#[unsafe(link_section = ".bi_entries")]
//#[used]
//pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
//    embassy_rp::binary_info::rp_program_name!(c"pico1541 by piers.rocks"),
//    embassy_rp::binary_info::rp_program_description!(c"A USB-based Commodore disk drive adapter for PCs, compatible with OpenCBM, a drop-in replacement for the xum1541, and provides additional, functionality, like WiFi support."),
//    embassy_rp::binary_info::rp_cargo_version!(),
//    embassy_rp::binary_info::rp_program_build_attribute!(),
//];
