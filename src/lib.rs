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
mod usb;
mod util;
mod wifi;

// Extra binary information that picotool can read.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"pico1541 by piers.rocks"),
    embassy_rp::binary_info::rp_program_description!(c"A USB-based Commodore disk drive adapter for PCs, compatible with OpenCBM, a drop-in replacement for the xum1541, and provides additional, functionality, like WiFi support."),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// A note about Statics
//
// We set up statics primarily to avoid lifetime issues, and to allow us to
// spawn tasks (accessing these statics), and to split our code into
// separate modules.
//
// These are a bit tricksy to get right, so here is some general guidance:
//
// - Use StaticCell for statics that cannot be initialized at compile time.
//
// - Use ConstStaticCell for statics that can be initialized at compile time.
//   Note that initialization is different than mutability.  A ConstStaticCell
//   can be mutable, when used with RefCell, but it must be initialized at
//   compile time.  It also cannot be take()n and then modified and then
//   take()n again.
//
// - If your static will be immutable, that is all that is required.
//
// - If your static will be mutable, but you will be passing ownership of it
//   to another object, then no Mutex is required either.
//
// - If you need mutable access, you need to use a Mutex.  If you are using a
//   embassy_sync::mutex::Mutex (which is async) you do not need a RefCell for
//   interior mutability.  If you use a blocking_mutexx::Mutex you do.
//   - Generally use CriticalSectionRawMutex, as these work on multi-core
//     systems.
//   - ThreadModeRawMutex is, as it sounds, so single threaded usage.
//   - NoopRawMutex is for when you don't need a mutex.
//
//   Our implementation is not sufficiently dependent on performance to
//   require optimization here, so we tend to use CriticalSectionRawMutex.
//
// The statics tend to be stored in the module that creates them.  So, for
// example, the GPIO static is in gpio.
