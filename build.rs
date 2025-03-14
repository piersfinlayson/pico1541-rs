//! This build script handles:
//! - Exposing build-time information to the application.
//! - Copying `memory.x` to the output directory to allow the firmware to be
//!   created.
//!
//! ## Build-time information
//!
//! ## `memory.x` file handling
//!
//! This build script copies the appropriate memory.x file from the `link/`
//! dir into a directory where the linker can find it at link time.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

// memory.x handling derived from embassy-rs examples.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Expose build-time information to the application.

    // Re-run this build script if anything in git changes.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs/");

    // Re-run this build script of DEFMT_LOG changes.
    println!("cargo:rerun-if-env-changed=DEFMT_LOG");

    // Get built-time information
    built::write_built_file().expect("Failed to acquire build-time information");

    // RP2040 and RP235X use different memory.x files.  Ensure the build
    // script is re-run if the appropriate memory.x file changes.  Note that
    // neither file should be called memory.x, as then the linker will pick up
    // that file from our root directory, instead of the version we put in
    // OUT_DIR, below.
    #[cfg(feature = "pico")]
    let memory_x = {
        println!("cargo:rerun-if-changed=link/memory.rp2040.x");
        include_bytes!("link/memory.rp2040.x")
    };
    #[cfg(feature = "pico2")]
    let memory_x = {
        println!("cargo:rerun-if-changed=link/memory.rp235x.x");
        include_bytes!("link/memory.rp235x.x")
    };

    // Put `memory.x` in our output directory and ensure it's on the linker
    // search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x)
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // Set embassy linker arguments for the binary.
    println!("cargo:rustc-link-arg=-v");
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tdevice.x");

    // Only RP2040 uses this linker file.
    #[cfg(feature = "pico")]
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
}
