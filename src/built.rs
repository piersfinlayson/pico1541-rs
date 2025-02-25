//! This file handles build-time information.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use defmt::info;

// Import the build-time information from the `built.rs` output file generated
// during the build process by `build.rs`.
mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

// Expose Git version so it can be used in the application.
pub use built_info::GIT_VERSION as GIT_VERSION;

// Expose rustc version so it can be used in the application.
pub use built_info::RUSTC_VERSION as RUSTC_VERSION;

// Expose package version so it can be used in the application.
pub use built_info::PKG_VERSION as PKG_VERSION;

/// Log build-time information to the console.
pub fn log_fw_info(bin_name: &str, serial: &str) {
    // General information
    info!("{} operating in {} mode", built_info::PKG_NAME, bin_name);
    info!("Author: {}", built_info::PKG_AUTHORS);
    info!("pico1541 Version: {}", built_info::PKG_VERSION);
    #[cfg(feature = "compatibility")]
    info!("xum1541 Version: {}", crate::constants::XUM1541_FIRMWARE_VERSION);
    info!("Serial: {}", serial);

    // Git information
    info!("Git commit: {}", built_info::GIT_COMMIT_HASH.unwrap_or("unknown"));
    info!("Git dirty: {}", built_info::GIT_DIRTY.unwrap_or(false));
    info!("Git version: {}", built_info::GIT_VERSION.unwrap_or("unknown"));

    // Build information
    info!("Built: {}", built_info::BUILT_TIME_UTC);
    info!("Rust version: {}", built_info::RUSTC_VERSION);
    info!("Host triple: {}", built_info::HOST);
    info!("Target triple: {}", built_info::TARGET);
    info!("Build profile: {}", built_info::PROFILE);
    info!("Enabled features: {:?}", built_info::FEATURES_LOWERCASE_STR);
}