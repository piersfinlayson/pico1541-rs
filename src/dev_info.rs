//! Used to define the USB device information based on the features enabled.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_rp::peripherals::{FLASH, DMA_CH0};
use crate::constants::{self, MAX_SERIAL_STRING_LEN};

#[cfg(feature = "compatibility")]
use core::fmt::Write;
#[cfg(feature = "extended")]
use embassy_rp::flash::Async;

// USB device information based on the features enabled.
#[cfg(feature = "compatibility")]
pub const MANUFACTURER: &str = constants::MANUFACTURER;
#[cfg(feature = "compatibility")]
pub const PRODUCT: &str = constants::XUM1541_PRODUCT;
#[cfg(feature = "compatibility")]
pub const VENDOR_ID: u16 = constants::XUM1541_VENDOR_ID;
#[cfg(feature = "compatibility")]
pub const PRODUCT_ID: u16 = constants::XUM1541_PRODUCT_ID;
#[cfg(feature = "compatibility")]
pub const OUT_EP: u8 = constants::XUM1541_OUT_EP;
#[cfg(feature = "compatibility")]
pub const IN_EP: u8 = constants::XUM1541_IN_EP;

#[cfg(feature = "extended")]
pub const MANUFACTURER: &str = constants::MANUFACTURER;
#[cfg(feature = "extended")]
pub const PRODUCT: &str = constants::PICO1541_PRODUCT;
#[cfg(feature = "extended")]
pub const VENDOR_ID: u16 = constants::PICO1541_VENDOR_ID;
#[cfg(feature = "extended")]
pub const PRODUCT_ID: u16 = constants::PICO1541_PRODUCT_ID;
#[cfg(feature = "extended")]
pub const OUT_EP: u8 = constants::PICO1541_OUT_EP;
#[cfg(feature = "extended")]
pub const IN_EP: u8 = constants::PICO1541_IN_EP;

/// Gets the device serial number as a string slice
///
/// This function retrieves the device's serial number in two different ways depending on compile features:
/// - With "compatibility" feature: Returns a constant predefined serial number
/// - With "extended" feature: Reads the unique Pico ID from flash memory and formats it as a hex string
///
/// # Arguments
/// * `p` - Mutable reference to device peripherals
/// * `byte_buf` - Buffer to store the raw 8-byte unique ID (for "extended" feature)
/// * `str_buf` - String buffer to store the formatted hex output
///
/// # Returns
/// A string slice reference to the formatted serial number
///
/// # Example
/// ```
/// let mut byte_buf = [0u8; 8];
/// let mut str_buf = heapless::String::<16>::new();
/// let serial = get_serial(&mut peripherals, &mut byte_buf, &mut str_buf);
/// ```
/// Gets the device serial number as a string slice
///
/// This function retrieves the device's serial number in two different ways depending on compile features:
/// - With "compatibility" feature: Returns a constant predefined serial number
/// - With "extended" feature: Reads the unique Pico ID from flash memory and formats it as a hex string
///
/// # Arguments
/// * `p` - Mutable reference to device peripherals (unused in compatibility mode)
/// * `str_buf` - String buffer to store the formatted hex output (unused in compatibility mode)
///
/// # Example
/// ```
/// let mut byte_buf = [0u8; 8];
/// let mut str_buf = heapless::String::<16>::new();
/// let serial = get_serial(&mut peripherals, &mut byte_buf, &mut str_buf);
/// ```
#[cfg(feature = "compatibility")]
pub fn get_serial<'a>(
    _flash: &mut FLASH,
    _dma_ch0: &mut DMA_CH0,
    serial: &'a mut heapless::String<MAX_SERIAL_STRING_LEN>
) {
    // Clear any previous content
    serial.clear();

    // Fill the buffer with the constant value
    match write!(serial, "{}", constants::XUM1541_SERIAL) {
        Ok(_) => {},
        Err(_) => {
            // Handle error - fallback to manual char-by-char copying
            for c in constants::XUM1541_SERIAL.chars() {
                // Try to push each character, but stop if the buffer is full
                if serial.push(c).is_err() {
                    break;
                }
            }
        }
    }
}
#[cfg(feature = "extended")]
pub fn get_serial<'a>(
    flash: &mut FLASH,
    dma_ch0: &mut DMA_CH0,
    serial: &'a mut heapless::String<MAX_SERIAL_STRING_LEN>
) {
    // Clear any previous content
    serial.clear();

    // Get the Pico serial number
    const FLASH_SIZE: usize = 2 * 1024 * 1024;
    let mut byte_buf = [0u8; 16];
    let mut flash = embassy_rp::flash::Flash::<_, Async, FLASH_SIZE>::new(flash, dma_ch0);

    // If actually getting the unique flash ID fails, the serial number will
    // be all zeroes. 
    let _ = flash.blocking_unique_id(&mut byte_buf);
    
    // Clear any previous content
    serial.clear();

    // Format only the last 8 bytes (or fewer) as hex string
    let bytes_to_format = if byte_buf.len() > (MAX_SERIAL_STRING_LEN/2) {
        &byte_buf[byte_buf.len() - (MAX_SERIAL_STRING_LEN/2)..]
    } else {
        &byte_buf[..]
    };
    
    // Format the bytes as hex string
    for b in bytes_to_format.iter() {
        // Using defmt's formatting or manual hex formatting
        use core::fmt::Write;
        match write!(serial, "{:02x}", b) {
            Ok(_) => {},
            Err(_) => {
                // Fallback to manual hex formatting if write! fails
                let hex_chars = b"0123456789abcdef";
                let _ = serial.push(hex_chars[(b >> 4) as usize] as char);
                let _ = serial.push(hex_chars[(b & 0xf) as usize] as char);
            }
        }
    }
}