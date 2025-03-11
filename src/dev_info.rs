//! Used to define the USB device information, contained in the USB device
//! and configuration descriptors, based on the feature(s) enabled.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use core::fmt::Write;
#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_rp::peripherals::{DMA_CH0, FLASH};
use heapless::String;
use static_cell::StaticCell;

use crate::constants::{self, MAX_SERIAL_STRING_LEN};

// Create a static String to store the serial number in to be consumed by the
// USB stack creation.
pub static USB_SERIAL: StaticCell<String<MAX_SERIAL_STRING_LEN>> = StaticCell::new();

// Create a static String to store the serial number in to be consumed by the
// built logging function.
static LOG_SERIAL: StaticCell<String<MAX_SERIAL_STRING_LEN>> = StaticCell::new();

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
/// This function retrieves the device's serial number in two different ways
/// depending on compile features:
/// - With "compatibility" feature: Returns a constant predefined serial number
/// - With "extended" feature: Reads the unique Pico ID from flash memory and
///   formats it as a hex string
pub async fn get_serial(
    _flash: &mut FLASH,
    _dma_ch0: &mut DMA_CH0,
) -> (
    &'static mut String<MAX_SERIAL_STRING_LEN>,
    &'static mut String<MAX_SERIAL_STRING_LEN>,
) {
    // Create a temporary stack String to store the serial number in.
    let mut serial = String::<MAX_SERIAL_STRING_LEN>::new();

    #[cfg(feature = "compatibility")]
    {
        // Fill the buffer with the constant value
        match write!(serial, "{}", constants::XUM1541_SERIAL) {
            Ok(_) => {}
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
    {
        // Get the Pico serial number.  This differs between the Pico and Pico
        // 2.  On the former, we have to get the flash chip ID, on the latter
        // we can get the unique chip ID from the OTP area.
        let mut byte_buf = [0u8; 16];

        #[cfg(feature = "pico")]
        {
            // Get the Pico serial number
            const FLASH_SIZE: usize = 2 * 1024 * 1024;
            let mut flash =
                embassy_rp::flash::Flash::<_, embassy_rp::flash::Async, FLASH_SIZE>::new(
                    _flash, _dma_ch0,
                );

            // If actually getting the unique flash ID fails, the serial number will
            // be all zeroes.
            flash.blocking_unique_id(&mut byte_buf).unwrap();
            debug!("Pico Flash ID: {:#x}", u128::from_be_bytes(byte_buf));
        }
        #[cfg(feature = "pico2")]
        {
            // Add some delay to give an attached debug probe time to parse
            // the defmt RTT header. Reading that header might touch flash
            // memory, which interferes with flash write operations.
            // https://github.com/knurling-rs/defmt/pull/683
            embassy_time::Timer::after_millis(10).await;

            // Get the Pico 2 serial number
            let chip_id = embassy_rp::otp::get_chipid().unwrap();
            debug!("Pico 2 Chip ID: {:#x}", chip_id);

            // Put the 64-bit chip ID into the buffer back how it came from
            // hardware
            byte_buf[..8].copy_from_slice(&chip_id.to_be_bytes());
        }

        // Format only the first 8 bytes (or fewer) as hex string
        let bytes_to_format = if byte_buf.len() > (MAX_SERIAL_STRING_LEN / 2) {
            &byte_buf[..(MAX_SERIAL_STRING_LEN / 2)]
        } else {
            &byte_buf[..]
        };

        // Format the bytes as hex string
        for b in bytes_to_format.iter() {
            // Using defmt's formatting or manual hex formatting if possible.
            match write!(serial, "{:02x}", b) {
                Ok(_) => {}
                Err(_) => {
                    // Fallback to manual hex formatting if write! fails
                    let hex_chars = b"0123456789abcdef";
                    let _ = serial.push(hex_chars[(b >> 4) as usize] as char);
                    let _ = serial.push(hex_chars[(b & 0xf) as usize] as char);
                }
            }
        }
    }

    // Store the serial number in the statics and return references to them.
    (USB_SERIAL.init(serial.clone()), LOG_SERIAL.init(serial))
}
