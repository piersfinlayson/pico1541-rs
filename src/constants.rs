//! This module contains constants for the embassy-rs Vendor Example.
//!
//! It has multiple versions of constants that are used by different
//! configurations, such as usb product string.  The correct version is
//! selected by the dev_info module.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

// Allow dead code in here, as some constants are not used, depending on the
// features enabled.
#![allow(dead_code)]

use embassy_time::Duration;

//
// Watchdog timers
//

/// Watchdog timer - the watchdog resets the system if it isn't feed at
/// least this frequently.
pub const WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often the runner threads aim to feed the watchdog timer so it doesn't
/// reset the device.
pub const WATCHDOG_LOOP_TIMER: Duration = Duration::from_millis(100);

/// How often the bulk task must feed the watchdog to prevent a reset.
pub const BULK_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often the status display must feed the watchdog to prevent a reset.
pub const STATUS_DISPLAY_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often core 1 must feed the watchdog to prevent a reset.
pub const CORE1_WATCHDOG_TIMER: Duration = Duration::from_secs(10);

//
// Task main runner and related timers.
//

/// Timer for the ProtocolHandler to pause between loops of its main runner.
/// This is a low value, to ensure we apply Control driven changes quickly,
/// and serve any outstanding data rapidly.
pub const PROTOCOL_HANDLER_TIMER: Duration = Duration::from_millis(1);

// Timer for the StatusDisplay spend on and off when blinking.
pub const STATUS_DISPLAY_BLINK_TIMER: Duration = Duration::from_millis(100);

// Timer for the StatusDisplay to pause between doing work.  Must be less
// than the minimum time the status LED can be on off.
pub const STATUS_DISPLAY_TIMER: Duration = Duration::from_millis(50);

// How often we aim to log from our primary loops to prove they are still
// alive.
pub const LOOP_LOG_INTERVAL: Duration = Duration::from_secs(5);

//
// USB device configuration constants.
//

/// USB Descriptor information - what current in mA this device draws.
pub const USB_POWER_MA: u16 = 100;

/// USB Descriptor information - maximum endpoint 0 (control endpoint)
/// packet size.
pub const MAX_PACKET_SIZE_0: u8 = 64;

/// USB Descriptor information - maximum vendor endpoint packet sizes.
pub const MAX_EP_PACKET_SIZE: u16 = 64;

/// OUT (Host to Devive) endpoint number.  Note on the Pi this cannot be
/// larger than 0x0F, as the Pi only supports up to 16 endpoints in its
/// hardware registers.  If it is, the firmware will panic during the
/// endpoint allocation, as the endpoint cannot be allocated.
pub const XUM1541_OUT_EP: u8 = 0x04;
pub const PICO1541_OUT_EP: u8 = 0x01;

/// IN (Device to Host) endpoint number/  As above, this cannot be larger
/// that 0x0F.
pub const XUM1541_IN_EP: u8 = 0x83;
pub const PICO1541_IN_EP: u8 = 0x81;

/// USB Descriptor information - Vendor ID and Product ID
pub const XUM1541_VENDOR_ID: u16 = 0x16d0;
pub const XUM1541_PRODUCT_ID: u16 = 0x0504;
pub const PICO1541_VENDOR_ID: u16 = 0x1209;
pub const PICO1541_PRODUCT_ID: u16 = 0xf541;

/// USB Descriptor information - manufacturer string
pub const MANUFACTURER: &str = "piers.rocks";

/// USB Descriptor info - product string
pub const XUM1541_PRODUCT: &str = "xum1541 floppy adapter (pico1541)";
pub const PICO1541_PRODUCT: &str = "pico1541 floppy adapter";

/// USB Descriptor info - serial number string
pub const XUM1541_SERIAL: &str = "000";
pub const MAX_SERIAL_STRING_LEN: usize = 16;

/// USB Descriptor info - device class, subclass, and protocol
pub const USB_CLASS: u8 = 0xff;
pub const USB_SUB_CLASS: u8 = 0;
pub const USB_PROTOCOL: u8 = 0;

//
// xum1541 Protocol constants
//

/// Maximum size of a Write command
pub const MAX_WRITE_SIZE: u16 = 32768;
pub const MAX_WRITE_SIZE_USIZE: usize = MAX_WRITE_SIZE as usize;

/// Maximum size of a Read command
pub const MAX_READ_SIZE: u16 = 32768;
#[allow(dead_code)]
pub const MAX_READ_SIZE_USIZE: usize = MAX_READ_SIZE as usize;

/// Maximum size of xum1541 debug information strings
pub const MAX_XUM_DEVINFO_SIZE: u16 = 8;
pub const MAX_XUM_DEVINFO_SIZE_USIZE: usize = MAX_XUM_DEVINFO_SIZE as usize;

/// The Init Control request response length.
pub const INIT_CONTROL_RESPONSE_LEN: usize = 8;

/// The Echo Control request response length.
pub const ECHO_CONTROL_RESPONSE_LEN: usize = 8;

/// The xum1541 firmware version the pico1541 is emulating.
#[cfg(feature = "compatibility")]
pub const XUM1541_FIRMWARE_VERSION: u8 = 8;

//
// Other constants
//

/// The size (in bytes) of the stack for core 1.
pub const CORE1_STACK_SIZE: usize = 4096;
