//! This module contains constants for the embassy-rs Vendor Example.
//!
//! It has multiple versions of constants that are used by different
//! configurations, such as usb product string.  The correct version is
//! selected by the dev_info module.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_time::Duration;
use static_assertions::const_assert;

//
// Watchdog timers
//

/// Watchdog timer - the watchdog resets the system if it isn't feed at
/// least this frequently.
pub const WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often the runner threads aim to feed the watchdog timer so it doesn't
/// reset the device.
pub const WATCHDOG_LOOP_TIMER: Duration = Duration::from_millis(100);

/// How often the protocol handler task must feed the watchdog to prevent a
/// reset.
pub const PROTOCOL_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often the status display must feed the watchdog to prevent a reset.
pub const STATUS_DISPLAY_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often the WiFi Control task must feed the watchdog to prevent a reset.
pub const WIFI_CONTROL_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

/// How often a drive (read/write) operation must feed the watchdog to prevent
/// a reset
pub const DRIVE_OPERATION_WATCHDOG_TIMER: Duration = Duration::from_secs(1);

//
// Task main runner and related timers.
//

// Timer for the StatusDisplay spend on and off when blinking.
pub const STATUS_DISPLAY_BLINK_TIMER: Duration = Duration::from_millis(100);

// Timer for the StatusDisplay to pause between doing work.  Must be less
// than the minimum time the status LED can be on off, and when the code
// changes the device status, it will take up to this time for the new status
// to be applied.
pub const STATUS_DISPLAY_TIMER: Duration = Duration::from_millis(50);

// How often we aim to log from our primary loops to prove they are still
// alive.
pub const LOOP_LOG_INTERVAL: Duration = Duration::from_secs(5);

/// How often the protocol handler task pauses so other tasks can run.  We
/// make this low, so it can shuffle data between the USB endpoint and the
/// protocol handler, but it doesn't need to be a low as the
/// USB_DATA_TRANSFER_WAIT_TIMER, as the USB endpoint deals with 64 bytes at
/// a time, whereas that may deal with single bytes at a time.
pub const PROTOCOL_LOOP_TIMER: Duration = Duration::from_micros(10);

/// How long to wait between checking whether a byte has become available in
/// the USB data transfer object.  This is as low as we an make it, because
/// we're handling individual bytes.
pub const USB_DATA_TRANSFER_WAIT_TIMER: Duration = Duration::from_micros(1);

// How long the WiFi Control task will wait before feeding the watchdog.
pub const WIFI_CONTROL_WAIT_TIMER: Duration = Duration::from_millis(100);

// How long to yield in Commodore drive protocol methods.  This matches the
// value using the xum1541 (iec.c iec_wait()).
pub const PROTOCOL_YIELD_TIMER_US: u64 = 10;

// How long to yield in Commodore drive protocol methods.
pub const PROTOCOL_YIELD_TIMER: Duration = Duration::from_micros(PROTOCOL_YIELD_TIMER_US);

//
// USB device configuration constants.
//

/// USB Descriptor information - what current in mA this device draws.  We'll
/// be conservative (i.e. high) and suggest it might draw as much as 500mA.
pub const USB_POWER_MA: u16 = 500;

/// USB Descriptor information - maximum endpoint 0 (control endpoint)
/// packet size.
pub const MAX_PACKET_SIZE_0: u8 = 64;

/// USB Descriptor information - maximum vendor endpoint packet sizes.
pub const MAX_EP_PACKET_SIZE: u16 = 64;
pub const MAX_EP_PACKET_SIZE_USIZE: usize = MAX_EP_PACKET_SIZE as usize;

/// OUT (Host to Devive) endpoint number.  Note on the Pi this cannot be
/// larger than 0x0F, as the Pi only supports up to 16 endpoints in its
/// hardware registers.  If it is, the firmware will panic during the
/// endpoint allocation, as the endpoint cannot be allocated.
#[cfg(feature = "compatibility")]
pub const XUM1541_OUT_EP: u8 = 0x04;
#[cfg(feature = "extended")]
pub const PICO1541_OUT_EP: u8 = 0x01;

/// IN (Device to Host) endpoint number/  As above, this cannot be larger
/// that 0x0F.
#[cfg(feature = "compatibility")]
pub const XUM1541_IN_EP: u8 = 0x83;
#[cfg(feature = "extended")]
pub const PICO1541_IN_EP: u8 = 0x81;

/// USB Descriptor information - Vendor ID and Product ID
#[cfg(feature = "compatibility")]
pub const XUM1541_VENDOR_ID: u16 = 0x16d0;
#[cfg(feature = "compatibility")]
pub const XUM1541_PRODUCT_ID: u16 = 0x0504;
#[cfg(feature = "extended")]
pub const PICO1541_VENDOR_ID: u16 = 0x1209;
#[cfg(feature = "extended")]
pub const PICO1541_PRODUCT_ID: u16 = 0xf541;

/// USB Descriptor information - manufacturer string
pub const MANUFACTURER: &str = "piers.rocks";

/// USB Descriptor info - product string
#[cfg(feature = "compatibility")]
pub const XUM1541_PRODUCT: &str = "xum1541 floppy adapter (pico1541)";
#[cfg(feature = "extended")]
pub const PICO1541_PRODUCT: &str = "pico1541 floppy adapter";

/// USB Descriptor info - serial number string
#[cfg(feature = "compatibility")]
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
#[allow(dead_code)]
pub const MAX_READ_SIZE: u16 = 32768;

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
#[cfg(feature = "compatibility")]
pub const FIRMWARE_VERSION: u8 = XUM1541_FIRMWARE_VERSION;
#[cfg(feature = "extended")]
pub const PICO1541_FIRMWARE_VERSION: u8 = 1;
#[cfg(feature = "extended")]
pub const FIRMWARE_VERSION: u8 = PICO1541_FIRMWARE_VERSION;

//
// Pin constants
//
// Not all pins have constants - see gpio.rs for other pin assignments
//

/// WiFi power pin
pub const WIFI_PWR_PIN: u8 = 23;

/// WiFi chip select pin
pub const WIFI_CS_PIN: u8 = 25;

/// WiFi SPI DIO pin
pub const WIFI_DIO_PIN: u8 = 24;

/// WiFi SPI CLK pin
pub const WIFI_CLK_PIN: u8 = 29;

/// ADC pin used to check voltage for WiFI capability detection
#[allow(dead_code)] // Only used within const_assert!()
pub const WIFI_DETECT_ADC_PIN: u8 = 29;

//
// Other constants
//

// Const for the data buffer size within UsbDataTransfer
pub const TRANSFER_DATA_BUFFER_SIZE: usize = MAX_EP_PACKET_SIZE_USIZE * 2;
const_assert!(MAX_EP_PACKET_SIZE_USIZE < TRANSFER_DATA_BUFFER_SIZE);

/// The size (in bytes) of the stack for core 1.
pub const CORE1_STACK_SIZE: usize = 4096;

// Number of MAX_READ_SIZE size buffers uses in the READ_DATA channel .  We'll
// stick with 1 for now, so we can only ever have a single (64-byte) packet
// outstanding
pub const READ_DATA_CHANNEL_SIZE: usize = 1;

/// Total number of "generic" GPIOs - those which are exposed from the Pico
/// externally, for general purposes, not including WiFi or ADC pins.
pub const TOTAL_GENERIC_GPIOS: usize = 23; // 0-22 inclusive
