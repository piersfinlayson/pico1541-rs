//! Objects for pico1541 emulated device tests.
//!
//! Usage is like this:
//! ```rust
//! #![no_std]
//! #![no_main]
//! 
//! use embassy_executor::Spawner;
//! use pico1541_rs::test::device::{IecDev, run_iec_device};
//! 
//! static mut DEVICE: Option<IecDev> = None;
//! 
//! #[embassy_executor::main]
//!     async fn main(spawner: Spawner) -> ! {
//!     let p = embassy_rp::init(Default::default());
//!     unsafe {
//!         DEVICE = Some(IecDev::new(p, 8));  // Device 8
//!         if let Some(ref mut device) = DEVICE {
//!             spawner.spawn(run_iec_device(device)).unwrap();
//!         }
//!     }
//!     loop {}
//! }
//! ```

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use defmt::{debug, info, trace, warn};
use embassy_futures::select::{Either::First, select};
use embassy_rp::Peripherals;
use embassy_rp::gpio::{AnyPin, Drive, Flex, Input, Pull};
use embassy_time::{Duration, Timer, with_timeout};
use crate::test::pins::{IEC_PINS_IN, IEC_PINS_OUT};

/// A line on the IEC bus
pub struct BusLine {
    input: Input<'static>,
    output: Flex<'static>,
    is_output: bool,
}

impl BusLine {
    /// Create a new `BusLine`
    /// 
    /// # Arguments
    /// - `input_pin`: The pin to use for input
    /// - `output_pin`: The pin to use for output
    /// 
    /// # Returns
    /// A new `BusLine` object
    /// 
    /// # Example
    /// ```rust
    /// let clock_line = BusLine::new(p.PIN_19.into(), p.PIN_11.into());
    /// ```
    #[must_use]
    pub fn new(input_pin: AnyPin, output_pin: AnyPin) -> Self {
        let input = Input::new(input_pin, Pull::Down);
        let mut output = Flex::new(output_pin);
        output.set_as_input();
        output.set_pull(Pull::None);
        Self {
            input,
            output,
            is_output: false,
        }
    }

    /// Set the line as an output
    /// 
    /// # Arguments
    /// - `high`: If true, set the line high, otherwise set it low 
    pub fn set_as_output(&mut self, high: bool) {
        if high {
            self.output.set_high();
        } else {
            self.output.set_low();
        }
        self.output.set_drive_strength(Drive::_4mA);
        self.output.set_as_output();
        self.is_output = true;
    }

    /// Set the output level of the line
    /// 
    /// # Arguments
    /// - `high`: If true, set the line high, otherwise set it low
    /// 
    /// # Panics
    /// Panics if `set()` is called on an input pin - call `set_as_output()`
    /// first (which also sets the level).
    pub fn set(&mut self, high: bool) {
        assert!(self.is_output, "set() called on input pin");
        // Invert
        if high {
            self.output.set_low();
        } else {
            self.output.set_high();
        }
    }

    /// Set the line as an input.
    /// 
    /// No pull-up or pull-down resistors are set.
    pub fn set_as_input(&mut self) {
        self.output.set_as_input();
        self.output.set_pull(Pull::None);
        self.is_output = false;
    }

    /// Get the input line level
    /// 
    /// # Returns
    /// - `true` if the line is high, `false` if it is low
    /// 
    /// # Panics
    /// Panics if `get()` is called on an output pin - call `set_as_input()`
    /// first.
    pub fn get(&mut self) -> bool {
        assert!(!self.is_output, "get() called on output pin");
        self.input.is_high()
    }

    /// Wait for the line to go low
    /// 
    /// # Panics
    /// Panics if `wait_for_low()` is called on an output pin - call
    /// `set_as_input()` first.
    pub async fn wait_for_low(&mut self) {
        assert!(!self.is_output, "wait_for_low() called on output pin");
        self.input.wait_for_high().await;
    }

    /// Wait for the line to go high
    /// 
    /// # Panics
    /// Panics if `wait_for_high()` is called on an output pin - call
    /// `set_as_input()` first.
    pub async fn wait_for_high(&mut self) {
        assert!(!self.is_output, "wait_for_high() called on output pin");
        self.input.wait_for_low().await;
    }
}

// Protocol constants
const IEC_LISTEN: u8 = 0x20;
const IEC_TALK: u8 = 0x40;
const IEC_UNLISTEN: u8 = 0x3F;
const IEC_UNTALK: u8 = 0x5F;
const IEC_OPEN_DATA: u8 = 0x60;
const IEC_CLOSE: u8 = 0xE0;
const IEC_OPEN: u8 = 0xF0;

enum DevSecCmd {
    OpenData { channel: u8 },
    Close { channel: u8 },
    Open { channel: u8, _bytes: [u8; 4] },
}

impl DevSecCmd {
    /// Create a `DevSecCmd` from the received bytes
    /// 
    /// # Arguments
    /// - `bytes`: The bytes received from the IEC bus
    /// 
    /// # Returns
    /// A `Result` containing the `DevSecCmd` or an error
    #[allow(clippy::result_unit_err)]
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ()> {
        if bytes.len() < 2 {
            warn!("Invalid secondary command length");
            Err(())
        } else if bytes[1] & IEC_OPEN_DATA == IEC_OPEN_DATA {
            let channel = bytes[1] & 0x0F;
            Ok(DevSecCmd::OpenData { channel })
        } else if bytes[1] & IEC_CLOSE == IEC_CLOSE {
            let channel = bytes[1] & 0x0F;
            Ok(DevSecCmd::Close { channel })
        } else if bytes[1] & IEC_OPEN == IEC_OPEN {
            let channel = bytes[1] & 0x0F;
            let mut store_bytes = [0; 4];
            for (ii, byte) in bytes.iter().enumerate().skip(2) {
                trace!("  extra byte {}: 0x{:02x}", ii, bytes);
                store_bytes[ii] = *byte;
            }
            Ok(DevSecCmd::Open {
                channel,
                _bytes: store_bytes,
            })
        } else {
            warn!("Invalid secondary command 0x{:02x}", bytes[0]);
            Err(())
        }
    }
}

enum DevCmd {
    Talk {
        device: u8,
        second: DevSecCmd,
    },
    Untalk,
    Listen {
        device: u8,
        second: DevSecCmd,
    },
    Unlisten,
}

impl DevCmd {
    fn get_talk_listen_device(command: &[u8]) -> u8 {
        assert!(
            !command.is_empty(),
            "get_talk_listen_device called with empty command"
        );
        command[0] & 0x1F
    }

    #[allow(dead_code)]
    fn get_open_close_device(command: &[u8]) -> u8 {
        assert!(
            !command.is_empty(),
            "get_open_close_device called with empty command"
        );
        command[0] & 0x0F
    }

    /// Create a `DevCmd` from the received bytes
    /// 
    /// # Arguments
    /// - `bytes`: The bytes received from the IEC bus
    /// 
    /// # Returns
    /// A `Result` containing the `DevCmd` or an error
    /// 
    /// # Panics
    /// Panics if `from_bytes()` is called with an empty byte array
    #[allow(clippy::result_unit_err)]
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, ()> {
        assert!(!bytes.is_empty(), "from_bytes called with empty bytes");
        if bytes[0] == IEC_UNLISTEN {
            Ok(DevCmd::Unlisten)
        } else if bytes[0] == IEC_UNTALK {
            Ok(DevCmd::Untalk)
        } else if bytes[0] & IEC_TALK == IEC_TALK {
            Ok(DevCmd::Talk {
                device: Self::get_talk_listen_device(bytes),
                second: DevSecCmd::from_bytes(bytes)?,
            })
        } else if bytes[0] & IEC_LISTEN == IEC_LISTEN {
            Ok(DevCmd::Listen {
                device: Self::get_talk_listen_device(bytes),
                second: DevSecCmd::from_bytes(bytes)?,
            })
        } else {
            warn!("Invalid command: 0x{:02x}", bytes[0]);
            Err(())
        }
    }

    /// Check if the command is for us
    /// 
    /// # Returns
    /// `true` if the command is for us, `false` if it is not
    #[must_use]
    pub fn for_us(&self, dev_id: u8) -> bool {
        match self {
            DevCmd::Talk { device, .. } | DevCmd::Listen { device, .. } => {
                *device == dev_id
            }
            _ => true,
        }
    }
}

// Boot status
const STATUS_STR_OK: &str = "ok";
const STATUS_STR_BOOT: &str = "pico1541 test drive";

// Status numbers
const STATUS_NUM_OK: u8 = 0;
const STATUS_NUM_BOOT: u8 = 73;

struct DevStatus {
    num: u8,
    string: &'static str,
    track: u8,
    sector: u8,
}

impl DevStatus {
    #[must_use]
    /// Create a new `DevStatus` object
    /// 
    /// # Arguments
    /// - `num`: The status number
    /// - `track`: The track number
    /// - `sector`: The sector number
    pub fn new(num: u8, track: u8, sector: u8) -> Self {
        let string = match num {
            STATUS_NUM_OK => STATUS_STR_OK,
            STATUS_NUM_BOOT => STATUS_STR_BOOT,
            _ => "unknown",
        };

        DevStatus {
            num,
            string,
            track,
            sector,
        }
    }
}

/// A device (as opposed to a controller) on the IEC bus
pub struct IecDev {
    device_id: u8,
    clock: BusLine,
    data: BusLine,
    atn: Option<BusLine>,
    status: DevStatus,
}

impl IecDev {
    const MAX_RECV_BYTES: usize = 64;

    /// Create a new `IecDev` object
    /// 
    /// # Arguments
    /// - `p`: The embassy peripherals to use
    /// - `device_id`: The device ID to use for this device
    /// 
    /// # Returns
    /// A new `IecDev` object
    /// 
    /// # Panics
    /// Panics if the pin numbers do not match the expected values - this would
    /// be a coding error.
    #[must_use]
    pub fn new(p: Peripherals, device_id: u8) -> Self {
        let clock = BusLine::new(p.PIN_19.into(), p.PIN_11.into());
        assert_eq!(IEC_PINS_IN.clock, 19);
        assert_eq!(IEC_PINS_OUT.clock, 11);

        let data = BusLine::new(p.PIN_20.into(), p.PIN_13.into());
        assert_eq!(IEC_PINS_IN.data, 20);
        assert_eq!(IEC_PINS_OUT.data, 13);

        let atn = BusLine::new(p.PIN_17.into(), p.PIN_12.into());
        assert_eq!(IEC_PINS_IN.atn, 17);
        assert_eq!(IEC_PINS_OUT.atn, 12);
        let atn = Some(atn);

        let status = DevStatus::new(STATUS_NUM_BOOT, 0, 0);

        IecDev { device_id, clock, data, atn, status }
    }

    async fn receive_bytes(&mut self, bytes: &mut [u8; Self::MAX_RECV_BYTES]) -> Result<usize, ()> {
        let mut byte_num = 0;
        loop {
            let (eoi, byte) = self.receive_byte().await?;
            bytes[byte_num] = byte;

            byte_num += 1;

            if byte_num >= Self::MAX_RECV_BYTES {
                warn!("Received maximum number of bytes");
                break;
            }

            if eoi {
                debug!("EOI");
                break;
            }
        }

        Ok(byte_num)
    }

    // We remove atn from the IecDev struct so we can run a select on it
    // and self simultaneouslty - otherwise we would need to borrow self
    // mutably twice.
    async fn run(&mut self) -> ! {
        // Set all lines to inputs
        trace!("Releasing CLK, DATA, ATN");
        self.clock.set_as_input();
        self.data.set_as_input();

        let mut atn = self.atn.take().unwrap();
        atn.set_as_input();

        let mut wait_atn = true;
        loop {
            if wait_atn {
                // Wait for ATN to be asserted
                trace!("Wait for ATN");
                atn.wait_for_low().await;
                trace!("ATN asserted");
            }

            // Read bytes
            let mut bytes = [0; Self::MAX_RECV_BYTES];
            let num_bytes = self.receive_bytes(&mut bytes).await.unwrap();

            debug!("Received {} bytes", num_bytes);
            let mut byte_num = 0;
            loop {
                debug!("Byte {}: 0x{:02x}", byte_num, bytes[byte_num]);
                byte_num += 1;
                if byte_num >= num_bytes {
                    break;
                }
            }

            // Handle the bytes - either:
            // - The message (TALK, LISTEN, UNTALK or UNLISTEN) wasn't for us
            // - It was a TALK for us
            // - It was a LISTEN for us
            // - It was an UNTALK for us
            // - It was an UNLISTEN for us
            //
            // Do this within a select so ATN will interrupt us if necessary.
            if let First(()) = select(
                atn.wait_for_low(),
                self.handle_atn_bytes(&bytes[0..num_bytes]),
            )
            .await
            {
                warn!("ATN asserted while processing previous ATN");
                wait_atn = false;
            } else {
                wait_atn = true;
            }
        }
    }

    async fn handle_atn_bytes(&mut self, bytes: &[u8]) -> Result<(), ()> {
        assert!(!bytes.is_empty(), "get_command called with empty bytes");

        // We will be holding DATA low when we enter this function - we need to
        // release DATA if we're not a listener.
        // If we've been told to TALK, we wait for CLK to be released, then
        // release DATA, then pull down CLK - this makes us the talker.

        // Get the command
        let command = DevCmd::from_bytes(bytes)?;

        if !command.for_us(self.device_id) {
            // If the command is not for us, we need to release DATA
            trace!("Release DATA");
            self.data.set_as_input();
            return Ok(());
        }

        match command {
            DevCmd::Talk { second, .. } => {
                trace!("TALK");
                self.talk(&second).await
            }
            DevCmd::Untalk => {
                trace!("UNTALK");
                self.untalk()
            }
            DevCmd::Listen { second, .. } => {
                trace!("LISTEN");
                self.listen(&second).await
            }
            DevCmd::Unlisten => {
                trace!("UNLISTEN");
                self.unlisten()
            }
        }
    }

    #[allow(clippy::unnecessary_wraps)]
    fn untalk(&mut self) -> Result<(), ()> {
        info!("UNTALK");
        self.data.set_as_input();
        Ok(())
    }

    #[allow(clippy::unnecessary_wraps)]
    fn unlisten(&mut self) -> Result<(), ()> {
        info!("UNLISTEN");
        self.data.set_as_input();
        Ok(())
    }

    #[allow(clippy::unnecessary_wraps)]
    async fn listen(&mut self, second: &DevSecCmd) -> Result<(), ()> {
        match second {
            DevSecCmd::OpenData { channel } => {
                info!("LISTEN OPEN DATA for channel {}", channel);
            }
            DevSecCmd::Close { channel } => {
                info!("LISTEN CLOSE for channel {}", channel);
            }
            DevSecCmd::Open { channel, _bytes: _ } => {
                info!("LISTEN OPEN for channel {}", channel);
            }
        }

        loop {
            let (eoi, byte) = self.receive_byte().await?;
            debug!("Received byte: 0x{:02x}", byte);
            if eoi {
                debug!("EOI");
                break;
            }
        }

        Ok(())
    }

    #[allow(clippy::unnecessary_wraps)]
    async fn talk(&mut self, second: &DevSecCmd) -> Result<(), ()> {
        // Wait for CLK to go high
        trace!("Wait for CLK deasserted");
        self.clock.wait_for_high().await;

        // Release DATA
        trace!("Release DATA");
        self.data.set_as_input();

        // Pull CLK low
        self.clock.set_as_output(false);

        // We're now ready to send data

        match second {
            DevSecCmd::OpenData { channel } => {
                info!("TALK OPEN DATA for channel {}", channel);
                if *channel == 15 {
                    self.send_status().await?;
                }
            }
            DevSecCmd::Close { channel } => {
                info!("TALK CLOSE for channel {}", channel);
            }
            DevSecCmd::Open { channel, _bytes: _ } => {
                info!("TALK OPEN for channel {}", channel);
            }
        }
        Ok(())
    }

    async fn receive_byte(&mut self) -> Result<(bool, u8), ()> {
        // Wait for CLK to go low
        trace!("Wait for CLK");
        self.clock.wait_for_low().await;

        // Pull DATA low to signify we're ready to receive
        trace!("Assert DATA");
        self.data.set_as_output(false);

        // Wait for CLK to go high and release DATA
        trace!("Wait for CLK deasserted");
        self.clock.wait_for_high().await;
        self.data.set_as_input();

        // Now the sender will pull CLK low within 200us, or it won't.  If
        // it doesn't, that's EOI.
        let mut eoi = false;
        if with_timeout(Duration::from_micros(200), self.clock.wait_for_low())
            .await
            .is_err()
        {
            trace!("EOI");
            eoi = true;

            // Pull DATA low for 60us to show we've spotted EOI
            self.data.set_as_output(false);
            Timer::after_micros(60).await;

            // Wait for CLK to go high and release DATA
            trace!("(EOI) Wait for CLK deasserted");
            self.clock.wait_for_high().await;
            self.data.set_as_input();
        }

        // Read the byte
        let mut byte: u8 = 0;
        let mut bit_num: u8 = 0;
        loop {
            // Wait for CLK to go low to signify a bit is waiting
            trace!("(Bit {}) Wait for CLK", bit_num);
            self.clock.wait_for_low().await;

            // Read data bit
            trace!("Read data bit");
            let rbit = self.data.get();
            if !rbit {
                // Low - so a 1
                byte |= 1 << bit_num;
            }

            // Wait for CLK to go high
            trace!("Wait for CLK deasserted");
            self.clock.wait_for_high().await;

            bit_num += 1;
            if bit_num >= 8 {
                break;
            }
        }

        Ok((eoi, byte))
    }

    #[allow(clippy::unnecessary_wraps)]
    async fn send_byte(&mut self, byte: u8, last: bool) -> Result<(), ()> {
        // Set CLK high
        self.clock.set(true);

        // Wait for DATA to go high
        self.data.wait_for_high().await;

        if last {
            // Signal EOI by not pulling CLK low - the listener should notice
            // and pull DATA low.
            trace!("Signal EOI");
            self.data.wait_for_low().await;
        }

        // Pull CLK low
        self.clock.set(false);

        let mut bit_num: u8 = 0;
        loop {
            // Set the DATA line to signal the bit
            let bit = (1 << bit_num) & byte;
            if bit == 0 {
                self.data.set_as_output(true);
            } else {
                self.data.set_as_output(false);
            }

            // Set CLK high and wait for 60us (to support the C64)
            self.clock.set(true);
            Timer::after_micros(60).await;

            // Set CLK low and set DATA high
            self.clock.set(false);
            self.data.set_as_input();

            bit_num += 1;

            if bit_num >= 8 {
                break;
            }
        }

        Ok(())
    }

    async fn send_decimal_2digits(&mut self, num: u8, last: bool) -> Result<(), ()> {
        if num > 99 {
            warn!("Sending a number > 100, {}, as 2 digits", num);
        }
        let tens = num / 10;
        let ones = num % 10;
        self.send_byte(b'0' + tens, false).await?;
        self.send_byte(b'0' + ones, last).await?;

        Ok(())
    }

    async fn send_status(&mut self) -> Result<(), ()> {
        // Send the status
        let result = self.send_status_ll().await;

        // Reset the status
        self.status = DevStatus::new(STATUS_NUM_OK, 0, 0);

        // Return the result of the send
        result
    }

    async fn send_status_ll(&mut self) -> Result<(), ()> {
        // Send status_num as 2 digits
        self.send_decimal_2digits(self.status.num, false).await?;

        // Send comma
        self.send_byte(b',', false).await?;

        // Send status string
        for &byte in self.status.string.as_bytes() {
            self.send_byte(byte, false).await?;
        }

        // Send comma
        self.send_byte(b',', false).await?;

        // Send track as 2 digits
        self.send_decimal_2digits(self.status.track, false).await?;

        // Send sector as 2 digits (not handling sectors > 99)
        self.send_decimal_2digits(self.status.sector, true).await?;

        Ok(())
    }
}

/// Run the IEC device
/// 
/// This is an embassy task, which can be spawned by the executor.
/// 
/// # Arguments
/// - `device`: A mutable reference to the `IecDev` object
#[embassy_executor::task]
pub async fn run_iec_device(device: &'static mut IecDev) -> ! {
    device.run().await
}
