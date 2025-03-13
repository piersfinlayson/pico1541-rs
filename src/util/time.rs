//! This file implements custom timing functions and macros used by pico1541
//!
//! In some cases we reimplement embassy-time functions, primarily to make it
//! clear what they are doing under the covers, but in some cases (like
//! delay_ns) because we can't get sub 1us delays with the embassy-time code
//! for Pico.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_time::{Duration, Instant};

/// Functiom to block until a specific instant.  This is similar to the
/// embassy-time::Delay::block_for function.
///
/// We always inline it to reduce function call/return overhead, as this is
/// typically used in timing critical functions
#[inline(always)]
pub fn block_until(expires: Instant) {
    while Instant::now() < expires {}
}

/// Function to block for a specific Duration.  This is similar to the
/// embassy-time::Delay::block_for function.
///
/// We always inline it to reduce function call/return overhead, as this is
/// typically used in timing critical functions
#[inline(always)]
pub fn block_for(duration: Duration) {
    block_until(Instant::now() + duration);
}

/// This function blocks for a specific number of nanoseconds.  This won't be
/// precise, as we're working with clock cycles, which take ~8ns on the Pico
/// and ~6 on the Pico 2, but it's better than using embassy-time, which can't
/// handle below the embassy-rp Pico tickrate of 1MHz.  Hence we need to
/// implement something ourselves.
///
/// We implement as a macro rather than a function to allow $ns to be used in a
/// compile time assert.
macro_rules! block_ns {
    ($ns:expr) => {{
        // Set clock frequency based on cargo features
        #[cfg(feature = "pico")]
        const CLOCK_FREQ_MHZ: u32 = 125;

        #[cfg(feature = "pico2")]
        const CLOCK_FREQ_MHZ: u32 = 150;

        // Calculate ns per cycle
        const NS_PER_CYCLE: u32 = 1000 / CLOCK_FREQ_MHZ;

        // Minimum practical delay (estimated overhead of the function call)
        #[allow(dead_code)]
        const MIN_PRACTICAL_NS: u32 = 3 * NS_PER_CYCLE;

        // Compile-time assertion to check if delay is too short
        static_assertions::const_assert!($ns >= MIN_PRACTICAL_NS);

        // Convert nanoseconds to cycles using div_ceil from integer_div_ceil
        // crate or a method that won't trigger Clippy warnings
        let cycles = {
            #[allow(clippy::manual_div_ceil)]
            let result = ($ns + NS_PER_CYCLE - 1) / NS_PER_CYCLE;
            result
        };

        // Use cortex_m assembly to burn exactly that number of cycles
        cortex_m::asm::delay(cycles);
    }};
}
pub(crate) use block_ns;

/// Block for a specific number of microseconds.
macro_rules! block_us {
    ($us:expr) => {
        crate::util::time::block_for(embassy_time::Duration::from_micros($us))
    };
}
pub(crate) use block_us;

/// Block for a specific number of milliseconds.
macro_rules! block_ms {
    ($ms:expr) => {
        crate::util::time::block_for(embassy_time::Duration::from_millis($ms))
    };
}
pub(crate) use block_ms;

/// Macro to briefly delay in order to let the bus lines settle.
macro_rules! iec_delay {
    () => {
        block_us!(2)
    };
}
pub(crate) use iec_delay;

/// Macro which yields to the scheduler for at least the specified time.  As
/// embassy will only come back to us after whatever has been scheduled has
/// paused, it could be longer than specified.  If you need a more accurate
/// pause, considering using block_us! instead.
macro_rules! yield_us {
    ($us:expr) => {
        embassy_time::Timer::after_micros($us).await
    };
}
pub(crate) use yield_us;

/// Macro which yields to the scheduler for at least the specified time.  As
/// embassy will only come back to us after whatever has been scheduled has
/// paused, it could be longer than specified.  If you need a more accurate
/// pause, considering using block_ms! instead.
macro_rules! yield_ms {
    ($ms:expr) => {
        embassy_time::Timer::after_millis($ms).await
    };
}
pub(crate) use yield_ms;

/// Macro which yields to the scheduler for at least the specified time.  As
/// embassy will only come back to us after whatever has been scheduled has
/// paused, it could be longer than specified.  If you need a more accurate
/// pause, considering using block_for() instead.
macro_rules! yield_for {
    ($dur:expr) => {
        embassy_time::Timer::after($dur).await
    };
}
pub(crate) use yield_for;

pub mod iec {
    //! IEC protocol timers
    #![allow(dead_code)]

    use embassy_time::Duration;

    /// Yield timeout for bus free check - how long to wait before checking
    /// again the loop.
    pub const BUS_FREE_CHECK_YIELD: Duration = Duration::from_millis(1);

    /// Total time to wait for the bus to be free, unless we're waiting
    /// forever.
    pub const BUS_FREE_TIMEOUT: Duration = Duration::from_millis(1500);

    /// Time to wait for device to release DATA, after being told to LISTEN.
    pub const LISTENER_WAIT_INTERVAL: Duration = Duration::from_micros(1);

    /// Time to wait for CLK to be pulled down by the drive after telling it
    /// to enter TALK.
    pub const WRITE_TALK_CLK_TIMEOUT: Duration = Duration::from_secs(1);

    /// Time to wait for CLK to be released at the beginning on the main CBM
    /// protocol read loop, before reading a byte.
    pub const READ_CLK_START_TIMEOUT: Duration = Duration::from_secs(1);

    /// Time to wait for drive to pull CLK low after releasing DATA when
    /// reading a byte
    pub const READ_CLK_TIMEOUT: Duration = Duration::from_micros(400);

    /// A "forever" timeout.  We can't use Duration::MAX, as that's a u64, and
    /// causes the underlying embassy_time methods to panic.  So we set it to a
    /// year (ish).
    pub const FOREVER_TIMEOUT: Duration = Duration::from_secs(60 * 60 * 24 * 365);

    // A set of low-level IEC bus timings, taken from xum1541.  All are in us.

    /// Max ATN response required time
    pub const IEC_T_AT: u64 = 1000;

    // Max listener hold off time
    // pub const IEC_T_LH: u64 = infinite;

    /// Typical non-EOI response to RFD time
    pub const IEC_T_NE: u64 = 40;

    /// Min talker bit setup time (70 typical)
    pub const IEC_T_S: u64 = 20;

    /// Min data valid time (20 typical)
    pub const IEC_T_V: u64 = 20;

    /// Max frame handshake time (20 typical)
    pub const IEC_T_F: u64 = 1000;

    /// Min frame to release of ATN time
    pub const IEC_T_R: u64 = 20;

    /// Min time between bytes
    pub const IEC_T_BB: u64 = 100;

    /// Min EOI response time (250 typical)
    pub const IEC_T_YE: u64 = 200;

    /// Min EOI response hold time
    pub const IEC_T_EI: u64 = 60;

    /// Max talker response limit (30 typical)
    pub const IEC_T_RY: u64 = 60;

    /// Min byte acknowledge hold time (30 typical)
    pub const IEC_T_PR: u64 = 20;

    // 20/30/100 talk-attention release time.  It is unclear what this value
    // does in the original xum1541 source code - it calls AVR _delay_us(-1)
    // which is undefined.
    // pub const IEC_T_TK: u64 = -1;

    // Talk attention acknowledge time
    // pub const IEC_T_DC: u64 = infinite;

    /// Min talk-attention ack hold time
    pub const IEC_T_DA: u64 = 80;

    /// Min EOI acknowledge time
    pub const IEC_T_FR: u64 = 60;
}
