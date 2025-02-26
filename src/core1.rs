//! Implements a task on core 1.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1 as rp_spawn_core1, Stack};
use embassy_rp::peripherals::CORE1;
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;

use crate::constants::CORE1_STACK_SIZE;
use crate::watchdog::{feed_watchdog, TaskId};

// A stack for core 1.  We're not wrapping it in anything, but we'll use an
// unsafe block when we retrieve this in spawn_core1().  This is a private
// static so no other modules can access it.
static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();

// An executor for core 1.
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// We're going to spawn a task on core 1.  We need to pass it a stack, which we
// have declared as a static.  We haven't bothered to mutex protect it, so we
// need to use an unsafe block to access it.
#[allow(static_mut_refs)]
pub fn spawn_core1(p_core1: CORE1) {
    rp_spawn_core1(p_core1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| spawner.spawn(core1_task()).unwrap())
    });
}

#[embassy_executor::task]
async fn core1_task() -> ! {
    loop {
        info!("Core 1 task");

        // Feed the watchdog
        feed_watchdog(TaskId::Core1);

        // Wait a while
        Timer::after(Duration::from_secs(5)).await;
    }
}
