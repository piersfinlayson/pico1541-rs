//! Implements task handling support.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1 as rp_spawn_core1, Stack};
use embassy_rp::peripherals::CORE1;
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::bulk::Bulk;
use crate::constants::CORE1_STACK_SIZE;
use crate::watchdog::reboot_normal;

// Threading and tasks model
//
// One the Pico we have 2 cores available - 0 and 1.
//
// Core 0 is the main core, and is where the main() function runs.  We use it
// for all of the non-protocol handling tasks, including
// - the embassy USB stack (which handles the USB protocol and control
//   handling)
// - the display task (which handles the status LED)
// - the watchdog task
//
// In the future, it is expected that core 0 will also gain WiFi and any other
// support.
//
// Core 1 handles bulk USB transfers and all Commodore protoco; handling.  This
// is an improvement over the situation on the stock xum1541, which only has a
// single core.  Hence the USB stack itself, control handling and the status
// LED are all handled on the same core as the protocol support.
//
// Tasks can be spawned only on core 0 using the Spawner object pass into
// main().  Core 1 tasks must be spawned via an Eexecutor.  We could use an
// executor to spawn a task on core 0 as well, but you can only spawn a single
// runner that way, which would mean handling scheduling for core 0's tasks.
// Hence we stick with the Spawner for core 0, and use the executor only for
// core 1
//
// Therefore, for consistency between the cores, we spawn tasks on voth cores
// using Executors.  These are stored by non-public statics, and hence can
// only be access via this module, which improved safety.

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
pub fn core1_spawn(p_core1: CORE1, bulk: &'static mut Bulk) {
    rp_spawn_core1(p_core1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|mut spawner| core1_main(&mut spawner, bulk))
    });
}

// Core 1's "main" function.  This gets run get the executor on core 1, and
// spawns core 1's tasks. 
pub fn core1_main(spawner: &Spawner, bulk: &'static mut Bulk) {
    let core: u32 = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Core 1 main started", core);

    // Spawn a dummy task
    spawn_or_reboot(spawner.spawn(core1_dummy()), "core1_dummy");

    // Spawn core'1 current task.
    spawner.spawn(core1_task(bulk)).unwrap();
}

#[embassy_executor::task]
async fn core1_dummy() -> ! {
    loop {
        Timer::after_micros(1000).await;
    }
}

#[embassy_executor::task]
async fn core1_task(bulk: &'static mut Bulk) -> ! {
    let core: u32 = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Bulk task started", core);

    bulk.run().await;
}

/// Method to spawn tasks.  Can be called on either core.
///
/// Using the Spawner object to spawn can fail, presumably because some memory
/// must be allocated.  We handle that by rebooting - but it shouldn't happen
/// if processes are only spawned at start of day.
///
/// Example:
/// ```ignore
/// spawn_or_reboot(spawner.spawn(my_task()), "my_task");
/// ```
pub fn spawn_or_reboot<T, E: defmt::Format>(spawn_result: Result<T, E>, task_name: &str) {
    match spawn_result {
        Ok(_) => {
            let core: u32 = embassy_rp::pac::SIO.cpuid().read();
            info!("Core{}: Spawned task {}", core, task_name);
        }
        Err(e) => {
            error!("Failed to spawn task: {}, error: {}", task_name, e);
            reboot_normal();
        }
    }
    if let Err(e) = spawn_result {
        error!("Failed to spawn task: {}, error: {}", task_name, e);
        reboot_normal();
    }
}
