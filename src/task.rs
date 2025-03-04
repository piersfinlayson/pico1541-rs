//! Implements task handling support, including dual core support.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{spawn_core1 as rp_spawn_core1, Stack};
use embassy_rp::peripherals::CORE1;
use static_cell::{ConstStaticCell, StaticCell};

use crate::bulk::bulk_task;
use crate::constants::CORE1_STACK_SIZE;
use crate::protocol::protocol_handler_task;
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
// Core 1 handles bulk USB transfers and all Commodore protocol handling.  This
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

//
// Statics
//

// A stack for core 1.  We will take it and use it mutably in core1_spawn.
static CORE1_STACK: ConstStaticCell<Stack<CORE1_STACK_SIZE>> = ConstStaticCell::new(Stack::new());

// An executor for core 1.
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// We're going to spawn a task on core 1.  This requires an executor, which
// needs to live for infinity, hence we have declared it as a static above.
// We need to pass it a stack, which we have also declared as a static.
#[allow(static_mut_refs)]
pub fn core1_spawn(p_core1: CORE1) {
    rp_spawn_core1(p_core1, CORE1_STACK.take(), move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            spawn_or_reboot(spawner.spawn(core1_main()), "Core 1");
        })
    });
}

// Initial function for core 1.  This gets run using the executor on core 1,
// and spawns core 1's tasks.
#[embassy_executor::task]
pub async fn core1_main() {
    // Get the core number, in order to log it.
    let core: u32 = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Core 1 main started", core);

    // Get the spawner
    let spawner = Spawner::for_current_executor().await;

    // Spawn the Protocol Handler task.  Do this first, so the Bulk runner
    // task can access the protocol handler.
    spawn_or_reboot(spawner.spawn(protocol_handler_task()), "Protocol Handler");

    // Spawn the Bulk runner task.
    spawn_or_reboot(spawner.spawn(bulk_task()), "Bulk");
}

/// Method to spawn tasks.  Can be called on either core.
///
/// Using the Spawner object to spawn can fail, becayse too many instances of
/// that task are already running.  By default only 1 is alllowed at once, but
/// is configurable with e.g. #[embassy_executor::task(pool_size = 4).
///
/// We handle that by rebooting - but it shouldn't happen if processes are only
/// spawned at start of day.
///
/// Example:
/// ```ignore
/// spawn_or_reboot(spawner.spawn(my_task()), "my_task");
/// ```
pub fn spawn_or_reboot<T, E: defmt::Format>(spawn_result: Result<T, E>, task_name: &str) {
    match spawn_result {
        Ok(_) => {
            let core: u32 = embassy_rp::pac::SIO.cpuid().read();
            debug!("Core{}: Spawned task {}", core, task_name);
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
