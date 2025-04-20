//! Implements task handling support, including dual core support.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use embassy_executor::{Executor, Spawner};
use embassy_rp::multicore::{Stack, spawn_core1 as rp_spawn_core1};
use embassy_rp::peripherals::CORE1;
use static_cell::{ConstStaticCell, StaticCell};

use crate::constants::CORE1_STACK_SIZE;
use crate::infra::watchdog::WatchdogType;
use crate::protocol::protocol_handler_task;
use crate::usb::bulk_task;
use crate::util::time::yield_us;

// See [`README.md`](README.md#execution-and-task-model) for a description of
// the execution and task model.

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
pub fn core1_spawn(p_core1: CORE1, watchdog: &'static WatchdogType) {
    rp_spawn_core1(p_core1, CORE1_STACK.take(), move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            spawn_or_panic(spawner.spawn(core1_main(watchdog)), "Core 1");
        })
    });

    // We don't need to yield after spawning core 1, as it will start running
    // immediately.  We also don't need to yield after the above embeddeed
    // spawn_or_panic, because there will be nothing else to run on core 1
    // other than what we spawned.
}

// Initial function for core 1.  This gets run using the executor on core 1,
// and spawns core 1's tasks.
#[embassy_executor::task]
pub async fn core1_main(watchdog: &'static WatchdogType) {
    // Get the core number, in order to log it.
    let core: u32 = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Core 1 main started", core);

    // Get the spawner
    let spawner = Spawner::for_current_executor().await;

    // Spawn the Protocol Handler task.  Do this first, so the Bulk runner
    // task can access the protocol handler.
    spawn_or_panic_yield(
        spawner.spawn(protocol_handler_task(watchdog)),
        "Protocol Handler",
    )
    .await;

    // Spawn the Bulk runner task.
    spawn_or_panic_yield(spawner.spawn(bulk_task()), "Bulk").await;
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
/// spawn_or_panic(spawner.spawn(my_task()), "my_task").await;
/// ```
pub async fn spawn_or_panic_yield<T, E: defmt::Format>(
    spawn_result: Result<T, E>,
    task_name: &str,
) {
    spawn_or_panic(spawn_result, task_name);

    // Yield to allow the task that has just been spawned to actually be
    // creatd and run.  We chose 250us as 100us seems to give enough time,
    // and 50us doesn't.  So we added some headroom onto 100us.  This method
    // is only used at start of delay, and delaying boot by 250us for each
    // task that is started is not meaningful.
    yield_us!(250);
}

/// Syncronous version of spawn_or_panic_yield.  This is used when spawning
/// the first task on core 1, as we're in a non-async context there, and it's
/// not necessary to yield, as there's nothing else on the core to run.
pub fn spawn_or_panic<T, E: defmt::Format>(spawn_result: Result<T, E>, task_name: &str) {
    match spawn_result {
        Ok(_) => {
            let core: u32 = embassy_rp::pac::SIO.cpuid().read();
            debug!("Core{}: Spawned task {}", core, task_name);
        }
        Err(e) => {
            error!("Failed to spawn task: {}, error: {}", task_name, e);
            panic!("Failed to spawn task: {}", task_name);
        }
    }
}
