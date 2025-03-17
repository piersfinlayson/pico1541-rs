# Developer's Guide

This document provides an overview of the pico1541-rs codebase, designed to help developers understand, modify, and extend the project.  It documents the module organization, key components, relationships between subsystems, and architectural decisions that shape the implementation.  This guide will help you navigate the codebase efficiently and understand the architectural and design principles behind it.

## Module Structure

### Overview

pico1541-rs is structured with the following top-level modules:

* [`lib`](lib.rs) - Top-level module definitions and firmware boot information
* [`constants`](constants.rs) - Project-wide constants
* [`entry`](entry.rs) - Core 0 main entry point and panic handlers
* [`task`](task.rs) - Task management and Core 1 executor
* [`infra`](infra/mod.rs) - Infrastructure components used throughout the project
* [`utils`](util/mod.rs) - Utility and miscellaneous functionality
* [`usb`](usb/mod.rs) - USB communication and data transfer handling
* [`wifi`](wifi/mod.rs) - WiFi support for compatible devices
* [`protocol`](protocol/mod.rs) - Protocols for Commodore disk drive communication

### Module Details

#### infra

The [`infra`](infra/mod.rs) module provides infrastructure used throughout the project:

* [`display`](infra/display.rs) - Implements a status display (status LED) via the [`StatusDisplay`] object, accessed through the [`STATUS_DISPLAY`] static and helper functions, allowing components to update device status.
* [`gpio`](infra/gpio.rs) - Implements the [`Gpio`] object, accessed via [`GPIO`] static to:
 * Contain the mapping of physical PINs to usage
 * Dynamically manage allocation of GPIO objects to other objects

#### utils

The [`utils`](util/mod.rs) module provides various utility and miscellaneous functionality:

* [`built`](util/built.rs) - Retrieves build information about the pico1541-rs crate and exposes it for inclusion in the source code, particularly for including build-time and other crate/application information in device logging.
* [`dev_info`](util/dev_info.rs) - Provides access to device information at runtime, particularly handling the retrieval of device serial numbers.
* [`time`](util/time.rs) - Provides functions and macros for both blocking (sync) and yielding (async) timers, and timeouts used by the Commodore IEC protocol.
* [`watchdog`](util/watchdog.rs) - Implements the [`Watchdog`] object, accessed via the [`WATCHDOG`] static and helper functions, to:
 * Manage interactions with the hardware watchdog
 * Allow tasks to register and deregister (i.e., be policed and unpoliced by the watchdog)
 * Allow processes to feed the watchdog
 * Reboot the system if a registered task fails to feed the watchdog in the required time
 * Provide additional functionality related to rebooting the system (such as entering DFU mode)

#### usb

The [`usb`](usb/mod.rs) module provides USB support:

* [`mod`](usb/mod.rs) - Implements the [`UsbStack`] object used to create the embassy_usb stack and provides the main USB task implementation.
* [`bulk`](usb/bulk.rs) - Implements the [`Bulk`] object, which handles bulk transfers on the OUT endpoint (from host to device), and passes them to [`ProtocolHandler`] using the [`WRITE_DATA_CHANNEL`]. Also provides the bulk task implementation.
* [`control`](usb/control.rs) - Implements the handling of USB control transfers for this vendor device via the [`Control`] object.
* [`transfer`](usb/transfer.rs) - Implements the [`UsbTransfer`] object, accessed via the [`USB_DATA_TRANSFER`] static and helper functions to handle individual transfer transactions (Read/Write or In/Out) spanning multiple USB bulk transfer messages. Used by [`Bulk`] and [`ProtocolHandler`].

#### wifi

The [`wifi`](wifi/mod.rs) module handles device WiFi support, including:
* Detecting whether WiFi is supported
* Initializing the WiFi stack
* Providing functionality to [`StatusDisplay`] to allow the Pico (2) W status LED attached to the WiFi chip to be flashed

#### protocol

The [`protocol`](protocol/mod.rs) module implements the protocols to communicate with Commodore disk drives. *(More detailed documentation to be added)*

## Architecture Principles

### Embassy-rs Foundation

pico1541-rs is built on the embassy-rs asynchronous framework, which fundamentally shapes our architectural approach. Embassy provides an async/await runtime specifically designed for embedded systems, offering several benefits that influenced our design:

- **Efficient asynchronous execution**: Embassy allows us to write non-blocking code that efficiently utilizes the limited resources of the RP2040 and RP2350 microcontrollers.

- **Task-based concurrency model**: Our codebase leverages embassy's task model to manage concurrent operations without the overhead of an RTOS, enabling responsive handling of USB communications, IEC protocol operations, and device status management.

- **HAL integration**: We utilize embassy's hardware abstraction layers for the RP2040 and RP2350, giving us a consistent interface to peripheral hardware while maintaining performance.

- **Time management**: Embassy's timer and time handling facilities provide the precise timing required for accurately implementing the Commodore IEC protocol timing requirements.  However, we tighter timing precision is required that embassy can provide, we use inline assembly and/or PIO implementations to achieve this. 

- **Static allocation**: Following embassy's design patterns, and as an embedded appliccation, we prefer static allocation over dynamic memory management to ensure predictable runtime behavior and avoid heap fragmentation.

Our implementation follows embassy-rs patterns consistently, with modules designed around:

- Static singletons for shared resources, initialized at startup
- Async functions for potentially blocking operations
- Channel-based communication between tasks
- Task spawning handled through our [`task`](src/task.rs) module abstractions
- Strategic distribution of workloads between the two RP2040 cores

Understanding embassy's execution model is essential for working with this codebase, as it influences everything from our initialization sequence to how we handle concurrent operations.

### Implementation Patterns

Throughout the codebase, several consistent implementation patterns are employed:

- **Static singletons pattern**: Critical system-wide resources are implemented as static singletons with controlled access patterns, enabling global access while maintaining safety through interior mutability.

- **Message-passing architecture**: Components communicate primarily through typed channels rather than shared mutable state, reducing concurrency issues and improving testability.

- **State machine modeling**: Protocol and device behaviors are modeled as explicit state machines with well-defined transitions and handlers.

- **Capability-based access**: Components are provided with specific interfaces that grant only the capabilities they need rather than direct access to underlying resources.

### Resource Management

Resource management follows specific patterns throughout the codebase:

- **Early initialization**: Critical resources are initialized at system startup before task spawning begins, ensuring they're available throughout the application lifecycle.

- **Ownership boundaries**: Each module clearly defines ownership of its resources, with explicit APIs for external components to request actions rather than directly manipulating internal state.

- **Error propagation**: Failures to initialize at startup generally panic.  During runtime, where errors can be handled they are.  Where they cannot they are logged using warn!() and error!().

### Static Resource Management

The codebase makes extensive use of static resources to solve lifetime issues, enable task spawning, and facilitate code organization across modules. Understanding our approach to statics is critical for working with the codebase effectively.

#### Static Patterns

We follow these general principles for static declarations:

- **Use `StaticCell` for statics that cannot be initialized at compile time.** These are typically complex objects that require runtime initialization.

- **Use `ConstStaticCell` for statics that can be initialized at compile time.** Note that initialization is different than mutability. A `ConstStaticCell` can be mutable when used with `RefCell`, but it must be initialized at compile time and cannot be `take()`n, modified, and then `take()`n again.

- **For immutable statics**, the above declarations are sufficient.

- **For statics where ownership will be transferred**, no mutex is required even if the object itself is mutable, as the static only holds it temporarily.

- **Use Mutexes, Signals and Channels** where information has to be shared or transferred between threads and cores.  However, objects which are not [`Send`] cannot be protected by a Mutex.

#### Mutex Selection

When mutable access to a static is required, we use mutex patterns with these guidelines:

- Use `embassy_sync::mutex::Mutex` for async contexts. This does not require a `RefCell` for interior mutability.

- Use `blocking_mutex::Mutex` with a `RefCell` for blocking contexts that require interior mutability.

- Generally prefer `CriticalSectionRawMutex`, as these protect across multiple cores.

- `ThreadModeRawMutex` is used only for single-threaded contexts.

- `NoopRawMutex` is used when synchronization is not needed but the API requires a mutex type.  We do not use it.

Our implementation prioritizes safety over performance optimization, so we typically use `CriticalSectionRawMutex` for simplicity and reliability.

#### Location Convention

Statics are typically defined in the module that creates them. For example, the `GPIO` static is defined in the `gpio` module. This keeps the implementation details encapsulated within the relevant module.

Where access takes some boilerplate code, such as locking Mutexes and then manipulating a value, we provide helper functions, within the module that defines that static, which other modules use to access it.  As examples see the [`StatusDisplay`](src/infra/display.rs) and [`Watchdog`]('src/infra/watchdog') modules. 

## Execution and Task Model

### Threading and Task Distribution

The pico1541-rs codebase is designed to effectively utilize both cores available on the RP2040 microcontroller, with a specific distribution of responsibilities:

#### Core 0 (Main Core)

Core 0 runs the `main()` function and handles all non-protocol tasks, including:
- The embassy USB stack (USB protocol and control handling)
- The display task (status LED management)
- The watchdog task
- WiFi support (planned for future implementation)

#### Core 1 (Protocol Core)

Core 1 is dedicated to handling:
- Bulk USB transfers 
- All Commodore protocol handling

This dual-core approach provides significant advantages over single-core implementations like the stock xum1541, allowing us to separate protocol handling from system management tasks.

#### Task Spawning Approach

Our task spawning model follows these principles:

- **Core 0 Spawning**: Task are spawned on core 0 using the `Spawner` object passed into `main()`.  Where the `Spawner` object is not available we get it using ```Spawner::for_current_executor()```.

- **Core 1 Spawning**: Apart from the primary task on core 1, which is spawned from core 0, other tasks on core 1 are spawned by retrieving the `Spawner` object using ```Spawner::for_current_executor()```.

The task module provides the primary interface for spawning tasks, abstracting away the details of which core is being targeted and ensuring proper initialization of the execution environment, and consistent logging.
