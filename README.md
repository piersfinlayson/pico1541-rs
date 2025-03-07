# pico1541

The pico1541 project re-implements the Commodore USB interface (the xum1541) for the Raspberry Pi Pico, and in Rust, using the [`Embassy`](https://github.com/embassy-rs/embassy) project.

The project is currently a work in progress, with the initial aim being to replicate the existing xum1541 functionality.

If you want to contribute to the project, contact the owner at <piers@piers.rocks>.

## Why the pico1541?

The Raspberry Pi brings much higher processing power to the pico1541, making it feasible to additional features and functionality to the traditional xum1541 device, such as
* a WiFi interface
* an in-memory cache.

It can also be migrated more easily to future even higher spec Raspberry Pi based hardware, such as the Pico 2.

The pico1541 aims to be able to operate in fully xum1541 backwards compatible mode, providing identical runtime functionality and performance to the ATMEGA based xum1541.  This allows the pico1541 to be a drop-in replacement for the xum1541, and operate with all existing tooling, including the [OpenCBM](https://github.com/OpenCBM/OpenCBM) cbm* commands, nibtools, etc.

However, it will also be possible to run the pico1541 in a higher functionality, non-backwards compatible, mode which adds additional capabilities, including more robust error handling.

## xum1541 vs pico1541

| Feature | xum1541 | pico1541 |
|---------|---------|----------|
| Processor | ATMEGA32u2 | RP2040 |
| Clock Speed | 16 MHz | 125 MHz |
| Memory | 8 KB SRAM | 264 KB SRAM |
| Flash Storage | 32 KB | 2 MB |
| Connectivity | USB | USB + WiFi* |
| In-memory cache | No | Yes* |
| Enhanced error handling | No | Yes* |
| OpenCBM compatibility | Full | Full† |
| Future expandability | Medium | High |

*In enhanced mode.  WiFi requires Pico W.
†In compatibility mode

## Why [`Embassy`](https://github.com/embassy-rs/embassy)?

To quote:

> The Rust programming language is blazingly fast and memory-efficient, with no runtime, garbage collector or OS. It catches a wide variety of bugs at compile time, thanks to its full memory- and thread-safety, and expressive type system.

> Rust's async/await allows for unprecedentedly easy and efficient multitasking in embedded systems. Tasks get transformed at compile time into state machines that get run cooperatively. It requires no dynamic memory allocation, and runs on a single stack, so no per-task stack size tuning is required. It obsoletes the need for a traditional RTOS with kernel context switching, and is faster and smaller than one!

## Quick Start

If you already have Rust, probe-rs, Rust RP2040 target support (thumbv6m-none-eabi) and the various required build tools installed, all you need to do to build, flash and run this firmware to operate in xum1541 compatibility-mode is: 

```bash
git clone https://github.com/piersfinlayson/pico1541-rs
cd pico1541-rs
cargo run --bin xum1541 --features xum1541
```

That's it - the pico1541 will now be up and running and connected to the USB bus, presenting itself as an xum1541.  See [`pcb/pico1541_schematic.pdf`](pcb/pico1541_schematic.pdf) for the pins used by this configuration.

For more instructions on building and running, see [BUILD.md](BUILD.md).

## License

pico1541-rs is licensed under the GPL Version 3.  See [LICENSE](LICENSE).

The GPLv3 was chosen to be compatible with the xum1541, which is licensed under the GPLv2 or later license.

## Acknowledgements

The author acknowledges the work of the authors of [OpenCBM](https://github.com/OpenCBM/OpenCBM) and [xum1541](http://www.root.org/~nate/c64/xum1541/), Spiro Trikaliotis and Nate Lawson, on which much of the protocol handling support of pico1541-rs is based.