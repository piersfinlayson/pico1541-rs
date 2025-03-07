# Building and running

## Pre-requisites

### Build Tools

You'll need various build tools to do everything in this README: 

```bash
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential libstdc++-arm-none-eabi-newlib libusb-1.0-0-dev pkg-config
```

### Debug Probe

You will need to have a debug probe in order to flash, and debug, this example.  You can find out more about this from [probe-rs](https://probe.rs).  If you want to use another Pico as a debug probe, see the section below on [Setting up a Pico Probe](#setting-up-a-pico-probe).  

### Install probe-rs

```bash
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
probe-rs complete install
```

### Install Rust

Skip if already installed.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

You 

### Install Rust target support  

Assuming you have Rust already installed

```bash
rustup target add thumbv6m-none-eabi
```

## Build and Flash the Firmware

Clone this project:

```bash
git clone https://github.com/piersfinlayson/pico1541-rs
cd pico1541-rs
cargo run
```

To build you needs to specify the binary name and feature name.  They are identical, and takes the values:
* xum1541
* pico1541
* pico1541w

For example:

```bash
cargo run --bin xum1541 --features xum1541 
```

Or:

```bash
cargo run --bin pico1541 --features pico1541
```

Or:

```bash
cargo run --bin pico1541w --features pico1541w
```

To flash and then auto-run the firmware, with your pico probe attached:

```bash
cargo run --bin xum1541 --features xum1541 # or pico1541 or pico1541w
```

Once the image has been flashed to your device, your host will detect a new USB device with the appropriate vendor ID/product ID, depending on which you specified.  Run ```dmesg``` - this examples shows it in xum1541 mode.

```
usb 1-1: New USB device found, idVendor=16d0, idProduct=0504, bcdDevice= 2.08
usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 1-1: Product: xum1541 floppy adapter (pico1541)
usb 1-1: Manufacturer: piers.rocks
usb 1-1: SerialNumber: 000
```

The pinouts used by the firmware are shown in the [schematic](pcb/pico1541_schematic.pdf).  There are other pin mappings supported via Cargo features.  These are defined in the `config` module within [`gpio.rs`](src/gpio.rs).  For example, to use the prototype pin mapping add the `prototype` feature like this:

```bash
cargo run --bin xum1541 --features xum1541,prototype
``` 

## Debugging

embassy-rs based applications expect to be debugged by Debug Probe.  See [Setting up a Pico Probe](#setting-up-a-pico-probe)

## Setting up a Pico Probe

Find another Pico - it can be a W, but there's no benefit, as it'll be controlled via USB.

### Building picoprobe

```bash
git clone https://github.com/raspberrypi/picoprobe
cd picoprobe
git submodule update --init
mkdir build
cd build
cmake -DDEBUG_ON_PICO=ON .. # -DDEBUG_ON_PICO=ON is important
```

### Flashing picoprobe

You'll need picotool to flash picoprobe to your Pico.  See [Installing picotool](#installing-picotool) for how to do that.  

Boot your pico into BOOTSEL mode (plugging in with button held).

```bash
picotool load debug_on_pico.elf
picotool reset
```

```dmesg``` should give output like this:

```
usb 1-2: New USB device found, idVendor=2e8a, idProduct=000c, bcdDevice= 2.21
usb 1-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 1-2: Product: Debugprobe on Pico (CMSIS-DAP)
usb 1-2: Manufacturer: Raspberry Pi
usb 1-2: SerialNumber: E660581122334455
cdc_acm 1-2:1.1: ttyACM1: USB ACM device
```

```probe-rs list``` should give output like this:

```
The following debug probes were found:
[0]: Debugprobe on Pico (CMSIS-DAP) -- 2e8a:000c:E661416677889900 (CMSIS-DAP)
```

This means you've successfully set this Pico up as a debug probe.

### Attaching the pico1541 to your probe

You need to connect the debug probe Pico to the 3 debug pins on the Pico you want to flash pico1541-rs onto.  You can find these 3 debug pins at the opposite end of the Pico to the USB connector.  They are labelled DEBUG on the top of the BOARD and SWDIO/GND/SWCLK on the bottom. 
* Probe pin 3 (GND) <-> Pico DEBUG GND
* Probe pin 4 (GP2) <-> Pico DEBUG SWCLK
* Probe pin 5 (GP3) <-> Pico DEBUG SWDIO 

Now when you run ```probe-rs info``` should give you this output (amongst some warnings about the lack of JTAG support):

```
ARM Chip with debug port Default:
No access ports found on this chip.
Debug Port: DPv2, MINDP, Designer: Raspberry Pi Trading Ltd, Part: 0x1002, Revision: 0x0, Instance: 0x0f
```

Here we can see that another Pico is connected to the probe Pico. 

## Installing picotool

To install picotool:

```bash
git clone https://github.com/raspberrypi/picotool
cd picotool
mkdir build
cd build
cmake ..
make -j 8
sudo make install
```

```picotool info``` will test you have a working picotool.  You should get output like:

```
No accessible RP-series devices in BOOTSEL mode were found.
```