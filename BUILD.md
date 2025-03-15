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

Skip if already installed.  Any version of Rust supporting the 2024 edition should be fine - pico1541-rs was primarily developed and tested using 1.85.

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

You 

### Install Rust target support  

Assuming you have Rust already installed

```bash
rustup target add thumbv6m-none-eabi         # For Pico
rustup target add thumbv8m.main-none-eabihf  # For Pico 2
```

## Build and Flash the Firmware

Clone this project:

```bash
git clone https://github.com/piersfinlayson/pico1541-rs
cd pico1541-rs
```

You have a number of options when building.  You can choose:
- xum1541 or pico1541 firmware (the former emulates the xum1541 and can be used with software like OpenCBM, the latter provides additional capabilities)
- Pico or Pico 2
- debug or release build

xum1541 emulates the xum1541, and can be used with software such as OpenCBM.

pico1541 adds additional functionality and is not compatible with software that expects an xum1541.

There is no need to specify a W (WiFi) or non-W variant of the Pico.  See  [WiFi Support](#wifi-support) for more information.

Scripts to build and run the chosen type of firmware are provided and are recommended for most users over using ```cargo``` directly.  Some examples:

```bash
./build.sh xum1541 pico            # Builds xum1541 for Pico, debug build
./build.sh pico1541 pico2 release  # Builds pico1541 for Pico 2, release build
./run.sh pico1541 pico             # Flashes pico1541 for Pico, debug
```

Sometimes the flashing process fails - this is because the erase step causes the device to panic and reboot in DFU/BOOTSEL mode.  This interrupts flashing.  Run the command again, and it should succeed the second time.

Alternatively you can use ```cargo build``` and ```cargo run``` directly.  In this case, as well as the binary and features, in the Pico 2 case you need to specify the correct target.  Some examples:

```bash
cargo run --bin xum1541 --features xum1541,pico 
cargo run --bin pico1541 --features pico1541,pico
cargo run --bin xum1541 --features xum1541,pico2 --target thumbv8m.main-none-eabihf
cargo run --bin pico1541 --features pico1541,pico2 --target thumbv8m.main-none-eabihf
```

Once the image has been flashed to your device, your host will detect a new USB device with the appropriate vendor ID/product ID, depending on which you specified.  Run ```dmesg``` - this example shows it in xum1541 mode.

```
usb 1-1: New USB device found, idVendor=16d0, idProduct=0504, bcdDevice= 2.08
usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
usb 1-1: Product: xum1541 floppy adapter (pico1541)
usb 1-1: Manufacturer: piers.rocks
usb 1-1: SerialNumber: 000
```

The pinouts used by the firmware are shown in the [schematic](pcb/pico1541_schematic.pdf).

## WiFi Support

There is no need to build a different image to include WiFi support.  The firmware will detect whether it's running on a Pico W (or Pico 2 W) variants, and includes WiFi support automatically if so.

## Debugging

embassy-rs based applications expect to be debugged by Debug Probe.  See [Setting up a Pico Probe](#setting-up-a-pico-probe)

## Setting up a Pico Probe

You can buy a dedicated Debug Probe from Raspberry Pi, or install on another Pico.  This can be a W, but there's no benefit, as it'll be controlled via USB.

### Building picoprobe

To build the picoprobe firmware to turn a Pico into a Debug Probe.

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

You need to connect the debug probe Pico to the 3 debug pins on the Pico you want to flash pico1541-rs onto.  You can find these 3 debug pins at the opposite end of the Pico to the USB connector, or, on a W variant, just above the WiFi IC shield.  They are labelled DEBUG on the top of the BOARD and SWDIO/GND/SWCLK on the bottom.

Using another Pico as the probe:
* Probe pin 3 (GND) <-> Pico DEBUG GND
* Probe pin 4 (GP2) <-> Pico DEBUG SWCLK
* Probe pin 5 (GP3) <-> Pico DEBUG SWDIO 

Or, if using a dedicated Debug Probe, the orange wire goes to SWCLK, the black to GND, and yellow to SWDIO.

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

## Using picotool to flash the firmware

If you'd rather use picotool to flash the firmware, you can do so by modifying [```.cargo/config/toml```](.cargo/config.toml).  Comment out the ```runner``` line for your target:

```toml
runner = "probe-rs run --no-location --chip RP2040"
# Or
runner = "probe-rs run --no-location --chip RP235x"
```

And uncomment this one:
```toml
#runner = "picotool load -u -v -x -t elf"
```

Or, you can run picotool directly, like this (where you may need to change the path to your firmware, depending on which version you want to flash):

```bash
picotool load -u -v -x -t elf target/thumbv6m-none-eabi/debug/xum1541
```

However, logging is only produced via RTT, not UART, and you need a Debug Probe to access to the RTT output.
