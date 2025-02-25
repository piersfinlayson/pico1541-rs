//! pico1541
//! 
//! This implements a USB device that allows Commodore disk drives to be
//! connected to a PC.
//! 
//! The device can either emulate an xum1541, and is therefore supported
//! directly by [`OpenCBM`](https://github.com/OpenCBM/OpenCBM), or provides
//! extended capabilities and is detected as a pico1541.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#![no_std]
#![no_main]

// Provide some feature guidance
#[cfg(not(any(feature = "compatibility", feature = "extended")))]
compile_error!("Either 'xum1541/compatibility' or 'extended' feature must be enabled");
#[cfg(all(feature = "compatibility", feature = "extended"))]
compile_error!("Features 'xum1541/compatibility' and 'extended' cannot be enabled simultaneously");

// Declare all of this library's modules.
mod bulk;
mod constants;
mod control;
mod protocol;
mod types;
mod dev_info;
mod built;

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};
use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::pac;
use embassy_rp::usb::{Driver as RpUsbDriver, Endpoint, In, InterruptHandler, Out};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{bind_interrupts, peripherals::USB};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
use embassy_time::Timer;
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint as DriverEndpoint, EndpointType};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::{ConstStaticCell, StaticCell};
use heapless::String;
use rp2040_rom::ROM;

use control::Control;
use protocol::ProtocolAction;
use bulk::Bulk;
use constants::{
    LOOP_LOG_INTERVAL, MAX_EP_PACKET_SIZE, MAX_PACKET_SIZE_0, USB_CLASS, USB_POWER_MA, USB_PROTOCOL, USB_SUB_CLASS, MAX_SERIAL_STRING_LEN,
    WATCHDOG_TIMER,
};
use dev_info::*;

// Statics
//
// We set up statics primarily to avoid lifetime issues, and to allow us to
// spawn tasks (accessing these statics), and to split our code into
// separate modules.
//
// These are a bit tricksy to get right, so here is some general guidance:
//
// - Use StaticCell for statics that cannot be initialized at compile time.
//
// - Use ConstStaticCell for statics that can be initialized at compile time.
//   Note that initialization is different than mutability.  A ConstStaticCell
//   can be mutable, when used with RefCell, but it must be initialized at
//   compile time.
//
// - If your static will be immutable, that is all that is required.
//
// - If your static will be mutable, but you will be passing ownership of it
//   to another object, then no Mutex is required either. 
//
// - If you need mutable access, you need to use a Mutex _and_ a RefCell.
//   - Generally use CriticalSectionRawMutex, as these work on multi-core
//     systems.
//   - ThreadModeRawMutex is, as it sounds, so single threaded usage.
//   - NoopRawMutex is for when you don't need a mutex.
//
//   Our implementation is not sufficiently dependent on performance to
//   require optimization here, so we always use CriticalSectionRawMutex.

// We use the WATCHDOG static to store the Watchdog object, so we can feed it
// from all of our tasks and objects.  This is shared and mutable.
//
// TODO - As we have multiple tasks running concurrently, we should probably
// implement a wrapper in order to ensure that one running runner (and other
// failed runners) don't keep the device alive.
static WATCHDOG: Mutex<CriticalSectionRawMutex, RefCell<Option<Watchdog>>> =
    Mutex::new(RefCell::new(None));

// Our Control Handler handles Control requests that come in on the Control
// endpoint, and the USB stack calls control_in() and control_out() for us
// to handle them.  It also handles various USB device lifecycles events, such
// as enabled, reset, address, configured, etc.  We pass ownership of this to
// our UsbDevice object, so we don't need anything other than a StaticCell.
static CONTROL: StaticCell<Control> = StaticCell::new();

// The BULK static contains our Bulk data handling object, which  a
// Protocol Handler object.
//
// The Bulk object consists of a runner that listens for data on the OUT
// endpoint and then passes it to the ProtocolHandler to process.  It also
// runs the ProtocolHandler's runner, which listens ProtocolActions which are
// signaled by the Control object in response to Control requests from the
// host.  Think Initialize, Reset, etc.  The ProtocolHandler's runner also
// handles any data which needs to be sent to the host in response to Bulk
// commands from the host.
//
// Ownership is passed to the USB task, so nothing other than a StaticCell is
// required. 
static BULK: StaticCell<Bulk> = StaticCell::new();

// The USB_DEVICE is primarily stored as a static to allow us to spawn a task
// using the USB runner.  (This is the runner that schedules our Control
// Handler.)  it is not used in other modules.
//
// Ownership is passed to the USB task, so nothing other than a StaticCell is
// required.
static USB_DEVICE: StaticCell<UsbDevice<'static, RpUsbDriver<'static, USB>>> = StaticCell::new();

// The PROTOCOL_ACTION static is a Mutex that is used to allow communication
// between the Control Handler and the Protocol Handler, so the Control
// Handler can instruct the Protocol Handler to perform actions in response to
// Control requests from the host.
//
// This object is shared between Control and ProtocolHandler (via Bulk), so we
// need both a Mutex, we need a RefCell to allow us to mutate the object.
static PROTOCOL_ACTION: Mutex<CriticalSectionRawMutex, RefCell<Option<ProtocolAction>>> =
    Mutex::new(RefCell::new(None));

// The following statics are used to store the USB descriptor buffers and
// control buffer.  We store them as statics to avoid lifetime issues when
// creating the USB builder.
//
// The ownership of these is passed to the USB builder, so we don't need

static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static BOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static MSOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static CONTROL_BUF: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);

// Create a static String to store the serial number in.  We'll make it a
// ConstStaticCell, so we initialize it  to allocate memory for it, and then
// take it later to give to USB config.
static SERIAL: ConstStaticCell<String::<MAX_SERIAL_STRING_LEN>> = ConstStaticCell::new(String::new());

// Bind the hardware USB interrupt to the USB stack.  Interrupts are the
// primary mechanism the USB stack uses to receive data from hardware.
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// Our main function.  This is the entry point for our application and like
// most embedded implementations, we do not want it to exit as that would mean
// the device has halted.
pub async fn common_main(spawner: Spawner, bin_name: &str) -> ! {
    // Get device peripherals.  This gives us access to the hardware - like
    // the USB and Watchdog.  We extract the ones we need to avoid having to
    // pass the entire object around, partially moving it.
    let p = embassy_rp::init(Default::default());
    let p_watchdog = p.WATCHDOG;
    let p_usb = p.USB;
    let mut p_flash = p.FLASH;
    let mut p_dma_ch0 = p.DMA_CH0;

    // Get the serial number.  Do it now, before RpUsbDriver::new() partially
    // moves p.
    let mut serial = SERIAL.take();
    get_serial(&mut p_flash, &mut p_dma_ch0, &mut serial);
    info!("Device serial: {}", serial);

    // Log the information about this firmware build.
    built::log_fw_info(bin_name, &serial);

    // Get the last reset reason.  There's supposed to be a reset_reason()
    // function on Watchdog, but while it's in the source, it doesn't seem to
    // be available in the embassy_rp crate.
    let watchdog = pac::WATCHDOG;
    let reason = watchdog.reason().read();
    let rr_str = if reason.force() {
        "forced";
    } else if reason.timer() {
        "watchdog timer";
    } else {
        "unknown";
    };
    info!("Last reset reason: {}", rr_str);

    // Set up the watchdog
    let mut watchdog = Watchdog::new(p_watchdog);
    watchdog.start(WATCHDOG_TIMER);
    WATCHDOG.lock(|w| *w.borrow_mut() = Some(watchdog));

    // Create a new USB device
    let mut driver = RpUsbDriver::new(p_usb, Irqs);

    // Set up the USB device descriptor.  Differences in the USB device
    // values between our different firmware build options/binaries are
    // handled within the dev_info module, so this doesn't need to worry
    // about that.
    let mut config = Config::new(VENDOR_ID, PRODUCT_ID);
    config.manufacturer = Some(MANUFACTURER);
    config.product = Some(PRODUCT);
    config.serial_number = Some(serial);
    config.max_power = USB_POWER_MA;
    config.max_packet_size_0 = MAX_PACKET_SIZE_0;

    // Set the device class, subclass, and protocol.
    config.device_class = USB_CLASS;
    config.device_sub_class = USB_SUB_CLASS;
    config.device_protocol = USB_PROTOCOL;

    // The default is composite with IADs, which gives use device class code
    // 0xEF, with is a miscellaneous device.
    config.composite_with_iads = false;

    // Allocate the endpoints.  We do this before we move driver into the
    // builder, as we need to assign specific endpoints numbers in xum1541
    // mode.
    //
    // If we didn't need to assign specific numbers, we could do this after
    // we create the InterfaceAltBuilder using alt_setting() below.
    //
    // ```rust
    // let ep_out = alt.endpoint_bulk_in(config.max_packet_size);
    // let ep_in = alt.endpoint_bulk_out(config.max_packet_size);
    // ``
    let (ep_in, ep_out) = allocate_endpoints(&mut driver);

    // Create a USB builder giving it our Static descriptors and control
    // buffer.
    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.take(),
        BOS_DESC.take(),
        MSOS_DESC.take(),
        CONTROL_BUF.take(),
    );

    // Set up the function and interface for the Vendor class
    let mut func = builder.function(USB_CLASS, USB_SUB_CLASS, USB_PROTOCOL);
    let mut interface = func.interface();
    let if_num = interface.interface_number();
    let mut alt = interface.alt_setting(USB_CLASS, USB_SUB_CLASS, USB_PROTOCOL, None);

    // Set the endpoints to those we created above using the
    // InterfaceAltBuilder
    alt.endpoint_descriptor(
        ep_in.info(),
        SynchronizationType::NoSynchronization,
        UsageType::DataEndpoint,
        &[],
    );
    alt.endpoint_descriptor(
        ep_out.info(),
        SynchronizationType::NoSynchronization,
        UsageType::DataEndpoint,
        &[],
    );

    // Drop func, to allow us to use the builder again.  Otherwise builder is
    // already borrowed mutably by func.
    drop(func);

    // Create a handler for USB events and set it using builder.  We make it
    // static to avoid lifetime issues.
    let handler = Control::new(if_num);
    let handler = CONTROL.init(handler);
    builder.handler(handler);

    // Build the UsbDevice and store it as a Static so we can spawn a task
    // with it.
    let usb = builder.build();
    let usb = USB_DEVICE.init(usb);

    // Create a Bulk object, which will listen for data on the OUT endpoint
    // and feed the watchdog.  We only feed it from Bulk::run(), and not
    // usb.run() because usb.run() is not cancel safe.
    let bulk = Bulk::new(ep_out, ep_in);
    let bulk = BULK.init(bulk);

    // Spawn a task to handle all USB related activity.
    match spawner.spawn(usb_task(usb, bulk)) {
        Ok(_) => (),
        Err(e) => {
            error!("Failed to spawn USB task: {}", e);
            reboot_normal();
        }
    }

    // Log every so often and avoid letting main() return.
    loop {
        info!("Main loop");
        Timer::after(LOOP_LOG_INTERVAL).await;
    }
}

// Our USB task.  This is the primary task that runs USB functionality.  It
// runs the USB stack, and the Bulk handler (which in turn runs the
// ProtocolHandler, and feeds the watchdog).
//
// This function never returns, so the device runs forever.
#[embassy_executor::task]
async fn usb_task(
    usb: &'static mut UsbDevice<'static, RpUsbDriver<'static, USB>>,
    bulk: &'static mut Bulk,
) -> ! {
    // Run a select on the USB stack and the Bulk handler.  If either of
    // these futures return, we want to know about it.  (If we used join,
    // we would only know when both returned.)
    let either = select(usb.run(), bulk.run()).await;

    // If we got here one of our futures returned.  This is very bad news,
    // as some work is not going to get done, so we reboot the device just
    // in case our watchdog strategy didn't work.
    match either {
        Either::First(_) => error!("USB future returned"),
        Either::Second(_) => error!("Bulk future returned"),
    }

    reboot_normal();
}

// Used to allocate specific endpoint numbers.  We do this to maintain
// backwards compatibility with the C version of this example (and another
// device which was the basis for the protocol support herein).  This example
// is designed to work with an existing host immplementation, and relies on
// endpoints numbers being as used here.
//
// If you change important USB descriptor information, like interfaces and
// endpoints numbers, but the OS has cached the device information (based on
// vendor ID and product ID), it can be a pain to recover from. To do so, on
// Windows, uninstall the device from Device Manager, on linux run:
// ```sudo udevadm control --reload-rules && sudo udevadm trigger```
fn allocate_endpoints(
    driver: &mut RpUsbDriver<'static, USB>,
) -> (Endpoint<'static, USB, In>, Endpoint<'static, USB, Out>) {
    // Loop through allocating the endpoint numbers until we get the ones we
    // want.  The Pi supports endpoints numbers 0-15 (0x00-0x0F) and will
    // assign them sequentially.  (The RP2040 chip only supports a total of
    // 16 endpoints in hardware, hence the restriction.)  Note that the IN
    // endpoint is ORed with 0x80, so we have to test for 0x80 | num to get
    // the right number.
    //
    // The actual required endpoint numbers are set in the dev_info module,
    // based on which firmware build is being compiled.

    // Get the IN endpoint (IN from device to host)
    let ep_in: Endpoint<'static, USB, In> = loop {
        let ep = driver
            .alloc_endpoint_in(EndpointType::Bulk, MAX_EP_PACKET_SIZE, 0)
            .expect("Unable to allocate IN endpoint");
        if ep.info().addr == IN_EP.into() {
            break ep;
        }
    };
    if ep_in.info().addr != IN_EP.into() {
        panic!("Unable to allocate 0x03 as OUT endpoint");
    }

    // Do the same for the OUT endpoint (OUT from host to device)
    let ep_out: Endpoint<'static, USB, Out> = loop {
        let ep = driver
            .alloc_endpoint_out(EndpointType::Bulk, MAX_EP_PACKET_SIZE, 0)
            .expect("Unable to allocate OUT endpoint");
        if ep.info().addr == OUT_EP.into() {
            break ep;
        }
    };
    if ep_out.info().addr != OUT_EP.into() {
        panic!("Unable to allocate 0x04 as OUT endpoint");
    }

    // Return the endpoints.
    (ep_in, ep_out)
}

// Called to perform a standard device reboot.  (Normally as in not entering
// BOOTSEL/DFU mode.)
fn reboot_normal() -> ! {
    // Try rebooting using the watchdog
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist  - will try and reset another way")
            .trigger_reset()
    });

    // Failed to reset.  Try again, differently.
    cortex_m::peripheral::SCB::sys_reset();
}

// Called to perform a reboot into BOOTSEL/DFU mode.
fn reboot_dfu() -> ! {
    unsafe {
        ROM::reset_usb_boot(0, 0);
    };
}

// Called to feed the watchdog
fn feed_watchdog() {
    WATCHDOG.lock(|w| {
        w.borrow_mut()
            .as_mut()
            .expect("Watchdog doesn't exist - can't feed it")
            .feed()
    });
}