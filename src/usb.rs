//! Handles creation of the embassy USB stack.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver as RpUsbDriver, Endpoint, In, InterruptHandler, Out};
use embassy_usb::descriptor::{SynchronizationType, UsageType};
use embassy_usb::driver::{Driver, Endpoint as DriverEndpoint, EndpointType};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::{ConstStaticCell, StaticCell};

use crate::bulk::{Bulk, BULK};
use crate::constants::{
    MAX_EP_PACKET_SIZE, MAX_PACKET_SIZE_0, USB_CLASS, USB_POWER_MA, USB_PROTOCOL, USB_SUB_CLASS,
};
use crate::control::Control;
use crate::dev_info::{IN_EP, MANUFACTURER, OUT_EP, PRODUCT, PRODUCT_ID, VENDOR_ID};
use crate::iec::IecBus;

// Bind the hardware USB interrupt to the USB stack.  Interrupts are the
// primary mechanism the USB stack uses to receive data from hardware.
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// The USB_DEVICE is primarily stored as a static to allow us to spawn a task
// using the USB runner.  (This is the runner that schedules our Control
// Handler.)  it is not used in other modules.
//
// Ownership is passed to the USB task, so nothing other than a StaticCell is
// required.
static USB_DEVICE: StaticCell<UsbDevice<'static, RpUsbDriver<'static, USB>>> = StaticCell::new();

// The following statics are used to store the USB descriptor buffers and
// control buffer.  We store them as statics to avoid lifetime issues when
// creating the USB builder.
//
// The ownership of these is passed to the USB builder, so we don't need

static CONFIG_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static BOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static MSOS_DESC: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);
static CONTROL_BUF: ConstStaticCell<[u8; 256]> = ConstStaticCell::new([0; 256]);

/// Used to create the embassy USB stack.
pub struct UsbStack {}

impl UsbStack {
    /// Creates the USB stack.
    ///
    /// # Arguments
    /// - `usb` - The USB peripheral
    /// - `iec_bus` - The IEC bus object
    ///
    /// # Returns
    /// (`USB, `Bulk`)
    pub fn create_static(
        p_usb: USB,
        serial: &'static str,
        iec_bus: IecBus,
    ) -> (
        &'static mut UsbDevice<'static, embassy_rp::usb::Driver<'static, USB>>,
        &'static mut Bulk,
    ) {
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

        // The default is composite with IADs, which gives use device class
        // code 0xEF, with is a miscellaneous device.
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
        let (ep_in, ep_out) = Self::allocate_endpoints(&mut driver);

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
            &ep_in.info().clone(),
            SynchronizationType::NoSynchronization,
            UsageType::DataEndpoint,
            &[],
        );
        alt.endpoint_descriptor(
            &ep_out.info().clone(),
            SynchronizationType::NoSynchronization,
            UsageType::DataEndpoint,
            &[],
        );

        // Drop func, to allow us to use the builder again.  Otherwise builder is
        // already borrowed mutably by func.
        drop(func);

        // Create a handler for USB events and set it using builder.  We make
        // it static to avoid lifetime issues.
        let handler = Control::create_static(if_num);
        builder.handler(handler);

        // Build the UsbDevice and store it as a Static so we can spawn a task
        // with it.
        let usb = builder.build();
        let usb = USB_DEVICE.init(usb);

        // Create a Bulk object, which will listen for data on the OUT endpoint
        // and feed the watchdog.
        let bulk = Bulk::new(ep_out, ep_in, iec_bus);
        let bulk = BULK.init(bulk);

        // Now return the USB device and the Bulk object.
        (usb, bulk)
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
        // Loop through allocating the endpoint numbers until we get the ones
        // we want.  The Pi supports endpoints numbers 0-15 (0x00-0x0F) and
        // assigns them sequentially.  (The RP2040 chip only supports a total
        // of 16 endpoints in hardware, hence the restriction.)  Note that the
        // IN endpoint is ORed with 0x80, so we have to test for 0x80 | num to
        // get the right number.
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
}

// Method to run the USB stack.
#[embassy_executor::task]
pub async fn usb_task(
    usb: &'static mut UsbDevice<'static, embassy_rp::usb::Driver<'static, USB>>,
) -> ! {
    usb.run().await
}
