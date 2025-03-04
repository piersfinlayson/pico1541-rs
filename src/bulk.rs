//! This module handles USB Bulk transfers on the OUT endpoint (i.e. received
//! by the device), and scheules the Protocol Handler to handle data in both
//! directions.

// Copyright (c) 2025 Piers Finlayson <piers@piers.rocks>
//
// GPLv3 licensed - see https://www.gnu.org/licenses/gpl-3.0.html

#[allow(unused_imports)]
use defmt::{debug, error, info, trace, warn};

use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Endpoint, Out};
use embassy_usb::driver::{Endpoint as DriverEndpoint, EndpointOut};

use crate::constants::{MAX_EP_PACKET_SIZE, MAX_WRITE_SIZE_USIZE};
use crate::display::{update_status, DisplayType};
use crate::protocol::WRITE_DATA_CHANNEL;
use crate::usb::READ_EP;

/// The Bulk object contains the runner which handles bulk transfers on the
/// OUT endpoint.
///
/// A simple example:
///
/// ```ignore
/// let bulk = Bulk::new(ep_out, ep_in);
/// bulk.run().await();
/// ```
pub struct Bulk {
    read_ep: Endpoint<'static, USB, Out>,
}

impl Bulk {
    /// Creates a new instance of the Bulk object.
    ///
    /// # Arguments
    /// - `out_ep` - OUT endpoint
    /// - `in_ep` = IN ep
    ///
    /// # Returns
    /// `Self`
    pub fn new(out_ep: Endpoint<'static, USB, Out>) -> Self {
        Self { read_ep: out_ep }
    }

    /// Runs the OUT bulk handler, by
    /// * waiting until the OUT endpoint is enabled
    /// * reading in any data
    /// * handling it (echoing it back or some other handling)
    pub async fn run(&mut self) -> ! {
        loop {
            debug!("Waiting for OUT endpoint to be enabled");
            // Wait for the read (IN) endpoint to be enabled.  We can wait
            // forever, as we don't watchdog police this task.
            self.read_ep.wait_enabled().await;
            debug!("OUT Endpoint enabled");

            // Device is now ready.
            update_status(DisplayType::Ready);

            loop {
                // Set up a buffer to read data into.
                let mut data = [0; MAX_EP_PACKET_SIZE as usize];

                // Now read in data.  Again, we can block here and run forever
                // as we are not being policed by the watchdog.
                //
                // Strictly, we can probably call read() in a select or similar
                // as read() appears to be cancel safe - that is it does't
                // lose data if the task is cancelled.
                match self.read_ep.read(&mut data).await {
                    Ok(size) => {
                        // We got bulk data.  Handle it.
                        if size <= MAX_WRITE_SIZE_USIZE {
                            // This call will block if the channel is full -
                            // i.e. the ProtocolHandler is not keeping up.
                            // This will in turn stop us reading in more USB
                            // and hence provide back-pressure.
                            WRITE_DATA_CHANNEL.send((size, data)).await;
                        } else {
                            warn!(
                                "Received more data than we can handle {} bytes - dropping",
                                size
                            );
                        }

                        debug!("Successfully handled {} bytes from OUT endpoint", size);
                    }
                    Err(e) => {
                        // This occurs if the endpoint is disabled - so we go
                        // around the outer loop again waiting for it to be
                        // re-enabled.
                        error!("Error reading from OUT endpoint: {:?}", e);
                        break;
                    }
                }

                // If we broke out of the read loop we need to update the
                // device status
                update_status(DisplayType::Init);
                debug!("OUT Endpoint disabled");
            }
        }
    }
}

/// Bulk task runner.
#[embassy_executor::task]
pub async fn bulk_task() -> ! {
    // Get the core number, in order to log it.
    let core = embassy_rp::pac::SIO.cpuid().read();
    info!("Core{}: Bulk task started", core);

    // Create the bulk object.
    let read_ep = READ_EP
        .lock()
        .await
        .take()
        .expect("Read endpoint not created");
    let mut bulk = Bulk::new(read_ep);

    // Start Bulks' runner.
    bulk.run().await;
}
