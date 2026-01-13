//! MAVLink Communication Task (RP2350 Platform)
//!
//! Embassy async task for MAVLink protocol communication on RP2350.
//!
//! # Task Responsibilities
//!
//! 1. **Message Reception**: Read MAVLink messages from UART asynchronously
//! 2. **Message Routing**: Dispatch messages to appropriate handlers via dispatcher
//! 3. **Telemetry Streaming**: Send periodic telemetry messages
//! 4. **Connection Monitoring**: Track GCS connection status

use crate::communication::mavlink::{
    task::{MavlinkConfig, MavlinkContext},
    vehicle::GroundRover,
};
use crate::platform::rp2350::Rp2350Flash;
use crate::platform::traits::flash::FlashInterface;
use embassy_time::{Duration, Instant, Timer};
use mavlink::Message;

/// MAVLink communication task (RP2350 Platform)
///
/// This task handles all MAVLink communication on the RP2350 platform.
/// It reads messages from UART, dispatches them to handlers, and sends
/// telemetry responses.
#[embassy_executor::task]
pub async fn mavlink_task(
    uart_rx: embassy_rp::uart::BufferedUartRx,
    uart_tx: embassy_rp::uart::BufferedUartTx,
    config: MavlinkConfig,
    flash: Rp2350Flash,
) {
    mavlink_task_impl(uart_rx, uart_tx, config, flash).await
}

/// Generic MAVLink task implementation
///
/// This helper function contains the actual task logic and is generic over
/// UART and Flash types. The task macro wrapper above uses concrete types
/// because Embassy tasks cannot be generic.
async fn mavlink_task_impl<R, W, F>(
    mut uart_rx: R,
    mut uart_tx: W,
    config: MavlinkConfig,
    mut flash: F,
) where
    R: embedded_io_async::Read,
    W: embedded_io_async::Write,
    F: FlashInterface,
{
    crate::log_info!("MAVLink task started");
    crate::log_info!("  System ID: {}", config.system_id);
    crate::log_info!("  Component ID: {}", config.component_id);

    let mut context = MavlinkContext::<GroundRover>::new(config, &mut flash);
    let mut last_telemetry = Instant::now();
    let telemetry_interval = Duration::from_millis(100); // 10Hz base rate

    loop {
        // Non-blocking message read attempt
        // Use a small timeout to allow telemetry updates
        let read_result = embassy_futures::select::select(
            Timer::after(Duration::from_millis(10)),
            context.parser.read_message(&mut uart_rx),
        )
        .await;

        // Handle message if one was received
        match read_result {
            embassy_futures::select::Either::Second(Ok((header, msg))) => {
                crate::log_trace!("RX MAVLink msg ID={}", msg.message_id());

                // Get current timestamp
                let timestamp_us = Instant::now().as_micros();

                // Route message to dispatcher
                let responses = context.dispatcher.dispatch(&header, &msg, timestamp_us);

                // Send response messages
                for response in &responses {
                    if let Err(e) = context.writer.write_message(&mut uart_tx, response).await {
                        crate::log_warn!("TX response error: {:?}", e);
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                crate::log_warn!("Parse error: {:?}", e);
            }
            embassy_futures::select::Either::First(_) => {
                // Timeout - no message received, continue to telemetry
            }
        }

        // Send telemetry at regular intervals
        if last_telemetry.elapsed() >= telemetry_interval {
            // Update attitude from AHRS before sending telemetry
            context
                .state
                .update_attitude(&crate::subsystems::ahrs::AHRS_STATE);

            let timestamp_us = Instant::now().as_micros();
            let telemetry_msgs = context
                .dispatcher
                .update_telemetry(&context.state, timestamp_us);

            for msg in &telemetry_msgs {
                if let Err(e) = context.writer.write_message(&mut uart_tx, msg).await {
                    crate::log_warn!("TX error: {:?}", e);
                }
            }

            last_telemetry = Instant::now();
        }

        // Small yield to prevent task starvation
        Timer::after_millis(1).await;
    }
}
