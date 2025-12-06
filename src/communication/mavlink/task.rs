//! MAVLink Communication Task
//!
//! Embassy async task for MAVLink protocol communication.
//!
//! # Task Responsibilities
//!
//! 1. **Message Reception**: Read MAVLink messages from UART asynchronously
//! 2. **Message Routing**: Dispatch messages to appropriate handlers via router
//! 3. **Telemetry Streaming**: Send periodic telemetry messages (Phase 3)
//! 4. **Connection Monitoring**: Track GCS connection status
//!
//! # Integration
//!
//! This task integrates with:
//! - Platform abstraction UART interface (from T-egg4f)
//! - Task scheduler (from T-g729p)
//! - System state management (local to this module)
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::task::mavlink_task;
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     // Initialize platform and get UART
//!     let uart = platform.create_uart(config)?;
//!
//!     // Spawn MAVLink task
//!     spawner.spawn(mavlink_task(uart)).unwrap();
//! }
//! ```

use super::{
    parser::MavlinkParser, router::MavlinkRouter, state::SystemState, writer::MavlinkWriter,
};
use crate::platform::traits::flash::FlashInterface;

#[cfg(feature = "pico2_w")]
use mavlink::Message;

/// MAVLink task configuration
#[derive(Debug, Clone, Copy)]
pub struct MavlinkConfig {
    /// System ID (MAVLink system ID for this autopilot)
    pub system_id: u8,
    /// Component ID (MAVLink component ID, typically 1 for autopilot)
    pub component_id: u8,
    /// UART baud rate (default: 115200)
    pub baud_rate: u32,
}

impl Default for MavlinkConfig {
    fn default() -> Self {
        Self {
            system_id: 1,
            component_id: 1,
            baud_rate: 115200,
        }
    }
}

/// MAVLink communication task context
///
/// Contains all state and components needed for MAVLink communication.
pub struct MavlinkContext<F: FlashInterface> {
    /// Message parser
    parser: MavlinkParser,
    /// Message writer
    writer: MavlinkWriter,
    /// Message router
    router: MavlinkRouter<F>,
    /// System state
    state: SystemState,
    /// Configuration (will be used in actual implementation)
    #[allow(dead_code)]
    config: MavlinkConfig,
}

#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
pub async fn mavlink_task(
    uart_rx: embassy_rp::uart::BufferedUartRx,
    uart_tx: embassy_rp::uart::BufferedUartTx,
    config: MavlinkConfig,
    flash: crate::platform::rp2350::Rp2350Flash,
) {
    mavlink_task_impl(uart_rx, uart_tx, config, flash).await
}

/// Generic MAVLink task implementation
///
/// This helper function contains the actual task logic and is generic over
/// UART and Flash types. The task macro wrapper above uses concrete types
/// because Embassy tasks cannot be generic.
#[cfg(feature = "pico2_w")]
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
    use embassy_time::{Duration, Instant, Timer};

    crate::log_info!("MAVLink task started");
    crate::log_info!("  System ID: {}", config.system_id);
    crate::log_info!("  Component ID: {}", config.component_id);

    let mut context = MavlinkContext::new(config, &mut flash);
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

                // Route message to handlers
                if let Err(e) = context.router.handle_message(&header, &msg, timestamp_us) {
                    if !matches!(e, super::router::RouterError::NoHandler) {
                        crate::log_warn!("Handler error: {:?}", e);
                    }
                }

                // Send any pending response messages (COMMAND_ACK, AUTOPILOT_VERSION, etc.)
                let pending = context.router.take_pending_messages();
                for pending_msg in &pending {
                    if let Err(e) = context.writer.write_message(&mut uart_tx, pending_msg).await {
                        crate::log_warn!("TX pending error: {:?}", e);
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
            let timestamp_us = Instant::now().as_micros();
            let telemetry_msgs = context.router.update_telemetry(timestamp_us);

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

impl<F: FlashInterface> MavlinkContext<F> {
    /// Create a new MAVLink task context with Flash-backed parameters
    ///
    /// # Arguments
    ///
    /// * `config` - MAVLink configuration
    /// * `flash` - Flash interface for parameter storage
    pub fn new(config: MavlinkConfig, flash: &mut F) -> Self {
        Self {
            parser: MavlinkParser::new(),
            writer: MavlinkWriter::new(config.system_id, config.component_id),
            router: MavlinkRouter::new(flash, config.system_id, config.component_id),
            state: SystemState::new(),
            config,
        }
    }

    /// Get immutable reference to system state
    pub fn state(&self) -> &SystemState {
        &self.state
    }

    /// Get mutable reference to system state
    pub fn state_mut(&mut self) -> &mut SystemState {
        &mut self.state
    }

    /// Get parser statistics
    pub fn parser_stats(&self) -> super::parser::ParserStats {
        self.parser.stats()
    }

    /// Get writer statistics
    pub fn writer_stats(&self) -> super::writer::WriterStats {
        self.writer.stats()
    }

    /// Get router statistics
    pub fn router_stats(&self) -> super::router::RouterStats {
        self.router.stats()
    }
}

/// MAVLink communication task (placeholder for Phase 1-2)
///
/// This is a placeholder implementation demonstrating the task structure.
/// The actual UART integration will be completed when integrating with
/// hardware examples.
///
/// # Phase 1-2 Progress
///
/// - Task structure and component integration ✓
/// - Parameter handlers integrated ✓
/// - Provide foundation for Phase 3 telemetry streaming
///
/// # Future Phases
///
/// - Phase 3: Add telemetry streaming
/// - Phase 4: Add command handlers
/// - Phase 5: Add mission protocol
pub async fn mavlink_task_placeholder<F: FlashInterface>(config: MavlinkConfig, flash: &mut F) {
    let _context = MavlinkContext::new(config, flash);

    // Placeholder task loop
    // Actual implementation will:
    // 1. Read messages from UART using parser
    // 2. Route messages using router
    // 3. Send telemetry using writer
    // 4. Update system state

    // Placeholder implementation differs for embedded vs host
    #[cfg(feature = "pico2_w")]
    {
        // On embedded: run infinite loop with periodic sleep
        loop {
            embassy_time::Timer::after_millis(1000).await;
            // Actual implementation will process messages here
        }
    }

    #[cfg(not(feature = "pico2_w"))]
    {
        // On host tests: just return to avoid infinite loop
        // Actual tests will use mock UART and process specific messages
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

    #[test]
    fn test_config_default() {
        let config = MavlinkConfig::default();
        assert_eq!(config.system_id, 1);
        assert_eq!(config.component_id, 1);
        assert_eq!(config.baud_rate, 115200);
    }

    #[test]
    fn test_context_creation() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();
        let context = MavlinkContext::new(config, &mut flash);

        assert!(!context.state().is_armed());
        assert_eq!(context.parser_stats().messages_received, 0);
        assert_eq!(context.writer_stats().messages_sent, 0);
        assert_eq!(context.router_stats().messages_processed, 0);
    }

    #[test]
    fn test_context_with_parameters() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();
        let context = MavlinkContext::new(config, &mut flash);

        // Verify parameter handler is initialized with default parameters
        assert_eq!(context.router_stats().messages_processed, 0);
    }

    #[test]
    fn test_state_access() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();
        let mut context = MavlinkContext::new(config, &mut flash);

        // Modify state through mutable reference
        context.state_mut().battery.voltage = 11.5;

        // Read state through immutable reference
        assert_eq!(context.state().battery.voltage, 11.5);
    }

    #[tokio::test]
    async fn test_task_placeholder() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();

        // Task should complete immediately on host (not in infinite loop)
        mavlink_task_placeholder(config, &mut flash).await;
    }
}
