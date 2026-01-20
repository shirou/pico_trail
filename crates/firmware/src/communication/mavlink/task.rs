//! MAVLink Communication Task
//!
//! Embassy async task for MAVLink protocol communication.
//!
//! # Task Responsibilities
//!
//! 1. **Message Reception**: Read MAVLink messages from UART asynchronously
//! 2. **Message Routing**: Dispatch messages to appropriate handlers via dispatcher
//! 3. **Telemetry Streaming**: Send periodic telemetry messages
//! 4. **Connection Monitoring**: Track GCS connection status
//!
//! # Integration
//!
//! This task integrates with:
//! - Platform abstraction UART interface
//! - Task scheduler
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
    dispatcher::{DispatcherStats, MessageDispatcher},
    handlers::{
        command::CommandHandler, mission::MissionHandler, param::ParamHandler,
        rc_input::RcInputHandler, telemetry::TelemetryStreamer,
    },
    parser::MavlinkParser,
    state::SystemState,
    vehicle::{GroundRover, VehicleType},
    writer::MavlinkWriter,
};
use crate::platform::traits::flash::FlashInterface;

// Re-export mavlink_task from platform
pub use crate::platform::rp2350::tasks::mavlink::mavlink_task;

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
pub struct MavlinkContext<V: VehicleType> {
    /// MAVLink message parser
    pub parser: MavlinkParser,
    /// MAVLink message writer
    pub writer: MavlinkWriter,
    /// Message dispatcher to handlers
    pub dispatcher: MessageDispatcher<V>,
    /// System state
    pub state: SystemState,
    #[allow(dead_code)]
    config: MavlinkConfig,
}

impl<V: VehicleType> MavlinkContext<V> {
    pub fn new<F: FlashInterface>(config: MavlinkConfig, flash: &mut F) -> Self {
        let param_handler = ParamHandler::new(flash);
        let command_handler = CommandHandler::new();
        let telemetry_streamer = TelemetryStreamer::new(config.system_id, config.component_id);
        let mission_handler = MissionHandler::new(config.system_id, config.component_id);
        let rc_input_handler = RcInputHandler::new();

        let dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );

        Self {
            parser: MavlinkParser::new(),
            writer: MavlinkWriter::new(config.system_id, config.component_id),
            dispatcher,
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

    /// Get mutable reference to parser
    pub fn parser(&mut self) -> &mut MavlinkParser {
        &mut self.parser
    }

    /// Get mutable reference to writer
    pub fn writer(&mut self) -> &mut MavlinkWriter {
        &mut self.writer
    }

    /// Get mutable reference to dispatcher
    pub fn dispatcher(&mut self) -> &mut MessageDispatcher<V> {
        &mut self.dispatcher
    }

    /// Get parser statistics
    pub fn parser_stats(&self) -> super::parser::ParserStats {
        self.parser.stats()
    }

    /// Get writer statistics
    pub fn writer_stats(&self) -> super::writer::WriterStats {
        self.writer.stats()
    }

    /// Get dispatcher statistics
    pub fn dispatcher_stats(&self) -> DispatcherStats {
        self.dispatcher.stats()
    }
}

/// MAVLink communication task (placeholder for testing)
///
/// This is a placeholder implementation demonstrating the task structure.
/// The actual UART integration will be completed when integrating with
/// hardware examples.
pub async fn mavlink_task_placeholder<F: FlashInterface>(config: MavlinkConfig, flash: &mut F) {
    let _context = MavlinkContext::<GroundRover>::new(config, flash);

    // Placeholder task loop
    // Actual implementation will:
    // 1. Read messages from UART using parser
    // 2. Route messages using dispatcher
    // 3. Send telemetry using writer
    // 4. Update system state

    // On embedded: run infinite loop with periodic sleep
    loop {
        embassy_time::Timer::after_millis(1000).await;
        // Actual implementation will process messages here
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
        let context = MavlinkContext::<GroundRover>::new(config, &mut flash);

        assert!(!context.state().is_armed());
        assert_eq!(context.parser_stats().messages_received, 0);
        assert_eq!(context.writer_stats().messages_sent, 0);
        assert_eq!(context.dispatcher_stats().messages_processed, 0);
    }

    #[test]
    fn test_context_with_parameters() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();
        let context = MavlinkContext::<GroundRover>::new(config, &mut flash);

        // Verify dispatcher is initialized
        assert_eq!(context.dispatcher_stats().messages_processed, 0);
    }

    #[test]
    fn test_state_access() {
        let config = MavlinkConfig::default();
        let mut flash = MockFlash::new();
        let mut context = MavlinkContext::<GroundRover>::new(config, &mut flash);

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
