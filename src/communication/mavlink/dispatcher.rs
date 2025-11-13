//! MAVLink Message Dispatcher
//!
//! Routes incoming MAVLink messages to appropriate handlers and collects responses.
//!
//! # Architecture
//!
//! The dispatcher acts as a central routing layer between the transport/task layer
//! and the protocol-specific handlers. It:
//!
//! 1. Receives parsed MAVLink messages from the task
//! 2. Routes them to the appropriate handler based on message type
//! 3. Collects response messages from handlers
//! 4. Returns responses to the task for transmission
//!
//! # Design Principles
//!
//! - **Zero-cost abstraction**: Uses match expressions, no trait objects
//! - **Handler ownership**: Dispatcher owns all handlers
//! - **Stateless routing**: Message routing logic separate from handler state
//! - **Response collection**: Handlers return heapless::Vec of responses

use super::{
    handlers::{CommandHandler, MissionHandler, ParamHandler, RcInputHandler, TelemetryStreamer},
    state::SystemState,
};
use heapless::Vec;
use mavlink::common::MavMessage;

/// Maximum number of response messages per incoming message
/// Most messages produce 1 response, PARAM_REQUEST_LIST can produce many
const MAX_RESPONSES: usize = 64;

/// MAVLink message dispatcher
///
/// Routes messages to appropriate handlers and manages handler lifecycle.
pub struct MessageDispatcher {
    /// Parameter protocol handler
    param_handler: ParamHandler,
    /// Command protocol handler
    command_handler: CommandHandler,
    /// Telemetry streaming handler
    telemetry_streamer: TelemetryStreamer,
    /// Mission protocol handler
    mission_handler: MissionHandler,
    /// RC input handler
    rc_input_handler: RcInputHandler,
}

impl MessageDispatcher {
    /// Create a new message dispatcher with initialized handlers
    ///
    /// # Arguments
    ///
    /// * `param_handler` - Handler for parameter protocol
    /// * `command_handler` - Handler for command protocol
    /// * `telemetry_streamer` - Handler for telemetry streaming
    /// * `mission_handler` - Handler for mission protocol
    /// * `rc_input_handler` - Handler for RC input messages
    pub fn new(
        param_handler: ParamHandler,
        command_handler: CommandHandler,
        telemetry_streamer: TelemetryStreamer,
        mission_handler: MissionHandler,
        rc_input_handler: RcInputHandler,
    ) -> Self {
        Self {
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        }
    }

    /// Dispatch an incoming MAVLink message to the appropriate handler
    ///
    /// Routes the message based on its type and collects any response messages
    /// that should be sent back to the GCS.
    ///
    /// # Arguments
    ///
    /// * `header` - MAVLink message header (system ID, component ID, sequence)
    /// * `message` - Parsed MAVLink message
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Vector of response messages to send back (may be empty)
    pub fn dispatch(
        &mut self,
        _header: &mavlink::MavHeader,
        message: &MavMessage,
        _timestamp_us: u64,
    ) -> Vec<MavMessage, MAX_RESPONSES> {
        use mavlink::common::MavMessage::*;

        match message {
            // Parameter protocol
            PARAM_REQUEST_LIST(data) => self.param_handler.handle_request_list(data),

            PARAM_REQUEST_READ(data) => {
                if let Some(msg) = self.param_handler.handle_request_read(data) {
                    let mut responses = Vec::new();
                    let _ = responses.push(msg);
                    responses
                } else {
                    Vec::new()
                }
            }

            PARAM_SET(data) => match self.param_handler.handle_set(data) {
                Ok(msg) => {
                    let mut responses = Vec::new();
                    let _ = responses.push(msg);
                    responses
                }
                Err(_) => Vec::new(),
            },

            // Command protocol
            COMMAND_LONG(data) => {
                let (ack, additional_messages) = self.command_handler.handle_command_long(data);
                let mut responses = Vec::new();
                let _ = responses.push(COMMAND_ACK(ack));
                // Add any additional messages (HEARTBEAT, STATUSTEXT, etc.)
                for msg in additional_messages {
                    let _ = responses.push(msg);
                }
                responses
            }

            // Mission protocol
            MISSION_REQUEST_LIST(data) => {
                let msg = self
                    .mission_handler
                    .handle_request_list(data, _timestamp_us);
                let mut responses = Vec::new();
                let _ = responses.push(MISSION_COUNT(msg));
                responses
            }

            MISSION_REQUEST_INT(data) => {
                match self.mission_handler.handle_request_int(data, _timestamp_us) {
                    Ok(item_data) => {
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ITEM_INT(item_data));
                        responses
                    }
                    Err(result) => {
                        // Send MISSION_ACK with error
                        let ack = self.mission_handler.create_ack(result, 1, 1);
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                }
            }

            MISSION_COUNT(data) => {
                let req = self.mission_handler.handle_count(data, _timestamp_us);
                let mut responses = Vec::new();
                let _ = responses.push(MISSION_REQUEST_INT(req));
                responses
            }

            MISSION_ITEM_INT(data) => {
                match self.mission_handler.handle_item_int(data, _timestamp_us) {
                    Ok(Some(req)) => {
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_REQUEST_INT(req));
                        responses
                    }
                    Ok(None) => {
                        // Upload complete, send ACK
                        let ack = self.mission_handler.create_ack(
                            mavlink::common::MavMissionResult::MAV_MISSION_ACCEPTED,
                            1,
                            1,
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                    Err(result) => {
                        let ack = self.mission_handler.create_ack(result, 1, 1);
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                }
            }

            MISSION_ACK(data) => {
                self.mission_handler.handle_ack(data);
                Vec::new()
            }

            // Messages that don't produce responses
            HEARTBEAT(_) => Vec::new(),

            // Unhandled messages
            _ => Vec::new(),
        }
    }

    /// Update telemetry streams and get messages that should be sent
    ///
    /// Should be called periodically (e.g., every 1ms) to check if any
    /// telemetry messages are due to be sent based on configured stream rates.
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Vector of telemetry messages to send (max 5: HEARTBEAT, ATTITUDE, GPS, SYS_STATUS, BATTERY_STATUS)
    pub fn update_telemetry(
        &mut self,
        state: &SystemState,
        timestamp_us: u64,
    ) -> Vec<MavMessage, 5> {
        self.telemetry_streamer.update(state, timestamp_us)
    }

    /// Check for mission protocol timeouts
    ///
    /// Should be called periodically to detect and handle mission transfer timeouts.
    ///
    /// # Arguments
    ///
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Optional MISSION_ACK message if timeout occurred
    pub fn check_mission_timeout(&mut self, timestamp_us: u64) -> Option<MavMessage> {
        if self.mission_handler.check_timeout(timestamp_us) {
            // Timeout occurred, send MISSION_ACK with error
            let ack = self.mission_handler.create_ack(
                mavlink::common::MavMissionResult::MAV_MISSION_OPERATION_CANCELLED,
                1,
                1,
            );
            Some(MavMessage::MISSION_ACK(ack))
        } else {
            None
        }
    }

    /// Get reference to command handler (for accessing system state)
    pub fn command_handler(&self) -> &CommandHandler {
        &self.command_handler
    }

    /// Get mutable reference to command handler (for updating system state)
    pub fn command_handler_mut(&mut self) -> &mut CommandHandler {
        &mut self.command_handler
    }

    /// Get reference to parameter handler
    pub fn param_handler(&self) -> &ParamHandler {
        &self.param_handler
    }

    /// Get mutable reference to parameter handler
    pub fn param_handler_mut(&mut self) -> &mut ParamHandler {
        &mut self.param_handler
    }

    /// Get reference to telemetry streamer
    pub fn telemetry_streamer(&self) -> &TelemetryStreamer {
        &self.telemetry_streamer
    }

    /// Get mutable reference to telemetry streamer
    pub fn telemetry_streamer_mut(&mut self) -> &mut TelemetryStreamer {
        &mut self.telemetry_streamer
    }

    /// Get reference to mission handler
    pub fn mission_handler(&self) -> &MissionHandler {
        &self.mission_handler
    }

    /// Get mutable reference to mission handler
    pub fn mission_handler_mut(&mut self) -> &mut MissionHandler {
        &mut self.mission_handler
    }

    /// Get reference to RC input handler
    pub fn rc_input_handler(&self) -> &RcInputHandler {
        &self.rc_input_handler
    }

    /// Get mutable reference to RC input handler
    pub fn rc_input_handler_mut(&mut self) -> &mut RcInputHandler {
        &mut self.rc_input_handler
    }

    /// Process RC input messages (async)
    ///
    /// RC messages update global state and don't produce response messages.
    /// This method should be called for RC_CHANNELS and RC_CHANNELS_OVERRIDE
    /// messages before calling dispatch().
    ///
    /// # Arguments
    ///
    /// * `message` - Parsed MAVLink message (RC_CHANNELS or RC_CHANNELS_OVERRIDE)
    /// * `timestamp_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// `true` if the message was handled, `false` if not an RC message
    pub async fn process_rc_input(&mut self, message: &MavMessage, timestamp_us: u64) -> bool {
        use mavlink::common::MavMessage::*;

        match message {
            RC_CHANNELS(rc_data) => {
                self.rc_input_handler
                    .handle_rc_channels(rc_data, timestamp_us)
                    .await;
                true
            }
            RC_CHANNELS_OVERRIDE(rc_override) => {
                self.rc_input_handler
                    .handle_rc_channels_override(rc_override, timestamp_us)
                    .await
            }
            _ => false,
        }
    }
}

#[cfg(all(test, feature = "pico2_w"))]
mod tests {
    use super::*;
    use crate::platform::rp2350::Rp2350Flash;

    #[test]
    fn test_dispatcher_creation() {
        let mut flash = Rp2350Flash::new();
        let param_handler = ParamHandler::new(&mut flash);
        let state = SystemState::default();
        let command_handler = CommandHandler::new(state);
        let telemetry_streamer = TelemetryStreamer::new(1, 1);
        let mission_handler = MissionHandler::new(1, 1);
        let rc_input_handler = RcInputHandler::new();

        let _dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );
    }

    #[test]
    fn test_dispatch_param_request_list() {
        let mut flash = Rp2350Flash::new();
        let param_handler = ParamHandler::new(&mut flash);
        let state = SystemState::default();
        let command_handler = CommandHandler::new(state);
        let telemetry_streamer = TelemetryStreamer::new(1, 1);
        let mission_handler = MissionHandler::new(1, 1);
        let rc_input_handler = RcInputHandler::new();

        let mut dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let msg = MavMessage::PARAM_REQUEST_LIST(mavlink::common::PARAM_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        });

        let responses = dispatcher.dispatch(&header, &msg, 0);

        // Should return parameter list
        assert!(!responses.is_empty());
    }

    #[test]
    fn test_dispatch_command_long() {
        let mut flash = Rp2350Flash::new();
        let param_handler = ParamHandler::new(&mut flash);
        let state = SystemState::default();
        let command_handler = CommandHandler::new(state);
        let telemetry_streamer = TelemetryStreamer::new(1, 1);
        let mission_handler = MissionHandler::new(1, 1);
        let rc_input_handler = RcInputHandler::new();

        let mut dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let msg = MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: 1.0, // arm
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });

        let responses = dispatcher.dispatch(&header, &msg, 0);

        // Should return COMMAND_ACK
        assert_eq!(responses.len(), 1);
        assert!(matches!(responses[0], MavMessage::COMMAND_ACK(_)));
    }

    #[test]
    fn test_update_telemetry() {
        let mut flash = Rp2350Flash::new();
        let param_handler = ParamHandler::new(&mut flash);
        let state = SystemState::default();
        let command_handler = CommandHandler::new(state.clone());
        let telemetry_streamer = TelemetryStreamer::new(1, 1);
        let mission_handler = MissionHandler::new(1, 1);
        let rc_input_handler = RcInputHandler::new();

        let mut dispatcher = MessageDispatcher::new(
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
        );

        // First call should produce HEARTBEAT
        let messages = dispatcher.update_telemetry(&state, 0);
        assert!(!messages.is_empty());
    }
}
