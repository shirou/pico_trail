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
    handlers::{
        CommandHandler, MissionHandler, NavigationHandler, ParamHandler, RcInputHandler,
        TelemetryStreamer,
    },
    state::SystemState,
    vehicle::VehicleType,
};
use core::marker::PhantomData;
use heapless::Vec;
use mavlink::common::MavMessage;

#[derive(Debug, Clone, Copy, Default)]
pub struct ConnectionState {
    pub connected: bool,
    pub last_heartbeat_us: u64,
    pub heartbeat_count: u32,
}

impl ConnectionState {
    pub fn is_active(&self, current_time_us: u64, timeout_us: u64) -> bool {
        self.connected && (current_time_us - self.last_heartbeat_us) < timeout_us
    }

    pub fn update_heartbeat(&mut self, timestamp_us: u64) {
        self.connected = true;
        self.last_heartbeat_us = timestamp_us;
        self.heartbeat_count += 1;
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct DispatcherStats {
    pub messages_processed: u32,
    pub unhandled_messages: u32,
    pub handler_errors: u32,
}

/// Maximum number of response messages per incoming message
/// Most messages produce 1 response, PARAM_REQUEST_LIST can produce many
const MAX_RESPONSES: usize = 64;

pub struct MessageDispatcher<V: VehicleType> {
    param_handler: ParamHandler,
    command_handler: CommandHandler<V>,
    telemetry_streamer: TelemetryStreamer<V>,
    mission_handler: MissionHandler,
    rc_input_handler: RcInputHandler,
    navigation_handler: NavigationHandler,
    connection: ConnectionState,
    stats: DispatcherStats,
    _vehicle: PhantomData<V>,
}

impl<V: VehicleType> MessageDispatcher<V> {
    pub fn new(
        param_handler: ParamHandler,
        command_handler: CommandHandler<V>,
        telemetry_streamer: TelemetryStreamer<V>,
        mission_handler: MissionHandler,
        rc_input_handler: RcInputHandler,
    ) -> Self {
        Self {
            param_handler,
            command_handler,
            telemetry_streamer,
            mission_handler,
            rc_input_handler,
            navigation_handler: NavigationHandler::new(),
            connection: ConnectionState::default(),
            stats: DispatcherStats::default(),
            _vehicle: PhantomData,
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
        header: &mavlink::MavHeader,
        message: &MavMessage,
        timestamp_us: u64,
    ) -> Vec<MavMessage, MAX_RESPONSES> {
        use mavlink::common::MavMessage::*;

        self.stats.messages_processed += 1;

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
                    // Sync parameter-based fields to global SYSTEM_STATE
                    critical_section::with(|cs| {
                        super::state::SYSTEM_STATE
                            .borrow_ref_mut(cs)
                            .sync_from_params(self.param_handler.store());
                    });
                    let mut responses = Vec::new();
                    let _ = responses.push(msg);
                    responses
                }
                Err(_) => Vec::new(),
            },

            // Command protocol
            COMMAND_LONG(data) => {
                use super::handlers::telemetry::TelemetryStreamer;
                use mavlink::common::MavCmd;

                let (ack, additional_messages) = self.command_handler.handle_command_long(
                    data,
                    header.system_id,
                    header.component_id,
                );
                let mut responses = Vec::new();
                let _ = responses.push(COMMAND_ACK(ack));
                // Add any additional messages (HEARTBEAT, STATUSTEXT, etc.)
                for msg in additional_messages {
                    let _ = responses.push(msg);
                }

                // Handle MAV_CMD_REQUEST_MESSAGE - queue requested message
                if data.command == MavCmd::MAV_CMD_REQUEST_MESSAGE {
                    const MAVLINK_MSG_ID_AUTOPILOT_VERSION: u32 = 148;
                    const MAVLINK_MSG_ID_PROTOCOL_VERSION: u32 = 300;
                    let message_id = data.param1 as u32;

                    match message_id {
                        MAVLINK_MSG_ID_PROTOCOL_VERSION => {
                            let _ =
                                responses.push(TelemetryStreamer::<V>::build_protocol_version());
                        }
                        MAVLINK_MSG_ID_AUTOPILOT_VERSION => {
                            let _ =
                                responses.push(TelemetryStreamer::<V>::build_autopilot_version());
                        }
                        _ => {}
                    }
                }

                #[allow(deprecated)]
                if data.command == MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES {
                    let _ = responses.push(TelemetryStreamer::<V>::build_autopilot_version());
                }

                responses
            }

            COMMAND_INT(data) => {
                let (ack, extra_messages) = self.command_handler.handle_command_int(
                    data,
                    header.system_id,
                    header.component_id,
                );
                let mut responses = Vec::new();
                let _ = responses.push(COMMAND_ACK(ack));
                // Add extra messages (e.g., HOME_POSITION)
                for msg in extra_messages {
                    let _ = responses.push(msg);
                }
                responses
            }

            // Mission protocol
            MISSION_REQUEST_LIST(data) => {
                crate::log_info!("RX MISSION_REQUEST_LIST: type={}", data.mission_type as u8);
                let msg = self.mission_handler.handle_request_list(
                    data,
                    timestamp_us,
                    header.system_id,
                    header.component_id,
                );
                crate::log_info!("TX MISSION_COUNT: count={}", msg.count);
                let mut responses = Vec::new();
                let _ = responses.push(MISSION_COUNT(msg));
                responses
            }

            MISSION_REQUEST_INT(data) => {
                crate::log_info!("RX MISSION_REQUEST_INT: seq={}", data.seq);
                match self.mission_handler.handle_request_int(
                    data,
                    timestamp_us,
                    header.system_id,
                    header.component_id,
                ) {
                    Ok(item_data) => {
                        crate::log_info!(
                            "TX MISSION_ITEM_INT: seq={} cmd={} lat={} lon={} alt={}",
                            item_data.seq,
                            item_data.command as u16,
                            item_data.x,
                            item_data.y,
                            item_data.z
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ITEM_INT(item_data));
                        responses
                    }
                    Err(result) => {
                        crate::log_warn!(
                            "TX MISSION_ACK: ERROR seq={} result={}",
                            data.seq,
                            result as u8
                        );
                        // Send MISSION_ACK with error - reply to sender
                        let ack = self.mission_handler.create_ack(
                            result,
                            header.system_id,
                            header.component_id,
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                }
            }

            MISSION_COUNT(data) => {
                crate::log_info!(
                    "RX MISSION_COUNT: count={} type={}",
                    data.count,
                    data.mission_type as u8
                );
                let req_int = self.mission_handler.handle_count(
                    data,
                    timestamp_us,
                    header.system_id,
                    header.component_id,
                );
                crate::log_info!("TX MISSION_REQUEST: seq={}", req_int.seq);
                // Use deprecated MISSION_REQUEST for compatibility with Mission Planner
                #[allow(deprecated)]
                let req = mavlink::common::MISSION_REQUEST_DATA {
                    target_system: req_int.target_system,
                    target_component: req_int.target_component,
                    seq: req_int.seq,
                    mission_type: mavlink::common::MavMissionType::MAV_MISSION_TYPE_MISSION,
                };
                let mut responses = Vec::new();
                #[allow(deprecated)]
                let _ = responses.push(MISSION_REQUEST(req));
                responses
            }

            MISSION_ITEM_INT(data) => {
                use crate::communication::mavlink::handlers::mission::MissionState;
                let _next_seq = match self.mission_handler.state() {
                    MissionState::Idle => 0,
                    MissionState::UploadInProgress { next_seq, .. } => next_seq,
                    MissionState::DownloadInProgress { .. } => 0,
                };
                crate::log_info!("RX ITEM_INT: seq={} expect={}", data.seq, _next_seq);
                match self.mission_handler.handle_item_int(data, timestamp_us) {
                    Ok(Some(req_int)) => {
                        crate::log_info!("TX MISSION_REQUEST: seq={}", req_int.seq);
                        // Use deprecated MISSION_REQUEST for compatibility with Mission Planner
                        #[allow(deprecated)]
                        let req = mavlink::common::MISSION_REQUEST_DATA {
                            target_system: req_int.target_system,
                            target_component: req_int.target_component,
                            seq: req_int.seq,
                            mission_type: mavlink::common::MavMissionType::MAV_MISSION_TYPE_MISSION,
                        };
                        let mut responses = Vec::new();
                        #[allow(deprecated)]
                        let _ = responses.push(MISSION_REQUEST(req));
                        responses
                    }
                    Ok(None) => {
                        // Upload complete, send ACK - reply to sender
                        crate::log_info!("TX MISSION_ACK: ACCEPTED (upload complete)");
                        let ack = self.mission_handler.create_ack(
                            mavlink::common::MavMissionResult::MAV_MISSION_ACCEPTED,
                            header.system_id,
                            header.component_id,
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                    Err(result) => {
                        crate::log_warn!("TX MISSION_ACK: ERROR result={}", result as u8);
                        let ack = self.mission_handler.create_ack(
                            result,
                            header.system_id,
                            header.component_id,
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                }
            }

            // MISSION_ITEM (deprecated since 2020-06) - convert to MISSION_ITEM_INT format
            // Mission Planner uses this for "Fly Here" command
            // NOTE: During upload, Mission Planner may send BOTH MISSION_ITEM and MISSION_ITEM_INT
            // for the same waypoint. We only accept MISSION_ITEM when idle (for "Fly Here").
            #[allow(deprecated)]
            MISSION_ITEM(data) => {
                use crate::communication::mavlink::handlers::mission::MissionState;
                use mavlink::common::MISSION_ITEM_INT_DATA;

                // Ignore MISSION_ITEM during upload - only accept MISSION_ITEM_INT
                if matches!(
                    self.mission_handler.state(),
                    MissionState::UploadInProgress { .. }
                ) {
                    crate::log_debug!(
                        "MISSION_ITEM ignored during upload (using MISSION_ITEM_INT): seq={}",
                        data.seq
                    );
                    return Vec::new();
                }

                // Debug: log incoming MISSION_ITEM details
                crate::log_debug!(
                    "MISSION_ITEM: seq={} frame={} cmd={} current={} lat={} lon={} alt={} target=({},{})",
                    data.seq,
                    data.frame as u8,
                    data.command as u16,
                    data.current,
                    data.x,
                    data.y,
                    data.z,
                    data.target_system,
                    data.target_component
                );

                // Convert float lat/lon (degrees) to int32 (degE7)
                let int_data = MISSION_ITEM_INT_DATA {
                    target_system: data.target_system,
                    target_component: data.target_component,
                    seq: data.seq,
                    frame: data.frame,
                    command: data.command,
                    current: data.current,
                    autocontinue: data.autocontinue,
                    param1: data.param1,
                    param2: data.param2,
                    param3: data.param3,
                    param4: data.param4,
                    x: (data.x as f64 * 1e7) as i32,
                    y: (data.y as f64 * 1e7) as i32,
                    z: data.z,
                    mission_type: data.mission_type,
                };

                match self
                    .mission_handler
                    .handle_item_int(&int_data, timestamp_us)
                {
                    Ok(Some(req)) => {
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_REQUEST_INT(req));
                        responses
                    }
                    Ok(None) => {
                        // ACK target is the sender (from header)
                        let ack = self.mission_handler.create_ack(
                            mavlink::common::MavMissionResult::MAV_MISSION_ACCEPTED,
                            header.system_id,
                            header.component_id,
                        );
                        crate::log_debug!(
                            "MISSION_ACK: result={} target=({},{})",
                            ack.mavtype as u8,
                            ack.target_system,
                            ack.target_component
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                    Err(result) => {
                        let ack = self.mission_handler.create_ack(
                            result,
                            header.system_id,
                            header.component_id,
                        );
                        crate::log_debug!(
                            "MISSION_ACK error: result={} target=({},{})",
                            ack.mavtype as u8,
                            ack.target_system,
                            ack.target_component
                        );
                        let mut responses = Vec::new();
                        let _ = responses.push(MISSION_ACK(ack));
                        responses
                    }
                }
            }

            MISSION_ACK(data) => {
                crate::log_info!("RX MISSION_ACK: result={}", data.mavtype as u8);
                self.mission_handler.handle_ack(data);
                Vec::new()
            }

            // MISSION_WRITE_PARTIAL_LIST - partial mission update (not fully implemented)
            // Log for debugging, but don't handle yet
            #[allow(unused_variables)]
            MISSION_WRITE_PARTIAL_LIST(data) => {
                crate::log_warn!(
                    "MISSION_WRITE_PARTIAL_LIST received: start={}, end={} (NOT IMPLEMENTED)",
                    data.start_index,
                    data.end_index
                );
                Vec::new()
            }

            // Handle HEARTBEAT from GCS - update connection state
            HEARTBEAT(_) => {
                self.connection.update_heartbeat(timestamp_us);
                Vec::new()
            }

            // Unhandled messages
            _ => {
                self.stats.unhandled_messages += 1;
                Vec::new()
            }
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
    /// Vector of telemetry messages to send (max 16: HEARTBEAT, ATTITUDE, GPS, SYS_STATUS, BATTERY_STATUS, STATUSTEXT)
    pub fn update_telemetry(
        &mut self,
        state: &SystemState,
        timestamp_us: u64,
    ) -> Vec<MavMessage, 16> {
        use crate::communication::mavlink::status_notifier;

        let mut messages = Vec::new();

        // Get telemetry messages (HEARTBEAT, SYS_STATUS, etc.)
        let telemetry_msgs = self.telemetry_streamer.update(state, timestamp_us);
        for msg in telemetry_msgs {
            let _ = messages.push(msg);
        }

        // Get pending STATUSTEXT messages from status_notifier
        let statustext_msgs = status_notifier::take_pending_statustext_messages();
        for statustext in statustext_msgs {
            let _ = messages.push(MavMessage::STATUSTEXT(statustext));
        }

        messages
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
        if let Some((sender_system_id, sender_component_id)) =
            self.mission_handler.check_timeout(timestamp_us)
        {
            // Timeout occurred, send MISSION_ACK with error to the original sender
            let ack = self.mission_handler.create_ack(
                mavlink::common::MavMissionResult::MAV_MISSION_OPERATION_CANCELLED,
                sender_system_id,
                sender_component_id,
            );
            Some(MavMessage::MISSION_ACK(ack))
        } else {
            None
        }
    }

    /// Get reference to command handler (for accessing system state)
    pub fn command_handler(&self) -> &CommandHandler<V> {
        &self.command_handler
    }

    /// Get mutable reference to command handler (for updating system state)
    pub fn command_handler_mut(&mut self) -> &mut CommandHandler<V> {
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

    pub fn telemetry_streamer(&self) -> &TelemetryStreamer<V> {
        &self.telemetry_streamer
    }

    pub fn telemetry_streamer_mut(&mut self) -> &mut TelemetryStreamer<V> {
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

    /// Process navigation input messages (async)
    ///
    /// Navigation messages update global NAV_TARGET and don't produce response messages.
    /// This method should be called for SET_POSITION_TARGET_GLOBAL_INT messages.
    ///
    /// # Arguments
    ///
    /// * `message` - Parsed MAVLink message
    ///
    /// # Returns
    ///
    /// `true` if the message was handled, `false` if not a navigation message
    pub async fn process_navigation_input(&mut self, message: &MavMessage) -> bool {
        use mavlink::common::MavMessage::*;

        match message {
            SET_POSITION_TARGET_GLOBAL_INT(data) => {
                self.navigation_handler
                    .handle_set_position_target(data)
                    .await
            }
            _ => false,
        }
    }

    /// Get reference to navigation handler
    pub fn navigation_handler(&self) -> &NavigationHandler {
        &self.navigation_handler
    }

    pub fn connection(&self) -> &ConnectionState {
        &self.connection
    }

    pub fn connection_mut(&mut self) -> &mut ConnectionState {
        &mut self.connection
    }

    pub fn stats(&self) -> DispatcherStats {
        self.stats
    }

    pub fn reset_stats(&mut self) {
        self.stats = DispatcherStats::default();
    }

    pub fn update_heartbeat(&mut self, timestamp_us: u64) {
        self.connection.update_heartbeat(timestamp_us);
    }

    pub fn increment_messages_processed(&mut self) {
        self.stats.messages_processed += 1;
    }

    pub fn increment_unhandled(&mut self) {
        self.stats.unhandled_messages += 1;
    }
}

#[cfg(test)]
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
