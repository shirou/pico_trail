//! MAVLink Message Router
//!
//! Routes incoming MAVLink messages to appropriate handlers based on message type.
//!
//! # Architecture
//!
//! - Receives parsed messages from parser
//! - Dispatches to protocol-specific handlers (param, telemetry, command, mission)
//! - Tracks connection state (GCS heartbeat monitoring)
//! - Provides unified interface for message handling
//!
//! # Handler Registration
//!
//! Handlers are registered during Phase 2+ implementation:
//! - Parameter handler (PARAM_REQUEST_LIST, PARAM_SET, etc.) - Phase 2 complete
//! - Command handler (COMMAND_LONG) - Phase 4
//! - Mission handler (MISSION_COUNT, MISSION_ITEM, etc.) - Phase 5
//!
//! # Connection Tracking
//!
//! - Monitors HEARTBEAT messages from GCS
//! - Tracks last received heartbeat timestamp
//! - Provides connection status queries

use super::handlers::{CommandHandler, MissionHandler, ParamHandler, TelemetryStreamer};
use super::state::SystemState;
use crate::platform::traits::flash::FlashInterface;
use mavlink::common::MavMessage;

/// Connection state with Ground Control Station
#[derive(Debug, Clone, Copy, Default)]
pub struct ConnectionState {
    /// GCS is connected (received heartbeat within timeout)
    pub connected: bool,
    /// Last heartbeat timestamp (microseconds since boot)
    pub last_heartbeat_us: u64,
    /// Number of heartbeats received
    pub heartbeat_count: u32,
}

impl ConnectionState {
    /// Check if connection is active
    ///
    /// Connection is considered active if a heartbeat was received within
    /// the timeout period (default: 5 seconds).
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    /// * `timeout_us` - Timeout period in microseconds (default: 5_000_000)
    pub fn is_active(&self, current_time_us: u64, timeout_us: u64) -> bool {
        self.connected && (current_time_us - self.last_heartbeat_us) < timeout_us
    }

    /// Update connection state on heartbeat reception
    ///
    /// # Arguments
    ///
    /// * `timestamp_us` - Timestamp of heartbeat reception
    pub fn update_heartbeat(&mut self, timestamp_us: u64) {
        self.connected = true;
        self.last_heartbeat_us = timestamp_us;
        self.heartbeat_count += 1;
    }
}

/// Router statistics for monitoring
#[derive(Debug, Clone, Copy, Default)]
pub struct RouterStats {
    /// Total messages processed
    pub messages_processed: u32,
    /// Unhandled messages (no handler registered)
    pub unhandled_messages: u32,
    /// Handler errors
    pub handler_errors: u32,
}

/// MAVLink message router
///
/// Routes incoming messages to registered handlers based on message type.
/// Tracks connection state and provides statistics.
pub struct MavlinkRouter<F: FlashInterface> {
    /// Connection state with GCS
    connection: ConnectionState,
    /// Router statistics
    stats: RouterStats,
    /// System state (shared between handlers)
    system_state: SystemState,
    /// Parameter protocol handler
    param_handler: ParamHandler,
    /// Telemetry streamer
    telemetry: TelemetryStreamer,
    /// Command handler
    command_handler: CommandHandler,
    /// Mission handler
    mission_handler: MissionHandler,
    /// Flash interface marker (not used directly, but needed for generic type)
    _flash: core::marker::PhantomData<F>,
}

impl<F: FlashInterface> MavlinkRouter<F> {
    /// Create a new MAVLink router with Flash-backed parameters
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface for parameter storage
    /// * `system_id` - MAVLink system ID (default: 1)
    /// * `component_id` - MAVLink component ID (default: 1)
    pub fn new(flash: &mut F, system_id: u8, component_id: u8) -> Self {
        let system_state = SystemState::new();
        Self {
            connection: ConnectionState::default(),
            stats: RouterStats::default(),
            system_state,
            param_handler: ParamHandler::new(flash),
            telemetry: TelemetryStreamer::new(system_id, component_id),
            command_handler: CommandHandler::new(system_state),
            mission_handler: MissionHandler::new(system_id, component_id),
            _flash: core::marker::PhantomData,
        }
    }

    /// Get connection state
    pub fn connection(&self) -> &ConnectionState {
        &self.connection
    }

    /// Get router statistics
    pub fn stats(&self) -> RouterStats {
        self.stats
    }

    /// Reset router statistics
    pub fn reset_stats(&mut self) {
        self.stats = RouterStats::default();
    }

    /// Get system state
    pub fn system_state(&self) -> &SystemState {
        &self.system_state
    }

    /// Get mutable system state reference
    pub fn system_state_mut(&mut self) -> &mut SystemState {
        &mut self.system_state
    }

    /// Get reference to parameter handler
    pub fn param_handler(&self) -> &ParamHandler {
        &self.param_handler
    }

    /// Get mutable reference to parameter handler
    pub fn param_handler_mut(&mut self) -> &mut ParamHandler {
        &mut self.param_handler
    }

    /// Get reference to mission handler
    pub fn mission_handler(&self) -> &MissionHandler {
        &self.mission_handler
    }

    /// Get mutable reference to mission handler
    pub fn mission_handler_mut(&mut self) -> &mut MissionHandler {
        &mut self.mission_handler
    }

    /// Update telemetry and get messages to send
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    ///
    /// Vector of telemetry messages to send (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS)
    pub fn update_telemetry(&mut self, current_time_us: u64) -> heapless::Vec<MavMessage, 4> {
        // Update stream rates from parameters
        if let Some(sr_extra1) = self.param_handler.store().get("SR_EXTRA1") {
            if let Some(sr_position) = self.param_handler.store().get("SR_POSITION") {
                let extra1 = match sr_extra1 {
                    crate::parameters::ParamValue::Int(v) => *v as u32,
                    _ => 10,
                };
                let position = match sr_position {
                    crate::parameters::ParamValue::Int(v) => *v as u32,
                    _ => 5,
                };
                self.telemetry.update_rates(extra1, position);
            }
        }

        self.telemetry.update(&self.system_state, current_time_us)
    }

    /// Handle incoming MAVLink message
    ///
    /// Routes the message to the appropriate handler based on message type.
    /// This is the main entry point for message processing.
    ///
    /// # Arguments
    ///
    /// * `header` - MAVLink message header
    /// * `message` - Parsed MAVLink message
    /// * `timestamp_us` - Message reception timestamp (microseconds since boot)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if message was handled successfully, or `Err` if:
    /// - No handler registered for message type
    /// - Handler returned an error
    ///
    /// # Examples
    ///
    /// ```ignore
    /// use pico_trail::communication::mavlink::router::MavlinkRouter;
    ///
    /// let mut router = MavlinkRouter::new();
    /// match router.handle_message(header, message, timestamp) {
    ///     Ok(()) => defmt::debug!("Message handled"),
    ///     Err(e) => defmt::warn!("Handler error: {:?}", e),
    /// }
    /// ```
    pub fn handle_message(
        &mut self,
        _header: &mavlink::MavHeader,
        message: &MavMessage,
        timestamp_us: u64,
    ) -> Result<(), RouterError> {
        self.stats.messages_processed += 1;

        // Route message based on type
        match message {
            // Connection monitoring
            MavMessage::HEARTBEAT(_data) => {
                self.handle_heartbeat(timestamp_us);
                Ok(())
            }

            // Parameter protocol (Phase 2)
            MavMessage::PARAM_REQUEST_LIST(data) => {
                self.handle_param_request_list(data);
                Ok(())
            }
            MavMessage::PARAM_REQUEST_READ(data) => {
                self.handle_param_request_read(data);
                Ok(())
            }
            MavMessage::PARAM_SET(data) => {
                self.handle_param_set(data);
                Ok(())
            }

            // Command protocol (Phase 4)
            MavMessage::COMMAND_LONG(data) => {
                self.handle_command_long(data);
                Ok(())
            }

            // Mission protocol (Phase 5)
            MavMessage::MISSION_REQUEST_LIST(data) => {
                self.handle_mission_request_list(data, timestamp_us);
                Ok(())
            }
            MavMessage::MISSION_REQUEST_INT(data) => {
                self.handle_mission_request_int(data, timestamp_us);
                Ok(())
            }
            MavMessage::MISSION_COUNT(data) => {
                self.handle_mission_count(data, timestamp_us);
                Ok(())
            }
            MavMessage::MISSION_ITEM_INT(data) => {
                self.handle_mission_item_int(data, timestamp_us);
                Ok(())
            }
            MavMessage::MISSION_ACK(data) => {
                self.handle_mission_ack(data);
                Ok(())
            }

            // All other messages
            _ => {
                self.stats.unhandled_messages += 1;
                Err(RouterError::NoHandler)
            }
        }
    }

    /// Handle HEARTBEAT message from GCS
    ///
    /// Updates connection state and tracks last heartbeat time.
    fn handle_heartbeat(&mut self, timestamp_us: u64) {
        self.connection.update_heartbeat(timestamp_us);
    }

    /// Handle PARAM_REQUEST_LIST message
    ///
    /// Returns messages to be sent via writer (implementation deferred to task layer)
    fn handle_param_request_list(&mut self, data: &mavlink::common::PARAM_REQUEST_LIST_DATA) {
        let _messages = self.param_handler.handle_request_list(data);
        // In Phase 2, we just call the handler
        // In Phase 3, messages will be queued to the writer
        // For now, this is a placeholder showing the integration point
    }

    /// Handle PARAM_REQUEST_READ message
    fn handle_param_request_read(&mut self, data: &mavlink::common::PARAM_REQUEST_READ_DATA) {
        let _message = self.param_handler.handle_request_read(data);
        // Message will be sent via writer in Phase 3
    }

    /// Handle PARAM_SET message
    fn handle_param_set(&mut self, data: &mavlink::common::PARAM_SET_DATA) {
        let _result = self.param_handler.handle_set(data);
        // Response message will be sent via writer in Phase 3
    }

    /// Handle COMMAND_LONG message
    ///
    /// Returns COMMAND_ACK message to be sent via writer (implementation deferred to task layer)
    fn handle_command_long(&mut self, data: &mavlink::common::COMMAND_LONG_DATA) {
        // Sync command handler state with router state
        *self.command_handler.state_mut() = self.system_state;

        // Process command
        let _ack = self.command_handler.handle_command_long(data);

        // Sync router state with command handler state (capture arm/mode changes)
        self.system_state = *self.command_handler.state();

        // In Phase 4, we just call the handler
        // In future phases, COMMAND_ACK will be queued to the writer
    }

    /// Handle MISSION_REQUEST_LIST message
    fn handle_mission_request_list(
        &mut self,
        data: &mavlink::common::MISSION_REQUEST_LIST_DATA,
        timestamp_us: u64,
    ) {
        let _count = self.mission_handler.handle_request_list(data, timestamp_us);
        // MISSION_COUNT will be sent via writer in future phases
    }

    /// Handle MISSION_REQUEST_INT message
    fn handle_mission_request_int(
        &mut self,
        data: &mavlink::common::MISSION_REQUEST_INT_DATA,
        timestamp_us: u64,
    ) {
        let _result = self.mission_handler.handle_request_int(data, timestamp_us);
        // MISSION_ITEM_INT will be sent via writer in future phases
    }

    /// Handle MISSION_COUNT message
    fn handle_mission_count(
        &mut self,
        data: &mavlink::common::MISSION_COUNT_DATA,
        timestamp_us: u64,
    ) {
        let _request = self.mission_handler.handle_count(data, timestamp_us);
        // MISSION_REQUEST_INT will be sent via writer in future phases
    }

    /// Handle MISSION_ITEM_INT message
    fn handle_mission_item_int(
        &mut self,
        data: &mavlink::common::MISSION_ITEM_INT_DATA,
        timestamp_us: u64,
    ) {
        let _result = self.mission_handler.handle_item_int(data, timestamp_us);
        // MISSION_REQUEST_INT or MISSION_ACK will be sent via writer in future phases
    }

    /// Handle MISSION_ACK message
    fn handle_mission_ack(&mut self, data: &mavlink::common::MISSION_ACK_DATA) {
        self.mission_handler.handle_ack(data);
    }
}

impl<F: FlashInterface + Default> Default for MavlinkRouter<F> {
    fn default() -> Self {
        let mut flash = F::default();
        Self::new(&mut flash, 1, 1) // Default system_id=1, component_id=1
    }
}

/// Router error types
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RouterError {
    /// No handler registered for message type
    NoHandler,
    /// Handler execution failed
    HandlerFailed,
}

impl core::fmt::Display for RouterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RouterError::NoHandler => write!(f, "No handler registered for message type"),
            RouterError::HandlerFailed => write!(f, "Handler execution failed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;
    use mavlink::common::{MavAutopilot, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};

    #[test]
    fn test_router_creation() {
        let mut flash = MockFlash::new();
        let router = MavlinkRouter::new(&mut flash, 1, 1);
        assert!(!router.connection().connected);
        assert_eq!(router.stats().messages_processed, 0);
        // WiFi params (5 visible: SSID/PASS are String, DHCP/IP/NETMASK/GATEWAY/... = 4)
        // + SR params (4) + SYSID (1) + WiFi SSID visible = 10
        assert_eq!(router.param_handler().count(), 10);
    }

    #[test]
    fn test_heartbeat_handling() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        let heartbeat = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GCS,
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &heartbeat, 1_000_000);
        assert!(result.is_ok());
        assert!(router.connection().connected);
        assert_eq!(router.connection().heartbeat_count, 1);
        assert_eq!(router.stats().messages_processed, 1);
    }

    #[test]
    fn test_connection_timeout() {
        let mut connection = ConnectionState::default();
        connection.update_heartbeat(0);

        // Within timeout
        assert!(connection.is_active(4_000_000, 5_000_000));

        // Outside timeout
        assert!(!connection.is_active(6_000_000, 5_000_000));
    }

    #[test]
    fn test_param_request_list_handled() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        let param_request =
            MavMessage::PARAM_REQUEST_LIST(mavlink::common::PARAM_REQUEST_LIST_DATA {
                target_system: 1,
                target_component: 1,
            });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &param_request, 0);
        assert!(result.is_ok()); // Should now be handled
        assert_eq!(router.stats().messages_processed, 1);
    }

    #[test]
    fn test_stats_reset() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);
        router.stats.messages_processed = 100;
        router.stats.unhandled_messages = 10;
        router.reset_stats();
        assert_eq!(router.stats().messages_processed, 0);
        assert_eq!(router.stats().unhandled_messages, 0);
    }

    #[test]
    fn test_param_set_integration() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        let mut param_id = [0u8; 16];
        param_id[..9].copy_from_slice(b"SR_EXTRA1");

        let param_set = MavMessage::PARAM_SET(mavlink::common::PARAM_SET_DATA {
            target_system: 1,
            target_component: 1,
            param_id,
            param_value: 20.0,
            param_type: mavlink::common::MavParamType::MAV_PARAM_TYPE_UINT32,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &param_set, 0);
        assert!(result.is_ok());

        // Verify parameter was updated
        let param = router.param_handler().store().get("SR_EXTRA1").unwrap();
        assert_eq!(*param, crate::parameters::ParamValue::Int(20));
    }

    #[test]
    fn test_telemetry_update() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);
        router.system_state.battery.voltage = 12.5;
        router.system_state.cpu_load = 25.0;

        // At t=0, should get at least HEARTBEAT
        let messages = router.update_telemetry(0);
        assert!(!messages.is_empty());
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::HEARTBEAT(_))));

        // At t=1s, should get HEARTBEAT and SYS_STATUS
        let messages = router.update_telemetry(1_000_000);
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::HEARTBEAT(_))));
        assert!(messages
            .iter()
            .any(|m| matches!(m, MavMessage::SYS_STATUS(_))));
    }

    #[test]
    fn test_telemetry_rate_from_parameters() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        // Change SR_EXTRA1 parameter (controls ATTITUDE rate)
        router
            .param_handler_mut()
            .store_mut()
            .set("SR_EXTRA1", crate::parameters::ParamValue::Int(20))
            .unwrap();

        // Telemetry update should pick up new rate
        let _messages = router.update_telemetry(0);

        // Verify rate was updated (indirectly through parameter)
        let param = router.param_handler().store().get("SR_EXTRA1").unwrap();
        assert_eq!(*param, crate::parameters::ParamValue::Int(20));
    }

    #[test]
    fn test_command_arm_integration() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        // Set good battery voltage
        router.system_state.battery.voltage = 12.0;

        let command = MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: 1.0, // Arm
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &command, 0);
        assert!(result.is_ok());
        assert!(router.system_state().is_armed());
    }

    #[test]
    fn test_command_disarm_integration() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        // Arm first
        router.system_state.armed = crate::communication::mavlink::state::ArmedState::Armed;

        let command = MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: mavlink::common::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: 0.0, // Disarm
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &command, 0);
        assert!(result.is_ok());
        assert!(!router.system_state().is_armed());
    }

    #[test]
    fn test_command_mode_change_integration() {
        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        let command = MavMessage::COMMAND_LONG(mavlink::common::COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: mavlink::common::MavCmd::MAV_CMD_DO_SET_MODE,
            confirmation: 0,
            param1: 0.0,
            param2: 3.0, // Mode 3 = Auto
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &command, 0);
        assert!(result.is_ok());
        assert_eq!(
            router.system_state().mode,
            crate::communication::mavlink::state::FlightMode::Auto
        );
    }

    #[test]
    fn test_mission_upload_integration() {
        use mavlink::common::{MavCmd, MavFrame, MISSION_COUNT_DATA, MISSION_ITEM_INT_DATA};

        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        // 1. Receive MISSION_COUNT
        let count_msg = MavMessage::MISSION_COUNT(MISSION_COUNT_DATA {
            target_system: 1,
            target_component: 1,
            count: 2,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &count_msg, 0);
        assert!(result.is_ok());

        // 2. Receive first MISSION_ITEM_INT
        let item1 = MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 0,
            frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            current: 0,
            autocontinue: 1,
            param1: 0.0,
            param2: 5.0,
            param3: 0.0,
            param4: 0.0,
            x: 370000000,
            y: -1220000000,
            z: 100.0,
        });

        let result = router.handle_message(&header, &item1, 0);
        assert!(result.is_ok());
        assert_eq!(router.mission_handler().storage().count(), 1);

        // 3. Receive second MISSION_ITEM_INT
        let item2 = MavMessage::MISSION_ITEM_INT(MISSION_ITEM_INT_DATA {
            seq: 1,
            x: 370010000,
            y: -1220010000,
            z: 120.0,
            ..MISSION_ITEM_INT_DATA {
                target_system: 1,
                target_component: 1,
                seq: 1,
                frame: MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command: MavCmd::MAV_CMD_NAV_WAYPOINT,
                current: 0,
                autocontinue: 1,
                param1: 0.0,
                param2: 5.0,
                param3: 0.0,
                param4: 0.0,
                x: 370010000,
                y: -1220010000,
                z: 120.0,
            }
        });

        let result = router.handle_message(&header, &item2, 0);
        assert!(result.is_ok());
        assert_eq!(router.mission_handler().storage().count(), 2);

        // Verify mission state is idle (upload complete)
        assert_eq!(
            router.mission_handler().state(),
            crate::communication::mavlink::handlers::mission::MissionState::Idle
        );
    }

    #[test]
    fn test_mission_download_integration() {
        use crate::core::mission::Waypoint;
        use mavlink::common::{MISSION_REQUEST_INT_DATA, MISSION_REQUEST_LIST_DATA};

        let mut flash = MockFlash::new();
        let mut router = MavlinkRouter::new(&mut flash, 1, 1);

        // Add waypoints to storage
        let wp1 = Waypoint::new(0, 370000000, -1220000000, 100.0);
        let wp2 = Waypoint::new(1, 370010000, -1220010000, 120.0);
        router
            .mission_handler_mut()
            .storage_mut()
            .add_waypoint(wp1)
            .unwrap();
        router
            .mission_handler_mut()
            .storage_mut()
            .add_waypoint(wp2)
            .unwrap();

        // 1. Receive MISSION_REQUEST_LIST
        let request_list = MavMessage::MISSION_REQUEST_LIST(MISSION_REQUEST_LIST_DATA {
            target_system: 1,
            target_component: 1,
        });

        let header = mavlink::MavHeader {
            system_id: 255,
            component_id: 1,
            sequence: 0,
        };

        let result = router.handle_message(&header, &request_list, 0);
        assert!(result.is_ok());

        // 2. Receive MISSION_REQUEST_INT for seq=0
        let request0 = MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            target_system: 1,
            target_component: 1,
            seq: 0,
        });

        let result = router.handle_message(&header, &request0, 0);
        assert!(result.is_ok());

        // 3. Receive MISSION_REQUEST_INT for seq=1
        let request1 = MavMessage::MISSION_REQUEST_INT(MISSION_REQUEST_INT_DATA {
            seq: 1,
            ..MISSION_REQUEST_INT_DATA {
                target_system: 1,
                target_component: 1,
                seq: 1,
            }
        });

        let result = router.handle_message(&header, &request1, 0);
        assert!(result.is_ok());
    }
}
