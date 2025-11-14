//! Command Protocol Handler
//!
//! Handles COMMAND_LONG messages from ground control station for vehicle control.
//!
//! # Supported Commands
//!
//! - **MAV_CMD_COMPONENT_ARM_DISARM**: Arm or disarm vehicle
//! - **MAV_CMD_DO_SET_MODE**: Change flight mode
//! - **MAV_CMD_PREFLIGHT_CALIBRATION**: Sensor calibration (placeholder)
//! - **MAV_CMD_REQUEST_MESSAGE**: Request specific message (e.g., PROTOCOL_VERSION)
//!
//! # Command Flow
//!
//! 1. GCS sends COMMAND_LONG message
//! 2. CommandHandler validates command and parameters
//! 3. Handler executes command or applies safety checks
//! 4. Handler sends COMMAND_ACK with result (ACCEPTED/DENIED/UNSUPPORTED)
//!
//! # Safety Checks
//!
//! - Arm only if battery voltage is adequate (>= 10.0V)
//! - Arm only if not already armed
//! - Disarm only if already armed
//! - Log all command rejections for debugging

use crate::communication::mavlink::state::{FlightMode, SystemState};
use crate::communication::mavlink::status_notifier;
use heapless::Vec;
use mavlink::common::{
    MavCmd, MavMessage, MavResult, COMMAND_ACK_DATA, COMMAND_LONG_DATA, PROTOCOL_VERSION_DATA,
};

/// MAVLink protocol version (MAVLink 2.0)
pub const MAVLINK_VERSION: u16 = 200;

/// Minimum supported MAVLink protocol version
pub const MAVLINK_MIN_VERSION: u16 = 200;

/// Maximum supported MAVLink protocol version
pub const MAVLINK_MAX_VERSION: u16 = 200;

/// Command handler for COMMAND_LONG messages
///
/// Manages vehicle commands from GCS including arming, mode changes, and calibration.
///
/// # Thread Safety
///
/// CommandHandler accesses the global SYSTEM_STATE via Mutex, allowing safe
/// concurrent access from multiple tasks.
#[derive(Default)]
pub struct CommandHandler {}

impl CommandHandler {
    /// Create a new command handler
    pub fn new() -> Self {
        Self::default()
    }

    /// Handle COMMAND_LONG message from GCS
    ///
    /// Returns COMMAND_ACK message and optional additional messages (HEARTBEAT, STATUSTEXT) to send back to GCS.
    /// The HEARTBEAT is sent immediately after arm/disarm to update GCS state without waiting.
    /// The STATUSTEXT is sent for force-arm/disarm operations to warn the operator.
    pub fn handle_command_long(
        &mut self,
        cmd: &COMMAND_LONG_DATA,
    ) -> (COMMAND_ACK_DATA, Vec<MavMessage, 4>) {
        crate::log_debug!("Received COMMAND_LONG: command={}", cmd.command as u32);

        let (result, should_send_heartbeat, extra_messages) = match cmd.command {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                let (result, heartbeat, messages) = self.handle_arm_disarm(cmd);
                (result, heartbeat, messages)
            }
            MavCmd::MAV_CMD_DO_SET_MODE => (self.handle_set_mode(cmd), false, Vec::new()),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION => {
                (self.handle_preflight_calibration(cmd), false, Vec::new())
            }
            MavCmd::MAV_CMD_REQUEST_MESSAGE => {
                (self.handle_request_message(cmd), false, Vec::new())
            }
            _ => {
                crate::log_warn!("Unsupported command: {}", cmd.command as u32);
                (MavResult::MAV_RESULT_UNSUPPORTED, false, Vec::new())
            }
        };

        let ack = COMMAND_ACK_DATA {
            command: cmd.command,
            result,
            progress: 0,         // MAVLink v2 extension
            result_param2: 0,    // MAVLink v2 extension
            target_system: 0,    // MAVLink v2 extension
            target_component: 0, // MAVLink v2 extension
        };

        let mut messages = extra_messages;
        if should_send_heartbeat {
            let _ = messages.push(self.build_heartbeat());
        }

        (ack, messages)
    }

    /// Handle MAV_CMD_COMPONENT_ARM_DISARM command
    ///
    /// param1: 1.0 to arm, 0.0 to disarm
    /// param2: 21196 (magic number) to force arm/disarm (bypass checks)
    ///
    /// Returns (result, should_send_heartbeat, additional_messages)
    fn handle_arm_disarm(
        &mut self,
        cmd: &COMMAND_LONG_DATA,
    ) -> (MavResult, bool, Vec<MavMessage, 4>) {
        use crate::communication::mavlink::state::SYSTEM_STATE;

        let should_arm = cmd.param1 > 0.5;
        let force = (cmd.param2 as i32) == 21196; // ArduPilot force-arm magic number

        if should_arm {
            // Arm vehicle
            let result = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                if force {
                    // Force-arm: bypass pre-arm checks
                    state.arm_forced()
                } else {
                    // Normal arm: run pre-arm checks
                    state.arm()
                }
            });

            match result {
                Ok(()) => {
                    if force {
                        crate::log_warn!("Vehicle FORCE ARMED (checks bypassed)");
                        status_notifier::send_warning("Armed (FORCED)");
                    } else {
                        crate::log_info!("Vehicle armed");
                        status_notifier::send_info("Armed");
                    }
                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(_reason) => {
                    crate::log_warn!("Arm rejected");
                    status_notifier::send_error("Arm rejected");
                    // TODO: Investigate duplicate STATUSTEXT messages
                    // Current observation: ARM/DISARM status messages sometimes appear twice in GCS
                    // Possible causes:
                    // - GCS sending duplicate commands (retry mechanism)
                    // - Message routing issue in dispatcher/router
                    // - Multiple consumers of status_notifier queue
                    // Future work: Add message deduplication or investigate root cause
                    (MavResult::MAV_RESULT_DENIED, false, Vec::new())
                }
            }
        } else {
            // Disarm vehicle
            let result = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                if force {
                    // Force-disarm: bypass pre-disarm validation
                    state.disarm_forced()
                } else {
                    // Normal disarm: run pre-disarm validation
                    state.disarm()
                }
            });

            match result {
                Ok(()) => {
                    if force {
                        crate::log_warn!("Vehicle FORCE DISARMED (validation bypassed)");
                        status_notifier::send_warning("Disarmed (FORCED)");
                    } else {
                        crate::log_info!("Vehicle disarmed");
                        status_notifier::send_info("Disarmed");
                    }
                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(_reason) => {
                    crate::log_warn!("Disarm rejected");
                    status_notifier::send_error("Disarm rejected");
                    (MavResult::MAV_RESULT_DENIED, false, Vec::new())
                }
            }
        }
    }

    /// Handle MAV_CMD_DO_SET_MODE command
    ///
    /// param1: MAV_MODE_FLAG base mode
    /// param2: custom mode (flight mode number)
    fn handle_set_mode(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::communication::mavlink::state::SYSTEM_STATE;

        let custom_mode = cmd.param2 as u32;

        match FlightMode::from_custom_mode(custom_mode) {
            Some(mode) => {
                let result = critical_section::with(|cs| {
                    let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                    state.set_mode(mode)
                });
                match result {
                    Ok(()) => {
                        crate::log_info!("Mode changed to {:?}", mode);
                        MavResult::MAV_RESULT_ACCEPTED
                    }
                    Err(_reason) => {
                        crate::log_warn!("Mode change rejected");
                        MavResult::MAV_RESULT_DENIED
                    }
                }
            }
            None => {
                crate::log_warn!("Invalid mode number: {}", custom_mode);
                MavResult::MAV_RESULT_DENIED
            }
        }
    }

    /// Handle MAV_CMD_PREFLIGHT_CALIBRATION command
    ///
    /// Placeholder: Always accepts but does nothing.
    /// Real calibration would trigger sensor calibration routines.
    fn handle_preflight_calibration(&mut self, _cmd: &COMMAND_LONG_DATA) -> MavResult {
        crate::log_info!("Preflight calibration requested (placeholder)");
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Handle MAV_CMD_REQUEST_MESSAGE command
    ///
    /// GCS requests a specific message to be sent.
    /// param1: Message ID to request (e.g., 300 for PROTOCOL_VERSION)
    ///
    /// Response should be sent via the requested message type, not COMMAND_ACK result_param2.
    /// The router/telemetry system handles sending the actual requested message.
    fn handle_request_message(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        let message_id = cmd.param1 as u32;

        // PROTOCOL_VERSION message ID is 300
        const MAVLINK_MSG_ID_PROTOCOL_VERSION: u32 = 300;

        match message_id {
            MAVLINK_MSG_ID_PROTOCOL_VERSION => {
                crate::log_debug!("Protocol version requested via MAV_CMD_REQUEST_MESSAGE");
                MavResult::MAV_RESULT_ACCEPTED
            }
            _ => {
                crate::log_warn!("Unsupported message ID in REQUEST_MESSAGE: {}", message_id);
                MavResult::MAV_RESULT_UNSUPPORTED
            }
        }
    }

    /// Create PROTOCOL_VERSION message
    ///
    /// Returns a PROTOCOL_VERSION message containing MAVLink version information.
    /// This should be sent in response to MAV_CMD_REQUEST_MESSAGE with param1=300.
    pub fn create_protocol_version_message() -> PROTOCOL_VERSION_DATA {
        PROTOCOL_VERSION_DATA {
            version: MAVLINK_VERSION,
            min_version: MAVLINK_MIN_VERSION,
            max_version: MAVLINK_MAX_VERSION,
            spec_version_hash: [0u8; 8], // TODO: Populate with actual MAVLink XML spec hash
            library_version_hash: [0u8; 8], // TODO: Populate with mavlink-rust version hash
        }
    }

    /// Get copy of current system state
    ///
    /// Returns a copy of the system state from the global SYSTEM_STATE.
    /// SystemState is Copy, so this is efficient.
    pub fn state(&self) -> SystemState {
        use crate::communication::mavlink::state::SYSTEM_STATE;
        critical_section::with(|cs| *SYSTEM_STATE.borrow_ref(cs))
    }

    /// Build HEARTBEAT message with current arm state
    ///
    /// This is sent immediately after arm/disarm to update GCS without waiting
    /// for the next scheduled HEARTBEAT from telemetry.
    fn build_heartbeat(&self) -> MavMessage {
        use crate::communication::mavlink::state::SYSTEM_STATE;
        use mavlink::common::{MavAutopilot, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};

        let is_armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());
        let base_mode = if is_armed {
            MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED
        } else {
            MavModeFlag::empty()
        };

        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: 0,
            mavtype: MavType::MAV_TYPE_GROUND_ROVER,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavlink::common::MavCmd;

    fn create_command_long(command: MavCmd, param1: f32, param2: f32) -> COMMAND_LONG_DATA {
        COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command,
            confirmation: 0,
            param1,
            param2,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        }
    }

    #[test]
    #[serial_test::serial]
    fn test_arm_command_accepted() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0; // Good battery
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_arm_command_denied_already_armed() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_arm_command_denied_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Critical voltage
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_disarm_command_accepted() {
        let mut state = SystemState::new();
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_disarm_command_denied_already_disarmed() {
        let state = SystemState::new(); // Default is disarmed
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_set_mode_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 3.0); // Mode 3 = Auto
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(handler.state().mode, FlightMode::Auto);
    }

    #[test]
    fn test_set_mode_denied_invalid() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 99.0); // Invalid mode
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_preflight_calibration_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_unsupported_command() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_NAV_WAYPOINT, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn test_command_ack_fields() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 0.0);

        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_request_protocol_version() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = CommandHandler::new();

        // MAV_CMD_REQUEST_MESSAGE with param1=300 (PROTOCOL_VERSION message ID)
        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 300.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_create_protocol_version_message() {
        let msg = CommandHandler::create_protocol_version_message();

        assert_eq!(msg.version, 200);
        assert_eq!(msg.min_version, 200);
        assert_eq!(msg.max_version, 200);
        // Hash arrays should be initialized (currently zeros)
        assert_eq!(msg.spec_version_hash.len(), 8);
        assert_eq!(msg.library_version_hash.len(), 8);
    }
}
