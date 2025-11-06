//! Command Protocol Handler
//!
//! Handles COMMAND_LONG messages from ground control station for vehicle control.
//!
//! # Supported Commands
//!
//! - **MAV_CMD_COMPONENT_ARM_DISARM**: Arm or disarm vehicle
//! - **MAV_CMD_DO_SET_MODE**: Change flight mode
//! - **MAV_CMD_PREFLIGHT_CALIBRATION**: Sensor calibration (placeholder)
//! - **MAV_CMD_REQUEST_PROTOCOL_VERSION**: Request MAVLink protocol version
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
use heapless::Vec;
use mavlink::common::{
    MavCmd, MavMessage, MavResult, MavSeverity, COMMAND_ACK_DATA, COMMAND_LONG_DATA,
    PROTOCOL_VERSION_DATA, STATUSTEXT_DATA,
};

/// MAVLink protocol version (MAVLink 2.0)
pub const MAVLINK_VERSION: u16 = 200;

/// Minimum supported MAVLink protocol version
pub const MAVLINK_MIN_VERSION: u16 = 200;

/// Maximum supported MAVLink protocol version
pub const MAVLINK_MAX_VERSION: u16 = 200;

#[cfg(feature = "defmt")]
use defmt::{debug, info, warn};

// Stub macros when defmt is not available
#[cfg(not(feature = "defmt"))]
macro_rules! debug {
    ($($arg:tt)*) => {{}};
}
#[cfg(not(feature = "defmt"))]
macro_rules! info {
    ($($arg:tt)*) => {{}};
}
#[cfg(not(feature = "defmt"))]
macro_rules! warn {
    ($($arg:tt)*) => {{}};
}

/// Command handler for COMMAND_LONG messages
///
/// Manages vehicle commands from GCS including arming, mode changes, and calibration.
pub struct CommandHandler {
    /// System state (shared with router)
    state: SystemState,
}

impl CommandHandler {
    /// Create a new command handler with given system state
    pub fn new(state: SystemState) -> Self {
        Self { state }
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
        debug!("Received COMMAND_LONG: command={}", cmd.command as u32);

        let (result, should_send_heartbeat, extra_messages) = match cmd.command {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
                let (result, heartbeat, messages) = self.handle_arm_disarm(cmd);
                (result, heartbeat, messages)
            }
            MavCmd::MAV_CMD_DO_SET_MODE => (self.handle_set_mode(cmd), false, Vec::new()),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION => {
                (self.handle_preflight_calibration(cmd), false, Vec::new())
            }
            MavCmd::MAV_CMD_REQUEST_PROTOCOL_VERSION => {
                (self.handle_request_protocol_version(cmd), false, Vec::new())
            }
            _ => {
                warn!("Unsupported command: {}", cmd.command as u32);
                (MavResult::MAV_RESULT_UNSUPPORTED, false, Vec::new())
            }
        };

        let ack = COMMAND_ACK_DATA {
            command: cmd.command,
            result,
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
        let should_arm = cmd.param1 > 0.5;
        let force = (cmd.param2 as i32) == 21196; // ArduPilot force-arm magic number

        if should_arm {
            // Arm vehicle
            let result = if force {
                // Force-arm: bypass pre-arm checks
                self.state.arm_forced()
            } else {
                // Normal arm: run pre-arm checks
                self.state.arm()
            };

            match result {
                Ok(()) => {
                    let mut messages = Vec::new();
                    if force {
                        warn!("Vehicle FORCE ARMED (checks bypassed)");
                        // Send STATUSTEXT warning to GCS
                        let _ = messages.push(MavMessage::STATUSTEXT(Self::create_statustext(
                            MavSeverity::MAV_SEVERITY_WARNING,
                            "Armed (FORCED)",
                        )));
                    } else {
                        info!("Vehicle armed");
                    }
                    (MavResult::MAV_RESULT_ACCEPTED, true, messages)
                }
                Err(_reason) => {
                    warn!("Arm rejected");
                    (MavResult::MAV_RESULT_DENIED, false, Vec::new())
                }
            }
        } else {
            // Disarm vehicle
            let result = if force {
                // Force-disarm: bypass pre-disarm validation
                self.state.disarm_forced()
            } else {
                // Normal disarm: run pre-disarm validation
                self.state.disarm()
            };

            match result {
                Ok(()) => {
                    let mut messages = Vec::new();
                    if force {
                        warn!("Vehicle FORCE DISARMED (validation bypassed)");
                        // Send STATUSTEXT warning to GCS
                        let _ = messages.push(MavMessage::STATUSTEXT(Self::create_statustext(
                            MavSeverity::MAV_SEVERITY_WARNING,
                            "Disarmed (FORCED)",
                        )));
                    } else {
                        info!("Vehicle disarmed");
                    }
                    (MavResult::MAV_RESULT_ACCEPTED, true, messages)
                }
                Err(_reason) => {
                    warn!("Disarm rejected");
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
        let custom_mode = cmd.param2 as u32;

        match FlightMode::from_custom_mode(custom_mode) {
            Some(mode) => match self.state.set_mode(mode) {
                Ok(()) => {
                    info!("Mode changed to {:?}", mode);
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(_reason) => {
                    warn!("Mode change rejected");
                    MavResult::MAV_RESULT_DENIED
                }
            },
            None => {
                warn!("Invalid mode number: {}", custom_mode);
                MavResult::MAV_RESULT_DENIED
            }
        }
    }

    /// Handle MAV_CMD_PREFLIGHT_CALIBRATION command
    ///
    /// Placeholder: Always accepts but does nothing.
    /// Real calibration would trigger sensor calibration routines.
    fn handle_preflight_calibration(&mut self, _cmd: &COMMAND_LONG_DATA) -> MavResult {
        info!("Preflight calibration requested (placeholder)");
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Handle MAV_CMD_REQUEST_PROTOCOL_VERSION command
    ///
    /// GCS requests the MAVLink protocol version being used.
    /// Response should be sent via PROTOCOL_VERSION message, not COMMAND_ACK result_param2.
    /// For now, we just acknowledge the command; actual PROTOCOL_VERSION message
    /// should be sent separately by the router/telemetry system.
    fn handle_request_protocol_version(&mut self, _cmd: &COMMAND_LONG_DATA) -> MavResult {
        debug!("Protocol version requested");
        // TODO: Send PROTOCOL_VERSION message via telemetry system
        // PROTOCOL_VERSION should contain:
        // - version: MAVLink protocol version (e.g., 200 for MAVLink 2.0)
        // - min_version: Minimum supported version
        // - max_version: Maximum supported version
        // - spec_version_hash: Git hash of mavlink/mavlink XML spec
        // - library_version_hash: Git hash of mavlink rust implementation
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Create PROTOCOL_VERSION message
    ///
    /// Returns a PROTOCOL_VERSION message containing MAVLink version information.
    /// This should be sent in response to MAV_CMD_REQUEST_PROTOCOL_VERSION.
    pub fn create_protocol_version_message() -> PROTOCOL_VERSION_DATA {
        PROTOCOL_VERSION_DATA {
            version: MAVLINK_VERSION,
            min_version: MAVLINK_MIN_VERSION,
            max_version: MAVLINK_MAX_VERSION,
            spec_version_hash: [0u8; 8], // TODO: Populate with actual MAVLink XML spec hash
            library_version_hash: [0u8; 8], // TODO: Populate with mavlink-rust version hash
        }
    }

    /// Create STATUSTEXT message
    ///
    /// Returns a STATUSTEXT message with the given severity and text.
    /// The text will be truncated to 50 characters if longer.
    ///
    /// # Arguments
    ///
    /// * `severity` - MAVLink severity level (e.g., MAV_SEVERITY_WARNING)
    /// * `text` - Status text message (will be truncated to 50 chars)
    fn create_statustext(severity: MavSeverity, text: &str) -> STATUSTEXT_DATA {
        let mut text_bytes = [0u8; 50];
        let bytes = text.as_bytes();
        let len = bytes.len().min(50);
        text_bytes[..len].copy_from_slice(&bytes[..len]);

        STATUSTEXT_DATA {
            severity,
            text: text_bytes,
        }
    }

    /// Get reference to current system state
    pub fn state(&self) -> &SystemState {
        &self.state
    }

    /// Get mutable system state reference for updates
    pub fn state_mut(&mut self) -> &mut SystemState {
        &mut self.state
    }

    /// Build HEARTBEAT message with current arm state
    ///
    /// This is sent immediately after arm/disarm to update GCS without waiting
    /// for the next scheduled HEARTBEAT from telemetry.
    fn build_heartbeat(&self) -> MavMessage {
        use mavlink::common::{MavAutopilot, MavModeFlag, MavState, MavType, HEARTBEAT_DATA};

        let base_mode = if self.state.is_armed() {
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
    fn test_arm_command_accepted() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0; // Good battery
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(handler.state().is_armed());
    }

    #[test]
    fn test_arm_command_denied_already_armed() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_arm_command_denied_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Critical voltage
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    fn test_disarm_command_accepted() {
        let mut state = SystemState::new();
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    fn test_disarm_command_denied_already_disarmed() {
        let state = SystemState::new(); // Default is disarmed
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_set_mode_accepted() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 3.0); // Mode 3 = Auto
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(handler.state().mode, FlightMode::Auto);
    }

    #[test]
    fn test_set_mode_denied_invalid() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 99.0); // Invalid mode
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_preflight_calibration_accepted() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_unsupported_command() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_NAV_WAYPOINT, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn test_command_ack_fields() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 0.0);

        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_request_protocol_version() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_PROTOCOL_VERSION, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_PROTOCOL_VERSION);
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
