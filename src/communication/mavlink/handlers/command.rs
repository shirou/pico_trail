//! Command Protocol Handler
//!
//! Handles COMMAND_LONG messages from ground control station for vehicle control.
//!
//! # Supported Commands
//!
//! - **MAV_CMD_COMPONENT_ARM_DISARM**: Arm or disarm vehicle
//! - **MAV_CMD_DO_SET_MODE**: Change flight mode
//! - **MAV_CMD_PREFLIGHT_CALIBRATION**: Sensor calibration (placeholder)
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
use mavlink::common::{MavCmd, MavResult, COMMAND_ACK_DATA, COMMAND_LONG_DATA};

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
    /// Returns COMMAND_ACK message to send back to GCS.
    pub fn handle_command_long(&mut self, cmd: &COMMAND_LONG_DATA) -> COMMAND_ACK_DATA {
        debug!("Received COMMAND_LONG: command={:?}", cmd.command);

        let result = match cmd.command {
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => self.handle_arm_disarm(cmd),
            MavCmd::MAV_CMD_DO_SET_MODE => self.handle_set_mode(cmd),
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION => self.handle_preflight_calibration(cmd),
            _ => {
                warn!("Unsupported command: {:?}", cmd.command);
                MavResult::MAV_RESULT_UNSUPPORTED
            }
        };

        COMMAND_ACK_DATA {
            command: cmd.command,
            result,
        }
    }

    /// Handle MAV_CMD_COMPONENT_ARM_DISARM command
    ///
    /// param1: 1.0 to arm, 0.0 to disarm
    fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        let should_arm = cmd.param1 > 0.5;

        if should_arm {
            // Arm vehicle
            match self.state.arm() {
                Ok(()) => {
                    info!("Vehicle armed");
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(_reason) => {
                    warn!("Arm rejected");
                    MavResult::MAV_RESULT_DENIED
                }
            }
        } else {
            // Disarm vehicle
            match self.state.disarm() {
                Ok(()) => {
                    info!("Vehicle disarmed");
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(_reason) => {
                    warn!("Disarm rejected");
                    MavResult::MAV_RESULT_DENIED
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

    /// Get reference to current system state
    pub fn state(&self) -> &SystemState {
        &self.state
    }

    /// Get mutable system state reference for updates
    pub fn state_mut(&mut self) -> &mut SystemState {
        &mut self.state
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
        let ack = handler.handle_command_long(&cmd);

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
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_arm_command_denied_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Critical voltage
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    fn test_disarm_command_accepted() {
        let mut state = SystemState::new();
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    fn test_disarm_command_denied_already_disarmed() {
        let state = SystemState::new(); // Default is disarmed
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_set_mode_accepted() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 3.0); // Mode 3 = Auto
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(handler.state().mode, FlightMode::Auto);
    }

    #[test]
    fn test_set_mode_denied_invalid() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 99.0); // Invalid mode
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_preflight_calibration_accepted() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0);
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_unsupported_command() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_NAV_WAYPOINT, 0.0, 0.0);
        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn test_command_ack_fields() {
        let state = SystemState::new();
        let mut handler = CommandHandler::new(state);

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 0.0);

        let ack = handler.handle_command_long(&cmd);

        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }
}
