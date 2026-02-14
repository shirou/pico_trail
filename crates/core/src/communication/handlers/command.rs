//! Command Protocol Handler
//!
//! Handles COMMAND_LONG and COMMAND_INT messages from ground control station for vehicle control.
//!
//! # Supported Commands (COMMAND_LONG)
//!
//! - **MAV_CMD_COMPONENT_ARM_DISARM**: Arm or disarm vehicle
//! - **MAV_CMD_DO_SET_MODE**: Change flight mode
//! - **MAV_CMD_DO_REPOSITION**: Navigate to position (Fly Here)
//! - **MAV_CMD_MISSION_START**: Start AUTO mode mission from waypoint 0
//! - **MAV_CMD_PREFLIGHT_CALIBRATION**: Sensor calibration (placeholder)
//! - **MAV_CMD_REQUEST_MESSAGE**: Request specific message (e.g., PROTOCOL_VERSION)
//! - **MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES** (520): Request AUTOPILOT_VERSION
//! - **MAV_CMD_REQUEST_CAMERA_INFORMATION** (521): Not supported (no camera)
//!
//! # Supported Commands (COMMAND_INT)
//!
//! - **MAV_CMD_DO_SET_HOME** (179): Set home position for RTL

use crate::autopilot::state::{FlightMode, SystemState};
use crate::autopilot::vehicle::VehicleType;
use crate::communication::status_notifier;
use core::fmt::Write;
use core::marker::PhantomData;
use heapless::{String, Vec};
use mavlink::common::{
    MavCmd, MavMessage, MavProtocolCapability, MavResult, AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA,
    COMMAND_INT_DATA, COMMAND_LONG_DATA, HOME_POSITION_DATA, PROTOCOL_VERSION_DATA,
};

mod mission_ops {
    use crate::autopilot::state::FlightMode;
    use crate::mission::{start_mission_from_beginning, start_mission_from_current, stop_mission};

    pub fn start_on_arm_if_guided(mode: FlightMode) {
        if mode == FlightMode::Guided && start_mission_from_current() {
            crate::log_info!("GUIDED: Mission started on ARM");
        } else if mode == FlightMode::Guided {
            crate::log_debug!("GUIDED: No waypoint, waiting for target");
        }
    }

    pub fn stop() {
        stop_mission();
    }

    pub fn start_from_beginning() -> Option<bool> {
        Some(start_mission_from_beginning())
    }
}

/// MAVLink protocol version (MAVLink 2.0)
pub const MAVLINK_VERSION: u16 = 200;

/// Minimum supported MAVLink protocol version
pub const MAVLINK_MIN_VERSION: u16 = 200;

/// Maximum supported MAVLink protocol version
pub const MAVLINK_MAX_VERSION: u16 = 200;

/// Command handler for COMMAND_LONG messages
pub struct CommandHandler<V: VehicleType> {
    _vehicle: PhantomData<V>,
}

impl<V: VehicleType> Default for CommandHandler<V> {
    fn default() -> Self {
        Self {
            _vehicle: PhantomData,
        }
    }
}

impl<V: VehicleType> CommandHandler<V> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn handle_command_long(
        &mut self,
        cmd: &COMMAND_LONG_DATA,
        sender_system_id: u8,
        sender_component_id: u8,
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
            MavCmd::MAV_CMD_DO_REPOSITION => (self.handle_do_reposition(cmd), false, Vec::new()),
            MavCmd::MAV_CMD_MISSION_START => (self.handle_mission_start(), false, Vec::new()),
            #[allow(deprecated)]
            MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => (
                self.handle_request_autopilot_capabilities(),
                false,
                Vec::new(),
            ),
            #[allow(deprecated)]
            MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION => {
                (MavResult::MAV_RESULT_UNSUPPORTED, false, Vec::new())
            }
            MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW => {
                (self.handle_fixed_mag_cal_yaw(cmd), false, Vec::new())
            }
            _ => {
                crate::log_warn!("Unsupported command: {}", cmd.command as u32);
                (MavResult::MAV_RESULT_UNSUPPORTED, false, Vec::new())
            }
        };

        let ack = COMMAND_ACK_DATA {
            command: cmd.command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: sender_system_id,
            target_component: sender_component_id,
        };

        let mut messages = extra_messages;
        if should_send_heartbeat {
            let _ = messages.push(self.build_heartbeat());
        }

        (ack, messages)
    }

    fn handle_arm_disarm(
        &mut self,
        cmd: &COMMAND_LONG_DATA,
    ) -> (MavResult, bool, Vec<MavMessage, 4>) {
        use crate::autopilot::state::SYSTEM_STATE;

        let should_arm = cmd.param1 > 0.5;
        let force = (cmd.param2 as i32) == 21196;

        if should_arm {
            let (result, mode) = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                let current_mode = state.mode;
                let arm_result = if force {
                    state.arm_forced()
                } else {
                    state.arm()
                };
                (arm_result, current_mode)
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

                    mission_ops::start_on_arm_if_guided(mode);

                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(reason) => {
                    let mut msg: String<64> = String::new();
                    match &reason {
                        crate::arming::ArmingError::CheckFailed {
                            reason: r,
                            category,
                        } => {
                            let _ = write!(msg, "PreArm: {}: {}", category, r);
                        }
                        crate::arming::ArmingError::InitializationFailed { subsystem } => {
                            let _ = write!(msg, "Arm: {} init failed", subsystem);
                        }
                        crate::arming::ArmingError::AlreadyArmed => {
                            let _ = write!(msg, "Arm: Already armed");
                        }
                    }
                    crate::log_warn!("{}", msg.as_str());
                    status_notifier::send_error(&msg);
                    (MavResult::MAV_RESULT_DENIED, false, Vec::new())
                }
            }
        } else {
            let result = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                if force {
                    state.disarm_forced()
                } else {
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

                    mission_ops::stop();

                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(reason) => {
                    let mut msg: String<64> = String::new();
                    match &reason {
                        crate::arming::DisarmError::ValidationFailed { reason: r } => {
                            let _ = write!(msg, "Disarm: {}", r);
                        }
                        crate::arming::DisarmError::NotArmed => {
                            let _ = write!(msg, "Disarm: Not armed");
                        }
                        crate::arming::DisarmError::ThrottleNotLow { current } => {
                            let _ = write!(msg, "Disarm: Throttle {:.0}% (needs <10%)", current);
                        }
                        crate::arming::DisarmError::VelocityTooHigh { current, max } => {
                            let _ = write!(msg, "Disarm: Speed {:.1}m/s (max {:.1})", current, max);
                        }
                    }
                    crate::log_warn!("{}", msg.as_str());
                    status_notifier::send_error(&msg);
                    (MavResult::MAV_RESULT_DENIED, false, Vec::new())
                }
            }
        }
    }

    fn handle_set_mode(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::autopilot::state::SYSTEM_STATE;

        let custom_mode = cmd.param2 as u32;

        match FlightMode::from_custom_mode(custom_mode) {
            Some(mode) => {
                let result = critical_section::with(|cs| {
                    let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                    state.set_mode(mode)
                });
                match result {
                    Ok(()) => {
                        crate::log_info!("Mode changed to {}", mode.as_str());
                        mission_ops::stop();

                        let mut msg: String<32> = String::new();
                        let _ = write!(msg, "Mode: {}", mode.as_str());
                        status_notifier::send_info(&msg);
                        MavResult::MAV_RESULT_ACCEPTED
                    }
                    Err(_reason) => {
                        crate::log_warn!("Mode change rejected");
                        status_notifier::send_error("Mode change rejected");
                        MavResult::MAV_RESULT_DENIED
                    }
                }
            }
            None => {
                crate::log_warn!("Invalid mode number: {}", custom_mode);
                let mut msg: String<32> = String::new();
                let _ = write!(msg, "Invalid mode: {}", custom_mode);
                status_notifier::send_error(&msg);
                MavResult::MAV_RESULT_DENIED
            }
        }
    }

    fn handle_preflight_calibration(&mut self, _cmd: &COMMAND_LONG_DATA) -> MavResult {
        crate::log_info!("Preflight calibration requested (placeholder)");
        MavResult::MAV_RESULT_ACCEPTED
    }

    fn handle_request_message(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        let message_id = cmd.param1 as u32;

        const MAVLINK_MSG_ID_AUTOPILOT_VERSION: u32 = 148;
        const MAVLINK_MSG_ID_PROTOCOL_VERSION: u32 = 300;
        const MAVLINK_MSG_ID_CAMERA_INFORMATION: u32 = 259;
        const MAVLINK_MSG_ID_CAMERA_SETTINGS: u32 = 260;
        const MAVLINK_MSG_ID_STORAGE_INFORMATION: u32 = 261;
        const MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: u32 = 262;
        const MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: u32 = 269;
        const MAVLINK_MSG_ID_VIDEO_STREAM_STATUS: u32 = 270;

        match message_id {
            MAVLINK_MSG_ID_PROTOCOL_VERSION => {
                crate::log_debug!("Protocol version requested via MAV_CMD_REQUEST_MESSAGE");
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAVLINK_MSG_ID_AUTOPILOT_VERSION => {
                crate::log_debug!("Autopilot version requested via MAV_CMD_REQUEST_MESSAGE");
                MavResult::MAV_RESULT_ACCEPTED
            }
            MAVLINK_MSG_ID_CAMERA_INFORMATION
            | MAVLINK_MSG_ID_CAMERA_SETTINGS
            | MAVLINK_MSG_ID_STORAGE_INFORMATION
            | MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS
            | MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION
            | MAVLINK_MSG_ID_VIDEO_STREAM_STATUS => {
                crate::log_debug!(
                    "Camera/Video message {} requested (not supported)",
                    message_id
                );
                MavResult::MAV_RESULT_UNSUPPORTED
            }
            _ => {
                crate::log_warn!("Unsupported message ID in REQUEST_MESSAGE: {}", message_id);
                MavResult::MAV_RESULT_UNSUPPORTED
            }
        }
    }

    fn handle_request_autopilot_capabilities(&mut self) -> MavResult {
        crate::log_debug!("Autopilot capabilities requested (command 520)");
        MavResult::MAV_RESULT_ACCEPTED
    }

    fn handle_do_reposition(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::autopilot::state::{FlightMode, SYSTEM_STATE};
        use crate::mission::{set_mission_state, set_single_waypoint, MissionState, Waypoint};

        let latitude = cmd.param5;
        let longitude = cmd.param6;
        let altitude = if cmd.param7.is_finite() && cmd.param7 != 0.0 {
            Some(cmd.param7)
        } else {
            None
        };

        if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
            crate::log_warn!(
                "Invalid reposition target: lat={}, lon={}",
                latitude,
                longitude
            );
            status_notifier::send_error("Invalid position");
            return MavResult::MAV_RESULT_DENIED;
        }

        // Create waypoint from reposition target
        let waypoint = Waypoint {
            seq: 0,
            frame: 0,    // MAV_FRAME_GLOBAL
            command: 16, // MAV_CMD_NAV_WAYPOINT
            current: 1,
            autocontinue: 0,
            param1: 0.0,
            param2: 2.0, // WP_RADIUS default
            param3: 0.0,
            param4: 0.0,
            x: (latitude * 1e7) as i32,
            y: (longitude * 1e7) as i32,
            z: altitude.unwrap_or(0.0),
        };

        // Update mission storage with single waypoint
        set_single_waypoint(waypoint);

        // If in Guided mode, start navigation immediately
        let mode = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode);
        if mode == FlightMode::Guided {
            set_mission_state(MissionState::Running);
            crate::log_info!(
                "GUIDED: Navigation started to lat={}, lon={}",
                latitude,
                longitude
            );
        } else {
            crate::log_info!("Reposition target set: lat={}, lon={}", latitude, longitude);
        }

        status_notifier::send_info("Fly to target set");
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Handle DO_REPOSITION via COMMAND_INT (int32 lat/lon for higher precision).
    fn handle_do_reposition_int(&mut self, cmd: &COMMAND_INT_DATA) -> MavResult {
        use crate::autopilot::state::{FlightMode, SYSTEM_STATE};
        use crate::mission::{set_mission_state, set_single_waypoint, MissionState, Waypoint};

        let latitude = cmd.x as f64 / 1e7;
        let longitude = cmd.y as f64 / 1e7;
        let altitude = if cmd.z.is_finite() && cmd.z != 0.0 {
            Some(cmd.z)
        } else {
            None
        };

        if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
            crate::log_warn!(
                "Invalid reposition target: lat={}, lon={}",
                latitude,
                longitude
            );
            status_notifier::send_error("Invalid position");
            return MavResult::MAV_RESULT_DENIED;
        }

        let waypoint = Waypoint {
            seq: 0,
            frame: 0,
            command: 16,
            current: 1,
            autocontinue: 0,
            param1: 0.0,
            param2: 2.0,
            param3: 0.0,
            param4: 0.0,
            x: cmd.x,
            y: cmd.y,
            z: altitude.unwrap_or(0.0),
        };

        set_single_waypoint(waypoint);

        let mode = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode);
        if mode == FlightMode::Guided {
            set_mission_state(MissionState::Running);
            crate::log_info!(
                "GUIDED: Navigation started to lat={}, lon={}",
                latitude,
                longitude
            );
        } else {
            crate::log_info!("Reposition target set: lat={}, lon={}", latitude, longitude);
        }

        status_notifier::send_info("Fly to target set");
        MavResult::MAV_RESULT_ACCEPTED
    }

    fn handle_mission_start(&mut self) -> MavResult {
        match mission_ops::start_from_beginning() {
            Some(true) => {
                crate::log_info!("AUTO: Mission started from waypoint 0");
                status_notifier::send_info("Mission started");
                MavResult::MAV_RESULT_ACCEPTED
            }
            Some(false) => {
                crate::log_warn!("MISSION_START failed: mission empty");
                status_notifier::send_error("Mission empty");
                MavResult::MAV_RESULT_FAILED
            }
            None => MavResult::MAV_RESULT_UNSUPPORTED,
        }
    }

    fn handle_fixed_mag_cal_yaw(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::autopilot::state::SYSTEM_STATE;
        use crate::navigation::GpsFixType;

        let true_yaw_degrees = cmd.param1;

        let (gps_state, current_ahrs_yaw) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.gps_position, state.attitude.yaw)
        });

        match gps_state {
            Some(gps) if gps.fix_type == GpsFixType::Fix3D => {
                let true_yaw_rad = true_yaw_degrees.to_radians();
                let yaw_offset = true_yaw_rad - current_ahrs_yaw;
                let yaw_offset = Self::normalize_angle(yaw_offset);

                critical_section::with(|cs| {
                    let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                    state.compass_yaw_offset = yaw_offset;
                });

                crate::log_info!(
                    "MagCal: true={} deg, AHRS={} deg, offset={} deg",
                    true_yaw_degrees,
                    current_ahrs_yaw.to_degrees(),
                    yaw_offset.to_degrees()
                );
                status_notifier::send_info("MagCal: Calibrated");
                MavResult::MAV_RESULT_ACCEPTED
            }
            Some(_gps) => {
                crate::log_warn!("MagCal: Denied - need 3D GPS fix");
                status_notifier::send_warning("MagCal: Need 3D GPS fix");
                MavResult::MAV_RESULT_DENIED
            }
            None => {
                crate::log_warn!("MagCal: Denied - No GPS fix");
                status_notifier::send_warning("MagCal: No GPS fix");
                MavResult::MAV_RESULT_DENIED
            }
        }
    }

    fn normalize_angle(angle: f32) -> f32 {
        let mut result = angle;
        while result > core::f32::consts::PI {
            result -= 2.0 * core::f32::consts::PI;
        }
        while result < -core::f32::consts::PI {
            result += 2.0 * core::f32::consts::PI;
        }
        result
    }

    pub fn handle_command_int(
        &mut self,
        cmd: &COMMAND_INT_DATA,
        sender_system_id: u8,
        sender_component_id: u8,
    ) -> (COMMAND_ACK_DATA, Vec<MavMessage, 4>) {
        crate::log_debug!("Received COMMAND_INT: command={}", cmd.command as u32);

        let (result, extra_messages) = match cmd.command {
            MavCmd::MAV_CMD_DO_SET_HOME => self.handle_set_home(cmd),
            MavCmd::MAV_CMD_DO_REPOSITION => (self.handle_do_reposition_int(cmd), Vec::new()),
            _ => {
                crate::log_warn!("Unsupported COMMAND_INT: {}", cmd.command as u32);
                (MavResult::MAV_RESULT_UNSUPPORTED, Vec::new())
            }
        };

        let ack = COMMAND_ACK_DATA {
            command: cmd.command,
            result,
            progress: 0,
            result_param2: 0,
            target_system: sender_system_id,
            target_component: sender_component_id,
        };

        (ack, extra_messages)
    }

    fn handle_set_home(&mut self, cmd: &COMMAND_INT_DATA) -> (MavResult, Vec<MavMessage, 4>) {
        use crate::autopilot::state::{HomePosition, SYSTEM_STATE};

        let use_current = cmd.param1 > 0.5;

        if use_current {
            let result = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                state.set_home_to_current()
            });

            match result {
                Ok(()) => {
                    crate::log_info!("Home set to current location");
                    status_notifier::send_info("Home set to current");

                    let home_msg = critical_section::with(|cs| {
                        let state = SYSTEM_STATE.borrow_ref(cs);
                        state
                            .home_position
                            .as_ref()
                            .map(|h| self.build_home_position_message(h))
                    });

                    let mut messages = Vec::new();
                    if let Some(msg) = home_msg {
                        let _ = messages.push(MavMessage::HOME_POSITION(msg));
                    }

                    (MavResult::MAV_RESULT_ACCEPTED, messages)
                }
                Err(_reason) => {
                    crate::log_warn!("Failed to set home: no GPS fix");
                    status_notifier::send_error("No GPS fix for home");
                    (MavResult::MAV_RESULT_DENIED, Vec::new())
                }
            }
        } else {
            let home = HomePosition::from_command_int(cmd.x, cmd.y, cmd.z);

            if !(-90.0..=90.0).contains(&home.latitude)
                || !(-180.0..=180.0).contains(&home.longitude)
            {
                crate::log_warn!(
                    "Invalid home position: lat={}, lon={}",
                    home.latitude,
                    home.longitude
                );
                status_notifier::send_error("Invalid home position");
                return (MavResult::MAV_RESULT_DENIED, Vec::new());
            }

            let home_pos_msg = self.build_home_position_message(&home);

            critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                state.set_home(home);
            });

            crate::log_info!(
                "Home set to lat={}, lon={}, alt={}",
                home.latitude,
                home.longitude,
                home.altitude
            );
            let mut msg: String<48> = String::new();
            let _ = write!(msg, "Home: {:.5},{:.5}", home.latitude, home.longitude);
            status_notifier::send_info(&msg);

            let mut messages = Vec::new();
            let _ = messages.push(MavMessage::HOME_POSITION(home_pos_msg));

            (MavResult::MAV_RESULT_ACCEPTED, messages)
        }
    }

    fn build_home_position_message(
        &self,
        home: &crate::autopilot::state::HomePosition,
    ) -> HOME_POSITION_DATA {
        HOME_POSITION_DATA {
            latitude: (home.latitude * 1e7) as i32,
            longitude: (home.longitude * 1e7) as i32,
            altitude: (home.altitude * 1000.0) as i32,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            q: [f32::NAN, f32::NAN, f32::NAN, f32::NAN],
            approach_x: 0.0,
            approach_y: 0.0,
            approach_z: 0.0,
            time_usec: 0,
        }
    }

    pub fn create_protocol_version_message() -> PROTOCOL_VERSION_DATA {
        PROTOCOL_VERSION_DATA {
            version: MAVLINK_VERSION,
            min_version: MAVLINK_MIN_VERSION,
            max_version: MAVLINK_MAX_VERSION,
            spec_version_hash: [0u8; 8],
            library_version_hash: [0u8; 8],
        }
    }

    pub fn create_autopilot_version_message() -> AUTOPILOT_VERSION_DATA {
        let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;

        #[allow(clippy::identity_op)]
        let flight_sw_version: u32 = (0 << 24) | (1 << 16) | (0 << 8) | 0;

        AUTOPILOT_VERSION_DATA {
            capabilities,
            uid: 0,
            flight_sw_version,
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            flight_custom_version: [0u8; 8],
            middleware_custom_version: [0u8; 8],
            os_custom_version: [0u8; 8],
            uid2: [0u8; 18],
        }
    }

    pub fn state(&self) -> SystemState {
        use crate::autopilot::state::SYSTEM_STATE;
        critical_section::with(|cs| *SYSTEM_STATE.borrow_ref(cs))
    }

    fn build_heartbeat(&self) -> MavMessage {
        use crate::autopilot::state::SYSTEM_STATE;
        use mavlink::common::{MavModeFlag, MavState, HEARTBEAT_DATA};

        let (is_armed, mode) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.is_armed(), state.mode)
        });

        let mut base_mode = MavModeFlag::from_bits_truncate(mode.to_base_mode_flags());

        if is_armed {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }

        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: mode.to_custom_mode(),
            mavtype: crate::autopilot::vehicle::to_mav_type::<V>(),
            autopilot: crate::autopilot::vehicle::to_mav_autopilot::<V>(),
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::autopilot::vehicle::GroundRover;
    use mavlink::common::MavCmd;
    use std::vec;

    type TestHandler = CommandHandler<GroundRover>;

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
        state.battery.voltage = 12.0;
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(ack.target_system, 255);
        assert_eq!(ack.target_component, 1);
        assert!(handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_arm_command_denied_already_armed() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;
        state.armed = crate::autopilot::state::ArmedState::Armed;
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_arm_command_denied_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0;
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_disarm_command_accepted() {
        let mut state = SystemState::new();
        state.armed = crate::autopilot::state::ArmedState::Armed;
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert!(!handler.state().is_armed());
    }

    #[test]
    #[serial_test::serial]
    fn test_set_mode_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 10.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(handler.state().mode, FlightMode::Auto);
    }

    #[test]
    #[serial_test::serial]
    fn test_set_mode_denied_invalid() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 99.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_unsupported_command() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_NAV_WAYPOINT, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn test_create_protocol_version_message() {
        let msg = TestHandler::create_protocol_version_message();
        assert_eq!(msg.version, 200);
        assert_eq!(msg.min_version, 200);
        assert_eq!(msg.max_version, 200);
    }

    #[test]
    fn test_autopilot_version_has_compass_calibration_capability() {
        let msg = TestHandler::create_autopilot_version_message();
        assert!(msg
            .capabilities
            .contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION),);
    }

    #[test]
    #[serial_test::serial]
    fn test_do_reposition_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            confirmation: 0,
            param1: -1.0,
            param2: 0.0,
            param3: 0.0,
            param4: f32::NAN,
            param5: 35.6762,
            param6: 139.6503,
            param7: 100.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_do_reposition_invalid_latitude() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            confirmation: 0,
            param1: -1.0,
            param2: 0.0,
            param3: 0.0,
            param4: f32::NAN,
            param5: 91.0,
            param6: 139.6503,
            param7: 100.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_specified_location() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let lat_e7 = (35.6762 * 1e7) as i32;
        let lon_e7 = (139.6503 * 1e7) as i32;
        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: lat_e7,
            y: lon_e7,
            z: 100.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::HOME_POSITION(_)));
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_current_location_no_gps() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(messages.is_empty());
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_current_location_with_gps() {
        use crate::navigation::{GpsFixType, GpsPosition};

        let mut state = SystemState::new();
        state.gps_position = Some(GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 50.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        });
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: 1.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(messages.len(), 1);
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_accepted_with_3d_gps() {
        use crate::navigation::{GpsFixType, GpsPosition};

        let mut state = SystemState::new();
        state.gps_position = Some(GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 50.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        });
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW,
            confirmation: 0,
            param1: 90.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_denied_without_gps() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW,
            confirmation: 0,
            param1: 180.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_disarm_command_denied_already_disarmed() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_preflight_calibration_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_command_ack_fields() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_request_protocol_version() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 300.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_request_autopilot_version() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_request_camera_info_unsupported() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 259.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_do_reposition_invalid_longitude() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            confirmation: 0,
            param1: -1.0,
            param2: 0.0,
            param3: 0.0,
            param4: f32::NAN,
            param5: 35.6762,
            param6: 181.0,
            param7: 100.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_do_reposition_negative_coords() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            confirmation: 0,
            param1: -1.0,
            param2: 0.0,
            param3: 0.0,
            param4: f32::NAN,
            param5: 40.7128,
            param6: -74.0060,
            param7: 50.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_invalid_coordinates() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let lat_e7 = (91.0 * 1e7) as i32;
        let lon_e7 = (139.0 * 1e7) as i32;
        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: lat_e7,
            y: lon_e7,
            z: 100.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        assert!(messages.is_empty());
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_negative_coordinates() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let lat_e7 = (-34.6037 * 1e7) as i32;
        let lon_e7 = (-58.3816 * 1e7) as i32;
        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: lat_e7,
            y: lon_e7,
            z: 25.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::HOME_POSITION(_)));
    }

    #[test]
    #[serial_test::serial]
    fn test_command_int_unsupported() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_NAV_WAYPOINT,
            current: 0,
            autocontinue: 0,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
        };
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_NAV_WAYPOINT);
        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
        assert!(messages.is_empty());
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_denied_with_2d_gps() {
        use crate::navigation::{GpsFixType, GpsPosition};

        let mut state = SystemState::new();
        state.gps_position = Some(GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 0.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix2D,
            satellites: 4,
        });
        critical_section::with(|cs| {
            *crate::autopilot::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW,
            confirmation: 0,
            param1: 270.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }
}
