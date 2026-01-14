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
//!
//! # Command Flow
//!
//! 1. GCS sends COMMAND_LONG or COMMAND_INT message
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
use crate::communication::mavlink::vehicle::VehicleType;
use core::fmt::Write;
use core::marker::PhantomData;
use heapless::{String, Vec};
use mavlink::common::{
    MavCmd, MavMessage, MavProtocolCapability, MavResult, AUTOPILOT_VERSION_DATA, COMMAND_ACK_DATA,
    COMMAND_INT_DATA, COMMAND_LONG_DATA, HOME_POSITION_DATA, PROTOCOL_VERSION_DATA,
};

// Mission operations module - embassy-specific implementations
#[cfg(feature = "embassy")]
mod mission_ops {
    use crate::communication::mavlink::state::FlightMode;
    use crate::core::mission::{
        start_mission_from_beginning, start_mission_from_current, stop_mission,
    };

    /// Start mission if in GUIDED mode (called on arm)
    pub fn start_on_arm_if_guided(mode: FlightMode) {
        if mode == FlightMode::Guided && start_mission_from_current() {
            crate::log_info!("GUIDED: Mission started on ARM");
        } else if mode == FlightMode::Guided {
            crate::log_debug!("GUIDED: No waypoint, waiting for target");
        }
    }

    /// Stop mission (called on disarm or mode change)
    pub fn stop() {
        stop_mission();
    }

    /// Start mission from beginning (called by MISSION_START command)
    /// Returns Some(true) if started, Some(false) if empty, None if not supported
    pub fn start_from_beginning() -> Option<bool> {
        Some(start_mission_from_beginning())
    }
}

// Mission operations module - stub implementations for non-embassy builds
#[cfg(not(feature = "embassy"))]
mod mission_ops {
    use crate::communication::mavlink::state::FlightMode;

    pub fn start_on_arm_if_guided(_mode: FlightMode) {}

    pub fn stop() {}

    pub fn start_from_beginning() -> Option<bool> {
        crate::log_warn!("MISSION_START not supported without embassy");
        None
    }
}

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

    /// Handle COMMAND_LONG message from GCS
    ///
    /// Returns COMMAND_ACK message and optional additional messages (HEARTBEAT, STATUSTEXT) to send back to GCS.
    /// The HEARTBEAT is sent immediately after arm/disarm to update GCS state without waiting.
    /// The STATUSTEXT is sent for force-arm/disarm operations to warn the operator.
    ///
    /// # Arguments
    ///
    /// * `cmd` - The COMMAND_LONG message data
    /// * `sender_system_id` - System ID of the sender (from MAVLink header)
    /// * `sender_component_id` - Component ID of the sender (from MAVLink header)
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
            // These commands are marked deprecated in mavlink crate but still used by Mission Planner
            #[allow(deprecated)]
            MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => (
                self.handle_request_autopilot_capabilities(),
                false,
                Vec::new(),
            ),
            #[allow(deprecated)]
            MavCmd::MAV_CMD_REQUEST_CAMERA_INFORMATION => {
                // Camera not supported - silently return unsupported to avoid log spam
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
            progress: 0,                           // MAVLink v2 extension
            result_param2: 0,                      // MAVLink v2 extension
            target_system: sender_system_id,       // Reply to sender
            target_component: sender_component_id, // Reply to sender
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
    /// In GUIDED mode: automatically starts mission if waypoint exists on ARM.
    /// On DISARM: stops any active mission.
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
            // Arm vehicle (also get mode for potential mission start)
            let (result, mode) = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                let current_mode = state.mode;
                let arm_result = if force {
                    // Force-arm: bypass pre-arm checks
                    state.arm_forced()
                } else {
                    // Normal arm: run pre-arm checks
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

                    // In GUIDED mode, start mission if waypoint exists
                    mission_ops::start_on_arm_if_guided(mode);

                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(reason) => {
                    // Build detailed error message for GCS
                    let mut msg: String<64> = String::new();
                    match &reason {
                        crate::core::arming::ArmingError::CheckFailed {
                            reason: r,
                            category,
                        } => {
                            let _ = write!(msg, "PreArm: {}: {}", category, r);
                        }
                        crate::core::arming::ArmingError::InitializationFailed { subsystem } => {
                            let _ = write!(msg, "Arm: {} init failed", subsystem);
                        }
                        crate::core::arming::ArmingError::AlreadyArmed => {
                            let _ = write!(msg, "Arm: Already armed");
                        }
                    }
                    crate::log_warn!("{}", msg.as_str());
                    status_notifier::send_error(&msg);
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

                    // Stop any active mission on disarm
                    mission_ops::stop();

                    (MavResult::MAV_RESULT_ACCEPTED, true, Vec::new())
                }
                Err(reason) => {
                    // Build detailed error message for GCS
                    let mut msg: String<64> = String::new();
                    match &reason {
                        crate::core::arming::DisarmError::ValidationFailed { reason: r } => {
                            let _ = write!(msg, "Disarm: {}", r);
                        }
                        crate::core::arming::DisarmError::NotArmed => {
                            let _ = write!(msg, "Disarm: Not armed");
                        }
                        crate::core::arming::DisarmError::ThrottleNotLow { current } => {
                            let _ = write!(msg, "Disarm: Throttle {:.0}% (needs <10%)", current);
                        }
                        crate::core::arming::DisarmError::VelocityTooHigh { current, max } => {
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
                        crate::log_info!("Mode changed to {}", mode.as_str());

                        // Reset mission state on mode change
                        // This stops any active navigation when switching modes
                        mission_ops::stop();

                        // Send STATUSTEXT to GCS
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

        // Common MAVLink message IDs
        const MAVLINK_MSG_ID_AUTOPILOT_VERSION: u32 = 148;
        const MAVLINK_MSG_ID_PROTOCOL_VERSION: u32 = 300;
        // Camera/Video-related message IDs (not supported on this rover)
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
            // Camera/Video-related messages: gracefully decline without warning (no camera on this rover)
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

    /// Handle MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (command 520)
    ///
    /// Requests AUTOPILOT_VERSION message to be sent.
    /// The actual AUTOPILOT_VERSION message will be sent by the telemetry handler.
    fn handle_request_autopilot_capabilities(&mut self) -> MavResult {
        crate::log_debug!("Autopilot capabilities requested (command 520)");
        // The actual AUTOPILOT_VERSION message will be sent by the telemetry handler
        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Handle MAV_CMD_DO_REPOSITION command (Fly Here)
    ///
    /// Sets navigation target for guided navigation.
    /// Mission Planner sends this when user clicks "Fly to here".
    ///
    /// # Parameters
    /// - param1: Ground speed (m/s), -1 for default (ignored for now)
    /// - param4: Yaw heading (degrees), NaN for unchanged (ignored for now)
    /// - param5: Latitude (degrees)
    /// - param6: Longitude (degrees)
    /// - param7: Altitude (meters)
    fn handle_do_reposition(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::subsystems::navigation::{set_reposition_target, PositionTarget};

        let latitude = cmd.param5;
        let longitude = cmd.param6;
        let altitude = if cmd.param7.is_finite() && cmd.param7 != 0.0 {
            Some(cmd.param7)
        } else {
            None
        };

        // Validate coordinates
        if !(-90.0..=90.0).contains(&latitude) || !(-180.0..=180.0).contains(&longitude) {
            crate::log_warn!(
                "Invalid reposition target: lat={}, lon={}",
                latitude,
                longitude
            );
            status_notifier::send_error("Invalid position");
            return MavResult::MAV_RESULT_DENIED;
        }

        let target = PositionTarget {
            latitude,
            longitude,
            altitude,
        };

        // Set reposition target (will be picked up by navigation_task)
        set_reposition_target(target);

        crate::log_info!("Reposition target set: lat={}, lon={}", latitude, longitude);
        status_notifier::send_info("Fly to target set");

        MavResult::MAV_RESULT_ACCEPTED
    }

    /// Handle MAV_CMD_MISSION_START command
    ///
    /// Starts mission execution from waypoint 0 in AUTO mode.
    ///
    /// # Behavior
    ///
    /// - Verifies mission is not empty
    /// - Resets current waypoint index to 0
    /// - Sets MissionState to Running
    ///
    /// # Returns
    ///
    /// - MAV_RESULT_ACCEPTED if mission started successfully
    /// - MAV_RESULT_FAILED if mission is empty
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

    /// Handle MAV_CMD_FIXED_MAG_CAL_YAW command (Large Vehicle MagCal)
    ///
    /// Mission Planner sends this command for compass calibration on large vehicles
    /// that cannot be rotated. Requires valid 3D GPS fix to determine true heading.
    ///
    /// # Parameters
    /// - param1: Yaw angle in degrees (0-360, magnetic north = 0)
    /// - param2: CompassMask (bitmask of compasses to calibrate)
    /// - param3: Latitude (optional)
    /// - param4: Longitude (optional)
    ///
    /// # Returns
    /// - MAV_RESULT_ACCEPTED if GPS fix is valid (>= 3D fix)
    /// - MAV_RESULT_DENIED if no GPS fix available
    fn handle_fixed_mag_cal_yaw(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
        use crate::communication::mavlink::state::SYSTEM_STATE;
        use crate::devices::gps::GpsFixType;

        // param1 is the true yaw heading in degrees (what direction vehicle is actually pointing)
        let true_yaw_degrees = cmd.param1;

        // Get GPS state and current AHRS yaw
        let (gps_state, current_ahrs_yaw) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.gps_position, state.attitude.yaw)
        });

        match gps_state {
            Some(gps) if gps.fix_type == GpsFixType::Fix3D => {
                // Convert true yaw from degrees to radians
                let true_yaw_rad = true_yaw_degrees.to_radians();

                // Calculate offset: true_heading - ahrs_reading
                // This offset will be added to AHRS yaw to get corrected heading
                let yaw_offset = true_yaw_rad - current_ahrs_yaw;

                // Normalize offset to [-PI, PI]
                let yaw_offset = Self::normalize_angle(yaw_offset);

                // Store the offset in SystemState
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

    /// Normalize angle to [-PI, PI] range
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

    /// Handle COMMAND_INT message from GCS
    ///
    /// COMMAND_INT is preferred over COMMAND_LONG for commands with position data
    /// because it uses integer lat/lon (degrees * 10^7) for higher precision.
    ///
    /// # Supported Commands
    ///
    /// - MAV_CMD_DO_SET_HOME (179): Set home position
    ///
    /// Returns COMMAND_ACK message and optional additional messages (HOME_POSITION) to send back to GCS.
    ///
    /// # Arguments
    ///
    /// * `cmd` - The COMMAND_INT message data
    /// * `sender_system_id` - System ID of the sender (from MAVLink header)
    /// * `sender_component_id` - Component ID of the sender (from MAVLink header)
    pub fn handle_command_int(
        &mut self,
        cmd: &COMMAND_INT_DATA,
        sender_system_id: u8,
        sender_component_id: u8,
    ) -> (COMMAND_ACK_DATA, Vec<MavMessage, 4>) {
        crate::log_debug!("Received COMMAND_INT: command={}", cmd.command as u32);

        let (result, extra_messages) = match cmd.command {
            MavCmd::MAV_CMD_DO_SET_HOME => self.handle_set_home(cmd),
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
            target_system: sender_system_id,       // Reply to sender
            target_component: sender_component_id, // Reply to sender
        };

        (ack, extra_messages)
    }

    /// Handle MAV_CMD_DO_SET_HOME command
    ///
    /// Sets the home position for RTL mode.
    ///
    /// # Parameters (COMMAND_INT)
    /// - param1: 1=use current location, 0=use specified location
    /// - x: Latitude in degrees * 10^7
    /// - y: Longitude in degrees * 10^7
    /// - z: Altitude in meters
    ///
    /// Returns the result and a HOME_POSITION message on success.
    fn handle_set_home(&mut self, cmd: &COMMAND_INT_DATA) -> (MavResult, Vec<MavMessage, 4>) {
        use crate::communication::mavlink::state::{HomePosition, SYSTEM_STATE};

        let use_current = cmd.param1 > 0.5;

        if use_current {
            // Set home to current GPS position
            let result = critical_section::with(|cs| {
                let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
                state.set_home_to_current()
            });

            match result {
                Ok(()) => {
                    crate::log_info!("Home set to current location");
                    status_notifier::send_info("Home set to current");

                    // Get the home position that was just set
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
            // Set home to specified location
            let home = HomePosition::from_command_int(cmd.x, cmd.y, cmd.z);

            // Validate coordinates
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

            // Build HOME_POSITION message before setting (we have the data)
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

    /// Build HOME_POSITION message from HomePosition
    fn build_home_position_message(
        &self,
        home: &crate::communication::mavlink::state::HomePosition,
    ) -> HOME_POSITION_DATA {
        HOME_POSITION_DATA {
            // Latitude/longitude in degE7 (degrees * 10^7)
            latitude: (home.latitude * 1e7) as i32,
            longitude: (home.longitude * 1e7) as i32,
            // Altitude in mm (MSL)
            altitude: (home.altitude * 1000.0) as i32,
            // Local position (NED) - set to 0 as we don't track local frame
            x: 0.0,
            y: 0.0,
            z: 0.0,
            // Quaternion - set to identity (no rotation info available)
            // NaN indicates unknown as per MAVLink spec
            q: [f32::NAN, f32::NAN, f32::NAN, f32::NAN],
            // Approach vector - not used for ground vehicles
            approach_x: 0.0,
            approach_y: 0.0,
            approach_z: 0.0,
            // Timestamp - 0 indicates not available
            time_usec: 0,
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

    /// Create AUTOPILOT_VERSION message
    ///
    /// Returns an AUTOPILOT_VERSION message containing autopilot capabilities and version info.
    /// This should be sent in response to:
    /// - MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (command 520)
    /// - MAV_CMD_REQUEST_MESSAGE with param1=148
    pub fn create_autopilot_version_message() -> AUTOPILOT_VERSION_DATA {
        // Define capabilities we support
        let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
            | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;

        // Firmware version: major.minor.patch.type encoded as 4 bytes
        // Version 0.1.0 with FIRMWARE_VERSION_TYPE_DEV (0)
        #[allow(clippy::identity_op)]
        let flight_sw_version: u32 = (0 << 24) | (1 << 16) | (0 << 8) | 0;

        AUTOPILOT_VERSION_DATA {
            capabilities,
            uid: 0, // No hardware UID available
            flight_sw_version,
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            flight_custom_version: [0u8; 8], // TODO: Populate with git hash
            middleware_custom_version: [0u8; 8],
            os_custom_version: [0u8; 8],
            uid2: [0u8; 18],
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
            mavtype: V::mav_type(),
            autopilot: V::autopilot_type(),
            base_mode,
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::vehicle::GroundRover;
    use mavlink::common::MavCmd;

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
        state.battery.voltage = 12.0; // Good battery
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1); // GCS sender IDs

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(ack.target_system, 255); // Reply to GCS
        assert_eq!(ack.target_component, 1);
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
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

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
        state.armed = crate::communication::mavlink::state::ArmedState::Armed;
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

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
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_set_mode_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // ArduPilot Rover mode numbers: Auto=10
        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 10.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        assert_eq!(handler.state().mode, FlightMode::Auto);
    }

    #[test]
    fn test_set_mode_denied_invalid() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_DO_SET_MODE, 0.0, 99.0); // Invalid mode
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_preflight_calibration_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 1.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_unsupported_command() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_NAV_WAYPOINT, 0.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn test_command_ack_fields() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_command_long(MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION, 0.0, 0.0);

        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_request_protocol_version() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // MAV_CMD_REQUEST_MESSAGE with param1=300 (PROTOCOL_VERSION message ID)
        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 300.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_create_protocol_version_message() {
        let msg = TestHandler::create_protocol_version_message();

        assert_eq!(msg.version, 200);
        assert_eq!(msg.min_version, 200);
        assert_eq!(msg.max_version, 200);
        // Hash arrays should be initialized (currently zeros)
        assert_eq!(msg.spec_version_hash.len(), 8);
        assert_eq!(msg.library_version_hash.len(), 8);
    }

    #[test]
    fn test_request_autopilot_version() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // MAV_CMD_REQUEST_MESSAGE with param1=148 (AUTOPILOT_VERSION message ID)
        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 148.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_request_camera_info_unsupported() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // MAV_CMD_REQUEST_MESSAGE with param1=259 (CAMERA_INFORMATION message ID)
        let cmd = create_command_long(MavCmd::MAV_CMD_REQUEST_MESSAGE, 259.0, 0.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_REQUEST_MESSAGE);
        // Camera is not supported, but should return UNSUPPORTED gracefully
        assert_eq!(ack.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    fn create_do_reposition_cmd(lat: f32, lon: f32, alt: f32) -> COMMAND_LONG_DATA {
        COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_DO_REPOSITION,
            confirmation: 0,
            param1: -1.0,     // Ground speed (-1 for default)
            param2: 0.0,      // Reserved
            param3: 0.0,      // Reserved
            param4: f32::NAN, // Yaw (NaN for unchanged)
            param5: lat,      // Latitude
            param6: lon,      // Longitude
            param7: alt,      // Altitude
        }
    }

    #[test]
    fn test_do_reposition_accepted() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Valid coordinates (Tokyo)
        let cmd = create_do_reposition_cmd(35.6762, 139.6503, 100.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn test_do_reposition_invalid_latitude() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Invalid latitude (> 90)
        let cmd = create_do_reposition_cmd(91.0, 139.6503, 100.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_do_reposition_invalid_longitude() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Invalid longitude (> 180)
        let cmd = create_do_reposition_cmd(35.6762, 181.0, 100.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn test_do_reposition_negative_coords() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Valid negative coordinates (New York area)
        let cmd = create_do_reposition_cmd(40.7128, -74.0060, 50.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_REPOSITION);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    // Helper function for creating COMMAND_INT messages
    fn create_command_int_set_home(
        use_current: bool,
        lat_e7: i32,
        lon_e7: i32,
        alt: f32,
    ) -> COMMAND_INT_DATA {
        COMMAND_INT_DATA {
            target_system: 1,
            target_component: 1,
            frame: mavlink::common::MavFrame::MAV_FRAME_GLOBAL,
            command: MavCmd::MAV_CMD_DO_SET_HOME,
            current: 0,
            autocontinue: 0,
            param1: if use_current { 1.0 } else { 0.0 },
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: lat_e7,
            y: lon_e7,
            z: alt,
        }
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_specified_location() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Set home to Tokyo (35.6762, 139.6503) - scaled by 10^7
        let lat_e7 = (35.6762 * 1e7) as i32;
        let lon_e7 = (139.6503 * 1e7) as i32;
        let cmd = create_command_int_set_home(false, lat_e7, lon_e7, 100.0);
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.target_system, 255); // Reply to GCS
        assert_eq!(ack.target_component, 1);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        // HOME_POSITION message should be included in response
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::HOME_POSITION(_)));

        // Verify home was set
        let home = critical_section::with(|cs| {
            crate::communication::mavlink::state::SYSTEM_STATE
                .borrow_ref(cs)
                .home_position
        });
        assert!(home.is_some());
        let home = home.unwrap();
        assert!((home.latitude - 35.6762).abs() < 0.0001);
        assert!((home.longitude - 139.6503).abs() < 0.0001);
        assert!((home.altitude - 100.0).abs() < 0.01);
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_current_location_no_gps() {
        let state = SystemState::new(); // No GPS fix
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Try to set home to current location (no GPS)
        let cmd = create_command_int_set_home(true, 0, 0, 0.0);
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        // No HOME_POSITION message when denied
        assert!(messages.is_empty());
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_current_location_with_gps() {
        use crate::devices::gps::{GpsFixType, GpsPosition};

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
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Set home to current location
        let cmd = create_command_int_set_home(true, 0, 0, 0.0);
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        // HOME_POSITION message should be included in response
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::HOME_POSITION(_)));

        // Verify home was set to GPS position
        let home = critical_section::with(|cs| {
            crate::communication::mavlink::state::SYSTEM_STATE
                .borrow_ref(cs)
                .home_position
        });
        assert!(home.is_some());
        let home = home.unwrap();
        assert!((home.latitude - 35.6762).abs() < 0.0001);
        assert!((home.longitude - 139.6503).abs() < 0.0001);
        assert!((home.altitude - 50.0).abs() < 0.01);
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_invalid_coordinates() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Invalid latitude (> 90 degrees)
        let lat_e7 = (91.0 * 1e7) as i32;
        let lon_e7 = (139.0 * 1e7) as i32;
        let cmd = create_command_int_set_home(false, lat_e7, lon_e7, 100.0);
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
        // No HOME_POSITION message when denied
        assert!(messages.is_empty());
    }

    #[test]
    #[serial_test::serial]
    fn test_set_home_negative_coordinates() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Negative coordinates (Buenos Aires: -34.6037, -58.3816)
        let lat_e7 = (-34.6037 * 1e7) as i32;
        let lon_e7 = (-58.3816 * 1e7) as i32;
        let cmd = create_command_int_set_home(false, lat_e7, lon_e7, 25.0);
        let (ack, messages) = handler.handle_command_int(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_DO_SET_HOME);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
        // HOME_POSITION message should be included in response
        assert_eq!(messages.len(), 1);
        assert!(matches!(messages[0], MavMessage::HOME_POSITION(_)));

        // Verify home was set
        let home = critical_section::with(|cs| {
            crate::communication::mavlink::state::SYSTEM_STATE
                .borrow_ref(cs)
                .home_position
        });
        assert!(home.is_some());
        let home = home.unwrap();
        assert!((home.latitude - (-34.6037)).abs() < 0.0001);
        assert!((home.longitude - (-58.3816)).abs() < 0.0001);
    }

    #[test]
    fn test_command_int_unsupported() {
        let state = SystemState::new();
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        // Unsupported COMMAND_INT command
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
        // No extra messages for unsupported commands
        assert!(messages.is_empty());
    }

    #[test]
    fn test_autopilot_version_has_compass_calibration_capability() {
        let msg = TestHandler::create_autopilot_version_message();

        // Check that COMPASS_CALIBRATION capability is set (value 4096 = 0x1000)
        assert!(
            msg.capabilities
                .contains(MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION),
            "AUTOPILOT_VERSION should include COMPASS_CALIBRATION capability"
        );
    }

    // Helper to create MAV_CMD_FIXED_MAG_CAL_YAW command
    fn create_fixed_mag_cal_yaw_cmd(yaw_degrees: f32) -> COMMAND_LONG_DATA {
        COMMAND_LONG_DATA {
            target_system: 1,
            target_component: 1,
            command: MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW,
            confirmation: 0,
            param1: yaw_degrees, // Yaw angle in degrees
            param2: 0.0,         // CompassMask
            param3: 0.0,         // Latitude (optional)
            param4: 0.0,         // Longitude (optional)
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        }
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_accepted_with_3d_gps() {
        use crate::devices::gps::{GpsFixType, GpsPosition};

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
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_fixed_mag_cal_yaw_cmd(90.0); // 90 degrees
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW);
        assert_eq!(ack.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_denied_without_gps() {
        let state = SystemState::new(); // No GPS fix
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_fixed_mag_cal_yaw_cmd(180.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    #[serial_test::serial]
    fn test_fixed_mag_cal_yaw_denied_with_2d_gps() {
        use crate::devices::gps::{GpsFixType, GpsPosition};

        let mut state = SystemState::new();
        state.gps_position = Some(GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 0.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix2D, // Only 2D fix
            satellites: 4,
        });
        critical_section::with(|cs| {
            *crate::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs) = state;
        });
        let mut handler = TestHandler::new();

        let cmd = create_fixed_mag_cal_yaw_cmd(270.0);
        let (ack, _) = handler.handle_command_long(&cmd, 255, 1);

        assert_eq!(ack.command, MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW);
        assert_eq!(ack.result, MavResult::MAV_RESULT_DENIED);
    }
}
