//! Auto Mode
//!
//! Mission-based autonomous navigation mode that executes waypoint missions.
//!
//! # Behavior
//!
//! - On entry: Validate GPS fix and mission loaded, start sequencer
//! - Each update: Delegate to MissionSequencer for dual-slot NAV/DO execution
//! - On mission complete: Stop motors via on_mission_complete callback
//! - On GPS loss: Return error to trigger Hold mode transition
//!
//! # Architecture
//!
//! Auto mode implements `MissionExecutor` from the core crate, allowing the
//! platform-agnostic `MissionSequencer` to drive physical command execution:
//! - `start_command`: Sets navigation target for NAV commands
//! - `verify_command`: Runs nav controller and returns at_target
//! - `on_mission_complete`: Stops motors
//!
//! # References
//!
//! - FR-jm7mj-auto-mode-mission-execution: Auto mode requirements
//! - ADR-0yapl-mission-execution-telemetry-architecture: Sequencer architecture
//! - ADR-h3k9f-heading-source-integration: Heading source integration

use super::Mode;
use crate::core::mission::{
    has_waypoints, push_current_changed, push_item_reached, set_mission_state, CommandStartResult,
    MissionEvent, MissionExecutor, MissionState, Waypoint, MISSION_SEQUENCER, MISSION_STORAGE,
};
use crate::core::traits::SharedState;
use crate::devices::gps::GpsFixType;
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::{
    NavigationController, PositionTarget, SimpleNavigationController,
};
use pico_trail_core::mission::{is_nav_command, MAV_CMD_DO_CHANGE_SPEED};

/// Default waypoint speed (m/s) representing full-throttle speed.
/// Used as the denominator for DO_CHANGE_SPEED throttle scaling.
const DEFAULT_WP_SPEED: f32 = 2.0;

/// Auto Mode state
///
/// Stores current GPS/heading for use by verify_command during sequencer update.
#[derive(Clone, Copy, Debug)]
struct AutoState {
    /// Navigation target set by start_command
    current_target: Option<PositionTarget>,
    /// Current GPS latitude (degrees)
    current_gps_lat: f32,
    /// Current GPS longitude (degrees)
    current_gps_lon: f32,
    /// Current heading (degrees, 0-360)
    current_heading: f32,
    /// Current update dt (seconds)
    current_dt: f32,
    /// Elapsed time since mission start (milliseconds)
    elapsed_ms: u64,
    /// Cached mission speed for throttle scaling (avoids re-entrant lock)
    cached_mission_speed: Option<f32>,
}

impl Default for AutoState {
    fn default() -> Self {
        Self {
            current_target: None,
            current_gps_lat: 0.0,
            current_gps_lon: 0.0,
            current_heading: 0.0,
            current_dt: 0.02,
            elapsed_ms: 0,
            cached_mission_speed: None,
        }
    }
}

/// Auto Mode
///
/// Provides mission-based autonomous navigation through uploaded waypoints.
/// Delegates sequencing to MissionSequencer and implements MissionExecutor
/// for physical command execution.
pub struct AutoMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// Auto state
    state: Option<AutoState>,
    /// GPS position provider function
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (returns heading in degrees, 0-360)
    heading_provider: fn() -> Option<f32>,
}

impl<'a> AutoMode<'a> {
    /// Create new Auto mode
    pub fn new(
        actuators: &'a mut dyn ActuatorInterface,
        gps_provider: fn() -> Option<GpsPosition>,
        heading_provider: fn() -> Option<f32>,
    ) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
            gps_provider,
            heading_provider,
        }
    }

    /// Check if Auto mode can be entered
    pub fn can_enter(gps_provider: fn() -> Option<GpsPosition>) -> Result<(), &'static str> {
        let gps = gps_provider().ok_or("Auto requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("Auto requires 3D GPS fix");
        }
        if !has_waypoints() {
            return Err("Auto requires mission loaded");
        }
        Ok(())
    }

    /// Check if navigation is active
    pub fn is_navigation_active(&self) -> bool {
        self.state
            .as_ref()
            .map(|s| s.current_target.is_some())
            .unwrap_or(false)
    }

    /// Check if mission is complete
    pub fn is_mission_complete(&self) -> bool {
        MISSION_SEQUENCER.with(|seq| seq.state() == MissionState::Completed)
    }

    /// Get current waypoint index
    pub fn current_waypoint_index(&self) -> u16 {
        MISSION_SEQUENCER.with(|seq| seq.current_nav_index())
    }
}

// ============================================================================
// MissionExecutor Implementation
// ============================================================================

impl<'a> MissionExecutor for AutoMode<'a> {
    fn start_command(&mut self, cmd: &Waypoint) -> CommandStartResult {
        if is_nav_command(cmd.command) {
            // NAV command: set navigation target
            let target = PositionTarget::from(cmd);
            if let Some(state) = &mut self.state {
                state.current_target = Some(target);
            }
            self.nav_controller.reset();
            CommandStartResult::Accepted
        } else {
            // DO commands: handled by sequencer (e.g., DO_CHANGE_SPEED)
            if cmd.command != MAV_CMD_DO_CHANGE_SPEED {
                crate::log_warn!("Auto: unsupported DO command {}", cmd.command);
            }
            CommandStartResult::Complete
        }
    }

    fn verify_command(&mut self, _cmd: &Waypoint) -> bool {
        let state = match &self.state {
            Some(s) => *s,
            None => return false,
        };

        let target = match &state.current_target {
            Some(t) => t,
            None => return false,
        };

        let output = self.nav_controller.update(
            state.current_gps_lat,
            state.current_gps_lon,
            target,
            state.current_heading,
            state.current_dt,
            None,
        );

        // Apply mission speed scaling if DO_CHANGE_SPEED is active (FR-0q1tf)
        let throttle = if let Some(speed) = state.cached_mission_speed {
            let scale = (speed / DEFAULT_WP_SPEED).clamp(0.0, 1.0);
            output.throttle * scale
        } else {
            output.throttle
        };

        // Apply navigation output to actuators
        let _ = self.actuators.set_steering(output.steering);
        let _ = self.actuators.set_throttle(throttle);

        output.at_target
    }

    fn on_mission_complete(&mut self) {
        let _ = self.actuators.set_steering(0.0);
        let _ = self.actuators.set_throttle(0.0);
        set_mission_state(MissionState::Completed);
        crate::log_info!("Auto: mission complete");
    }
}

// ============================================================================
// Mode Trait Implementation
// ============================================================================

impl<'a> Mode for AutoMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Validate GPS fix
        let gps = (self.gps_provider)().ok_or("No GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("Auto requires 3D GPS fix");
        }

        // Validate mission loaded
        if !has_waypoints() {
            return Err("Auto requires mission loaded");
        }

        // Initialize state with current GPS position
        self.state = Some(AutoState {
            current_gps_lat: gps.latitude,
            current_gps_lon: gps.longitude,
            cached_mission_speed: None,
            ..AutoState::default()
        });

        // Reset navigation controller
        self.nav_controller.reset();

        // Start mission via sequencer
        let sequencer_running = MISSION_SEQUENCER.with_mut(|sequencer| {
            MISSION_STORAGE.with(|storage| {
                let _events = sequencer.start(storage, self);
            });
            matches!(sequencer.state(), MissionState::Running)
        });

        if !sequencer_running {
            self.state = None;
            return Err("Auto mission could not be started");
        }

        // Sync global state
        set_mission_state(MissionState::Running);

        crate::log_info!("Auto mode entered");
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        if self.state.is_none() {
            return Err("Auto not initialized");
        }

        // Check sequencer state
        let seq_state = MISSION_SEQUENCER.with(|seq| seq.state());
        if seq_state == MissionState::Completed {
            let _ = self.actuators.set_steering(0.0);
            let _ = self.actuators.set_throttle(0.0);
            return Ok(());
        }
        if seq_state != MissionState::Running {
            let _ = self.actuators.set_steering(0.0);
            let _ = self.actuators.set_throttle(0.0);
            return Ok(());
        }

        // Get current GPS position
        let gps = (self.gps_provider)().ok_or("GPS lost during Auto")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("GPS fix lost during Auto");
        }

        // Get heading
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available for Auto")?;

        // Store current position/heading for verify_command
        if let Some(state) = &mut self.state {
            state.current_gps_lat = gps.latitude;
            state.current_gps_lon = gps.longitude;
            state.current_heading = heading;
            state.current_dt = dt;
            state.elapsed_ms += (dt * 1000.0) as u64;
        }

        let now_ms = self.state.as_ref().map(|s| s.elapsed_ms).unwrap_or(0);

        // Check if SET_CURRENT requires navigation target refresh (FR-aulp3)
        let nav_refresh = MISSION_SEQUENCER.with(|seq| {
            if seq.needs_nav_refresh() {
                Some(seq.current_nav_index())
            } else {
                None
            }
        });
        if let Some(nav_index) = nav_refresh {
            let wp = MISSION_STORAGE.with(|storage| storage.get_waypoint(nav_index).copied());
            if let Some(wp) = wp {
                self.start_command(&wp);
            }
            MISSION_SEQUENCER.with_mut(|seq| seq.clear_nav_refresh());
        }

        // Cache mission speed for verify_command (avoid re-entrant lock access)
        if let Some(state) = &mut self.state {
            state.cached_mission_speed = MISSION_SEQUENCER.with(|seq| seq.mission_speed());
        }

        // Delegate to sequencer
        let events = MISSION_SEQUENCER.with_mut(|sequencer| {
            MISSION_STORAGE.with(|storage| sequencer.update(storage, self, now_ms))
        });

        // Hold time passivity: zero actuators when holding at waypoint (FR-4dq92)
        let is_holding = MISSION_SEQUENCER.with(|seq| seq.is_holding());
        if is_holding {
            let _ = self.actuators.set_steering(0.0);
            let _ = self.actuators.set_throttle(0.0);
        }

        // Process events for logging and telemetry
        for event in &events {
            match event {
                MissionEvent::CurrentChanged(seq) => {
                    crate::log_info!("Auto: navigating to waypoint {}", seq);
                    push_current_changed(*seq);
                }
                MissionEvent::ItemReached(seq) => {
                    crate::log_info!("Auto: waypoint {} reached", seq);
                    push_item_reached(*seq);
                }
                MissionEvent::MissionComplete => {
                    // on_mission_complete already called by sequencer
                }
                MissionEvent::MissionCleared => {
                    crate::log_info!("Auto: mission cleared");
                }
            }
        }

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Auto mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Stop sequencer
        MISSION_SEQUENCER.with_mut(|sequencer| {
            sequencer.stop();
        });

        // Clear state
        self.state = None;
        self.nav_controller.reset();

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Auto"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};
    use crate::libraries::{ActuatorConfig, Actuators};
    use crate::platform::traits::pwm::PwmInterface;
    use crate::platform::Result;

    // Mock PWM for testing
    struct MockPwm {
        duty_cycle: f32,
    }

    impl MockPwm {
        fn new() -> Self {
            Self { duty_cycle: 0.0 }
        }
    }

    impl PwmInterface for MockPwm {
        fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()> {
            if !(0.0..=1.0).contains(&duty_cycle) {
                return Err(crate::platform::PlatformError::Pwm(
                    crate::platform::error::PwmError::InvalidDutyCycle,
                ));
            }
            self.duty_cycle = duty_cycle;
            Ok(())
        }

        fn duty_cycle(&self) -> f32 {
            self.duty_cycle
        }

        fn set_frequency(&mut self, _frequency: u32) -> Result<()> {
            Ok(())
        }

        fn frequency(&self) -> u32 {
            50
        }

        fn enable(&mut self) {}
        fn disable(&mut self) {}

        fn is_enabled(&self) -> bool {
            true
        }
    }

    // ========== AutoMode Tests ==========

    #[test]
    fn test_auto_state_default() {
        let state = AutoState::default();
        assert!(state.current_target.is_none());
        assert_eq!(state.current_gps_lat, 0.0);
        assert_eq!(state.elapsed_ms, 0);
    }

    #[test]
    fn test_auto_mode_creation() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );

        fn mock_gps() -> Option<GpsPosition> {
            None
        }
        fn mock_heading() -> Option<f32> {
            None
        }

        let auto_mode = AutoMode::new(&mut actuators, mock_gps, mock_heading);
        assert_eq!(auto_mode.name(), "Auto");
    }
}
