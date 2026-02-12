//! Motor Control Runner
//!
//! Shared motor control step logic used by both firmware and SITL.
//! Selects input source based on flight mode, applies differential drive
//! kinematics, and enforces safety checks.
//!
//! ## Usage
//!
//! Both firmware's `motor_control_task` and SITL call `MotorControlRunner::step()`
//! each tick. The caller provides RC/NAV inputs via function pointers and handles
//! writing motor speeds to hardware.

use crate::autopilot::state::FlightMode;
use crate::kinematics::DifferentialDrive;
use crate::rc::RcStatus;

/// Output from one motor control step.
pub struct MotorControlOutput {
    /// Motor speeds: [left_front, left_rear, right_rear, right_front]
    /// Range: [-1.0, +1.0] for each motor
    pub motor_speeds: [f32; 4],
    /// Whether all motors should be stopped (disarmed, RC lost, or Hold mode)
    pub should_stop: bool,
}

/// Motor control runner that encapsulates the shared motor control loop logic.
///
/// Stateless for now, but the struct allows future state (e.g., rate limiting).
pub struct MotorControlRunner;

impl MotorControlRunner {
    /// Create a new motor control runner.
    pub fn new() -> Self {
        Self
    }

    /// Execute one motor control step.
    ///
    /// Selects input source based on `mode`, applies `DifferentialDrive::mix()`,
    /// maps to 4WD layout, and checks safety conditions.
    ///
    /// # Arguments
    ///
    /// * `mode` - Current flight mode (determines input source)
    /// * `is_armed` - Whether the vehicle is armed
    /// * `rc_provider` - Returns (steering, throttle, Option<RcStatus>) from RC input
    /// * `nav_provider` - Returns (steering, throttle) from navigation controller
    ///
    /// # Returns
    ///
    /// `MotorControlOutput` with 4WD motor speeds and stop flag.
    pub fn step(
        &mut self,
        mode: FlightMode,
        is_armed: bool,
        rc_provider: fn() -> (f32, f32, Option<RcStatus>),
        nav_provider: fn() -> (f32, f32),
    ) -> MotorControlOutput {
        if !is_armed {
            return MotorControlOutput {
                motor_speeds: [0.0; 4],
                should_stop: true,
            };
        }

        // Select input source based on mode
        let (steering, throttle, rc_status) = match mode {
            FlightMode::Manual => rc_provider(),
            FlightMode::Guided | FlightMode::Auto => {
                let (s, t) = nav_provider();
                (s, t, None)
            }
            FlightMode::Hold => {
                return MotorControlOutput {
                    motor_speeds: [0.0; 4],
                    should_stop: true,
                };
            }
            _ => {
                // Other modes (Stabilize, Loiter, RTL, etc.): RC fallback
                rc_provider()
            }
        };

        // Check RC timeout (only when RC is the input source)
        if let Some(status) = rc_status {
            if status != RcStatus::Active {
                return MotorControlOutput {
                    motor_speeds: [0.0; 4],
                    should_stop: true,
                };
            }
        }

        // Apply differential drive kinematics
        let (left_speed, right_speed) = DifferentialDrive::mix(steering, throttle);

        // Map to 4WD: Left side = M1+M2, Right side = M3+M4
        MotorControlOutput {
            motor_speeds: [left_speed, left_speed, right_speed, right_speed],
            should_stop: false,
        }
    }
}

impl Default for MotorControlRunner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rc_straight() -> (f32, f32, Option<RcStatus>) {
        (0.0, 0.5, Some(RcStatus::Active))
    }

    fn rc_turn_right() -> (f32, f32, Option<RcStatus>) {
        (0.5, 0.5, Some(RcStatus::Active))
    }

    fn rc_lost() -> (f32, f32, Option<RcStatus>) {
        (0.5, 0.5, Some(RcStatus::Lost))
    }

    fn rc_never_connected() -> (f32, f32, Option<RcStatus>) {
        (0.0, 0.0, Some(RcStatus::NeverConnected))
    }

    fn nav_forward() -> (f32, f32) {
        (0.0, 0.5)
    }

    fn nav_turn_left() -> (f32, f32) {
        (-0.3, 0.5)
    }

    fn no_nav() -> (f32, f32) {
        (0.0, 0.0)
    }

    #[test]
    fn test_disarmed_stops_motors() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Manual, false, rc_straight, no_nav);
        assert!(output.should_stop);
        assert_eq!(output.motor_speeds, [0.0; 4]);
    }

    #[test]
    fn test_manual_mode_uses_rc() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Manual, true, rc_straight, no_nav);
        assert!(!output.should_stop);
        // steering=0, throttle=0.5 -> left=0.5, right=0.5
        assert_eq!(output.motor_speeds, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_manual_mode_turn_right() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Manual, true, rc_turn_right, no_nav);
        assert!(!output.should_stop);
        // steering=0.5, throttle=0.5 -> DifferentialDrive::mix(0.5, 0.5) = (1.0, 0.0)
        assert_eq!(output.motor_speeds[0], 1.0); // left front
        assert_eq!(output.motor_speeds[1], 1.0); // left rear
        assert_eq!(output.motor_speeds[2], 0.0); // right rear
        assert_eq!(output.motor_speeds[3], 0.0); // right front
    }

    #[test]
    fn test_guided_mode_uses_nav() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Guided, true, rc_straight, nav_forward);
        assert!(!output.should_stop);
        // nav_forward returns (0.0, 0.5) -> same as straight
        assert_eq!(output.motor_speeds, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_auto_mode_uses_nav() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Auto, true, rc_straight, nav_turn_left);
        assert!(!output.should_stop);
        // nav_turn_left returns (-0.3, 0.5) -> DifferentialDrive::mix(-0.3, 0.5) = (0.2, 0.8)
        let (left, right) = DifferentialDrive::mix(-0.3, 0.5);
        assert_eq!(output.motor_speeds[0], left);
        assert_eq!(output.motor_speeds[2], right);
    }

    #[test]
    fn test_hold_mode_stops_motors() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Hold, true, rc_straight, nav_forward);
        assert!(output.should_stop);
        assert_eq!(output.motor_speeds, [0.0; 4]);
    }

    #[test]
    fn test_rc_lost_stops_motors() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Manual, true, rc_lost, no_nav);
        assert!(output.should_stop);
        assert_eq!(output.motor_speeds, [0.0; 4]);
    }

    #[test]
    fn test_rc_never_connected_stops_motors() {
        let mut runner = MotorControlRunner::new();
        let output = runner.step(FlightMode::Manual, true, rc_never_connected, no_nav);
        assert!(output.should_stop);
        assert_eq!(output.motor_speeds, [0.0; 4]);
    }

    #[test]
    fn test_fallback_mode_uses_rc() {
        let mut runner = MotorControlRunner::new();
        // Stabilize mode falls back to RC
        let output = runner.step(FlightMode::Stabilize, true, rc_straight, no_nav);
        assert!(!output.should_stop);
        assert_eq!(output.motor_speeds, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn test_guided_ignores_rc_status() {
        let mut runner = MotorControlRunner::new();
        // In Guided mode, RC status is irrelevant
        let output = runner.step(FlightMode::Guided, true, rc_lost, nav_forward);
        assert!(!output.should_stop);
        assert_eq!(output.motor_speeds, [0.5, 0.5, 0.5, 0.5]);
    }
}
