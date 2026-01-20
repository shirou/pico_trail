//! Differential drive kinematics for skid-steer vehicles
//!
//! This module implements the conversion from steering/throttle inputs to left/right
//! motor speeds for differential drive (skid-steer) vehicles. The implementation matches
//! ArduPilot's differential drive behavior for consistency.
//!
//! # Examples
//!
//! ```
//! use pico_trail::libraries::kinematics::DifferentialDrive;
//!
//! // Straight forward at half speed
//! let (left, right) = DifferentialDrive::mix(0.0, 0.5);
//! assert_eq!(left, 0.5);
//! assert_eq!(right, 0.5);
//!
//! // Spin right in place
//! let (left, right) = DifferentialDrive::mix(1.0, 0.0);
//! assert_eq!(left, 1.0);
//! assert_eq!(right, -1.0);
//!
//! // Forward right turn (normalized)
//! let (left, right) = DifferentialDrive::mix(0.5, 0.5);
//! assert_eq!(left, 1.0);
//! assert_eq!(right, 0.0);
//! ```

/// Differential drive kinematics converter (zero-sized type)
///
/// This struct provides a pure function for converting steering and throttle inputs
/// into left and right motor speeds with automatic normalization. It is a zero-sized
/// type used as a namespace for the conversion function.
pub struct DifferentialDrive;

impl DifferentialDrive {
    /// Convert steering/throttle to left/right motor speeds with normalization
    ///
    /// This function implements differential drive mixing with normalization to ensure
    /// that output values never exceed the valid [-1.0, +1.0] range. When normalization
    /// is required (e.g., full throttle + full steering), the proportional relationship
    /// between left and right speeds is maintained.
    ///
    /// # Algorithm
    ///
    /// 1. Calculate raw speeds: `left = throttle + steering`, `right = throttle - steering`
    /// 2. If either exceeds [-1.0, +1.0], divide both by `max(abs(left), abs(right))`
    /// 3. Return normalized (left_speed, right_speed)
    ///
    /// This matches ArduPilot's `AP_MotorsUGV::output_skid_steering()` behavior.
    ///
    /// # Arguments
    ///
    /// * `steering` - Steering input: -1.0 (full left) to +1.0 (full right)
    /// * `throttle` - Throttle input: -1.0 (full reverse) to +1.0 (full forward)
    ///
    /// # Returns
    ///
    /// A tuple `(left_speed, right_speed)` where both values are normalized to [-1.0, +1.0]
    ///
    /// # Examples
    ///
    /// ```
    /// use pico_trail::libraries::kinematics::DifferentialDrive;
    ///
    /// // Straight forward
    /// let (left, right) = DifferentialDrive::mix(0.0, 0.5);
    /// assert_eq!(left, 0.5);
    /// assert_eq!(right, 0.5);
    ///
    /// // Right turn (normalized from raw left=1.0, right=0.0)
    /// let (left, right) = DifferentialDrive::mix(0.5, 0.5);
    /// assert_eq!(left, 1.0);
    /// assert_eq!(right, 0.0);
    ///
    /// // Spin right in place
    /// let (left, right) = DifferentialDrive::mix(1.0, 0.0);
    /// assert_eq!(left, 1.0);
    /// assert_eq!(right, -1.0);
    /// ```
    #[inline]
    pub fn mix(steering: f32, throttle: f32) -> (f32, f32) {
        let mut left = throttle + steering;
        let mut right = throttle - steering;

        // Normalize if either exceeds [-1.0, +1.0] range
        let max_magnitude = left.abs().max(right.abs());
        if max_magnitude > 1.0 {
            left /= max_magnitude;
            right /= max_magnitude;
        }

        (left, right)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_straight_forward() {
        let (left, right) = DifferentialDrive::mix(0.0, 0.5);
        assert_eq!(left, 0.5);
        assert_eq!(right, 0.5);
    }

    #[test]
    fn test_straight_reverse() {
        let (left, right) = DifferentialDrive::mix(0.0, -0.5);
        assert_eq!(left, -0.5);
        assert_eq!(right, -0.5);
    }

    #[test]
    fn test_spin_right_in_place() {
        let (left, right) = DifferentialDrive::mix(1.0, 0.0);
        assert_eq!(left, 1.0);
        assert_eq!(right, -1.0);
    }

    #[test]
    fn test_spin_left_in_place() {
        let (left, right) = DifferentialDrive::mix(-1.0, 0.0);
        assert_eq!(left, -1.0);
        assert_eq!(right, 1.0);
    }

    #[test]
    fn test_forward_right_turn_normalized() {
        let (left, right) = DifferentialDrive::mix(0.5, 0.5);
        assert_eq!(left, 1.0);
        assert_eq!(right, 0.0);
    }

    #[test]
    fn test_forward_left_turn_normalized() {
        let (left, right) = DifferentialDrive::mix(-0.3, 0.8);
        // Raw values: left = 0.5, right = 1.1
        // Normalized by 1.1: left ≈ 0.4545, right ≈ 1.0
        assert!((left - 0.4545).abs() < 0.01);
        assert!((right - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_full_throttle_full_steering() {
        let (left, right) = DifferentialDrive::mix(1.0, 1.0);
        // Raw values: left = 2.0, right = 0.0
        // Normalized by 2.0: left = 1.0, right = 0.0
        assert_eq!(left, 1.0);
        assert_eq!(right, 0.0);
    }

    #[test]
    fn test_full_reverse_full_steering() {
        let (left, right) = DifferentialDrive::mix(1.0, -1.0);
        // Raw values: left = 0.0, right = -2.0
        // Normalized by 2.0: left = 0.0, right = -1.0
        assert_eq!(left, 0.0);
        assert_eq!(right, -1.0);
    }

    #[test]
    fn test_zero_inputs() {
        let (left, right) = DifferentialDrive::mix(0.0, 0.0);
        assert_eq!(left, 0.0);
        assert_eq!(right, 0.0);
    }

    // Property-based test: outputs always in [-1.0, +1.0]
    #[test]
    fn test_output_range_property() {
        let test_values = [-1.0, -0.5, -0.3, 0.0, 0.3, 0.5, 0.8, 1.0];
        let valid_range = -1.0..=1.0;
        for &steering in &test_values {
            for &throttle in &test_values {
                let (left, right) = DifferentialDrive::mix(steering, throttle);
                assert!(
                    valid_range.contains(&left),
                    "left={} out of range for steering={}, throttle={}",
                    left,
                    steering,
                    throttle
                );
                assert!(
                    valid_range.contains(&right),
                    "right={} out of range for steering={}, throttle={}",
                    right,
                    steering,
                    throttle
                );
            }
        }
    }

    // Symmetry property: steering=-x produces opposite results to steering=+x
    #[test]
    fn test_steering_symmetry() {
        let throttle = 0.5;
        let (left_pos, right_pos) = DifferentialDrive::mix(0.5, throttle);
        let (left_neg, right_neg) = DifferentialDrive::mix(-0.5, throttle);

        // Left with +steering should equal right with -steering
        assert!((left_pos - right_neg).abs() < 0.01);
        // Right with +steering should equal left with -steering
        assert!((right_pos - left_neg).abs() < 0.01);
    }

    // Zero throttle property: produces opposite left/right speeds
    #[test]
    fn test_zero_throttle_opposite_speeds() {
        let (left, right) = DifferentialDrive::mix(0.7, 0.0);
        assert_eq!(left, -right);
    }
}
