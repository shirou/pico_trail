//! AHRS Trait and Types
//!
//! Provides the unified `Ahrs` trait and related types for attitude estimation.
//! Platform-specific implementations (SharedAhrsState, sensor drivers) are in the firmware crate.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// AHRS type for runtime identification and diagnostics
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsType {
    /// Software EKF/DCM processing raw IMU data
    Software,
    /// External AHRS with on-chip fusion (BNO086, VectorNav, etc.)
    External,
}

/// AHRS error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsError {
    /// Sensor communication failed
    SensorError,
    /// Sensor not initialized
    NotInitialized,
    /// Invalid data received from sensor
    InvalidData,
    /// Timeout waiting for data
    Timeout,
    /// AHRS not converged (software EKF only)
    NotConverged,
    /// Sensor in reset/recovery state
    Resetting,
}

impl AhrsError {
    /// Return variant name as a static string (usable with defmt on embedded)
    pub fn as_str(&self) -> &'static str {
        match self {
            AhrsError::SensorError => "SensorError",
            AhrsError::NotInitialized => "NotInitialized",
            AhrsError::InvalidData => "InvalidData",
            AhrsError::Timeout => "Timeout",
            AhrsError::NotConverged => "NotConverged",
            AhrsError::Resetting => "Resetting",
        }
    }
}

/// AHRS state output - unified attitude representation with quaternion
///
/// This struct represents the complete attitude state from any AHRS source.
/// It includes both quaternion (for computation) and Euler angles (for display/logging).
///
/// # Coordinate System
///
/// - Frame: NED (North-East-Down)
/// - Quaternion: scalar-first (w, x, y, z)
/// - Euler sequence: ZYX (yaw-pitch-roll)
#[derive(Debug, Clone, Copy)]
pub struct AhrsState {
    /// Attitude quaternion (NED frame, scalar-first: w, x, y, z)
    pub quaternion: Quaternion<f32>,

    /// Roll angle in radians
    pub roll: f32,

    /// Pitch angle in radians
    pub pitch: f32,

    /// Yaw angle in radians (heading, 0 = north)
    pub yaw: f32,

    /// Angular rates in body frame (rad/s)
    pub angular_rate: Vector3<f32>,

    /// Linear acceleration in body frame (m/s²)
    pub acceleration: Vector3<f32>,

    /// Timestamp (microseconds since boot)
    pub timestamp_us: u64,

    /// Health indicator - true if data is valid and converged
    pub healthy: bool,

    /// Accuracy estimate in radians (from sensor, if available)
    pub accuracy_rad: Option<f32>,
}

impl Default for AhrsState {
    fn default() -> Self {
        Self {
            quaternion: Quaternion::identity(),
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            angular_rate: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            timestamp_us: 0,
            healthy: false,
            accuracy_rad: None,
        }
    }
}

impl AhrsState {
    /// Create a new AhrsState with given Euler angles
    pub fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        let quaternion = euler_to_quaternion_zyx(roll, pitch, yaw);
        Self {
            quaternion,
            roll,
            pitch,
            yaw,
            ..Default::default()
        }
    }

    /// Create AhrsState from a quaternion
    ///
    /// Extracts Euler angles from the quaternion using ZYX convention.
    pub fn from_quaternion(q: Quaternion<f32>, timestamp_us: u64) -> Self {
        let (roll, pitch, yaw) = quaternion_to_euler_zyx(&q);

        Self {
            quaternion: q,
            roll,
            pitch,
            yaw,
            angular_rate: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            timestamp_us,
            healthy: true,
            accuracy_rad: None,
        }
    }

    /// Check if the quaternion is valid (unit quaternion within tolerance)
    pub fn is_valid(&self) -> bool {
        let norm = self.quaternion.norm();
        (norm - 1.0).abs() < 0.01 // 1% tolerance
    }

    /// Check if attitude data is fresh (updated within threshold)
    pub fn is_fresh(&self, current_time_us: u64, threshold_us: u64) -> bool {
        current_time_us.saturating_sub(self.timestamp_us) < threshold_us
    }

    /// Get roll in degrees
    pub fn roll_deg(&self) -> f32 {
        self.roll.to_degrees()
    }

    /// Get pitch in degrees
    pub fn pitch_deg(&self) -> f32 {
        self.pitch.to_degrees()
    }

    /// Get yaw in degrees
    pub fn yaw_deg(&self) -> f32 {
        self.yaw.to_degrees()
    }

    /// Set angular rates from gyroscope reading
    pub fn with_angular_rate(mut self, rate: Vector3<f32>) -> Self {
        self.angular_rate = rate;
        self
    }

    /// Set linear acceleration from accelerometer reading
    pub fn with_acceleration(mut self, accel: Vector3<f32>) -> Self {
        self.acceleration = accel;
        self
    }

    /// Set timestamp
    pub fn with_timestamp(mut self, timestamp_us: u64) -> Self {
        self.timestamp_us = timestamp_us;
        self
    }

    /// Set healthy flag
    pub fn with_healthy(mut self, healthy: bool) -> Self {
        self.healthy = healthy;
        self
    }
}

/// Convert quaternion to Euler angles (ZYX convention)
///
/// Returns (roll, pitch, yaw) in radians.
pub fn quaternion_to_euler_zyx(q: &Quaternion<f32>) -> (f32, f32, f32) {
    let q = UnitQuaternion::from_quaternion(*q);
    q.euler_angles()
}

/// Convert Euler angles to quaternion (ZYX convention)
pub fn euler_to_quaternion_zyx(roll: f32, pitch: f32, yaw: f32) -> Quaternion<f32> {
    let uq = UnitQuaternion::from_euler_angles(roll, pitch, yaw);
    *uq.quaternion()
}

/// AHRS output - common interface for all attitude sources
///
/// This trait abstracts over different AHRS implementations:
/// - `ExternalAhrs`: Wraps sensors with on-chip fusion (BNO086)
/// - `SoftwareAhrs`: Wraps raw IMU + software EKF (future)
#[allow(async_fn_in_trait)]
pub trait Ahrs {
    /// Get current attitude estimate
    ///
    /// Returns the latest attitude state including quaternion, Euler angles,
    /// angular rates, and timestamp.
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError>;

    /// Check if AHRS is healthy and producing valid data
    fn is_healthy(&self) -> bool;

    /// Get AHRS type identifier for logging/diagnostics
    fn ahrs_type(&self) -> AhrsType;
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::PI;

    const EPSILON: f32 = 0.001;

    #[test]
    fn test_ahrs_type_equality() {
        assert_eq!(AhrsType::Software, AhrsType::Software);
        assert_eq!(AhrsType::External, AhrsType::External);
        assert_ne!(AhrsType::Software, AhrsType::External);
    }

    #[test]
    fn test_ahrs_state_default() {
        let state = AhrsState::default();

        assert_eq!(state.quaternion, Quaternion::identity());
        assert_eq!(state.roll, 0.0);
        assert_eq!(state.pitch, 0.0);
        assert_eq!(state.yaw, 0.0);
        assert!(!state.healthy);
    }

    #[test]
    fn test_ahrs_state_new() {
        let state = AhrsState::new(0.1, 0.2, 0.3);
        assert!((state.roll - 0.1).abs() < EPSILON);
        assert!((state.pitch - 0.2).abs() < EPSILON);
        assert!((state.yaw - 0.3).abs() < EPSILON);
    }

    #[test]
    fn test_ahrs_state_from_identity_quaternion() {
        let q = Quaternion::identity();
        let state = AhrsState::from_quaternion(q, 1000);

        assert!(state.roll.abs() < EPSILON);
        assert!(state.pitch.abs() < EPSILON);
        assert!(state.yaw.abs() < EPSILON);
        assert_eq!(state.timestamp_us, 1000);
        assert!(state.healthy);
    }

    #[test]
    fn test_quaternion_euler_roundtrip() {
        let test_cases = [
            (0.0, 0.0, 0.0),
            (PI / 4.0, 0.0, 0.0),
            (0.0, PI / 6.0, 0.0),
            (0.1, 0.2, 0.3),
        ];

        for (roll, pitch, yaw) in test_cases {
            let q = euler_to_quaternion_zyx(roll, pitch, yaw);
            let (r2, p2, y2) = quaternion_to_euler_zyx(&q);

            assert!(
                (roll - r2).abs() < EPSILON,
                "Roll mismatch: {} vs {}",
                roll,
                r2
            );
            assert!(
                (pitch - p2).abs() < EPSILON,
                "Pitch mismatch: {} vs {}",
                pitch,
                p2
            );
            assert!(
                (yaw - y2).abs() < EPSILON,
                "Yaw mismatch: {} vs {}",
                yaw,
                y2
            );
        }
    }

    #[test]
    fn test_ahrs_state_is_valid() {
        let valid_state = AhrsState::from_quaternion(Quaternion::identity(), 0);
        assert!(valid_state.is_valid());

        let invalid_state = AhrsState {
            quaternion: Quaternion::new(2.0, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(!invalid_state.is_valid());
    }

    #[test]
    fn test_ahrs_state_is_valid_edge_cases() {
        let zero_state = AhrsState {
            quaternion: Quaternion::new(0.0, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(!zero_state.is_valid());

        let near_unit = AhrsState {
            quaternion: Quaternion::new(0.995, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(near_unit.is_valid());

        let outside_tolerance = AhrsState {
            quaternion: Quaternion::new(0.98, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(!outside_tolerance.is_valid());

        let sqrt2_2 = core::f32::consts::FRAC_1_SQRT_2;
        let normalized = AhrsState {
            quaternion: Quaternion::new(sqrt2_2, sqrt2_2, 0.0, 0.0),
            ..Default::default()
        };
        assert!(normalized.is_valid());
    }

    #[test]
    fn test_from_quaternion_known_values() {
        let sqrt2_2 = (2.0_f32).sqrt() / 2.0;

        // 90 degree roll
        let q_roll_90 = Quaternion::new(sqrt2_2, sqrt2_2, 0.0, 0.0);
        let state = AhrsState::from_quaternion(q_roll_90, 0);
        assert!((state.roll - PI / 2.0).abs() < EPSILON);
        assert!(state.pitch.abs() < EPSILON);
        assert!(state.yaw.abs() < EPSILON);

        // 90 degree pitch
        let q_pitch_90 = Quaternion::new(sqrt2_2, 0.0, sqrt2_2, 0.0);
        let state = AhrsState::from_quaternion(q_pitch_90, 0);
        assert!(state.roll.abs() < EPSILON);
        assert!((state.pitch - PI / 2.0).abs() < EPSILON);
        assert!(state.yaw.abs() < EPSILON);

        // 90 degree yaw
        let q_yaw_90 = Quaternion::new(sqrt2_2, 0.0, 0.0, sqrt2_2);
        let state = AhrsState::from_quaternion(q_yaw_90, 0);
        assert!(state.roll.abs() < EPSILON);
        assert!(state.pitch.abs() < EPSILON);
        assert!((state.yaw - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_euler_angle_accuracy() {
        let test_angles = [
            (0.0, 0.0, 0.0),
            (0.1, 0.2, 0.3),
            (-0.5, 0.25, 0.75),
            (PI / 4.0, PI / 6.0, PI / 3.0),
        ];

        for (roll, pitch, yaw) in test_angles {
            let q = euler_to_quaternion_zyx(roll, pitch, yaw);
            let state = AhrsState::from_quaternion(q, 0);

            assert!((state.roll - roll).abs() < EPSILON);
            assert!((state.pitch - pitch).abs() < EPSILON);
            assert!((state.yaw - yaw).abs() < EPSILON);
        }
    }

    #[test]
    fn test_ahrs_state_degree_conversions() {
        let state = AhrsState {
            roll: PI / 2.0,
            pitch: PI / 4.0,
            yaw: -PI / 6.0,
            ..Default::default()
        };

        assert!((state.roll_deg() - 90.0).abs() < 0.01);
        assert!((state.pitch_deg() - 45.0).abs() < 0.01);
        assert!((state.yaw_deg() + 30.0).abs() < 0.01);
    }
}
