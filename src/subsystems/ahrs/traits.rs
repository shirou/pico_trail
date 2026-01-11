//! AHRS Trait and Types
//!
//! Provides the unified `Ahrs` trait for all attitude estimation sources.
//! Supports both External AHRS (BNO086) and Software AHRS (EKF).
//!
//! ## Architecture (ADR-nzvfy)
//!
//! ```text
//!                        ┌──────────────────┐
//!                        │  Flight Control  │
//!                        └────────▲─────────┘
//!                                 │
//!                        ┌────────┴─────────┐
//!                        │    Ahrs Trait    │
//!                        └────────▲─────────┘
//!                                 │
//!          ┌──────────────────────┼──────────────────────┐
//!          │                      │                      │
//! ┌────────┴────────┐    ┌────────┴────────┐    ┌────────┴────────┐
//! │  SoftwareAhrs   │    │  ExternalAhrs   │    │  ExternalAhrs   │
//! │  (EKF)          │    │  (BNO086)       │    │  (Future)       │
//! └─────────────────┘    └─────────────────┘    └─────────────────┘
//! ```

use crate::devices::traits::{QuaternionError, QuaternionReading};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// AHRS type for runtime identification and diagnostics
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum AhrsType {
    /// Software EKF/DCM processing raw IMU data
    Software,
    /// External AHRS with on-chip fusion (BNO086, VectorNav, etc.)
    External,
}

/// AHRS error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
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

impl From<QuaternionError> for AhrsError {
    fn from(err: QuaternionError) -> Self {
        match err {
            QuaternionError::I2cError => AhrsError::SensorError,
            QuaternionError::ProtocolError => AhrsError::SensorError,
            QuaternionError::NotInitialized => AhrsError::NotInitialized,
            QuaternionError::Resetting => AhrsError::Resetting,
            QuaternionError::InvalidData => AhrsError::InvalidData,
            QuaternionError::Timeout => AhrsError::Timeout,
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

    /// Create AhrsState from QuaternionReading (BNO086 output)
    pub fn from_quaternion_reading(reading: &QuaternionReading) -> Self {
        let (roll, pitch, yaw) = quaternion_to_euler_zyx(&reading.quaternion);

        Self {
            quaternion: reading.quaternion,
            roll,
            pitch,
            yaw,
            angular_rate: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            timestamp_us: reading.timestamp_us,
            healthy: reading.is_valid(),
            accuracy_rad: Some(reading.accuracy_rad),
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

/// Extension trait for creating AhrsState from QuaternionReading
pub trait QuaternionReadingExt {
    /// Convert QuaternionReading to AhrsState
    fn to_ahrs_state(&self) -> AhrsState;
}

impl QuaternionReadingExt for QuaternionReading {
    fn to_ahrs_state(&self) -> AhrsState {
        AhrsState::from_quaternion_reading(self)
    }
}

/// Thread-safe wrapper for AhrsState
///
/// Uses critical sections for atomic read/write operations.
/// Provides shared access to attitude state across tasks.
pub struct SharedAhrsState {
    state: core::cell::UnsafeCell<AhrsState>,
}

// Safety: AhrsState is a simple data struct with no internal mutability.
// We ensure thread safety through critical sections in the accessor methods.
unsafe impl Sync for SharedAhrsState {}
unsafe impl Send for SharedAhrsState {}

impl SharedAhrsState {
    /// Create new shared AHRS state
    pub const fn new() -> Self {
        Self {
            state: core::cell::UnsafeCell::new(AhrsState {
                quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                angular_rate: Vector3::new(0.0, 0.0, 0.0),
                acceleration: Vector3::new(0.0, 0.0, 0.0),
                timestamp_us: 0,
                healthy: false,
                accuracy_rad: None,
            }),
        }
    }

    /// Read current AHRS state (thread-safe)
    pub fn read(&self) -> AhrsState {
        critical_section::with(|_cs| unsafe { *self.state.get() })
    }

    /// Write new AHRS state (thread-safe)
    pub fn write(&self, state: AhrsState) {
        critical_section::with(|_cs| unsafe {
            *self.state.get() = state;
        });
    }

    /// Update attitude from Euler angles (convenience method)
    pub fn update_euler(&self, roll: f32, pitch: f32, yaw: f32, timestamp_us: u64) {
        critical_section::with(|_cs| unsafe {
            let state = &mut *self.state.get();
            state.quaternion = euler_to_quaternion_zyx(roll, pitch, yaw);
            state.roll = roll;
            state.pitch = pitch;
            state.yaw = yaw;
            state.timestamp_us = timestamp_us;
        });
    }

    /// Update attitude from quaternion (convenience method)
    pub fn update_quaternion(&self, q: Quaternion<f32>, timestamp_us: u64) {
        critical_section::with(|_cs| unsafe {
            let state = &mut *self.state.get();
            let (roll, pitch, yaw) = quaternion_to_euler_zyx(&q);
            state.quaternion = q;
            state.roll = roll;
            state.pitch = pitch;
            state.yaw = yaw;
            state.timestamp_us = timestamp_us;
        });
    }

    /// Get roll angle (thread-safe)
    pub fn get_roll(&self) -> f32 {
        critical_section::with(|_cs| unsafe { (*self.state.get()).roll })
    }

    /// Get pitch angle (thread-safe)
    pub fn get_pitch(&self) -> f32 {
        critical_section::with(|_cs| unsafe { (*self.state.get()).pitch })
    }

    /// Get yaw angle (thread-safe)
    pub fn get_yaw(&self) -> f32 {
        critical_section::with(|_cs| unsafe { (*self.state.get()).yaw })
    }

    /// Get quaternion (thread-safe)
    pub fn get_quaternion(&self) -> Quaternion<f32> {
        critical_section::with(|_cs| unsafe { (*self.state.get()).quaternion })
    }

    /// Get angular rates (thread-safe)
    pub fn get_angular_rate(&self) -> Vector3<f32> {
        critical_section::with(|_cs| unsafe { (*self.state.get()).angular_rate })
    }

    /// Check if healthy (thread-safe)
    pub fn is_healthy(&self) -> bool {
        critical_section::with(|_cs| unsafe { (*self.state.get()).healthy })
    }

    /// Set healthy flag (thread-safe)
    pub fn set_healthy(&self, healthy: bool) {
        critical_section::with(|_cs| unsafe {
            (*self.state.get()).healthy = healthy;
        });
    }
}

impl Default for SharedAhrsState {
    fn default() -> Self {
        Self::new()
    }
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
    fn test_ahrs_error_from_quaternion_error_all_variants() {
        // Test all QuaternionError variants map correctly
        assert_eq!(
            AhrsError::from(QuaternionError::I2cError),
            AhrsError::SensorError
        );
        assert_eq!(
            AhrsError::from(QuaternionError::ProtocolError),
            AhrsError::SensorError
        );
        assert_eq!(
            AhrsError::from(QuaternionError::NotInitialized),
            AhrsError::NotInitialized
        );
        assert_eq!(
            AhrsError::from(QuaternionError::Resetting),
            AhrsError::Resetting
        );
        assert_eq!(
            AhrsError::from(QuaternionError::InvalidData),
            AhrsError::InvalidData
        );
        assert_eq!(
            AhrsError::from(QuaternionError::Timeout),
            AhrsError::Timeout
        );
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
        // Zero quaternion - definitely invalid
        let zero_state = AhrsState {
            quaternion: Quaternion::new(0.0, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(!zero_state.is_valid());

        // Near-unit quaternion (within 1% tolerance) - should be valid
        let near_unit = AhrsState {
            quaternion: Quaternion::new(0.995, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(near_unit.is_valid());

        // Just outside 1% tolerance - should be invalid
        let outside_tolerance = AhrsState {
            quaternion: Quaternion::new(0.98, 0.0, 0.0, 0.0),
            ..Default::default()
        };
        assert!(!outside_tolerance.is_valid());

        // Normalized non-identity quaternion - should be valid
        let sqrt2_2 = core::f32::consts::FRAC_1_SQRT_2;
        let normalized = AhrsState {
            quaternion: Quaternion::new(sqrt2_2, sqrt2_2, 0.0, 0.0), // 90 deg roll
            ..Default::default()
        };
        assert!(normalized.is_valid());
    }

    #[test]
    fn test_from_quaternion_known_values() {
        // 90 degree roll: quaternion (cos(45), sin(45), 0, 0)
        let sqrt2_2 = (2.0_f32).sqrt() / 2.0;
        let q_roll_90 = Quaternion::new(sqrt2_2, sqrt2_2, 0.0, 0.0);
        let state = AhrsState::from_quaternion(q_roll_90, 0);
        assert!(
            (state.roll - PI / 2.0).abs() < EPSILON,
            "Expected roll=PI/2, got {}",
            state.roll
        );
        assert!(
            state.pitch.abs() < EPSILON,
            "Expected pitch=0, got {}",
            state.pitch
        );
        assert!(
            state.yaw.abs() < EPSILON,
            "Expected yaw=0, got {}",
            state.yaw
        );

        // 90 degree pitch: quaternion (cos(45), 0, sin(45), 0)
        let q_pitch_90 = Quaternion::new(sqrt2_2, 0.0, sqrt2_2, 0.0);
        let state = AhrsState::from_quaternion(q_pitch_90, 0);
        assert!(
            state.roll.abs() < EPSILON,
            "Expected roll=0, got {}",
            state.roll
        );
        assert!(
            (state.pitch - PI / 2.0).abs() < EPSILON,
            "Expected pitch=PI/2, got {}",
            state.pitch
        );
        assert!(
            state.yaw.abs() < EPSILON,
            "Expected yaw=0, got {}",
            state.yaw
        );

        // 90 degree yaw: quaternion (cos(45), 0, 0, sin(45))
        let q_yaw_90 = Quaternion::new(sqrt2_2, 0.0, 0.0, sqrt2_2);
        let state = AhrsState::from_quaternion(q_yaw_90, 0);
        assert!(
            state.roll.abs() < EPSILON,
            "Expected roll=0, got {}",
            state.roll
        );
        assert!(
            state.pitch.abs() < EPSILON,
            "Expected pitch=0, got {}",
            state.pitch
        );
        assert!(
            (state.yaw - PI / 2.0).abs() < EPSILON,
            "Expected yaw=PI/2, got {}",
            state.yaw
        );
    }

    #[test]
    fn test_euler_angle_accuracy() {
        // Test that Euler angle extraction is within ±0.001 rad accuracy
        let test_angles = [
            (0.0, 0.0, 0.0),
            (0.1, 0.2, 0.3),
            (-0.5, 0.25, 0.75),
            (PI / 4.0, PI / 6.0, PI / 3.0),
        ];

        for (roll, pitch, yaw) in test_angles {
            let q = euler_to_quaternion_zyx(roll, pitch, yaw);
            let state = AhrsState::from_quaternion(q, 0);

            assert!(
                (state.roll - roll).abs() < EPSILON,
                "Roll accuracy failed: expected {}, got {} (diff: {})",
                roll,
                state.roll,
                (state.roll - roll).abs()
            );
            assert!(
                (state.pitch - pitch).abs() < EPSILON,
                "Pitch accuracy failed: expected {}, got {} (diff: {})",
                pitch,
                state.pitch,
                (state.pitch - pitch).abs()
            );
            assert!(
                (state.yaw - yaw).abs() < EPSILON,
                "Yaw accuracy failed: expected {}, got {} (diff: {})",
                yaw,
                state.yaw,
                (state.yaw - yaw).abs()
            );
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

    #[test]
    fn test_from_quaternion_reading() {
        let reading = QuaternionReading {
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            accuracy_rad: 0.05,
            timestamp_us: 5000,
        };

        let state = AhrsState::from_quaternion_reading(&reading);

        assert_eq!(state.quaternion, reading.quaternion);
        assert_eq!(state.timestamp_us, 5000);
        assert_eq!(state.accuracy_rad, Some(0.05));
        assert!(state.healthy);
    }

    #[test]
    fn test_shared_ahrs_state() {
        let shared = SharedAhrsState::new();

        // Initial state should be default
        let state = shared.read();
        assert_eq!(state.roll, 0.0);
        assert!(!state.healthy);

        // Write new state
        let new_state = AhrsState::new(0.5, 0.6, 0.7).with_healthy(true);
        shared.write(new_state);

        // Read back
        let read_state = shared.read();
        assert!((read_state.roll - 0.5).abs() < EPSILON);
        assert!((read_state.pitch - 0.6).abs() < EPSILON);
        assert!((read_state.yaw - 0.7).abs() < EPSILON);
        assert!(read_state.healthy);
    }

    #[test]
    fn test_shared_ahrs_state_getters() {
        let shared = SharedAhrsState::new();

        shared.update_euler(1.0, 2.0, 3.0, 100_000);

        assert!((shared.get_roll() - 1.0).abs() < EPSILON);
        assert!((shared.get_pitch() - 2.0).abs() < EPSILON);
        assert!((shared.get_yaw() - 3.0).abs() < EPSILON);

        let state = shared.read();
        assert_eq!(state.timestamp_us, 100_000);
    }

    #[test]
    fn test_shared_ahrs_state_healthy_flag() {
        let shared = SharedAhrsState::new();

        assert!(!shared.is_healthy());

        shared.set_healthy(true);
        assert!(shared.is_healthy());

        shared.set_healthy(false);
        assert!(!shared.is_healthy());
    }
}
