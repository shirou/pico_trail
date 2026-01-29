//! AHRS Firmware Extensions
//!
//! Firmware-specific AHRS types that depend on platform features.
//! Core AHRS types (AhrsState, AhrsType, AhrsError, Ahrs trait) are in pico_trail_core.

use crate::devices::traits::{QuaternionError, QuaternionReading};
use nalgebra::{Quaternion, Vector3};

// Re-export core AHRS types for firmware consumers
pub use pico_trail_core::ahrs::traits::{
    euler_to_quaternion_zyx, quaternion_to_euler_zyx, Ahrs, AhrsError, AhrsState, AhrsType,
};

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

/// Extension trait for creating AhrsState from QuaternionReading
pub trait QuaternionReadingExt {
    /// Convert QuaternionReading to AhrsState
    fn to_ahrs_state(&self) -> AhrsState;
}

impl QuaternionReadingExt for QuaternionReading {
    fn to_ahrs_state(&self) -> AhrsState {
        let (roll, pitch, yaw) = quaternion_to_euler_zyx(&self.quaternion);

        AhrsState {
            quaternion: self.quaternion,
            roll,
            pitch,
            yaw,
            angular_rate: Vector3::zeros(),
            acceleration: Vector3::zeros(),
            timestamp_us: self.timestamp_us,
            healthy: self.is_valid(),
            accuracy_rad: Some(self.accuracy_rad),
        }
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

    const EPSILON: f32 = 0.001;

    #[test]
    fn test_ahrs_error_from_quaternion_error_all_variants() {
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
    fn test_from_quaternion_reading() {
        let reading = QuaternionReading {
            quaternion: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            accuracy_rad: 0.05,
            timestamp_us: 5000,
        };

        let state = reading.to_ahrs_state();

        assert_eq!(state.quaternion, reading.quaternion);
        assert_eq!(state.timestamp_us, 5000);
        assert_eq!(state.accuracy_rad, Some(0.05));
        assert!(state.healthy);
    }

    #[test]
    fn test_shared_ahrs_state() {
        let shared = SharedAhrsState::new();

        let state = shared.read();
        assert_eq!(state.roll, 0.0);
        assert!(!state.healthy);

        let new_state = AhrsState::new(0.5, 0.6, 0.7).with_healthy(true);
        shared.write(new_state);

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
