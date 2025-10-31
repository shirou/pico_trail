//! Shared attitude state for inter-task communication
//!
//! Provides thread-safe access to current attitude estimate from AHRS.
//! Other subsystems (navigation, control) can read this state without blocking.

use nalgebra::Vector3;

/// Quality indicators for attitude estimate
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeQuality {
    /// Has the filter converged? (true after ~5 seconds)
    pub converged: bool,

    /// Is magnetometer data valid and being used?
    pub mag_valid: bool,

    /// Timestamp of last update (milliseconds since startup)
    pub last_update_ms: u64,
}

/// Attitude state - roll, pitch, yaw and angular rates
///
/// Represents the vehicle's orientation in space and its rotational motion.
/// All angles are in radians, rates in rad/s.
///
/// # Coordinate System
///
/// - Roll (φ): Rotation around X-axis (positive = right wing down)
/// - Pitch (θ): Rotation around Y-axis (positive = nose up)
/// - Yaw (ψ): Rotation around Z-axis (positive = clockwise from above, 0 = north)
#[derive(Debug, Clone, Copy)]
pub struct AttitudeState {
    /// Roll angle in radians
    pub roll: f32,

    /// Pitch angle in radians
    pub pitch: f32,

    /// Yaw angle in radians (heading)
    pub yaw: f32,

    /// Angular rates in body frame (rad/s)
    pub angular_rates: Vector3<f32>,

    /// Timestamp of this estimate (milliseconds since startup)
    pub timestamp_ms: u64,

    /// Quality indicators
    pub quality: AttitudeQuality,
}

impl Default for AttitudeState {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            angular_rates: Vector3::zeros(),
            timestamp_ms: 0,
            quality: AttitudeQuality::default(),
        }
    }
}

impl AttitudeState {
    /// Create new attitude state with given angles
    pub fn new(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            ..Default::default()
        }
    }

    /// Check if attitude data is fresh (updated within last 100ms)
    pub fn is_fresh(&self, current_time_ms: u64) -> bool {
        current_time_ms.saturating_sub(self.quality.last_update_ms) < 100
    }

    /// Get roll in degrees (convenience method)
    pub fn roll_deg(&self) -> f32 {
        self.roll.to_degrees()
    }

    /// Get pitch in degrees (convenience method)
    pub fn pitch_deg(&self) -> f32 {
        self.pitch.to_degrees()
    }

    /// Get yaw in degrees (convenience method)
    pub fn yaw_deg(&self) -> f32 {
        self.yaw.to_degrees()
    }
}

/// Thread-safe wrapper for attitude state
///
/// Uses critical sections for atomic read/write operations.
/// Since reads/writes are very fast (just copying a struct), this is
/// more efficient than a mutex for this use case.
pub struct SharedAttitudeState {
    state: core::cell::UnsafeCell<AttitudeState>,
}

// Safety: AttitudeState is a simple data struct with no internal mutability.
// We ensure thread safety through critical sections in the accessor methods.
unsafe impl Sync for SharedAttitudeState {}
unsafe impl Send for SharedAttitudeState {}

impl SharedAttitudeState {
    /// Create new shared attitude state
    pub const fn new() -> Self {
        Self {
            state: core::cell::UnsafeCell::new(AttitudeState {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                angular_rates: Vector3::new(0.0, 0.0, 0.0),
                timestamp_ms: 0,
                quality: AttitudeQuality {
                    converged: false,
                    mag_valid: false,
                    last_update_ms: 0,
                },
            }),
        }
    }

    /// Read current attitude state (thread-safe)
    pub fn read(&self) -> AttitudeState {
        critical_section::with(|_cs| unsafe { *self.state.get() })
    }

    /// Write new attitude state (thread-safe)
    pub fn write(&self, state: AttitudeState) {
        critical_section::with(|_cs| unsafe {
            *self.state.get() = state;
        });
    }

    /// Update attitude angles only (convenience method)
    pub fn update_attitude(&self, roll: f32, pitch: f32, yaw: f32, timestamp_ms: u64) {
        critical_section::with(|_cs| unsafe {
            let state = &mut *self.state.get();
            state.roll = roll;
            state.pitch = pitch;
            state.yaw = yaw;
            state.timestamp_ms = timestamp_ms;
            state.quality.last_update_ms = timestamp_ms;
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

    /// Get angular rates (thread-safe)
    pub fn get_angular_rates(&self) -> Vector3<f32> {
        critical_section::with(|_cs| unsafe { (*self.state.get()).angular_rates })
    }

    /// Check if converged (thread-safe)
    pub fn is_converged(&self) -> bool {
        critical_section::with(|_cs| unsafe { (*self.state.get()).quality.converged })
    }

    /// Set convergence flag (thread-safe)
    pub fn set_converged(&self, converged: bool) {
        critical_section::with(|_cs| unsafe {
            (*self.state.get()).quality.converged = converged;
        });
    }
}

impl Default for SharedAttitudeState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_attitude_state_default() {
        let state = AttitudeState::default();
        assert_eq!(state.roll, 0.0);
        assert_eq!(state.pitch, 0.0);
        assert_eq!(state.yaw, 0.0);
        assert_eq!(state.angular_rates, Vector3::zeros());
        assert!(!state.quality.converged);
    }

    #[test]
    fn test_attitude_state_new() {
        let state = AttitudeState::new(0.1, 0.2, 0.3);
        assert_eq!(state.roll, 0.1);
        assert_eq!(state.pitch, 0.2);
        assert_eq!(state.yaw, 0.3);
    }

    #[test]
    fn test_is_fresh() {
        let mut state = AttitudeState::default();
        state.quality.last_update_ms = 100;

        assert!(state.is_fresh(150)); // 50ms ago - fresh
        assert!(state.is_fresh(199)); // 99ms ago - fresh
        assert!(!state.is_fresh(201)); // 101ms ago - stale
    }

    #[test]
    fn test_degree_conversions() {
        use core::f32::consts::PI;

        let state = AttitudeState::new(PI / 2.0, PI, -PI / 4.0);

        assert!((state.roll_deg() - 90.0).abs() < 0.001);
        assert!((state.pitch_deg() - 180.0).abs() < 0.001);
        assert!((state.yaw_deg() + 45.0).abs() < 0.001);
    }

    #[test]
    fn test_shared_attitude_state() {
        let shared = SharedAttitudeState::new();

        // Initial state should be default
        let state = shared.read();
        assert_eq!(state.roll, 0.0);

        // Write new state
        let new_state = AttitudeState::new(0.5, 0.6, 0.7);
        shared.write(new_state);

        // Read back
        let read_state = shared.read();
        assert_eq!(read_state.roll, 0.5);
        assert_eq!(read_state.pitch, 0.6);
        assert_eq!(read_state.yaw, 0.7);
    }

    #[test]
    fn test_shared_attitude_getters() {
        let shared = SharedAttitudeState::new();

        shared.update_attitude(1.0, 2.0, 3.0, 100);

        assert_eq!(shared.get_roll(), 1.0);
        assert_eq!(shared.get_pitch(), 2.0);
        assert_eq!(shared.get_yaw(), 3.0);

        let state = shared.read();
        assert_eq!(state.timestamp_ms, 100);
    }

    #[test]
    fn test_convergence_flag() {
        let shared = SharedAttitudeState::new();

        assert!(!shared.is_converged());

        shared.set_converged(true);
        assert!(shared.is_converged());

        shared.set_converged(false);
        assert!(!shared.is_converged());
    }
}
