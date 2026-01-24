//! Armed State Monitoring
//!
//! Provides multi-rate continuous health monitoring during armed operation.
//!
//! # Monitoring Rates
//!
//! - **High-frequency (400 Hz)**: RC signal age, sensor health flags
//! - **Medium-frequency (10 Hz)**: Battery voltage, EKF health validation
//! - **Low-frequency (1 Hz)**: Geofence violation checking, GCS status reporting
//!
//! # Architecture
//!
//! The monitoring system uses separate async tasks for each rate tier,
//! allowing fine-grained control over monitoring priorities and CPU usage.

use crate::communication::mavlink::state::SystemState;

/// Sensor health flags for quick status checking
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorHealthFlags {
    /// IMU/accelerometer healthy
    pub imu_healthy: bool,
    /// Gyroscope healthy
    pub gyro_healthy: bool,
    /// Magnetometer healthy
    pub mag_healthy: bool,
    /// GPS healthy
    pub gps_healthy: bool,
    /// Barometer healthy
    pub baro_healthy: bool,
}

/// EKF health status
#[derive(Debug, Clone, Copy, Default)]
pub struct EkfStatus {
    /// EKF is running
    pub running: bool,
    /// Position estimate is valid
    pub position_valid: bool,
    /// Velocity estimate is valid
    pub velocity_valid: bool,
    /// Attitude estimate is valid
    pub attitude_valid: bool,
}

impl EkfStatus {
    /// Check if EKF is healthy (running and all estimates valid)
    pub fn is_healthy(&self) -> bool {
        self.running && self.position_valid && self.velocity_valid && self.attitude_valid
    }
}

/// Geofence status
#[derive(Debug, Clone, Copy, Default)]
pub struct FenceStatus {
    /// Geofence is enabled
    pub enabled: bool,
    /// Vehicle is outside geofence boundaries
    pub violated: bool,
    /// Distance to nearest boundary (meters)
    pub distance_to_boundary: f32,
}

/// Failsafe reason for armed state monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FailsafeReason {
    /// RC signal lost (no signal for > 1 second)
    RcLoss,
    /// Battery voltage critically low
    BatteryCritical,
    /// Geofence boundary violated
    FenceViolation,
    /// GCS heartbeat lost
    GcsLoss,
    /// Sensor failure detected
    SensorFailure,
}

impl core::fmt::Display for FailsafeReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            FailsafeReason::RcLoss => write!(f, "RC signal loss"),
            FailsafeReason::BatteryCritical => write!(f, "Battery critical"),
            FailsafeReason::FenceViolation => write!(f, "Geofence violation"),
            FailsafeReason::GcsLoss => write!(f, "GCS heartbeat loss"),
            FailsafeReason::SensorFailure => write!(f, "Sensor failure"),
        }
    }
}

/// Armed state monitor with multi-rate health checking
///
/// Monitors system health at different rates during armed operation:
/// - Fast checks (400 Hz): RC signal, sensor flags
/// - Medium checks (10 Hz): Battery voltage, EKF health
/// - Slow checks (1 Hz): Geofence, GCS status
pub struct ArmedStateMonitor {
    // High-frequency state (400 Hz)
    /// Last RC signal received timestamp (milliseconds since boot)
    rc_last_received_ms: Option<u64>,
    /// Sensor health flags
    sensor_health: SensorHealthFlags,

    // Medium-frequency state (10 Hz)
    /// Battery voltage (volts)
    battery_voltage: f32,
    /// Battery critical threshold (volts)
    battery_critical_threshold: f32,
    /// EKF health status
    ekf_status: EkfStatus,

    // Low-frequency state (1 Hz)
    /// Geofence status
    fence_status: FenceStatus,
    /// Last GCS heartbeat timestamp (milliseconds since boot)
    gcs_last_heartbeat_ms: Option<u64>,
}

impl ArmedStateMonitor {
    /// Create a new armed state monitor
    ///
    /// # Arguments
    ///
    /// * `battery_critical_threshold` - Battery voltage threshold for critical failsafe (volts)
    pub fn new(battery_critical_threshold: f32) -> Self {
        Self {
            rc_last_received_ms: None,
            sensor_health: SensorHealthFlags::default(),
            battery_voltage: 0.0,
            battery_critical_threshold,
            ekf_status: EkfStatus::default(),
            fence_status: FenceStatus::default(),
            gcs_last_heartbeat_ms: None,
        }
    }

    /// Update high-frequency monitoring state (400 Hz)
    ///
    /// Checks:
    /// - RC signal age (timeout = 1 second)
    /// - Sensor health flags
    ///
    /// Returns Some(FailsafeReason) if a failsafe condition is detected.
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    /// * `current_time_ms` - Current system time (milliseconds since boot)
    pub fn update_fast(
        &mut self,
        _state: &SystemState,
        current_time_ms: u64,
    ) -> Option<FailsafeReason> {
        // Update RC signal age
        // TODO: When RC system is implemented, read from context.rc.last_received()
        // For now, use placeholder
        if let Some(last_rc) = self.rc_last_received_ms {
            let rc_age_ms = current_time_ms.saturating_sub(last_rc);
            if rc_age_ms > 1000 {
                // RC timeout: 1 second
                crate::log_warn!("RC signal timeout: {} ms", rc_age_ms);
                return Some(FailsafeReason::RcLoss);
            }
        }

        // Update sensor health flags
        // TODO: When sensor system is implemented, read from context.sensors
        // For now, assume all sensors healthy
        self.sensor_health = SensorHealthFlags {
            imu_healthy: true,
            gyro_healthy: true,
            mag_healthy: true,
            gps_healthy: true,
            baro_healthy: true,
        };

        // Check for sensor failures
        if !self.sensor_health.imu_healthy || !self.sensor_health.gyro_healthy {
            crate::log_warn!("Critical sensor failure detected");
            return Some(FailsafeReason::SensorFailure);
        }

        None
    }

    /// Update medium-frequency monitoring state (10 Hz)
    ///
    /// Checks:
    /// - Battery voltage (critical threshold)
    /// - EKF health validation
    ///
    /// Returns Some(FailsafeReason) if a failsafe condition is detected.
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    pub fn update_medium(&mut self, state: &SystemState) -> Option<FailsafeReason> {
        // Update battery voltage
        self.battery_voltage = state.battery.voltage;

        // Check battery critical threshold
        if self.battery_voltage < self.battery_critical_threshold {
            crate::log_warn!(
                "Battery voltage critical: {}V < {}V",
                self.battery_voltage,
                self.battery_critical_threshold
            );
            return Some(FailsafeReason::BatteryCritical);
        }

        // Update EKF health
        // TODO: When AHRS/EKF system is implemented, read from context.ahrs.ekf_status()
        // For now, assume EKF healthy
        self.ekf_status = EkfStatus {
            running: true,
            position_valid: true,
            velocity_valid: true,
            attitude_valid: true,
        };

        // Check EKF health
        if !self.ekf_status.is_healthy() {
            crate::log_warn!("EKF health degraded");
            // EKF unhealthy is a warning, not an immediate failsafe
            // May trigger failsafe based on vehicle mode requirements
        }

        None
    }

    /// Update low-frequency monitoring state (1 Hz)
    ///
    /// Checks:
    /// - Geofence violations
    /// - GCS heartbeat timeout
    ///
    /// Returns Some(FailsafeReason) if a failsafe condition is detected.
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    /// * `current_time_ms` - Current system time (milliseconds since boot)
    pub fn update_slow(
        &mut self,
        _state: &SystemState,
        current_time_ms: u64,
    ) -> Option<FailsafeReason> {
        // Update geofence status
        // TODO: When fence system is implemented, read from context.fence.check_boundaries()
        // For now, assume no fence violation
        self.fence_status = FenceStatus {
            enabled: false,
            violated: false,
            distance_to_boundary: 1000.0,
        };

        // Check geofence violation
        if self.fence_status.enabled && self.fence_status.violated {
            crate::log_warn!("Geofence violation detected");
            return Some(FailsafeReason::FenceViolation);
        }

        // Check GCS heartbeat timeout
        // TODO: When MAVLink telemetry tracking is implemented
        if let Some(last_heartbeat) = self.gcs_last_heartbeat_ms {
            let heartbeat_age_ms = current_time_ms.saturating_sub(last_heartbeat);
            if heartbeat_age_ms > 5000 {
                // GCS timeout: 5 seconds
                crate::log_warn!("GCS heartbeat timeout: {} ms", heartbeat_age_ms);
                return Some(FailsafeReason::GcsLoss);
            }
        }

        // Generate SYS_STATUS message for GCS
        // TODO: When MAVLink telemetry is implemented, send SYS_STATUS
        crate::log_debug!(
            "SYS_STATUS: bat={}V sensors_ok={}",
            self.battery_voltage,
            self.sensor_health.imu_healthy
        );

        None
    }

    /// Update RC signal timestamp (called when RC data received)
    ///
    /// # Arguments
    ///
    /// * `timestamp_ms` - Timestamp when RC signal was received (milliseconds since boot)
    pub fn update_rc_timestamp(&mut self, timestamp_ms: u64) {
        self.rc_last_received_ms = Some(timestamp_ms);
    }

    /// Update GCS heartbeat timestamp (called when GCS heartbeat received)
    ///
    /// # Arguments
    ///
    /// * `timestamp_ms` - Timestamp when GCS heartbeat was received (milliseconds since boot)
    pub fn update_gcs_heartbeat(&mut self, timestamp_ms: u64) {
        self.gcs_last_heartbeat_ms = Some(timestamp_ms);
    }

    /// Get current sensor health flags
    pub fn sensor_health(&self) -> &SensorHealthFlags {
        &self.sensor_health
    }

    /// Get current EKF status
    pub fn ekf_status(&self) -> &EkfStatus {
        &self.ekf_status
    }

    /// Get current fence status
    pub fn fence_status(&self) -> &FenceStatus {
        &self.fence_status
    }

    /// Get current battery voltage
    pub fn battery_voltage(&self) -> f32 {
        self.battery_voltage
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_monitor_new() {
        let monitor = ArmedStateMonitor::new(10.5);
        assert_eq!(monitor.battery_critical_threshold, 10.5);
        assert!(monitor.rc_last_received_ms.is_none());
        assert!(monitor.gcs_last_heartbeat_ms.is_none());
    }

    #[test]
    fn test_update_fast_rc_timeout() {
        let mut monitor = ArmedStateMonitor::new(10.5);
        let state = SystemState::new();

        // Set RC received at t=0
        monitor.update_rc_timestamp(0);

        // Update at t=500ms - should be OK
        let result = monitor.update_fast(&state, 500);
        assert!(result.is_none());

        // Update at t=1500ms - should trigger RC loss failsafe
        let result = monitor.update_fast(&state, 1500);
        assert_eq!(result, Some(FailsafeReason::RcLoss));
    }

    #[test]
    fn test_update_medium_battery_critical() {
        let mut monitor = ArmedStateMonitor::new(10.5);
        let mut state = SystemState::new();

        // Battery above threshold - should be OK
        state.battery.voltage = 12.0;
        let result = monitor.update_medium(&state);
        assert!(result.is_none());

        // Battery below threshold - should trigger failsafe
        state.battery.voltage = 10.0;
        let result = monitor.update_medium(&state);
        assert_eq!(result, Some(FailsafeReason::BatteryCritical));
    }

    #[test]
    fn test_update_slow_gcs_timeout() {
        let mut monitor = ArmedStateMonitor::new(10.5);
        let state = SystemState::new();

        // Set GCS heartbeat at t=0
        monitor.update_gcs_heartbeat(0);

        // Update at t=3000ms - should be OK
        let result = monitor.update_slow(&state, 3000);
        assert!(result.is_none());

        // Update at t=6000ms - should trigger GCS loss failsafe
        let result = monitor.update_slow(&state, 6000);
        assert_eq!(result, Some(FailsafeReason::GcsLoss));
    }

    #[test]
    fn test_ekf_status_healthy() {
        let healthy = EkfStatus {
            running: true,
            position_valid: true,
            velocity_valid: true,
            attitude_valid: true,
        };
        assert!(healthy.is_healthy());

        let unhealthy = EkfStatus {
            running: true,
            position_valid: false,
            velocity_valid: true,
            attitude_valid: true,
        };
        assert!(!unhealthy.is_healthy());
    }

    #[test]
    fn test_failsafe_reason_display() {
        assert_eq!(format!("{}", FailsafeReason::RcLoss), "RC signal loss");
        assert_eq!(
            format!("{}", FailsafeReason::BatteryCritical),
            "Battery critical"
        );
        assert_eq!(
            format!("{}", FailsafeReason::FenceViolation),
            "Geofence violation"
        );
    }

    #[test]
    fn test_sensor_health_accessors() {
        let mut monitor = ArmedStateMonitor::new(10.5);
        let mut state = SystemState::new();
        state.battery.voltage = 12.0; // Set battery above critical

        // Trigger updates to populate all monitoring state
        let _ = monitor.update_fast(&state, 0); // Populates sensor_health
        let _ = monitor.update_medium(&state); // Populates ekf_status

        // Verify accessors work
        assert!(monitor.sensor_health().imu_healthy);
        assert!(monitor.ekf_status().is_healthy());
    }
}
