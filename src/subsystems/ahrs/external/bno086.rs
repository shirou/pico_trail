//! BNO086 External AHRS Implementation
//!
//! Wraps any `QuaternionSensor` driver (BNO086, BNO085, etc.) to provide
//! the unified `Ahrs` trait interface. This includes `Bno086Driver` (polling)
//! and `Bno086DriverWithGpio` (INT-driven).
//!
//! ## Coordinate Frame Conversion
//!
//! The BNO086 outputs in ENU coordinate system:
//! - X = East (Right when facing North)
//! - Y = North (Forward)
//! - Z = Up
//!
//! This implementation converts to NED (North-East-Down) frame used in MAVLink/ArduPilot:
//! - X = Forward (North)
//! - Y = Right (East)
//! - Z = Down
//!
//! Conversion formula for quaternion: q_ned = (w, y_enu, -x_enu, -z_enu)
//! Conversion formula for angular rate: gyro_ned = (y_enu, -x_enu, -z_enu)
//!
//! ## Requirements
//!
//! - ADR-nzvfy: AHRS Abstraction Architecture
//! - T-7khm3: AHRS Abstraction Layer Implementation
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::subsystems::ahrs::external::Bno086ExternalAhrs;
//! use pico_trail::subsystems::ahrs::Ahrs;
//! use pico_trail::devices::imu::bno086::Bno086Driver;
//!
//! // With basic polling driver
//! let driver = Bno086Driver::new(transport, config);
//! driver.init().await?;
//! let mut ahrs = Bno086ExternalAhrs::new(driver);
//!
//! // Or with GPIO driver (INT-driven, recommended)
//! let driver = Bno086DriverWithGpio::new(transport, int_pin, rst_pin, config);
//! driver.init().await?;
//! let mut ahrs = Bno086ExternalAhrs::new(driver);
//!
//! let state = ahrs.get_attitude().await?;
//! ```

use crate::devices::traits::QuaternionSensor;
use crate::subsystems::ahrs::{Ahrs, AhrsError, AhrsState, AhrsType};

/// BNO086 External AHRS wrapper
///
/// Provides the `Ahrs` trait implementation for any `QuaternionSensor`.
/// Works with `Bno086Driver` (polling mode) or `Bno086DriverWithGpio`
/// (INT-driven mode, recommended for reliable operation).
///
/// ## Features
///
/// - On-chip 9-axis sensor fusion (no software EKF needed)
/// - 100Hz quaternion and gyroscope output
/// - Angular rate in NED frame for PID D-term
/// - Accuracy estimate from sensor
///
/// ## Type Parameter
///
/// * `D` - Any driver implementing `QuaternionSensor` trait
pub struct Bno086ExternalAhrs<D: QuaternionSensor> {
    /// Underlying quaternion sensor driver
    driver: D,
    /// Last AHRS state (cached)
    last_state: Option<AhrsState>,
}

impl<D: QuaternionSensor> Bno086ExternalAhrs<D> {
    /// Create a new BNO086 External AHRS
    ///
    /// # Arguments
    ///
    /// * `driver` - Initialized driver implementing `QuaternionSensor`
    ///
    /// # Note
    ///
    /// The driver must be initialized (`driver.init().await?`) before
    /// creating the ExternalAhrs wrapper.
    pub fn new(driver: D) -> Self {
        Self {
            driver,
            last_state: None,
        }
    }

    /// Get reference to underlying driver
    pub fn driver(&self) -> &D {
        &self.driver
    }

    /// Get mutable reference to underlying driver
    pub fn driver_mut(&mut self) -> &mut D {
        &mut self.driver
    }

    /// Consume the wrapper and return the underlying driver
    pub fn into_driver(self) -> D {
        self.driver
    }
}

impl<D: QuaternionSensor> Ahrs for Bno086ExternalAhrs<D> {
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError> {
        // Read quaternion from BNO086 (Android frame)
        let reading = self.driver.read_quaternion().await?;

        // Convert quaternion from ENU to NED frame
        // BNO086 ENU frame: X=East(right), Y=North(forward), Z=Up
        // NED frame: X=North(forward), Y=East(right), Z=Down
        //
        // Axis mapping:
        // NED_X (North) = ENU_Y (North)
        // NED_Y (East)  = ENU_X (East), but negated to fix pitch direction
        // NED_Z (Down)  = -ENU_Z (Up inverted)
        //
        // Quaternion transformation: q_ned = (w, y, -x, -z) from q_enu = (w, x, y, z)
        let q_enu = reading.quaternion;
        let q_ned = nalgebra::Quaternion::new(
            q_enu.w,  // w unchanged
            q_enu.j,  // x_ned = y_enu (North)
            -q_enu.i, // y_ned = -x_enu (East, negated for pitch direction)
            -q_enu.k, // z_ned = -z_enu (Down)
        );

        // Create modified reading with NED quaternion
        let reading_ned = crate::devices::traits::QuaternionReading {
            quaternion: q_ned,
            accuracy_rad: reading.accuracy_rad,
            timestamp_us: reading.timestamp_us,
        };

        // Get angular rate and convert to NED frame
        // ENU gyro: x=around East, y=around North, z=around Up
        // NED gyro: x=roll(around North), y=pitch(around East), z=yaw(around Down)
        //
        // NED roll (around North)  = ENU gyro_y (around North)
        // NED pitch (around East)  = -ENU gyro_x (around East, negated)
        // NED yaw (around Down)    = -ENU gyro_z (around Up, inverted)
        let gyro_enu = self.driver.angular_rate();
        let gyro_ned = nalgebra::Vector3::new(
            gyro_enu.y,  // X_ned (roll) = Y_enu
            -gyro_enu.x, // Y_ned (pitch) = -X_enu
            -gyro_enu.z, // Z_ned (yaw) = -Z_enu
        );

        // Build AhrsState with NED quaternion and angular rate
        let state = AhrsState::from_quaternion_reading(&reading_ned).with_angular_rate(gyro_ned);

        // Debug: log ENU→NED conversion to verify attitude extraction
        #[cfg(feature = "pico2_w")]
        crate::log_debug!(
            "ENU q=({},{},{},{}) -> NED q=({},{},{},{}) -> pitch={}deg",
            q_enu.w,
            q_enu.i,
            q_enu.j,
            q_enu.k,
            q_ned.w,
            q_ned.i,
            q_ned.j,
            q_ned.k,
            state.pitch.to_degrees()
        );

        self.last_state = Some(state);
        Ok(state)
    }

    fn is_healthy(&self) -> bool {
        self.driver.is_healthy()
    }

    fn ahrs_type(&self) -> AhrsType {
        AhrsType::External
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::shtp::{ShtpError, ShtpPacket, ShtpTransport};
    use crate::devices::imu::bno086::{Bno086Config, Bno086Driver};

    /// Mock transport for testing
    struct MockTransport;

    impl ShtpTransport for MockTransport {
        async fn read_packet<const N: usize>(
            &mut self,
            _packet: &mut ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            Err(ShtpError::NoData)
        }

        async fn write_packet<const N: usize>(
            &mut self,
            _packet: &ShtpPacket<N>,
        ) -> Result<(), ShtpError> {
            Ok(())
        }

        fn reset(&mut self) {}
    }

    #[test]
    fn test_bno086_external_ahrs_new() {
        let transport = MockTransport;
        let driver = Bno086Driver::new(transport, Bno086Config::default());
        let ahrs = Bno086ExternalAhrs::new(driver);

        // Should not be healthy (not initialized)
        assert!(!ahrs.is_healthy());
        assert_eq!(ahrs.ahrs_type(), AhrsType::External);
    }

    #[test]
    fn test_bno086_external_ahrs_driver_access() {
        let transport = MockTransport;
        let driver = Bno086Driver::new(transport, Bno086Config::default());
        let ahrs = Bno086ExternalAhrs::new(driver);

        // Can access driver (is_healthy is false because not initialized)
        assert!(!ahrs.driver().is_healthy());

        // Can get driver back
        let _driver = ahrs.into_driver();
    }
}
