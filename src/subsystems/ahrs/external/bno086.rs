//! BNO086 External AHRS Implementation
//!
//! Wraps the BNO086 driver to provide the unified `Ahrs` trait interface.
//! The BNO086 has on-chip sensor fusion (ARM Cortex-M0+) that outputs
//! quaternion orientation directly, eliminating the need for software EKF.
//!
//! ## Coordinate Frame
//!
//! The BNO086 outputs in Android-style coordinate system (X=right, Y=up, Z=out).
//! This implementation converts to NED (North-East-Down) frame used in flight control.
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
//! let driver = Bno086Driver::new(transport, config);
//! driver.init().await?;
//!
//! let mut ahrs = Bno086ExternalAhrs::new(driver);
//! let state = ahrs.get_attitude().await?;
//! ```

use crate::communication::shtp::ShtpTransport;
use crate::devices::imu::bno086::Bno086Driver;
use crate::devices::traits::QuaternionSensor;
use crate::subsystems::ahrs::{Ahrs, AhrsError, AhrsState, AhrsType};

/// BNO086 External AHRS wrapper
///
/// Provides the `Ahrs` trait implementation for BNO086 sensors.
/// The BNO086's on-chip fusion runs at 400Hz internally, providing
/// high-quality quaternion output with minimal CPU overhead.
///
/// ## Features
///
/// - On-chip 9-axis sensor fusion (no software EKF needed)
/// - 100Hz quaternion and gyroscope output
/// - Angular rate in NED frame for PID D-term
/// - Accuracy estimate from sensor
pub struct Bno086ExternalAhrs<T: ShtpTransport> {
    /// Underlying BNO086 driver
    driver: Bno086Driver<T>,
    /// Last AHRS state (cached)
    last_state: Option<AhrsState>,
}

impl<T: ShtpTransport> Bno086ExternalAhrs<T> {
    /// Create a new BNO086 External AHRS
    ///
    /// # Arguments
    ///
    /// * `driver` - Initialized BNO086 driver
    ///
    /// # Note
    ///
    /// The driver must be initialized (`driver.init().await?`) before
    /// creating the ExternalAhrs wrapper.
    pub fn new(driver: Bno086Driver<T>) -> Self {
        Self {
            driver,
            last_state: None,
        }
    }

    /// Get reference to underlying driver
    pub fn driver(&self) -> &Bno086Driver<T> {
        &self.driver
    }

    /// Get mutable reference to underlying driver
    pub fn driver_mut(&mut self) -> &mut Bno086Driver<T> {
        &mut self.driver
    }

    /// Consume the wrapper and return the underlying driver
    pub fn into_driver(self) -> Bno086Driver<T> {
        self.driver
    }
}

impl<T: ShtpTransport> Ahrs for Bno086ExternalAhrs<T> {
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError> {
        // Read quaternion from BNO086
        let reading = self.driver.read_quaternion().await?;

        // Get angular rate (already in NED frame from driver)
        let angular_rate = self.driver.angular_rate();

        // Build AhrsState with quaternion and angular rate
        let state = AhrsState::from_quaternion_reading(&reading).with_angular_rate(angular_rate);

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
        use crate::devices::imu::bno086::Bno086Config;

        let transport = MockTransport;
        let driver = Bno086Driver::new(transport, Bno086Config::default());
        let ahrs = Bno086ExternalAhrs::new(driver);

        // Should not be healthy (not initialized)
        assert!(!ahrs.is_healthy());
        assert_eq!(ahrs.ahrs_type(), AhrsType::External);
    }

    #[test]
    fn test_bno086_external_ahrs_driver_access() {
        use crate::devices::imu::bno086::Bno086Config;

        let transport = MockTransport;
        let driver = Bno086Driver::new(transport, Bno086Config::default());
        let ahrs = Bno086ExternalAhrs::new(driver);

        // Can access driver
        assert!(!ahrs.driver().is_initialized());

        // Can get driver back
        let _driver = ahrs.into_driver();
    }
}
