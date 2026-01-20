//! Raw IMU Sensor Trait
//!
//! Device-independent interface for raw IMU sensors without on-chip fusion.
//! This trait is used by Software AHRS (EKF/DCM) to process sensor data.
//!
//! ## Requirements
//!
//! - FR-z1fdo: ImuSensor Trait Interface
//! - ADR-nzvfy: AHRS Abstraction Architecture
//!
//! ## Relationship to ImuSensor
//!
//! `RawImu` is a simplified interface focused on the minimal data needed for
//! attitude estimation. `ImuSensor` provides more comprehensive functionality
//! including calibration management.
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::traits::{RawImu, ImuError};
//!
//! async fn run_software_ahrs<I: RawImu>(mut imu: I) {
//!     loop {
//!         let accel = imu.read_accel().await?;
//!         let gyro = imu.read_gyro().await?;
//!         // Process with EKF/DCM...
//!     }
//! }
//! ```

use super::ImuError;
use nalgebra::Vector3;

/// Raw IMU sensor interface for sensors without on-chip fusion
///
/// This trait provides the minimal interface needed for software AHRS
/// implementations (EKF, DCM) to process raw sensor data.
///
/// Unlike sensors with on-chip fusion (BNO086), raw IMU sensors output
/// unprocessed accelerometer, gyroscope, and optionally magnetometer data
/// that must be fused in software.
///
/// # Coordinate System
///
/// All outputs are in calibrated SI units with NED body frame convention:
/// - X: Right (starboard)
/// - Y: Forward (bow)
/// - Z: Down
#[allow(async_fn_in_trait)]
pub trait RawImu {
    /// Read calibrated accelerometer data (m/s²)
    ///
    /// Returns the current accelerometer reading in body frame.
    /// Includes gravity component (not compensated).
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated gyroscope data (rad/s)
    ///
    /// Returns the current angular rate in body frame.
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated magnetometer data (µT)
    ///
    /// Returns `Ok(Some(mag))` if magnetometer is available and ready,
    /// `Ok(None)` if magnetometer is not available on this sensor,
    /// or `Err` if there was a communication/data error.
    async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError>;

    /// Get sensor sample rate in Hz
    ///
    /// Returns the configured or actual sample rate of the sensor.
    /// Used by software AHRS for timing calculations.
    fn sample_rate(&self) -> u32;

    /// Check if sensor is healthy
    ///
    /// Returns `true` if the sensor is producing valid data.
    /// Returns `false` if there are communication errors, stuck values,
    /// or other indicators of sensor malfunction.
    fn is_healthy(&self) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock RawImu implementation for testing
    struct MockRawImu {
        accel: Vector3<f32>,
        gyro: Vector3<f32>,
        mag: Option<Vector3<f32>>,
        sample_rate_hz: u32,
        healthy: bool,
    }

    impl MockRawImu {
        fn new() -> Self {
            Self {
                accel: Vector3::new(0.0, 0.0, 9.80665),
                gyro: Vector3::zeros(),
                mag: None,
                sample_rate_hz: 100,
                healthy: true,
            }
        }

        fn with_mag(mut self, mag: Vector3<f32>) -> Self {
            self.mag = Some(mag);
            self
        }

        fn with_sample_rate(mut self, rate: u32) -> Self {
            self.sample_rate_hz = rate;
            self
        }

        fn set_unhealthy(&mut self) {
            self.healthy = false;
        }
    }

    impl RawImu for MockRawImu {
        async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
            if self.healthy {
                Ok(self.accel)
            } else {
                Err(ImuError::I2cError)
            }
        }

        async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
            if self.healthy {
                Ok(self.gyro)
            } else {
                Err(ImuError::I2cError)
            }
        }

        async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError> {
            if self.healthy {
                Ok(self.mag)
            } else {
                Err(ImuError::I2cError)
            }
        }

        fn sample_rate(&self) -> u32 {
            self.sample_rate_hz
        }

        fn is_healthy(&self) -> bool {
            self.healthy
        }
    }

    #[test]
    fn test_mock_raw_imu_creation() {
        let imu = MockRawImu::new();
        assert!(imu.is_healthy());
        assert_eq!(imu.sample_rate(), 100);
    }

    #[test]
    fn test_mock_raw_imu_with_mag() {
        let mag = Vector3::new(30.0, 0.0, -45.0);
        let imu = MockRawImu::new().with_mag(mag);
        assert!(imu.mag.is_some());
    }

    #[test]
    fn test_mock_raw_imu_sample_rate() {
        let imu = MockRawImu::new().with_sample_rate(400);
        assert_eq!(imu.sample_rate(), 400);
    }

    #[test]
    fn test_mock_raw_imu_unhealthy() {
        let mut imu = MockRawImu::new();
        assert!(imu.is_healthy());

        imu.set_unhealthy();
        assert!(!imu.is_healthy());
    }

    /// Simple blocking executor for async tests (no external deps)
    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        fn dummy_raw_waker() -> RawWaker {
            fn no_op(_: *const ()) {}
            fn clone(_: *const ()) -> RawWaker {
                dummy_raw_waker()
            }
            const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
            RawWaker::new(core::ptr::null(), &VTABLE)
        }

        let waker = unsafe { Waker::from_raw(dummy_raw_waker()) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = core::pin::pin!(fut);

        loop {
            match fut.as_mut().poll(&mut cx) {
                Poll::Ready(result) => return result,
                Poll::Pending => continue,
            }
        }
    }

    #[test]
    fn test_read_accel_healthy() {
        let mut imu = MockRawImu::new();
        let result = block_on(imu.read_accel());
        assert!(result.is_ok());

        let accel = result.unwrap();
        assert!((accel.z - 9.80665).abs() < 0.001);
    }

    #[test]
    fn test_read_accel_unhealthy() {
        let mut imu = MockRawImu::new();
        imu.set_unhealthy();

        let result = block_on(imu.read_accel());
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ImuError::I2cError);
    }

    #[test]
    fn test_read_gyro() {
        let mut imu = MockRawImu::new();
        let result = block_on(imu.read_gyro());
        assert!(result.is_ok());

        let gyro = result.unwrap();
        assert_eq!(gyro, Vector3::zeros());
    }

    #[test]
    fn test_read_mag_none() {
        let mut imu = MockRawImu::new();
        let result = block_on(imu.read_mag());
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_read_mag_some() {
        let mag = Vector3::new(25.0, 10.0, -40.0);
        let mut imu = MockRawImu::new().with_mag(mag);

        let result = block_on(imu.read_mag());
        assert!(result.is_ok());

        let read_mag = result.unwrap().unwrap();
        assert_eq!(read_mag, mag);
    }
}
