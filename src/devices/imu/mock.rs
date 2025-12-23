//! Mock IMU implementation for testing
//!
//! Provides a configurable mock IMU that implements `ImuSensor` trait.
//! Useful for unit testing EKF and other subsystems without hardware.
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::imu::MockImu;
//! use pico_trail::devices::traits::{ImuSensor, ImuReading};
//!
//! // Create with default reading
//! let mut imu = MockImu::with_default_reading();
//!
//! // Create with sequence of readings
//! let readings = vec![reading1, reading2, reading3];
//! let mut imu = MockImu::with_readings(readings);
//!
//! // Read data
//! let reading = imu.read_all().await?;
//! ```

use crate::devices::traits::{ImuCalibration, ImuError, ImuReading, ImuSensor};
use nalgebra::Vector3;

/// Mock IMU for testing
///
/// Provides preset readings in sequence for testing EKF and other
/// IMU consumers without hardware.
pub struct MockImu {
    /// Queue of readings to return
    readings: heapless::Deque<ImuReading, 64>,

    /// Default reading when queue is empty
    default_reading: ImuReading,

    /// Current calibration
    calibration: ImuCalibration,

    /// Health status (can be set for testing error handling)
    healthy: bool,

    /// Error count for health tracking
    error_count: u32,

    /// Timestamp counter (microseconds)
    timestamp_us: u64,
}

impl MockImu {
    /// Create a new mock IMU with default reading
    pub fn with_default_reading() -> Self {
        Self {
            readings: heapless::Deque::new(),
            default_reading: ImuReading::default(),
            calibration: ImuCalibration::default(),
            healthy: true,
            error_count: 0,
            timestamp_us: 0,
        }
    }

    /// Create a mock IMU with a sequence of readings
    pub fn with_readings(readings: &[ImuReading]) -> Self {
        let mut deque = heapless::Deque::new();
        for reading in readings.iter().take(64) {
            let _ = deque.push_back(*reading);
        }

        Self {
            readings: deque,
            default_reading: ImuReading::default(),
            calibration: ImuCalibration::default(),
            healthy: true,
            error_count: 0,
            timestamp_us: 0,
        }
    }

    /// Set the default reading to return when queue is empty
    pub fn set_default_reading(&mut self, reading: ImuReading) {
        self.default_reading = reading;
    }

    /// Push a new reading onto the queue
    pub fn push_reading(&mut self, reading: ImuReading) -> Result<(), ImuReading> {
        self.readings.push_back(reading)
    }

    /// Set health status (for testing error handling)
    pub fn set_healthy(&mut self, healthy: bool) {
        self.healthy = healthy;
    }

    /// Get the next reading, applying calibration
    fn next_reading(&mut self) -> ImuReading {
        // Advance timestamp by 2500us (400Hz)
        self.timestamp_us += 2500;

        let mut reading = self.readings.pop_front().unwrap_or(self.default_reading);
        reading.timestamp_us = self.timestamp_us;

        // Apply calibration
        reading.gyro = self.calibration.apply_gyro(reading.gyro);
        reading.accel = self.calibration.apply_accel(reading.accel);
        reading.mag = self.calibration.apply_mag(reading.mag);

        reading
    }
}

impl ImuSensor for MockImu {
    async fn read_all(&mut self) -> Result<ImuReading, ImuError> {
        if !self.healthy {
            self.error_count += 1;
            return Err(ImuError::I2cError);
        }

        self.error_count = 0;
        Ok(self.next_reading())
    }

    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
        if !self.healthy {
            return Err(ImuError::I2cError);
        }

        let reading = self.next_reading();
        Ok(reading.gyro)
    }

    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
        if !self.healthy {
            return Err(ImuError::I2cError);
        }

        let reading = self.next_reading();
        Ok(reading.accel)
    }

    async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError> {
        if !self.healthy {
            return Err(ImuError::I2cError);
        }

        let reading = self.next_reading();
        Ok(reading.mag)
    }

    fn set_calibration(&mut self, calibration: ImuCalibration) {
        self.calibration = calibration;
    }

    fn calibration(&self) -> &ImuCalibration {
        &self.calibration
    }

    fn is_healthy(&self) -> bool {
        self.healthy && self.error_count < 3
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        // Simple blocking executor for tests
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
    fn test_mock_imu_default_reading() {
        let mut imu = MockImu::with_default_reading();

        let reading = block_on(imu.read_all()).unwrap();

        assert_eq!(reading.gyro, Vector3::zeros());
        assert!((reading.accel.z - 9.80665).abs() < 0.001);
        assert!(reading.timestamp_us > 0);
    }

    #[test]
    fn test_mock_imu_with_readings() {
        let reading1 = ImuReading {
            gyro: Vector3::new(0.1, 0.2, 0.3),
            ..Default::default()
        };
        let reading2 = ImuReading {
            gyro: Vector3::new(0.4, 0.5, 0.6),
            ..Default::default()
        };

        let mut imu = MockImu::with_readings(&[reading1, reading2]);

        let r1 = block_on(imu.read_all()).unwrap();
        let r2 = block_on(imu.read_all()).unwrap();

        assert!((r1.gyro.x - 0.1).abs() < 1e-6);
        assert!((r2.gyro.x - 0.4).abs() < 1e-6);
    }

    #[test]
    fn test_mock_imu_unhealthy() {
        let mut imu = MockImu::with_default_reading();
        imu.set_healthy(false);

        let result = block_on(imu.read_all());
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ImuError::I2cError);
    }

    #[test]
    fn test_mock_imu_calibration() {
        let mut imu = MockImu::with_default_reading();

        let cal = ImuCalibration {
            gyro_bias: Vector3::new(0.01, 0.01, 0.01),
            ..Default::default()
        };
        imu.set_calibration(cal);

        // Set a default reading with known gyro values
        let reading = ImuReading {
            gyro: Vector3::new(0.1, 0.1, 0.1),
            ..Default::default()
        };
        imu.set_default_reading(reading);

        let result = block_on(imu.read_all()).unwrap();

        // Calibration should subtract bias
        assert!((result.gyro.x - 0.09).abs() < 1e-6);
    }

    #[test]
    fn test_mock_imu_timestamp_increment() {
        let mut imu = MockImu::with_default_reading();

        let r1 = block_on(imu.read_all()).unwrap();
        let r2 = block_on(imu.read_all()).unwrap();

        // 400Hz = 2500us per sample
        assert_eq!(r2.timestamp_us - r1.timestamp_us, 2500);
    }

    #[test]
    fn test_mock_imu_is_healthy() {
        let mut imu = MockImu::with_default_reading();
        assert!(imu.is_healthy());

        imu.set_healthy(false);
        assert!(!imu.is_healthy());
    }
}
