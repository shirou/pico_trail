//! Quaternion Sensor Trait and Data Types
//!
//! Device-independent interface for sensors that provide direct quaternion output,
//! such as BNO086, BNO085, BNO080, and FSM300 with on-chip sensor fusion.
//!
//! ## Requirements
//!
//! - FR-z1fdo: ImuSensor Trait Interface (related)
//! - AN-srhcj: BNO086 IMU Integration Analysis
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::traits::{QuaternionSensor, QuaternionReading};
//!
//! async fn run_attitude<Q: QuaternionSensor>(mut sensor: Q) {
//!     loop {
//!         let reading = sensor.read_quaternion().await?;
//!         // Use reading.quaternion for attitude estimation
//!     }
//! }
//! ```

use nalgebra::Quaternion;

/// Quaternion sensor error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum QuaternionError {
    /// I2C/SPI communication failed
    I2cError,

    /// Protocol error (e.g., SHTP packet parsing failed)
    ProtocolError,

    /// Sensor not initialized
    NotInitialized,

    /// Sensor reset in progress
    Resetting,

    /// Invalid data received from sensor
    InvalidData,

    /// Timeout waiting for data
    Timeout,
}

/// Quaternion reading with accuracy and timestamp
///
/// Represents a single quaternion measurement from a sensor with on-chip fusion.
#[derive(Debug, Clone, Copy)]
pub struct QuaternionReading {
    /// Unit quaternion representing orientation (w, x, y, z)
    ///
    /// Convention: w is the scalar component, (x, y, z) is the vector part.
    /// The quaternion is normalized (unit quaternion).
    pub quaternion: Quaternion<f32>,

    /// Accuracy estimate in radians
    ///
    /// Represents the estimated accuracy of the quaternion measurement.
    /// Lower values indicate higher accuracy. Typical range: 0.0 to 3.14 radians.
    pub accuracy_rad: f32,

    /// Timestamp: microseconds since boot
    pub timestamp_us: u64,
}

impl Default for QuaternionReading {
    fn default() -> Self {
        Self {
            quaternion: Quaternion::identity(),
            accuracy_rad: core::f32::consts::PI, // Maximum uncertainty
            timestamp_us: 0,
        }
    }
}

impl QuaternionReading {
    /// Create a new quaternion reading
    pub fn new(quaternion: Quaternion<f32>, accuracy_rad: f32, timestamp_us: u64) -> Self {
        Self {
            quaternion,
            accuracy_rad,
            timestamp_us,
        }
    }

    /// Check if the quaternion is valid (unit quaternion within tolerance)
    pub fn is_valid(&self) -> bool {
        let norm = self.quaternion.norm();
        (norm - 1.0).abs() < 0.01 // 1% tolerance
    }
}

/// Device-independent interface for quaternion-native sensors
///
/// This trait abstracts sensors that provide direct quaternion output via
/// on-chip sensor fusion (e.g., BNO086, BNO085, BNO080). Unlike `ImuSensor`
/// which provides raw sensor data, `QuaternionSensor` provides pre-computed
/// orientation quaternions.
///
/// # Advantages over ImuSensor + EKF
///
/// - Reduced CPU overhead (fusion computed on sensor's dedicated processor)
/// - Consistent fusion quality (sensor vendor's optimized algorithms)
/// - Lower latency (no external EKF computation delay)
///
/// # Implementation Notes
///
/// - Implementations should handle sensor initialization internally
/// - Health status should reflect both communication and data quality
/// - Timestamp should use a consistent clock source (embassy_time recommended)
#[allow(async_fn_in_trait)]
pub trait QuaternionSensor {
    /// Read quaternion with accuracy estimate
    ///
    /// Returns the latest quaternion reading from the sensor, including
    /// accuracy estimate and timestamp.
    ///
    /// # Errors
    ///
    /// - `QuaternionError::NotInitialized` - Sensor not initialized
    /// - `QuaternionError::I2cError` - Communication failure
    /// - `QuaternionError::Timeout` - No data within expected interval
    /// - `QuaternionError::InvalidData` - Data failed validation
    async fn read_quaternion(&mut self) -> Result<QuaternionReading, QuaternionError>;

    /// Get sensor health status
    ///
    /// Returns `true` if the sensor is operating normally:
    /// - Initialization complete
    /// - Recent successful read
    /// - No consecutive errors
    ///
    /// Returns `false` if:
    /// - Not initialized
    /// - Multiple consecutive communication errors
    /// - Sensor in reset/recovery state
    fn is_healthy(&self) -> bool;

    /// Get last update timestamp (microseconds since boot)
    ///
    /// Returns 0 if no successful read has occurred.
    fn last_update_us(&self) -> u64;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quaternion_reading_default() {
        let reading = QuaternionReading::default();
        assert_eq!(reading.quaternion, Quaternion::identity());
        assert!((reading.accuracy_rad - core::f32::consts::PI).abs() < 0.001);
        assert_eq!(reading.timestamp_us, 0);
    }

    #[test]
    fn test_quaternion_reading_new() {
        let q = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let reading = QuaternionReading::new(q, 0.1, 1000);

        assert_eq!(reading.quaternion.w, 1.0);
        assert_eq!(reading.accuracy_rad, 0.1);
        assert_eq!(reading.timestamp_us, 1000);
    }

    #[test]
    fn test_quaternion_reading_validity() {
        // Valid unit quaternion
        let valid = QuaternionReading::new(Quaternion::new(1.0, 0.0, 0.0, 0.0), 0.1, 0);
        assert!(valid.is_valid());

        // Valid normalized quaternion
        let normalized = QuaternionReading::new(
            Quaternion::new(0.707, 0.707, 0.0, 0.0), // approximately sqrt(2)/2
            0.1,
            0,
        );
        assert!(normalized.is_valid());

        // Invalid (not unit quaternion)
        let invalid = QuaternionReading::new(Quaternion::new(2.0, 0.0, 0.0, 0.0), 0.1, 0);
        assert!(!invalid.is_valid());
    }

    #[test]
    fn test_quaternion_error_variants() {
        // Verify all error variants exist and are distinct
        let errors = [
            QuaternionError::I2cError,
            QuaternionError::ProtocolError,
            QuaternionError::NotInitialized,
            QuaternionError::Resetting,
            QuaternionError::InvalidData,
            QuaternionError::Timeout,
        ];

        for (i, e1) in errors.iter().enumerate() {
            for (j, e2) in errors.iter().enumerate() {
                if i == j {
                    assert_eq!(e1, e2);
                } else {
                    assert_ne!(e1, e2);
                }
            }
        }
    }
}
