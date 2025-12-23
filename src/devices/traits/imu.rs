//! IMU Sensor Trait and Data Types
//!
//! Device-independent interface for IMU sensors to be consumed by the EKF subsystem.
//!
//! ## Requirements
//!
//! - FR-z1fdo: ImuSensor Trait Interface
//! - ADR-t5cq4: MPU-9250 I2C Driver Architecture
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::traits::{ImuSensor, ImuReading, ImuCalibration};
//!
//! async fn run_ekf<I: ImuSensor>(mut imu: I) {
//!     loop {
//!         let reading = imu.read_all().await?;
//!         // Process reading.gyro, reading.accel, reading.mag
//!     }
//! }
//! ```

use nalgebra::{Matrix3, Vector3};

/// IMU error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum ImuError {
    /// I2C/SPI communication failed
    I2cError,

    /// Data validation failed (e.g., stuck sensor)
    InvalidData,

    /// Driver not initialized
    NotInitialized,

    /// Sensor self-test failed
    SelfTestFailed,

    /// Magnetometer data not ready
    MagNotReady,
}

/// Combined IMU reading with calibration applied
///
/// All values are in SI units with NED body frame convention:
/// - X: Right (starboard)
/// - Y: Forward (bow)
/// - Z: Down
#[derive(Debug, Clone, Copy)]
pub struct ImuReading {
    /// Gyroscope: rad/s, body frame
    pub gyro: Vector3<f32>,

    /// Accelerometer: m/s², body frame (includes gravity)
    pub accel: Vector3<f32>,

    /// Magnetometer: µT, body frame
    pub mag: Vector3<f32>,

    /// Temperature: °C
    pub temperature: f32,

    /// Timestamp: microseconds since boot
    pub timestamp_us: u64,
}

impl Default for ImuReading {
    fn default() -> Self {
        Self {
            gyro: Vector3::zeros(),
            accel: Vector3::new(0.0, 0.0, 9.80665), // 1g down
            mag: Vector3::zeros(),
            temperature: 25.0,
            timestamp_us: 0,
        }
    }
}

/// Calibration data for IMU sensors
///
/// Calibration is applied as:
/// - Gyro: raw - gyro_bias
/// - Accel: (raw - accel_offset) * accel_scale
/// - Mag: mag_scale * (raw - mag_offset)
#[derive(Debug, Clone, Copy)]
pub struct ImuCalibration {
    /// Gyroscope bias: rad/s offset to subtract
    pub gyro_bias: Vector3<f32>,

    /// Accelerometer offset: m/s² to subtract
    pub accel_offset: Vector3<f32>,

    /// Accelerometer scale: per-axis scale factors (default 1.0)
    pub accel_scale: Vector3<f32>,

    /// Magnetometer hard iron offset: µT to subtract
    pub mag_offset: Vector3<f32>,

    /// Magnetometer soft iron matrix: 3x3 correction matrix (default identity)
    pub mag_scale: Matrix3<f32>,
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self {
            gyro_bias: Vector3::zeros(),
            accel_offset: Vector3::zeros(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            mag_offset: Vector3::zeros(),
            mag_scale: Matrix3::identity(),
        }
    }
}

impl ImuCalibration {
    /// Apply calibration to raw gyroscope reading
    pub fn apply_gyro(&self, raw: Vector3<f32>) -> Vector3<f32> {
        raw - self.gyro_bias
    }

    /// Apply calibration to raw accelerometer reading
    pub fn apply_accel(&self, raw: Vector3<f32>) -> Vector3<f32> {
        (raw - self.accel_offset).component_mul(&self.accel_scale)
    }

    /// Apply calibration to raw magnetometer reading
    pub fn apply_mag(&self, raw: Vector3<f32>) -> Vector3<f32> {
        self.mag_scale * (raw - self.mag_offset)
    }
}

/// Device-independent IMU interface for EKF
///
/// This trait abstracts IMU hardware specifics, enabling:
/// - Testability with mock implementations
/// - Sensor independence for EKF code
/// - Future sensor upgrades without EKF changes
#[allow(async_fn_in_trait)]
pub trait ImuSensor {
    /// Read all 9 axes: gyro (rad/s), accel (m/s²), mag (µT)
    ///
    /// Returns calibrated sensor data with timestamp.
    async fn read_all(&mut self) -> Result<ImuReading, ImuError>;

    /// Read gyroscope only (rad/s, body frame)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read accelerometer only (m/s², body frame)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read magnetometer only (µT, body frame)
    async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Apply calibration data
    ///
    /// Calibration is applied to all subsequent readings.
    fn set_calibration(&mut self, calibration: ImuCalibration);

    /// Get current calibration data
    fn calibration(&self) -> &ImuCalibration;

    /// Get sensor health status
    ///
    /// Returns false if sensor has consecutive read errors or
    /// data appears invalid (e.g., stuck values).
    fn is_healthy(&self) -> bool;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_reading_default() {
        let reading = ImuReading::default();
        assert_eq!(reading.gyro, Vector3::zeros());
        assert!((reading.accel.z - 9.80665).abs() < 0.001);
        assert_eq!(reading.temperature, 25.0);
    }

    #[test]
    fn test_imu_calibration_default() {
        let cal = ImuCalibration::default();
        assert_eq!(cal.gyro_bias, Vector3::zeros());
        assert_eq!(cal.accel_scale, Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(cal.mag_scale, Matrix3::identity());
    }

    #[test]
    fn test_apply_gyro_calibration() {
        let cal = ImuCalibration {
            gyro_bias: Vector3::new(0.01, -0.02, 0.005),
            ..Default::default()
        };
        let raw = Vector3::new(0.1, 0.2, 0.3);
        let calibrated = cal.apply_gyro(raw);

        assert!((calibrated.x - 0.09).abs() < 1e-6);
        assert!((calibrated.y - 0.22).abs() < 1e-6);
        assert!((calibrated.z - 0.295).abs() < 1e-6);
    }

    #[test]
    fn test_apply_accel_calibration() {
        let cal = ImuCalibration {
            accel_offset: Vector3::new(0.1, 0.2, 0.3),
            accel_scale: Vector3::new(1.0, 1.01, 0.99),
            ..Default::default()
        };
        let raw = Vector3::new(0.0, 0.0, 9.8);
        let calibrated = cal.apply_accel(raw);

        assert!((calibrated.x - (-0.1)).abs() < 1e-6);
        assert!((calibrated.y - (-0.202)).abs() < 1e-6);
        assert!((calibrated.z - (9.5 * 0.99)).abs() < 1e-4);
    }

    #[test]
    fn test_apply_mag_calibration() {
        let cal = ImuCalibration {
            mag_offset: Vector3::new(5.0, 10.0, -5.0),
            mag_scale: Matrix3::identity(),
            ..Default::default()
        };
        let raw = Vector3::new(25.0, 5.0, -40.0);
        let calibrated = cal.apply_mag(raw);

        assert!((calibrated.x - 20.0).abs() < 1e-6);
        assert!((calibrated.y - (-5.0)).abs() < 1e-6);
        assert!((calibrated.z - (-35.0)).abs() < 1e-6);
    }
}
