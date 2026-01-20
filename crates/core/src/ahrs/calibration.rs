//! IMU calibration data structures and application
//!
//! Provides hard/soft iron calibration for magnetometer and offset/scale
//! calibration for accelerometer. Gyro bias is handled separately in DCM.
//!
//! Platform-specific parameter loading/saving is in the firmware crate.

use nalgebra::Vector3;

/// Parameter names for calibration data
pub const PARAM_ACCEL_OFFSET_X: &str = "AHRS_ACC_OFS_X";
pub const PARAM_ACCEL_OFFSET_Y: &str = "AHRS_ACC_OFS_Y";
pub const PARAM_ACCEL_OFFSET_Z: &str = "AHRS_ACC_OFS_Z";
pub const PARAM_ACCEL_SCALE_X: &str = "AHRS_ACC_SCL_X";
pub const PARAM_ACCEL_SCALE_Y: &str = "AHRS_ACC_SCL_Y";
pub const PARAM_ACCEL_SCALE_Z: &str = "AHRS_ACC_SCL_Z";

pub const PARAM_MAG_OFFSET_X: &str = "AHRS_MAG_OFS_X";
pub const PARAM_MAG_OFFSET_Y: &str = "AHRS_MAG_OFS_Y";
pub const PARAM_MAG_OFFSET_Z: &str = "AHRS_MAG_OFS_Z";
pub const PARAM_MAG_SCALE_X: &str = "AHRS_MAG_SCL_X";
pub const PARAM_MAG_SCALE_Y: &str = "AHRS_MAG_SCL_Y";
pub const PARAM_MAG_SCALE_Z: &str = "AHRS_MAG_SCL_Z";

pub const PARAM_GYRO_BIAS_X: &str = "AHRS_GYR_OFS_X";
pub const PARAM_GYRO_BIAS_Y: &str = "AHRS_GYR_OFS_Y";
pub const PARAM_GYRO_BIAS_Z: &str = "AHRS_GYR_OFS_Z";

/// Calibration data for all IMU sensors
///
/// Store calibration parameters obtained from calibration procedures:
/// - Accelerometer: 6-position static calibration
/// - Magnetometer: Sphere fitting or ellipsoid fitting
/// - Gyroscope: Static bias estimation (handled in DCM)
#[derive(Debug, Clone, Copy)]
pub struct CalibrationData {
    /// Accelerometer offset (m/s²)
    /// Subtracted from raw readings before scaling
    pub accel_offset: Vector3<f32>,

    /// Accelerometer scale factors (dimensionless)
    /// Applied after offset removal (typically near 1.0)
    pub accel_scale: Vector3<f32>,

    /// Magnetometer hard iron offset (µT)
    /// Subtracted from raw readings (compensates for constant magnetic fields)
    pub mag_offset: Vector3<f32>,

    /// Magnetometer soft iron scale matrix diagonal (µT)
    /// For simplicity, we use diagonal scaling only (full matrix calibration is Phase 3 enhancement)
    pub mag_scale: Vector3<f32>,

    /// Gyroscope bias (rad/s)
    /// Subtracted from raw gyro readings during DCM update
    pub gyro_bias: Vector3<f32>,
}

impl Default for CalibrationData {
    /// Create uncalibrated (identity) calibration
    ///
    /// This is a safe fallback but will result in poor attitude accuracy.
    /// Users should perform calibration procedure before flight.
    fn default() -> Self {
        Self {
            accel_offset: Vector3::zeros(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            mag_offset: Vector3::zeros(),
            mag_scale: Vector3::new(1.0, 1.0, 1.0),
            gyro_bias: Vector3::zeros(),
        }
    }
}

impl CalibrationData {
    /// Apply accelerometer calibration to raw reading
    ///
    /// # Arguments
    ///
    /// * `raw` - Raw accelerometer reading (m/s²)
    ///
    /// # Returns
    ///
    /// Calibrated accelerometer reading (m/s²)
    ///
    /// # Formula
    ///
    /// ```text
    /// calibrated = (raw - offset) .* scale
    /// ```
    /// where `.*` is element-wise multiplication
    pub fn apply_accel_calibration(&self, raw: Vector3<f32>) -> Vector3<f32> {
        Vector3::new(
            (raw.x - self.accel_offset.x) * self.accel_scale.x,
            (raw.y - self.accel_offset.y) * self.accel_scale.y,
            (raw.z - self.accel_offset.z) * self.accel_scale.z,
        )
    }

    /// Apply magnetometer calibration to raw reading
    ///
    /// # Arguments
    ///
    /// * `raw` - Raw magnetometer reading (µT)
    ///
    /// # Returns
    ///
    /// Calibrated magnetometer reading (µT)
    ///
    /// # Formula
    ///
    /// ```text
    /// calibrated = (raw - hard_iron_offset) .* soft_iron_scale
    /// ```
    pub fn apply_mag_calibration(&self, raw: Vector3<f32>) -> Vector3<f32> {
        Vector3::new(
            (raw.x - self.mag_offset.x) * self.mag_scale.x,
            (raw.y - self.mag_offset.y) * self.mag_scale.y,
            (raw.z - self.mag_offset.z) * self.mag_scale.z,
        )
    }

    /// Check if calibration is valid (non-default)
    ///
    /// Returns `false` if calibration appears to be default/uncalibrated.
    /// This is a heuristic check, not a guarantee of calibration quality.
    pub fn is_calibrated(&self) -> bool {
        // Check if any calibration parameters differ from default
        let accel_offset_nonzero = self.accel_offset.norm() > 0.01;
        let mag_offset_nonzero = self.mag_offset.norm() > 0.01;
        let gyro_bias_nonzero = self.gyro_bias.norm() > 0.001;

        // At least one sensor should have non-zero calibration
        accel_offset_nonzero || mag_offset_nonzero || gyro_bias_nonzero
    }
}

/// Estimate gyro bias from static IMU samples
///
/// Accumulates gyro samples for a specified duration and computes mean bias.
/// The vehicle must be stationary during this process.
///
/// # Arguments
///
/// * `samples` - Slice of gyro samples (rad/s)
///
/// # Returns
///
/// Estimated gyro bias (rad/s)
pub fn estimate_gyro_bias(samples: &[Vector3<f32>]) -> Vector3<f32> {
    if samples.is_empty() {
        return Vector3::zeros();
    }

    let sum = samples
        .iter()
        .fold(Vector3::zeros(), |acc, sample| acc + sample);
    sum / (samples.len() as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-6;

    #[test]
    fn test_default_calibration() {
        let cal = CalibrationData::default();

        assert_eq!(cal.accel_offset, Vector3::zeros());
        assert_eq!(cal.accel_scale, Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(cal.mag_offset, Vector3::zeros());
        assert_eq!(cal.mag_scale, Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(cal.gyro_bias, Vector3::zeros());
    }

    #[test]
    fn test_apply_accel_calibration() {
        let cal = CalibrationData {
            accel_offset: Vector3::new(0.1, -0.05, 0.02),
            accel_scale: Vector3::new(1.01, 0.99, 1.02),
            ..Default::default()
        };

        let raw = Vector3::new(1.0, 2.0, 9.81);
        let calibrated = cal.apply_accel_calibration(raw);

        // Expected: (1.0 - 0.1) * 1.01, (2.0 - (-0.05)) * 0.99, (9.81 - 0.02) * 1.02
        let expected = Vector3::new(0.909, 2.0295, 9.9858);

        assert!((calibrated.x - expected.x).abs() < EPSILON);
        assert!((calibrated.y - expected.y).abs() < EPSILON);
        assert!((calibrated.z - expected.z).abs() < EPSILON);
    }

    #[test]
    fn test_apply_mag_calibration() {
        let cal = CalibrationData {
            mag_offset: Vector3::new(5.0, -3.0, 2.0),
            mag_scale: Vector3::new(1.1, 0.9, 1.05),
            ..Default::default()
        };

        let raw = Vector3::new(25.0, 17.0, 42.0);
        let calibrated = cal.apply_mag_calibration(raw);

        // Expected: (25 - 5) * 1.1, (17 - (-3)) * 0.9, (42 - 2) * 1.05
        let expected = Vector3::new(22.0, 18.0, 42.0);

        assert!((calibrated.x - expected.x).abs() < EPSILON);
        assert!((calibrated.y - expected.y).abs() < EPSILON);
        assert!((calibrated.z - expected.z).abs() < EPSILON);
    }

    #[test]
    fn test_is_calibrated() {
        let uncalibrated = CalibrationData::default();
        assert!(!uncalibrated.is_calibrated());

        let calibrated_accel = CalibrationData {
            accel_offset: Vector3::new(0.1, 0.1, 0.1),
            ..Default::default()
        };
        assert!(calibrated_accel.is_calibrated());

        let calibrated_mag = CalibrationData {
            mag_offset: Vector3::new(5.0, 3.0, 2.0),
            ..Default::default()
        };
        assert!(calibrated_mag.is_calibrated());

        let calibrated_gyro = CalibrationData {
            gyro_bias: Vector3::new(0.01, -0.005, 0.002),
            ..Default::default()
        };
        assert!(calibrated_gyro.is_calibrated());
    }

    #[test]
    fn test_estimate_gyro_bias() {
        let samples = [
            Vector3::new(0.01, -0.005, 0.002),
            Vector3::new(0.011, -0.006, 0.001),
            Vector3::new(0.009, -0.004, 0.003),
            Vector3::new(0.010, -0.005, 0.002),
        ];

        let bias = estimate_gyro_bias(&samples);

        // Expected mean: (0.01 + 0.011 + 0.009 + 0.010) / 4 = 0.01
        let expected = Vector3::new(0.01, -0.005, 0.002);

        assert!((bias.x - expected.x).abs() < EPSILON);
        assert!((bias.y - expected.y).abs() < EPSILON);
        assert!((bias.z - expected.z).abs() < EPSILON);
    }

    #[test]
    fn test_estimate_gyro_bias_empty() {
        let samples: [Vector3<f32>; 0] = [];
        let bias = estimate_gyro_bias(&samples);
        assert_eq!(bias, Vector3::zeros());
    }
}
