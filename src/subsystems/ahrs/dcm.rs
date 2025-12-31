//! Direction Cosine Matrix (DCM) algorithm for attitude estimation
//!
//! Implements sensor fusion of gyroscope, accelerometer, and magnetometer
//! using a rotation matrix representation with PI controller corrections.
//!
//! ## Algorithm Overview
//!
//! 1. **Gyro Integration**: Integrate angular rates to update DCM matrix
//! 2. **Normalization**: Apply Gram-Schmidt orthonormalization to prevent drift
//! 3. **Accel Correction**: Use accelerometer as gravity reference for roll/pitch
//! 4. **Mag Correction**: Use magnetometer for heading correction
//!
//! ## References
//!
//! - ADR-6twis: AHRS Algorithm Selection
//! - ArduPilot DCM: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
//! - DCM Tutorial: <http://www.starlino.com/dcm_tutorial.html>

// micromath::F32Ext provides atan2 and asin methods for f32 in no_std environments
#[allow(unused_imports)]
use micromath::F32Ext;
use nalgebra::{Matrix3, Vector3};

/// Z-axis unit vector representing gravity direction in NED frame.
/// Defined as a constant to avoid repeated allocation in the 100Hz update loop.
const Z_UNIT: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

/// Direction Cosine Matrix state
///
/// Represents the rotation from body frame to earth frame using a 3x3 matrix.
/// Includes gyro bias estimation and PI correction terms for sensor fusion.
pub struct DcmState {
    /// 3x3 rotation matrix (body to earth frame)
    /// The DCM matrix represents the rotation that transforms body frame coordinates
    /// to earth frame coordinates. Each column represents a body axis in earth frame.
    dcm_matrix: Matrix3<f32>,

    /// Estimated gyro bias (rad/s)
    /// Gyro bias is estimated during initialization and updated via integral correction
    gyro_bias: Vector3<f32>,

    /// Proportional correction term (rad/s)
    /// Computed from accelerometer and magnetometer error vectors
    omega_p: Vector3<f32>,

    /// Integral correction term (rad/s)
    /// Accumulated over time to remove steady-state gyro drift
    omega_i: Vector3<f32>,
}

/// DCM algorithm configuration
///
/// Tuning parameters for PI controller gains. Higher gains increase
/// convergence speed but may cause oscillation or noise sensitivity.
#[derive(Debug, Clone, Copy)]
pub struct DcmConfig {
    /// P gain for roll/pitch correction (default: 0.2)
    /// Higher values increase responsiveness to accelerometer
    pub kp_roll_pitch: f32,

    /// I gain for roll/pitch correction (default: 0.0005)
    /// Higher values reduce long-term drift but may cause wind-up
    pub ki_roll_pitch: f32,

    /// P gain for yaw correction (default: 1.0)
    /// Higher values increase responsiveness to magnetometer
    pub kp_yaw: f32,

    /// I gain for yaw correction (default: 0.00005)
    /// Higher values reduce long-term heading drift
    pub ki_yaw: f32,
}

impl Default for DcmConfig {
    fn default() -> Self {
        // Default gains based on ArduPilot DCM tuning
        // These work well for 100Hz update rate with typical IMU noise levels
        Self {
            kp_roll_pitch: 0.2,
            ki_roll_pitch: 0.0005,
            kp_yaw: 1.0,
            ki_yaw: 0.00005,
        }
    }
}

/// Direction Cosine Matrix attitude estimator
pub struct Dcm {
    /// Current DCM state
    state: DcmState,

    /// Configuration parameters
    config: DcmConfig,
}

impl Dcm {
    /// Create a new DCM estimator with given configuration
    ///
    /// Initializes the DCM matrix to identity (no rotation), which assumes
    /// the vehicle starts level. For accurate initialization, consider
    /// using accelerometer data to compute initial roll/pitch.
    pub fn new(config: DcmConfig) -> Self {
        Self {
            state: DcmState {
                // Initialize with identity matrix (vehicle assumed level)
                dcm_matrix: Matrix3::identity(),
                gyro_bias: Vector3::zeros(),
                omega_p: Vector3::zeros(),
                omega_i: Vector3::zeros(),
            },
            config,
        }
    }

    /// Update DCM using gyroscope and accelerometer measurements
    ///
    /// This is the main update cycle called at 100Hz. It performs:
    /// 1. Gyro integration with bias removal and PI correction
    /// 2. Matrix normalization to prevent numerical drift
    /// 3. Accelerometer error computation for roll/pitch correction
    /// 4. PI controller update
    ///
    /// # Arguments
    ///
    /// * `gyro` - Angular rates in rad/s (body frame)
    /// * `accel` - Acceleration in m/s² (body frame, includes gravity)
    /// * `dt` - Time step in seconds (typically 0.01 for 100Hz)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let gyro = Vector3::new(0.01, -0.02, 0.0);  // rad/s
    /// let accel = Vector3::new(0.0, 0.0, 9.81);   // m/s² (level, gravity down)
    /// dcm.update(gyro, accel, 0.01);  // 100Hz update
    /// ```
    pub fn update(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        // 1. Integrate gyro to update DCM matrix
        let omega = gyro - self.state.gyro_bias + self.state.omega_p + self.state.omega_i;
        self.state.dcm_matrix *= rotation_matrix(omega, dt);

        // 2. Normalize DCM matrix (prevent drift)
        self.state.dcm_matrix = orthonormalize(self.state.dcm_matrix);

        // 3. Error correction from accelerometer
        // Accelerometer measures gravity vector in body frame
        // DCM * Z_UNIT gives expected gravity direction in body frame
        let accel_normalized = accel.normalize();
        let gravity_body_expected = self.state.dcm_matrix * Z_UNIT;
        let error = accel_normalized.cross(&gravity_body_expected);

        // 4. Proportional and integral corrections
        self.state.omega_p = error * self.config.kp_roll_pitch;
        self.state.omega_i += error * self.config.ki_roll_pitch * dt;
    }

    /// Update heading using magnetometer measurement
    ///
    /// Should be called at lower rate than `update()` (e.g., 10Hz vs 100Hz)
    /// since magnetometer is more susceptible to interference.
    ///
    /// # Arguments
    ///
    /// * `mag` - Magnetic field in µT (body frame, calibrated)
    /// * `dt` - Time step since last magnetometer update
    ///
    /// # Note
    ///
    /// Magnetometer must be calibrated (hard/soft iron correction) before use.
    /// Uncalibrated magnetometer is worse than no magnetometer.
    pub fn update_with_mag(&mut self, mag: Vector3<f32>, dt: f32) {
        // Transform magnetometer reading to earth frame
        let mag_earth = self.state.dcm_matrix.transpose() * mag;

        // Expected heading from magnetometer (atan2 in earth frame)
        let mag_heading = mag_earth.y.atan2(mag_earth.x);

        // Current heading from DCM
        let (_, _, yaw) = self.get_euler_angles();

        // Heading error (normalize to -π to π)
        let heading_error = normalize_angle(mag_heading - yaw);

        // Apply heading correction to yaw axis only
        self.state.omega_p.z += heading_error * self.config.kp_yaw;
        self.state.omega_i.z += heading_error * self.config.ki_yaw * dt;
    }

    /// Get current attitude as Euler angles
    ///
    /// Returns (roll, pitch, yaw) in radians.
    ///
    /// # Coordinate System
    ///
    /// - Roll: Rotation around X-axis (positive = right wing down)
    /// - Pitch: Rotation around Y-axis (positive = nose up)
    /// - Yaw: Rotation around Z-axis (positive = clockwise from above, 0 = north)
    ///
    /// # Note
    ///
    /// Euler angles have gimbal lock at ±90° pitch. For applications requiring
    /// full 3D orientation, use the DCM matrix directly or convert to quaternion.
    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
        // Extract Euler angles from DCM matrix using ZYX convention
        // DCM matrix elements (row, col):
        //   [R00, R01, R02]
        //   [R10, R11, R12]
        //   [R20, R21, R22]

        let roll = self.state.dcm_matrix[(2, 1)].atan2(self.state.dcm_matrix[(2, 2)]);
        let pitch = (-self.state.dcm_matrix[(2, 0)]).asin();
        let yaw = self.state.dcm_matrix[(1, 0)].atan2(self.state.dcm_matrix[(0, 0)]);

        (roll, pitch, yaw)
    }

    /// Get DCM matrix (for advanced use)
    ///
    /// Returns the raw rotation matrix representing body-to-earth transformation.
    pub fn dcm_matrix(&self) -> &Matrix3<f32> {
        &self.state.dcm_matrix
    }

    /// Get estimated gyro bias
    ///
    /// Returns the current gyro bias estimate in rad/s
    pub fn gyro_bias(&self) -> Vector3<f32> {
        self.state.gyro_bias
    }

    /// Set gyro bias (typically from calibration)
    ///
    /// Used during initialization to set pre-computed gyro bias
    pub fn set_gyro_bias(&mut self, bias: Vector3<f32>) {
        self.state.gyro_bias = bias;
    }
}

/// Compute rotation matrix from angular velocity vector
///
/// Uses small angle approximation for efficiency. Accurate for typical
/// IMU update rates (100Hz) and angular velocities (< 5 rad/s).
///
/// For omega = [wx, wy, wz] * dt, the rotation matrix is approximately:
/// ```text
/// R ≈ I + [omega]× where [omega]× is the skew-symmetric matrix:
///     [ 0   -wz   wy ]
///     [ wz   0   -wx ]
///     [-wy   wx   0  ]
/// ```
fn rotation_matrix(omega: Vector3<f32>, dt: f32) -> Matrix3<f32> {
    let wx = omega.x * dt;
    let wy = omega.y * dt;
    let wz = omega.z * dt;

    // Small angle approximation: R ≈ I + [omega × dt]
    Matrix3::new(1.0, -wz, wy, wz, 1.0, -wx, -wy, wx, 1.0)
}

/// Orthonormalize DCM matrix using Gram-Schmidt process
///
/// Ensures the matrix remains a valid rotation matrix (orthonormal columns/rows)
/// despite numerical drift from repeated floating-point operations.
///
/// # Algorithm
///
/// 1. Compute error: dot product of first two rows (should be 0)
/// 2. Correct both rows to be orthogonal
/// 3. Compute third row as cross product (ensures right-handed frame)
/// 4. Normalize all rows
///
/// # Determinant Check
///
/// The determinant should be +1 for a valid rotation matrix.
/// If |det - 1| > 0.1, numerical instability is severe and DCM should be reset.
fn orthonormalize(mut dcm: Matrix3<f32>) -> Matrix3<f32> {
    // Get row vectors
    let mut x = dcm.row(0).transpose();
    let mut y = dcm.row(1).transpose();

    // Compute orthogonality error
    let error = x.dot(&y);

    // Correct both rows to be orthogonal
    let x_orth = x - y * (error / 2.0);
    let y_orth = y - x * (error / 2.0);

    // Normalize corrected rows
    x = x_orth.normalize();
    y = y_orth.normalize();

    // Third row is cross product (ensures right-handed coordinate system)
    let z = x.cross(&y);

    // Reconstruct matrix with orthonormal rows
    dcm.set_row(0, &x.transpose());
    dcm.set_row(1, &y.transpose());
    dcm.set_row(2, &z.transpose());

    // Determinant check - debug builds only to avoid overhead in release
    #[cfg(debug_assertions)]
    {
        let det = dcm.determinant();
        if (det - 1.0).abs() > 0.1 {
            crate::log_warn!("DCM determinant drift: {}", det);
        }
    }

    dcm
}

/// Normalize angle to range [-π, π]
fn normalize_angle(angle: f32) -> f32 {
    let pi = core::f32::consts::PI;
    let mut a = angle;

    while a > pi {
        a -= 2.0 * pi;
    }
    while a < -pi {
        a += 2.0 * pi;
    }

    a
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f32 = 1e-4;

    #[test]
    fn test_dcm_initialization() {
        let config = DcmConfig::default();
        let dcm = Dcm::new(config);

        // Should start with identity matrix (no rotation)
        let expected = Matrix3::identity();
        assert_matrix_near(dcm.dcm_matrix(), &expected, EPSILON);

        // Should start with zero bias
        assert_eq!(dcm.gyro_bias(), Vector3::zeros());
    }

    #[test]
    fn test_rotation_matrix_z_axis() {
        // Rotate 10° around Z-axis (small angle approximation is accurate for small angles)
        let omega = Vector3::new(0.0, 0.0, 0.1); // 0.1 rad = ~5.7°
        let dt = 1.0;

        let rot = rotation_matrix(omega, dt);

        // For small angles, rotation matrix is approximately I + [omega]×
        // which is what our function implements
        let expected = Matrix3::new(1.0, -0.1, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0, 1.0);

        assert_matrix_near(&rot, &expected, EPSILON);
    }

    #[test]
    fn test_orthonormalize_preserves_orthogonality() {
        // Start with slightly non-orthogonal matrix
        let mut dcm = Matrix3::new(1.01, -0.01, 0.0, 0.01, 0.99, 0.0, 0.0, 0.0, 1.0);

        dcm = orthonormalize(dcm);

        // Check orthogonality: row0 · row1 should be 0
        let row0 = dcm.row(0).transpose();
        let row1 = dcm.row(1).transpose();
        assert!((row0.dot(&row1)).abs() < EPSILON);

        // Check normalization: each row should have magnitude 1
        assert!((row0.norm() - 1.0).abs() < EPSILON);
        assert!((row1.norm() - 1.0).abs() < EPSILON);

        // Check determinant is 1
        assert!((dcm.determinant() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_euler_angle_extraction() {
        let config = DcmConfig::default();
        let mut dcm = Dcm::new(config);

        // Set known DCM matrix for 30° roll, 0° pitch, 0° yaw
        // DCM for roll-only rotation (body frame tilted right):
        // Rx(roll) = [1    0         0     ]
        //            [0  cos(r)  -sin(r)   ]
        //            [0  sin(r)   cos(r)   ]
        // But DCM is transpose for body-to-earth, so:
        // DCM = [1    0         0     ]
        //       [0  cos(r)   sin(r)   ]
        //       [0 -sin(r)   cos(r)   ]
        let roll = 30.0_f32.to_radians();
        let cos_r = roll.cos();
        let sin_r = roll.sin();

        dcm.state.dcm_matrix = Matrix3::new(1.0, 0.0, 0.0, 0.0, cos_r, -sin_r, 0.0, sin_r, cos_r);

        let (r, p, y) = dcm.get_euler_angles();

        assert!(
            (r - roll).abs() < EPSILON,
            "Roll mismatch: expected {}, got {}",
            roll,
            r
        );
        assert!(p.abs() < EPSILON, "Pitch should be 0, got {}", p);
        assert!(y.abs() < EPSILON, "Yaw should be 0, got {}", y);
    }

    #[test]
    fn test_normalize_angle() {
        use core::f32::consts::PI;

        assert!((normalize_angle(0.0)).abs() < EPSILON);
        assert!((normalize_angle(PI) - PI).abs() < EPSILON);
        assert!((normalize_angle(-PI) + PI).abs() < EPSILON);
        assert!((normalize_angle(3.0 * PI) - PI).abs() < EPSILON);
        assert!((normalize_angle(-3.0 * PI) + PI).abs() < EPSILON);
    }

    // Helper function to assert matrix equality with tolerance
    fn assert_matrix_near(actual: &Matrix3<f32>, expected: &Matrix3<f32>, epsilon: f32) {
        for i in 0..3 {
            for j in 0..3 {
                let diff = (actual[(i, j)] - expected[(i, j)]).abs();
                assert!(
                    diff < epsilon,
                    "Matrix element ({}, {}) differs: {} vs {} (diff: {})",
                    i,
                    j,
                    actual[(i, j)],
                    expected[(i, j)],
                    diff
                );
            }
        }
    }
}
