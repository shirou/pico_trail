//! AHRS (Attitude and Heading Reference System)
//!
//! Implements Direction Cosine Matrix (DCM) algorithm for attitude estimation
//! by fusing gyroscope, accelerometer, and magnetometer data.
//!
//! ## Requirements
//!
//! - FR-eyuh8: AHRS Attitude Estimation
//! - ADR-6twis: DCM Algorithm Selection
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::subsystems::ahrs::{Dcm, DcmConfig};
//!
//! let config = DcmConfig::default();
//! let mut dcm = Dcm::new(config);
//!
//! loop {
//!     // Read sensors (gyro in rad/s, accel in m/s²)
//!     let gyro = Vector3::new(gx, gy, gz);
//!     let accel = Vector3::new(ax, ay, az);
//!     let dt = 0.01; // 100Hz update rate
//!
//!     // Update DCM
//!     dcm.update(gyro, accel, dt);
//!
//!     // Get attitude
//!     let (roll, pitch, yaw) = dcm.get_euler_angles();
//! }
//! ```

pub mod calibration;
pub mod dcm;
pub mod state;
pub mod task;

pub use calibration::{estimate_gyro_bias, CalibrationData};
pub use dcm::{Dcm, DcmConfig};
pub use state::{AttitudeQuality, AttitudeState, SharedAttitudeState};
pub use task::{run_ahrs_task, AhrsTaskConfig, ImuData};
