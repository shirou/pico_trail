//! AHRS (Attitude and Heading Reference System) types and utilities
//!
//! This module provides platform-agnostic AHRS calibration types and algorithms.
//! Platform-specific implementations (parameter loading, async operations) are in the firmware crate.

pub mod calibration;
pub mod dcm;
pub mod traits;

pub use calibration::{
    estimate_gyro_bias, CalibrationData, PARAM_ACCEL_OFFSET_X, PARAM_ACCEL_OFFSET_Y,
    PARAM_ACCEL_OFFSET_Z, PARAM_ACCEL_SCALE_X, PARAM_ACCEL_SCALE_Y, PARAM_ACCEL_SCALE_Z,
    PARAM_GYRO_BIAS_X, PARAM_GYRO_BIAS_Y, PARAM_GYRO_BIAS_Z, PARAM_MAG_OFFSET_X,
    PARAM_MAG_OFFSET_Y, PARAM_MAG_OFFSET_Z, PARAM_MAG_SCALE_X, PARAM_MAG_SCALE_Y,
    PARAM_MAG_SCALE_Z,
};
pub use dcm::{Dcm, DcmConfig};
pub use traits::{
    euler_to_quaternion_zyx, quaternion_to_euler_zyx, Ahrs, AhrsError, AhrsState, AhrsType,
};
