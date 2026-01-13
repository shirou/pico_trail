//! AHRS (Attitude and Heading Reference System)
//!
//! Provides unified attitude estimation through the `Ahrs` trait, supporting:
//! - **External AHRS**: Sensors with on-chip fusion (BNO086, BNO085)
//! - **Software AHRS**: DCM or EKF processing raw IMU data
//!
//! ## Requirements
//!
//! - FR-eyuh8: AHRS Attitude Estimation
//! - ADR-nzvfy: AHRS Abstraction Architecture
//! - ADR-ymkzt: EKF AHRS Implementation
//!
//! ## Architecture (ADR-nzvfy)
//!
//! ```text
//!                        ┌──────────────────┐
//!                        │  Flight Control  │
//!                        └────────▲─────────┘
//!                                 │
//!                        ┌────────┴─────────┐
//!                        │    Ahrs Trait    │
//!                        └────────▲─────────┘
//!                                 │
//!          ┌──────────────────────┼──────────────────────┐
//!          │                      │                      │
//! ┌────────┴────────┐    ┌────────┴────────┐    ┌────────┴────────┐
//! │  SoftwareAhrs   │    │  ExternalAhrs   │    │  ExternalAhrs   │
//! │  (DCM/EKF)      │    │  (BNO086)       │    │  (Future)       │
//! └─────────────────┘    └─────────────────┘    └─────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::subsystems::ahrs::{Ahrs, AhrsState};
//!
//! async fn attitude_loop<A: Ahrs>(ahrs: &mut A) {
//!     loop {
//!         let state = ahrs.get_attitude().await.unwrap();
//!         // Use state.quaternion, state.roll, state.pitch, state.yaw
//!     }
//! }
//! ```

pub mod calibration;
pub mod dcm;
pub mod external;
pub mod task;
pub mod traits;

// Core exports
pub use calibration::{estimate_gyro_bias, CalibrationData};
pub use dcm::{Dcm, DcmConfig};
pub use task::{run_ahrs_task, AhrsTaskConfig, ImuData, ImuTaskConfig};

// AHRS abstraction exports (ADR-nzvfy) - unified interface
pub use traits::{
    euler_to_quaternion_zyx, quaternion_to_euler_zyx, Ahrs, AhrsError, AhrsState, AhrsType,
    QuaternionReadingExt, SharedAhrsState,
};

// External AHRS implementations
pub use external::Bno086ExternalAhrs;

#[cfg(feature = "embassy")]
pub use task::run_imu_task;

/// Global AHRS state shared between AHRS task and other subsystems
///
/// This static provides thread-safe access to the current attitude estimate.
/// The AHRS task writes to this state, and other subsystems (navigation,
/// MAVLink telemetry) read from it.
///
/// # Usage
///
/// ```ignore
/// use pico_trail::subsystems::ahrs::AHRS_STATE;
///
/// // In AHRS task: write attitude
/// AHRS_STATE.update_euler(roll, pitch, yaw, timestamp_us);
/// AHRS_STATE.set_healthy(true);
///
/// // In MAVLink task: read attitude
/// system_state.update_attitude(&AHRS_STATE);
/// ```
pub static AHRS_STATE: SharedAhrsState = SharedAhrsState::new();
