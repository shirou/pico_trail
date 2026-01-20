//! External AHRS implementations
//!
//! This module contains wrappers for sensors with on-chip fusion
//! that provide direct quaternion output.
//!
//! ## Available Implementations
//!
//! - `Bno086ExternalAhrs` - BNO086 9-axis IMU with integrated sensor fusion
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::subsystems::ahrs::external::Bno086ExternalAhrs;
//! use pico_trail::subsystems::ahrs::Ahrs;
//!
//! let ahrs = Bno086ExternalAhrs::new(driver);
//! let state = ahrs.get_attitude().await?;
//! ```

pub mod bno086;
pub use bno086::Bno086ExternalAhrs;
