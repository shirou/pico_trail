//! Direction Cosine Matrix (DCM) algorithm for attitude estimation
//!
//! Re-exports from `pico_trail_core::ahrs::dcm`. The platform-independent
//! algorithm lives in the core crate.

pub use pico_trail_core::ahrs::dcm::{Dcm, DcmConfig};
