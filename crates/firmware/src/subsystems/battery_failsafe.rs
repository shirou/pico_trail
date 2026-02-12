//! Battery Failsafe Checker (re-exported from core)
//!
//! This module re-exports the battery failsafe implementation from `pico_trail_core`.
//! All logic has been moved to the core crate for sharing between firmware and SITL.

pub use pico_trail_core::autopilot::battery::{
    BatteryFailsafeChecker, BatteryFailsafeConfig, BatteryFailsafeEvent, BatteryFailsafeLevel,
};
