//! Autopilot types and logic
//!
//! This module contains core autopilot types shared between firmware and SITL:
//! - `battery`: Battery failsafe checker (voltage-based failsafe with hysteresis)
//! - `mavlink_runner`: MAVLink loop battery failsafe action mapping
//! - `state`: System state, arming, flight modes, battery, attitude, GPS
//! - `vehicle`: Vehicle type trait and implementations (GroundRover, SurfaceBoat)

pub mod battery;
pub mod mavlink_runner;
pub mod state;
pub mod vehicle;
