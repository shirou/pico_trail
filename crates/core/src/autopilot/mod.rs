//! Autopilot types and logic
//!
//! This module contains core autopilot types shared between firmware and SITL:
//! - `battery`: Battery failsafe checker (voltage-based failsafe with hysteresis)
//! - `gcs_failsafe`: GCS communication lost failsafe checker (two-stage timeout)
//! - `mavlink_runner`: MAVLink loop failsafe action mapping
//! - `state`: System state, arming, flight modes, battery, attitude, GPS
//! - `vehicle`: Vehicle type trait and implementations (GroundRover, SurfaceBoat)

pub mod battery;
pub mod gcs_failsafe;
pub mod mavlink_runner;
pub mod state;
pub mod vehicle;
