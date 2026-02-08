//! Autopilot types and logic
//!
//! This module contains core autopilot types shared between firmware and SITL:
//! - `state`: System state, arming, flight modes, battery, attitude, GPS
//! - `vehicle`: Vehicle type trait and implementations (GroundRover, SurfaceBoat)

pub mod state;
pub mod vehicle;
