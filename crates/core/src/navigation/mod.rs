//! Navigation types and utilities
//!
//! This module contains core types and geographic algorithms for the navigation subsystem.

pub mod controller;
pub mod geo;
pub mod heading;
pub mod path_recorder;
mod types;

pub use geo::{
    calculate_bearing, calculate_distance, haversine_distance_bearing, normalize_angle,
    offset_position, wrap_180, wrap_360,
};
pub use types::{NavigationOutput, PositionTarget, SimpleNavConfig};
