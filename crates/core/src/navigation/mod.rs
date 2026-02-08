//! Navigation types and utilities
//!
//! This module contains core types and geographic algorithms for the navigation subsystem.

pub mod controller;
pub mod geo;
pub mod heading;
pub mod heading_filter;
pub mod path_recorder;
mod types;

pub use geo::{
    calculate_bearing, calculate_distance, haversine_distance_bearing, normalize_angle,
    offset_position, wrap_180, wrap_360,
};
pub use types::{GpsFixType, GpsPosition, NavigationOutput, PositionTarget, SimpleNavConfig};

#[cfg(feature = "embassy")]
use crate::traits::sync::EmbassyState;

/// Global reposition target for MAV_CMD_DO_REPOSITION
///
/// Set by CommandHandler when receiving MAV_CMD_DO_REPOSITION.
/// Read and cleared by navigation_task to update NAV_TARGET.
#[cfg(feature = "embassy")]
pub static REPOSITION_TARGET: EmbassyState<Option<PositionTarget>> = EmbassyState::new(None);

/// Set reposition target from command handler (synchronous)
///
/// Called by CommandHandler when receiving MAV_CMD_DO_REPOSITION.
/// The navigation_task will pick this up and update NAV_TARGET.
#[cfg(feature = "embassy")]
pub fn set_reposition_target(target: PositionTarget) {
    use crate::traits::sync::SharedState;
    REPOSITION_TARGET.with_mut(|t| *t = Some(target));
}

/// Take and clear the reposition target
///
/// Returns and clears the target.
#[cfg(feature = "embassy")]
pub fn take_reposition_target() -> Option<PositionTarget> {
    use crate::traits::sync::SharedState;
    REPOSITION_TARGET.with_mut(|t| t.take())
}
