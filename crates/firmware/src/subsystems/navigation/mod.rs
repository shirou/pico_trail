//! Navigation subsystem
//!
//! This module provides navigation functionality for autonomous modes:
//! - Geographic calculations (bearing, distance)
//! - Navigation controller for position-based steering
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                     Mode (Guided/Auto)                       │
//! │                      update(dt) called at 50Hz               │
//! └─────────────────────────┬───────────────────────────────────┘
//!                           │
//!                           │ controller.update(current, target, dt)
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │              NavigationController (trait)                    │
//! └─────────────────────────┬───────────────────────────────────┘
//!                           │
//!               ┌───────────┴───────────┐
//!               ▼                       ▼
//! ┌─────────────────────────┐ ┌─────────────────────────────────┐
//! │ SimpleNavigationController│ │ L1NavigationController (Future)│
//! └─────────────────────────┘ └─────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use crate::subsystems::navigation::{
//!     SimpleNavigationController, NavigationController, PositionTarget
//! };
//!
//! let mut controller = SimpleNavigationController::new();
//! let target = PositionTarget::new(35.6762, 139.6503);
//!
//! // In control loop:
//! let output = controller.update(&current_gps, &target, dt);
//! motor.set_steering(output.steering);
//! motor.set_throttle(output.throttle);
//! ```

mod controller;
mod geo;
pub mod heading;
pub mod path_recorder;
mod types;

// Re-export public API
pub use controller::{NavigationController, SimpleNavigationController};
pub use geo::{calculate_bearing, calculate_distance, offset_position, wrap_180, wrap_360};
pub use heading::{FusedHeadingSource, HeadingSource, HeadingSourceType};
pub use path_recorder::{PathPoint, PathRecorder, PATH_RECORDER};
pub use types::{NavigationOutput, PositionTarget, SimpleNavConfig};

use crate::core::traits::EmbassyState;

/// Global navigation target (protected by EmbassyState)
///
/// **DEPRECATED**: Use `MISSION_STORAGE` from `crate::core::mission` instead.
/// This global is being replaced by the unified MissionStorage approach
/// where all waypoints (from MISSION_ITEM protocol or SET_POSITION_TARGET)
/// are stored in a single source of truth.
///
/// Set by MAVLink handler when receiving SET_POSITION_TARGET_GLOBAL_INT.
/// Read by navigation_task to compute steering/throttle commands.
pub static NAV_TARGET: EmbassyState<Option<PositionTarget>> = EmbassyState::new(None);

/// Global navigation output (protected by EmbassyState)
///
/// Updated by navigation_task at 50Hz.
/// Read by motor_control_task when in Guided/Auto mode.
pub static NAV_OUTPUT: EmbassyState<NavigationOutput> = EmbassyState::new(NavigationOutput {
    steering: 0.0,
    throttle: 0.0,
    distance_m: 0.0,
    bearing_deg: 0.0,
    heading_error_deg: 0.0,
    at_target: false,
});
