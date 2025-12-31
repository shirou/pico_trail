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
mod types;

// Re-export public API
pub use controller::{NavigationController, SimpleNavigationController};
pub use geo::{calculate_bearing, calculate_distance, offset_position, wrap_180, wrap_360};
pub use types::{NavigationOutput, PositionTarget, SimpleNavConfig};

// ============================================================================
// Embassy Implementation (embedded targets)
// ============================================================================

#[cfg(feature = "embassy")]
mod embassy_impl {
    use super::{NavigationOutput, PositionTarget};
    use crate::core::traits::{EmbassyState, SharedState};

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

    /// Reposition target from MAV_CMD_DO_REPOSITION command (synchronous access)
    ///
    /// Set by CommandHandler when receiving MAV_CMD_DO_REPOSITION.
    /// Read and cleared by navigation_task to update NAV_TARGET.
    pub static REPOSITION_TARGET: EmbassyState<Option<PositionTarget>> = EmbassyState::new(None);

    /// Set reposition target from command handler (synchronous)
    ///
    /// Called by CommandHandler when receiving MAV_CMD_DO_REPOSITION.
    /// The navigation_task will pick this up and update NAV_TARGET.
    pub fn set_reposition_target(target: PositionTarget) {
        REPOSITION_TARGET.with_mut(|t| *t = Some(target));
    }

    /// Take reposition target if available (synchronous)
    ///
    /// Called by navigation_task to check for reposition commands.
    /// Returns and clears the target.
    pub fn take_reposition_target() -> Option<PositionTarget> {
        REPOSITION_TARGET.with_mut(|t| t.take())
    }
}

#[cfg(feature = "embassy")]
pub use embassy_impl::{
    set_reposition_target, take_reposition_target, NAV_OUTPUT, NAV_TARGET, REPOSITION_TARGET,
};

// ============================================================================
// Host Test Stubs (no-op implementations)
// ============================================================================

#[cfg(not(feature = "embassy"))]
pub fn set_reposition_target(_target: PositionTarget) {
    // No-op for host tests
}

#[cfg(not(feature = "embassy"))]
pub fn take_reposition_target() -> Option<PositionTarget> {
    None
}
