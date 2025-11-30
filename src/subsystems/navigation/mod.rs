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
pub use geo::{calculate_bearing, calculate_distance, wrap_180, wrap_360};
pub use types::{NavigationOutput, PositionTarget, SimpleNavConfig};

// Global navigation state (for multi-task access)
#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "embassy")]
use embassy_sync::mutex::Mutex;

/// Global navigation target (protected by Mutex)
///
/// Set by MAVLink handler when receiving SET_POSITION_TARGET_GLOBAL_INT.
/// Read by navigation_task to compute steering/throttle commands.
#[cfg(feature = "embassy")]
pub static NAV_TARGET: Mutex<CriticalSectionRawMutex, Option<PositionTarget>> = Mutex::new(None);

/// Global navigation output (protected by Mutex)
///
/// Updated by navigation_task at 50Hz.
/// Read by motor_control_task when in Guided/Auto mode.
#[cfg(feature = "embassy")]
pub static NAV_OUTPUT: Mutex<CriticalSectionRawMutex, NavigationOutput> =
    Mutex::new(NavigationOutput {
        steering: 0.0,
        throttle: 0.0,
        distance_m: 0.0,
        bearing_deg: 0.0,
        heading_error_deg: 0.0,
        at_target: false,
    });
