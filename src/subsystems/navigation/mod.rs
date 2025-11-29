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
