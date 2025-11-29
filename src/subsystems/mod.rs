//! Autopilot subsystems
//!
//! This module contains high-level subsystems built on top of core systems and devices:
//! - AHRS: Attitude and Heading Reference System (sensor fusion)
//! - Navigation: Waypoint navigation and path following
//! - Control: Steering and throttle control

pub mod ahrs;
pub mod navigation;
