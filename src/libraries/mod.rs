//! Common libraries
//!
//! This module contains vehicle-agnostic libraries shared across different vehicle types
//! (Rover, Boat, Copter, etc.), following ArduPilot's library architecture.
//!
//! ## Libraries
//!
//! - `kinematics`: Vehicle kinematics conversions (differential drive, etc.)
//! - `rc_channel`: RC input processing (RC_Channel equivalent)
//! - `srv_channel`: Servo/motor output processing (SRV_Channel equivalent)

pub mod kinematics;
pub mod rc_channel;
pub mod srv_channel;

// Re-export commonly used types
#[cfg(feature = "pico2_w")]
pub use rc_channel::RC_INPUT;
pub use rc_channel::{RcInput, RcStatus};

pub use srv_channel::{ActuatorConfig, ActuatorInterface, Actuators};
