//! Command Protocol Handler - re-exported from core
//!
//! See `pico_trail_core::communication::handlers::command` for implementation.

pub use pico_trail_core::communication::handlers::command::{
    CommandHandler, MAVLINK_MAX_VERSION, MAVLINK_MIN_VERSION, MAVLINK_VERSION,
};
