//! Mission Protocol Handler - re-exported from core
//!
//! See `pico_trail_core::communication::handlers::mission` for implementation.

pub use pico_trail_core::communication::handlers::mission::{
    MissionHandler, MissionState, MISSION_TIMEOUT_US,
};
