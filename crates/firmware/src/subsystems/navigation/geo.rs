//! Geographic calculations for navigation
//!
//! Re-exports from `pico_trail_core::navigation::geo`. The platform-independent
//! algorithms live in the core crate.

pub use pico_trail_core::navigation::geo::{
    calculate_bearing, calculate_distance, offset_position, wrap_180, wrap_360,
};
