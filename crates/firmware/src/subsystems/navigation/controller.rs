//! Navigation controller (firmware wrapper)
//!
//! Re-exports core navigation controller types and provides
//! a compatibility wrapper for GpsPosition-based calls.

pub use pico_trail_core::navigation::controller::{
    NavigationController, SimpleNavigationController,
};
