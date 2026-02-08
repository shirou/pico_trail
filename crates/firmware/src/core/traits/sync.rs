//! Synchronized state abstraction traits for platform-agnostic state access.
//!
//! Re-exports from `pico_trail_core::traits::sync` to maintain backward
//! compatibility with existing firmware imports.

pub use pico_trail_core::traits::sync::{EmbassyState, MockState, SharedState};
