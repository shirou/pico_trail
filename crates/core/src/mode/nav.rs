//! Navigation calculation utilities
//!
//! Re-exports from `crate::navigation::geo` for backwards compatibility.
//! New code should use `crate::navigation::geo` directly.

pub use crate::navigation::geo::{haversine_distance_bearing, normalize_angle};
