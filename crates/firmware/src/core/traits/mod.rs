//! Core traits for platform-agnostic autopilot functionality.
//!
//! This module re-exports traits from `pico_trail_core::traits` and provides
//! firmware-specific implementations (Embassy-based time, etc.).
//!
//! # Architecture
//!
//! ```text
//! +-------------------------------------------------------------+
//! |                    Application Layer                          |
//! |  (examples/, rover/, communication/mavlink/handlers/)         |
//! |                           |                                   |
//! |                           v                                   |
//! |  +-------------------------------------------------------+   |
//! |  |                     Core Traits                         |  |
//! |  |  +------------+  +---------------------------------+   |  |
//! |  |  | TimeSource |  | SharedState<T>                  |   |  |
//! |  |  | + now_ms() |  | + with(f: Fn(&T) -> R)         |   |  |
//! |  |  | + now_us() |  | + with_mut(f: Fn(&mut T) -> R) |   |  |
//! |  |  +------------+  +---------------------------------+   |  |
//! |  +-------------------------------------------------------+   |
//! |                           |                                   |
//! |          +----------------+----------------+                  |
//! |          v                                 v                  |
//! |  +----------------------+    +----------------------------+   |
//! |  | Embassy Impl         |    | Mock Impl                  |  |
//! |  |                      |    |                             |  |
//! |  | EmbassyTime,         |    | MockTime, MockState<T>     |  |
//! |  | EmbassyState<T>      |    |                             |  |
//! |  +----------------------+    +----------------------------+   |
//! +-------------------------------------------------------------+
//! ```

pub mod sync;
pub mod time;

// Re-export traits and implementations
pub use sync::{EmbassyState, MockState, SharedState};
pub use time::{EmbassyTime, MockTime, TimeSource};
