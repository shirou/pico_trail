//! Core traits for platform-agnostic autopilot functionality.
//!
//! This module provides trait abstractions that decouple core autopilot logic
//! from platform-specific implementations (Embassy, mock, etc.).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Application Layer                            │
//! │  (examples/, rover/, communication/mavlink/handlers/)           │
//! │                           │                                     │
//! │                           ▼                                     │
//! │  ┌─────────────────────────────────────────────────────────┐   │
//! │  │              Core Traits (no feature gates)             │   │
//! │  │  ┌──────────────┐  ┌─────────────────────────────────┐ │   │
//! │  │  │ TimeSource   │  │ SharedState<T>                  │ │   │
//! │  │  │ + now_ms()   │  │ + with(f: Fn(&T) -> R)         │ │   │
//! │  │  │ + now_us()   │  │ + with_mut(f: Fn(&mut T) -> R) │ │   │
//! │  │  └──────────────┘  └─────────────────────────────────┘ │   │
//! │  └─────────────────────────────────────────────────────────┘   │
//! │                           │                                     │
//! │          ┌────────────────┴────────────────┐                   │
//! │          ▼                                 ▼                   │
//! │  ┌──────────────────────┐    ┌──────────────────────────────┐ │
//! │  │ Embassy Impl         │    │ Mock Impl                     │ │
//! │  │ #[cfg(feature =      │    │ (always available)            │ │
//! │  │   "embassy")]        │    │                               │ │
//! │  │                      │    │ MockTime, MockState<T>        │ │
//! │  │ EmbassyTime,         │    │                               │ │
//! │  │ EmbassyState<T>      │    │                               │ │
//! │  └──────────────────────┘    └──────────────────────────────┘ │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Features
//!
//! - **`embassy`**: Enables Embassy implementations (`EmbassyTime`, `EmbassyState<T>`)
//! - Mock implementations are always available for host testing
//!
//! # Example
//!
//! ```ignore
//! use pico_trail::core::traits::{TimeSource, SharedState, MockTime, MockState};
//!
//! // For testing
//! let time = MockTime::new();
//! let state = MockState::new(0u32);
//!
//! // Use traits in generic functions
//! fn do_work<T: TimeSource, S: SharedState<u32>>(time: &T, state: &S) {
//!     let now = time.now_us();
//!     state.with_mut(|v| *v = now as u32);
//! }
//! ```

pub mod sync;
pub mod time;

// Re-export traits and mock implementations (always available)
pub use sync::{MockState, SharedState};
pub use time::{MockTime, TimeSource};

// Re-export Embassy implementations when embassy feature is enabled
#[cfg(feature = "embassy")]
pub use sync::EmbassyState;

#[cfg(feature = "embassy")]
pub use time::EmbassyTime;
