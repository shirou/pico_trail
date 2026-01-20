//! Task scheduler types and statistics for autopilot task management
//!
//! This module provides core types for task scheduling without any
//! async runtime dependencies. The actual task execution is handled
//! by the firmware crate using Embassy.
//!
//! # Components
//!
//! - [`types`]: Core types (TaskMetadata, TaskStats, SchedulerStats)
//! - [`registry`]: Task registration and lookup
//! - [`stats`]: Statistics tracking and CPU load calculation
//!
//! # Example
//!
//! ```rust
//! use pico_trail_core::scheduler::{TaskMetadata, register_task};
//!
//! let metadata = TaskMetadata {
//!     name: "control_task",
//!     rate_hz: 50,
//!     priority: 10,
//!     budget_us: 15000,
//! };
//!
//! let task_id = register_task(metadata);
//! ```

pub mod registry;
pub mod stats;
pub mod types;

pub use registry::*;
pub use stats::*;
pub use types::*;
