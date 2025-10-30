//! Task scheduler for periodic execution of autopilot tasks
//!
//! This module provides a task scheduler built on Embassy async framework.
//! Tasks can be registered with configurable rates (1Hz-400Hz) and execute
//! with deterministic timing. The scheduler provides:
//!
//! - Task execution time monitoring
//! - CPU load tracking
//! - Deadline miss detection
//! - Jitter measurement
//!
//! # Example
//!
//! ```rust,ignore
//! use embassy_executor::Spawner;
//! use embassy_time::{Duration, Ticker};
//!
//! #[embassy_executor::task]
//! async fn imu_task() {
//!     let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
//!     loop {
//!         ticker.next().await;
//!         // Sample IMU sensor
//!         sample_imu().await;
//!     }
//! }
//! ```

pub mod monitor;
pub mod registry;
pub mod stats;
pub mod task;
pub mod tasks;
pub mod types;

pub use monitor::*;
pub use registry::*;
pub use stats::*;
pub use task::*;
pub use types::*;
