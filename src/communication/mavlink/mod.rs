//! MAVLink 2.0 Protocol Communication
//!
//! This module implements MAVLink 2.0 protocol for communication with Ground Control
//! Stations (GCS) such as QGroundControl and Mission Planner.
//!
//! # Architecture
//!
//! - **Parser**: Async message parsing from UART
//! - **Writer**: Async message serialization to UART
//! - **Router**: Message dispatch to handlers
//! - **Handlers**: Protocol-specific message handlers (param, telemetry, command, mission)
//! - **State**: System state tracking (armed, mode, connection)
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::MavlinkTask;
//!
//! #[embassy_executor::task]
//! async fn mavlink_task(uart: impl UartInterface) {
//!     // MAVLink task will be implemented here
//! }
//! ```
//!
//! # Transport
//!
//! Supports multiple transport types via trait abstraction:
//! - UART: Primary transport for GCS communication (115200 baud)
//! - UDP: Network transport over WiFi
//! - TCP: Reliable network transport (future)

pub mod dispatcher; // Message dispatcher (routing to handlers)
pub mod handlers; // Message handlers
pub mod parser; // Message parsing
pub mod router; // Protocol message routing
pub mod state; // System state
pub mod status_notifier; // STATUSTEXT notification system
pub mod task; // MAVLink task
pub mod transport; // Transport abstraction layer
pub mod transport_router; // Multi-transport routing
pub mod writer; // Message writing
