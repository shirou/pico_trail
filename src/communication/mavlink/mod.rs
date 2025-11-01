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
//! - UDP: Network transport over WiFi (Phase 2+)
//! - TCP: Reliable network transport (future)

// Module structure populated during Phase 1-2
pub mod handlers; // Message handlers (Phase 2+)
pub mod parser; // Message parsing (Phase 1)
pub mod performance; // Performance metrics tracking (Phase 3)
pub mod router; // Protocol message routing (Phase 1)
pub mod state; // System state (Phase 1)
pub mod task; // MAVLink task (Phase 1)
pub mod transport; // Transport abstraction layer (Phase 1+)
pub mod transport_router; // Multi-transport routing (Phase 1+)
pub mod writer; // Message writing (Phase 1)
