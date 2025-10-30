//! MAVLink Protocol Handlers
//!
//! Message-specific handlers for MAVLink protocol implementation.
//!
//! # Handlers
//!
//! - **Parameter Handler** (Phase 2): PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET
//! - **Telemetry Streamer** (Phase 3): HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS
//! - **Command Handler** (Phase 4): COMMAND_LONG, COMMAND_ACK
//! - **Mission Handler** (Phase 5): MISSION_COUNT, MISSION_ITEM, MISSION_ACK

pub mod command;
pub mod mission;
pub mod param;
pub mod telemetry;

// Re-export commonly used types
pub use command::CommandHandler;
pub use mission::MissionHandler;
pub use param::ParamHandler;
pub use telemetry::TelemetryStreamer;
