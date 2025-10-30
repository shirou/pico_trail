//! Communication Protocols
//!
//! This module provides communication protocol implementations for the autopilot,
//! including MAVLink for ground control station integration.
//!
//! # Protocols
//!
//! - **MAVLink 2.0**: Primary GCS communication protocol
//!   - Telemetry streaming (HEARTBEAT, ATTITUDE, GPS, etc.)
//!   - Parameter management (PARAM_* messages)
//!   - Command execution (COMMAND_LONG)
//!   - Mission protocol (MISSION_* messages)
//!
//! # Transport Layers
//!
//! - UART (115200 baud, 8N1) - Initial implementation
//! - USB CDC - Future enhancement

pub mod mavlink;
