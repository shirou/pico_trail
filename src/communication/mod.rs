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
//! - **SHTP (Sensor Hub Transport Protocol)**: For Hillcrest/CEVA IMU sensors
//!   - Used by BNO080, BNO085, BNO086, FSM300 series
//!   - Packet-based protocol over I2C/SPI/UART
//!   - Supports sensor reports and configuration
//!
//! # Transport Layers
//!
//! - UART (115200 baud, 8N1) - MAVLink
//! - USB CDC - Future enhancement
//! - I2C (400kHz) - SHTP

pub mod mavlink;
pub mod shtp;
