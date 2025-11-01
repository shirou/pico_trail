//! Parameter Storage System
//!
//! This module provides Flash-backed parameter storage for persistent configuration.
//! Parameters are stored in Flash and loaded at boot, supporting runtime changes
//! via MAVLink protocol.
//!
//! # Architecture
//!
//! ```text
//! ┌────────────────────────────────────────┐
//! │     MAVLink Parameter Protocol          │
//! │  (PARAM_REQUEST_LIST/READ/SET)         │
//! └──────────────┬─────────────────────────┘
//!                │
//!                ▼
//! ┌────────────────────────────────────────┐
//! │        ParameterStore                   │
//! │  - In-memory parameter map              │
//! │  - Flash persistence                    │
//! │  - Type-safe access                     │
//! └──────────────┬─────────────────────────┘
//!                │
//!                ▼
//! ┌────────────────────────────────────────┐
//! │         Flash Interface                 │
//! │  (Parameter blocks 0-3)                 │
//! └────────────────────────────────────────┘
//! ```
//!
//! # Flash Layout
//!
//! ```text
//! [Parameter Block 0]  0x040000 - 0x041000 (4 KB) - Primary
//! [Parameter Block 1]  0x041000 - 0x042000 (4 KB) - Backup
//! [Parameter Block 2]  0x042000 - 0x043000 (4 KB) - Backup
//! [Parameter Block 3]  0x043000 - 0x044000 (4 KB) - Backup
//! ```
//!
//! # Parameter Types
//!
//! - `String<N>` - Fixed-capacity string (e.g., WiFi SSID)
//! - `Bool` - Boolean value
//! - `Int` - 32-bit signed integer
//! - `Float` - 32-bit floating point
//! - `Ipv4` - IPv4 address (4 bytes)
//!
//! # Features
//!
//! - **Flash Persistence**: Parameters saved to Flash, loaded at boot
//! - **Type Safety**: Strongly-typed parameter access
//! - **Hidden Parameters**: Password hiding (NET_PASS not readable via MAVLink)
//! - **Redundancy**: Multiple Flash blocks for reliability
//! - **CRC Validation**: Data integrity checking

pub mod storage;
pub mod wifi;

pub use storage::{ParamValue, ParameterStore};
pub use wifi::WifiParams;
