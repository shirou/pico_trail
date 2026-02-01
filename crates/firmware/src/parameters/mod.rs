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

pub mod board;
pub mod storage;

// Re-export core parameter group modules (moved from firmware to core)
pub use pico_trail_core::parameters::arming;
pub use pico_trail_core::parameters::battery;
pub use pico_trail_core::parameters::circle;
pub use pico_trail_core::parameters::compass;
pub use pico_trail_core::parameters::failsafe;
pub use pico_trail_core::parameters::fence;
pub use pico_trail_core::parameters::loiter;
pub use pico_trail_core::parameters::wifi;

// Re-export core parameter group types
pub use pico_trail_core::parameters::arming::ArmingParams;
pub use pico_trail_core::parameters::battery::BatteryParams;
pub use pico_trail_core::parameters::circle::{CircleDirection, CircleParams};
pub use pico_trail_core::parameters::compass::CompassParams;
pub use pico_trail_core::parameters::error::ParameterError;
pub use pico_trail_core::parameters::failsafe::FailsafeParams;
pub use pico_trail_core::parameters::fence::FenceParams;
pub use pico_trail_core::parameters::loiter::LoiterParams;
pub use pico_trail_core::parameters::storage::{
    ParamFlags, ParamMetadata, ParamValue, ParameterStore, MAX_STRING_LEN, PARAM_NAME_LEN,
};
pub use pico_trail_core::parameters::wifi::WifiParams;

// Local firmware modules
pub use board::BoardParams;
pub use storage::{load_from_flash, save_to_flash};
