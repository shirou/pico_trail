//! Parameter management types and utilities
//!
//! This module provides core parameter types for configuration management
//! and Flash storage. Platform-specific implementations (Flash drivers,
//! async saving) are in the firmware crate.

pub mod arming;
pub mod battery;
pub mod block;
pub mod circle;
pub mod compass;
pub mod crc;
pub mod error;
pub mod failsafe;
pub mod fence;
pub mod loiter;
pub mod registry;
pub mod storage;
pub mod wifi;

pub use arming::ArmingParams;
pub use battery::BatteryParams;
pub use block::{
    hash_param_name, Parameter, ParameterBlockHeader, ParameterFlags, MAX_PARAMS_PER_BLOCK,
};
pub use circle::{CircleDirection, CircleParams};
pub use compass::CompassParams;
pub use crc::{calculate_crc32, validate_crc32};
pub use error::ParameterError;
pub use failsafe::FailsafeParams;
pub use fence::FenceParams;
pub use loiter::LoiterParams;
pub use registry::{ParamType, RegistryError};
pub use storage::{ParamFlags, ParamMetadata, ParamValue, ParameterStore};
pub use storage::{
    MAX_PARAMS, MAX_STRING_LEN, PARAM_BLOCK_BASE, PARAM_BLOCK_COUNT, PARAM_BLOCK_SIZE, PARAM_MAGIC,
    PARAM_NAME_LEN, PARAM_VERSION,
};
pub use wifi::WifiParams;
