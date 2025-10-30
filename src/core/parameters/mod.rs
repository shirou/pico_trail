//! Parameter management system
//!
//! This module provides parameter storage and management for the autopilot system.
//! Parameters are stored in Flash memory with redundant block rotation for wear leveling.

pub mod block;
pub mod crc;
pub mod registry;
#[cfg(feature = "pico2_w")]
pub mod saver;
pub mod storage;

// Re-export commonly used types
pub use block::{hash_param_name, Parameter, ParameterBlockHeader, ParameterFlags};
pub use crc::{calculate_crc32, validate_crc32};
pub use registry::{ParamMetadata, ParamType, ParamValue, ParameterRegistry, RegistryError};
#[cfg(feature = "pico2_w")]
pub use saver::{ParamSaver, SaveRequest};
pub use storage::{FlashParamStorage, StorageStats};
