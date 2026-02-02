//! Parameter management system
//!
//! This module provides parameter storage and management for the autopilot system.
//! Parameters are stored in Flash memory with redundant block rotation for wear leveling.
//!
//! Core parameter types (block, crc) are provided by `pico_trail_core::parameters`.

pub mod registry;
pub mod saver;
pub mod storage;

// Re-export core types
pub use pico_trail_core::parameters::{
    calculate_crc32, hash_param_name, validate_crc32, Parameter, ParameterBlockHeader,
    ParameterFlags, MAX_PARAMS_PER_BLOCK,
};
pub use registry::{ParamMetadata, ParamType, ParamValue, ParameterRegistry, RegistryError};
pub use saver::{ParamSaver, SaveRequest};
pub use storage::{FlashParamStorage, StorageStats};
