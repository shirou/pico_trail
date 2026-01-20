//! Parameter management types and utilities
//!
//! This module provides core parameter types for Flash storage management.
//! Platform-specific implementations (Flash drivers, async saving) are in the firmware crate.

pub mod block;
pub mod crc;
pub mod registry;

pub use block::{hash_param_name, Parameter, ParameterBlockHeader, ParameterFlags, MAX_PARAMS};
pub use crc::{calculate_crc32, validate_crc32};
pub use registry::{ParamMetadata, ParamType, ParamValue, RegistryError};
