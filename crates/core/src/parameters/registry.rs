//! Parameter registry types
//!
//! This module provides platform-agnostic parameter types for configuration management.
//! The actual registry implementation with Flash persistence is in the firmware crate.

/// Parameter type enumeration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamType {
    /// 32-bit floating point parameter
    Float,
    /// 32-bit unsigned integer parameter
    Uint32,
}

/// Parameter value (union of supported types)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RegistryParamValue {
    /// Float value
    Float(f32),
    /// Unsigned integer value
    Uint32(u32),
}

impl RegistryParamValue {
    /// Convert parameter value to u32 representation (for Flash storage)
    pub fn to_u32(self) -> u32 {
        match self {
            RegistryParamValue::Float(f) => f.to_bits(),
            RegistryParamValue::Uint32(u) => u,
        }
    }

    /// Create parameter value from u32 and type
    pub fn from_u32(value: u32, param_type: ParamType) -> Self {
        match param_type {
            ParamType::Float => RegistryParamValue::Float(f32::from_bits(value)),
            ParamType::Uint32 => RegistryParamValue::Uint32(value),
        }
    }

    /// Get parameter type
    pub fn param_type(&self) -> ParamType {
        match self {
            RegistryParamValue::Float(_) => ParamType::Float,
            RegistryParamValue::Uint32(_) => ParamType::Uint32,
        }
    }
}

/// Parameter metadata (definition and current value)
#[derive(Debug, Clone)]
pub struct RegistryParamMetadata {
    /// Parameter name (max 16 characters, MAVLink standard)
    pub name: &'static str,
    /// Parameter type
    pub param_type: ParamType,
    /// Current value
    pub value: RegistryParamValue,
    /// Default value
    pub default: RegistryParamValue,
    /// Minimum allowed value
    pub min: RegistryParamValue,
    /// Maximum allowed value
    pub max: RegistryParamValue,
    /// Modified flag (true if changed since last save)
    pub modified: bool,
}

impl RegistryParamMetadata {
    /// Create new parameter metadata with Float type
    pub const fn new_float(name: &'static str, default: f32, min: f32, max: f32) -> Self {
        Self {
            name,
            param_type: ParamType::Float,
            value: RegistryParamValue::Float(default),
            default: RegistryParamValue::Float(default),
            min: RegistryParamValue::Float(min),
            max: RegistryParamValue::Float(max),
            modified: false,
        }
    }

    /// Create new parameter metadata with Uint32 type
    pub const fn new_uint32(name: &'static str, default: u32, min: u32, max: u32) -> Self {
        Self {
            name,
            param_type: ParamType::Uint32,
            value: RegistryParamValue::Uint32(default),
            default: RegistryParamValue::Uint32(default),
            min: RegistryParamValue::Uint32(min),
            max: RegistryParamValue::Uint32(max),
            modified: false,
        }
    }

    /// Validate value is within bounds
    pub fn is_valid(&self, value: RegistryParamValue) -> bool {
        if value.param_type() != self.param_type {
            return false;
        }

        match (value, self.min, self.max) {
            (
                RegistryParamValue::Float(v),
                RegistryParamValue::Float(min),
                RegistryParamValue::Float(max),
            ) => v >= min && v <= max,
            (
                RegistryParamValue::Uint32(v),
                RegistryParamValue::Uint32(min),
                RegistryParamValue::Uint32(max),
            ) => v >= min && v <= max,
            _ => false,
        }
    }
}

/// Parameter registry error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegistryError {
    /// Parameter not found
    NotFound,
    /// Invalid parameter value (out of bounds or wrong type)
    InvalidValue,
    /// Flash operation failed
    FlashError,
    /// Parameter list full
    Full,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_value_conversion() {
        let float_val = RegistryParamValue::Float(core::f32::consts::PI);
        let u32_val = float_val.to_u32();
        let recovered = RegistryParamValue::from_u32(u32_val, ParamType::Float);
        assert_eq!(float_val, recovered);

        let uint_val = RegistryParamValue::Uint32(42);
        let u32_val = uint_val.to_u32();
        let recovered = RegistryParamValue::from_u32(u32_val, ParamType::Uint32);
        assert_eq!(uint_val, recovered);
    }

    #[test]
    fn test_param_metadata_validation() {
        let param = RegistryParamMetadata::new_float("TEST_PARAM", 10.0, 0.0, 100.0);

        assert!(param.is_valid(RegistryParamValue::Float(50.0)));
        assert!(param.is_valid(RegistryParamValue::Float(0.0)));
        assert!(param.is_valid(RegistryParamValue::Float(100.0)));
        assert!(!param.is_valid(RegistryParamValue::Float(-1.0)));
        assert!(!param.is_valid(RegistryParamValue::Float(101.0)));
        assert!(!param.is_valid(RegistryParamValue::Uint32(50)));
    }

    #[test]
    fn test_param_metadata_new_uint32() {
        let param = RegistryParamMetadata::new_uint32("TEST_U32", 50, 0, 100);

        assert_eq!(param.name, "TEST_U32");
        assert_eq!(param.param_type, ParamType::Uint32);
        assert_eq!(param.value, RegistryParamValue::Uint32(50));
        assert_eq!(param.default, RegistryParamValue::Uint32(50));
        assert!(!param.modified);

        assert!(param.is_valid(RegistryParamValue::Uint32(0)));
        assert!(param.is_valid(RegistryParamValue::Uint32(100)));
        assert!(!param.is_valid(RegistryParamValue::Uint32(101)));
        assert!(!param.is_valid(RegistryParamValue::Float(50.0)));
    }

    #[test]
    fn test_registry_error() {
        let err = RegistryError::NotFound;
        assert_eq!(err, RegistryError::NotFound);

        let err = RegistryError::InvalidValue;
        assert_eq!(err, RegistryError::InvalidValue);
    }
}
