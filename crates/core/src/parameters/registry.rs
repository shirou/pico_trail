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
pub enum ParamValue {
    /// Float value
    Float(f32),
    /// Unsigned integer value
    Uint32(u32),
}

impl ParamValue {
    /// Convert parameter value to u32 representation (for Flash storage)
    pub fn to_u32(self) -> u32 {
        match self {
            ParamValue::Float(f) => f.to_bits(),
            ParamValue::Uint32(u) => u,
        }
    }

    /// Create parameter value from u32 and type
    pub fn from_u32(value: u32, param_type: ParamType) -> Self {
        match param_type {
            ParamType::Float => ParamValue::Float(f32::from_bits(value)),
            ParamType::Uint32 => ParamValue::Uint32(value),
        }
    }

    /// Get parameter type
    pub fn param_type(&self) -> ParamType {
        match self {
            ParamValue::Float(_) => ParamType::Float,
            ParamValue::Uint32(_) => ParamType::Uint32,
        }
    }
}

/// Parameter metadata (definition and current value)
#[derive(Debug, Clone)]
pub struct ParamMetadata {
    /// Parameter name (max 16 characters, MAVLink standard)
    pub name: &'static str,
    /// Parameter type
    pub param_type: ParamType,
    /// Current value
    pub value: ParamValue,
    /// Default value
    pub default: ParamValue,
    /// Minimum allowed value
    pub min: ParamValue,
    /// Maximum allowed value
    pub max: ParamValue,
    /// Modified flag (true if changed since last save)
    pub modified: bool,
}

impl ParamMetadata {
    /// Create new parameter metadata with Float type
    pub const fn new_float(name: &'static str, default: f32, min: f32, max: f32) -> Self {
        Self {
            name,
            param_type: ParamType::Float,
            value: ParamValue::Float(default),
            default: ParamValue::Float(default),
            min: ParamValue::Float(min),
            max: ParamValue::Float(max),
            modified: false,
        }
    }

    /// Create new parameter metadata with Uint32 type
    pub const fn new_uint32(name: &'static str, default: u32, min: u32, max: u32) -> Self {
        Self {
            name,
            param_type: ParamType::Uint32,
            value: ParamValue::Uint32(default),
            default: ParamValue::Uint32(default),
            min: ParamValue::Uint32(min),
            max: ParamValue::Uint32(max),
            modified: false,
        }
    }

    /// Validate value is within bounds
    pub fn is_valid(&self, value: ParamValue) -> bool {
        if value.param_type() != self.param_type {
            return false;
        }

        match (value, self.min, self.max) {
            (ParamValue::Float(v), ParamValue::Float(min), ParamValue::Float(max)) => {
                v >= min && v <= max
            }
            (ParamValue::Uint32(v), ParamValue::Uint32(min), ParamValue::Uint32(max)) => {
                v >= min && v <= max
            }
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
        let float_val = ParamValue::Float(core::f32::consts::PI);
        let u32_val = float_val.to_u32();
        let recovered = ParamValue::from_u32(u32_val, ParamType::Float);
        assert_eq!(float_val, recovered);

        let uint_val = ParamValue::Uint32(42);
        let u32_val = uint_val.to_u32();
        let recovered = ParamValue::from_u32(u32_val, ParamType::Uint32);
        assert_eq!(uint_val, recovered);
    }

    #[test]
    fn test_param_metadata_validation() {
        let param = ParamMetadata::new_float("TEST_PARAM", 10.0, 0.0, 100.0);

        assert!(param.is_valid(ParamValue::Float(50.0)));
        assert!(param.is_valid(ParamValue::Float(0.0)));
        assert!(param.is_valid(ParamValue::Float(100.0)));
        assert!(!param.is_valid(ParamValue::Float(-1.0)));
        assert!(!param.is_valid(ParamValue::Float(101.0)));
        assert!(!param.is_valid(ParamValue::Uint32(50)));
    }

    #[test]
    fn test_param_metadata_new_uint32() {
        let param = ParamMetadata::new_uint32("TEST_U32", 50, 0, 100);

        assert_eq!(param.name, "TEST_U32");
        assert_eq!(param.param_type, ParamType::Uint32);
        assert_eq!(param.value, ParamValue::Uint32(50));
        assert_eq!(param.default, ParamValue::Uint32(50));
        assert!(!param.modified);

        assert!(param.is_valid(ParamValue::Uint32(0)));
        assert!(param.is_valid(ParamValue::Uint32(100)));
        assert!(!param.is_valid(ParamValue::Uint32(101)));
        assert!(!param.is_valid(ParamValue::Float(50.0)));
    }

    #[test]
    fn test_registry_error() {
        let err = RegistryError::NotFound;
        assert_eq!(err, RegistryError::NotFound);

        let err = RegistryError::InvalidValue;
        assert_eq!(err, RegistryError::InvalidValue);
    }
}
