//! Parameter error types
//!
//! Provides error types for parameter store operations.

/// Errors from parameter store operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterError {
    /// Invalid configuration (e.g., duplicate registration, unknown parameter)
    InvalidConfig,
    /// Store is full
    StoreFull,
    /// Read-only parameter cannot be modified
    ReadOnly,
}

impl core::fmt::Display for ParameterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParameterError::InvalidConfig => write!(f, "invalid parameter configuration"),
            ParameterError::StoreFull => write!(f, "parameter store full"),
            ParameterError::ReadOnly => write!(f, "parameter is read-only"),
        }
    }
}
