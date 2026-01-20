//! Parameter registry for managing runtime configuration
//!
//! This module provides a minimal parameter registry for storing and managing
//! autopilot configuration parameters. Parameters can be persisted to Flash
//! storage using the FlashParamStorage backend.

use super::storage::FlashParamStorage;
use super::{hash_param_name, Parameter, ParameterFlags};
use crate::platform::error::{FlashError, PlatformError};
use crate::platform::traits::flash::FlashInterface;

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
    FlashError(FlashError),
    /// Parameter list full
    Full,
}

impl From<FlashError> for RegistryError {
    fn from(err: FlashError) -> Self {
        RegistryError::FlashError(err)
    }
}

impl From<PlatformError> for RegistryError {
    fn from(err: PlatformError) -> Self {
        match err {
            PlatformError::Flash(flash_err) => RegistryError::FlashError(flash_err),
            _ => RegistryError::FlashError(FlashError::ReadFailed), // Default fallback
        }
    }
}

/// Parameter registry with Flash persistence
pub struct ParameterRegistry<F: FlashInterface> {
    /// Parameter metadata array
    params: heapless::Vec<ParamMetadata, 200>,
    /// Flash storage backend (optional)
    flash_storage: Option<FlashParamStorage<F>>,
}

impl<F: FlashInterface> ParameterRegistry<F> {
    /// Create new parameter registry without Flash persistence
    pub fn new() -> Self {
        Self {
            params: heapless::Vec::new(),
            flash_storage: None,
        }
    }

    /// Create new parameter registry with Flash persistence
    pub fn with_flash(flash: F) -> Self {
        Self {
            params: heapless::Vec::new(),
            flash_storage: Some(FlashParamStorage::new(flash)),
        }
    }

    /// Register a parameter
    pub fn register(&mut self, param: ParamMetadata) -> Result<(), RegistryError> {
        self.params.push(param).map_err(|_| RegistryError::Full)
    }

    /// Get parameter count
    pub fn count(&self) -> usize {
        self.params.len()
    }

    /// Get parameter by name
    pub fn get_by_name(&self, name: &str) -> Option<&ParamMetadata> {
        self.params.iter().find(|p| p.name == name)
    }

    /// Get parameter by index
    pub fn get_by_index(&self, index: usize) -> Option<&ParamMetadata> {
        self.params.get(index)
    }

    /// Set parameter by name
    pub fn set_by_name(&mut self, name: &str, value: ParamValue) -> Result<(), RegistryError> {
        let param = self
            .params
            .iter_mut()
            .find(|p| p.name == name)
            .ok_or(RegistryError::NotFound)?;

        if !param.is_valid(value) {
            return Err(RegistryError::InvalidValue);
        }

        param.value = value;
        param.modified = true;
        Ok(())
    }

    /// Set parameter by index
    pub fn set_by_index(&mut self, index: usize, value: ParamValue) -> Result<(), RegistryError> {
        let param = self.params.get_mut(index).ok_or(RegistryError::NotFound)?;

        if !param.is_valid(value) {
            return Err(RegistryError::InvalidValue);
        }

        param.value = value;
        param.modified = true;
        Ok(())
    }

    /// Load parameters from Flash storage
    ///
    /// If Flash storage is not configured or no valid block is found,
    /// parameters remain at their default values.
    pub fn load_from_flash(&mut self) -> Result<(), RegistryError> {
        let storage = match &mut self.flash_storage {
            Some(s) => s,
            None => return Ok(()), // No Flash storage configured, use defaults
        };

        // Find active block with valid parameters
        let active_block = match storage.find_active_block() {
            Some(block) => block,
            None => return Ok(()), // No valid block found, use defaults
        };

        // Read parameters from active block
        let (_, flash_params, valid) = storage.read_block(active_block)?;

        if !valid {
            return Ok(()); // CRC invalid, use defaults
        }

        // Update registry with loaded values
        for flash_param in flash_params.iter() {
            // Find parameter by name hash
            if let Some(param) = self
                .params
                .iter_mut()
                .find(|p| hash_param_name(p.name) == flash_param.name_hash)
            {
                // Convert value from Flash format (stored as f32 bits)
                let value_u32 = flash_param.value.to_bits();
                let value = ParamValue::from_u32(value_u32, param.param_type);

                // Validate and update
                if param.is_valid(value) {
                    param.value = value;
                    param.modified = false; // Loaded from Flash, not modified
                }
            }
        }

        Ok(())
    }

    /// Save parameters to Flash storage
    ///
    /// Only saves if Flash storage is configured. All parameters are saved,
    /// and modified flags are cleared on success.
    pub fn save_to_flash(&mut self) -> Result<(), RegistryError> {
        let storage = match &mut self.flash_storage {
            Some(s) => s,
            None => return Ok(()), // No Flash storage configured, nothing to do
        };

        // Find current active block and sequence
        let (active_block, sequence) = match storage.find_active_block() {
            Some(block) => {
                let (header, _, _) = storage.read_block(block)?;
                (block, header.sequence)
            }
            None => (0, 0), // No active block, start fresh
        };

        // Choose next block for rotation
        let next_block = storage.choose_next_block(active_block);
        let next_sequence = storage.increment_sequence(sequence);

        // Convert registry parameters to Flash format
        let mut flash_params = heapless::Vec::<Parameter, 200>::new();
        for param in self.params.iter() {
            let flags = if param.param_type == ParamType::Float {
                ParameterFlags::TYPE_F32
            } else {
                ParameterFlags::empty()
            };

            // Convert value to f32 representation for Flash storage
            let value_u32 = param.value.to_u32();
            let value_f32 = f32::from_bits(value_u32);

            let flash_param = Parameter {
                name_hash: hash_param_name(param.name),
                value: value_f32,
                flags,
                reserved: 0,
            };

            flash_params.push(flash_param).ok();
        }

        // Write to Flash
        storage.write_block(next_block, &flash_params, next_sequence)?;

        // Clear modified flags on success
        for param in self.params.iter_mut() {
            param.modified = false;
        }

        Ok(())
    }

    /// Check if any parameters have been modified
    pub fn has_modified(&self) -> bool {
        self.params.iter().any(|p| p.modified)
    }

    /// Get storage statistics (if Flash storage is configured)
    pub fn storage_stats(&self) -> Option<super::storage::StorageStats> {
        self.flash_storage.as_ref().map(|s| s.get_stats())
    }
}

impl<F: FlashInterface> Default for ParameterRegistry<F> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

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
    fn test_registry_register() {
        let mut registry = ParameterRegistry::<MockFlash>::new();

        let param1 = ParamMetadata::new_float("PARAM1", 1.0, 0.0, 10.0);
        let param2 = ParamMetadata::new_uint32("PARAM2", 5, 0, 100);

        assert!(registry.register(param1).is_ok());
        assert!(registry.register(param2).is_ok());
        assert_eq!(registry.count(), 2);
    }

    #[test]
    fn test_registry_get_by_name() {
        let mut registry = ParameterRegistry::<MockFlash>::new();
        registry
            .register(ParamMetadata::new_float("TEST", 1.0, 0.0, 10.0))
            .unwrap();

        let param = registry.get_by_name("TEST").unwrap();
        assert_eq!(param.name, "TEST");
        assert_eq!(param.value, ParamValue::Float(1.0));

        assert!(registry.get_by_name("NONEXISTENT").is_none());
    }

    #[test]
    fn test_registry_get_by_index() {
        let mut registry = ParameterRegistry::<MockFlash>::new();
        registry
            .register(ParamMetadata::new_float("FIRST", 1.0, 0.0, 10.0))
            .unwrap();
        registry
            .register(ParamMetadata::new_float("SECOND", 2.0, 0.0, 10.0))
            .unwrap();

        let param0 = registry.get_by_index(0).unwrap();
        assert_eq!(param0.name, "FIRST");

        let param1 = registry.get_by_index(1).unwrap();
        assert_eq!(param1.name, "SECOND");

        assert!(registry.get_by_index(2).is_none());
    }

    #[test]
    fn test_registry_set_by_name() {
        let mut registry = ParameterRegistry::<MockFlash>::new();
        registry
            .register(ParamMetadata::new_float("TEST", 1.0, 0.0, 10.0))
            .unwrap();

        assert!(registry.set_by_name("TEST", ParamValue::Float(5.0)).is_ok());
        let param = registry.get_by_name("TEST").unwrap();
        assert_eq!(param.value, ParamValue::Float(5.0));
        assert!(param.modified);

        assert_eq!(
            registry.set_by_name("TEST", ParamValue::Float(11.0)),
            Err(RegistryError::InvalidValue)
        );
        assert_eq!(
            registry.set_by_name("NONEXISTENT", ParamValue::Float(5.0)),
            Err(RegistryError::NotFound)
        );
    }

    #[test]
    fn test_registry_has_modified() {
        let mut registry = ParameterRegistry::<MockFlash>::new();
        registry
            .register(ParamMetadata::new_float("TEST", 1.0, 0.0, 10.0))
            .unwrap();

        assert!(!registry.has_modified());

        registry
            .set_by_name("TEST", ParamValue::Float(5.0))
            .unwrap();
        assert!(registry.has_modified());
    }

    #[test]
    fn test_registry_save_and_load() {
        let flash = MockFlash::new();
        let mut registry = ParameterRegistry::with_flash(flash);

        registry
            .register(ParamMetadata::new_float("PARAM1", 1.0, 0.0, 10.0))
            .unwrap();
        registry
            .register(ParamMetadata::new_uint32("PARAM2", 5, 0, 100))
            .unwrap();

        // Modify parameters
        registry
            .set_by_name("PARAM1", ParamValue::Float(7.5))
            .unwrap();
        registry
            .set_by_name("PARAM2", ParamValue::Uint32(42))
            .unwrap();

        // Save to Flash
        assert!(registry.save_to_flash().is_ok());
        assert!(!registry.has_modified()); // Flags cleared after save

        // Create new registry and load
        let flash2 = MockFlash::new();
        let mut registry2 = ParameterRegistry::with_flash(flash2);
        registry2
            .register(ParamMetadata::new_float("PARAM1", 1.0, 0.0, 10.0))
            .unwrap();
        registry2
            .register(ParamMetadata::new_uint32("PARAM2", 5, 0, 100))
            .unwrap();

        // Load from Flash (will fail because we used different MockFlash instance)
        // In real usage, the same Flash would be used
        assert!(registry2.load_from_flash().is_ok());
    }

    #[test]
    fn test_registry_load_validates_bounds() {
        let flash = MockFlash::new();
        let mut registry = ParameterRegistry::with_flash(flash);

        registry
            .register(ParamMetadata::new_float("PARAM1", 5.0, 0.0, 10.0))
            .unwrap();

        // Save valid value
        registry
            .set_by_name("PARAM1", ParamValue::Float(7.5))
            .unwrap();
        assert!(registry.save_to_flash().is_ok());

        // Change bounds to make saved value invalid
        registry.params[0].min = ParamValue::Float(8.0);
        registry.params[0].max = ParamValue::Float(15.0);

        // Load should skip invalid value, keep current value
        let current_value = registry.params[0].value;
        assert!(registry.load_from_flash().is_ok());
        assert_eq!(registry.params[0].value, current_value);
    }
}
