//! Parameter Storage Types
//!
//! Provides core parameter types and the `ParameterStore` for configuration management.
//! Flash persistence methods are in the firmware crate.

use super::error::ParameterError;
use bitflags::bitflags;
use heapless::index_map::FnvIndexMap;
use heapless::String;

/// Maximum parameter name length
pub const PARAM_NAME_LEN: usize = 16;

/// Maximum number of parameters
pub const MAX_PARAMS: usize = 64;

/// Maximum string parameter length
pub const MAX_STRING_LEN: usize = 63;

/// Parameter block base address (Flash offset)
pub const PARAM_BLOCK_BASE: u32 = 0x040000; // 256 KB

/// Parameter block size (4 KB per block)
pub const PARAM_BLOCK_SIZE: u32 = 4096;

/// Number of parameter blocks (for redundancy)
pub const PARAM_BLOCK_COUNT: u32 = 4;

/// Magic number for parameter blocks ("PARA")
pub const PARAM_MAGIC: [u8; 4] = *b"PARA";

/// Parameter format version
pub const PARAM_VERSION: u32 = 1;

bitflags! {
    /// Parameter flags
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ParamFlags: u8 {
        /// Parameter is hidden from MAVLink PARAM_REQUEST_READ/LIST
        const HIDDEN = 0b00000001;
        /// Parameter is read-only (cannot be modified via MAVLink)
        const READ_ONLY = 0b00000010;
    }
}

/// Parameter value types
#[derive(Debug, Clone, PartialEq)]
pub enum ParamValue {
    /// String parameter (max 63 chars)
    String(String<MAX_STRING_LEN>),
    /// Boolean parameter
    Bool(bool),
    /// 32-bit signed integer
    Int(i32),
    /// 32-bit floating point
    Float(f32),
    /// IPv4 address (4 bytes)
    Ipv4([u8; 4]),
}

impl ParamValue {
    /// Get type discriminant for serialization
    pub fn type_id(&self) -> u8 {
        match self {
            ParamValue::String(_) => 0,
            ParamValue::Bool(_) => 1,
            ParamValue::Int(_) => 2,
            ParamValue::Float(_) => 3,
            ParamValue::Ipv4(_) => 4,
        }
    }
}

/// Parameter metadata
#[derive(Debug, Clone)]
pub struct ParamMetadata {
    /// Parameter flags
    pub flags: ParamFlags,
}

/// Parameter store for configuration management
///
/// Stores parameters as key-value pairs with metadata (flags).
/// Flash persistence is handled by free functions in the firmware crate.
pub struct ParameterStore {
    /// Parameter values
    parameters: FnvIndexMap<String<PARAM_NAME_LEN>, ParamValue, MAX_PARAMS>,
    /// Parameter metadata
    metadata: FnvIndexMap<String<PARAM_NAME_LEN>, ParamMetadata, MAX_PARAMS>,
    /// Dirty flag (needs Flash write)
    dirty: bool,
}

impl ParameterStore {
    /// Create a new empty parameter store
    pub fn new() -> Self {
        Self {
            parameters: FnvIndexMap::new(),
            metadata: FnvIndexMap::new(),
            dirty: false,
        }
    }

    /// Get parameter value
    pub fn get(&self, name: &str) -> Option<&ParamValue> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name).ok()?;
        self.parameters.get(&key)
    }

    /// Set parameter value
    ///
    /// Marks the store as dirty (needs Flash write).
    pub fn set(&mut self, name: &str, value: ParamValue) -> Result<(), ParameterError> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name)
            .map_err(|_| ParameterError::InvalidConfig)?;

        // Check if parameter exists
        if !self.parameters.contains_key(&key) {
            return Err(ParameterError::InvalidConfig);
        }

        // Check if read-only
        if let Some(meta) = self.metadata.get(&key) {
            if meta.flags.contains(ParamFlags::READ_ONLY) {
                return Err(ParameterError::ReadOnly);
            }
        }

        self.parameters.insert(key, value).ok();
        self.dirty = true;
        Ok(())
    }

    /// Register a new parameter with default value and flags
    ///
    /// If the parameter already exists, this is a no-op (idempotent).
    pub fn register(
        &mut self,
        name: &str,
        default_value: ParamValue,
        flags: ParamFlags,
    ) -> Result<(), ParameterError> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name)
            .map_err(|_| ParameterError::InvalidConfig)?;

        if self.parameters.contains_key(&key) {
            // Already exists, don't overwrite
            return Ok(());
        }

        self.parameters
            .insert(key.clone(), default_value)
            .map_err(|_| ParameterError::StoreFull)?;
        self.metadata
            .insert(key, ParamMetadata { flags })
            .map_err(|_| ParameterError::StoreFull)?;
        self.dirty = true;
        Ok(())
    }

    /// Check if parameter is hidden
    ///
    /// Hidden parameters (e.g., NET_PASS) are not readable via MAVLink.
    pub fn is_hidden(&self, name: &str) -> bool {
        let mut key = String::<PARAM_NAME_LEN>::new();
        if key.push_str(name).is_err() {
            return false;
        }
        if let Some(meta) = self.metadata.get(&key) {
            meta.flags.contains(ParamFlags::HIDDEN)
        } else {
            false
        }
    }

    /// Get all parameter names (excluding hidden parameters)
    pub fn iter_names(&self) -> impl Iterator<Item = &String<PARAM_NAME_LEN>> {
        self.parameters
            .keys()
            .filter(|name| !self.is_hidden(name.as_str()))
    }

    /// Get parameter count (excluding hidden parameters)
    pub fn count(&self) -> usize {
        self.iter_names().count()
    }

    /// Check if store has unsaved changes
    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    /// Clear dirty flag (called after successful flash save)
    pub fn clear_dirty(&mut self) {
        self.dirty = false;
    }

    /// Get total parameter count (including hidden parameters)
    pub fn len(&self) -> usize {
        self.parameters.len()
    }

    /// Check if the store is empty
    pub fn is_empty(&self) -> bool {
        self.parameters.is_empty()
    }

    /// Iterate over all parameters (including hidden) as (name, value) pairs
    ///
    /// Used by Flash persistence for serialization.
    pub fn iter_all(&self) -> impl Iterator<Item = (&String<PARAM_NAME_LEN>, &ParamValue)> {
        self.parameters.iter()
    }

    /// Get metadata for a parameter by name
    pub fn get_metadata(&self, name: &str) -> Option<&ParamMetadata> {
        let mut key: String<PARAM_NAME_LEN> = String::new();
        key.push_str(name).ok()?;
        self.metadata.get(&key)
    }

    /// Insert a parameter directly without validation
    ///
    /// Used by Flash persistence for deserialization. Bypasses read-only
    /// and existence checks since data comes from a trusted source (Flash).
    pub fn insert_raw(
        &mut self,
        name: String<PARAM_NAME_LEN>,
        value: ParamValue,
        flags: ParamFlags,
    ) {
        self.parameters.insert(name.clone(), value).ok();
        self.metadata.insert(name, ParamMetadata { flags }).ok();
    }
}

impl Default for ParameterStore {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_value_type_id() {
        assert_eq!(
            ParamValue::String(String::try_from("test").unwrap()).type_id(),
            0
        );
        assert_eq!(ParamValue::Bool(true).type_id(), 1);
        assert_eq!(ParamValue::Int(42).type_id(), 2);
        assert_eq!(ParamValue::Float(1.0).type_id(), 3);
        assert_eq!(ParamValue::Ipv4([0; 4]).type_id(), 4);
    }

    #[test]
    fn test_parameter_store_new() {
        let store = ParameterStore::new();
        assert_eq!(store.count(), 0);
        assert!(!store.is_dirty());
    }

    #[test]
    fn test_parameter_store_register_and_get() {
        let mut store = ParameterStore::new();
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(42)));
    }

    #[test]
    fn test_parameter_store_set() {
        let mut store = ParameterStore::new();
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        store.set("TEST", ParamValue::Int(100)).unwrap();
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(100)));
        assert!(store.is_dirty());
    }

    #[test]
    fn test_parameter_store_set_unknown() {
        let mut store = ParameterStore::new();
        assert!(store.set("UNKNOWN", ParamValue::Int(1)).is_err());
    }

    #[test]
    fn test_parameter_store_register_idempotent() {
        let mut store = ParameterStore::new();
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        store.set("TEST", ParamValue::Int(100)).unwrap();
        // Re-register should not overwrite
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(100)));
    }

    #[test]
    fn test_parameter_store_dirty() {
        let mut store = ParameterStore::new();
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        assert!(store.is_dirty());
        store.clear_dirty();
        assert!(!store.is_dirty());
        store.set("TEST", ParamValue::Int(100)).unwrap();
        assert!(store.is_dirty());
    }

    #[test]
    fn test_parameter_store_count() {
        let mut store = ParameterStore::new();
        store
            .register("A", ParamValue::Int(1), ParamFlags::empty())
            .unwrap();
        store
            .register("B", ParamValue::Int(2), ParamFlags::empty())
            .unwrap();
        assert_eq!(store.count(), 2);
    }

    #[test]
    fn test_parameter_store_iter_names() {
        let mut store = ParameterStore::new();
        store
            .register("A", ParamValue::Int(1), ParamFlags::empty())
            .unwrap();
        store
            .register("B", ParamValue::Int(2), ParamFlags::empty())
            .unwrap();
        let names: heapless::Vec<_, MAX_PARAMS> = store.iter_names().collect();

        assert_eq!(names.len(), 2);
    }

    #[test]
    fn test_parameter_hidden() {
        let mut store = ParameterStore::new();
        store
            .register(
                "SECRET",
                ParamValue::String(String::try_from("password").unwrap()),
                ParamFlags::HIDDEN,
            )
            .unwrap();
        assert!(store.is_hidden("SECRET"));
        assert_eq!(store.count(), 0); // Hidden parameters not counted
    }

    #[test]
    fn test_parameter_read_only() {
        let mut store = ParameterStore::new();
        store
            .register("READONLY", ParamValue::Int(42), ParamFlags::READ_ONLY)
            .unwrap();
        assert!(store.set("READONLY", ParamValue::Int(100)).is_err());
    }

    #[test]
    fn test_param_value_equality() {
        // Same variant, same value
        assert_eq!(ParamValue::Float(1.0), ParamValue::Float(1.0));
        assert_eq!(ParamValue::Int(42), ParamValue::Int(42));
        assert_eq!(ParamValue::Bool(true), ParamValue::Bool(true));
        assert_eq!(
            ParamValue::Ipv4([192, 168, 1, 1]),
            ParamValue::Ipv4([192, 168, 1, 1])
        );
        assert_eq!(
            ParamValue::String(String::try_from("abc").unwrap()),
            ParamValue::String(String::try_from("abc").unwrap())
        );

        // Same variant, different value
        assert_ne!(ParamValue::Int(1), ParamValue::Int(2));
        assert_ne!(ParamValue::Float(1.0), ParamValue::Float(2.0));
        assert_ne!(ParamValue::Bool(true), ParamValue::Bool(false));

        // Different variants
        assert_ne!(ParamValue::Int(1), ParamValue::Float(1.0));
    }
}
