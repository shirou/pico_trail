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

// =============================================================================
// Flash persistence
// =============================================================================

use crate::traits::flash::{FlashError, FlashInterface};
use heapless::Vec;

/// Serialize a ParamValue to bytes (for Flash storage)
fn serialize_value(value: &ParamValue, buf: &mut Vec<u8, 256>) -> Result<(), FlashError> {
    match value {
        ParamValue::String(s) => {
            buf.push(s.len() as u8)
                .map_err(|_| FlashError::InvalidData)?;
            buf.extend_from_slice(s.as_bytes())
                .map_err(|_| FlashError::InvalidData)?;
        }
        ParamValue::Bool(b) => {
            buf.push(if *b { 1 } else { 0 })
                .map_err(|_| FlashError::InvalidData)?;
        }
        ParamValue::Int(i) => {
            buf.extend_from_slice(&i.to_le_bytes())
                .map_err(|_| FlashError::InvalidData)?;
        }
        ParamValue::Float(f) => {
            buf.extend_from_slice(&f.to_le_bytes())
                .map_err(|_| FlashError::InvalidData)?;
        }
        ParamValue::Ipv4(ip) => {
            buf.extend_from_slice(ip)
                .map_err(|_| FlashError::InvalidData)?;
        }
    }
    Ok(())
}

/// Deserialize a ParamValue from bytes (for Flash storage)
fn deserialize_value(
    type_id: u8,
    buf: &[u8],
    offset: &mut usize,
) -> Result<ParamValue, FlashError> {
    match type_id {
        0 => {
            // String
            if *offset >= buf.len() {
                return Err(FlashError::InvalidData);
            }
            let len = buf[*offset] as usize;
            *offset += 1;

            if *offset + len > buf.len() {
                return Err(FlashError::InvalidData);
            }

            let s_str = core::str::from_utf8(&buf[*offset..*offset + len])
                .map_err(|_| FlashError::InvalidData)?;
            *offset += len;

            let mut s = String::<MAX_STRING_LEN>::new();
            s.push_str(s_str).map_err(|_| FlashError::InvalidData)?;
            Ok(ParamValue::String(s))
        }
        1 => {
            // Bool
            if *offset >= buf.len() {
                return Err(FlashError::InvalidData);
            }
            let b = buf[*offset] != 0;
            *offset += 1;
            Ok(ParamValue::Bool(b))
        }
        2 => {
            // Int
            if *offset + 4 > buf.len() {
                return Err(FlashError::InvalidData);
            }
            let i = i32::from_le_bytes([
                buf[*offset],
                buf[*offset + 1],
                buf[*offset + 2],
                buf[*offset + 3],
            ]);
            *offset += 4;
            Ok(ParamValue::Int(i))
        }
        3 => {
            // Float
            if *offset + 4 > buf.len() {
                return Err(FlashError::InvalidData);
            }
            let f = f32::from_le_bytes([
                buf[*offset],
                buf[*offset + 1],
                buf[*offset + 2],
                buf[*offset + 3],
            ]);
            *offset += 4;
            Ok(ParamValue::Float(f))
        }
        4 => {
            // Ipv4
            if *offset + 4 > buf.len() {
                return Err(FlashError::InvalidData);
            }
            let ip = [
                buf[*offset],
                buf[*offset + 1],
                buf[*offset + 2],
                buf[*offset + 3],
            ];
            *offset += 4;
            Ok(ParamValue::Ipv4(ip))
        }
        _ => Err(FlashError::InvalidData),
    }
}

/// Load parameters from a specific Flash block
fn load_from_block<F: FlashInterface>(
    flash: &mut F,
    address: u32,
) -> Result<ParameterStore, FlashError> {
    let mut buf = [0u8; PARAM_BLOCK_SIZE as usize];
    flash.read(address, &mut buf)?;

    // Validate magic
    if buf[0..4] != PARAM_MAGIC {
        return Err(FlashError::InvalidData);
    }

    // Validate version
    let version = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
    if version != PARAM_VERSION {
        return Err(FlashError::InvalidData);
    }

    // Validate CRC
    let stored_crc = u32::from_le_bytes([
        buf[PARAM_BLOCK_SIZE as usize - 4],
        buf[PARAM_BLOCK_SIZE as usize - 3],
        buf[PARAM_BLOCK_SIZE as usize - 2],
        buf[PARAM_BLOCK_SIZE as usize - 1],
    ]);

    let calculated_crc = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC)
        .checksum(&buf[0..PARAM_BLOCK_SIZE as usize - 4]);

    if stored_crc != calculated_crc {
        return Err(FlashError::InvalidData);
    }

    // Parse parameter count
    let param_count = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as usize;
    if param_count > MAX_PARAMS {
        return Err(FlashError::InvalidData);
    }

    // Deserialize parameters
    let mut store = ParameterStore::new();
    let mut offset = 12;

    for _ in 0..param_count {
        // Read name (16 bytes, null-terminated)
        if offset + PARAM_NAME_LEN > buf.len() {
            break;
        }

        let name_bytes = &buf[offset..offset + PARAM_NAME_LEN];
        let name_len = name_bytes
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(PARAM_NAME_LEN);
        let name_str =
            core::str::from_utf8(&name_bytes[..name_len]).map_err(|_| FlashError::InvalidData)?;
        let mut name = String::<PARAM_NAME_LEN>::new();
        name.push_str(name_str).ok();
        offset += PARAM_NAME_LEN;

        // Read type ID
        if offset >= buf.len() {
            break;
        }
        let type_id = buf[offset];
        offset += 1;

        // Read flags
        if offset >= buf.len() {
            break;
        }
        let flags = ParamFlags::from_bits_truncate(buf[offset]);
        offset += 1;

        // Deserialize value
        match deserialize_value(type_id, &buf, &mut offset) {
            Ok(value) => {
                store.insert_raw(name, value, flags);
            }
            Err(_) => break,
        }
    }

    Ok(store)
}

/// Load parameters from Flash
///
/// Attempts to read from all parameter blocks and uses the first valid block found.
pub fn load_from_flash<F: FlashInterface>(flash: &mut F) -> Result<ParameterStore, FlashError> {
    for block_id in 0..PARAM_BLOCK_COUNT {
        let address = PARAM_BLOCK_BASE + (block_id * PARAM_BLOCK_SIZE);

        match load_from_block(flash, address) {
            Ok(loaded_store) => {
                return Ok(loaded_store);
            }
            Err(_) => {
                continue;
            }
        }
    }

    // No valid parameter blocks found, return defaults
    Ok(ParameterStore::new())
}

/// Save parameters to Flash
///
/// Writes to the primary parameter block (block 0).
pub fn save_to_flash<F: FlashInterface>(
    store: &mut ParameterStore,
    flash: &mut F,
) -> Result<(), FlashError> {
    if !store.is_dirty() {
        return Ok(()); // No changes to save
    }

    let address = PARAM_BLOCK_BASE; // Use block 0 as primary

    // Serialize parameters
    let mut buf = [0xFFu8; PARAM_BLOCK_SIZE as usize];

    // Write magic
    buf[0..4].copy_from_slice(&PARAM_MAGIC);

    // Write version
    buf[4..8].copy_from_slice(&PARAM_VERSION.to_le_bytes());

    // Write parameter count
    let param_count = store.len() as u32;
    buf[8..12].copy_from_slice(&param_count.to_le_bytes());

    // Write parameters
    let mut offset = 12;
    let mut temp_buf = Vec::<u8, 256>::new();

    for (name, value) in store.iter_all() {
        // Write name (16 bytes, null-terminated, zero-padded)
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), PARAM_NAME_LEN);
        // Zero the name field first (buffer is 0xFF-initialized)
        buf[offset..offset + PARAM_NAME_LEN].fill(0);
        buf[offset..offset + copy_len].copy_from_slice(&name_bytes[..copy_len]);
        offset += PARAM_NAME_LEN;

        // Write type ID
        buf[offset] = value.type_id();
        offset += 1;

        // Write flags
        let metadata_flags = store
            .get_metadata(name.as_str())
            .map(|m| m.flags)
            .unwrap_or(ParamFlags::empty());
        buf[offset] = metadata_flags.bits();
        offset += 1;

        // Serialize value
        temp_buf.clear();
        serialize_value(value, &mut temp_buf)?;
        buf[offset..offset + temp_buf.len()].copy_from_slice(&temp_buf);
        offset += temp_buf.len();
    }

    // Calculate and write CRC
    let crc = crc::Crc::<u32>::new(&crc::CRC_32_ISO_HDLC)
        .checksum(&buf[0..PARAM_BLOCK_SIZE as usize - 4]);
    buf[PARAM_BLOCK_SIZE as usize - 4..].copy_from_slice(&crc.to_le_bytes());

    // Erase block
    flash.erase(address, PARAM_BLOCK_SIZE)?;

    // Write block
    flash.write(address, &buf)?;

    store.clear_dirty();

    Ok(())
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

    extern crate alloc;

    /// Simple in-memory Flash mock for core tests
    struct TestFlash {
        data: alloc::vec::Vec<u8>,
    }

    impl TestFlash {
        fn new() -> Self {
            Self {
                data: alloc::vec![0xFF; 4 * 1024 * 1024], // 4 MB
            }
        }
    }

    impl crate::traits::flash::FlashInterface for TestFlash {
        fn read(
            &mut self,
            address: u32,
            buf: &mut [u8],
        ) -> Result<(), crate::traits::flash::FlashError> {
            let addr = address as usize;
            buf.copy_from_slice(&self.data[addr..addr + buf.len()]);
            Ok(())
        }
        fn write(
            &mut self,
            address: u32,
            data: &[u8],
        ) -> Result<(), crate::traits::flash::FlashError> {
            let addr = address as usize;
            for (i, &byte) in data.iter().enumerate() {
                self.data[addr + i] &= byte; // Flash behavior: can only clear bits
            }
            Ok(())
        }
        fn erase(
            &mut self,
            address: u32,
            size: u32,
        ) -> Result<(), crate::traits::flash::FlashError> {
            let addr = address as usize;
            for i in 0..size as usize {
                self.data[addr + i] = 0xFF;
            }
            Ok(())
        }
        fn block_size(&self) -> u32 {
            4096
        }
        fn capacity(&self) -> u32 {
            4 * 1024 * 1024
        }
    }

    #[test]
    fn test_flash_round_trip() {
        let mut store = ParameterStore::new();
        store
            .register("TEST_INT", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();
        store.set("TEST_INT", ParamValue::Int(99)).unwrap();

        let mut flash = TestFlash::new();
        save_to_flash(&mut store, &mut flash).unwrap();
        assert!(!store.is_dirty());

        let loaded = load_from_flash(&mut flash).unwrap();
        assert_eq!(loaded.get("TEST_INT"), Some(&ParamValue::Int(99)));
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
