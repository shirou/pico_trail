//! Flash-backed Parameter Storage
//!
//! Provides persistent parameter storage using Flash memory with CRC validation
//! and redundant block support for reliability.
//!
//! # Flash Block Format
//!
//! ```text
//! ┌───────────────────────────────────────────────┐
//! │ Magic: [u8; 4] = b"PARA"                     │  Offset: 0
//! ├───────────────────────────────────────────────┤
//! │ Version: u32 = 1                              │  Offset: 4
//! ├───────────────────────────────────────────────┤
//! │ Parameter Count: u32                          │  Offset: 8
//! ├───────────────────────────────────────────────┤
//! │ Parameters: [(name, value)]                   │  Offset: 12
//! │   name: [u8; 16] (null-terminated)           │
//! │   type: u8 (0=String, 1=Bool, 2=Int, etc.)   │
//! │   value: Variable size based on type          │
//! ├───────────────────────────────────────────────┤
//! │ CRC32: u32                                    │  Offset: block_size - 4
//! └───────────────────────────────────────────────┘
//! ```
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::parameters::ParameterStore;
//! use pico_trail::platform::rp2350::Rp2350Flash;
//!
//! let mut flash = Rp2350Flash::new();
//! let mut store = ParameterStore::load_from_flash(&mut flash).unwrap();
//!
//! // Access parameters
//! if let Some(value) = store.get("NET_SSID") {
//!     // Use value
//! }
//!
//! // Modify parameters
//! store.set("NET_SSID", ParamValue::String("MyNetwork".into())).unwrap();
//! store.save_to_flash(&mut flash).unwrap();
//! ```

use crate::platform::traits::FlashInterface;
use crate::platform::Result;
use bitflags::bitflags;
use heapless::{FnvIndexMap, String, Vec};

/// Parameter block base address (Flash offset)
const PARAM_BLOCK_BASE: u32 = 0x040000; // 256 KB

/// Parameter block size (4 KB per block)
const PARAM_BLOCK_SIZE: u32 = 4096;

/// Number of parameter blocks (for redundancy)
const PARAM_BLOCK_COUNT: u32 = 4;

/// Magic number for parameter blocks ("PARA")
const PARAM_MAGIC: [u8; 4] = *b"PARA";

/// Parameter format version
const PARAM_VERSION: u32 = 1;

/// Maximum parameter name length
const PARAM_NAME_LEN: usize = 16;

/// Maximum number of parameters
const MAX_PARAMS: usize = 64;

/// Maximum string parameter length
pub const MAX_STRING_LEN: usize = 63;

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
    fn type_id(&self) -> u8 {
        match self {
            ParamValue::String(_) => 0,
            ParamValue::Bool(_) => 1,
            ParamValue::Int(_) => 2,
            ParamValue::Float(_) => 3,
            ParamValue::Ipv4(_) => 4,
        }
    }

    /// Serialize value to bytes
    fn serialize(&self, buf: &mut Vec<u8, 256>) -> Result<()> {
        match self {
            ParamValue::String(s) => {
                // Write length (1 byte) + string bytes
                buf.push(s.len() as u8)
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
                buf.extend_from_slice(s.as_bytes())
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            }
            ParamValue::Bool(b) => {
                buf.push(if *b { 1 } else { 0 })
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            }
            ParamValue::Int(i) => {
                buf.extend_from_slice(&i.to_le_bytes())
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            }
            ParamValue::Float(f) => {
                buf.extend_from_slice(&f.to_le_bytes())
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            }
            ParamValue::Ipv4(ip) => {
                buf.extend_from_slice(ip)
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            }
        }
        Ok(())
    }

    /// Deserialize value from bytes
    fn deserialize(type_id: u8, buf: &[u8], offset: &mut usize) -> Result<Self> {
        match type_id {
            0 => {
                // String
                if *offset >= buf.len() {
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
                }
                let len = buf[*offset] as usize;
                *offset += 1;

                if *offset + len > buf.len() {
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
                }

                let s_str = core::str::from_utf8(&buf[*offset..*offset + len])
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
                *offset += len;

                let mut s = String::new();
                s.push_str(s_str)
                    .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
                Ok(ParamValue::String(s))
            }
            1 => {
                // Bool
                if *offset >= buf.len() {
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
                }
                let b = buf[*offset] != 0;
                *offset += 1;
                Ok(ParamValue::Bool(b))
            }
            2 => {
                // Int
                if *offset + 4 > buf.len() {
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
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
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
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
                    return Err(crate::platform::error::PlatformError::InvalidConfig);
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
            _ => Err(crate::platform::error::PlatformError::InvalidConfig),
        }
    }
}

/// Parameter metadata
#[derive(Debug, Clone)]
struct ParamMetadata {
    /// Parameter flags
    flags: ParamFlags,
}

/// Parameter store with Flash persistence
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

    /// Load parameters from Flash
    ///
    /// Attempts to read from all parameter blocks and uses the most recent valid block.
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface
    ///
    /// # Returns
    ///
    /// Parameter store loaded from Flash, or empty store if no valid blocks found.
    pub fn load_from_flash<F: FlashInterface>(flash: &mut F) -> Result<Self> {
        // Try loading from each parameter block (most recent first)
        for block_id in 0..PARAM_BLOCK_COUNT {
            let address = PARAM_BLOCK_BASE + (block_id * PARAM_BLOCK_SIZE);

            match Self::load_from_block(flash, address) {
                Ok(loaded_store) => {
                    #[cfg(feature = "pico2_w")]
                    defmt::info!("Loaded parameters from block {}", block_id);
                    return Ok(loaded_store);
                }
                Err(_) => {
                    // Try next block
                    continue;
                }
            }
        }

        #[cfg(feature = "pico2_w")]
        defmt::warn!("No valid parameter blocks found, using defaults");

        Ok(Self::new())
    }

    /// Load parameters from a specific Flash block
    fn load_from_block<F: FlashInterface>(flash: &mut F, address: u32) -> Result<Self> {
        let mut buf = [0u8; PARAM_BLOCK_SIZE as usize];
        flash.read(address, &mut buf)?;

        // Validate magic
        if buf[0..4] != PARAM_MAGIC {
            return Err(crate::platform::error::PlatformError::InvalidConfig);
        }

        // Validate version
        let version = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        if version != PARAM_VERSION {
            return Err(crate::platform::error::PlatformError::InvalidConfig);
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
            return Err(crate::platform::error::PlatformError::InvalidConfig);
        }

        // Parse parameter count
        let param_count = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]) as usize;
        if param_count > MAX_PARAMS {
            return Err(crate::platform::error::PlatformError::InvalidConfig);
        }

        // Deserialize parameters
        let mut store = Self::new();
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
            let name_str = core::str::from_utf8(&name_bytes[..name_len])
                .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
            let mut name = String::new();
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
            match ParamValue::deserialize(type_id, &buf, &mut offset) {
                Ok(value) => {
                    store.parameters.insert(name.clone(), value).ok();
                    store.metadata.insert(name, ParamMetadata { flags }).ok();
                }
                Err(_) => break,
            }
        }

        Ok(store)
    }

    /// Save parameters to Flash
    ///
    /// Writes to the primary parameter block (block 0).
    ///
    /// # Arguments
    ///
    /// * `flash` - Flash interface
    ///
    /// # Returns
    ///
    /// Ok if saved successfully
    pub fn save_to_flash<F: FlashInterface>(&mut self, flash: &mut F) -> Result<()> {
        if !self.dirty {
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
        let param_count = self.parameters.len() as u32;
        buf[8..12].copy_from_slice(&param_count.to_le_bytes());

        // Write parameters
        let mut offset = 12;
        let mut temp_buf = Vec::<u8, 256>::new();

        for (name, value) in &self.parameters {
            // Write name (16 bytes, null-terminated)
            let name_bytes = name.as_bytes();
            let copy_len = core::cmp::min(name_bytes.len(), PARAM_NAME_LEN);
            buf[offset..offset + copy_len].copy_from_slice(&name_bytes[..copy_len]);
            offset += PARAM_NAME_LEN;

            // Write type ID
            buf[offset] = value.type_id();
            offset += 1;

            // Write flags
            let metadata = self
                .metadata
                .get(name)
                .map(|m| m.flags)
                .unwrap_or(ParamFlags::empty());
            buf[offset] = metadata.bits();
            offset += 1;

            // Serialize value
            temp_buf.clear();
            value.serialize(&mut temp_buf)?;
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

        self.dirty = false;

        #[cfg(feature = "pico2_w")]
        defmt::info!("Saved {} parameters to Flash", param_count);

        Ok(())
    }

    /// Get parameter value
    ///
    /// # Arguments
    ///
    /// * `name` - Parameter name
    ///
    /// # Returns
    ///
    /// Parameter value if found
    pub fn get(&self, name: &str) -> Option<&ParamValue> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name).ok()?;
        self.parameters.get(&key)
    }

    /// Set parameter value
    ///
    /// Marks the store as dirty (needs Flash write).
    ///
    /// # Arguments
    ///
    /// * `name` - Parameter name
    /// * `value` - New parameter value
    ///
    /// # Returns
    ///
    /// Ok if set successfully
    pub fn set(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name)
            .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;

        // Check if parameter exists
        if !self.parameters.contains_key(&key) {
            return Err(crate::platform::error::PlatformError::InvalidConfig);
        }

        // Check if read-only
        if let Some(meta) = self.metadata.get(&key) {
            if meta.flags.contains(ParamFlags::READ_ONLY) {
                return Err(crate::platform::error::PlatformError::InvalidConfig);
            }
        }

        self.parameters.insert(key, value).ok();
        self.dirty = true;
        Ok(())
    }

    /// Register a new parameter with default value and flags
    ///
    /// # Arguments
    ///
    /// * `name` - Parameter name
    /// * `default_value` - Default value
    /// * `flags` - Parameter flags
    ///
    /// # Returns
    ///
    /// Ok if registered successfully
    pub fn register(
        &mut self,
        name: &str,
        default_value: ParamValue,
        flags: ParamFlags,
    ) -> Result<()> {
        let mut key = String::<PARAM_NAME_LEN>::new();
        key.push_str(name)
            .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;

        if self.parameters.contains_key(&key) {
            // Already exists, don't overwrite
            return Ok(());
        }

        self.parameters
            .insert(key.clone(), default_value)
            .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
        self.metadata
            .insert(key, ParamMetadata { flags })
            .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
        self.dirty = true;
        Ok(())
    }

    /// Check if parameter is hidden
    ///
    /// Hidden parameters (e.g., NET_PASS) are not readable via MAVLink.
    ///
    /// # Arguments
    ///
    /// * `name` - Parameter name
    ///
    /// # Returns
    ///
    /// true if parameter is hidden
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
    ///
    /// # Returns
    ///
    /// Iterator over parameter names
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
    fn test_param_value_serialization() {
        let mut buf = Vec::<u8, 256>::new();

        // Test String
        let s = ParamValue::String(String::try_from("test").unwrap());
        s.serialize(&mut buf).unwrap();
        assert_eq!(buf.len(), 5); // 1 byte length + 4 bytes data

        // Test Bool
        buf.clear();
        let b = ParamValue::Bool(true);
        b.serialize(&mut buf).unwrap();
        assert_eq!(buf.len(), 1);

        // Test Int
        buf.clear();
        let i = ParamValue::Int(42);
        i.serialize(&mut buf).unwrap();
        assert_eq!(buf.len(), 4);

        // Test Float
        buf.clear();
        let f = ParamValue::Float(3.14);
        f.serialize(&mut buf).unwrap();
        assert_eq!(buf.len(), 4);

        // Test Ipv4
        buf.clear();
        let ip = ParamValue::Ipv4([192, 168, 1, 1]);
        ip.serialize(&mut buf).unwrap();
        assert_eq!(buf.len(), 4);
    }

    #[test]
    fn test_parameter_store_basic() {
        let mut store = ParameterStore::new();

        // Register parameter
        store
            .register("TEST", ParamValue::Int(42), ParamFlags::empty())
            .unwrap();

        // Get parameter
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(42)));

        // Set parameter
        store.set("TEST", ParamValue::Int(100)).unwrap();
        assert_eq!(store.get("TEST"), Some(&ParamValue::Int(100)));
        assert!(store.is_dirty());
    }

    #[test]
    fn test_parameter_hidden() {
        let mut store = ParameterStore::new();

        // Register hidden parameter
        store
            .register(
                "SECRET",
                ParamValue::String(String::try_from("password").unwrap()),
                ParamFlags::HIDDEN,
            )
            .unwrap();

        // Check if hidden
        assert!(store.is_hidden("SECRET"));
        assert_eq!(store.count(), 0); // Hidden parameters not counted
    }

    #[test]
    fn test_parameter_read_only() {
        let mut store = ParameterStore::new();

        // Register read-only parameter
        store
            .register("READONLY", ParamValue::Int(42), ParamFlags::READ_ONLY)
            .unwrap();

        // Try to set (should fail)
        assert!(store.set("READONLY", ParamValue::Int(100)).is_err());
    }
}
