//! Flash-backed Parameter Storage
//!
//! Provides Flash persistence for the parameter store.
//! Core types (`ParameterStore`, `ParamValue`, `ParamFlags`, `ParamMetadata`)
//! are defined in `pico_trail_core::parameters::storage`.
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

use crate::platform::traits::FlashInterface;
use crate::platform::Result;
use heapless::{String, Vec};
pub use pico_trail_core::parameters::storage::{
    ParamFlags, ParamValue, ParameterStore, MAX_PARAMS, MAX_STRING_LEN, PARAM_BLOCK_BASE,
    PARAM_BLOCK_COUNT, PARAM_BLOCK_SIZE, PARAM_MAGIC, PARAM_NAME_LEN, PARAM_VERSION,
};
pub use pico_trail_core::parameters::ParamMetadata;

/// Serialize a ParamValue to bytes (for Flash storage)
fn serialize_value(value: &ParamValue, buf: &mut Vec<u8, 256>) -> Result<()> {
    match value {
        ParamValue::String(s) => {
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

/// Deserialize a ParamValue from bytes (for Flash storage)
fn deserialize_value(type_id: u8, buf: &[u8], offset: &mut usize) -> Result<ParamValue> {
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

            let mut s = String::<MAX_STRING_LEN>::new();
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

/// Load parameters from Flash
///
/// Attempts to read from all parameter blocks and uses the most recent valid block.
pub fn load_from_flash<F: FlashInterface>(flash: &mut F) -> Result<ParameterStore> {
    for block_id in 0..PARAM_BLOCK_COUNT {
        let address = PARAM_BLOCK_BASE + (block_id * PARAM_BLOCK_SIZE);

        match load_from_block(flash, address) {
            Ok(loaded_store) => {
                crate::log_info!("Loaded parameters from block {}", block_id);
                return Ok(loaded_store);
            }
            Err(_) => {
                continue;
            }
        }
    }

    crate::log_warn!("No valid parameter blocks found, using defaults");
    Ok(ParameterStore::new())
}

/// Load parameters from a specific Flash block
fn load_from_block<F: FlashInterface>(flash: &mut F, address: u32) -> Result<ParameterStore> {
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
        let name_str = core::str::from_utf8(&name_bytes[..name_len])
            .map_err(|_| crate::platform::error::PlatformError::InvalidConfig)?;
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
pub fn save_to_flash<F: FlashInterface>(store: &mut ParameterStore, flash: &mut F) -> Result<()> {
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
    let param_count = store.parameters.len() as u32;
    buf[8..12].copy_from_slice(&param_count.to_le_bytes());

    // Write parameters
    let mut offset = 12;
    let mut temp_buf = Vec::<u8, 256>::new();

    for (name, value) in &store.parameters {
        // Write name (16 bytes, null-terminated)
        let name_bytes = name.as_bytes();
        let copy_len = core::cmp::min(name_bytes.len(), PARAM_NAME_LEN);
        buf[offset..offset + copy_len].copy_from_slice(&name_bytes[..copy_len]);
        offset += PARAM_NAME_LEN;

        // Write type ID
        buf[offset] = value.type_id();
        offset += 1;

        // Write flags
        let metadata_flags = store
            .metadata
            .get(name)
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

    crate::log_info!("Saved {} parameters to Flash", param_count);

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_param_value_serialization() {
        let mut buf = Vec::<u8, 256>::new();

        // Test String
        let s = ParamValue::String(String::try_from("test").unwrap());
        serialize_value(&s, &mut buf).unwrap();
        assert_eq!(buf.len(), 5); // 1 byte length + 4 bytes data

        // Test Bool
        buf.clear();
        let b = ParamValue::Bool(true);
        serialize_value(&b, &mut buf).unwrap();
        assert_eq!(buf.len(), 1);

        // Test Int
        buf.clear();
        let i = ParamValue::Int(42);
        serialize_value(&i, &mut buf).unwrap();
        assert_eq!(buf.len(), 4);

        // Test Float
        buf.clear();
        let f = ParamValue::Float(1.23);
        serialize_value(&f, &mut buf).unwrap();
        assert_eq!(buf.len(), 4);

        // Test Ipv4
        buf.clear();
        let ip = ParamValue::Ipv4([192, 168, 1, 1]);
        serialize_value(&ip, &mut buf).unwrap();
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
