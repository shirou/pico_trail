//! Parameter block format for Flash storage
//!
//! This module defines the binary format for parameter blocks stored in Flash.
//! Each block contains a header, parameter data, and a CRC32 checksum.

use bitflags::bitflags;

/// Parameter block magic number (ASCII "PARA")
pub const PARAM_MAGIC: u32 = 0x50415241;

/// Parameter format version
pub const PARAM_VERSION: u16 = 1;

/// Maximum number of parameters per flash block
pub const MAX_PARAMS_PER_BLOCK: usize = 200;

/// Parameter block header
///
/// This structure is stored at the beginning of each Flash block.
/// It provides metadata for validating and versioning parameter data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct ParameterBlockHeader {
    /// Magic number (0x50415241 = "PARA")
    pub magic: u32,
    /// Parameter format version
    pub version: u16,
    /// Sequence number (increments on each write, for wear leveling)
    pub sequence: u16,
    /// Number of parameters in this block
    pub param_count: u16,
    /// Reserved for future use
    pub reserved: u16,
}

impl ParameterBlockHeader {
    /// Size of header in bytes
    pub const SIZE: usize = core::mem::size_of::<Self>();

    /// Create a new header
    pub fn new(sequence: u16, param_count: u16) -> Self {
        Self {
            magic: PARAM_MAGIC,
            version: PARAM_VERSION,
            sequence,
            param_count,
            reserved: 0,
        }
    }

    /// Serialize header to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.magic.to_le_bytes());
        buf[4..6].copy_from_slice(&self.version.to_le_bytes());
        buf[6..8].copy_from_slice(&self.sequence.to_le_bytes());
        buf[8..10].copy_from_slice(&self.param_count.to_le_bytes());
        buf[10..12].copy_from_slice(&self.reserved.to_le_bytes());
        buf
    }

    /// Deserialize header from bytes (little-endian)
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }

        let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let version = u16::from_le_bytes([buf[4], buf[5]]);
        let sequence = u16::from_le_bytes([buf[6], buf[7]]);
        let param_count = u16::from_le_bytes([buf[8], buf[9]]);
        let reserved = u16::from_le_bytes([buf[10], buf[11]]);

        // Validate magic number
        if magic != PARAM_MAGIC {
            return None;
        }

        Some(Self {
            magic,
            version,
            sequence,
            param_count,
            reserved,
        })
    }

    /// Check if this header is valid
    pub fn is_valid(&self) -> bool {
        self.magic == PARAM_MAGIC
            && self.version == PARAM_VERSION
            && self.param_count <= MAX_PARAMS_PER_BLOCK as u16
    }
}

/// Parameter entry
///
/// Each parameter is stored as a name hash, value, and metadata.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Parameter {
    /// Hash of parameter name (FNV-1a hash for fast lookup)
    pub name_hash: u32,
    /// Parameter value (f32 or u32, discriminated by flags)
    pub value: f32,
    /// Parameter type and flags
    pub flags: ParameterFlags,
    /// Reserved for future use
    pub reserved: u16,
}

impl Parameter {
    /// Size of parameter entry in bytes
    pub const SIZE: usize = core::mem::size_of::<Self>();

    /// Create a new f32 parameter
    pub fn new_f32(name_hash: u32, value: f32) -> Self {
        Self {
            name_hash,
            value,
            flags: ParameterFlags::TYPE_F32,
            reserved: 0,
        }
    }

    /// Create a new u32 parameter
    pub fn new_u32(name_hash: u32, value: u32) -> Self {
        Self {
            name_hash,
            value: f32::from_bits(value),
            flags: ParameterFlags::TYPE_U32,
            reserved: 0,
        }
    }

    /// Get value as f32
    pub fn as_f32(&self) -> Option<f32> {
        if self.flags.contains(ParameterFlags::TYPE_F32) {
            Some(self.value)
        } else {
            None
        }
    }

    /// Get value as u32
    pub fn as_u32(&self) -> Option<u32> {
        if self.flags.contains(ParameterFlags::TYPE_U32) {
            Some(self.value.to_bits())
        } else {
            None
        }
    }

    /// Serialize parameter to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.name_hash.to_le_bytes());
        buf[4..8].copy_from_slice(&self.value.to_bits().to_le_bytes());
        buf[8..10].copy_from_slice(&self.flags.bits().to_le_bytes());
        buf[10..12].copy_from_slice(&self.reserved.to_le_bytes());
        buf
    }

    /// Deserialize parameter from bytes (little-endian)
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }

        let name_hash = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let value_bits = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let flags_bits = u16::from_le_bytes([buf[8], buf[9]]);
        let reserved = u16::from_le_bytes([buf[10], buf[11]]);

        let value = f32::from_bits(value_bits);
        let flags = ParameterFlags::from_bits_truncate(flags_bits);

        Some(Self {
            name_hash,
            value,
            flags,
            reserved,
        })
    }
}

bitflags! {
    /// Parameter type and status flags
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ParameterFlags: u16 {
        /// Parameter type: f32 floating-point
        const TYPE_F32 = 0b0001;
        /// Parameter type: u32 unsigned integer
        const TYPE_U32 = 0b0010;
        /// Parameter has been modified (not yet saved)
        const MODIFIED = 0b0100;
        /// Parameter is read-only
        const READ_ONLY = 0b1000;
    }
}

/// Calculate FNV-1a hash of parameter name
///
/// Used for fast parameter lookup by name hash.
pub fn hash_param_name(name: &str) -> u32 {
    const FNV_OFFSET_BASIS: u32 = 2166136261;
    const FNV_PRIME: u32 = 16777619;

    let mut hash = FNV_OFFSET_BASIS;
    for byte in name.bytes() {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_header_serialization() {
        let header = ParameterBlockHeader::new(42, 10);
        let bytes = header.to_bytes();
        let decoded = ParameterBlockHeader::from_bytes(&bytes).unwrap();

        assert_eq!(header, decoded);
        assert!(decoded.is_valid());
    }

    #[test]
    fn test_header_validation() {
        let mut header = ParameterBlockHeader::new(0, 0);
        assert!(header.is_valid());

        // Invalid magic
        header.magic = 0xDEADBEEF;
        assert!(!header.is_valid());

        // Too many params
        header.magic = PARAM_MAGIC;
        header.param_count = (MAX_PARAMS_PER_BLOCK + 1) as u16;
        assert!(!header.is_valid());
    }

    #[test]
    fn test_parameter_f32_serialization() {
        let param = Parameter::new_f32(0x12345678, core::f32::consts::PI);
        let bytes = param.to_bytes();
        let decoded = Parameter::from_bytes(&bytes).unwrap();

        assert_eq!(param.name_hash, decoded.name_hash);
        assert_eq!(param.as_f32().unwrap(), decoded.as_f32().unwrap());
        assert!(decoded.flags.contains(ParameterFlags::TYPE_F32));
    }

    #[test]
    fn test_parameter_u32_serialization() {
        let param = Parameter::new_u32(0xABCDEF00, 12345);
        let bytes = param.to_bytes();
        let decoded = Parameter::from_bytes(&bytes).unwrap();

        assert_eq!(param.name_hash, decoded.name_hash);
        assert_eq!(param.as_u32().unwrap(), decoded.as_u32().unwrap());
        assert!(decoded.flags.contains(ParameterFlags::TYPE_U32));
    }

    #[test]
    fn test_hash_param_name() {
        let hash1 = hash_param_name("WP_SPEED");
        let hash2 = hash_param_name("WP_SPEED");
        let hash3 = hash_param_name("WP_ACCEL");

        // Same name produces same hash
        assert_eq!(hash1, hash2);
        // Different names produce different hashes
        assert_ne!(hash1, hash3);
    }

    #[test]
    fn test_parameter_type_discrimination() {
        let f32_param = Parameter::new_f32(0, 1.5);
        let u32_param = Parameter::new_u32(0, 42);

        assert!(f32_param.as_f32().is_some());
        assert!(f32_param.as_u32().is_none());

        assert!(u32_param.as_u32().is_some());
        assert!(u32_param.as_f32().is_none());
    }
}
