//! CRC32 calculation for parameter block validation
//!
//! This module provides CRC32 checksum calculation for detecting
//! corrupted parameter blocks in Flash storage.

use crc::{Crc, CRC_32_ISO_HDLC};

/// CRC32 algorithm (ISO HDLC / Ethernet / ZIP)
const CRC32: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);

/// Calculate CRC32 checksum of data
///
/// Uses the CRC-32-ISO-HDLC algorithm (polynomial 0x04C11DB7),
/// which is the same as used in Ethernet, ZIP, and PNG.
///
/// # Example
///
/// ```
/// use pico_trail::core::parameters::crc::calculate_crc32;
///
/// let data = b"Hello, World!";
/// let checksum = calculate_crc32(data);
/// assert_ne!(checksum, 0);
/// ```
pub fn calculate_crc32(data: &[u8]) -> u32 {
    CRC32.checksum(data)
}

/// Validate data against CRC32 checksum
///
/// Returns `true` if the checksum matches.
///
/// # Example
///
/// ```
/// use pico_trail::core::parameters::crc::{calculate_crc32, validate_crc32};
///
/// let data = b"Test data";
/// let crc = calculate_crc32(data);
/// assert!(validate_crc32(data, crc));
/// assert!(!validate_crc32(data, crc + 1)); // Wrong CRC
/// ```
pub fn validate_crc32(data: &[u8], expected_crc: u32) -> bool {
    calculate_crc32(data) == expected_crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc32_known_values() {
        // Known CRC32 values for test vectors
        let test_cases = [
            (b"" as &[u8], 0x00000000u32), // Empty data
            (b"a", 0xE8B7BE43),
            (b"abc", 0x352441C2),
            (b"123456789", 0xCBF43926),
        ];

        for (data, expected) in test_cases {
            let crc = calculate_crc32(data);
            assert_eq!(
                crc,
                expected,
                "CRC mismatch for '{}'",
                core::str::from_utf8(data).unwrap()
            );
        }
    }

    #[test]
    fn test_validate_crc32() {
        let data = b"Test data for CRC validation";
        let crc = calculate_crc32(data);

        assert!(validate_crc32(data, crc));
        assert!(!validate_crc32(data, crc + 1));
        assert!(!validate_crc32(data, 0));
    }

    #[test]
    fn test_crc32_detects_corruption() {
        let data = b"Original data";
        let crc = calculate_crc32(data);

        // Flip a single bit
        let mut corrupted = data.to_vec();
        corrupted[0] ^= 0x01;

        assert!(!validate_crc32(&corrupted, crc));
    }

    #[test]
    fn test_crc32_same_data_same_result() {
        let data = b"Consistent data";
        let crc1 = calculate_crc32(data);
        let crc2 = calculate_crc32(data);

        assert_eq!(crc1, crc2);
    }
}
