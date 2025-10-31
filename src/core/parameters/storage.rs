//! Flash parameter storage implementation
//!
//! This module provides Flash-backed parameter persistence with redundant
//! block rotation for wear leveling.

use super::block::{Parameter, ParameterBlockHeader, MAX_PARAMS};
use super::crc::{calculate_crc32, validate_crc32};
use crate::platform::{FlashInterface, Result};

/// Flash block addresses for parameter storage
///
/// Uses 4 blocks (16 KB total) with round-robin rotation for 4x wear leveling.
pub const PARAM_BLOCK_ADDRESSES: [u32; 4] = [
    0x040000, // Block 0: 256 KB offset
    0x041000, // Block 1: 260 KB offset
    0x042000, // Block 2: 264 KB offset
    0x043000, // Block 3: 268 KB offset
];

/// Size of CRC32 field (4 bytes)
const CRC_SIZE: usize = 4;

/// Number of Flash blocks for parameter storage
const NUM_BLOCKS: usize = PARAM_BLOCK_ADDRESSES.len();

/// Storage statistics for wear leveling monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageStats {
    /// Total number of parameter saves
    pub total_saves: u32,
    /// Current active block ID (0-3)
    pub active_block: Option<u8>,
    /// Erase count per block (for wear monitoring)
    pub erase_counts: [u32; NUM_BLOCKS],
}

/// Flash parameter storage
///
/// Manages parameter persistence to Flash with:
/// - Redundant block rotation (4 blocks)
/// - CRC32 validation
/// - Sequence number tracking (newest block selection)
///
/// # Example
///
/// ```no_run
/// use pico_trail::platform::mock::MockFlash;
/// use pico_trail::core::parameters::storage::FlashParamStorage;
/// use pico_trail::core::parameters::block::Parameter;
///
/// let flash = MockFlash::new();
/// let mut storage = FlashParamStorage::new(flash);
///
/// // Write parameters to Flash
/// let params = vec![Parameter::new_f32(0x12345678, 3.14159)];
/// storage.write_block(0, &params, 1).unwrap();
///
/// // Read parameters back
/// let (header, params, valid) = storage.read_block(0).unwrap();
/// assert!(valid);
/// assert_eq!(params.len(), 1);
/// ```
pub struct FlashParamStorage<F: FlashInterface> {
    /// Flash interface
    flash: F,
    /// Storage statistics
    stats: StorageStats,
}

impl<F: FlashInterface> FlashParamStorage<F> {
    /// Create a new Flash parameter storage instance
    pub fn new(flash: F) -> Self {
        Self {
            flash,
            stats: StorageStats {
                total_saves: 0,
                active_block: None,
                erase_counts: [0; NUM_BLOCKS],
            },
        }
    }

    /// Write parameter block to Flash
    ///
    /// # Arguments
    ///
    /// - `block_id`: Block index (0-3)
    /// - `params`: Parameters to write
    /// - `sequence`: Sequence number (for wear leveling)
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Block ID is out of range
    /// - Too many parameters (> MAX_PARAMS)
    /// - Flash erase/write fails
    pub fn write_block(&mut self, block_id: u8, params: &[Parameter], sequence: u16) -> Result<()> {
        if block_id as usize >= PARAM_BLOCK_ADDRESSES.len() {
            return Err(crate::platform::error::FlashError::InvalidAddress.into());
        }

        if params.len() > MAX_PARAMS {
            return Err(crate::platform::error::FlashError::WriteFailed.into());
        }

        let address = PARAM_BLOCK_ADDRESSES[block_id as usize];

        // Erase block first
        self.flash.erase(address, self.flash.block_size())?;

        // Build block data: header + params + CRC
        let header = ParameterBlockHeader::new(sequence, params.len() as u16);
        let header_bytes = header.to_bytes();

        // Serialize parameters
        let mut param_bytes = heapless::Vec::<u8, { MAX_PARAMS * Parameter::SIZE }>::new();
        for param in params {
            let bytes = param.to_bytes();
            param_bytes
                .extend_from_slice(&bytes)
                .map_err(|_| crate::platform::error::FlashError::WriteFailed)?;
        }

        // Calculate CRC over header + params
        let mut crc_data = heapless::Vec::<
            u8,
            { ParameterBlockHeader::SIZE + MAX_PARAMS * Parameter::SIZE },
        >::new();
        crc_data
            .extend_from_slice(&header_bytes)
            .map_err(|_| crate::platform::error::FlashError::WriteFailed)?;
        crc_data
            .extend_from_slice(&param_bytes)
            .map_err(|_| crate::platform::error::FlashError::WriteFailed)?;

        let crc = calculate_crc32(&crc_data);
        let crc_bytes = crc.to_le_bytes();

        // Write header
        self.flash.write(address, &header_bytes)?;

        // Write parameters
        if !param_bytes.is_empty() {
            self.flash
                .write(address + ParameterBlockHeader::SIZE as u32, &param_bytes)?;
        }

        // Write CRC
        let crc_offset = address + ParameterBlockHeader::SIZE as u32 + param_bytes.len() as u32;
        self.flash.write(crc_offset, &crc_bytes)?;

        // Update statistics
        self.stats.total_saves += 1;
        self.stats.active_block = Some(block_id);
        self.stats.erase_counts[block_id as usize] += 1;

        Ok(())
    }

    /// Read parameter block from Flash
    ///
    /// Returns (header, parameters, valid_crc).
    ///
    /// # Arguments
    ///
    /// - `block_id`: Block index (0-3)
    ///
    /// # Returns
    ///
    /// - `header`: Block header
    /// - `params`: Vector of parameters
    /// - `valid`: True if CRC is valid
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Block ID is out of range
    /// - Flash read fails
    /// - Header is invalid
    pub fn read_block(
        &mut self,
        block_id: u8,
    ) -> Result<(
        ParameterBlockHeader,
        heapless::Vec<Parameter, MAX_PARAMS>,
        bool,
    )> {
        if block_id as usize >= PARAM_BLOCK_ADDRESSES.len() {
            return Err(crate::platform::error::FlashError::InvalidAddress.into());
        }

        let address = PARAM_BLOCK_ADDRESSES[block_id as usize];

        // Read header
        let mut header_buf = [0u8; ParameterBlockHeader::SIZE];
        self.flash.read(address, &mut header_buf)?;

        let header = ParameterBlockHeader::from_bytes(&header_buf)
            .ok_or(crate::platform::error::FlashError::ReadFailed)?;

        if !header.is_valid() {
            return Err(crate::platform::error::FlashError::ReadFailed.into());
        }

        // Read parameters
        let param_count = header.param_count as usize;
        let param_data_size = param_count * Parameter::SIZE;

        let mut param_buf = heapless::Vec::<u8, { MAX_PARAMS * Parameter::SIZE }>::new();
        param_buf.resize(param_data_size, 0).ok();

        if param_count > 0 {
            self.flash
                .read(address + ParameterBlockHeader::SIZE as u32, &mut param_buf)?;
        }

        // Parse parameters
        let mut params = heapless::Vec::<Parameter, MAX_PARAMS>::new();
        for i in 0..param_count {
            let offset = i * Parameter::SIZE;
            let param = Parameter::from_bytes(&param_buf[offset..offset + Parameter::SIZE])
                .ok_or(crate::platform::error::FlashError::ReadFailed)?;
            params
                .push(param)
                .map_err(|_| crate::platform::error::FlashError::ReadFailed)?;
        }

        // Read CRC
        let crc_offset = address + ParameterBlockHeader::SIZE as u32 + param_data_size as u32;
        let mut crc_buf = [0u8; CRC_SIZE];
        self.flash.read(crc_offset, &mut crc_buf)?;
        let stored_crc = u32::from_le_bytes(crc_buf);

        // Validate CRC
        let mut crc_data = heapless::Vec::<
            u8,
            { ParameterBlockHeader::SIZE + MAX_PARAMS * Parameter::SIZE },
        >::new();
        crc_data.extend_from_slice(&header_buf).ok();
        crc_data.extend_from_slice(&param_buf).ok();

        let valid = validate_crc32(&crc_data, stored_crc);

        Ok((header, params, valid))
    }

    /// Find the active block (block with highest valid sequence number)
    ///
    /// Scans all blocks and returns the block ID with the highest sequence number
    /// and valid CRC. Returns None if all blocks are invalid.
    ///
    /// # Corruption Recovery
    ///
    /// If the block with the highest sequence has invalid CRC, tries the next highest
    /// sequence until a valid block is found.
    pub fn find_active_block(&mut self) -> Option<u8> {
        let mut candidates: heapless::Vec<(u8, u16), NUM_BLOCKS> = heapless::Vec::new();

        // Scan all blocks
        for block_id in 0..NUM_BLOCKS as u8 {
            if let Ok((header, _, valid)) = self.read_block(block_id) {
                if valid && header.is_valid() {
                    candidates.push((block_id, header.sequence)).ok();
                }
            }
        }

        if candidates.is_empty() {
            return None;
        }

        // Find block with highest sequence number
        let mut max_block = candidates[0].0;
        let mut max_seq = candidates[0].1;

        for &(block_id, seq) in candidates.iter().skip(1) {
            if seq > max_seq {
                max_block = block_id;
                max_seq = seq;
            }
        }

        Some(max_block)
    }

    /// Choose next block for rotation (round-robin)
    ///
    /// Returns the next block ID in round-robin order.
    pub fn choose_next_block(&self, current_block: u8) -> u8 {
        (current_block + 1) % NUM_BLOCKS as u8
    }

    /// Increment sequence number with wrap-around
    ///
    /// Increments sequence number, wrapping to 0 at u16::MAX.
    pub fn increment_sequence(&self, current: u16) -> u16 {
        current.wrapping_add(1)
    }

    /// Get storage statistics
    ///
    /// Returns statistics for wear leveling monitoring.
    pub fn get_stats(&self) -> StorageStats {
        self.stats
    }

    /// Get Flash interface reference (for testing)
    pub fn flash_mut(&mut self) -> &mut F {
        &mut self.flash
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockFlash;

    #[test]
    fn test_write_and_read_block() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        // Write parameters
        let params = heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[
            Parameter::new_f32(0x12345678, core::f32::consts::PI),
            Parameter::new_u32(0xABCDEF00, 12345),
        ])
        .unwrap();

        storage.write_block(0, &params, 1).unwrap();

        // Read back
        let (header, read_params, valid) = storage.read_block(0).unwrap();

        assert!(valid);
        assert_eq!(header.sequence, 1);
        assert_eq!(header.param_count, 2);
        assert_eq!(read_params.len(), 2);
        assert_eq!(read_params[0].name_hash, 0x12345678);
        assert_eq!(read_params[1].name_hash, 0xABCDEF00);
    }

    #[test]
    fn test_crc_validation() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        // Write parameters
        let params =
            heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_f32(0, 1.0)])
                .unwrap();

        storage.write_block(0, &params, 1).unwrap();

        // Corrupt data
        storage
            .flash_mut()
            .inject_corruption(PARAM_BLOCK_ADDRESSES[0] + 20, 4);

        // Read should detect corruption
        let (_, _, valid) = storage.read_block(0).unwrap();
        assert!(!valid);
    }

    #[test]
    fn test_multiple_blocks() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        // Write to all 4 blocks
        for block_id in 0..4 {
            let params = heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_u32(
                block_id as u32,
                block_id as u32,
            )])
            .unwrap();

            storage
                .write_block(block_id as u8, &params, block_id as u16)
                .unwrap();
        }

        // Read back all blocks
        for block_id in 0..4 {
            let (header, params, valid) = storage.read_block(block_id as u8).unwrap();
            assert!(valid);
            assert_eq!(header.sequence, block_id as u16);
            assert_eq!(params[0].as_u32().unwrap(), block_id as u32);
        }
    }

    #[test]
    fn test_empty_parameter_list() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        // Write empty parameter list
        let params = heapless::Vec::<Parameter, MAX_PARAMS>::new();
        storage.write_block(0, &params, 0).unwrap();

        // Read back
        let (header, read_params, valid) = storage.read_block(0).unwrap();
        assert!(valid);
        assert_eq!(header.param_count, 0);
        assert_eq!(read_params.len(), 0);
    }

    #[test]
    fn test_invalid_block_id() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        let params = heapless::Vec::<Parameter, MAX_PARAMS>::new();

        // Block ID out of range
        let result = storage.write_block(4, &params, 0);
        assert!(result.is_err());

        let result = storage.read_block(4);
        assert!(result.is_err());
    }

    #[test]
    fn test_find_active_block() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        let params =
            heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_f32(0, 1.0)])
                .unwrap();

        // No active block initially
        assert_eq!(storage.find_active_block(), None);

        // Write to blocks with different sequences
        storage.write_block(0, &params, 1).unwrap();
        storage.write_block(1, &params, 3).unwrap();
        storage.write_block(2, &params, 2).unwrap();

        // Should find block 1 (highest sequence = 3)
        assert_eq!(storage.find_active_block(), Some(1));
    }

    #[test]
    fn test_block_rotation() {
        let flash = MockFlash::new();
        let storage = FlashParamStorage::new(flash);

        // Round-robin rotation
        assert_eq!(storage.choose_next_block(0), 1);
        assert_eq!(storage.choose_next_block(1), 2);
        assert_eq!(storage.choose_next_block(2), 3);
        assert_eq!(storage.choose_next_block(3), 0); // Wrap around
    }

    #[test]
    fn test_sequence_wrap() {
        let flash = MockFlash::new();
        let storage = FlashParamStorage::new(flash);

        // Normal increment
        assert_eq!(storage.increment_sequence(0), 1);
        assert_eq!(storage.increment_sequence(100), 101);

        // Wrap at u16::MAX
        assert_eq!(storage.increment_sequence(u16::MAX), 0);
    }

    #[test]
    fn test_corruption_recovery() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        let params =
            heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_f32(0, 1.0)])
                .unwrap();

        // Write multiple blocks
        storage.write_block(0, &params, 1).unwrap();
        storage.write_block(1, &params, 3).unwrap();
        storage.write_block(2, &params, 2).unwrap();

        // Corrupt the block with highest sequence (block 1)
        storage
            .flash_mut()
            .inject_corruption(PARAM_BLOCK_ADDRESSES[1] + 20, 4);

        // Should fall back to block 2 (next highest valid sequence = 2)
        assert_eq!(storage.find_active_block(), Some(2));
    }

    #[test]
    fn test_statistics_tracking() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        let params =
            heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_f32(0, 1.0)])
                .unwrap();

        // Initial stats
        let stats = storage.get_stats();
        assert_eq!(stats.total_saves, 0);
        assert_eq!(stats.active_block, None);
        assert_eq!(stats.erase_counts, [0, 0, 0, 0]);

        // Write to block 0
        storage.write_block(0, &params, 1).unwrap();
        let stats = storage.get_stats();
        assert_eq!(stats.total_saves, 1);
        assert_eq!(stats.active_block, Some(0));
        assert_eq!(stats.erase_counts, [1, 0, 0, 0]);

        // Write to block 1
        storage.write_block(1, &params, 2).unwrap();
        let stats = storage.get_stats();
        assert_eq!(stats.total_saves, 2);
        assert_eq!(stats.active_block, Some(1));
        assert_eq!(stats.erase_counts, [1, 1, 0, 0]);

        // Write to block 0 again
        storage.write_block(0, &params, 3).unwrap();
        let stats = storage.get_stats();
        assert_eq!(stats.total_saves, 3);
        assert_eq!(stats.active_block, Some(0));
        assert_eq!(stats.erase_counts, [2, 1, 0, 0]);
    }

    #[test]
    fn test_all_blocks_corrupted() {
        let flash = MockFlash::new();
        let mut storage = FlashParamStorage::new(flash);

        let params =
            heapless::Vec::<Parameter, MAX_PARAMS>::from_slice(&[Parameter::new_f32(0, 1.0)])
                .unwrap();

        // Write to all blocks
        storage.write_block(0, &params, 1).unwrap();
        storage.write_block(1, &params, 2).unwrap();
        storage.write_block(2, &params, 3).unwrap();
        storage.write_block(3, &params, 4).unwrap();

        // Corrupt all blocks
        for &addr in &PARAM_BLOCK_ADDRESSES {
            storage.flash_mut().inject_corruption(addr + 20, 4);
        }

        // Should return None
        assert_eq!(storage.find_active_block(), None);
    }
}
