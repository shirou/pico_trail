//! Mock Flash implementation for testing
//!
//! Provides in-memory Flash simulation for unit tests.

use crate::platform::{error::FlashError, traits::FlashInterface, Result};
use core::cell::RefCell;
use std::vec::Vec;

/// Flash block size (4 KB)
const BLOCK_SIZE: u32 = 4096;

/// Flash capacity (4 MB, same as Pico 2 W)
const FLASH_CAPACITY: u32 = 4 * 1024 * 1024;

/// Minimum firmware size (protect first 256 KB)
const FIRMWARE_SIZE: u32 = 0x40000;

/// Mock Flash implementation
///
/// Simulates Flash storage in memory for testing. Supports:
/// - Read/write/erase operations
/// - Corruption injection for testing error handling
/// - Erase count tracking for wear leveling validation
/// - Power-loss simulation for reliability testing
///
/// # Example
///
/// ```
/// use pico_trail::platform::mock::MockFlash;
/// use pico_trail::platform::traits::FlashInterface;
///
/// let mut flash = MockFlash::new();
///
/// // Erase a block
/// flash.erase(0x040000, 4096).unwrap();
///
/// // Write data
/// let data = [0x50, 0x41, 0x52, 0x41]; // "PARA" magic
/// flash.write(0x040000, &data).unwrap();
///
/// // Read back
/// let mut buf = [0u8; 4];
/// flash.read(0x040000, &mut buf).unwrap();
/// assert_eq!(buf, data);
///
/// // Check erase count
/// assert_eq!(flash.get_erase_count(0x040000), 1);
/// ```
#[derive(Debug)]
pub struct MockFlash {
    /// Flash storage (initialized to 0xFF - erased state)
    storage: RefCell<Vec<u8>>,
    /// Erase count per block (for wear leveling testing)
    erase_counts: RefCell<Vec<u32>>,
    /// Simulated power loss flag
    power_loss: RefCell<bool>,
}

impl MockFlash {
    /// Create a new mock Flash instance
    pub fn new() -> Self {
        let storage = vec![0xFF; FLASH_CAPACITY as usize];
        let block_count = (FLASH_CAPACITY / BLOCK_SIZE) as usize;
        let erase_counts = vec![0; block_count];

        Self {
            storage: RefCell::new(storage),
            erase_counts: RefCell::new(erase_counts),
            power_loss: RefCell::new(false),
        }
    }

    /// Get Flash contents (for test verification)
    pub fn get_contents(&self, address: u32, len: usize) -> Vec<u8> {
        let storage = self.storage.borrow();
        storage[address as usize..(address as usize + len)].to_vec()
    }

    /// Inject corruption at address (for testing error recovery)
    ///
    /// Writes random data to simulate Flash corruption.
    pub fn inject_corruption(&mut self, address: u32, len: usize) {
        let mut storage = self.storage.borrow_mut();
        for i in 0..len {
            storage[address as usize + i] = 0xAA; // Corrupt pattern
        }
    }

    /// Get erase count for a block (for wear leveling validation)
    ///
    /// Returns the number of times a block has been erased.
    pub fn get_erase_count(&self, address: u32) -> u32 {
        let block_id = (address / BLOCK_SIZE) as usize;
        self.erase_counts.borrow()[block_id]
    }

    /// Get total erase count across all blocks
    pub fn get_total_erase_count(&self) -> u32 {
        self.erase_counts.borrow().iter().sum()
    }

    /// Simulate power loss during next write operation
    ///
    /// The next write will only partially complete, simulating
    /// power loss mid-operation for reliability testing.
    pub fn simulate_power_loss(&mut self) {
        *self.power_loss.borrow_mut() = true;
    }

    /// Reset power loss flag
    fn clear_power_loss(&mut self) {
        *self.power_loss.borrow_mut() = false;
    }

    /// Check if address is in writable region
    fn is_writable(&self, address: u32) -> bool {
        (FIRMWARE_SIZE..FLASH_CAPACITY).contains(&address)
    }

    /// Check if address is block-aligned
    fn is_block_aligned(&self, address: u32) -> bool {
        address.is_multiple_of(BLOCK_SIZE)
    }
}

impl Default for MockFlash {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashInterface for MockFlash {
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<()> {
        // Validate address range
        if address >= FLASH_CAPACITY {
            return Err(FlashError::InvalidAddress.into());
        }

        if address as usize + buf.len() > FLASH_CAPACITY as usize {
            return Err(FlashError::InvalidAddress.into());
        }

        // Copy from storage
        let storage = self.storage.borrow();
        buf.copy_from_slice(&storage[address as usize..(address as usize + buf.len())]);

        Ok(())
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<()> {
        // Validate address is in writable region
        if !self.is_writable(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate write doesn't exceed capacity
        if address as usize + data.len() > FLASH_CAPACITY as usize {
            return Err(FlashError::InvalidAddress.into());
        }

        // Simulate power loss (partial write)
        let write_len = if *self.power_loss.borrow() {
            // Only write half the data to simulate power loss
            self.clear_power_loss();
            data.len() / 2
        } else {
            data.len()
        };

        // Write to storage
        // Flash can only change bits from 1→0 (simulate this behavior)
        let mut storage = self.storage.borrow_mut();
        for i in 0..write_len {
            storage[address as usize + i] &= data[i];
        }

        Ok(())
    }

    fn erase(&mut self, address: u32, size: u32) -> Result<()> {
        // Validate address is in writable region
        if !self.is_writable(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate address is block-aligned
        if !self.is_block_aligned(address) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate size is multiple of block size
        if !size.is_multiple_of(BLOCK_SIZE) {
            return Err(FlashError::InvalidAddress.into());
        }

        // Validate erase doesn't exceed capacity
        if address + size > FLASH_CAPACITY {
            return Err(FlashError::InvalidAddress.into());
        }

        // Erase blocks (set to 0xFF)
        let mut storage = self.storage.borrow_mut();
        for i in 0..size as usize {
            storage[address as usize + i] = 0xFF;
        }

        // Update erase counts
        let block_count = size / BLOCK_SIZE;
        let start_block = (address / BLOCK_SIZE) as usize;
        let mut erase_counts = self.erase_counts.borrow_mut();
        for i in 0..block_count {
            erase_counts[start_block + i as usize] += 1;
        }

        Ok(())
    }

    fn block_size(&self) -> u32 {
        BLOCK_SIZE
    }

    fn capacity(&self) -> u32 {
        FLASH_CAPACITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_flash_read_write() {
        let mut flash = MockFlash::new();

        // Erase block first
        flash.erase(0x040000, 4096).unwrap();

        // Write data
        let data = [0x50, 0x41, 0x52, 0x41]; // "PARA"
        flash.write(0x040000, &data).unwrap();

        // Read back
        let mut buf = [0u8; 4];
        flash.read(0x040000, &mut buf).unwrap();
        assert_eq!(buf, data);
    }

    #[test]
    fn test_mock_flash_erase() {
        let mut flash = MockFlash::new();

        // Write data
        flash.erase(0x040000, 4096).unwrap();
        flash.write(0x040000, &[0x55; 256]).unwrap();

        // Erase
        flash.erase(0x040000, 4096).unwrap();

        // Verify erased (0xFF)
        let contents = flash.get_contents(0x040000, 256);
        assert!(contents.iter().all(|&b| b == 0xFF));
    }

    #[test]
    fn test_mock_flash_erase_count() {
        let mut flash = MockFlash::new();

        // Erase same block multiple times
        flash.erase(0x040000, 4096).unwrap();
        flash.erase(0x040000, 4096).unwrap();
        flash.erase(0x040000, 4096).unwrap();

        // Check erase count
        assert_eq!(flash.get_erase_count(0x040000), 3);
    }

    #[test]
    fn test_mock_flash_invalid_address() {
        let mut flash = MockFlash::new();

        // Try to write to firmware region (protected)
        let result = flash.write(0x000000, &[0x00; 4]);
        assert!(result.is_err());

        // Try to read beyond capacity
        let mut buf = [0u8; 4];
        let result = flash.read(FLASH_CAPACITY, &mut buf);
        assert!(result.is_err());
    }

    #[test]
    fn test_mock_flash_unaligned_erase() {
        let mut flash = MockFlash::new();

        // Try to erase at unaligned address
        let result = flash.erase(0x040100, 4096);
        assert!(result.is_err());

        // Try to erase with invalid size
        let result = flash.erase(0x040000, 1024);
        assert!(result.is_err());
    }

    #[test]
    fn test_mock_flash_power_loss() {
        let mut flash = MockFlash::new();

        // Erase block
        flash.erase(0x040000, 4096).unwrap();

        // Simulate power loss during write
        flash.simulate_power_loss();
        flash.write(0x040000, &[0x55; 256]).unwrap();

        // Only half should be written
        let contents = flash.get_contents(0x040000, 256);
        assert_eq!(&contents[..128], &[0x55; 128]);
        assert_eq!(&contents[128..], &[0xFF; 128]); // Rest still erased
    }

    #[test]
    fn test_mock_flash_write_only_clears_bits() {
        let mut flash = MockFlash::new();

        // Erase block (all 0xFF)
        flash.erase(0x040000, 4096).unwrap();

        // Write 0x0F (clears upper 4 bits)
        flash.write(0x040000, &[0x0F]).unwrap();
        let mut buf = [0u8; 1];
        flash.read(0x040000, &mut buf).unwrap();
        assert_eq!(buf[0], 0x0F);

        // Try to write 0xFF (should not change anything - can't set bits)
        flash.write(0x040000, &[0xFF]).unwrap();
        flash.read(0x040000, &mut buf).unwrap();
        assert_eq!(buf[0], 0x0F); // Still 0x0F, not 0xFF
    }
}
