//! I2C interface trait
//!
//! This module defines the I2C bus communication interface that platform implementations must provide.

use crate::platform::Result;

/// I2C configuration
#[derive(Debug, Clone, Copy)]
pub struct I2cConfig {
    /// Bus frequency in Hz (typically 100_000 or 400_000)
    pub frequency: u32,
    /// Timeout in microseconds
    pub timeout_us: u32,
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            frequency: 100_000,    // 100 kHz standard mode
            timeout_us: 1_000_000, // 1 second
        }
    }
}

/// I2C interface trait
///
/// Platform implementations must provide this interface for I2C bus communication.
///
/// # Safety Invariants
///
/// - I2C peripheral must be initialized before use
/// - Only one owner per I2C bus instance
/// - No concurrent access to the same I2C bus from multiple contexts
/// - Address must be 7-bit (valid range: 0x00..=0x7F)
pub trait I2cInterface {
    /// Write data to I2C device
    ///
    /// Performs a complete I2C write transaction:
    /// START - ADDR(W) - DATA - STOP
    ///
    /// # Arguments
    ///
    /// * `addr` - 7-bit I2C device address
    /// * `data` - Data bytes to write
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::I2c` if:
    /// - Device does not acknowledge (NACK)
    /// - Bus error occurs
    /// - Timeout expires
    fn write(&mut self, addr: u8, data: &[u8]) -> Result<()>;

    /// Read data from I2C device
    ///
    /// Performs a complete I2C read transaction:
    /// START - ADDR(R) - DATA - STOP
    ///
    /// # Arguments
    ///
    /// * `addr` - 7-bit I2C device address
    /// * `buffer` - Buffer to receive data
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::I2c` if:
    /// - Device does not acknowledge (NACK)
    /// - Bus error occurs
    /// - Timeout expires
    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()>;

    /// Write then read from I2C device
    ///
    /// Performs a combined write-read transaction with a repeated START:
    /// START - ADDR(W) - WRITE_DATA - REPEATED_START - ADDR(R) - READ_DATA - STOP
    ///
    /// This is commonly used to write a register address then read the register value.
    ///
    /// # Arguments
    ///
    /// * `addr` - 7-bit I2C device address
    /// * `write_data` - Data bytes to write (typically register address)
    /// * `read_buffer` - Buffer to receive read data
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::I2c` if:
    /// - Device does not acknowledge (NACK)
    /// - Bus error occurs
    /// - Timeout expires
    fn write_read(&mut self, addr: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<()>;

    /// Set I2C bus frequency
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::I2c` if the frequency cannot be achieved
    /// with the current clock configuration.
    fn set_frequency(&mut self, frequency: u32) -> Result<()>;
}
