//! SPI interface trait
//!
//! This module defines the SPI bus communication interface that platform implementations must provide.

use crate::platform::Result;

/// SPI configuration
#[derive(Debug, Clone, Copy)]
pub struct SpiConfig {
    /// Bus frequency in Hz
    pub frequency: u32,
    /// SPI mode (CPOL and CPHA)
    pub mode: SpiMode,
    /// Bit order
    pub bit_order: SpiBitOrder,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            frequency: 1_000_000, // 1 MHz
            mode: SpiMode::Mode0,
            bit_order: SpiBitOrder::MsbFirst,
        }
    }
}

/// SPI mode (Clock Polarity and Phase)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    /// CPOL=0, CPHA=0
    Mode0,
    /// CPOL=0, CPHA=1
    Mode1,
    /// CPOL=1, CPHA=0
    Mode2,
    /// CPOL=1, CPHA=1
    Mode3,
}

/// SPI bit order
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiBitOrder {
    /// Most significant bit first
    MsbFirst,
    /// Least significant bit first
    LsbFirst,
}

/// SPI interface trait
///
/// Platform implementations must provide this interface for SPI bus communication.
///
/// # Safety Invariants
///
/// - SPI peripheral must be initialized before use
/// - Only one owner per SPI bus instance
/// - No concurrent access to the same SPI bus from multiple contexts
/// - Chip select (CS) is managed separately by the caller (typically via GPIO)
pub trait SpiInterface {
    /// Transfer data (full-duplex)
    ///
    /// Simultaneously transmits data from `write_buffer` and receives data into `read_buffer`.
    /// Both buffers must have the same length.
    ///
    /// # Arguments
    ///
    /// * `write_buffer` - Data to transmit
    /// * `read_buffer` - Buffer to receive data
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Spi` if:
    /// - Transfer fails
    /// - Buffers have different lengths
    /// - Timeout occurs
    fn transfer(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<()>;

    /// Write data (transmit only)
    ///
    /// Transmits data and discards received bytes.
    ///
    /// # Arguments
    ///
    /// * `data` - Data to transmit
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Spi` if the write operation fails.
    fn write(&mut self, data: &[u8]) -> Result<()>;

    /// Read data (receive only)
    ///
    /// Receives data while transmitting dummy bytes (typically 0x00 or 0xFF).
    ///
    /// # Arguments
    ///
    /// * `buffer` - Buffer to receive data
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Spi` if the read operation fails.
    fn read(&mut self, buffer: &mut [u8]) -> Result<()>;

    /// Set SPI bus frequency
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Spi` if the frequency cannot be achieved
    /// with the current clock configuration.
    fn set_frequency(&mut self, frequency: u32) -> Result<()>;
}
