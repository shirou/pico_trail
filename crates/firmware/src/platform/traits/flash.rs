//! Flash interface trait
//!
//! This module defines the Flash storage interface that platform implementations must provide.
//! Flash is used for parameter persistence, mission storage, and data logging.

use crate::platform::Result;

/// Flash interface trait
///
/// Platform implementations must provide this interface for Flash read/write/erase operations.
///
/// # Flash Characteristics
///
/// - Flash is organized in blocks (typically 4 KB on RP2040/RP2350)
/// - Erase operations set all bytes to 0xFF
/// - Write operations can only change bits from 1→0 (must erase first to reset to 1)
/// - Flash operations are blocking and can take 100ms+ (wrap in async tasks)
///
/// # Safety Invariants
///
/// - Flash peripheral must be initialized before use
/// - Only one owner per Flash instance (no concurrent access)
/// - Must not erase/write firmware region (implementations must validate addresses)
/// - Interrupts may need to be disabled during Flash operations (platform-specific)
///
/// # Memory Layout (RP2040/RP2350)
///
/// ```text
/// [Firmware]           0x000000 - 0x040000 (256 KB) - DO NOT WRITE
/// [Parameter Block 0]  0x040000 - 0x041000 (4 KB)
/// [Parameter Block 1]  0x041000 - 0x042000 (4 KB)
/// [Parameter Block 2]  0x042000 - 0x043000 (4 KB)
/// [Parameter Block 3]  0x043000 - 0x044000 (4 KB)
/// [Mission Storage]    0x044000 - 0x046000 (8 KB)
/// [Log Storage]        0x046000 - 0x200000+ (remaining)
/// ```
pub trait FlashInterface {
    /// Read data from Flash
    ///
    /// Reads `buf.len()` bytes from Flash starting at `address`.
    ///
    /// # Arguments
    ///
    /// - `address`: Flash address to read from (must be within Flash bounds)
    /// - `buf`: Buffer to read data into
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Flash(FlashError::InvalidAddress)` if address is out of bounds.
    /// Returns `PlatformError::Flash(FlashError::ReadFailed)` if the read operation fails.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<()>;

    /// Write data to Flash
    ///
    /// Writes `data` to Flash starting at `address`.
    ///
    /// # Important
    ///
    /// - Flash must be erased (0xFF) before writing
    /// - Writing can only change bits from 1→0
    /// - Caller must erase the target region before writing
    ///
    /// # Arguments
    ///
    /// - `address`: Flash address to write to (must be within writable bounds)
    /// - `data`: Data to write
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Flash(FlashError::InvalidAddress)` if address is in firmware region.
    /// Returns `PlatformError::Flash(FlashError::WriteFailed)` if the write operation fails.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<()>;

    /// Erase Flash region
    ///
    /// Erases Flash starting at `address` for `size` bytes.
    /// Sets all bytes in the region to 0xFF.
    ///
    /// # Important
    ///
    /// - Erase size must be aligned to Flash block size (typically 4 KB)
    /// - Erase can take 100ms+ (blocking operation)
    /// - Address must be block-aligned
    ///
    /// # Arguments
    ///
    /// - `address`: Flash address to erase (must be block-aligned)
    /// - `size`: Number of bytes to erase (must be multiple of block size)
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Flash(FlashError::InvalidAddress)` if:
    /// - Address is in firmware region
    /// - Address is not block-aligned
    /// - Size is not a multiple of block size
    ///
    /// Returns `PlatformError::Flash(FlashError::EraseFailed)` if the erase operation fails.
    fn erase(&mut self, address: u32, size: u32) -> Result<()>;

    /// Get Flash block size
    ///
    /// Returns the minimum erasable unit size (typically 4096 bytes for RP2040/RP2350).
    fn block_size(&self) -> u32;

    /// Get total Flash size
    ///
    /// Returns the total Flash capacity in bytes.
    fn capacity(&self) -> u32;
}
