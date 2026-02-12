//! Flash interface trait
//!
//! Platform-agnostic Flash storage interface for parameter persistence,
//! mission storage, and data logging. Platform implementations (RP2350, Mock)
//! implement this trait.

/// Flash-specific errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// Erase operation failed
    EraseFailed,
    /// Write operation failed
    WriteFailed,
    /// Read operation failed
    ReadFailed,
    /// Invalid address (out of bounds)
    InvalidAddress,
    /// Verify failed (data mismatch after write)
    VerifyFailed,
    /// Flash is busy
    Busy,
    /// Invalid data (serialization/deserialization errors)
    InvalidData,
}

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
pub trait FlashInterface {
    /// Read data from Flash
    ///
    /// Reads `buf.len()` bytes from Flash starting at `address`.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), FlashError>;

    /// Write data to Flash
    ///
    /// Writes `data` to Flash starting at `address`.
    /// Flash must be erased (0xFF) before writing.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FlashError>;

    /// Erase Flash region
    ///
    /// Erases Flash starting at `address` for `size` bytes.
    /// Sets all bytes in the region to 0xFF.
    fn erase(&mut self, address: u32, size: u32) -> Result<(), FlashError>;

    /// Get Flash block size
    ///
    /// Returns the minimum erasable unit size (typically 4096 bytes for RP2040/RP2350).
    fn block_size(&self) -> u32;

    /// Get total Flash size
    ///
    /// Returns the total Flash capacity in bytes.
    fn capacity(&self) -> u32;
}
