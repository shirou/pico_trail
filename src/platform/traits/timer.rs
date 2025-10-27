//! Timer interface trait
//!
//! This module defines the timer and delay interface that platform implementations must provide.

use crate::platform::Result;

/// Timer interface trait
///
/// Platform implementations must provide this interface for timing and delays.
///
/// # Safety Invariants
///
/// - Timer peripheral must be initialized before use
/// - Microsecond-level precision required
/// - Monotonic time source (never goes backwards)
pub trait TimerInterface {
    /// Delay for specified number of microseconds
    ///
    /// Blocks execution for at least `us` microseconds.
    ///
    /// # Arguments
    ///
    /// * `us` - Delay duration in microseconds
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Timer` if the delay operation fails.
    fn delay_us(&mut self, us: u32) -> Result<()>;

    /// Delay for specified number of milliseconds
    ///
    /// Blocks execution for at least `ms` milliseconds.
    ///
    /// # Arguments
    ///
    /// * `ms` - Delay duration in milliseconds
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Timer` if the delay operation fails.
    fn delay_ms(&mut self, ms: u32) -> Result<()>;

    /// Get current time in microseconds
    ///
    /// Returns a monotonic timestamp in microseconds since platform initialization.
    /// The timestamp wraps around after approximately 71 minutes (2^32 microseconds).
    ///
    /// # Note
    ///
    /// Applications must handle timestamp wraparound when calculating durations.
    fn now_us(&self) -> u64;

    /// Get current time in milliseconds
    ///
    /// Returns a monotonic timestamp in milliseconds since platform initialization.
    fn now_ms(&self) -> u64 {
        self.now_us() / 1000
    }
}
