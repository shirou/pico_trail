//! Time abstraction traits for platform-agnostic timing operations.
//!
//! This module provides the `TimeSource` trait that abstracts over different
//! time providers (Embassy, mock, etc.) to enable host testing without
//! embedded dependencies.

use core::cell::Cell;

/// Platform-agnostic time source for control loops and timing.
///
/// This trait abstracts over different time providers:
/// - `EmbassyTime` (in firmware crate) for embedded targets using Embassy
/// - `MockTime` for host testing with controllable time
///
/// # Example
///
/// ```
/// use pico_trail_core::traits::{TimeSource, MockTime};
///
/// fn control_loop<T: TimeSource>(time: &T, last_update: &mut u64) {
///     let now = time.now_us();
///     let elapsed = time.elapsed_since(*last_update);
///     if elapsed >= 20_000 { // 50Hz
///         // Do control update
///         *last_update = now;
///     }
/// }
///
/// let time = MockTime::new();
/// let mut last = 0;
/// control_loop(&time, &mut last);
/// ```
pub trait TimeSource: Clone + Send + Sync {
    /// Returns current time in milliseconds since system start.
    fn now_ms(&self) -> u64;

    /// Returns current time in microseconds since system start.
    fn now_us(&self) -> u64;

    /// Returns elapsed time in microseconds since a reference point.
    ///
    /// Uses saturating subtraction to handle potential overflow.
    fn elapsed_since(&self, reference_us: u64) -> u64 {
        self.now_us().saturating_sub(reference_us)
    }
}

// ============================================================================
// Mock Implementation (always available for testing)
// ============================================================================

/// Mock time source for testing with controllable time advancement.
///
/// This implementation allows tests to control time progression,
/// enabling deterministic testing of timing-dependent code.
///
/// # Example
///
/// ```
/// use pico_trail_core::traits::{MockTime, TimeSource};
///
/// let time = MockTime::new();
/// assert_eq!(time.now_us(), 0);
///
/// time.advance(1000); // Advance 1ms
/// assert_eq!(time.now_us(), 1000);
/// assert_eq!(time.now_ms(), 1);
/// ```
#[derive(Clone, Default)]
pub struct MockTime {
    current_us: Cell<u64>,
}

// Safety: MockTime is only used in single-threaded test contexts
// where Cell is safe. The Send+Sync bounds on TimeSource trait
// are required for embedded contexts, but MockTime is not used there.
unsafe impl Send for MockTime {}
unsafe impl Sync for MockTime {}

impl MockTime {
    /// Creates a new `MockTime` starting at time 0.
    pub fn new() -> Self {
        Self {
            current_us: Cell::new(0),
        }
    }

    /// Creates a new `MockTime` starting at the specified time.
    pub fn with_initial(us: u64) -> Self {
        Self {
            current_us: Cell::new(us),
        }
    }

    /// Sets the current time to an absolute value.
    pub fn set(&self, us: u64) {
        self.current_us.set(us);
    }

    /// Advances the current time by the specified amount.
    pub fn advance(&self, us: u64) {
        self.current_us.set(self.current_us.get() + us);
    }
}

impl TimeSource for MockTime {
    fn now_ms(&self) -> u64 {
        self.current_us.get() / 1000
    }

    fn now_us(&self) -> u64 {
        self.current_us.get()
    }
}

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_time_initial_value() {
        let time = MockTime::new();
        assert_eq!(time.now_us(), 0);
        assert_eq!(time.now_ms(), 0);
    }

    #[test]
    fn mock_time_with_initial() {
        let time = MockTime::with_initial(5_000_000);
        assert_eq!(time.now_us(), 5_000_000);
        assert_eq!(time.now_ms(), 5000);
    }

    #[test]
    fn mock_time_set() {
        let time = MockTime::new();
        time.set(1_000_000);
        assert_eq!(time.now_us(), 1_000_000);
        assert_eq!(time.now_ms(), 1000);
    }

    #[test]
    fn mock_time_advance() {
        let time = MockTime::new();
        time.advance(500_000);
        assert_eq!(time.now_us(), 500_000);

        time.advance(500_000);
        assert_eq!(time.now_us(), 1_000_000);
        assert_eq!(time.now_ms(), 1000);
    }

    #[test]
    fn mock_time_elapsed_since() {
        let time = MockTime::new();
        time.set(10_000);

        let reference = 3_000;
        assert_eq!(time.elapsed_since(reference), 7_000);
    }

    #[test]
    fn mock_time_elapsed_since_saturates() {
        let time = MockTime::new();
        time.set(1_000);

        // Reference is in the "future" - should saturate to 0
        let reference = 5_000;
        assert_eq!(time.elapsed_since(reference), 0);
    }

    #[test]
    fn mock_time_ms_conversion() {
        let time = MockTime::new();

        // Test that ms rounds down
        time.set(1_999);
        assert_eq!(time.now_ms(), 1);

        time.set(2_000);
        assert_eq!(time.now_ms(), 2);
    }
}
