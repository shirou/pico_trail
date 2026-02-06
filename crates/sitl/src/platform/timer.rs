//! Simulated time source for SITL.
//!
//! Wraps a shared atomic counter for simulation time, allowing
//! the bridge and platform to share a consistent time reference.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

/// Simulated time source backed by a shared atomic counter.
///
/// Multiple clones share the same underlying counter, enabling the bridge
/// to advance time while the platform reads it.
#[derive(Debug, Clone)]
pub struct SitlTimeSource {
    time_us: Arc<AtomicU64>,
}

impl SitlTimeSource {
    /// Create a new time source starting at zero.
    pub fn new() -> Self {
        Self {
            time_us: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Get current simulation time in microseconds.
    pub fn now_us(&self) -> u64 {
        self.time_us.load(Ordering::Relaxed)
    }

    /// Get current simulation time in milliseconds.
    pub fn now_ms(&self) -> u64 {
        self.now_us() / 1000
    }

    /// Advance simulation time by the given number of microseconds.
    pub fn advance_us(&self, us: u64) {
        self.time_us.fetch_add(us, Ordering::Relaxed);
    }

    /// Set simulation time to an absolute value.
    pub fn set_us(&self, us: u64) {
        self.time_us.store(us, Ordering::Relaxed);
    }

    /// Simulate a delay by advancing time (non-blocking).
    pub fn delay_us(&self, us: u32) {
        self.advance_us(us as u64);
    }

    /// Simulate a delay in milliseconds by advancing time (non-blocking).
    pub fn delay_ms(&self, ms: u32) {
        self.advance_us(ms as u64 * 1000);
    }
}

impl Default for SitlTimeSource {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_starts_at_zero() {
        let ts = SitlTimeSource::new();
        assert_eq!(ts.now_us(), 0);
        assert_eq!(ts.now_ms(), 0);
    }

    #[test]
    fn test_advance_time() {
        let ts = SitlTimeSource::new();
        ts.advance_us(1000);
        assert_eq!(ts.now_us(), 1000);
        assert_eq!(ts.now_ms(), 1);
    }

    #[test]
    fn test_set_time() {
        let ts = SitlTimeSource::new();
        ts.set_us(5_000_000);
        assert_eq!(ts.now_us(), 5_000_000);
        assert_eq!(ts.now_ms(), 5000);
    }

    #[test]
    fn test_delay_advances_time() {
        let ts = SitlTimeSource::new();
        ts.delay_us(500);
        assert_eq!(ts.now_us(), 500);
        ts.delay_ms(2);
        assert_eq!(ts.now_us(), 2500);
    }

    #[test]
    fn test_shared_time_via_clone() {
        let ts1 = SitlTimeSource::new();
        let ts2 = ts1.clone();
        ts1.advance_us(1000);
        assert_eq!(ts2.now_us(), 1000);
    }
}
