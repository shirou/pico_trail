//! Mock Timer implementation for testing

use crate::platform::{Result, traits::TimerInterface};

/// Mock Timer implementation
///
/// Uses simulated time for delays in test environment.
/// For actual timing tests, use platform-specific timers.
#[derive(Debug)]
pub struct MockTimer {
    start_time_us: u64,
}

impl MockTimer {
    /// Create a new mock timer
    pub fn new() -> Self {
        Self { start_time_us: 0 }
    }
}

impl Default for MockTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl TimerInterface for MockTimer {
    fn delay_us(&mut self, us: u32) -> Result<()> {
        // In mock implementation, we just pretend to delay
        // Real tests would use actual std::thread::sleep if needed
        self.start_time_us = self.start_time_us.wrapping_add(us as u64);
        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<()> {
        self.delay_us(ms.saturating_mul(1000))
    }

    fn now_us(&self) -> u64 {
        self.start_time_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_timer_delay_us() {
        let mut timer = MockTimer::new();
        assert_eq!(timer.now_us(), 0);

        timer.delay_us(1000).unwrap();
        assert_eq!(timer.now_us(), 1000);

        timer.delay_us(500).unwrap();
        assert_eq!(timer.now_us(), 1500);
    }

    #[test]
    fn test_mock_timer_delay_ms() {
        let mut timer = MockTimer::new();
        assert_eq!(timer.now_us(), 0);

        timer.delay_ms(1).unwrap();
        assert_eq!(timer.now_us(), 1000);

        timer.delay_ms(5).unwrap();
        assert_eq!(timer.now_us(), 6000);
    }

    #[test]
    fn test_mock_timer_now_ms() {
        let mut timer = MockTimer::new();
        timer.delay_us(3500).unwrap();
        assert_eq!(timer.now_ms(), 3);
    }
}
