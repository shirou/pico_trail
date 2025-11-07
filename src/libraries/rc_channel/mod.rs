//! RC input state and processing
//!
//! This module handles RC input from MAVLink RC_CHANNELS messages, including:
//! - Channel normalization (0-65535 → -1.0 to +1.0)
//! - Timeout detection (1 second threshold)
//! - RC status tracking (Active, Lost, NeverConnected)
//!
//! ## References
//!
//! - ADR-ea7fw-rc-input-processing: RC input design
//! - FR-993xy-rc-channels-processing: RC requirements

#[cfg(feature = "pico2_w")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "pico2_w")]
use embassy_sync::mutex::Mutex;

/// RC timeout threshold (1 second in microseconds)
const RC_TIMEOUT_US: u64 = 1_000_000;

/// RC input state (shared between MAVLink handler and vehicle task)
pub struct RcInput {
    /// Channel values (normalized -1.0 to +1.0)
    pub channels: [f32; 18],
    /// Number of active channels
    pub channel_count: u8,
    /// Last update timestamp (microseconds)
    pub last_update_us: u64,
    /// RC connection status
    pub status: RcStatus,
}

/// RC connection status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RcStatus {
    /// RC input active, recent message received
    Active,
    /// RC input lost, timeout exceeded
    Lost,
    /// RC never connected
    NeverConnected,
}

impl Default for RcInput {
    fn default() -> Self {
        Self::new()
    }
}

impl RcInput {
    /// Create new RC input state
    pub const fn new() -> Self {
        Self {
            channels: [0.0; 18],
            channel_count: 0,
            last_update_us: 0,
            status: RcStatus::NeverConnected,
        }
    }

    /// Update from MAVLink RC_CHANNELS message
    ///
    /// # Arguments
    ///
    /// * `raw_channels` - Raw channel values (0-65535)
    /// * `channel_count` - Number of active channels
    /// * `current_time_us` - Current timestamp in microseconds
    pub fn update_from_mavlink(
        &mut self,
        raw_channels: &[u16],
        channel_count: u8,
        current_time_us: u64,
    ) {
        // Normalize channels
        let count = core::cmp::min(raw_channels.len(), 18);
        for (i, &raw_value) in raw_channels.iter().enumerate().take(count) {
            self.channels[i] = Self::normalize_channel(raw_value);
        }

        self.channel_count = channel_count;
        self.last_update_us = current_time_us;
        self.status = RcStatus::Active;
    }

    /// Normalize channel (0-65535 → -1.0 to +1.0)
    ///
    /// # MAVLink RC_CHANNELS specification
    ///
    /// - 0 = minimum (-1.0)
    /// - 32768 = center (0.0)
    /// - 65535 = maximum (+1.0)
    ///
    /// # Arguments
    ///
    /// * `raw` - Raw channel value (0-65535)
    ///
    /// # Returns
    ///
    /// Normalized value (-1.0 to +1.0)
    pub fn normalize_channel(raw: u16) -> f32 {
        // Convert 0-65535 to -1.0 to +1.0
        // Formula: (raw - 32768) / 32768.0
        let centered = raw as i32 - 32768;
        centered as f32 / 32768.0
    }

    /// Get channel value (1-indexed, like MAVLink)
    ///
    /// # Arguments
    ///
    /// * `channel` - Channel number (1-18)
    ///
    /// # Returns
    ///
    /// Normalized channel value (-1.0 to +1.0), or 0.0 if out of range
    pub fn get_channel(&self, channel: usize) -> f32 {
        if (1..=18).contains(&channel) {
            self.channels[channel - 1]
        } else {
            0.0
        }
    }

    /// Check timeout (call at 50 Hz from vehicle task)
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current timestamp in microseconds
    pub fn check_timeout(&mut self, current_time_us: u64) {
        if self.status == RcStatus::Active {
            let elapsed_us = current_time_us.saturating_sub(self.last_update_us);
            if elapsed_us > RC_TIMEOUT_US {
                self.status = RcStatus::Lost;
                // Zero all channels on timeout
                self.channels.fill(0.0);
            }
        }
    }

    /// Check if RC is active
    pub fn is_active(&self) -> bool {
        self.status == RcStatus::Active
    }

    /// Check if RC is lost
    pub fn is_lost(&self) -> bool {
        self.status == RcStatus::Lost || self.status == RcStatus::NeverConnected
    }
}

/// Global RC input (protected by Mutex)
#[cfg(feature = "pico2_w")]
pub static RC_INPUT: Mutex<CriticalSectionRawMutex, RcInput> = Mutex::new(RcInput::new());

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_channel_boundaries() {
        // Minimum
        assert!((RcInput::normalize_channel(0) - (-1.0)).abs() < 0.001);

        // Center
        assert_eq!(RcInput::normalize_channel(32768), 0.0);

        // Maximum (floating point precision)
        assert!((RcInput::normalize_channel(65535) - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_normalize_channel_intermediate() {
        // Quarter negative
        let val = RcInput::normalize_channel(16384);
        assert!((val - (-0.5)).abs() < 0.01);

        // Quarter positive
        let val = RcInput::normalize_channel(49152);
        assert!((val - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_update_from_mavlink() {
        let mut rc = RcInput::new();
        assert_eq!(rc.status, RcStatus::NeverConnected);

        // Update with RC data
        let channels = [32768, 40000, 25000]; // Center, +25%, -25%
        rc.update_from_mavlink(&channels, 3, 1000);

        assert_eq!(rc.status, RcStatus::Active);
        assert_eq!(rc.channel_count, 3);
        assert_eq!(rc.last_update_us, 1000);

        // Check normalized values
        assert!((rc.channels[0] - 0.0).abs() < 0.01); // Center
        assert!(rc.channels[1] > 0.0); // Positive
        assert!(rc.channels[2] < 0.0); // Negative
    }

    #[test]
    fn test_get_channel() {
        let mut rc = RcInput::new();
        let channels = [16384, 32768, 49152]; // -50%, 0%, +50%
        rc.update_from_mavlink(&channels, 3, 1000);

        // 1-indexed access
        assert!(rc.get_channel(1) < 0.0);
        assert_eq!(rc.get_channel(2), 0.0);
        assert!(rc.get_channel(3) > 0.0);

        // Out of range
        assert_eq!(rc.get_channel(0), 0.0);
        assert_eq!(rc.get_channel(19), 0.0);
    }

    #[test]
    fn test_timeout_detection() {
        let mut rc = RcInput::new();

        // Initial update at timestamp 1_000_000 (1 second)
        let channels = [32768; 18];
        rc.update_from_mavlink(&channels, 18, 1_000_000);
        assert_eq!(rc.status, RcStatus::Active);

        // Check timeout before threshold (0.5 seconds later)
        rc.check_timeout(1_500_000);
        assert_eq!(rc.status, RcStatus::Active);

        // Check timeout after threshold (1.5 seconds later = 2.5 seconds total)
        rc.check_timeout(2_500_000);
        assert_eq!(rc.status, RcStatus::Lost);

        // Verify channels zeroed
        assert!(rc.channels.iter().all(|&ch| ch == 0.0));
    }

    #[test]
    fn test_is_active_is_lost() {
        let mut rc = RcInput::new();

        // Initially never connected
        assert!(!rc.is_active());
        assert!(rc.is_lost());

        // After update, active
        let channels = [32768; 18];
        rc.update_from_mavlink(&channels, 18, 1_000_000);
        assert!(rc.is_active());
        assert!(!rc.is_lost());

        // After timeout, lost
        rc.check_timeout(2_500_000);
        assert!(!rc.is_active());
        assert!(rc.is_lost());
    }
}
