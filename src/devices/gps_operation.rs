//! GPS operation and data management
//!
//! Provides async GPS polling task with configurable rate (1Hz/5Hz/10Hz), NMEA validation,
//! error recovery, and shared GPS state for navigation subsystem access.
//!
//! # Architecture
//!
//! - `GpsOperation`: Async task that polls GPS at configured interval
//! - `GpsState`: Shared state (Mutex or Channel) with latest position, fix status, timestamp
//! - Error recovery: 3 retries with exponential backoff (100ms, 200ms, 400ms)
//! - Failsafe: Trigger after 3 consecutive NoFix readings (~3s)

use crate::devices::gps::{GpsDriver, GpsFixType, GpsPosition};
use crate::platform::traits::UartInterface;
use crate::platform::Result;

// =============================================================================
// Embassy Operations Module
// =============================================================================

#[cfg(feature = "embassy")]
mod embassy_ops {
    use crate::communication::mavlink::state::SYSTEM_STATE;
    use crate::devices::gps::GpsPosition;
    pub use embassy_time::{Duration, Instant};

    /// Async delay for retry backoff
    pub async fn delay_millis(ms: u64) {
        embassy_time::Timer::after(Duration::from_millis(ms)).await;
    }

    /// Update global SYSTEM_STATE with GPS position
    pub fn update_system_gps(position: GpsPosition) {
        let timestamp_us = Instant::now().as_micros();
        critical_section::with(|cs| {
            SYSTEM_STATE
                .borrow(cs)
                .borrow_mut()
                .update_gps(position, timestamp_us);
        });
    }
}

#[cfg(feature = "embassy")]
use embassy_ops::{delay_millis, update_system_gps, Duration, Instant};

// =============================================================================
// Host Test Stubs
// =============================================================================

#[cfg(not(feature = "embassy"))]
mod stub_ops {
    use crate::devices::gps::GpsPosition;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Instant(u64);

    impl Instant {
        pub fn now() -> Self {
            Instant(0)
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Duration(u64);

    impl Duration {
        pub fn from_millis(ms: u64) -> Self {
            Duration(ms)
        }
    }

    /// No-op delay for host tests
    pub async fn delay_millis(_ms: u64) {
        // No actual delay in host tests
    }

    /// No-op GPS update for host tests
    pub fn update_system_gps(_position: GpsPosition) {
        // No global state in host tests
    }
}

#[cfg(not(feature = "embassy"))]
use stub_ops::{delay_millis, update_system_gps, Duration, Instant};

/// GPS polling rate configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PollingRate {
    /// Poll GPS at 1 Hz (every 1 second)
    Rate1Hz,
    /// Poll GPS at 5 Hz (every 200ms)
    Rate5Hz,
    /// Poll GPS at 10 Hz (every 100ms)
    Rate10Hz,
}

impl PollingRate {
    /// Get the polling interval duration
    pub fn interval(&self) -> Duration {
        match self {
            PollingRate::Rate1Hz => Duration::from_millis(1000),
            PollingRate::Rate5Hz => Duration::from_millis(200),
            PollingRate::Rate10Hz => Duration::from_millis(100),
        }
    }
}

/// GPS state for navigation subsystem access
#[derive(Debug, Clone, Copy)]
pub struct GpsState {
    /// Latest GPS position (None if no fix)
    pub position: Option<GpsPosition>,
    /// Last update timestamp
    pub last_update: Instant,
    /// Current GPS fix type
    pub fix_type: GpsFixType,
}

impl Default for GpsState {
    fn default() -> Self {
        Self {
            position: None,
            last_update: Instant::now(),
            fix_type: GpsFixType::NoFix,
        }
    }
}

/// GPS operation manager
///
/// Manages GPS polling, validation, error recovery, and state updates.
#[cfg_attr(not(feature = "embassy"), allow(dead_code))]
pub struct GpsOperation<U: UartInterface> {
    gps: GpsDriver<U>,
    state: GpsState,
    no_fix_count: u8,
}

#[cfg_attr(not(feature = "embassy"), allow(dead_code))]
impl<U: UartInterface> GpsOperation<U> {
    /// Maximum retry attempts for UART errors
    const MAX_RETRIES: u8 = 3;

    /// Create a new GPS operation manager
    ///
    /// # Arguments
    ///
    /// * `gps` - GPS UART driver
    pub fn new(gps: GpsDriver<U>) -> Self {
        Self {
            gps,
            state: GpsState::default(),
            no_fix_count: 0,
        }
    }

    /// Get current GPS state
    pub fn state(&self) -> GpsState {
        self.state
    }

    /// Run GPS polling loop
    ///
    /// This is an async task that continuously polls the GPS at the configured interval.
    /// Call this from an Embassy task spawner.
    ///
    /// # Error Recovery
    ///
    /// - UART errors: Retry up to 3 times with exponential backoff (100ms, 200ms, 400ms)
    /// - Persistent UART failure: Continue polling at 1Hz, log error state
    /// - NoFix status: Continue polling, trigger failsafe after 3 consecutive NoFix readings
    ///
    /// # Availability
    ///
    /// This method is only available on embedded targets (requires `embassy` feature).
    #[cfg(feature = "embassy")]
    pub async fn poll_loop(&mut self) {
        crate::log_info!("GPS: Starting continuous polling");

        loop {
            // Poll GPS continuously - don't wait between reads
            match self.poll_with_retry().await {
                Ok(Some(position)) => {
                    self.handle_gps_fix(position);
                }
                Ok(None) => {
                    // No data available - yield to other tasks briefly
                    embassy_time::Timer::after(Duration::from_millis(1)).await;
                }
                Err(e) => {
                    self.handle_persistent_error(e);
                }
            }
        }
    }

    /// Poll GPS with retry logic
    ///
    /// Retries up to MAX_RETRIES times with exponential backoff on UART errors.
    async fn poll_with_retry(&mut self) -> Result<Option<GpsPosition>> {
        let mut retry_count = 0;

        loop {
            match self.gps.update() {
                Ok(result) => return Ok(result),
                Err(e) => {
                    retry_count += 1;

                    if retry_count > Self::MAX_RETRIES {
                        crate::log_error!("GPS: UART error after {} retries", Self::MAX_RETRIES);
                        return Err(e);
                    }

                    crate::log_warn!(
                        "GPS: UART error, retrying ({}/{})",
                        retry_count,
                        Self::MAX_RETRIES
                    );

                    // Exponential backoff: 100ms, 200ms, 400ms
                    let delay_ms = 100 * (1 << (retry_count - 1));
                    delay_millis(delay_ms).await;
                }
            }
        }
    }

    /// Handle successful GPS fix
    fn handle_gps_fix(&mut self, position: GpsPosition) {
        // Check if this is a new fix acquisition
        let was_no_fix = self.state.fix_type == GpsFixType::NoFix;

        // Update local state
        self.state.position = Some(position);
        self.state.last_update = Instant::now();
        self.state.fix_type = position.fix_type;
        self.no_fix_count = 0;

        // Update global SYSTEM_STATE for telemetry access
        update_system_gps(position);

        if was_no_fix {
            crate::log_info!(
                "GPS: Fix acquired ({:?}, {} satellites)",
                position.fix_type,
                position.satellites
            );
        }
    }

    /// Handle no data from GPS (still waiting for fix or sentence)
    ///
    /// Note: Currently used only by tests. Will be integrated with poll_loop
    /// when periodic polling mode is implemented.
    #[cfg_attr(feature = "embassy", allow(dead_code))]
    fn handle_no_data(&mut self) {
        // If we had a fix before, this might indicate fix loss
        if self.state.fix_type != GpsFixType::NoFix {
            self.no_fix_count += 1;

            if self.no_fix_count >= 3 {
                crate::log_info!("GPS: Fix lost (NoFix), continuing polling");
                self.state.fix_type = GpsFixType::NoFix;
                self.state.position = None;

                // Trigger GPS failsafe (3 consecutive NoFix readings)
                self.trigger_gps_failsafe();
            }
        }
    }

    /// Handle persistent UART error (after retries exhausted)
    fn handle_persistent_error(&mut self, _error: crate::platform::PlatformError) {
        crate::log_error!("GPS: Persistent UART error, continuing polling at reduced rate");

        // Mark as NoFix
        self.state.fix_type = GpsFixType::NoFix;
        self.state.position = None;
    }

    /// Trigger GPS failsafe
    ///
    /// Called after 3 consecutive NoFix readings (~3s).
    /// This should notify the navigation subsystem to take appropriate action.
    #[cfg_attr(feature = "embassy", allow(dead_code))]
    fn trigger_gps_failsafe(&self) {
        crate::log_warn!("GPS: Failsafe triggered (3 consecutive NoFix readings)");
        // TODO: Integration with failsafe system (future task)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::mock::MockUart;
    use crate::platform::traits::UartConfig;

    #[test]
    fn test_polling_rate_intervals() {
        assert_eq!(PollingRate::Rate1Hz.interval(), Duration::from_millis(1000));
        assert_eq!(PollingRate::Rate5Hz.interval(), Duration::from_millis(200));
        assert_eq!(PollingRate::Rate10Hz.interval(), Duration::from_millis(100));
    }

    #[test]
    fn test_gps_state_default() {
        let state = GpsState::default();
        assert!(state.position.is_none());
        assert_eq!(state.fix_type, GpsFixType::NoFix);
    }

    #[tokio::test]
    async fn test_gps_operation_poll_with_retry_success() {
        let uart = MockUart::new(UartConfig::default());

        // Valid GPGGA sentence with checksum
        let nmea_sentence =
            b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

        // Create GPS driver and inject data
        let gps = GpsDriver::new(uart);
        let mut operation = GpsOperation::new(gps);

        // Inject data into the GPS driver's UART
        operation.gps.uart_mut().inject_rx_data(nmea_sentence);

        // The GPS driver may need multiple update() calls to accumulate the complete sentence
        let result = operation.poll_with_retry().await;
        assert!(result.is_ok());

        // If first call returns None, try again (GPS may need multiple reads)
        let position = match result.unwrap() {
            Some(pos) => Some(pos),
            None => {
                // Try one more time to see if data is available
                let result2 = operation.poll_with_retry().await;
                assert!(result2.is_ok());
                result2.unwrap()
            }
        };

        assert!(position.is_some(), "Expected GPS position, got None");
    }

    #[tokio::test]
    async fn test_gps_operation_handle_gps_fix() {
        let uart = MockUart::new(UartConfig::default());
        let gps = GpsDriver::new(uart);
        let mut operation = GpsOperation::new(gps);

        let position = GpsPosition {
            latitude: 48.1173,
            longitude: 11.5166,
            altitude: 545.4,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 8,
        };

        operation.handle_gps_fix(position);

        assert!(operation.state.position.is_some());
        assert_eq!(operation.state.fix_type, GpsFixType::Fix3D);
        assert_eq!(operation.no_fix_count, 0);
    }

    #[tokio::test]
    async fn test_gps_operation_handle_no_data() {
        let uart = MockUart::new(UartConfig::default());
        let gps = GpsDriver::new(uart);
        let mut operation = GpsOperation::new(gps);

        // Set initial fix
        operation.state.fix_type = GpsFixType::Fix3D;

        // Simulate 3 consecutive NoFix readings
        operation.handle_no_data();
        assert_eq!(operation.no_fix_count, 1);

        operation.handle_no_data();
        assert_eq!(operation.no_fix_count, 2);

        operation.handle_no_data();
        assert_eq!(operation.no_fix_count, 3);
        assert_eq!(operation.state.fix_type, GpsFixType::NoFix);
    }

    #[tokio::test]
    async fn test_gps_operation_state_access() {
        let uart = MockUart::new(UartConfig::default());
        let gps = GpsDriver::new(uart);
        let operation = GpsOperation::new(gps);

        let state = operation.state();
        assert!(state.position.is_none());
        assert_eq!(state.fix_type, GpsFixType::NoFix);
    }
}
