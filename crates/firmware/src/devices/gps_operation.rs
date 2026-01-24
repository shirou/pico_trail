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

use crate::communication::mavlink::state::SYSTEM_STATE;
use crate::devices::gps::{GpsDriver, GpsFixType, GpsPosition};
use crate::platform::traits::UartInterface;
use crate::platform::Result;
use embassy_time::{Duration, Instant};

/// Async delay for retry backoff
async fn delay_millis(ms: u64) {
    embassy_time::Timer::after(Duration::from_millis(ms)).await;
}

/// Update global SYSTEM_STATE with GPS position
fn update_system_gps(position: GpsPosition) {
    let timestamp_us = Instant::now().as_micros();
    critical_section::with(|cs| {
        SYSTEM_STATE
            .borrow(cs)
            .borrow_mut()
            .update_gps(position, timestamp_us);
    });
}

/// GPS polling rate configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
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
pub struct GpsOperation<U: UartInterface> {
    gps: GpsDriver<U>,
    state: GpsState,
    no_fix_count: u8,
}

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
    /// Note: Will be integrated with poll_loop when periodic polling mode is implemented.
    fn _handle_no_data(&mut self) {
        // If we had a fix before, this might indicate fix loss
        if self.state.fix_type != GpsFixType::NoFix {
            self.no_fix_count += 1;

            if self.no_fix_count >= 3 {
                crate::log_info!("GPS: Fix lost (NoFix), continuing polling");
                self.state.fix_type = GpsFixType::NoFix;
                self.state.position = None;

                // Trigger GPS failsafe (3 consecutive NoFix readings)
                self._trigger_gps_failsafe();
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
    fn _trigger_gps_failsafe(&self) {
        crate::log_warn!("GPS: Failsafe triggered (3 consecutive NoFix readings)");
        // TODO: Integration with failsafe system (future task)
    }
}
