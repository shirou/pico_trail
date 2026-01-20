//! Arming System Async Tasks
//!
//! Provides Embassy async task integration for multi-rate armed state monitoring.
//!
//! # Usage
//!
//! When Embassy executor is available, spawn monitoring tasks after arming:
//!
//! ```ignore
//! use embassy_executor::Spawner;
//!
//! // After successful arming
//! spawner.spawn(monitoring_task_fast(monitor_handle)).ok();
//! spawner.spawn(monitoring_task_medium(monitor_handle)).ok();
//! spawner.spawn(monitoring_task_slow(monitor_handle)).ok();
//! ```

use embassy_time::{Duration, Timer};

/// Fast monitoring task (400 Hz / 2.5ms period)
///
/// Monitors:
/// - RC signal age and timeout
/// - Sensor health flags
///
/// This task runs at high frequency to detect rapid changes in critical
/// systems like RC signal loss.
///
/// # Arguments
///
/// * `monitor` - Shared reference to ArmedStateMonitor (requires Mutex/RefCell wrapper)
/// * `state` - Shared reference to SystemState (requires Mutex/RefCell wrapper)
///
/// # Example
///
/// ```ignore
/// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
/// use embassy_sync::mutex::Mutex;
///
/// static MONITOR: Mutex<CriticalSectionRawMutex, ArmedStateMonitor> = ...;
/// static STATE: Mutex<CriticalSectionRawMutex, SystemState> = ...;
///
/// #[embassy_executor::task]
/// async fn monitoring_fast() {
///     loop {
///         let current_time_ms = embassy_time::Instant::now().as_millis();
///
///         let mut monitor = MONITOR.lock().await;
///         let state = STATE.lock().await;
///
///         if let Some(reason) = monitor.update_fast(&state, current_time_ms) {
///             // Trigger failsafe action
///             warn!("Failsafe triggered: {}", reason);
///         }
///
///         drop(state);
///         drop(monitor);
///
///         Timer::after(Duration::from_millis(2)).await; // ~400 Hz (2.5ms)
///     }
/// }
/// ```
pub async fn monitoring_loop_fast<F>(mut check_fn: F)
where
    F: FnMut(u64) -> Option<super::monitoring::FailsafeReason>,
{
    loop {
        let current_time_ms = embassy_time::Instant::now().as_millis();

        if let Some(reason) = check_fn(current_time_ms) {
            crate::log_warn!("Fast monitor failsafe: {}", reason);
            // TODO: Trigger failsafe action via channel
        }

        Timer::after(Duration::from_micros(2500)).await; // 400 Hz (2.5ms)
    }
}

/// Medium monitoring task (10 Hz / 100ms period)
///
/// Monitors:
/// - Battery voltage vs critical threshold
/// - EKF health validation
///
/// This task runs at medium frequency for health metrics that don't
/// require sub-millisecond response times.
///
/// # Example
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn monitoring_medium() {
///     loop {
///         let mut monitor = MONITOR.lock().await;
///         let state = STATE.lock().await;
///
///         if let Some(reason) = monitor.update_medium(&state) {
///             warn!("Failsafe triggered: {}", reason);
///         }
///
///         drop(state);
///         drop(monitor);
///
///         Timer::after(Duration::from_millis(100)).await; // 10 Hz
///     }
/// }
/// ```
pub async fn monitoring_loop_medium<F>(mut check_fn: F)
where
    F: FnMut() -> Option<super::monitoring::FailsafeReason>,
{
    loop {
        if let Some(reason) = check_fn() {
            crate::log_warn!("Medium monitor failsafe: {}", reason);
            // TODO: Trigger failsafe action via channel
        }

        Timer::after(Duration::from_millis(100)).await; // 10 Hz
    }
}

/// Slow monitoring task (1 Hz / 1000ms period)
///
/// Monitors:
/// - Geofence violations
/// - GCS heartbeat timeout
/// - Sends SYS_STATUS to GCS
///
/// This task runs at low frequency for non-critical monitoring and
/// periodic telemetry reporting.
///
/// # Example
///
/// ```ignore
/// #[embassy_executor::task]
/// async fn monitoring_slow() {
///     loop {
///         let current_time_ms = embassy_time::Instant::now().as_millis();
///
///         let mut monitor = MONITOR.lock().await;
///         let state = STATE.lock().await;
///
///         if let Some(reason) = monitor.update_slow(&state, current_time_ms) {
///             warn!("Failsafe triggered: {}", reason);
///         }
///
///         drop(state);
///         drop(monitor);
///
///         Timer::after(Duration::from_millis(1000)).await; // 1 Hz
///     }
/// }
/// ```
pub async fn monitoring_loop_slow<F>(mut check_fn: F)
where
    F: FnMut(u64) -> Option<super::monitoring::FailsafeReason>,
{
    loop {
        let current_time_ms = embassy_time::Instant::now().as_millis();

        if let Some(reason) = check_fn(current_time_ms) {
            crate::log_warn!("Slow monitor failsafe: {}", reason);
            // TODO: Trigger failsafe action via channel
        }

        Timer::after(Duration::from_secs(1)).await; // 1 Hz
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_monitoring_functions_exist() {
        // This test verifies that the monitoring functions are defined
        // They can only be called in async context, so we just check they exist
        let _ = super::monitoring_loop_fast::<fn(u64) -> Option<super::super::monitoring::FailsafeReason>>;
        let _ = super::monitoring_loop_medium::<fn() -> Option<super::super::monitoring::FailsafeReason>>;
        let _ = super::monitoring_loop_slow::<fn(u64) -> Option<super::super::monitoring::FailsafeReason>>;
    }
}
