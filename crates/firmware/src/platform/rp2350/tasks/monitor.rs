//! Monitoring task for scheduler health and performance (Embassy version)
//!
//! This module provides the Embassy async monitoring task that periodically
//! collects statistics from all tasks. The core monitoring logic remains in
//! `crate::core::scheduler::monitor`.

use crate::core::scheduler::monitor::collect_and_report_stats;
use embassy_time::{Duration, Instant, Ticker};

/// Monitoring task (Embassy version)
///
/// Runs at 1Hz to collect and report scheduler statistics.
/// This task should be spawned alongside user tasks.
#[embassy_executor::task]
pub async fn monitor_task() {
    let start = Instant::now();
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        let uptime_ms = start.elapsed().as_millis();
        collect_and_report_stats(uptime_ms);
    }
}
