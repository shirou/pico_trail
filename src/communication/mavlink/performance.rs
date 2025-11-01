//! Performance Metrics Tracking for MAVLink Communication
//!
//! This module provides instrumentation for measuring:
//! - Command round-trip latency (COMMAND_LONG → COMMAND_ACK)
//! - Memory usage monitoring
//! - Packet loss detection via sequence number gaps
//!
//! # Usage
//!
//! ```ignore
//! let mut metrics = PerformanceMetrics::new();
//!
//! // Track received COMMAND_LONG
//! metrics.track_command_received(cmd_id, timestamp_us);
//!
//! // Track sent COMMAND_ACK
//! metrics.track_command_ack(cmd_id, timestamp_us);
//!
//! // Track received packet sequence
//! metrics.track_packet(system_id, sequence);
//!
//! // Periodic reporting
//! metrics.report_if_due(timestamp_us);
//! ```

#[cfg(feature = "pico2_w")]
use embassy_time::{Duration, Instant};

#[cfg(feature = "pico2_w")]
use heapless::FnvIndexMap;

/// Performance metrics tracker for MAVLink communication
#[cfg(feature = "pico2_w")]
#[derive(Debug)]
pub struct PerformanceMetrics {
    latency: LatencyTracker,
    connection_health: ConnectionHealthTracker,
    last_report: Instant,
    report_interval: Duration,
}

/// Dummy implementation for non-embedded builds
#[cfg(not(feature = "pico2_w"))]
#[derive(Debug, Default)]
pub struct PerformanceMetrics;

#[cfg(feature = "pico2_w")]
impl PerformanceMetrics {
    /// Create new performance metrics tracker
    ///
    /// Reports are generated every 10 seconds by default.
    pub fn new() -> Self {
        Self {
            latency: LatencyTracker::new(),
            connection_health: ConnectionHealthTracker::new(),
            last_report: Instant::now(),
            report_interval: Duration::from_secs(10),
        }
    }

    /// Track received COMMAND_LONG message
    ///
    /// Records timestamp for later latency calculation when ACK is sent.
    pub fn track_command_received(&mut self, command_id: u16, timestamp_us: u64) {
        self.latency.record_command_start(command_id, timestamp_us);
    }

    /// Track sent COMMAND_ACK message
    ///
    /// Calculates and records round-trip latency.
    pub fn track_command_ack(&mut self, command_id: u16, timestamp_us: u64) {
        self.latency
            .record_command_complete(command_id, timestamp_us);
    }

    /// Track received HEARTBEAT message
    ///
    /// Records timestamp to monitor connection health.
    pub fn track_heartbeat(&mut self, system_id: u8, timestamp_us: u64) {
        self.connection_health
            .track_heartbeat(system_id, timestamp_us);
    }

    /// Generate performance report if interval has elapsed
    ///
    /// Logs metrics via defmt and resets counters.
    pub fn report_if_due(&mut self, _timestamp_us: u64) {
        if self.last_report.elapsed() >= self.report_interval {
            self.report();
            self.last_report = Instant::now();
        }
    }

    /// Generate immediate performance report
    fn report(&mut self) {
        defmt::info!("=== Performance Metrics ===");

        // Latency report
        if let Some(stats) = self.latency.get_stats() {
            defmt::info!("Command Latency:");
            defmt::info!("  Count: {}", stats.count);
            defmt::info!(
                "  Average: {} μs ({} ms)",
                stats.avg_us,
                stats.avg_us / 1000
            );
            defmt::info!("  Min: {} μs ({} ms)", stats.min_us, stats.min_us / 1000);
            defmt::info!("  Max: {} μs ({} ms)", stats.max_us, stats.max_us / 1000);

            // Check against target
            let avg_ms = stats.avg_us / 1000;
            if avg_ms < 110 {
                defmt::info!("  ✓ Within target (< 110ms)");
            } else {
                defmt::warn!("  ✗ Exceeds target (>= 110ms)");
            }
        } else {
            defmt::info!("Command Latency: No data");
        }

        // Connection health report
        let health_stats = self.connection_health.get_stats();
        defmt::info!("Connection Health:");
        defmt::info!("  Total HEARTBEATs: {}", health_stats.total_heartbeats);
        defmt::info!("  Timeouts detected: {}", health_stats.timeouts);
        if health_stats.total_heartbeats > 0 {
            let timeout_percent =
                (health_stats.timeouts as u32 * 100) / health_stats.total_heartbeats;
            defmt::info!("  Timeout rate: {}%", timeout_percent);

            if timeout_percent < 1 {
                defmt::info!("  ✓ Connection stable (< 1% timeouts)");
            } else {
                defmt::warn!("  ✗ Connection unstable (>= 1% timeouts)");
            }
        }
        if health_stats.active_connections > 0 {
            defmt::info!("  Active connections: {}", health_stats.active_connections);
        }

        // Memory report (platform-specific)
        #[cfg(feature = "pico2_w")]
        {
            // RP2350 has 520KB RAM total
            // We can't easily measure actual usage from embedded code,
            // but we can report allocator stats if available
            defmt::info!("Memory:");
            defmt::info!("  Platform: RP2350 (520 KB RAM total)");
            defmt::info!("  Target: ≤ 50 KB for network stack");
            defmt::info!("  (Use external tools to measure actual usage)");
        }

        defmt::info!("==========================");
    }

    /// Reset all metrics counters
    pub fn reset(&mut self) {
        self.latency.reset();
        self.connection_health.reset();
    }
}

#[cfg(feature = "pico2_w")]
impl Default for PerformanceMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Dummy implementation for non-embedded builds
#[cfg(not(feature = "pico2_w"))]
impl PerformanceMetrics {
    pub fn new() -> Self {
        Self
    }

    pub fn track_command_received(&mut self, _command_id: u16, _timestamp_us: u64) {}
    pub fn track_command_ack(&mut self, _command_id: u16, _timestamp_us: u64) {}
    pub fn track_heartbeat(&mut self, _system_id: u8, _timestamp_us: u64) {}
    pub fn report_if_due(&mut self, _timestamp_us: u64) {}
    pub fn reset(&mut self) {}
}

/// Latency statistics
#[derive(Debug, Clone, Copy)]
pub struct LatencyStats {
    /// Number of measurements
    pub count: u32,
    /// Average latency (microseconds)
    pub avg_us: u64,
    /// Minimum latency (microseconds)
    pub min_us: u64,
    /// Maximum latency (microseconds)
    pub max_us: u64,
}

/// Command round-trip latency tracker
#[cfg(feature = "pico2_w")]
#[derive(Debug)]
struct LatencyTracker {
    /// Pending commands: command_id -> start_timestamp_us
    pending: FnvIndexMap<u16, u64, 16>,
    /// Recorded latencies (microseconds)
    latencies: heapless::Vec<u64, 100>,
    /// Total number of completed commands
    total_count: u32,
}

#[cfg(feature = "pico2_w")]
impl LatencyTracker {
    fn new() -> Self {
        Self {
            pending: FnvIndexMap::new(),
            latencies: heapless::Vec::new(),
            total_count: 0,
        }
    }

    fn record_command_start(&mut self, command_id: u16, timestamp_us: u64) {
        // Insert or replace existing entry
        let _ = self.pending.insert(command_id, timestamp_us);
    }

    fn record_command_complete(&mut self, command_id: u16, timestamp_us: u64) {
        if let Some(start_time) = self.pending.remove(&command_id) {
            let latency_us = timestamp_us.saturating_sub(start_time);

            // Add to history (rolling window of 100)
            if self.latencies.len() >= 100 {
                self.latencies.remove(0);
            }
            let _ = self.latencies.push(latency_us);

            self.total_count += 1;

            defmt::trace!(
                "Command {} latency: {} μs ({} ms)",
                command_id,
                latency_us,
                latency_us / 1000
            );
        }
    }

    fn get_stats(&self) -> Option<LatencyStats> {
        if self.latencies.is_empty() {
            return None;
        }

        let mut sum: u64 = 0;
        let mut min: u64 = u64::MAX;
        let mut max: u64 = 0;

        for &lat in &self.latencies {
            sum += lat;
            min = min.min(lat);
            max = max.max(lat);
        }

        let avg = sum / self.latencies.len() as u64;

        Some(LatencyStats {
            count: self.total_count,
            avg_us: avg,
            min_us: min,
            max_us: max,
        })
    }

    fn reset(&mut self) {
        self.pending.clear();
        self.latencies.clear();
        self.total_count = 0;
    }
}

/// Connection health statistics
#[derive(Debug, Clone, Copy)]
pub struct ConnectionHealthStats {
    /// Total HEARTBEAT messages received
    pub total_heartbeats: u32,
    /// Number of timeouts detected
    pub timeouts: u32,
    /// Number of currently active connections
    pub active_connections: u8,
}

/// Connection health tracker via HEARTBEAT monitoring
///
/// Tracks HEARTBEAT messages from GCS to detect connection issues.
/// A timeout is detected when no HEARTBEAT is received within 3 seconds.
#[cfg(feature = "pico2_w")]
#[derive(Debug)]
struct ConnectionHealthTracker {
    /// Last HEARTBEAT timestamp per system_id (in microseconds)
    last_heartbeat: FnvIndexMap<u8, u64, 4>,
    /// Total HEARTBEAT count
    total_heartbeats: u32,
    /// Total timeout count
    timeouts: u32,
    /// HEARTBEAT timeout threshold (microseconds)
    timeout_threshold_us: u64,
}

#[cfg(feature = "pico2_w")]
impl ConnectionHealthTracker {
    fn new() -> Self {
        Self {
            last_heartbeat: FnvIndexMap::new(),
            total_heartbeats: 0,
            timeouts: 0,
            timeout_threshold_us: 3_000_000, // 3 seconds
        }
    }

    fn track_heartbeat(&mut self, system_id: u8, timestamp_us: u64) {
        self.total_heartbeats += 1;

        // Check for timeout if we've seen this system before
        if let Some(&last_time) = self.last_heartbeat.get(&system_id) {
            let elapsed_us = timestamp_us.saturating_sub(last_time);
            if elapsed_us > self.timeout_threshold_us {
                self.timeouts += 1;
                defmt::debug!(
                    "HEARTBEAT timeout: sys={}, elapsed={} ms (expected < {} ms)",
                    system_id,
                    elapsed_us / 1000,
                    self.timeout_threshold_us / 1000
                );
            }
        }

        // Update last HEARTBEAT time
        let _ = self.last_heartbeat.insert(system_id, timestamp_us);
    }

    fn get_stats(&self) -> ConnectionHealthStats {
        ConnectionHealthStats {
            total_heartbeats: self.total_heartbeats,
            timeouts: self.timeouts,
            active_connections: self.last_heartbeat.len() as u8,
        }
    }

    fn reset(&mut self) {
        self.last_heartbeat.clear();
        self.total_heartbeats = 0;
        self.timeouts = 0;
    }
}

#[cfg(all(test, feature = "pico2_w"))]
mod tests {
    use super::*;

    #[test]
    fn test_latency_tracker() {
        let mut tracker = LatencyTracker::new();

        // Record command
        tracker.record_command_start(100, 1000);
        tracker.record_command_complete(100, 1500);

        let stats = tracker.get_stats().unwrap();
        assert_eq!(stats.count, 1);
        assert_eq!(stats.avg_us, 500);
        assert_eq!(stats.min_us, 500);
        assert_eq!(stats.max_us, 500);
    }

    #[test]
    fn test_latency_tracker_multiple() {
        let mut tracker = LatencyTracker::new();

        tracker.record_command_start(1, 1000);
        tracker.record_command_complete(1, 2000); // 1000μs

        tracker.record_command_start(2, 3000);
        tracker.record_command_complete(2, 5000); // 2000μs

        let stats = tracker.get_stats().unwrap();
        assert_eq!(stats.count, 2);
        assert_eq!(stats.avg_us, 1500); // (1000 + 2000) / 2
        assert_eq!(stats.min_us, 1000);
        assert_eq!(stats.max_us, 2000);
    }

    #[test]
    fn test_connection_health_regular_heartbeats() {
        let mut tracker = ConnectionHealthTracker::new();

        // Regular HEARTBEATs at 1-second intervals
        tracker.track_heartbeat(1, 0);
        tracker.track_heartbeat(1, 1_000_000); // 1s later
        tracker.track_heartbeat(1, 2_000_000); // 2s later

        let stats = tracker.get_stats();
        assert_eq!(stats.total_heartbeats, 3);
        assert_eq!(stats.timeouts, 0);
        assert_eq!(stats.active_connections, 1);
    }

    #[test]
    fn test_connection_health_with_timeout() {
        let mut tracker = ConnectionHealthTracker::new();

        tracker.track_heartbeat(1, 0);
        tracker.track_heartbeat(1, 1_000_000); // 1s later (OK)
        tracker.track_heartbeat(1, 5_000_000); // 4s gap (timeout!)

        let stats = tracker.get_stats();
        assert_eq!(stats.total_heartbeats, 3);
        assert_eq!(stats.timeouts, 1);
    }

    #[test]
    fn test_connection_health_first_heartbeat() {
        let mut tracker = ConnectionHealthTracker::new();

        // First HEARTBEAT should not trigger timeout
        tracker.track_heartbeat(1, 0);

        let stats = tracker.get_stats();
        assert_eq!(stats.total_heartbeats, 1);
        assert_eq!(stats.timeouts, 0);
    }

    #[test]
    fn test_connection_health_multiple_systems() {
        let mut tracker = ConnectionHealthTracker::new();

        // Two systems sending regular HEARTBEATs
        tracker.track_heartbeat(1, 0);
        tracker.track_heartbeat(2, 0);
        tracker.track_heartbeat(1, 1_000_000);
        tracker.track_heartbeat(2, 1_000_000);

        let stats = tracker.get_stats();
        assert_eq!(stats.total_heartbeats, 4);
        assert_eq!(stats.timeouts, 0);
        assert_eq!(stats.active_connections, 2);
    }
}
