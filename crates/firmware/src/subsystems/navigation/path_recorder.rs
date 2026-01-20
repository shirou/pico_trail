//! Path Recorder for SmartRTL
//!
//! Records GPS positions during armed operation for path retracing in SmartRTL mode.
//!
//! # Features
//!
//! - Ring buffer storage with configurable capacity
//! - Distance/time threshold for recording points
//! - Path simplification using distance-based algorithm
//! - Reverse iteration for return path generation
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::subsystems::navigation::path_recorder::{PathRecorder, PathPoint};
//!
//! let mut recorder = PathRecorder::new();
//!
//! // Start recording on arm
//! recorder.start();
//!
//! // Record points during operation
//! recorder.record(35.6762, 139.6503, timestamp_ms);
//!
//! // Get return path on RTL
//! for point in recorder.iter_return_path() {
//!     // Navigate to point
//! }
//!
//! // Stop recording on disarm
//! recorder.stop();
//! ```
//!
//! # References
//!
//! - FR-nywcm-smartrtl-path-recording: Path recording requirements
//! - FR-me7q8-smartrtl-path-simplification: Path simplification requirements
//! - NFR-leuwp-smartrtl-memory-budget: Memory budget constraint (< 10 KB)
//! - ArduPilot SmartRTL: https://ardupilot.org/rover/docs/smartrtl-mode.html

use super::geo::calculate_distance;

/// Maximum number of points in the path buffer
///
/// Based on NFR-leuwp-smartrtl-memory-budget: < 10 KB RAM
/// PathPoint size: ~16 bytes (4 + 4 + 4 + 4 padding)
/// 300 points × 16 bytes = 4.8 KB
const SRTL_POINTS_MAX: usize = 300;

/// Minimum distance between recorded points (meters)
///
/// Points closer than this are not recorded to avoid redundant data.
const SRTL_MIN_DISTANCE_M: f32 = 5.0;

/// Minimum time between recorded points (milliseconds)
///
/// Ensures at least one point every 3 seconds even if distance threshold not met.
const SRTL_MIN_TIME_MS: u32 = 3000;

/// Path simplification accuracy (meters)
///
/// Points within this distance of the simplified path are removed.
const SRTL_ACCURACY_M: f32 = 2.0;

/// Single point in the recorded path
#[derive(Clone, Copy, Debug, Default)]
pub struct PathPoint {
    /// Latitude in degrees
    pub latitude: f32,
    /// Longitude in degrees
    pub longitude: f32,
    /// Timestamp in milliseconds since recording start
    pub timestamp_ms: u32,
}

impl PathPoint {
    /// Create a new path point
    pub const fn new(latitude: f32, longitude: f32, timestamp_ms: u32) -> Self {
        Self {
            latitude,
            longitude,
            timestamp_ms,
        }
    }
}

/// Path recorder for SmartRTL
///
/// Records GPS positions during armed operation using a ring buffer.
/// Provides reverse iteration for path retracing.
pub struct PathRecorder {
    /// Ring buffer storage
    buffer: [PathPoint; SRTL_POINTS_MAX],
    /// Number of points currently stored
    count: usize,
    /// Write index (next position to write)
    write_idx: usize,
    /// Last recorded point (for distance/time threshold check)
    last_point: Option<PathPoint>,
    /// Recording state
    recording: bool,
    /// Start timestamp (ms)
    start_timestamp_ms: u32,
}

impl PathRecorder {
    /// Create a new path recorder
    pub const fn new() -> Self {
        Self {
            buffer: [PathPoint::new(0.0, 0.0, 0); SRTL_POINTS_MAX],
            count: 0,
            write_idx: 0,
            last_point: None,
            recording: false,
            start_timestamp_ms: 0,
        }
    }

    /// Start recording (called on arm)
    ///
    /// Clears any existing path and begins recording.
    pub fn start(&mut self) {
        self.clear();
        self.recording = true;
        crate::log_info!("PathRecorder: recording started");
    }

    /// Stop recording (called on disarm)
    pub fn stop(&mut self) {
        self.recording = false;
        crate::log_info!("PathRecorder: recording stopped, {} points", self.count);
    }

    /// Clear all recorded points
    pub fn clear(&mut self) {
        self.count = 0;
        self.write_idx = 0;
        self.last_point = None;
        self.start_timestamp_ms = 0;
    }

    /// Check if recording is active
    pub fn is_recording(&self) -> bool {
        self.recording
    }

    /// Record a GPS position
    ///
    /// The point is only recorded if:
    /// - Recording is active
    /// - Distance from last point >= SRTL_MIN_DISTANCE_M, OR
    /// - Time since last point >= SRTL_MIN_TIME_MS
    ///
    /// # Arguments
    ///
    /// * `latitude` - Latitude in degrees
    /// * `longitude` - Longitude in degrees
    /// * `timestamp_ms` - Current timestamp in milliseconds
    ///
    /// # Returns
    ///
    /// true if point was recorded, false if skipped or not recording
    pub fn record(&mut self, latitude: f32, longitude: f32, timestamp_ms: u32) -> bool {
        if !self.recording {
            return false;
        }

        // Set start timestamp on first point
        if self.count == 0 && self.start_timestamp_ms == 0 {
            self.start_timestamp_ms = timestamp_ms;
        }

        let relative_timestamp = timestamp_ms.saturating_sub(self.start_timestamp_ms);

        // Check if we should record this point
        if let Some(last) = &self.last_point {
            let distance = calculate_distance(last.latitude, last.longitude, latitude, longitude);
            let time_diff = relative_timestamp.saturating_sub(last.timestamp_ms);

            // Skip if neither distance nor time threshold met
            if distance < SRTL_MIN_DISTANCE_M && time_diff < SRTL_MIN_TIME_MS {
                return false;
            }
        }

        // Create new point
        let point = PathPoint::new(latitude, longitude, relative_timestamp);

        // Store in ring buffer
        self.buffer[self.write_idx] = point;
        self.write_idx = (self.write_idx + 1) % SRTL_POINTS_MAX;

        // Update count (cap at buffer size)
        if self.count < SRTL_POINTS_MAX {
            self.count += 1;
        }

        // Update last point
        self.last_point = Some(point);

        true
    }

    /// Check if a return path is available
    pub fn has_path(&self) -> bool {
        self.count > 0
    }

    /// Get the number of recorded points
    pub fn count(&self) -> usize {
        self.count
    }

    /// Get an iterator over the return path (reverse order)
    ///
    /// Returns points from most recent to oldest, which is the order
    /// needed for path retracing.
    pub fn iter_return_path(&self) -> ReturnPathIter<'_> {
        ReturnPathIter::new(self)
    }

    /// Simplify the recorded path
    ///
    /// Removes points that are within SRTL_ACCURACY_M of the line between
    /// their neighbors. This reduces memory usage while preserving path shape.
    ///
    /// Uses a simplified Douglas-Peucker-like algorithm optimized for
    /// sequential processing.
    pub fn simplify(&mut self) {
        if self.count < 3 {
            return; // Need at least 3 points to simplify
        }

        // Collect points into a temporary vector for simplification
        // Note: In a no_std environment, we work in-place or use a small stack buffer
        let mut keep_flags = [true; SRTL_POINTS_MAX];
        let mut kept_count = self.count;

        // Iterate through middle points
        for i in 1..self.count.saturating_sub(1) {
            if !keep_flags[i] {
                continue;
            }

            // Find previous kept point
            let mut prev_idx = i.saturating_sub(1);
            while prev_idx > 0 && !keep_flags[prev_idx] {
                prev_idx -= 1;
            }

            // Find next kept point
            let mut next_idx = i + 1;
            while next_idx < self.count && !keep_flags[next_idx] {
                next_idx += 1;
            }

            if next_idx >= self.count {
                continue;
            }

            // Get actual buffer indices
            let prev_buf_idx = self.buffer_index(prev_idx);
            let curr_buf_idx = self.buffer_index(i);
            let next_buf_idx = self.buffer_index(next_idx);

            let prev = &self.buffer[prev_buf_idx];
            let curr = &self.buffer[curr_buf_idx];
            let next = &self.buffer[next_buf_idx];

            // Calculate perpendicular distance from curr to line prev-next
            let dist = Self::perpendicular_distance(prev, curr, next);

            if dist < SRTL_ACCURACY_M {
                keep_flags[i] = false;
                kept_count -= 1;
            }
        }

        // Compact the buffer if points were removed
        if kept_count < self.count {
            self.compact_buffer(&keep_flags, kept_count);
        }
    }

    /// Calculate perpendicular distance from point to line
    fn perpendicular_distance(start: &PathPoint, point: &PathPoint, end: &PathPoint) -> f32 {
        // Simple approximation using cross product
        // For short distances, treat as Cartesian coordinates

        // Convert to approximate meters (rough approximation at mid-latitudes)
        const DEG_TO_M: f32 = 111_320.0;
        let cos_lat = libm::cosf(point.latitude * core::f32::consts::PI / 180.0);

        let x1 = (start.longitude - point.longitude) * DEG_TO_M * cos_lat;
        let y1 = (start.latitude - point.latitude) * DEG_TO_M;
        let x2 = (end.longitude - point.longitude) * DEG_TO_M * cos_lat;
        let y2 = (end.latitude - point.latitude) * DEG_TO_M;

        // Line direction
        let dx = x2 - x1;
        let dy = y2 - y1;
        let line_len_sq = dx * dx + dy * dy;

        if line_len_sq < 0.001 {
            // Start and end are same point, return distance to point
            return libm::sqrtf(x1 * x1 + y1 * y1);
        }

        // Calculate perpendicular distance using cross product
        let cross = libm::fabsf(x1 * dy - y1 * dx);
        cross / libm::sqrtf(line_len_sq)
    }

    /// Get buffer index for logical index
    fn buffer_index(&self, logical_idx: usize) -> usize {
        if self.count < SRTL_POINTS_MAX {
            // Buffer hasn't wrapped yet
            logical_idx
        } else {
            // Buffer has wrapped
            (self.write_idx + logical_idx) % SRTL_POINTS_MAX
        }
    }

    /// Compact buffer after simplification
    fn compact_buffer(&mut self, keep_flags: &[bool; SRTL_POINTS_MAX], new_count: usize) {
        let mut new_buffer = [PathPoint::new(0.0, 0.0, 0); SRTL_POINTS_MAX];
        let mut write_pos = 0;

        for (i, &keep) in keep_flags.iter().enumerate().take(self.count) {
            if keep {
                let buf_idx = self.buffer_index(i);
                new_buffer[write_pos] = self.buffer[buf_idx];
                write_pos += 1;
            }
        }

        self.buffer = new_buffer;
        self.count = new_count;
        self.write_idx = new_count;

        crate::log_debug!("PathRecorder: simplified to {} points", new_count);
    }
}

impl Default for PathRecorder {
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator over return path (reverse order)
pub struct ReturnPathIter<'a> {
    recorder: &'a PathRecorder,
    /// Current index (counting down from count-1 to 0)
    current_idx: usize,
    /// Whether iteration has started
    started: bool,
}

impl<'a> ReturnPathIter<'a> {
    fn new(recorder: &'a PathRecorder) -> Self {
        Self {
            recorder,
            current_idx: recorder.count.saturating_sub(1),
            started: false,
        }
    }
}

impl<'a> Iterator for ReturnPathIter<'a> {
    type Item = PathPoint;

    fn next(&mut self) -> Option<Self::Item> {
        if self.recorder.count == 0 {
            return None;
        }

        if !self.started {
            self.started = true;
            let buf_idx = self.recorder.buffer_index(self.current_idx);
            return Some(self.recorder.buffer[buf_idx]);
        }

        if self.current_idx == 0 {
            return None;
        }

        self.current_idx -= 1;
        let buf_idx = self.recorder.buffer_index(self.current_idx);
        Some(self.recorder.buffer[buf_idx])
    }
}

use crate::core::traits::EmbassyState;

/// Global path recorder (protected by EmbassyState)
///
/// Access from arming handler to start/stop recording.
/// Access from SmartRTL mode to retrieve return path.
pub static PATH_RECORDER: EmbassyState<PathRecorder> = EmbassyState::new(PathRecorder::new());

#[cfg(test)]
mod tests {
    use super::*;

    // ========== PathPoint Tests ==========

    #[test]
    fn test_path_point_new() {
        let point = PathPoint::new(35.6762, 139.6503, 1000);
        assert!((point.latitude - 35.6762).abs() < 0.0001);
        assert!((point.longitude - 139.6503).abs() < 0.0001);
        assert_eq!(point.timestamp_ms, 1000);
    }

    #[test]
    fn test_path_point_default() {
        let point = PathPoint::default();
        assert_eq!(point.latitude, 0.0);
        assert_eq!(point.longitude, 0.0);
        assert_eq!(point.timestamp_ms, 0);
    }

    // ========== PathRecorder Basic Tests ==========

    #[test]
    fn test_path_recorder_new() {
        let recorder = PathRecorder::new();
        assert_eq!(recorder.count(), 0);
        assert!(!recorder.has_path());
        assert!(!recorder.is_recording());
    }

    #[test]
    fn test_path_recorder_start_stop() {
        let mut recorder = PathRecorder::new();

        recorder.start();
        assert!(recorder.is_recording());

        recorder.stop();
        assert!(!recorder.is_recording());
    }

    #[test]
    fn test_path_recorder_clear() {
        let mut recorder = PathRecorder::new();
        recorder.start();
        recorder.record(0.0, 0.0, 0);
        recorder.record(1.0, 0.0, 4000);
        assert_eq!(recorder.count(), 2);

        recorder.clear();
        assert_eq!(recorder.count(), 0);
        assert!(!recorder.has_path());
    }

    // ========== Recording Tests ==========

    #[test]
    fn test_record_not_recording() {
        let mut recorder = PathRecorder::new();
        assert!(!recorder.record(0.0, 0.0, 0));
        assert_eq!(recorder.count(), 0);
    }

    #[test]
    fn test_record_first_point() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        assert!(recorder.record(35.6762, 139.6503, 1000));
        assert_eq!(recorder.count(), 1);
        assert!(recorder.has_path());
    }

    #[test]
    fn test_record_distance_threshold() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // First point
        recorder.record(0.0, 0.0, 0);

        // Too close (< 5m)
        assert!(!recorder.record(0.00001, 0.0, 100)); // ~1m

        // Far enough (> 5m)
        assert!(recorder.record(0.0001, 0.0, 200)); // ~10m
        assert_eq!(recorder.count(), 2);
    }

    #[test]
    fn test_record_time_threshold() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // First point
        recorder.record(0.0, 0.0, 0);

        // Same position but within time threshold
        assert!(!recorder.record(0.0, 0.0, 1000)); // 1s

        // Same position but past time threshold
        assert!(recorder.record(0.0, 0.0, 4000)); // 4s > 3s
        assert_eq!(recorder.count(), 2);
    }

    #[test]
    fn test_record_multiple_points() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // Record several points with sufficient distance
        for i in 0..10 {
            let lat = i as f32 * 0.0001; // ~10m apart
            recorder.record(lat, 0.0, i * 1000);
        }

        assert_eq!(recorder.count(), 10);
    }

    // ========== Ring Buffer Tests ==========

    #[test]
    fn test_ring_buffer_overflow() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // Fill buffer beyond capacity
        for i in 0..350 {
            let lat = i as f32 * 0.001; // Ensure distance threshold met
            recorder.record(lat, 0.0, i * 4000);
        }

        // Count should be capped at SRTL_POINTS_MAX
        assert_eq!(recorder.count(), SRTL_POINTS_MAX);
    }

    // ========== Return Path Iterator Tests ==========

    #[test]
    fn test_iter_return_path_empty() {
        let recorder = PathRecorder::new();
        let mut iter = recorder.iter_return_path();
        assert!(iter.next().is_none());
    }

    #[test]
    fn test_iter_return_path_single() {
        let mut recorder = PathRecorder::new();
        recorder.start();
        recorder.record(35.0, 139.0, 0);

        let points: Vec<_> = recorder.iter_return_path().collect();
        assert_eq!(points.len(), 1);
        assert!((points[0].latitude - 35.0).abs() < 0.001);
    }

    #[test]
    fn test_iter_return_path_reverse_order() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // Record points with sufficient distance
        recorder.record(0.0, 0.0, 0);
        recorder.record(0.001, 0.0, 1000); // ~100m
        recorder.record(0.002, 0.0, 2000); // ~200m

        let points: Vec<_> = recorder.iter_return_path().collect();

        // Should be in reverse order (most recent first)
        assert_eq!(points.len(), 3);
        assert!((points[0].latitude - 0.002).abs() < 0.0001);
        assert!((points[1].latitude - 0.001).abs() < 0.0001);
        assert!((points[2].latitude - 0.0).abs() < 0.0001);
    }

    // ========== Path Simplification Tests ==========

    #[test]
    fn test_simplify_too_few_points() {
        let mut recorder = PathRecorder::new();
        recorder.start();
        recorder.record(0.0, 0.0, 0);
        recorder.record(0.001, 0.0, 1000);

        recorder.simplify();
        assert_eq!(recorder.count(), 2); // No change
    }

    #[test]
    fn test_simplify_collinear_points() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // Record 5 collinear points on a straight line
        for i in 0..5 {
            let lat = i as f32 * 0.001; // Straight line north
            recorder.record(lat, 0.0, i * 4000);
        }
        assert_eq!(recorder.count(), 5);

        recorder.simplify();

        // Middle points should be removed (within accuracy of straight line)
        // Should keep first and last at minimum
        assert!(recorder.count() < 5);
        assert!(recorder.count() >= 2);
    }

    #[test]
    fn test_simplify_preserves_turns() {
        let mut recorder = PathRecorder::new();
        recorder.start();

        // Record an L-shaped path
        // First segment: go north
        recorder.record(0.0, 0.0, 0);
        recorder.record(0.001, 0.0, 1000);

        // Turn point (should be preserved)
        recorder.record(0.002, 0.0, 2000);

        // Second segment: go east
        recorder.record(0.002, 0.001, 3000);
        recorder.record(0.002, 0.002, 4000);

        let original_count = recorder.count();
        recorder.simplify();

        // Turn point should be preserved
        assert!(recorder.count() >= 3); // At least start, turn, end
        assert!(recorder.count() <= original_count);
    }

    // ========== Perpendicular Distance Tests ==========

    #[test]
    fn test_perpendicular_distance_on_line() {
        let start = PathPoint::new(0.0, 0.0, 0);
        let mid = PathPoint::new(0.0005, 0.0, 500);
        let end = PathPoint::new(0.001, 0.0, 1000);

        // Point on the line should have ~0 perpendicular distance
        let dist = PathRecorder::perpendicular_distance(&start, &mid, &end);
        assert!(dist < 1.0, "Expected ~0m, got {}m", dist);
    }

    #[test]
    fn test_perpendicular_distance_off_line() {
        let start = PathPoint::new(0.0, 0.0, 0);
        let mid = PathPoint::new(0.0005, 0.0001, 500); // Off to the east
        let end = PathPoint::new(0.001, 0.0, 1000);

        // Point off the line should have non-zero distance
        let dist = PathRecorder::perpendicular_distance(&start, &mid, &end);
        assert!(dist > 5.0, "Expected > 5m, got {}m", dist);
    }

    // ========== Memory Size Test ==========

    #[test]
    fn test_memory_size() {
        // Verify PathPoint size is reasonable
        let point_size = core::mem::size_of::<PathPoint>();
        assert!(
            point_size <= 16,
            "PathPoint too large: {} bytes",
            point_size
        );

        // Verify total buffer size is within budget (< 10 KB)
        let buffer_size = core::mem::size_of::<[PathPoint; SRTL_POINTS_MAX]>();
        assert!(
            buffer_size < 10_000,
            "Buffer exceeds 10KB budget: {} bytes",
            buffer_size
        );
    }
}
