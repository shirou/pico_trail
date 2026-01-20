//! Core types for task scheduler
//!
//! This module defines the fundamental types used by the task scheduler:
//! - Task metadata (compile-time configuration)
//! - Task statistics (runtime monitoring)
//! - Scheduler statistics (global metrics)

/// Task metadata registered at compile-time
///
/// Each task in the scheduler has associated metadata that defines its
/// execution characteristics and resource requirements.
#[derive(Debug, Clone, Copy)]
pub struct TaskMetadata {
    /// Human-readable task name for logging and debugging
    pub name: &'static str,

    /// Target execution rate in Hz (1-400)
    pub rate_hz: u32,

    /// Priority hint (0-255, higher = more important)
    ///
    /// Note: Embassy executor does not enforce task priorities directly.
    /// This field is used for:
    /// - Interrupt priority configuration (higher priority tasks use higher interrupt priorities)
    /// - Resource allocation decisions
    /// - Monitoring and reporting
    pub priority: u8,

    /// Execution time budget in microseconds
    ///
    /// If a task execution exceeds this budget, a deadline miss warning is logged.
    /// This should be set to less than the task period to allow overhead.
    ///
    /// Example: For a 400Hz task (2500us period), budget might be 2000us to leave
    /// 500us margin for other tasks and scheduler overhead.
    pub budget_us: u32,
}

impl TaskMetadata {
    /// Calculate the task period in microseconds from the rate
    #[inline]
    pub const fn period_us(&self) -> u32 {
        1_000_000 / self.rate_hz
    }

    /// Check if execution time is within budget
    #[inline]
    pub const fn is_within_budget(&self, execution_us: u32) -> bool {
        execution_us <= self.budget_us
    }

    /// Check if period deviation exceeds tolerance (5%)
    #[inline]
    pub fn is_period_acceptable(&self, actual_period_us: u32) -> bool {
        let target = self.period_us();
        let tolerance = target / 20; // 5% tolerance
        let lower = target.saturating_sub(tolerance);
        let upper = target.saturating_add(tolerance);
        actual_period_us >= lower && actual_period_us <= upper
    }
}

/// Runtime statistics for a single task
///
/// These statistics are updated after each task execution and can be
/// queried for monitoring and debugging purposes.
#[derive(Debug, Clone, Copy, Default)]
pub struct TaskStats {
    /// Last execution time in microseconds
    pub last_execution_us: u32,

    /// Average execution time in microseconds (exponential moving average)
    ///
    /// Uses EMA with alpha = 0.1 to smooth out variations while remaining responsive
    /// to changes in execution time.
    pub avg_execution_us: u32,

    /// Maximum execution time observed in microseconds
    pub max_execution_us: u32,

    /// Number of deadline misses (execution time > budget)
    pub deadline_misses: u32,

    /// Last measured period in microseconds (time between executions)
    pub last_period_us: u32,

    /// Average jitter in microseconds (deviation from target period)
    ///
    /// Uses EMA to track typical period deviation. Lower jitter indicates
    /// more deterministic timing.
    pub avg_jitter_us: u32,

    /// Total number of executions
    pub execution_count: u64,
}

impl TaskStats {
    /// Update statistics with a new execution measurement
    ///
    /// # Arguments
    ///
    /// * `execution_us` - Duration of the task execution in microseconds
    /// * `period_us` - Time since last execution in microseconds
    /// * `target_period_us` - Expected period based on task rate
    /// * `budget_us` - Maximum allowed execution time
    pub fn update(
        &mut self,
        execution_us: u32,
        period_us: u32,
        target_period_us: u32,
        budget_us: u32,
    ) {
        self.last_execution_us = execution_us;
        self.last_period_us = period_us;
        self.execution_count = self.execution_count.saturating_add(1);

        // Update average execution time using exponential moving average (alpha = 0.1)
        // EMA formula: avg_new = alpha * value + (1 - alpha) * avg_old
        // Using fixed-point arithmetic: avg_new = (value + 9 * avg_old) / 10
        if self.avg_execution_us == 0 {
            self.avg_execution_us = execution_us;
        } else {
            self.avg_execution_us = (execution_us + 9 * self.avg_execution_us) / 10;
        }

        // Update maximum execution time
        if execution_us > self.max_execution_us {
            self.max_execution_us = execution_us;
        }

        // Detect deadline miss
        if execution_us > budget_us {
            self.deadline_misses = self.deadline_misses.saturating_add(1);
        }

        // Calculate and update jitter (absolute deviation from target period)
        let jitter = period_us.abs_diff(target_period_us);

        if self.avg_jitter_us == 0 {
            self.avg_jitter_us = jitter;
        } else {
            self.avg_jitter_us = (jitter + 9 * self.avg_jitter_us) / 10;
        }
    }

    /// Reset all statistics to initial state
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Global scheduler statistics
///
/// Tracks overall system health and resource utilization.
#[derive(Debug, Clone, Copy, Default)]
pub struct SchedulerStats {
    /// CPU load as percentage (0-100)
    ///
    /// Calculated as: sum(task_execution_time * task_rate) / measurement_window
    /// A value of 100% means the CPU is fully utilized executing tasks.
    pub cpu_load_percent: u8,

    /// Total deadline misses across all tasks
    pub total_deadline_misses: u32,

    /// System uptime in milliseconds
    pub uptime_ms: u64,
}

impl SchedulerStats {
    /// Update CPU load percentage
    ///
    /// # Arguments
    ///
    /// * `total_execution_us` - Sum of all task execution times in measurement window
    /// * `window_us` - Duration of measurement window in microseconds
    pub fn update_cpu_load(&mut self, total_execution_us: u64, window_us: u64) {
        if window_us > 0 {
            let load = (total_execution_us * 100) / window_us;
            self.cpu_load_percent = load.min(100) as u8;
        }
    }

    /// Add deadline misses from all tasks
    pub fn update_deadline_misses(&mut self, task_stats: &[TaskStats]) {
        self.total_deadline_misses = task_stats.iter().map(|s| s.deadline_misses).sum();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_task_metadata_period_calculation() {
        let task = TaskMetadata {
            name: "test_task",
            rate_hz: 400,
            priority: 10,
            budget_us: 2000,
        };

        assert_eq!(task.period_us(), 2500); // 1_000_000 / 400
    }

    #[test]
    fn test_task_metadata_budget_check() {
        let task = TaskMetadata {
            name: "test_task",
            rate_hz: 400,
            priority: 10,
            budget_us: 2000,
        };

        assert!(task.is_within_budget(1500));
        assert!(task.is_within_budget(2000));
        assert!(!task.is_within_budget(2001));
    }

    #[test]
    fn test_task_metadata_period_tolerance() {
        let task = TaskMetadata {
            name: "test_task",
            rate_hz: 400, // 2500us period
            priority: 10,
            budget_us: 2000,
        };

        // 5% tolerance = ±125us
        assert!(task.is_period_acceptable(2500)); // Exact
        assert!(task.is_period_acceptable(2375)); // -5%
        assert!(task.is_period_acceptable(2625)); // +5%
        assert!(!task.is_period_acceptable(2300)); // -8%
        assert!(!task.is_period_acceptable(2700)); // +8%
    }

    #[test]
    fn test_task_stats_update() {
        let mut stats = TaskStats::default();

        // First execution
        stats.update(1500, 2500, 2500, 2000);

        assert_eq!(stats.last_execution_us, 1500);
        assert_eq!(stats.avg_execution_us, 1500);
        assert_eq!(stats.max_execution_us, 1500);
        assert_eq!(stats.deadline_misses, 0);
        assert_eq!(stats.execution_count, 1);
        assert_eq!(stats.avg_jitter_us, 0); // No jitter on first execution

        // Second execution - normal
        stats.update(1600, 2500, 2500, 2000);

        assert_eq!(stats.last_execution_us, 1600);
        assert_eq!(stats.avg_execution_us, (1600 + 9 * 1500) / 10); // EMA
        assert_eq!(stats.max_execution_us, 1600);
        assert_eq!(stats.deadline_misses, 0);
        assert_eq!(stats.execution_count, 2);

        // Third execution - deadline miss
        stats.update(2100, 2500, 2500, 2000);

        assert_eq!(stats.last_execution_us, 2100);
        assert_eq!(stats.max_execution_us, 2100);
        assert_eq!(stats.deadline_misses, 1);
        assert_eq!(stats.execution_count, 3);
    }

    #[test]
    fn test_task_stats_jitter_calculation() {
        let mut stats = TaskStats::default();

        // First execution - establishes baseline
        stats.update(1500, 2500, 2500, 2000);
        assert_eq!(stats.avg_jitter_us, 0);

        // Second execution - 100us late
        stats.update(1500, 2600, 2500, 2000);
        assert_eq!(stats.avg_jitter_us, 100);

        // Third execution - 50us early
        stats.update(1500, 2450, 2500, 2000);
        let expected = (50 + 9 * 100) / 10;
        assert_eq!(stats.avg_jitter_us, expected);
    }

    #[test]
    fn test_scheduler_stats_cpu_load() {
        let mut stats = SchedulerStats::default();

        // 50% CPU load: 500us execution in 1000us window
        stats.update_cpu_load(500, 1000);
        assert_eq!(stats.cpu_load_percent, 50);

        // 100% CPU load
        stats.update_cpu_load(1000, 1000);
        assert_eq!(stats.cpu_load_percent, 100);

        // Over 100% is clamped
        stats.update_cpu_load(1200, 1000);
        assert_eq!(stats.cpu_load_percent, 100);

        // Low load
        stats.update_cpu_load(50, 1000);
        assert_eq!(stats.cpu_load_percent, 5);
    }

    #[test]
    fn test_scheduler_stats_deadline_misses() {
        let mut stats = SchedulerStats::default();

        let task_stats = [
            TaskStats {
                deadline_misses: 5,
                ..Default::default()
            },
            TaskStats {
                deadline_misses: 3,
                ..Default::default()
            },
            TaskStats {
                deadline_misses: 0,
                ..Default::default()
            },
        ];

        stats.update_deadline_misses(&task_stats);
        assert_eq!(stats.total_deadline_misses, 8);
    }
}
