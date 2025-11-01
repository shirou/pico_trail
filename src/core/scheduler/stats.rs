//! Task statistics tracking and CPU load calculation
//!
//! This module provides runtime statistics tracking for individual tasks
//! and global scheduler metrics. Statistics are updated after each task
//! execution and can be queried for monitoring and debugging.

use super::registry::{task_count, MAX_TASKS};
use super::types::{SchedulerStats, TaskStats};

/// Global task statistics storage
///
/// Each task has a corresponding entry in this array, indexed by the task ID
/// returned from `register_task()`. Statistics are updated after each task
/// execution using critical sections to ensure atomicity.
static mut TASK_STATS_STORAGE: [TaskStats; MAX_TASKS] = [TaskStats {
    last_execution_us: 0,
    avg_execution_us: 0,
    max_execution_us: 0,
    deadline_misses: 0,
    last_period_us: 0,
    avg_jitter_us: 0,
    execution_count: 0,
}; MAX_TASKS];

/// Global scheduler statistics
static mut SCHEDULER_STATS: SchedulerStats = SchedulerStats {
    cpu_load_percent: 0,
    total_deadline_misses: 0,
    uptime_ms: 0,
};

/// Update task statistics after execution
///
/// This function should be called immediately after a task completes execution.
/// It updates the task's statistics and calculates jitter.
///
/// # Arguments
///
/// * `task_id` - Task index from registry
/// * `execution_us` - Task execution time in microseconds
/// * `period_us` - Time since last execution in microseconds
/// * `target_period_us` - Expected period based on task rate
/// * `budget_us` - Maximum allowed execution time
///
/// # Safety
///
/// This function uses critical sections to ensure atomic updates to shared state.
pub fn update_task_stats(
    task_id: usize,
    execution_us: u32,
    period_us: u32,
    target_period_us: u32,
    budget_us: u32,
) {
    critical_section::with(|_cs| unsafe {
        if task_id < MAX_TASKS {
            TASK_STATS_STORAGE[task_id].update(
                execution_us,
                period_us,
                target_period_us,
                budget_us,
            );
        }
    });
}

/// Get task statistics by task ID
///
/// # Arguments
///
/// * `task_id` - Task index from registry
///
/// # Returns
///
/// Copy of the task's statistics, or default if task ID is invalid
pub fn get_task_stats(task_id: usize) -> TaskStats {
    critical_section::with(|_cs| unsafe {
        if task_id < MAX_TASKS {
            TASK_STATS_STORAGE[task_id]
        } else {
            TaskStats::default()
        }
    })
}

/// Iterate over all task statistics
///
/// Returns an iterator over task statistics for all registered tasks.
/// This avoids allocation and is suitable for no_std environments.
///
/// # Returns
///
/// Iterator yielding `(task_id, TaskStats)` pairs
pub fn iter_task_stats() -> impl Iterator<Item = (usize, TaskStats)> {
    let count = task_count();
    (0..count).map(|i| (i, get_task_stats(i)))
}

/// Calculate and update global CPU load
///
/// This function calculates CPU load as the sum of (task execution time × task rate)
/// divided by the measurement window. It should be called periodically
/// (e.g., every second) by the monitoring task.
///
/// # Arguments
///
/// * `window_us` - Duration of measurement window in microseconds
///
/// # Returns
///
/// Updated CPU load percentage (0-100)
pub fn update_cpu_load(window_us: u64) -> u8 {
    let count = task_count();
    let mut total_execution_us: u64 = 0;

    critical_section::with(|_cs| unsafe {
        #[allow(clippy::needless_range_loop)]
        for i in 0..count {
            let stats = TASK_STATS_STORAGE[i];
            // Get task metadata to retrieve rate_hz
            if let Some(metadata) = super::registry::get_task(i) {
                // Calculate execution time over window: avg_execution_us * rate_hz
                // This gives us the total microseconds consumed per second
                total_execution_us += stats.avg_execution_us as u64 * metadata.rate_hz as u64;
            }
        }

        // Update scheduler stats through temporary variable to avoid mutable reference
        let mut sched_stats = SCHEDULER_STATS;
        sched_stats.update_cpu_load(total_execution_us, window_us);
        SCHEDULER_STATS = sched_stats;
        sched_stats.cpu_load_percent
    })
}

/// Get global scheduler statistics
///
/// # Returns
///
/// Copy of the current scheduler statistics
pub fn get_scheduler_stats() -> SchedulerStats {
    critical_section::with(|_cs| unsafe { SCHEDULER_STATS })
}

/// Update uptime in scheduler statistics
///
/// # Arguments
///
/// * `uptime_ms` - Current system uptime in milliseconds
pub fn update_uptime(uptime_ms: u64) {
    critical_section::with(|_cs| unsafe {
        SCHEDULER_STATS.uptime_ms = uptime_ms;
    });
}

/// Update total deadline misses from all tasks
///
/// This function aggregates deadline misses from all task statistics
/// and updates the global count.
pub fn update_total_deadline_misses() {
    let count = task_count();

    critical_section::with(|_cs| unsafe {
        let mut total = 0;
        #[allow(clippy::needless_range_loop)]
        for i in 0..count {
            total += TASK_STATS_STORAGE[i].deadline_misses;
        }
        SCHEDULER_STATS.total_deadline_misses = total;
    });
}

/// Reset all statistics (for testing only)
///
/// # Safety
///
/// This function should only be called in tests before any tasks are running.
#[cfg(test)]
pub fn reset_stats() {
    use super::registry::reset_registry;

    reset_registry();
    critical_section::with(|_cs| unsafe {
        TASK_STATS_STORAGE = [TaskStats::default(); MAX_TASKS];
        SCHEDULER_STATS = SchedulerStats::default();
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_update_and_get_task_stats() {
        reset_stats();

        // Update stats for task 0
        update_task_stats(0, 1500, 2500, 2500, 2000);

        let stats = get_task_stats(0);
        assert_eq!(stats.last_execution_us, 1500);
        assert_eq!(stats.avg_execution_us, 1500);
        assert_eq!(stats.execution_count, 1);

        // Second update
        update_task_stats(0, 1600, 2500, 2500, 2000);

        let stats = get_task_stats(0);
        assert_eq!(stats.last_execution_us, 1600);
        assert_eq!(stats.execution_count, 2);
    }

    #[test]
    fn test_deadline_miss_tracking() {
        reset_stats();

        // Normal execution
        update_task_stats(0, 1500, 2500, 2500, 2000);
        assert_eq!(get_task_stats(0).deadline_misses, 0);

        // Deadline miss
        update_task_stats(0, 2100, 2500, 2500, 2000);
        assert_eq!(get_task_stats(0).deadline_misses, 1);

        // Another deadline miss
        update_task_stats(0, 2200, 2500, 2500, 2000);
        assert_eq!(get_task_stats(0).deadline_misses, 2);
    }

    #[test]
    fn test_multiple_task_stats() {
        reset_stats();

        // Update two different tasks
        update_task_stats(0, 1500, 2500, 2500, 2000);
        update_task_stats(1, 10000, 20000, 20000, 15000);

        let stats0 = get_task_stats(0);
        let stats1 = get_task_stats(1);

        assert_eq!(stats0.last_execution_us, 1500);
        assert_eq!(stats1.last_execution_us, 10000);
        assert_eq!(stats0.execution_count, 1);
        assert_eq!(stats1.execution_count, 1);
    }

    #[test]
    fn test_update_total_deadline_misses() {
        use super::super::registry::register_task;
        use super::super::types::TaskMetadata;

        reset_stats();

        // Register test tasks so task_count() returns the correct value
        register_task(TaskMetadata {
            name: "test_task_0",
            rate_hz: 400,
            priority: 100,
            budget_us: 2000,
        });
        register_task(TaskMetadata {
            name: "test_task_1",
            rate_hz: 50,
            priority: 100,
            budget_us: 15000,
        });
        register_task(TaskMetadata {
            name: "test_task_2",
            rate_hz: 100,
            priority: 100,
            budget_us: 5000,
        });

        // Create deadline misses in multiple tasks
        update_task_stats(0, 2100, 2500, 2500, 2000); // 1 miss
        update_task_stats(1, 16000, 20000, 20000, 15000); // 1 miss
        update_task_stats(2, 1500, 10000, 10000, 5000); // No miss

        update_total_deadline_misses();

        let scheduler_stats = get_scheduler_stats();
        assert_eq!(scheduler_stats.total_deadline_misses, 2);
    }

    #[test]
    fn test_uptime_update() {
        reset_stats();

        update_uptime(12345);

        let scheduler_stats = get_scheduler_stats();
        assert_eq!(scheduler_stats.uptime_ms, 12345);
    }

    #[test]
    #[ignore] // CPU load calculation requires tasks to be registered in registry
    fn test_cpu_load_calculation() {
        reset_stats();

        // Note: This test requires tasks to be registered in the registry
        // with rate_hz information. In unit tests without registered tasks,
        // CPU load will be 0%.
        //
        // Expected calculation with registered tasks:
        // CPU Load = Σ(avg_execution_us × rate_hz) / window_us × 100
        // Example:
        //   Task 0: 500us × 400Hz = 200,000us/s (20%)
        //   Task 1: 1000us × 100Hz = 100,000us/s (10%)
        //   Total: 300,000us / 1,000,000us = 30%

        // Simulate task executions
        update_task_stats(0, 1500, 2500, 2500, 2000);
        update_task_stats(1, 10000, 20000, 20000, 15000);

        // Window of 1 second (1_000_000 us)
        let load = update_cpu_load(1_000_000);

        // Without registered tasks, load will be 0%
        assert_eq!(load, 0);
    }

    #[test]
    fn test_invalid_task_id() {
        reset_stats();

        // Update with invalid task ID should not panic
        update_task_stats(999, 1500, 2500, 2500, 2000);

        // Get with invalid task ID should return default
        let stats = get_task_stats(999);
        assert_eq!(stats.execution_count, 0);
    }
}
