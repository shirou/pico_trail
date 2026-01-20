//! Monitoring task for scheduler health and performance
//!
//! This module provides a monitoring task that periodically collects
//! statistics from all tasks and reports system health metrics including:
//! - CPU load
//! - Deadline misses
//! - Task execution times
//! - Jitter measurements

use super::registry::{iter_tasks, task_count};
use super::stats::{
    get_scheduler_stats, get_task_stats, update_cpu_load, update_total_deadline_misses,
    update_uptime,
};

/// CPU load warning threshold (percentage)
const CPU_LOAD_WARNING_THRESHOLD: u8 = 75;

/// High jitter warning threshold (microseconds)
const HIGH_JITTER_WARNING_US: u32 = 1000;

/// Monitoring task statistics
///
/// Collects and reports statistics for all registered tasks.
/// This should be called periodically (e.g., every 1 second) by the monitoring task.
pub fn collect_and_report_stats(uptime_ms: u64) {
    // Update global statistics
    update_uptime(uptime_ms);
    update_total_deadline_misses();

    // Calculate CPU load over 1 second window
    let cpu_load = update_cpu_load(1_000_000); // 1 second in microseconds

    // Get global stats
    let scheduler_stats = get_scheduler_stats();

    // Report summary
    log_scheduler_summary(&scheduler_stats, cpu_load);

    // Check for warnings
    check_warnings(cpu_load);

    // Report per-task statistics
    report_task_stats();
}

/// Log scheduler summary
#[allow(unused_variables)]
fn log_scheduler_summary(scheduler_stats: &super::types::SchedulerStats, cpu_load: u8) {
    crate::log_info!(
        "Scheduler: uptime={}ms cpu={}% deadline_misses={}",
        scheduler_stats.uptime_ms,
        cpu_load,
        scheduler_stats.total_deadline_misses
    );
}

/// Check for warning conditions
#[allow(unused_variables)]
fn check_warnings(cpu_load: u8) {
    // CPU load warning
    if cpu_load >= CPU_LOAD_WARNING_THRESHOLD {
        crate::log_warn!("High CPU load: {}%", cpu_load);
    }

    // Check per-task warnings
    for (task_id, metadata) in iter_tasks() {
        let stats = get_task_stats(task_id);

        // Deadline miss warning
        if stats.deadline_misses > 0 {
            crate::log_warn!(
                "Task '{}': {} deadline misses",
                metadata.name,
                stats.deadline_misses
            );
        }

        // High jitter warning
        if stats.avg_jitter_us > HIGH_JITTER_WARNING_US {
            crate::log_warn!(
                "Task '{}': high jitter {}us (target period: {}us)",
                metadata.name,
                stats.avg_jitter_us,
                metadata.period_us()
            );
        }
    }
}

/// Report per-task statistics
#[allow(unused_variables)]
fn report_task_stats() {
    let count = task_count();
    if count == 0 {
        return;
    }

    crate::log_info!("Task statistics ({} tasks):", count);

    for (task_id, metadata) in iter_tasks() {
        let stats = get_task_stats(task_id);

        crate::log_info!(
            "  {}: exec={}us (avg={}us, max={}us) jitter={}us misses={} count={}",
            metadata.name,
            stats.last_execution_us,
            stats.avg_execution_us,
            stats.max_execution_us,
            stats.avg_jitter_us,
            stats.deadline_misses,
            stats.execution_count
        );
    }
}

// Re-export monitor_task from platform module
pub use crate::platform::rp2350::tasks::monitor::monitor_task;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::scheduler::{
        register_task, reset_stats, stats::update_task_stats, types::TaskMetadata,
    };

    #[test]
    #[serial_test::serial]
    fn test_collect_and_report_stats() {
        reset_stats();

        // Register test tasks
        let task1 = TaskMetadata {
            name: "task1",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };
        let task2 = TaskMetadata {
            name: "task2",
            rate_hz: 50,
            priority: 3,
            budget_us: 10000,
        };

        let id1 = register_task(task1);
        let id2 = register_task(task2);

        // Simulate some task executions
        update_task_stats(id1, 1500, 10000, 10000, 5000);
        update_task_stats(id2, 8000, 20000, 20000, 10000);

        // Collect and report (should not panic)
        collect_and_report_stats(1000);

        // Verify global stats were updated
        let scheduler_stats = get_scheduler_stats();
        assert_eq!(scheduler_stats.uptime_ms, 1000);
        assert_eq!(scheduler_stats.total_deadline_misses, 0);
    }

    #[test]
    #[serial_test::serial]
    fn test_check_warnings_high_cpu() {
        reset_stats();

        // Simulate high CPU load scenario
        let task = TaskMetadata {
            name: "busy_task",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };

        let id = register_task(task);

        // Simulate task execution that would cause high CPU
        for _ in 0..10 {
            update_task_stats(id, 4500, 10000, 10000, 5000);
        }

        // Should trigger warning (output to console in test)
        check_warnings(80);
    }

    #[test]
    #[serial_test::serial]
    fn test_report_task_stats_empty() {
        reset_stats();

        // Should handle empty task list gracefully
        report_task_stats();
    }
}
