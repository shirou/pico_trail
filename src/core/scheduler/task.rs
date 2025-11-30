//! Task execution helpers and timing measurement
//!
//! This module provides utilities for measuring task execution time and
//! updating statistics. It wraps user task logic with timing instrumentation.

use super::stats::update_task_stats;
use super::types::TaskMetadata;

/// Measure execution time of a task and update statistics
///
/// This function wraps a task execution, measures its duration, and updates
/// the task's statistics. It should be called for each task invocation.
///
/// # Arguments
///
/// * `task_id` - Task index from registry
/// * `metadata` - Task metadata (for period and budget)
/// * `last_execution_us` - Timestamp of last execution in microseconds
/// * `now_us` - Current timestamp in microseconds
/// * `f` - Closure containing the task logic to execute
///
/// # Returns
///
/// Result from the task closure
///
/// # Example
///
/// ```rust,ignore
/// use embassy_time::Instant;
///
/// let start = Instant::now();
/// let result = execute_with_timing(
///     task_id,
///     &metadata,
///     last_execution.as_micros(),
///     Instant::now().as_micros(),
///     || {
///         // Task logic here
///         read_imu_sensor()
///     }
/// );
/// ```
pub fn execute_with_timing<F, R>(
    task_id: usize,
    metadata: &TaskMetadata,
    last_execution_us: u64,
    now_us: u64,
    f: F,
) -> R
where
    F: FnOnce() -> R,
{
    let start_us = now_us;

    // Execute user task
    let result = f();

    let end_us = current_time_us();
    let execution_us = (end_us - start_us) as u32;
    let period_us = if last_execution_us > 0 {
        (now_us - last_execution_us) as u32
    } else {
        metadata.period_us()
    };

    // Update statistics
    update_task_stats(
        task_id,
        execution_us,
        period_us,
        metadata.period_us(),
        metadata.budget_us,
    );

    result
}

/// Get current time in microseconds
///
/// This is a placeholder that should be replaced with platform-specific
/// timer access in production. For now, it uses a simple counter for testing.
#[cfg(not(feature = "embassy"))]
fn current_time_us() -> u64 {
    // In tests/host environment, use std::time
    #[cfg(test)]
    {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_micros() as u64
    }
    #[cfg(not(test))]
    {
        // Placeholder for non-test, non-hardware builds
        0
    }
}

/// Get current time in microseconds (Embassy version)
///
/// Uses Embassy's time driver for hardware targets
#[cfg(feature = "embassy")]
fn current_time_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}

/// Helper macro to wrap task execution with timing
///
/// This macro simplifies task instrumentation by automatically handling
/// timing measurement and statistics updates.
///
/// # Example
///
/// ```rust,ignore
/// use pico_trail::execute_task;
///
/// #[embassy_executor::task]
/// async fn imu_task() {
///     let task_id = 0;
///     let metadata = get_task_metadata(task_id);
///     let mut last_execution = 0;
///
///     let mut ticker = Ticker::every(Duration::from_micros(metadata.period_us() as u64));
///     loop {
///         ticker.next().await;
///
///         execute_task!(task_id, metadata, last_execution, {
///             // Task logic here
///             sample_imu().await;
///         });
///     }
/// }
/// ```
#[macro_export]
macro_rules! execute_task {
    ($task_id:expr, $metadata:expr, $last_execution:expr, $body:block) => {{
        let now_us = $crate::core::scheduler::task::current_time_us();
        let result = $crate::core::scheduler::task::execute_with_timing(
            $task_id,
            $metadata,
            $last_execution,
            now_us,
            || $body,
        );
        $last_execution = now_us;
        result
    }};
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::scheduler::{
        register_task, registry::reset_registry, reset_stats, stats::get_task_stats,
        types::TaskMetadata,
    };

    #[test]
    #[serial_test::serial]
    fn test_execute_with_timing() {
        reset_registry();
        reset_stats();

        let metadata = TaskMetadata {
            name: "test_task",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };

        let task_id = register_task(metadata);

        // First execution
        let now = current_time_us();
        let result = execute_with_timing(task_id, &metadata, 0, now, || {
            // Simulate some work
            42
        });

        assert_eq!(result, 42);

        let stats = get_task_stats(task_id);
        assert_eq!(stats.execution_count, 1);
        // Execution time should be recorded
        assert!(stats.last_execution_us < 100000); // Reasonable upper bound
    }

    #[test]
    #[serial_test::serial]
    fn test_execute_with_timing_multiple() {
        reset_registry();
        reset_stats();

        let metadata = TaskMetadata {
            name: "test_task",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };

        let task_id = register_task(metadata);

        // Multiple executions
        let mut last_execution = 0;
        for _ in 0..5 {
            let now = current_time_us();
            execute_with_timing(task_id, &metadata, last_execution, now, || {});
            last_execution = now;
        }

        let stats = get_task_stats(task_id);
        assert_eq!(stats.execution_count, 5);
    }
}
