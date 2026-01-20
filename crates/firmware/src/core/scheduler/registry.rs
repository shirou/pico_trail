//! Task registry for compile-time task metadata
//!
//! The registry holds static metadata for all tasks in the system.
//! Tasks are registered at compile-time using the `register_task!` macro
//! to avoid heap allocation and dynamic registration overhead.

use super::types::TaskMetadata;

/// Maximum number of tasks that can be registered
///
/// This limit is set conservatively to avoid excessive static memory usage.
/// Current allocation: 32 tasks × ~32 bytes = ~1KB
pub const MAX_TASKS: usize = 32;

/// Global task registry (compile-time initialization)
///
/// This static array holds metadata for all registered tasks.
/// It is populated at compile-time using the `register_task!` macro.
///
/// # Safety
///
/// This is a mutable static, but it is only written during initialization
/// (before any tasks start executing). Once initialized, it is read-only.
/// Access is synchronized via `TASK_COUNT` which is atomically incremented
/// during registration.
static mut TASK_REGISTRY: [Option<TaskMetadata>; MAX_TASKS] = [None; MAX_TASKS];

/// Number of registered tasks
///
/// This counter is incremented atomically during task registration.
/// Once all tasks are registered (before scheduler starts), it becomes read-only.
static mut TASK_COUNT: usize = 0;

/// Register a task in the global registry
///
/// This function is called at initialization time (before the scheduler starts)
/// to populate the task registry. It is not thread-safe and should only be
/// called during single-threaded initialization.
///
/// # Arguments
///
/// * `metadata` - Task metadata to register
///
/// # Panics
///
/// Panics if the registry is full (more than MAX_TASKS registered).
pub fn register_task(metadata: TaskMetadata) -> usize {
    unsafe {
        if TASK_COUNT >= MAX_TASKS {
            panic!(
                "Task registry full: cannot register more than {} tasks",
                MAX_TASKS
            );
        }

        let index = TASK_COUNT;
        TASK_REGISTRY[index] = Some(metadata);
        TASK_COUNT += 1;
        index
    }
}

/// Get task metadata by index
///
/// # Arguments
///
/// * `index` - Task index returned by `register_task`
///
/// # Returns
///
/// `Some(TaskMetadata)` if index is valid, `None` otherwise
pub fn get_task(index: usize) -> Option<TaskMetadata> {
    unsafe {
        if index < TASK_COUNT {
            TASK_REGISTRY[index]
        } else {
            None
        }
    }
}

/// Get task metadata by name
///
/// Performs a linear search through the registry. This is not optimized
/// for hot paths - use task index for runtime lookups.
///
/// # Arguments
///
/// * `name` - Task name to search for
///
/// # Returns
///
/// `Some((index, TaskMetadata))` if found, `None` otherwise
pub fn find_task_by_name(name: &str) -> Option<(usize, TaskMetadata)> {
    unsafe {
        let count = TASK_COUNT;
        #[allow(clippy::needless_range_loop)]
        for i in 0..count {
            if let Some(metadata) = TASK_REGISTRY[i] {
                if metadata.name == name {
                    return Some((i, metadata));
                }
            }
        }
        None
    }
}

/// Get total number of registered tasks
pub fn task_count() -> usize {
    unsafe { TASK_COUNT }
}

/// Iterate over all registered tasks
///
/// # Returns
///
/// Iterator over `(index, TaskMetadata)` pairs
pub fn iter_tasks() -> impl Iterator<Item = (usize, TaskMetadata)> {
    let count = task_count();
    (0..count).filter_map(|i| get_task(i).map(|meta| (i, meta)))
}

/// Reset the task registry (for testing only)
///
/// # Safety
///
/// This function should only be called in tests before any tasks are running.
#[cfg(test)]
pub fn reset_registry() {
    unsafe {
        TASK_REGISTRY = [None; MAX_TASKS];
        TASK_COUNT = 0;
    }
}

/// Macro to register a task with compile-time metadata
///
/// # Example
///
/// ```rust,ignore
/// use pico_trail::core::scheduler::{register_task_metadata, TaskMetadata};
///
/// const IMU_TASK: TaskMetadata = TaskMetadata {
///     name: "imu_task",
///     rate_hz: 400,
///     priority: 10,
///     budget_us: 2000,
/// };
///
/// let task_id = register_task_metadata!(IMU_TASK);
/// ```
#[macro_export]
macro_rules! register_task_metadata {
    ($metadata:expr) => {{
        $crate::core::scheduler::register_task($metadata)
    }};
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_register_and_get_task() {
        reset_registry();

        let metadata = TaskMetadata {
            name: "test_task",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };

        let index = register_task(metadata);
        assert_eq!(index, 0);

        let retrieved = get_task(index);
        assert!(retrieved.is_some());

        let retrieved_meta = retrieved.unwrap();
        assert_eq!(retrieved_meta.name, "test_task");
        assert_eq!(retrieved_meta.rate_hz, 100);
        assert_eq!(retrieved_meta.priority, 5);
        assert_eq!(retrieved_meta.budget_us, 5000);
    }

    #[test]
    fn test_multiple_registrations() {
        reset_registry();

        let task1 = TaskMetadata {
            name: "task1",
            rate_hz: 400,
            priority: 10,
            budget_us: 2000,
        };

        let task2 = TaskMetadata {
            name: "task2",
            rate_hz: 50,
            priority: 5,
            budget_us: 15000,
        };

        let idx1 = register_task(task1);
        let idx2 = register_task(task2);

        assert_eq!(idx1, 0);
        assert_eq!(idx2, 1);
        assert_eq!(task_count(), 2);

        assert_eq!(get_task(idx1).unwrap().name, "task1");
        assert_eq!(get_task(idx2).unwrap().name, "task2");
    }

    #[test]
    fn test_find_task_by_name() {
        reset_registry();

        let task1 = TaskMetadata {
            name: "imu_task",
            rate_hz: 400,
            priority: 10,
            budget_us: 2000,
        };

        let task2 = TaskMetadata {
            name: "control_task",
            rate_hz: 50,
            priority: 5,
            budget_us: 15000,
        };

        register_task(task1);
        register_task(task2);

        let (index, metadata) = find_task_by_name("control_task").expect("Task not found");
        assert_eq!(index, 1);
        assert_eq!(metadata.name, "control_task");
        assert_eq!(metadata.rate_hz, 50);

        assert!(find_task_by_name("nonexistent").is_none());
    }

    #[test]
    fn test_iter_tasks() {
        reset_registry();

        let task1 = TaskMetadata {
            name: "task1",
            rate_hz: 400,
            priority: 10,
            budget_us: 2000,
        };

        let task2 = TaskMetadata {
            name: "task2",
            rate_hz: 50,
            priority: 5,
            budget_us: 15000,
        };

        register_task(task1);
        register_task(task2);

        let tasks: Vec<_> = iter_tasks().collect();
        assert_eq!(tasks.len(), 2);
        assert_eq!(tasks[0].1.name, "task1");
        assert_eq!(tasks[1].1.name, "task2");
    }

    #[test]
    fn test_get_invalid_index() {
        reset_registry();

        let task = TaskMetadata {
            name: "task1",
            rate_hz: 100,
            priority: 5,
            budget_us: 5000,
        };

        register_task(task);

        assert!(get_task(0).is_some());
        assert!(get_task(1).is_none());
        assert!(get_task(999).is_none());
    }

    #[test]
    #[should_panic(expected = "Task registry full")]
    fn test_registry_overflow() {
        reset_registry();

        // Try to register more tasks than MAX_TASKS
        for _i in 0..=MAX_TASKS {
            let task = TaskMetadata {
                name: "overflow_task",
                rate_hz: 100,
                priority: 5,
                budget_us: 5000,
            };
            register_task(task);
        }
    }
}
