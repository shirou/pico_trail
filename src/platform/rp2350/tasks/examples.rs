//! Example Embassy tasks with timing instrumentation
//!
//! These tasks demonstrate the usage pattern for scheduler integration.
//! They use placeholder logic and will be replaced with real implementations
//! in future phases.

use crate::core::scheduler::{register_task, types::TaskMetadata};
use embassy_time::{Duration, Instant, Ticker};

// Task metadata constants
const IMU_TASK_META: TaskMetadata = TaskMetadata {
    name: "imu_task",
    rate_hz: 400,
    priority: 10,
    budget_us: 2000,
};

const AHRS_TASK_META: TaskMetadata = TaskMetadata {
    name: "ahrs_task",
    rate_hz: 100,
    priority: 8,
    budget_us: 8000,
};

const CONTROL_TASK_META: TaskMetadata = TaskMetadata {
    name: "control_task",
    rate_hz: 50,
    priority: 7,
    budget_us: 15000,
};

const TELEMETRY_TASK_META: TaskMetadata = TaskMetadata {
    name: "telemetry_task",
    rate_hz: 10,
    priority: 3,
    budget_us: 50000,
};

/// IMU sampling task (400Hz)
///
/// High-frequency task for reading IMU sensor data.
/// This is a placeholder - real implementation will read from actual IMU hardware.
#[embassy_executor::task]
pub async fn imu_task() {
    let task_id = register_task(IMU_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(IMU_TASK_META.period_us() as u64));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        // Execute task with timing
        crate::core::scheduler::task::execute_with_timing(
            task_id,
            &IMU_TASK_META,
            last_execution_us,
            now_us,
            || {
                // Simulate IMU sensor reading workload (~500us)
                let start = Instant::now();
                while start.elapsed().as_micros() < 500 {
                    // Busy wait to simulate work
                    core::hint::spin_loop();
                }
            },
        );

        last_execution_us = now_us;
    }
}

/// AHRS update task (100Hz)
///
/// Medium-frequency task for attitude estimation.
/// This is a placeholder - real implementation will run AHRS algorithm.
#[embassy_executor::task]
pub async fn ahrs_task() {
    let task_id = register_task(AHRS_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(AHRS_TASK_META.period_us() as u64));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        crate::core::scheduler::task::execute_with_timing(
            task_id,
            &AHRS_TASK_META,
            last_execution_us,
            now_us,
            || {
                // Simulate AHRS algorithm workload (~1000us)
                let start = Instant::now();
                while start.elapsed().as_micros() < 1000 {
                    core::hint::spin_loop();
                }
            },
        );

        last_execution_us = now_us;
    }
}

/// Control loop task (50Hz)
///
/// Medium-frequency task for vehicle control.
/// This is a placeholder - real implementation will run control algorithms.
#[embassy_executor::task]
pub async fn control_task() {
    let task_id = register_task(CONTROL_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(CONTROL_TASK_META.period_us() as u64));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        crate::core::scheduler::task::execute_with_timing(
            task_id,
            &CONTROL_TASK_META,
            last_execution_us,
            now_us,
            || {
                // Simulate control loop workload (~1500us)
                let start = Instant::now();
                while start.elapsed().as_micros() < 1500 {
                    core::hint::spin_loop();
                }
            },
        );

        last_execution_us = now_us;
    }
}

/// Telemetry task (10Hz)
///
/// Low-frequency task for telemetry transmission.
/// This is a placeholder - real implementation will send MAVLink messages.
#[embassy_executor::task]
pub async fn telemetry_task() {
    let task_id = register_task(TELEMETRY_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(TELEMETRY_TASK_META.period_us() as u64));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        crate::core::scheduler::task::execute_with_timing(
            task_id,
            &TELEMETRY_TASK_META,
            last_execution_us,
            now_us,
            || {
                // Simulate telemetry processing workload (~500us)
                let start = Instant::now();
                while start.elapsed().as_micros() < 500 {
                    core::hint::spin_loop();
                }
            },
        );

        last_execution_us = now_us;
    }
}
