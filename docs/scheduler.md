# Task Scheduler

## Overview

pico_trail uses an Embassy-based async task scheduler for deterministic execution of periodic tasks. The scheduler manages tasks running at different frequencies (1Hz-400Hz), provides execution time monitoring, CPU load tracking, and ensures real-time performance guarantees for autopilot control loops.

**Key Features:**

- Periodic task execution with configurable rates (1Hz-400Hz)
- Task execution time measurement and monitoring
- CPU load calculation and reporting
- Deadline miss detection
- Jitter measurement (timing accuracy)
- Zero-cost abstractions via Embassy async framework

**Performance Characteristics (RP2350):**

- IMU task jitter: <1ms at 400Hz
- Control loop latency: <2ms
- Scheduler overhead: <5% CPU
- Zero deadline misses under 75% CPU load

## Quick Start

### Basic Task Example

```rust
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use pico_trail::core::scheduler::{register_task, TaskMetadata};

// Define task metadata
const MY_TASK_META: TaskMetadata = TaskMetadata {
    name: "my_task",
    rate_hz: 100,           // Run at 100Hz
    priority: 5,            // Priority hint
    budget_us: 5000,        // 5ms execution budget
};

#[embassy_executor::task]
pub async fn my_task() {
    // Register task and get task ID
    let task_id = register_task(MY_TASK_META);

    // Create ticker for periodic execution
    let mut ticker = Ticker::every(Duration::from_micros(MY_TASK_META.period_us() as u64));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        // Execute task with timing measurement
        pico_trail::core::scheduler::task::execute_with_timing(
            task_id,
            &MY_TASK_META,
            last_execution_us,
            now_us,
            || {
                // Your task logic here
                do_work();
            },
        );

        last_execution_us = now_us;
    }
}
```

### Spawning Tasks

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let p = embassy_rp::init(Default::default());

    // Spawn tasks
    spawner.spawn(my_task()).unwrap();
    spawner.spawn(monitor_task()).unwrap();
}
```

## Core Concepts

### TaskMetadata

Task metadata defines static configuration for each task:

```rust
pub struct TaskMetadata {
    pub name: &'static str,    // Task name for debugging/logging
    pub rate_hz: u32,           // Target execution rate (Hz)
    pub priority: u8,           // Priority hint (0-255, higher = more important)
    pub budget_us: u32,         // Execution time budget (microseconds)
}
```

**Helper Method:**

- `period_us()`: Calculates period in microseconds from rate (1,000,000 / rate_hz)

**Example:**

```rust
const IMU_TASK_META: TaskMetadata = TaskMetadata {
    name: "imu_task",
    rate_hz: 400,           // 400Hz = 2.5ms period
    priority: 10,           // High priority
    budget_us: 2000,        // Must complete in <2ms
};
```

### TaskStats

Runtime statistics tracked for each task:

```rust
pub struct TaskStats {
    pub last_execution_us: u32,      // Last execution time
    pub avg_execution_us: u32,       // Average execution time (EMA)
    pub max_execution_us: u32,       // Maximum execution time
    pub deadline_misses: u32,        // Number of deadline misses
    pub last_period_us: u32,         // Last period measurement
    pub avg_jitter_us: u32,          // Average jitter (EMA)
}
```

Statistics are updated automatically by `execute_with_timing()`.

### SchedulerStats

Global scheduler statistics:

```rust
pub struct SchedulerStats {
    pub cpu_load_percent: u8,        // CPU load percentage (0-100)
    pub total_deadline_misses: u32,  // Total deadline misses across all tasks
    pub uptime_ms: u64,              // System uptime in milliseconds
}
```

## Task Registration

### Static Registration

Tasks are registered at runtime using the `register_task()` function:

```rust
let task_id = register_task(MY_TASK_META);
```

This returns a task ID (index) used for statistics lookup. Registration is thread-safe and uses critical sections.

**Registry Limits:**

- Maximum 16 tasks (defined in `registry.rs`)
- Registration is first-come-first-served
- Panics if registry is full (compile-time task count should be known)

### Task Metadata Storage

Task metadata is stored in a static array and never modified after registration:

```rust
static TASK_REGISTRY: Mutex<RefCell<TaskRegistry>> = Mutex::new(RefCell::new(TaskRegistry::new()));
```

## Task Implementation

### Execution Pattern

All tasks follow this pattern:

1. Define `TaskMetadata` constant
2. Create async task function with `#[embassy_executor::task]`
3. Register task using `register_task()`
4. Create `Ticker` for periodic wakeup
5. Use `execute_with_timing()` to wrap task logic

### execute_with_timing()

Wraps task execution with timing measurement:

```rust
pub fn execute_with_timing<F>(
    task_id: usize,
    metadata: &TaskMetadata,
    last_execution_us: u64,
    current_time_us: u64,
    task_fn: F,
) where
    F: FnOnce(),
```

**Behavior:**

- Measures execution time
- Calculates period (time since last execution)
- Detects deadline misses (execution time > budget)
- Calculates jitter (period deviation from target)
- Updates task statistics automatically

### Example Tasks

**High-frequency task (400Hz IMU sampling):**

```rust
const IMU_TASK_META: TaskMetadata = TaskMetadata {
    name: "imu_task",
    rate_hz: 400,
    priority: 10,
    budget_us: 2000,
};

#[embassy_executor::task]
pub async fn imu_task() {
    let task_id = register_task(IMU_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        execute_with_timing(task_id, &IMU_TASK_META, last_execution_us, now_us, || {
            // Read IMU sensor
            let (gyro, accel) = imu::read();
            process_imu_data(gyro, accel);
        });

        last_execution_us = now_us;
    }
}
```

**Medium-frequency task (50Hz control loop):**

```rust
const CONTROL_TASK_META: TaskMetadata = TaskMetadata {
    name: "control_task",
    rate_hz: 50,
    priority: 7,
    budget_us: 15000,
};

#[embassy_executor::task]
pub async fn control_task() {
    let task_id = register_task(CONTROL_TASK_META);
    let mut ticker = Ticker::every(Duration::from_micros(20000)); // 50Hz
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        execute_with_timing(task_id, &CONTROL_TASK_META, last_execution_us, now_us, || {
            // Run control loop
            update_pid_controllers();
            set_motor_outputs();
        });

        last_execution_us = now_us;
    }
}
```

## Execution Model

### Embassy Executor

The scheduler uses Embassy's single-threaded cooperative executor:

- Tasks are `async` functions that yield at `.await` points
- No preemption within tasks (cooperative multitasking)
- Executor runs tasks to completion until they yield
- Hardware interrupts can preempt the executor

### Ticker Mechanism

`embassy_time::Ticker` provides periodic task wakeup:

```rust
let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
loop {
    ticker.next().await;  // Suspend task until next tick
    // Task work
}
```

Ticker uses hardware timers for accurate timing.

### Task Priorities

Priority values in `TaskMetadata` are hints, not enforced by Embassy:

- Embassy executor does not implement priority-based scheduling
- All tasks are equal priority (cooperative scheduling)
- Tasks should complete quickly and yield frequently
- Use interrupt priorities for true preemption if needed

## Statistics and Monitoring

### Accessing Statistics

**Get task metadata:**

```rust
let task_count = pico_trail::core::scheduler::task_count();
for i in 0..task_count {
    if let Some(metadata) = pico_trail::core::scheduler::get_task(i) {
        println!("Task: {}", metadata.name);
    }
}
```

**Get task statistics:**

```rust
let stats = pico_trail::core::scheduler::get_task_stats(task_id);
println!("Avg execution: {}us", stats.avg_execution_us);
println!("Jitter: {}us", stats.avg_jitter_us);
println!("Deadline misses: {}", stats.deadline_misses);
```

**Get scheduler statistics:**

```rust
let sched_stats = pico_trail::core::scheduler::get_scheduler_stats();
println!("CPU load: {}%", sched_stats.cpu_load_percent);
println!("Total misses: {}", sched_stats.total_deadline_misses);
```

### CPU Load Calculation

Update CPU load periodically:

```rust
// Update every 1 second (1,000,000 microseconds)
let cpu_load = pico_trail::core::scheduler::update_cpu_load(1_000_000);
```

CPU load is calculated as:

```
CPU Load = Σ(task_execution_time × task_rate) / measurement_window
```

For example:

- IMU: 500us × 400Hz = 200,000us/s = 20%
- Control: 1500us × 50Hz = 75,000us/s = 7.5%
- Total: 27.5% CPU load

### Monitor Task

The monitor task runs at 1Hz and reports statistics:

```rust
#[embassy_executor::task]
pub async fn monitor_task() {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        // Update CPU load
        let cpu_load = update_cpu_load(1_000_000);
        let stats = get_scheduler_stats();

        defmt::info!("CPU: {}%, Misses: {}", cpu_load, stats.total_deadline_misses);

        // Log per-task statistics
        for i in 0..task_count() {
            if let Some(meta) = get_task(i) {
                let task_stats = get_task_stats(i);
                defmt::info!(
                    "{}: {}us (jitter: {}us)",
                    meta.name,
                    task_stats.avg_execution_us,
                    task_stats.avg_jitter_us
                );
            }
        }
    }
}
```

Spawn the monitor task to enable automatic statistics reporting:

```rust
spawner.spawn(monitor_task()).unwrap();
```

## Performance Characteristics

### Measured Performance (RP2350, Pico 2 W)

Based on hardware validation (T-00001 Phase 3):

| Metric                    | Target  | Measured | Status |
| ------------------------- | ------- | -------- | ------ |
| IMU task rate             | 400Hz   | 400Hz    | ✅     |
| IMU jitter                | <1ms    | 569us    | ✅     |
| Control loop latency      | ≤20ms   | 1.5ms    | ✅     |
| Control loop jitter       | -       | 925us    | ✅     |
| Scheduler overhead        | <5% CPU | \~2%     | ✅     |
| CPU load (4 tasks)        | -       | 38%      | ✅     |
| Deadline misses (4 tasks) | 0       | 0        | ✅     |

### Platform Considerations

**RP2350 (Pico 2 W):**

- Cortex-M33 @ 150MHz
- Hardware FPU available
- More headroom for complex tasks
- Recommended for development

**RP2040 (Pico W):**

- Cortex-M0+ @ 133MHz
- No FPU
- May require optimization for 400Hz tasks
- Not yet validated (hardware not available)

## Best Practices

### Task Design

1. **Keep tasks short**: Tasks should complete in <50% of their period
2. **Avoid blocking**: Use async I/O instead of busy-waiting
3. **Minimize allocations**: Prefer stack allocation or static buffers
4. **Set realistic budgets**: Budget should be 2-3x typical execution time for headroom

### Execution Budget Guidelines

| Task Rate | Period | Suggested Budget | Max Work Time |
| --------- | ------ | ---------------- | ------------- |
| 400Hz     | 2.5ms  | 1-2ms            | <1.25ms       |
| 100Hz     | 10ms   | 5-8ms            | <5ms          |
| 50Hz      | 20ms   | 10-15ms          | <10ms         |
| 10Hz      | 100ms  | 50ms             | <50ms         |
| 1Hz       | 1000ms | 500ms            | <500ms        |

### Priority Assignment

Higher priority values for more critical tasks:

- **10**: Time-critical sensor sampling (IMU)
- **8-9**: Attitude estimation (AHRS)
- **6-7**: Control loops
- **3-5**: Telemetry, logging
- **1-2**: Background tasks, housekeeping

### CPU Load Management

- Target <75% CPU load for normal operation
- Reserve 25% for transient spikes and safety margins
- Monitor CPU load and reduce task rates if needed
- Consider disabling non-critical tasks under high load

### Jitter Minimization

- Avoid long critical sections (disable interrupts)
- Keep tasks short and predictable
- Avoid dynamic memory allocation in high-frequency tasks
- Use fixed-size buffers

## Troubleshooting

### Deadline Misses

**Symptoms:** `deadline_misses` counter increments, warnings in logs

**Causes:**

- Task execution time exceeds budget
- CPU overload (total load >100%)
- Interrupt handler taking too long

**Solutions:**

1. Check task execution times: `stats.avg_execution_us`
2. Optimize slow tasks (profiling, algorithm improvements)
3. Reduce task rates if acceptable
4. Move work to lower-priority tasks
5. Disable non-critical tasks

### High Jitter

**Symptoms:** `avg_jitter_us` significantly larger than expected

**Causes:**

- Variable task execution time
- Interrupt latency variations
- Other tasks blocking the executor

**Solutions:**

1. Make task execution time more predictable
2. Reduce work in interrupt handlers
3. Check for long-running tasks blocking the executor
4. Use higher interrupt priority for critical timers

### CPU Overload

**Symptoms:** CPU load >90%, system becomes unresponsive

**Causes:**

- Too many tasks or too high frequencies
- Tasks taking longer than expected
- Inefficient algorithms

**Solutions:**

1. Review task rates and reduce if possible
2. Optimize task implementations
3. Profile with `defmt` timestamps
4. Consider hardware upgrade (RP2040 → RP2350)

### Task Not Running

**Symptoms:** Task statistics show zero executions

**Causes:**

- Task not spawned in `main()`
- Task panicked (check panic handler)
- Ticker period too long (task appears frozen)

**Solutions:**

1. Verify `spawner.spawn(task()).unwrap()` in main
2. Check for panic messages in logs
3. Add `defmt::info!()` in task loop to verify execution
4. Check task is registered correctly

## Examples

### Complete Scheduler Demo

See `examples/scheduler_demo_usb.rs` for a complete working example with:

- Multiple tasks at different frequencies (400Hz, 100Hz, 50Hz, 10Hz)
- USB-CDC output for statistics
- Monitor task for periodic reporting

**Build and run:**

```bash
./scripts/build-rp2350.sh scheduler_demo_usb --release
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/scheduler_demo_usb
```

**Expected output:**

```
=== t=73s ===
CPU:38% Miss:0
imu_task: 500us j=569us
ahrs_task: 1000us j=723us
control_task: 1500us j=925us
telemetry_task: 500us j=450us
```

### Minimal Example

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use pico_trail::core::scheduler::{register_task, TaskMetadata};
use {defmt_rtt as _, panic_probe as _};

const BLINK_TASK_META: TaskMetadata = TaskMetadata {
    name: "blink",
    rate_hz: 1,
    priority: 1,
    budget_us: 100000,
};

#[embassy_executor::task]
async fn blink_task() {
    let task_id = register_task(BLINK_TASK_META);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut last_execution_us = 0u64;

    loop {
        ticker.next().await;
        let now_us = Instant::now().as_micros();

        pico_trail::core::scheduler::task::execute_with_timing(
            task_id,
            &BLINK_TASK_META,
            last_execution_us,
            now_us,
            || {
                defmt::info!("Blink!");
            },
        );

        last_execution_us = now_us;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    spawner.spawn(blink_task()).unwrap();
}
```

## Related Documentation

- [Architecture](architecture.md) - System architecture and component overview
- [Task Design Document](archive/tasks/T-00001-embassy-task-scheduler/design.md) - Detailed scheduler design
- [Task Plan](archive/tasks/T-00001-embassy-task-scheduler/plan.md) - Implementation plan and validation results
- [ADR-00005](adr/ADR-00005-task-scheduler-selection.md) - Scheduler framework selection decision
- [FR-00007](requirements/FR-00007-task-scheduler.md) - Task scheduler functional requirements
- [NFR-00001](requirements/NFR-00001-control-loop-latency.md) - Control loop latency requirement
- [NFR-00002](requirements/NFR-00002-imu-sampling-rate.md) - IMU sampling rate requirement
- [Embassy Documentation](https://embassy.dev/) - Official Embassy framework docs
