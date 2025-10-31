# T-g729p Embassy Task Scheduler

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-g729p-embassy-task-scheduler-plan](plan.md)
- Related ADRs:
  - [ADR-vywkw-task-scheduler-selection](../../../adr/ADR-vywkw-task-scheduler-selection.md)
- Related Requirements:
  - [FR-5inw2-task-scheduler](../../../requirements/FR-5inw2-task-scheduler.md)
  - [NFR-ukjvr-control-loop-latency](../../../requirements/NFR-ukjvr-control-loop-latency.md)
  - [NFR-3wlo1-imu-sampling-rate](../../../requirements/NFR-3wlo1-imu-sampling-rate.md)

## Overview

Implement a task scheduler using Embassy async framework that executes periodic tasks at configurable rates (1Hz-400Hz) with deterministic timing. The scheduler provides task execution time monitoring, CPU load reporting, and meets real-time requirements for autopilot control loops.

## Success Metrics

- [ ] Tasks execute within 5% of target period over 10-second window
- [ ] Control loop latency ≤ 20ms (meets NFR-ukjvr)
- [ ] IMU sampling rate = 400Hz ± 5% (meets NFR-3wlo1)
- [ ] Scheduler overhead < 5% CPU
- [ ] Zero deadline misses under 75% CPU load
- [ ] All existing tests pass; no regressions

## Background and Current State

- Context: Core infrastructure for pico_trail autopilot. All control loops, sensor sampling, and telemetry depend on this scheduler.
- Current behavior: No scheduler exists yet - this is greenfield implementation.
- Pain points: N/A (new implementation)
- Constraints:
  - `no_std` environment
  - RP2040: 264 KB RAM, 133 MHz Cortex-M0+ (no FPU)
  - RP2350: 520 KB RAM, 150 MHz Cortex-M33 (with FPU)
  - Must work on both platforms
- Related ADRs: ADR-vywkw selected Embassy over RTIC and custom scheduler

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     Embassy Executor                         │
│  (Single-threaded async runtime on Cortex-M)                 │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│ High-Freq    │    │ Medium-Freq  │    │ Low-Freq     │
│ Tasks        │    │ Tasks        │    │ Tasks        │
│              │    │              │    │              │
│ - IMU (400Hz)│    │ - Control    │    │ - Telemetry  │
│ - AHRS(100Hz)│    │   (50Hz)     │    │   (10Hz)     │
│              │    │ - Nav (10Hz) │    │ - Logging    │
│              │    │              │    │   (5Hz)      │
└──────────────┘    └──────────────┘    └──────────────┘
        │                   │                   │
        └───────────────────┼───────────────────┘
                            ▼
                  ┌──────────────────┐
                  │ Task Statistics  │
                  │ - Execution time │
                  │ - CPU load       │
                  │ - Jitter         │
                  │ - Deadline miss  │
                  └──────────────────┘
```

### Components

#### 1. Task Registry (`src/core/scheduler/registry.rs`)

- Static registry of all tasks with metadata:
  - Task name (for debugging/logging)
  - Target rate (Hz)
  - Priority hint (not enforced by Embassy, but used for interrupt priorities)
  - Execution time budget (microseconds)
- Implemented as static array to avoid heap allocation

#### 2. Task Execution Wrapper (`src/core/scheduler/task.rs`)

- Wrapper around user task functions
- Measures execution time using platform timer
- Detects deadline misses (execution time > period)
- Logs warnings for overruns
- Updates task statistics

#### 3. Task Statistics (`src/core/scheduler/stats.rs`)

- Per-task execution statistics:
  - Last execution time
  - Average execution time (exponential moving average)
  - Maximum execution time
  - Deadline miss count
  - Jitter (deviation from target period)
- Global CPU load calculation:
  - Sum of (task execution time × task rate) over measurement window
  - Reported as percentage

#### 4. Ticker Tasks (`src/core/scheduler/tasks/*.rs`)

- Individual async tasks using `embassy_time::Ticker`
- Each task wraps user logic with execution time measurement
- Example structure:

```rust
#[embassy_executor::task]
async fn imu_task() {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        let start = Instant::now();
        ticker.next().await;

        // User task logic
        sample_imu().await;

        let elapsed = start.elapsed();
        update_task_stats("imu_task", elapsed, Duration::from_micros(2500));
    }
}
```

#### 5. Main Executor (`src/main.rs` or platform-specific entry point)

- Initializes Embassy executor
- Spawns all tasks
- Example:

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    init_platform();

    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(ahrs_task()).unwrap();
    spawner.spawn(control_task()).unwrap();
    spawner.spawn(telemetry_task()).unwrap();
    spawner.spawn(monitor_task()).unwrap();
}
```

### Data Flow

1. **Task Registration (Compile-time)**
   - Tasks are defined as `#[embassy_executor::task]` functions
   - Task metadata registered in static registry

2. **Task Execution (Runtime)**
   - Embassy executor schedules tasks cooperatively
   - Each task uses `Ticker` to wake at target rate
   - Task wrapper measures execution time before/after user logic
   - Statistics updated after each execution

3. **Monitoring (Periodic)**
   - Dedicated monitoring task runs at 1Hz
   - Collects statistics from all tasks
   - Calculates CPU load
   - Logs warnings for deadline misses or high CPU load

### Data Models and Types

```rust
/// Task metadata (compile-time registry)
pub struct TaskMetadata {
    pub name: &'static str,
    pub rate_hz: u32,
    pub priority: u8,
    pub budget_us: u32,
}

/// Task statistics (runtime)
pub struct TaskStats {
    pub last_execution_us: u32,
    pub avg_execution_us: u32,
    pub max_execution_us: u32,
    pub deadline_misses: u32,
    pub last_period_us: u32,
    pub avg_jitter_us: u32,
}

/// Global scheduler statistics
pub struct SchedulerStats {
    pub cpu_load_percent: u8,
    pub total_deadline_misses: u32,
    pub uptime_ms: u64,
}
```

### Error Handling

- **Deadline Miss**: Log warning with task name, expected period, actual execution time
- **CPU Overload**: Log warning when CPU load exceeds 75% for > 5 seconds
- **Task Panic**: Embassy will halt; use defmt or panic handler to log
- Error messages in English

### Security Considerations

Not applicable for embedded autopilot (no external input/network exposure in scheduler).

### Performance Considerations

#### Hot Paths

- Task execution measurement (called every task invocation)
  - Use hardware timer (`TIMER->TIMERAWL` on RP2040/RP2350) instead of software counter
  - Minimize overhead: single register read before/after task
- Statistics update
  - Use exponential moving average to avoid storing full history
  - Update inline (no separate task)

#### Caching Strategy

- Task metadata: Static array (no runtime lookup)
- Task statistics: Static mutable array (protected by critical section or atomic operations)

#### Async/Concurrency

- Embassy executor is single-threaded, so no mutex needed for task-local state
- Global statistics use `cortex_m::interrupt::free` for critical sections
- No heap allocation (all static)

#### Interrupt Priorities

- Embassy executor runs at priority 0 (lowest)
- High-priority tasks (IMU) use interrupt-driven wakeup at higher priority
- Embassy's `Ticker` uses hardware timer interrupts

### Platform Considerations

#### RP2040 (Pico W)

- Cortex-M0+: No FPU, limited CPU (133 MHz)
- Embassy tested and proven on RP2040
- Use `embassy-rp` crate v0.2+
- Timer: RP2040 has 64-bit microsecond timer
- Challenges: May need optimization to meet 400Hz IMU target

#### RP2350 (Pico 2 W)

- Cortex-M33: FPU available, 150 MHz
- More headroom for complex tasks
- Use `embassy-rp` with RP2350 support
- Timer: Same 64-bit microsecond timer

#### Cross-Platform

- Abstract platform-specific initialization in `src/platform/*/mod.rs`
- Use Embassy's `embassy-time` for portable timer access
- Feature flags: `pico_w` (RP2040), `pico2_w` (RP2350)

## Alternatives Considered

### Alternative A: RTIC Framework

- **Pros**:
  - Zero-cost abstractions (compile-time scheduling)
  - Lower overhead (< 2% CPU vs Embassy's \~5%)
  - Strong real-time guarantees
- **Cons**:
  - Steeper learning curve (resource model)
  - No built-in async I/O support
  - Cortex-M only (no future ESP32/STM32 portability)
- **Decision Rationale**: Embassy chosen for development velocity and async I/O (see ADR-vywkw)

### Alternative B: Custom Timer-based Scheduler

- **Pros**:
  - Full control over scheduling policy
  - Minimal overhead (tailored to exact needs)
- **Cons**:
  - High development effort (implement from scratch)
  - Requires extensive testing
  - No async I/O support
- **Decision Rationale**: Reinventing the wheel; Embassy provides proven solution

## Migration and Compatibility

- Backward compatibility: N/A (greenfield implementation)
- Forward compatibility: Design allows adding new tasks without breaking existing ones
- Rollout plan: Single-phase implementation (no phased rollout needed)

## Testing Strategy

### Unit Tests

- Task statistics calculation (average, jitter, deadline detection)
- CPU load calculation
- Task metadata registration
- Place tests in `src/core/scheduler/*/tests.rs` with `#[cfg(test)]`

### Integration Tests

- Spawn multiple tasks with different rates
- Measure actual execution periods (use mock timer or host system clock)
- Verify CPU load calculation accuracy
- Verify deadline miss detection
- Tests in `tests/scheduler_integration.rs`

### Hardware Tests

- Deploy to Pico W and Pico 2 W
- Measure jitter for 400Hz IMU task over 10,000 samples
- Measure control loop latency under full load
- Verify no deadline misses under 75% CPU load
- Manual testing (not automated)

### Performance Benchmarks

- Measure scheduler overhead (execution time measurement + statistics update)
- Benchmark on both RP2040 and RP2350
- Compare against target < 5% CPU overhead
- Use `defmt-rtt` for timing logs or cycle counter

## Documentation Impact

- Add `docs/scheduler.md` explaining task registration and usage
- Update `docs/architecture.md` with scheduler component
- Add code examples in scheduler module documentation

## External References

- Embassy Framework: <https://embassy.dev/>
- Embassy Time: <https://docs.embassy.dev/embassy-time/>
- Embassy RP2040/RP2350 HAL: <https://docs.embassy.dev/embassy-rp/>
- ArduPilot Scheduler: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>

## Open Questions

- [ ] Should we implement priority preemption using interrupt priorities? → Next step: Prototype and measure impact on jitter
- [ ] Is exponential moving average sufficient for task statistics, or do we need full history? → Method: Implement EMA first, add full history only if needed for debugging
- [ ] Should we support dynamic task registration, or only compile-time? → Decision: Start with compile-time only (simpler, zero overhead), add dynamic if needed

## Appendix

### Diagrams

#### Task Execution Timing

```text
Time (ms)
│
│ ┌───┐     ┌───┐     ┌───┐     ┌───┐   IMU Task (400Hz, 2.5ms period)
│ │   │     │   │     │   │     │   │
├─┴─┬─┴─────┴─┬─┴─────┴─┬─┴─────┴─┬─┴──
│   │         │         │         │
│   └─────┐   └─────┐   └─────┐   └────   Control Task (50Hz, 20ms period)
│         │         │         │
├─────────┴─────────┴─────────┴────────
│
```

### Examples

```rust
// Example: Register and spawn IMU task
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker, Instant};

#[embassy_executor::task]
async fn imu_task() {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        let start = Instant::now();
        ticker.next().await;

        // Sample IMU sensor
        let (gyro, accel) = imu::read().await;
        process_imu_data(gyro, accel);

        // Update statistics
        let elapsed = start.elapsed();
        scheduler::update_task_stats("imu", elapsed, Duration::from_micros(2500));
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let p = embassy_rp::init(Default::default());

    // Spawn tasks
    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(control_task()).unwrap();
    spawner.spawn(telemetry_task()).unwrap();
}
```

### Glossary

- **Ticker**: Embassy abstraction for periodic task wakeup
- **Spawner**: Embassy mechanism to create tasks
- **Deadline Miss**: Task execution time exceeds its period
- **Jitter**: Variation in task execution period
- **EMA**: Exponential Moving Average (used for statistics)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../../templates/README.md#design-template-designmd) in the templates README.
