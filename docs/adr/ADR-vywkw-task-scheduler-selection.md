# ADR-vywkw Task Scheduler Selection: Embassy Async Framework

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-5inw2-task-scheduler](../requirements/FR-5inw2-task-scheduler.md)
  - [NFR-ukjvr-control-loop-latency](../requirements/NFR-ukjvr-control-loop-latency.md)
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
- Supersedes ADRs: N/A
- Related Tasks: Will be created after approval

## Context

The autopilot requires a task scheduler to execute periodic tasks at different rates:

- **High-frequency**: IMU sampling (400Hz), AHRS update (100Hz)
- **Medium-frequency**: Control loops (50Hz), Navigation (10Hz)
- **Low-frequency**: Telemetry (10Hz), Logger (5Hz)

### Problem

We need deterministic, low-jitter task execution on resource-constrained microcontrollers (Pico W: 264 KB RAM, Pico 2 W: 520 KB RAM) while maintaining:

- Real-time performance (control loop ≤ 20ms latency)
- Low scheduler overhead (< 5% CPU)
- Priority-based preemption
- Task runtime monitoring

### Constraints

- Must work on both Cortex-M0+ (Pico W) and Cortex-M33 (Pico 2 W)
- No operating system (bare metal or lightweight runtime)
- Memory budget: < 10 KB for scheduler infrastructure
- Must integrate with async I/O (UART, I2C, SPI for sensors)

### Prior Art

- **ArduPilot**: Custom scheduler with fixed-rate tasks, proven but C++-based
- **PX4**: NuttX RTOS with uORB messaging, too heavyweight for Pico
- **Embedded Rust**: Embassy (async), RTIC (interrupt-driven), custom timer-based

## Success Metrics

- **Jitter**: < 1ms for 400Hz IMU task over 10-second window
- **Latency**: Control loop completes within 20ms (50Hz)
- **CPU overhead**: Scheduler consumes < 5% CPU time
- **Memory**: Scheduler infrastructure < 10 KB RAM
- **Reliability**: Zero missed deadlines under 75% CPU load

## Decision

**We will use Embassy async framework as the primary task scheduler for pico_trail.**

Embassy provides:

- Modern async/await syntax for readability
- Hardware timer-based executor with low jitter
- Proven on Cortex-M0+ and M33 platforms
- Active community and ecosystem support
- Built-in async drivers for peripherals (UART, I2C, SPI)

### Decision Drivers

1. **Cortex-M0+ Support**: Must work on Pico W (no FPU, limited CPU)
2. **Async I/O Integration**: Sensors use I2C/SPI, async prevents blocking
3. **Development Velocity**: async/await easier to write and maintain than custom scheduler
4. **Ecosystem**: Embassy has drivers for RP2040/RP2350
5. **Real-time Guarantees**: Hardware timer-based executor provides deterministic timing

### Considered Options

- **Option A: Embassy Async Framework** ⭐ Selected
- **Option B: RTIC (Real-Time Interrupt-driven Concurrency)**
- **Option C: Custom Timer-based Scheduler**

### Option Analysis

**Option A: Embassy Async Framework**

- **Pros**:
  - Modern async/await syntax, easy to understand
  - Proven on Cortex-M0+ and M33
  - Built-in async drivers (UART, I2C, SPI, Timer)
  - Active development and community support
  - `Ticker` abstraction for periodic tasks
- **Cons**:
  - Async overhead (futures, waker machinery)
  - Larger binary size than RTIC
  - Learning curve for async embedded patterns
- **Estimated overhead**: \~5 KB RAM, \~10 KB Flash, < 5% CPU

**Option B: RTIC**

- **Pros**:
  - Zero-cost abstractions (compile-time scheduling)
  - Minimal overhead (< 1 KB RAM)
  - Strong real-time guarantees
  - Cortex-M optimized
- **Cons**:
  - Steeper learning curve (resource model)
  - Less flexible than async (fixed task structure)
  - Cortex-M only (no portability to RISC-V or other architectures)
  - Smaller ecosystem (fewer async drivers)
- **Estimated overhead**: \~1 KB RAM, \~5 KB Flash, < 2% CPU

**Option C: Custom Timer-based Scheduler**

- **Pros**:
  - Full control over scheduling policy
  - Minimal overhead (tailored to exact needs)
  - No external dependencies
- **Cons**:
  - High development effort (implement scheduler from scratch)
  - Requires extensive testing (scheduler bugs are hard to debug)
  - No async I/O support (need to implement separately)
  - Reinventing the wheel
- **Estimated overhead**: \~2 KB RAM, \~8 KB Flash, \~3% CPU (after implementation)

## Rationale

Embassy was chosen over RTIC and custom scheduler for the following reasons:

1. **Development Velocity**: async/await is easier to write and maintain than RTIC's resource model or a custom scheduler. This accelerates development and reduces bugs.

2. **Async I/O**: Sensors (GPS, IMU, compass) use UART, I2C, and SPI. Embassy's async drivers prevent blocking control loops during I/O operations.

3. **Proven on Pico W**: Embassy has been successfully deployed on RP2040 (Cortex-M0+) in production systems, demonstrating it can meet performance requirements even without an FPU.

4. **Ecosystem**: Embassy provides async drivers for RP2040/RP2350, reducing the need to write custom HAL code.

5. **Future Portability**: If we later port to ESP32 or STM32, Embassy supports those platforms (unlike RTIC which is Cortex-M only).

### Trade-offs Accepted

- **Overhead**: Embassy has \~5% CPU overhead compared to RTIC's < 2%. We accept this trade-off for development velocity and async I/O benefits.
- **Binary Size**: Embassy binaries are \~10 KB larger than RTIC. This is acceptable given Pico 2 W has 4 MB Flash.
- **Learning Curve**: Async embedded Rust has a learning curve, but modern documentation and examples make this manageable.

### Why Not RTIC

RTIC is excellent for hard real-time systems and has lower overhead than Embassy. However:

- RTIC's resource model is more complex to use than async/await
- RTIC lacks built-in async I/O support (would need to implement separately)
- RTIC is Cortex-M only (limits future portability)

**Decision**: We prioritize development velocity and async I/O over micro-optimizations. If profiling reveals Embassy's overhead is unacceptable, we can revisit RTIC in a future ADR.

## Consequences

### Positive

- **Faster Development**: async/await syntax is intuitive, reducing time to implement control loops and I/O
- **Async I/O**: Non-blocking sensor reads prevent control loop stalls
- **Ecosystem**: Access to Embassy's async drivers (RP2040/RP2350 HAL, USB, network)
- **Community Support**: Active development, regular updates, extensive documentation
- **Portability**: Embassy supports multiple platforms (RP2040, RP2350, ESP32, STM32)

### Negative

- **Overhead**: \~5% CPU overhead and \~5 KB RAM compared to RTIC
- **Binary Size**: +\~10 KB Flash compared to RTIC
- **Async Complexity**: Developers must understand async/await and executors
- **Potential Jitter**: Async scheduling may have higher jitter than RTIC's compile-time scheduling (needs profiling)

### Neutral

- **No OS**: Embassy is not a full RTOS, just an async executor (intentional for simplicity)
- **Single Executor**: One executor per core (acceptable for single-threaded control loops)

## Implementation Notes

### Task Structure

```rust
#[embassy_executor::task]
async fn imu_task() {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        ticker.next().await;
        let (gyro, accel) = imu.read().await;
        imu_buffer.push((gyro, accel));
    }
}

#[embassy_executor::task]
async fn control_task() {
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50Hz
    loop {
        ticker.next().await;
        run_control_loop().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(control_task()).unwrap();
    spawner.spawn(telemetry_task()).unwrap();
}
```

### Priority Management

Embassy does not have explicit task priorities. To ensure high-frequency tasks run first:

1. Use **interrupt priorities**: IMU data-ready interrupt at highest priority
2. Use **cooperative scheduling**: High-frequency tasks yield quickly (short execution time)
3. Monitor **task execution time**: Log warnings if tasks exceed budget

### Performance Validation

Before finalizing this decision, we must:

1. **Profile on Pico W**: Measure actual jitter and CPU overhead on RP2040
2. **Stress Test**: Run all tasks simultaneously, ensure no deadline misses
3. **Benchmark vs RTIC**: If performance is insufficient, implement proof-of-concept with RTIC for comparison

## Examples

```rust
// High-frequency IMU sampling task (400Hz)
#[embassy_executor::task]
async fn imu_task(mut imu: impl ImuSensor) {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        ticker.next().await;

        // Sample IMU (non-blocking async I2C/SPI read)
        let gyro = imu.read_gyro().await;
        let accel = imu.read_accel().await;

        // Push to ring buffer for AHRS processing
        IMU_BUFFER.lock().await.push((gyro, accel));
    }
}

// Medium-frequency AHRS update task (100Hz)
#[embassy_executor::task]
async fn ahrs_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10)); // 100Hz
    loop {
        ticker.next().await;

        // Consume last 4 IMU samples (400Hz / 100Hz = 4 samples per update)
        let samples = IMU_BUFFER.lock().await.pop_n(4);

        // Update AHRS with high-rate IMU data
        AHRS.lock().await.update(&samples);
    }
}
```

## Platform Considerations

- **Pico W (RP2040, Cortex-M0+)**: Embassy supports RP2040 with `embassy-rp` crate. No FPU, so async overhead must be profiled carefully.
- **Pico 2 W (RP2350, Cortex-M33)**: Embassy supports RP2350. FPU available for faster math operations.
- **Cross-Platform**: Embassy also supports ESP32, STM32, nRF52, enabling future ports if needed.

## Monitoring & Logging

- **Task Execution Time**: Log execution time for each task, warn if exceeds budget
- **Deadline Misses**: Log warnings when tasks miss their target period (> 5% deviation)
- **CPU Load**: Periodically calculate and report overall CPU utilization
- **Jitter**: Measure and log jitter for critical tasks (IMU sampling)

## Open Questions

- [ ] Is Embassy's jitter acceptable for 400Hz IMU sampling on Pico W? → Next step: Implement IMU sampling prototype, measure jitter over 10,000 samples
- [ ] Can we achieve < 20ms control loop latency with async overhead? → Method: Profile control loop on Pico W under full load
- [ ] Should we implement RTIC fallback for Pico W if Embassy performance is insufficient? → Decision: Defer until profiling results available

## External References

- Embassy Framework: <https://embassy.dev/>
- Embassy RP2040 HAL: <https://docs.embassy.dev/embassy-rp/>
- RTIC Framework: <https://rtic.rs/>
- ArduPilot Scheduler: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
