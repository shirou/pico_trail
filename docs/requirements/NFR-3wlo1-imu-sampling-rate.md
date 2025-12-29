# NFR-3wlo1 IMU Sampling Rate

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
- Dependent Requirements:
  - [FR-eyuh8-ahrs-attitude-estimation](FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-slm3x-icm20948-i2c-driver](FR-slm3x-icm20948-i2c-driver.md)
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
  - [FR-3f2cn-quaternion-ekf-ahrs](FR-3f2cn-quaternion-ekf-ahrs.md)
  - [NFR-ulsja-imu-i2c-read-latency](NFR-ulsja-imu-i2c-read-latency.md)
- Related Tasks:
  - [T-49k7n-ahrs-dcm-implementation](../tasks/T-49k7n-ahrs-dcm-implementation/README.md)
  - ~~[T-qwvco-bmi088-imu-driver-implementation](../tasks/T-qwvco-bmi088-imu-driver-implementation/README.md)~~ (Deprecated)

## Requirement Statement

Critical sensor data shall be sampled at 400Hz minimum for IMU (gyroscope and accelerometer) with jitter below 1ms, ensuring smooth attitude estimation and stable control loop performance.

## Rationale

High-frequency IMU sampling is essential for:

- **Smooth AHRS**: Higher sample rate reduces integration error, improves attitude accuracy
- **Vibration Rejection**: 400Hz sampling captures high-frequency vibrations for filtering
- **ArduPilot Standard**: ArduPilot samples IMU at 400Hz minimum for all vehicle types
- **Nyquist Theorem**: 400Hz sampling captures motion up to 200Hz (well above typical vehicle dynamics)

Jitter (variation in sample timing) introduces errors in gyroscope integration. Keeping jitter below 1ms (0.4% of 2.5ms period) minimizes these errors.

## User Story (if applicable)

The system shall sample IMU data at 400Hz with low jitter to ensure accurate attitude estimation and stable control, particularly during high-rate maneuvers or in the presence of vibrations.

## Acceptance Criteria

- [ ] IMU sampling rate ≥ 400Hz (2.5ms period) measured over 10-second window
- [ ] IMU sampling jitter < 1ms (measured via timestamp statistics)
- [ ] IMU sampling continues at 400Hz under 75% CPU load
- [ ] No missed IMU samples during normal operation
- [ ] Gyroscope and accelerometer sampled simultaneously (minimize latency between sensors)
- [ ] Sampling rate verified on both Pico W and Pico 2 W platforms

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance:**

- **Sample Rate**: 400Hz (2.5ms period)
  - Period: 2.5ms ± 0.025ms (1% tolerance, equivalent to < 1ms jitter)
  - Target 400 samples per second over 10-second measurement window
- **Jitter Budget**:
  - Ideal: < 0.1ms (negligible impact)
  - Acceptable: < 1ms (0.4% timing error)
  - Maximum: < 2.5ms (avoid missing samples)

**Measurement Method:**

```rust
// Timestamp each IMU sample
let mut timestamps = [0u64; 4000]; // 10 seconds @ 400Hz
for i in 0..4000 {
    timestamps[i] = timer.now_micros();
    sample_imu();
}

// Calculate statistics
let periods: Vec<u64> = timestamps.windows(2)
    .map(|w| w[1] - w[0])
    .collect();

let mean_period = periods.iter().sum::<u64>() / periods.len() as u64;
let jitter = periods.iter()
    .map(|p| (*p as i64 - mean_period as i64).abs())
    .max().unwrap();

assert!(mean_period >= 2400 && mean_period <= 2600); // 2.5ms ± 4%
assert!(jitter < 1000); // < 1ms jitter
```

**Implementation Strategies:**

**Option A: Interrupt-Driven (Recommended)**

```rust
// IMU data-ready interrupt triggers sampling at exactly 400Hz
#[interrupt]
fn IMU_DATA_READY() {
    let gyro = imu.read_gyro();
    let accel = imu.read_accel();
    imu_buffer.push((gyro, accel));
}
```

**Pros**: Lowest jitter, no polling overhead
**Cons**: Requires IMU with data-ready interrupt pin

**Option B: Hardware Timer**

```rust
// Timer interrupt at 400Hz
#[interrupt]
fn TIMER_IRQ() {
    let gyro = imu.read_gyro();
    let accel = imu.read_accel();
    imu_buffer.push((gyro, accel));
}
```

**Pros**: Deterministic timing, works with any IMU
**Cons**: Slightly higher jitter if I2C/SPI transaction time varies

**Option C: Async Task (Not Recommended for Critical Sampling)**

```rust
#[embassy_executor::task]
async fn imu_task() {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        ticker.next().await;
        let (gyro, accel) = imu.read().await;
        // Process...
    }
}
```

**Pros**: Easy to write, integrates with async runtime
**Cons**: Higher jitter (10-50ms possible under load), not acceptable for critical sampling

**Sensor Configuration:**

Common IMU sensors and their capabilities:

- **MPU6050**: 400Hz max ODR (Output Data Rate), I2C interface, data-ready interrupt
- **MPU9250**: 1000Hz max ODR, I2C/SPI interface, data-ready interrupt
- **BMI088**: 1600Hz gyro, 1600Hz accel, SPI interface, data-ready interrupts

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- I2C transaction (\~1ms) may be tight for 2.5ms period
- Use SPI for faster IMU communication (< 0.5ms)
- May need to optimize I2C/SPI driver for low latency

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU:

- More headroom for IMU processing
- Can use I2C or SPI comfortably at 400Hz
- FPU accelerates AHRS calculations after sampling

### Cross-Platform

IMU sampling must meet 400Hz requirement on both platforms. Use platform-optimized I2C/SPI drivers.

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                                  | Validation                               |
| -------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ---------------------------------------- |
| I2C transaction time exceeds 2.5ms     | High   | Medium     | Use SPI instead (10x faster), optimize I2C clock rate       | Measure I2C read time with oscilloscope  |
| Interrupt latency causes jitter        | Medium | Medium     | Use high-priority interrupt for IMU, minimize ISR duration  | Measure jitter over 10,000 samples       |
| AHRS cannot process 400Hz data in time | Medium | Low        | AHRS runs at 100Hz (decimates 4:1), not 400Hz               | Verify AHRS update completes within 10ms |
| Missed samples under high CPU load     | High   | Low        | Ensure IMU sampling has highest priority, profile CPU usage | Stress test with 100% CPU load           |

## Implementation Notes

**Recommended Architecture:**

```rust
// High-priority interrupt samples IMU at 400Hz
#[interrupt(priority = 1)] // Highest priority
fn IMU_DATA_READY() {
    let (gyro, accel) = imu.read_raw(); // Fast read, no processing
    imu_ringbuffer.push((gyro, accel)); // Buffered for AHRS
}

// AHRS task runs at 100Hz, consumes IMU data
#[embassy_executor::task]
async fn ahrs_task() {
    let mut ticker = Ticker::every(Duration::from_millis(10)); // 100Hz
    loop {
        ticker.next().await;

        // Consume last 4 IMU samples (400Hz / 100Hz = 4 samples per update)
        let samples: [(Vector3, Vector3); 4] = imu_ringbuffer.pop_n(4);

        // Update AHRS with high-rate IMU data
        ahrs.update(&samples);
    }
}
```

**Performance Optimization:**

- Use **DMA** for I2C/SPI transfers to minimize CPU usage
- Use **SPI** instead of I2C for lower latency (< 0.5ms vs 1-2ms)
- **Batch reads**: Read gyro + accel + temp in single transaction
- **Inline** small functions in IMU driver to reduce call overhead

Related code areas:

- `src/devices/imu/` - IMU device drivers
- `src/platform/*/i2c.rs`, `src/platform/*/spi.rs` - I2C/SPI implementations
- `src/subsystems/ahrs/` - AHRS consuming IMU data

## External References

- ArduPilot IMU Sampling: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>
- MPU6050 Datasheet: <https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/>
- BMI088 Datasheet: <https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
