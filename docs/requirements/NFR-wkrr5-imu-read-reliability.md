# NFR-wkrr5 IMU Read Reliability

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)

## Requirement Statement

IMU sensor reads shall have a reliability of less than 1 read error per 1000 samples under normal operation, with retry logic to recover from transient I2C failures.

## Rationale

Reliable IMU data is critical for attitude estimation:

- **EKF Continuity**: Missing samples cause prediction-only updates, increasing uncertainty
- **400Hz Requirement**: At 400Hz, 1 error per 1000 samples = 0.4 errors per second
- **Safety**: Attitude errors can lead to navigation failures
- **I2C Robustness**: I2C is susceptible to EMI and bus glitches

Retry logic with bounded attempts ensures transient errors don't propagate while preventing infinite retry loops.

## User Story (if applicable)

The system shall maintain < 0.1% IMU read error rate to ensure continuous, reliable attitude estimation without degraded performance from sensor communication failures.

## Acceptance Criteria

- [ ] Error rate < 1 per 1000 reads under normal operation (bench testing)
- [ ] Error rate < 5 per 1000 reads under vibration/EMI conditions
- [ ] Retry logic attempts up to 3 reads before returning error
- [ ] Consecutive error count tracked for health monitoring
- [ ] Sensor marked unhealthy after 10 consecutive errors
- [ ] Error events logged with diagnostic information
- [ ] Sensor recovers automatically after transient error clears

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Reliability Metrics:**

| Condition             | Target Error Rate | Maximum Error Rate |
| --------------------- | ----------------- | ------------------ |
| Normal operation      | < 0.05% (1/2000)  | < 0.1% (1/1000)    |
| Vibration environment | < 0.2% (2/1000)   | < 0.5% (5/1000)    |
| High EMI environment  | < 0.5% (5/1000)   | < 1.0% (10/1000)   |

**Retry Strategy:**

```rust
const MAX_RETRIES: u8 = 3;
const RETRY_DELAY_US: u64 = 100; // 100µs between retries

async fn read_with_retry<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u8,
    buf: &mut [u8],
) -> Result<(), ImuError> {
    for attempt in 0..MAX_RETRIES {
        match i2c.write_read(addr, &[reg], buf).await {
            Ok(()) => return Ok(()),
            Err(e) => {
                if attempt < MAX_RETRIES - 1 {
                    log_warn!("IMU read retry {}/{}", attempt + 1, MAX_RETRIES);
                    Timer::after_micros(RETRY_DELAY_US).await;
                } else {
                    log_error!("IMU read failed after {} attempts: {:?}",
                        MAX_RETRIES, e);
                    return Err(ImuError::I2cError);
                }
            }
        }
    }
    Err(ImuError::I2cError)
}
```

**Health Monitoring:**

```rust
pub struct ImuHealth {
    /// Total read attempts
    pub total_reads: u32,

    /// Successful reads
    pub successful_reads: u32,

    /// Total errors (after retries exhausted)
    pub error_count: u32,

    /// Consecutive error count (reset on success)
    pub consecutive_errors: u8,

    /// Maximum consecutive errors seen
    pub max_consecutive_errors: u8,

    /// Sensor healthy flag
    pub healthy: bool,
}

impl ImuHealth {
    const UNHEALTHY_THRESHOLD: u8 = 10;

    pub fn record_success(&mut self) {
        self.total_reads += 1;
        self.successful_reads += 1;
        self.consecutive_errors = 0;
        self.healthy = true;
    }

    pub fn record_error(&mut self) {
        self.total_reads += 1;
        self.error_count += 1;
        self.consecutive_errors += 1;
        self.max_consecutive_errors =
            self.max_consecutive_errors.max(self.consecutive_errors);

        if self.consecutive_errors >= Self::UNHEALTHY_THRESHOLD {
            self.healthy = false;
            log_error!("IMU marked unhealthy: {} consecutive errors",
                self.consecutive_errors);
        }
    }

    pub fn error_rate(&self) -> f32 {
        if self.total_reads == 0 {
            0.0
        } else {
            self.error_count as f32 / self.total_reads as f32
        }
    }
}
```

**Data Validation:**

```rust
fn validate_reading(reading: &ImuReading) -> bool {
    // Check for stuck sensor (all zeros or all max)
    let gyro_ok = reading.gyro.norm() < 100.0; // < 100 rad/s
    let accel_ok = (1.0..100.0).contains(&reading.accel.norm()); // 1-100 m/s²
    let mag_ok = reading.mag.norm() < 1000.0; // < 1000 µT

    gyro_ok && accel_ok && mag_ok
}
```

## Platform Considerations

### Cross-Platform

- Same retry logic on both RP2040 and RP2350
- Health tracking uses platform-independent counters
- Error logging through standard logging macros

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                              | Validation                               |
| ----------------------------------------- | ------ | ---------- | --------------------------------------- | ---------------------------------------- |
| Repeated transient errors exhaust retries | Medium | Low        | Short retry delay, log first error      | Monitor retry statistics                 |
| Stuck sensor returns valid-looking data   | High   | Low        | Implement variance monitoring over time | Test with sensor disconnected            |
| EMI from motors causes I2C errors         | High   | Medium     | Shield I2C wires, use dedicated bus     | Test with motors running                 |
| Health flag doesn't recover               | Medium | Low        | Clear consecutive count on success      | Test recovery after disconnect/reconnect |

## Implementation Notes

**Error Statistics Logging:**

```rust
// Log error statistics periodically (every 10 seconds)
if sample_count % 4000 == 0 { // 4000 samples @ 400Hz = 10 seconds
    log_info!("IMU health: reads={}, errors={}, rate={:.2}%",
        health.total_reads,
        health.error_count,
        health.error_rate() * 100.0);
}
```

**Telemetry Integration:**

Error rate and health status should be included in health telemetry (see FR-1k7yd-health-telemetry-reporting).

## External References

- N/A - No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
