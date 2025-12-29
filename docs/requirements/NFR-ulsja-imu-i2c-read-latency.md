# NFR-ulsja IMU I2C Read Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-slm3x-icm20948-i2c-driver](FR-slm3x-icm20948-i2c-driver.md)
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
  - [NFR-3wlo1-imu-sampling-rate](NFR-3wlo1-imu-sampling-rate.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-0kbo4-icm20948-driver-implementation](../tasks/T-0kbo4-icm20948-driver-implementation/README.md)
  - [T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)

## Requirement Statement

Full 9-axis IMU read (gyroscope, accelerometer, magnetometer, temperature) via I2C shall complete within 1.5ms to allow sustained 400Hz sampling with headroom for processing.

## Rationale

At 400Hz sampling (2.5ms period), the I2C transaction time directly impacts available processing time:

- **2.5ms period** - Total time budget per sample
- **1.5ms I2C read** - Communication time (target)
- **1.0ms remaining** - For calibration, buffering, task switching

I2C at 400kHz (Fast Mode) can theoretically transfer at \~40KB/s. A 21-byte read (14 from MPU-9250 + 7 from AK8963) requires:

- Address + register: \~0.2ms
- Data transfer: \~0.5ms
- Two transactions: \~1.0-1.5ms total

This timing provides adequate margin for sustained 400Hz operation.

## User Story (if applicable)

The system shall complete I2C IMU reads within 1.5ms to ensure reliable 400Hz sampling without timing overruns or missed samples.

## Acceptance Criteria

- [ ] Single 14-byte MPU-9250 read (gyro+accel+temp) completes in < 1.0ms
- [ ] Single 7-byte AK8963 read (magnetometer) completes in < 0.5ms
- [ ] Combined 9-axis read completes in < 1.5ms (95th percentile)
- [ ] Maximum read time < 2.0ms (no sample period overrun)
- [ ] Latency measured on both RP2040 and RP2350 platforms
- [ ] No I2C clock stretching exceeding 0.5ms

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance Targets:**

| Operation                | Target  | Maximum | Notes                      |
| ------------------------ | ------- | ------- | -------------------------- |
| MPU-9250 read (14 bytes) | < 0.8ms | < 1.0ms | Address + read transaction |
| AK8963 read (7 bytes)    | < 0.4ms | < 0.5ms | Address + read transaction |
| Combined 9-axis          | < 1.2ms | < 1.5ms | Sequential transactions    |
| 95th percentile          | < 1.5ms | -       | Under normal operation     |
| Maximum                  | -       | < 2.0ms | No sample overrun          |

**I2C Configuration:**

- Clock: 400kHz (Fast Mode)
- Pull-up resistors: 4.7kΩ (standard) or 2.2kΩ (for longer wires)
- Bus capacitance: < 400pF (I2C spec for Fast Mode)

**Measurement Method:**

```rust
// Measure I2C read latency
let start = timer.now_micros();
let mut buf = [0u8; 14];
i2c.write_read(MPU9250_ADDR, &[ACCEL_XOUT_H], &mut buf).await?;
let mpu_time = timer.now_micros() - start;

let start = timer.now_micros();
let mut mag_buf = [0u8; 7];
i2c.write_read(AK8963_ADDR, &[HXL], &mut mag_buf).await?;
let mag_time = timer.now_micros() - start;

log_debug!("I2C latency: MPU={}us, Mag={}us, Total={}us",
    mpu_time, mag_time, mpu_time + mag_time);
```

**Optimization Strategies:**

1. **DMA Transfers**: Use DMA to reduce CPU blocking during I2C
2. **Combined Reads**: Batch MPU-9250 and AK8963 reads if possible
3. **No Repeated Starts**: Avoid repeated start conditions if driver supports
4. **Higher Clock**: Consider 1MHz (Fast Mode Plus) if supported

## Platform Considerations

### Pico W (RP2040)

- I2C via `embassy_rp::i2c::I2c`
- DMA available but may not improve latency significantly for small transfers
- 133MHz CPU provides adequate headroom

### Pico 2 W (RP2350)

- Same I2C peripheral interface
- 150MHz CPU provides more margin
- Should comfortably meet latency targets

### Cross-Platform

- Use async I2C driver to allow concurrent operations
- Monitor latency statistics in debug builds

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                                              | Validation                              |
| -------------------------------- | ------ | ---------- | ------------------------------------------------------- | --------------------------------------- |
| Clock stretching causes delays   | High   | Medium     | Use sensors with predictable timing, timeout on stretch | Log stretch events, validate with scope |
| Bus contention increases latency | Medium | Low        | Dedicated I2C bus for IMU                               | Test with GPS on same bus               |
| Long wires increase capacitance  | Medium | Medium     | Use shorter cables, lower pull-ups                      | Measure rise times with scope           |
| Async runtime adds jitter        | Medium | Low        | Use high-priority async task                            | Profile async overhead                  |

## Implementation Notes

**Latency Monitoring:**

```rust
// Collect latency statistics
struct I2cLatencyStats {
    count: u32,
    sum_us: u64,
    max_us: u32,
    over_1500us: u32, // Count of reads > 1.5ms
}

impl I2cLatencyStats {
    fn record(&mut self, latency_us: u32) {
        self.count += 1;
        self.sum_us += latency_us as u64;
        self.max_us = self.max_us.max(latency_us);
        if latency_us > 1500 {
            self.over_1500us += 1;
            log_warn!("I2C latency exceeded 1.5ms: {}us", latency_us);
        }
    }

    fn mean_us(&self) -> u32 {
        (self.sum_us / self.count as u64) as u32
    }
}
```

**Timeout Handling:**

```rust
// I2C read with timeout
async fn read_with_timeout<I: I2c>(
    i2c: &mut I,
    addr: u8,
    reg: u8,
    buf: &mut [u8],
) -> Result<(), ImuError> {
    match with_timeout(Duration::from_millis(2), async {
        i2c.write_read(addr, &[reg], buf).await
    }).await {
        Ok(Ok(())) => Ok(()),
        Ok(Err(_)) => Err(ImuError::I2cError),
        Err(_) => {
            log_error!("I2C read timeout");
            Err(ImuError::I2cError)
        }
    }
}
```

## External References

- [I2C Specification (NXP)](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) - I2C timing requirements
- [Embassy RP I2C](https://docs.embassy.dev/embassy-rp/git/rp2040/i2c/index.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
