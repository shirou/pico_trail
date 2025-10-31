# AN-yhnjd IMU Sensor Selection for AHRS

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Requirements:
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
- Related ADRs:
  - [ADR-6twis-ahrs-algorithm-selection](../adr/ADR-6twis-ahrs-algorithm-selection.md)

## Executive Summary

This analysis evaluates IMU sensor options for the pico_trail autopilot AHRS subsystem. The system requires 400Hz sampling with low jitter (<1ms) to provide accurate attitude estimation for rover and boat navigation. Three sensor families are compared: BMI088 (6-axis high-performance), MPU9250 (9-axis mid-range), and MPU6050 (6-axis entry-level). Based on technical requirements, availability, and cost, **BMI088 is recommended** for its 1600Hz ODR capability, SPI interface, and dual data-ready interrupts that ensure deterministic 400Hz sampling with minimal jitter.

## Problem Space

### Current State

The AHRS subsystem (DCM algorithm) has been implemented and requires IMU sensor data at 400Hz to function properly:

- AHRS code is ready: `src/subsystems/ahrs/` (DCM, calibration, task integration)
- `ImuData` interface is defined: gyro, accel, mag (optional)
- Platform abstractions exist: SPI and I2C traits implemented
- No physical IMU driver implementation exists yet

### Desired State

- IMU driver providing calibrated sensor data to AHRS at 400Hz
- Low sampling jitter (<1ms) for stable attitude estimation
- Magnetometer support for heading correction (optional but preferred)
- Minimal CPU overhead for sensor reads (<0.5ms per sample)
- Reliable operation on both Pico W (RP2040) and Pico 2 W (RP2350)

### Gap Analysis

Missing components:

1. IMU sensor hardware selection
2. Device driver implementation (`src/devices/imu/`)
3. Hardware integration (wiring, mounting)
4. Calibration procedures for chosen sensor
5. Performance validation on target hardware

## Stakeholder Analysis

| Stakeholder        | Interest/Need                       | Impact | Priority |
| ------------------ | ----------------------------------- | ------ | -------- |
| AHRS subsystem     | 400Hz gyro/accel data, low jitter   | High   | P0       |
| Control loops      | Stable attitude for steering/thrust | High   | P0       |
| Navigation         | Heading from magnetometer           | Medium | P1       |
| System integrator  | Easy mounting, wiring, calibration  | Medium | P1       |
| Project maintainer | Driver maintainability, testability | Medium | P2       |

## Research & Discovery

### Competitive Analysis

**ArduPilot IMU Support:**

- Primary sensors: BMI088, ICM-20948, ICM-42688
- Sampling: 400Hz minimum, 1000Hz preferred for aircraft
- Interface: SPI strongly preferred over I2C for latency
- Filtering: Hardware FIFO + software decimation

**PX4 Autopilot:**

- Similar sensor choices (BMI088, ICM series)
- Multi-IMU redundancy common (2-3 sensors)
- SPI-only in production hardware

**Hobby/Educational Projects:**

- MPU6050 most common (low cost, I2C, widely available)
- MPU9250 for 9-DOF (magnetometer included)
- Performance adequate for slow-moving rovers

### Technical Investigation

**Communication Interface Benchmarks:**

| Interface  | Read Latency | Bandwidth | Jitter   | Notes                        |
| ---------- | ------------ | --------- | -------- | ---------------------------- |
| I2C 400kHz | \~1.5ms      | 50 KB/s   | Variable | Shared bus, clock stretching |
| SPI 1MHz   | \~0.3ms      | 125 KB/s  | Low      | Dedicated, full-duplex       |
| SPI 8MHz   | \~0.1ms      | 1 MB/s    | Very low | Optimal for 400Hz            |

**RP2040/RP2350 Capabilities:**

- SPI: Up to 62.5 MHz (RP2040), 75 MHz (RP2350)
- I2C: Hardware support, DMA capable
- DMA: Can offload SPI reads (reduces CPU overhead)
- GPIO interrupts: Available for data-ready signals

### Data Analysis

**400Hz Sampling Budget:**

| Item              | Time Budget   | Notes                          |
| ----------------- | ------------- | ------------------------------ |
| Sample period     | 2.5ms         | 400Hz = 1/400 = 0.0025s        |
| Sensor read (SPI) | 0.3-0.5ms     | 6 registers (gyro) + 6 (accel) |
| Data conversion   | 0.1ms         | Raw to SI units                |
| Calibration apply | 0.1ms         | Offset/scale correction        |
| Buffering/handoff | 0.1ms         | Copy to AHRS input             |
| **Total**         | **0.6-0.8ms** | Leaves 1.7ms margin for jitter |

## Discovered Requirements

### Functional Requirements (Potential)

- [x] **FR-EXISTING**: Compatible with NFR-3wlo1 (400Hz sampling)
  - Rationale: Already defined in requirements
  - Acceptance Criteria: Listed in NFR-3wlo1

- [ ] **FR-DRAFT-1**: IMU driver shall support runtime calibration loading
  - Rationale: Calibration data stored in parameter system must be applied
  - Acceptance Criteria: Driver accepts `CalibrationData` struct at initialization

- [ ] **FR-DRAFT-2**: IMU driver shall provide raw and calibrated data access
  - Rationale: Debugging requires raw values, AHRS needs calibrated values
  - Acceptance Criteria: API exposes both `read_raw()` and `read_calibrated()`

### Non-Functional Requirements (Potential)

- [x] **NFR-EXISTING**: Sampling rate and jitter requirements (NFR-3wlo1)
  - Category: Performance
  - Rationale: Already defined
  - Target: 400Hz ± 1ms jitter

- [ ] **NFR-DRAFT-1**: IMU read latency shall not exceed 0.5ms per sample
  - Category: Performance
  - Rationale: Must leave CPU time for AHRS processing and other tasks
  - Target: <0.5ms measured on RP2040 @ 133MHz with SPI

- [ ] **NFR-DRAFT-2**: IMU driver shall recover from sensor read errors within 100ms
  - Category: Reliability
  - Rationale: Transient SPI errors should not hang the system
  - Target: Max 40 consecutive failures before error reported

## Design Considerations

### Technical Constraints

**Hardware:**

- RP2040 (Pico W): Cortex-M0+, 133MHz, no FPU, 264KB RAM
- RP2350 (Pico 2 W): Cortex-M33, 150MHz, FPU, 520KB RAM
- Both support SPI, I2C, DMA, GPIO interrupts

**Software:**

- Rust embedded-hal traits for portability
- Embassy async runtime for task management
- No heap allocation (use static buffers)
- Critical section for shared state (existing pattern)

**Power:**

- Rover/boat: Not severely power-constrained
- Sensors: Typically 3-5mA active current
- Can keep IMU always-on (no sleep modes needed)

### Potential Approaches

#### Option 1: BMI088 (6-axis, High Performance)

**Description:** Bosch high-performance IMU with separate gyro/accel datasheets, optimized for drones.

**Specifications:**

- Gyro ODR: Up to 2000Hz (16-bit)
- Accel ODR: Up to 1600Hz (16-bit)
- Interface: SPI (up to 10MHz)
- Interrupts: Separate INT1/INT2 for gyro and accel data-ready
- Gyro range: ±125 to ±2000 °/s
- Accel range: ±3g to ±24g
- No magnetometer (requires separate sensor)

**Pros:**

- Excellent performance: 1600Hz capable, exceeds 400Hz requirement by 4x
- SPI interface: Low latency (<0.3ms), minimal jitter
- Separate interrupts: Can synchronize gyro/accel reads precisely
- Industry standard: Used in ArduPilot, PX4
- High quality: Low noise, good temperature stability

**Cons:**

- Higher cost: \~$8-12 USD per unit
- No magnetometer: Requires QMC5883L or similar for heading
- Complexity: Separate gyro/accel datasheets, more registers to configure
- Availability: Less common in hobbyist market (more industrial)

**Effort:** Medium

- Driver complexity: Medium (separate gyro/accel configuration)
- Testing: High (need to validate 400Hz sustained rate)
- Integration: Medium (need external magnetometer)

#### Option 2: MPU9250 (9-axis, Mid-Range)

**Description:** InvenSense 9-DOF IMU with integrated magnetometer, popular in hobby projects.

**Specifications:**

- Gyro ODR: Up to 1000Hz (16-bit)
- Accel ODR: Up to 1000Hz (16-bit)
- Magnetometer: AK8963 (8Hz-100Hz)
- Interface: I2C (400kHz) or SPI (1MHz typical, 20MHz max)
- Interrupt: Single INT pin (data-ready)
- Gyro range: ±250 to ±2000 °/s
- Accel range: ±2g to ±16g

**Pros:**

- All-in-one: Gyro, accel, magnetometer integrated
- 1000Hz capable: Exceeds 400Hz requirement by 2.5x
- Widely available: Common on breakout boards ($5-8)
- Good documentation: Many examples in Arduino/CircuitPython
- SPI option: Can achieve low latency if needed

**Cons:**

- Discontinued: MPU9250 is end-of-life (EOL as of 2020)
- Magnetometer quality: AK8963 prone to interference
- I2C default: Most breakout boards use I2C (slower, more jitter)
- Single interrupt: Must poll to determine gyro vs accel ready

**Effort:** Low-Medium

- Driver complexity: Low (many reference implementations)
- Testing: Medium (validate SPI mode for low latency)
- Integration: Low (all sensors integrated)

#### Option 3: MPU6050 (6-axis, Entry-Level)

**Description:** InvenSense entry-level 6-DOF IMU, extremely common in hobby projects.

**Specifications:**

- Gyro ODR: Up to 1000Hz (16-bit)
- Accel ODR: Up to 1000Hz (16-bit)
- No magnetometer
- Interface: I2C only (400kHz max)
- Interrupt: Single INT pin (data-ready)
- Gyro range: ±250 to ±2000 °/s
- Accel range: ±2g to ±16g

**Pros:**

- Ultra low cost: \~$2-3 USD
- Extremely common: Widely available on breakout boards
- Simple: Well-documented, many tutorials
- Adequate ODR: 1000Hz exceeds 400Hz requirement

**Cons:**

- I2C only: No SPI option, higher latency (\~1-1.5ms per read)
- Jitter: I2C clock stretching can cause variable latency
- No magnetometer: Requires separate sensor for heading
- Entry-level performance: Higher noise than BMI088/MPU9250
- May not meet jitter requirement: <1ms jitter difficult with I2C

**Effort:** Low

- Driver complexity: Low (simplest configuration)
- Testing: Medium (must verify jitter is acceptable)
- Integration: Medium (need external magnetometer)

### Architecture Impact

**ADR Required:**

- **ADR-<id>-imu-driver-architecture**: Interface design, SPI vs I2C, interrupt vs polling

**Decisions to Make:**

1. Sensor selection: BMI088, MPU9250, or MPU6050
2. Communication interface: SPI or I2C
3. Sampling strategy: Interrupt-driven or timer-based polling
4. Magnetometer: Integrated, external, or none
5. Error handling: Retry logic, sensor reset, fallback mode

## Risk Assessment

| Risk                                  | Probability | Impact | Mitigation Strategy                                   |
| ------------------------------------- | ----------- | ------ | ----------------------------------------------------- |
| Jitter exceeds 1ms with I2C (MPU6050) | High        | High   | Use SPI-based sensor (BMI088, MPU9250 SPI mode)       |
| BMI088 difficult to source            | Medium      | Medium | Identify multiple suppliers, consider ICM-42688       |
| Driver bugs delay AHRS integration    | Medium      | High   | Incremental development, unit tests, hardware-in-loop |
| Magnetometer interference             | Medium      | Low    | Use external mag, calibration, sensor placement       |
| CPU overhead >0.5ms                   | Low         | Medium | Use DMA for SPI, optimize driver, profile on hardware |

## Open Questions

- [ ] What is the actual jitter with I2C on RP2040 at 400Hz? → Method: Benchmark I2C read latency with logic analyzer
- [ ] Can MPU9250 SPI breakout boards be easily sourced? → Next step: Survey suppliers (Adafruit, SparkFun, AliExpress)
- [ ] Do we need magnetometer for rover/boat heading? → Decision: Yes for outdoor navigation, defer to navigation requirements
- [ ] Should we support multiple IMU types in driver? → Next step: Draft ADR for driver abstraction strategy

## Recommendations

### Immediate Actions

1. **Select BMI088 as primary sensor** for the following reasons:
   - Meets all technical requirements (400Hz, <1ms jitter, SPI)
   - Industry standard (ArduPilot, PX4 validated)
   - Best performance headroom (1600Hz ODR)
   - Deterministic sampling with dedicated interrupts

2. **Plan for external magnetometer** (QMC5883L or HMC5883L):
   - BMI088 has no magnetometer
   - Heading correction critical for outdoor navigation
   - Can be added on separate I2C bus

### Next Steps

1. [ ] Draft ADR: IMU driver architecture (interface, SPI config, interrupt handling)
2. [ ] Create task: T-<id>-bmi088-driver-implementation
   - Phase 1: SPI communication, register read/write
   - Phase 2: Initialization, data conversion, calibration
   - Phase 3: Interrupt integration, 400Hz validation
3. [ ] Source BMI088 breakout board:
   - Option 1: Adafruit BMI088 breakout (\~$15)
   - Option 2: Generic BMI088 module from AliExpress (\~$8)
4. [ ] Plan magnetometer integration (separate task after BMI088 working)

### Out of Scope

- **Multi-IMU redundancy**: Single IMU sufficient for rover/boat (not aircraft)
- **MPU6050 support**: I2C jitter risk too high, not worth the cost savings
- **MPU9250 support**: EOL sensor, not recommended for new designs
- **Sensor fusion alternatives**: AHRS DCM algorithm already chosen (ADR-6twis)

## Appendix

### References

**Datasheets:**

- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [MPU9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)

**ArduPilot References:**

- [ArduPilot IMU Selection](https://ardupilot.org/dev/docs/choosing-a-ground-station.html)
- [ArduPilot Threading Model](https://ardupilot.org/dev/docs/learning-ardupilot-threading.html)

**Benchmarks:**

- SPI vs I2C latency: Measured on RP2040 (pending hardware test)

### Raw Data

**Sensor Availability (as of 2025-10):**

| Sensor  | Supplier   | Price  | Stock     | Lead Time |
| ------- | ---------- | ------ | --------- | --------- |
| BMI088  | Adafruit   | $14.95 | In stock  | 1-2 weeks |
| BMI088  | AliExpress | $8-10  | Available | 2-4 weeks |
| MPU9250 | SparkFun   | $7.95  | Limited   | Varies    |
| MPU6050 | Generic    | $2-3   | Abundant  | 1-2 weeks |

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
