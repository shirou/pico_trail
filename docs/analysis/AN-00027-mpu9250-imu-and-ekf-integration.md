# AN-00027 9-Axis IMU and EKF Integration

## Metadata

- Type: Analysis
- Status: Complete

## Change History

- **2025-01-XX**: Updated to include ICM-20948 as primary sensor option due to MPU-9250 unavailability. MPU-9250 implementation retained for reference.

## Links

- Related Analyses:
  - ~~[AN-00005-imu-sensor-selection](../analysis/AN-00005-imu-sensor-selection.md)~~ (Deprecated - replaced by this analysis)
- Related Requirements:
  - [FR-00105-icm20948-i2c-driver](../requirements/FR-00105-icm20948-i2c-driver.md)
  - [FR-00103-mpu9250-i2c-driver](../requirements/FR-00103-mpu9250-i2c-driver.md)
  - [FR-00101-imu-sensor-trait](../requirements/FR-00101-imu-sensor-trait.md)
  - [NFR-00002-imu-sampling-rate](../requirements/NFR-00002-imu-sampling-rate.md)
  - [FR-00001-ahrs-attitude-estimation](../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [FR-00102-large-vehicle-magcal](../requirements/FR-00102-large-vehicle-magcal.md)
- Related ADRs:
  - [ADR-00026-mpu9250-i2c-driver-architecture](../adr/ADR-00026-mpu9250-i2c-driver-architecture.md)
  - [ADR-00025-ekf-ahrs-implementation](../adr/ADR-00025-ekf-ahrs-implementation.md)
  - ~~[ADR-00001-ahrs-algorithm-selection](../adr/ADR-00001-ahrs-algorithm-selection.md)~~ (Superseded)
  - ~~[ADR-00006-imu-driver-architecture](../adr/ADR-00006-imu-driver-architecture.md)~~ (Deprecated)
- Related Tasks:
  - [T-00024-icm20948-driver-implementation](../tasks/T-00024-icm20948-driver-implementation/README.md) (Primary)
  - [T-00023-mpu9250-driver-implementation](../tasks/T-00023-mpu9250-driver-implementation/README.md) (Cancelled)
  - [T-00022-ekf-ahrs-implementation](../tasks/T-00022-ekf-ahrs-implementation/README.md)

## Executive Summary

This analysis evaluates the integration of 9-axis IMU sensors with an Extended Kalman Filter (EKF) for attitude estimation in the pico_trail autopilot. The supported sensors (ICM-20948, MPU-9250) provide gyroscope, accelerometer, and magnetometer data that feeds into the EKF to produce accurate attitude (roll, pitch, yaw) estimates. The system architecture emphasizes hardware abstraction to support multiple sensors, global state management for sharing EKF outputs, MAVLink telemetry for GCS communication, and comprehensive sensor calibration procedures.

**Key Decisions:**

- **Primary Sensor**: ICM-20948 (9-axis IMU, MPU-9250 successor, actively produced)
- **Backup Sensor**: MPU-9250 (9-axis IMU, EOL but implementation retained)
- **Interface**: I2C (simpler wiring, adequate for 400Hz sampling)
- **Algorithm**: EKF (7-state quaternion-based filter) replaces DCM
- **State Management**: Global `ATTITUDE_STATE` accessible by navigation and control subsystems
- **Telemetry**: MAVLink `ATTITUDE`, `ATTITUDE_QUATERNION` messages at 10Hz

## Problem Space

### Current State

The pico_trail autopilot has:

- AHRS subsystem with DCM algorithm implemented (`src/subsystems/ahrs/`)
- `ImuData` interface defined for gyro, accel, mag data
- Platform abstractions for SPI and I2C traits
- GPS state management pattern (`GPS_STATE`) that can be reused
- MAVLink communication infrastructure in place
- No physical IMU driver implementation exists yet

Previous sensor selection (AN-00005) recommended BMI088 with SPI, but project constraints and hardware availability now favor 9-axis IMU sensors with I2C. ICM-20948 is the primary choice as the actively-produced successor to MPU-9250. The MPU-9250 implementation is retained for cases where that hardware is available.

### Desired State

- IMU drivers (ICM-20948, MPU-9250) providing 9-axis sensor data via abstracted `ImuSensor` trait
- EKF-based attitude estimation providing accurate roll, pitch, yaw
- Global `ATTITUDE_STATE` accessible by:
  - Navigation controller for heading-based steering
  - MAVLink telemetry for GCS display
  - Logging subsystem for flight data recording
- Comprehensive calibration system for gyro bias, accel offsets/scales, and magnetometer hard/soft iron
- MAVLink integration sending attitude data to Mission Planner/QGroundControl

### Gap Analysis

| Component            | Current State             | Required State                                    |
| -------------------- | ------------------------- | ------------------------------------------------- |
| IMU Driver           | MPU-9250 partial (Draft)  | ICM-20948 + MPU-9250 I2C drivers with abstraction |
| Attitude Algorithm   | DCM (implemented)         | EKF (7-state quaternion filter)                   |
| State Management     | None for attitude         | Global ATTITUDE_STATE with mutex                  |
| MAVLink Attitude     | Not implemented           | ATTITUDE message at 10Hz                          |
| Calibration          | Structures defined        | Full calibration procedure                        |
| Hardware Abstraction | Partial (ImuSensor trait) | Complete ImuSensor trait with multiple impl       |

## Stakeholder Analysis

| Stakeholder           | Interest/Need                            | Impact | Priority |
| --------------------- | ---------------------------------------- | ------ | -------- |
| Navigation System     | Accurate heading for waypoint navigation | High   | P0       |
| Control Loops         | Stable attitude for steering/thrust      | High   | P0       |
| GCS (Mission Planner) | Real-time attitude display               | Medium | P1       |
| Logging System        | Attitude data for post-flight analysis   | Medium | P1       |
| Calibration User      | Simple, reliable calibration procedure   | Medium | P1       |
| System Integrator     | Easy mounting, wiring, and configuration | Medium | P2       |

## Research & Discovery

### Competitive Analysis

**ArduPilot EKF Implementation:**

- EKF2/EKF3 with 24+ states for full navigation
- Simplified 7-state AHRS EKF for attitude-only estimation
- Supports multiple IMU instances for redundancy
- MPU-9250 widely used in ArduPilot-compatible boards (BBBMINI, Navio2)

**PX4 Autopilot:**

- Uses similar EKF approach with quaternion representation
- Attitude Estimator Q (simplified quaternion EKF)
- Sensor fusion with GPS velocity for heading correction

**Embedded AHRS Libraries:**

- Madgwick/Mahony filters: Simple but less accurate than EKF
- STM32 MPU9250 EKF: 7-state EKF with quaternion + gyro bias

### Technical Investigation

**ICM-20948 Specifications (Primary Sensor):**

| Parameter         | Value                             | Notes                             |
| ----------------- | --------------------------------- | --------------------------------- |
| Gyroscope ODR     | Up to 9000Hz                      | 1000Hz sufficient for 400Hz AHRS  |
| Accelerometer ODR | Up to 4500Hz                      | 1000Hz sufficient                 |
| Magnetometer ODR  | Up to 100Hz                       | AK09916 integrated                |
| Interface         | I2C (400kHz) or SPI (7MHz)        | I2C simpler wiring                |
| Gyro Range        | ±250, ±500, ±1000, ±2000°/s       | ±2000°/s for fast maneuvers       |
| Accel Range       | ±2, ±4, ±8, ±16g                  | ±8g for rover/boat                |
| Mag Range         | ±4900µT                           | Fixed range (AK09916)             |
| Supply Voltage    | VDD: 1.71-3.6V, VDDIO: 1.71-1.95V | Requires level shifter or 1.8V IO |
| WHO_AM_I          | 0xEA                              | Different from MPU-9250 (0x71)    |
| I2C Address       | 0x69 (default), 0x68              | AD0 jumper selectable             |
| Status            | Active production                 | MPU-9250 successor                |

**MPU-9250 Specifications (Backup Sensor, EOL):**

| Parameter         | Value                       | Notes                              |
| ----------------- | --------------------------- | ---------------------------------- |
| Gyroscope ODR     | Up to 8000Hz                | 1000Hz sufficient for 400Hz AHRS   |
| Accelerometer ODR | Up to 4000Hz                | 1000Hz sufficient                  |
| Magnetometer ODR  | Up to 100Hz                 | AK8963 integrated                  |
| Interface         | I2C (400kHz) or SPI (1MHz)  | I2C simpler wiring                 |
| Gyro Range        | ±250, ±500, ±1000, ±2000°/s | ±2000°/s for fast maneuvers        |
| Accel Range       | ±2, ±4, ±8, ±16g            | ±8g for rover/boat                 |
| Mag Range         | ±4800µT                     | Fixed range (AK8963)               |
| Supply Voltage    | 2.4V - 3.6V                 | 3.3V compatible                    |
| WHO_AM_I          | 0x71                        | Different from ICM-20948 (0xEA)    |
| I2C Address       | 0x68 (default), 0x69        | AD0 jumper selectable              |
| Status            | End of Life (EOL)           | Still available on breakout boards |

**ICM-20948 vs MPU-9250 Key Differences:**

| Aspect              | ICM-20948                | MPU-9250              |
| ------------------- | ------------------------ | --------------------- |
| Production Status   | Active                   | EOL                   |
| Register Banks      | 4 banks (bank switching) | Single register space |
| Magnetometer        | AK09916                  | AK8963                |
| Default I2C Address | 0x69                     | 0x68                  |
| WHO_AM_I            | 0xEA                     | 0x71                  |
| VDDIO               | 1.71-1.95V (separate)    | Same as VDD           |
| Code Compatibility  | Different register map   | —                     |

**I2C vs SPI Trade-offs:**

| Aspect      | I2C                          | SPI                                      |
| ----------- | ---------------------------- | ---------------------------------------- |
| Wiring      | 4 wires (VCC, GND, SDA, SCL) | 6+ wires (VCC, GND, MOSI, MISO, CLK, CS) |
| Latency     | \~1.5ms per full read        | \~0.3ms per full read                    |
| Jitter      | Medium (clock stretching)    | Low                                      |
| Complexity  | Lower (shared bus)           | Higher (dedicated pins)                  |
| Suitability | Adequate for 400Hz           | Optimal for >1000Hz                      |

**Decision**: I2C is adequate for 400Hz IMU sampling and simplifies wiring/integration.

**EKF State Definition (7-state):**

```
State Vector x = [q0, q1, q2, q3, bx, by, bz]
- q0, q1, q2, q3: Quaternion (attitude)
- bx, by, bz: Gyroscope bias (rad/s)

Measurements z = [ax, ay, az, mx, my, mz]
- ax, ay, az: Accelerometer (m/s²)
- mx, my, mz: Magnetometer (µT)
```

### Calibration Research

**Gyroscope Calibration:**

- Bias estimation: Average readings over 1-2 seconds while stationary
- Temperature compensation: Optional, requires characterization
- No scale factor calibration needed (factory calibrated)

**Accelerometer Calibration (6-position):**

1. Place sensor flat (Z up): Measure gravity on Z-axis
2. Flip upside down (Z down): Measure gravity on -Z
3. Repeat for X and Y axes (4 more positions)
4. Compute offset and scale for each axis

**Magnetometer Calibration (Hard/Soft Iron):**

1. Rotate sensor in all orientations (figure-8 or sphere)
2. Collect min/max values for each axis
3. Hard iron: Offset = (max + min) / 2
4. Soft iron: Scale = (max - min) / 2, normalize to sphere
5. Store as 3x3 transformation matrix + offset vector

**Calibration Storage:**

- Store calibration data in parameter system (existing infrastructure)
- Load at startup, apply to raw readings before EKF input
- Parameters: `ACCEL_OFFSETS`, `ACCEL_SCALES`, `MAG_OFFSETS`, `MAG_SCALES`, `MAG_DECLINATION`

### Data Analysis

**400Hz Sampling Budget with I2C:**

| Item              | Time Budget   | Notes                      |
| ----------------- | ------------- | -------------------------- |
| Sample period     | 2.5ms         | 400Hz = 1/400 = 0.0025s    |
| I2C read (all 9)  | 1.0-1.5ms     | 14 registers @ 400kHz I2C  |
| Data conversion   | 0.1ms         | Raw to SI units            |
| Calibration apply | 0.1ms         | Offset/scale correction    |
| EKF predict       | 0.2ms         | State propagation          |
| EKF update        | 0.3ms         | Measurement update (100Hz) |
| **Total**         | **1.7-2.2ms** | Leaves 0.3-0.8ms margin    |

Note: EKF update runs at 100Hz (every 4th IMU sample), not 400Hz.

## Discovered Requirements

### Functional Requirements (Potential)

- [x] **FR-EXISTING**: Compatible with FR-00001 (AHRS attitude estimation)
  - Rationale: Already defined in requirements
  - Acceptance Criteria: Listed in FR-00001

- [ ] **FR-DRAFT-1**: IMU driver shall provide calibrated 9-axis sensor data
  - Rationale: EKF requires calibrated gyro, accel, and mag data
  - Acceptance Criteria: `read_calibrated()` returns data with offsets/scales applied

- [ ] **FR-DRAFT-2**: EKF shall estimate quaternion attitude and gyro bias
  - Rationale: Quaternion avoids gimbal lock, gyro bias improves accuracy
  - Acceptance Criteria: 7-state EKF outputs quaternion (4) + gyro bias (3)

- [ ] **FR-DRAFT-3**: Global attitude state shall be accessible by all subsystems
  - Rationale: Navigation, control, and telemetry need attitude data
  - Acceptance Criteria: `ATTITUDE_STATE.lock().get_euler()` returns roll/pitch/yaw

- [ ] **FR-DRAFT-4**: MAVLink ATTITUDE message shall be sent at configurable rate
  - Rationale: GCS requires attitude telemetry for display and logging
  - Acceptance Criteria: ATTITUDE message sent at 10Hz (configurable)

- [ ] **FR-DRAFT-5**: Calibration procedure shall be executable via MAVLink command
  - Rationale: Field calibration without code changes
  - Acceptance Criteria: MAV_CMD_PREFLIGHT_CALIBRATION triggers calibration

### Non-Functional Requirements (Potential)

- [x] **NFR-EXISTING**: Sampling rate and jitter requirements (NFR-00002)
  - Category: Performance
  - Rationale: Already defined
  - Target: 400Hz ± 1ms jitter

- [ ] **NFR-DRAFT-1**: EKF update shall complete within 10ms (100Hz rate)
  - Category: Performance
  - Rationale: Must leave CPU time for control loops and other tasks
  - Target: <10ms per EKF cycle on RP2040 @ 133MHz

- [ ] **NFR-DRAFT-2**: Attitude accuracy shall be ±2° roll/pitch, ±5° heading
  - Category: Accuracy
  - Rationale: Required for reliable waypoint navigation
  - Target: Measured during static and slow-moving conditions

- [ ] **NFR-DRAFT-3**: EKF shall converge within 5 seconds of startup
  - Category: Performance
  - Rationale: Vehicle should be operational quickly after power-on
  - Target: Attitude accurate within 5 seconds

## Design Considerations

### Technical Constraints

**Hardware:**

- RP2040 (Pico W): Cortex-M0+, 133MHz, no FPU, 264KB RAM
- RP2350 (Pico 2 W): Cortex-M33, 150MHz, FPU, 520KB RAM
- I2C0 bus shared with GPS (if using I2C GPS)
- MPU-9250 mounted on vehicle with known orientation

**Software:**

- Rust embedded-hal traits for portability
- Embassy async runtime for task management
- No heap allocation (use static buffers)
- `nalgebra` for matrix/quaternion operations
- Critical section for shared state (existing pattern)

**Power:**

- MPU-9250: \~3.5mA active current
- Always-on operation (no sleep modes needed)

### Potential Approaches

#### Option 1: MPU-9250 + I2C + 7-State EKF (Recommended)

**Description:** MPU-9250 driver using I2C, feeding a 7-state quaternion-based EKF for attitude estimation.

**Specifications:**

- I2C @ 400kHz for sensor reading
- 400Hz IMU sampling, 100Hz EKF update
- 7-state EKF: quaternion (4) + gyro bias (3)
- Magnetometer fusion at 10Hz
- Global `ATTITUDE_STATE` for data sharing

**Pros:**

- Integrated 9-axis sensor simplifies hardware
- I2C simplifies wiring (4 wires vs 6+ for SPI)
- EKF provides better accuracy than DCM
- Gyro bias estimation improves long-term stability
- Quaternion avoids gimbal lock at extreme attitudes

**Cons:**

- I2C latency (\~1.5ms) higher than SPI (\~0.3ms)
- MPU-9250 is EOL (end-of-life), but still widely available
- EKF more complex than DCM to implement
- Higher memory usage (\~8KB vs \~2KB for DCM)

**Effort:** Medium

- Driver complexity: Medium (I2C, 9-axis, DMP optional)
- EKF complexity: Medium-High (quaternion math, covariance)
- Integration: Medium (state management, MAVLink)

#### Option 2: MPU-9250 + I2C + DCM (Keep Existing)

**Description:** Continue with MPU-9250 driver feeding existing DCM algorithm.

**Pros:**

- DCM already implemented (T-00005 complete)
- Lower memory usage (\~2KB)
- Simpler mathematics

**Cons:**

- Lower accuracy than EKF (±2-3° vs ±1-2°)
- No explicit covariance estimation
- Gyro bias drift requires separate compensation

**Effort:** Low (reuse existing DCM)

#### Option 3: ICM-20948 + I2C + EKF (Recommended)

**Description:** ICM-20948 driver using I2C, feeding a 7-state quaternion-based EKF for attitude estimation. This is the MPU-9250 successor and is actively produced.

**Specifications:**

- I2C @ 400kHz for sensor reading
- 400Hz IMU sampling, 100Hz EKF update
- 7-state EKF: quaternion (4) + gyro bias (3)
- Magnetometer fusion at 10Hz (AK09916)
- Global `ATTITUDE_STATE` for data sharing

**Pros:**

- Actively produced (not EOL like MPU-9250)
- Integrated 9-axis sensor simplifies hardware
- I2C simplifies wiring (4 wires vs 6+ for SPI)
- Better specs than MPU-9250 (higher ODR capability)
- Same EKF approach as MPU-9250 option

**Cons:**

- Different register map from MPU-9250 (not code-compatible)
- VDDIO requires 1.8V or level shifter on some breakouts
- Register bank switching adds complexity to driver

**Effort:** Medium (similar to MPU-9250, different register handling)

#### Option 4: MPU-9250 + SPI + EKF

**Description:** Use SPI interface for lower latency, combined with EKF.

**Pros:**

- Lower latency (<0.3ms vs 1.5ms)
- Better jitter characteristics
- Same EKF benefits

**Cons:**

- More complex wiring (6+ wires)
- Requires dedicated SPI pins (conflict with other peripherals?)
- Not significantly beneficial at 400Hz
- MPU-9250 is EOL

**Effort:** Medium-High (SPI driver + EKF)

### Option Analysis

- **Option 1 (MPU-9250 + I2C + EKF)** — Good balance of simplicity and capability, but sensor is EOL
- **Option 2 (MPU-9250 + DCM)** — Simpler, but lower accuracy and still EOL sensor
- **Option 3 (ICM-20948 + I2C + EKF)** — **Recommended**: Actively produced, better specs, same architecture
- **Option 4 (MPU-9250 + SPI + EKF)** — Overkill for 400Hz, added complexity, EOL sensor

### Architecture Impact

**ADR Required:**

- **ADR-00026-imu-i2c-driver-architecture**: I2C communication, multi-sensor support via ImuSensor trait
- **ADR-<id>-ekf-ahrs-implementation**: EKF state, tuning, convergence

**Decisions to Make:**

1. ~~Sensor selection~~ → ICM-20948 primary, MPU-9250 backup (decided)
2. ~~Communication interface~~ → I2C (decided)
3. Algorithm: EKF or keep DCM?
4. State sharing: Global static or message passing?
5. MAVLink rate: 10Hz or configurable?
6. Calibration: On-device or external tool?

### State Management Design

Following the GPS state pattern (`GPS_STATE`), attitude state will be:

```rust
pub struct AttitudeState {
    /// Quaternion representation (avoids gimbal lock)
    quaternion: Quaternion<f32>,
    /// Euler angles (roll, pitch, yaw in radians)
    euler: EulerAngles,
    /// Angular rates (rad/s)
    rates: Vector3<f32>,
    /// Estimated gyro bias (rad/s)
    gyro_bias: Vector3<f32>,
    /// EKF covariance diagonal (uncertainty)
    covariance: [f32; 7],
    /// Health/validity flags
    healthy: bool,
    /// Timestamp of last update (ms)
    timestamp_ms: u32,
}

pub static ATTITUDE_STATE: Mutex<CriticalSectionRawMutex, AttitudeState> =
    Mutex::new(AttitudeState::new());
```

### MAVLink Integration

**Messages to Send:**

1. `ATTITUDE` (ID 30): Roll, pitch, yaw, and rates at 10Hz
2. `ATTITUDE_QUATERNION` (ID 31): Full quaternion at 10Hz (optional)

**Messages to Receive:**

1. `MAV_CMD_PREFLIGHT_CALIBRATION` (ID 241): Trigger sensor calibration

## Risk Assessment

| Risk                                     | Probability | Impact  | Mitigation Strategy                                         |
| ---------------------------------------- | ----------- | ------- | ----------------------------------------------------------- |
| I2C jitter exceeds 1ms at 400Hz          | Medium      | Medium  | Optimize I2C driver, consider timer-based sampling          |
| ~~MPU-9250 sourcing (EOL product)~~      | ~~High~~    | ~~Med~~ | ~~Mitigated: ICM-20948 now primary sensor~~                 |
| ICM-20948 VDDIO voltage mismatch         | Low         | Medium  | Use breakout boards with level shifter (Adafruit, SparkFun) |
| EKF divergence during fast maneuvers     | Low         | High    | Tune process noise, implement divergence detection          |
| Magnetometer interference from motors    | Medium      | Medium  | Mount mag away from motors, calibration, GPS heading        |
| CPU insufficient for 100Hz EKF on RP2040 | Medium      | High    | Optimize matrix operations, reduce to 50Hz if needed        |
| I2C bus contention with GPS              | Medium      | Medium  | Use separate I2C buses or careful timing                    |
| ICM-20948 register bank complexity       | Low         | Low     | Well-documented bank switching, existing Rust crates        |

## Open Questions

- [ ] Should EKF run at 100Hz or 50Hz on RP2040? → Method: Profile CPU usage on hardware
- [ ] Do we need GPS velocity fusion for heading? → Decision: Defer to navigation integration
- [x] ~~Can we use MPU-9250 DMP for preprocessing?~~ → Decision: Not using DMP, EKF handles fusion
- [ ] What is acceptable I2C jitter on RP2040? → Method: Measure with logic analyzer
- [ ] Should calibration be interactive (user prompts) or automatic? → Decision: Start with interactive
- [x] ~~Is ICM-20948 a suitable backup if MPU-9250 unavailable?~~ → Decision: **ICM-20948 is now primary sensor**

## Recommendations

### Immediate Actions

1. **Select ICM-20948 + I2C + EKF** as the primary implementation approach:
   - Actively produced (MPU-9250 successor, not EOL)
   - 9-axis integration eliminates need for separate magnetometer
   - I2C simplifies wiring and is adequate for 400Hz
   - EKF provides better accuracy and explicit uncertainty estimation
   - Quaternion representation avoids gimbal lock

2. **Retain MPU-9250 implementation** as backup option:
   - Existing partial implementation in `src/devices/imu/mpu9250/`
   - Useful for users who have MPU-9250 hardware

3. **Implement abstracted IMU trait** (`ImuSensor`) to support multiple sensors:
   - Abstract over ICM-20948, MPU-9250, or other sensors
   - Enable mock implementation for testing

4. **Adopt GPS state management pattern** for attitude:
   - Global `ATTITUDE_STATE` with mutex protection
   - Consistent access from navigation, control, telemetry

### Next Steps

1. [x] ~~Draft ADR: MPU-9250 I2C driver architecture~~ → ADR-00026 (update for multi-sensor)
2. [ ] Draft ADR: EKF AHRS implementation (supersedes ADR-00001)
3. [ ] Create requirements: FR/NFR for EKF and attitude state
4. [x] ~~Create task: T-00023-mpu9250-driver-implementation~~ → Cancelled (MPU-9250 unavailable)
5. [ ] Create task: T-<id>-icm20948-driver-implementation
   - Phase 1: I2C communication, register bank access, raw data
   - Phase 2: Calibration, data conversion, ImuSensor trait
6. [ ] Create task: T-<id>-ekf-ahrs-implementation
   - Phase 1: EKF state, prediction, update functions
   - Phase 2: Magnetometer fusion, covariance tuning
   - Phase 3: Embassy task, state sharing, MAVLink
7. [x] ~~Source MPU-9250 modules~~ → ICM-20948 obtained
   - Primary: Adafruit ICM-20948 (STEMMA QT) \~$15
   - Alternative: SparkFun 9DoF IMU Breakout (Qwiic) \~$18
8. [ ] Plan calibration procedures and user interface

### Out of Scope

- **BMI088 support**: Previous decision (AN-00005) deprecated; focus on ICM-20948/MPU-9250
- **Multi-IMU redundancy**: Single IMU sufficient for rover/boat
- **Full navigation EKF**: 24-state EKF with GPS/baro deferred; 7-state AHRS sufficient initially
- **Temperature compensation**: Optional enhancement after basic EKF validated
- **DMP usage**: Not using ICM-20948/MPU-9250 DMP; EKF handles sensor fusion

## Appendix

### References

**ICM-20948 Resources (Primary Sensor):**

- [ICM-20948 Datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf)
- [ICM-20948 Product Page](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [Adafruit ICM-20948 Guide](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tdk-invensense-icm-20948-9-dof-imu.pdf)
- [SparkFun ICM-20948 Breakout](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

**MPU-9250 Resources (Backup Sensor, EOL):**

- [MPU-9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [MPU-9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [SparkFun MPU-9250 Hookup Guide](https://learn.sparkfun.com/tutorials/mpu-9250-hookup-guide/all)

**Calibration:**

- [MPU-9250 Calibration (GitHub)](https://github.com/makerportal/mpu92-calibration)
- [MPU-9250 ESP32 Calibration Library](https://github.com/kian2attari/MPU-9250-ESP32-Library-Calibration-EEPROM)
- [MATLAB Orientation Estimation with MPU-9250](https://www.mathworks.com/help/nav/ug/estimating-orientation-using-MPU-9250.html)

**EKF AHRS:**

- [STM32 MPU9250 EKF (GitHub)](https://github.com/suhetao/stm32f4_mpu9250)
- [Quaternion AHRS with MPU9250 and STM32](https://ibrahimcahitozdemir.com/2022/01/08/quaternion-based-ahrs-estimation-using-mpu9250-and-stm32g431/)
- [IMU AHRS Estimation (GitHub)](https://github.com/mschoder/imu-ahrs-estimation)
- [EKF-Based AHRS Accuracy Improvement (MDPI)](https://www.mdpi.com/1424-8220/20/14/4055)

**ArduPilot:**

- [BBBMINI with ArduPilot and Dual MPU-9250](https://diydrones.com/profiles/blogs/bbbmini-with-ardupilot-ekf2-and-dual-imu-mpu-9250)
- [ArduPilot EKF Documentation](https://ardupilot.org/dev/docs/extended-kalman-filter.html)

### Calibration Procedure Summary

**Gyroscope Calibration (Automatic at startup):**

1. Keep vehicle stationary for 2 seconds
2. Collect 200 samples (100Hz)
3. Compute mean of each axis → gyro bias
4. Store in `ATTITUDE_STATE.gyro_bias`

**Accelerometer Calibration (Interactive, 6-position):**

1. User places vehicle level (Z up) → record
2. User flips upside down (Z down) → record
3. User tilts left (Y up) → record
4. User tilts right (Y down) → record
5. User tips forward (X down) → record
6. User tips backward (X up) → record
7. Compute offsets and scales for each axis
8. Store in parameter system

**Magnetometer Calibration (Interactive, rotation):**

1. User rotates vehicle in all directions for 30 seconds
2. Collect continuous samples during rotation
3. Compute hard iron offsets (center of sphere)
4. Compute soft iron matrix (scale to sphere)
5. Store in parameter system
6. Set magnetic declination for location

### EKF Algorithm Outline

```
State: x = [q0, q1, q2, q3, bx, by, bz]^T

Prediction (400Hz):
  1. Read gyroscope: ω = [ωx, ωy, ωz]
  2. Subtract bias: ω_corrected = ω - [bx, by, bz]
  3. Quaternion derivative: q_dot = 0.5 * q ⊗ [0, ω_corrected]
  4. Integrate: q_new = q + q_dot * dt
  5. Normalize quaternion
  6. Propagate covariance: P = F * P * F^T + Q

Update (100Hz, every 4th sample):
  1. Read accelerometer: a = [ax, ay, az]
  2. Predicted gravity in body: g_pred = R(q)^T * [0, 0, 1]
  3. Innovation: y = a/|a| - g_pred
  4. Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
  5. State update: x = x + K * y
  6. Covariance update: P = (I - K * H) * P

Magnetometer Update (10Hz):
  1. Read magnetometer: m = [mx, my, mz]
  2. Apply calibration: m_cal = S * (m - offset)
  3. Predicted heading from q
  4. Heading innovation
  5. Similar Kalman update for yaw only
```
