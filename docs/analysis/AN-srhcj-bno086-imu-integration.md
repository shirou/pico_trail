# AN-srhcj BNO086 9-Axis IMU Integration

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
  - ~~[AN-yhnjd-imu-sensor-selection](../analysis/AN-yhnjd-imu-sensor-selection.md)~~ (Deprecated)
- Related Requirements:
  - [FR-z1fdo-imu-sensor-trait](../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
  - [ADR-ymkzt-ekf-ahrs-implementation](../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Related Tasks:
  - ~~[T-0kbo4-icm20948-driver-implementation](../tasks/T-0kbo4-icm20948-driver-implementation/README.md)~~ (Cancelled - counterfeit hardware)
  - ~~[T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)~~ (Cancelled)

## Executive Summary

This analysis evaluates the BNO086 9-axis IMU as the new primary sensor for attitude estimation in the pico_trail autopilot. After encountering counterfeit ICM-20948 hardware (with non-functional magnetometer), the BNO086 is selected as a more reliable alternative. Unlike raw IMU sensors, the BNO086 features an integrated sensor fusion processor that outputs pre-computed quaternion (Rotation Vector) data, eliminating the need for external EKF implementation.

**Key Characteristics:**

- **On-chip Sensor Fusion**: BNO086 provides Rotation Vector output (quaternion) directly
- **Protocol**: SHTP (Sensor Hub Transport Protocol) over I2C
- **Target Output**: Quaternion at 100Hz
- **Reliability**: Hardware reset capability via RST pin for error recovery
- **Efficiency**: Interrupt-driven design (no polling) using INT pin

## Problem Space

### Current State

The pico_trail autopilot has experienced multiple IMU sensor issues:

1. **MPU-9250**: End-of-life (EOL), discontinued
2. **ICM-20948**: Purchased hardware was counterfeit; magnetometer (AK09916) non-functional

Existing implementation status:

- EKF AHRS design in progress (`ADR-ymkzt-ekf-ahrs-implementation`)
- `ImuSensor` trait defined (`src/devices/traits/imu.rs`)
- ICM-20948 driver code exists (`src/devices/imu/icm20948/`) but unusable due to counterfeit hardware
- MPU-9250 driver code exists (`src/devices/imu/mpu9250/`) but sensor unavailable

### Desired State

- Reliable 9-axis IMU providing accurate attitude estimation
- Quaternion output at 100Hz for navigation and control
- Robust error recovery for long-term drone/rover operation
- Efficient async design using interrupt-driven data acquisition
- Simple integration without external sensor fusion complexity

### Gap Analysis

| Component        | Current State                   | Required State                    |
| ---------------- | ------------------------------- | --------------------------------- |
| IMU Hardware     | ICM-20948 counterfeit, unusable | BNO086 genuine sensor             |
| Sensor Fusion    | EKF planned but incomplete      | On-chip fusion (BNO086 internal)  |
| Communication    | Standard I2C register access    | SHTP protocol implementation      |
| Error Recovery   | None                            | RST pin hardware reset on timeout |
| Data Acquisition | Polling or timer-based          | Interrupt-driven (INT pin)        |

## Stakeholder Analysis

| Stakeholder           | Interest/Need                             | Impact | Priority |
| --------------------- | ----------------------------------------- | ------ | -------- |
| Navigation System     | Accurate heading for waypoint navigation  | High   | P0       |
| Control Loops         | Stable attitude for steering/thrust       | High   | P0       |
| System Reliability    | Long-term operation without manual resets | High   | P0       |
| GCS (Mission Planner) | Real-time attitude display                | Medium | P1       |
| System Integrator     | Simple wiring and configuration           | Medium | P1       |

## Research & Discovery

### Competitive Analysis

**BNO086 vs Raw IMU Sensors:**

| Aspect          | BNO086                       | ICM-20948/MPU-9250      |
| --------------- | ---------------------------- | ----------------------- |
| Sensor Fusion   | On-chip (ARM Cortex-M0+)     | External (host CPU)     |
| Output          | Quaternion, Euler, etc.      | Raw gyro/accel/mag      |
| CPU Load (host) | Very low (read results only) | High (EKF computation)  |
| Protocol        | SHTP (complex)               | Simple register access  |
| Calibration     | Automatic (internal)         | Manual (user procedure) |
| Error Recovery  | RST pin + self-recovery      | Limited                 |
| Price           | Higher (\~$20-30)            | Lower (\~$5-15)         |

**ArduPilot BNO08x Support:**

- ArduPilot added BNO08x support in recent versions
- Uses SHTP protocol with sensor fusion outputs
- Preferred for applications where CPU is constrained

### Technical Investigation

**BNO086 Specifications:**

| Parameter            | Value                    | Notes                     |
| -------------------- | ------------------------ | ------------------------- |
| Processor            | ARM Cortex-M0+ (32-bit)  | On-chip sensor fusion     |
| Gyroscope Range      | ±2000°/s                 | 16-bit resolution         |
| Accelerometer Range  | ±8g                      | 16-bit resolution         |
| Magnetometer Range   | ±1300µT                  | Integrated                |
| Rotation Vector Rate | Up to 400Hz              | Quaternion output         |
| Interface            | I2C (400kHz), SPI, UART  | I2C with clock stretching |
| I2C Address          | 0x4A (default), 0x4B     | SA0 pin selectable        |
| INT Pin              | Active low, falling edge | Data ready indication     |
| RST Pin              | Active low               | Hardware reset            |
| Supply Voltage       | 2.4V - 3.6V              | 3.3V compatible           |
| Current (active)     | \~10mA                   | Higher than raw sensors   |

**SHTP Protocol Overview:**

The Sensor Hub Transport Protocol (SHTP) is a packet-based protocol used by Hillcrest Labs/CEVA sensors:

1. **Header (4 bytes)**: Contains payload length and channel information
2. **Payload**: Variable length data (reports, commands, responses)
3. **Channels**: Different channels for control, input reports, output reports

```
SHTP Packet Structure:
+--------+--------+--------+--------+------------------+
| Len_L  | Len_H  | Chan   | SeqNum |     Payload      |
+--------+--------+--------+--------+------------------+
  Byte 0   Byte 1   Byte 2   Byte 3      N bytes
```

**Data Flow:**

```
          ┌──────────────────────────────────────────┐
          │              BNO086 Sensor               │
          │  ┌─────────────────────────────────────┐ │
          │  │  Gyro + Accel + Mag (raw sensors)  │ │
          │  └──────────────┬──────────────────────┘ │
          │                 │                        │
          │  ┌──────────────▼──────────────────────┐ │
          │  │   ARM Cortex-M0+ Sensor Fusion      │ │
          │  │   - Rotation Vector (Quaternion)    │ │
          │  │   - Game Rotation Vector            │ │
          │  │   - Gravity Vector                  │ │
          │  │   - Linear Acceleration             │ │
          │  └──────────────┬──────────────────────┘ │
          │                 │ INT (falling edge)     │
          └─────────────────┼────────────────────────┘
                            │
           ┌────────────────▼────────────────────────┐
           │              RP2350 Host                │
           │  1. await INT falling edge              │
           │  2. I2C read header (4 bytes)           │
           │  3. I2C read payload (N bytes)          │
           │  4. Parse Rotation Vector               │
           │  5. Update ATTITUDE_STATE               │
           └─────────────────────────────────────────┘
```

**Rotation Vector Report (ID 0x05):**

```
Report Structure (14 bytes total):
+--------+--------+--------+--------+--------+--------+--------+
| RptID  | SeqNum |Status  | Delay  |  Qi    |  Qj    |  Qk    |
+--------+--------+--------+--------+--------+--------+--------+
   0x05     Seq     Stat   Delay_L/H  Q_i     Q_j      Q_k

+--------+--------+--------+
| Qreal  | Accuracy | (pad)|
+--------+--------+--------+
  Q_real   Acc_rad

Quaternion Format: Q16 fixed-point (divide by 2^14 for float)
Accuracy: radians (Q12 fixed-point)
```

**Initialization Sequence:**

1. Hardware reset via RST pin (low pulse > 10ms)
2. Wait for INT assertion (boot complete indication)
3. Read and discard initial reports
4. Send "Set Feature Command" to enable Rotation Vector at 100Hz
5. Enter main loop: await INT → read → parse → update state

**Error Recovery Strategy:**

For long-term operation (drones, rovers), robust error handling is critical:

```
Error Recovery Flow:
┌────────────────────────────────────────────────┐
│                 Normal Operation               │
│        await INT (with 500ms timeout)          │
└─────────────────────┬──────────────────────────┘
                      │
          ┌───────────┴───────────┐
          │                       │
   INT received             Timeout (500ms)
          │                       │
          ▼                       ▼
   Read I2C data           Hardware Reset
          │                       │
          │                 RST low 20ms
          │                 RST high
          │                 Wait boot (100ms)
          │                 Re-initialize
          │                       │
          └───────────┬───────────┘
                      │
              Update state
                      │
                      ▼
               Loop back
```

### Data Analysis

**100Hz Quaternion Sampling Budget:**

| Item                 | Time Budget   | Notes                 |
| -------------------- | ------------- | --------------------- |
| Sample period        | 10ms          | 100Hz = 1/100 = 0.01s |
| INT wait             | \~0ms (async) | Await, no CPU usage   |
| I2C header read      | \~0.2ms       | 4 bytes @ 400kHz      |
| I2C payload read     | \~0.5ms       | \~20 bytes typical    |
| Parse quaternion     | \~0.1ms       | Q16 to f32 conversion |
| State update         | \~0.1ms       | Mutex lock, copy      |
| **Total per sample** | **\~0.9ms**   | Leaves 9.1ms margin   |

Note: CPU is mostly idle during await, freeing cycles for other tasks.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: BNO086 driver shall implement SHTP protocol over I2C
  - Rationale: BNO086 uses SHTP, not standard register access
  - Acceptance Criteria: Successfully read/write SHTP packets

- [ ] **FR-DRAFT-2**: Driver shall use interrupt-driven data acquisition (INT pin)
  - Rationale: Efficient async design without polling
  - Acceptance Criteria: Data read only on INT falling edge, not timer-based

- [ ] **FR-DRAFT-3**: Driver shall enable Rotation Vector report at 100Hz
  - Rationale: Provides quaternion for attitude estimation
  - Acceptance Criteria: Report ID 0x05 received at \~100Hz rate

- [ ] **FR-DRAFT-4**: Driver shall implement hardware reset recovery via RST pin
  - Rationale: Long-term reliability for autonomous operation
  - Acceptance Criteria: Automatic reset after 500ms INT timeout

- [ ] **FR-DRAFT-5**: Driver shall implement `ImuSensor` trait for integration
  - Rationale: Compatibility with existing AHRS/navigation subsystems
  - Acceptance Criteria: Trait methods return valid quaternion data

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Quaternion output shall be available within 1 second of boot
  - Category: Performance
  - Rationale: Fast startup for operational readiness
  - Target: First valid quaternion < 1s after power-on

- [ ] **NFR-DRAFT-2**: I2C read latency shall not exceed 2ms per sample
  - Category: Performance
  - Rationale: Must fit within 10ms sample period
  - Target: <2ms measured I2C transaction time

- [ ] **NFR-DRAFT-3**: Driver shall recover from errors without manual intervention
  - Category: Reliability
  - Rationale: Autonomous drone/rover operation
  - Target: Automatic recovery within 200ms of error detection

- [ ] **NFR-DRAFT-4**: Driver memory usage shall not exceed 2KB
  - Category: Resource Usage
  - Rationale: Embedded system constraints
  - Target: <2KB static allocation

## Design Considerations

### Technical Constraints

**Hardware:**

- RP2350 (Pico 2 W): Cortex-M33, 150MHz, FPU, 520KB RAM
- I2C0 bus (consider separate bus if GPS uses I2C)
- GPIO required: INT (input, pull-up), RST (output)
- 3.3V compatible

**Software:**

- Rust `no_std` with Embassy async runtime
- `embedded-hal-async::i2c::I2c` trait for portability
- No blocking operations in data path
- Static memory allocation only

**I2C Considerations:**

- BNO086 requires clock stretching support
- Set generous I2C timeout (sensor may stretch clock during processing)
- 400kHz Fast Mode recommended

### Potential Approaches

#### Option 1: Minimal SHTP Implementation (Recommended)

**Description:** Implement only the essential SHTP functionality needed for Rotation Vector reading.

**Features:**

- Header parsing for length determination
- Rotation Vector report (0x05) parsing only
- Set Feature Command for enabling reports
- INT-driven async read loop
- RST-based error recovery

**Pros:**

- Minimal code size (\~500-800 lines)
- Focused on project needs (quaternion only)
- Easier to maintain and debug
- No external crate dependencies (questionable BNO08x Rust crates)

**Cons:**

- Limited to Rotation Vector output
- Cannot easily add other report types later
- Requires understanding SHTP protocol internals

**Effort:** Medium

#### Option 2: Full SHTP Implementation

**Description:** Implement complete SHTP protocol with all report types.

**Pros:**

- Access to all BNO086 outputs (gravity, linear accel, raw data)
- Reusable for other projects
- More flexible

**Cons:**

- Higher complexity and code size
- More time to implement
- May not need most features

**Effort:** High

#### Option 3: Use Existing Rust BNO08x Crate

**Description:** Attempt to use existing `bno080` or similar crates.

**Pros:**

- Less development time (if crate works)
- Community-maintained

**Cons:**

- Available crates are blocking (not async)
- May not fit Embassy async model
- Quality and maintenance uncertain
- May need significant modification

**Effort:** Low-Medium (if works), High (if needs modification)

### Architecture Impact

**ADR Updates:**

- May need to update `ADR-ymkzt-ekf-ahrs-implementation` since BNO086 provides quaternion directly (no external EKF needed for attitude)
- Consider new ADR for BNO086-specific architecture decisions

**Module Structure:**

```
src/devices/imu/
├── mod.rs                  # Public exports
├── bno086/
│   ├── mod.rs              # Driver public API
│   ├── shtp.rs             # SHTP protocol implementation
│   ├── reports.rs          # Report parsing (Rotation Vector)
│   └── driver.rs           # Core driver with INT/RST handling
├── icm20948/               # Retained (for future genuine hardware)
├── mpu9250/                # Retained (for reference)
└── mock.rs                 # Mock IMU for testing
```

**State Management:**

With BNO086 providing quaternion directly, the `ATTITUDE_STATE` can be updated without EKF:

```rust
// BNO086 provides quaternion directly
pub async fn bno086_task(/* ... */) {
    loop {
        // Wait for INT (data ready)
        int_pin.wait_for_falling_edge().await;

        // Read quaternion from BNO086
        let quat = driver.read_rotation_vector().await?;

        // Update global state directly (no EKF needed)
        let mut state = ATTITUDE_STATE.lock().await;
        state.quaternion = quat;
        state.euler = quat.to_euler();
        state.timestamp_ms = now();
    }
}
```

## Risk Assessment

| Risk                                | Probability | Impact | Mitigation Strategy                                 |
| ----------------------------------- | ----------- | ------ | --------------------------------------------------- |
| BNO086 hardware also counterfeit    | Low         | High   | Purchase from reputable source (Adafruit, SparkFun) |
| SHTP protocol implementation errors | Medium      | Medium | Follow datasheet closely, extensive testing         |
| I2C clock stretching issues         | Low         | Medium | Configure generous timeout, test with scope         |
| INT pin timing issues               | Low         | Medium | Use Embassy GPIO interrupt handling                 |
| Quaternion accuracy insufficient    | Low         | Medium | BNO086 is industry-standard; calibrate properly     |
| Recovery loop (repeated resets)     | Low         | Medium | Implement reset counter, alert on repeated failures |

## Open Questions

- [ ] Which BNO086 breakout board to use? → Options: Adafruit BNO085, SparkFun BNO086
- [ ] Should raw gyro/accel data also be exposed for logging? → Method: Review logging requirements
- [ ] Is 100Hz quaternion rate sufficient or should we use higher? → Decision: Start with 100Hz, adjust if needed
- [ ] How to integrate with existing `ImuSensor` trait (designed for raw data)? → Next step: Review trait design, possibly extend

## Recommendations

### Immediate Actions

1. **Select BNO086 as primary IMU sensor** for the following reasons:
   - On-chip sensor fusion eliminates EKF complexity
   - Quaternion output directly usable for attitude
   - Hardware reset capability for reliability
   - Interrupt-driven design is async-friendly

2. **Purchase from reputable source** to avoid counterfeit issues:
   - Adafruit BNO085/BNO086 (\~$20-25)
   - SparkFun BNO086 Qwiic (\~$25)

3. **Implement minimal SHTP driver** (Option 1):
   - Focus on Rotation Vector report only
   - INT-driven async design
   - RST-based error recovery

### Next Steps

1. [ ] Create formal requirements: FR/NFR for BNO086 driver
2. [ ] Draft ADR: BNO086 driver architecture (SHTP, INT/RST handling)
3. [ ] Create task: T-<id>-bno086-driver-implementation
   - Phase 1: SHTP protocol basics, I2C communication
   - Phase 2: Rotation Vector parsing, INT handling
   - Phase 3: RST error recovery, `ImuSensor` trait
4. [ ] Update `ADR-ymkzt-ekf-ahrs-implementation` to note BNO086 alternative
5. [ ] Review `ImuSensor` trait for compatibility with quaternion-native sensors

### Out of Scope

- **Full SHTP protocol implementation**: Only Rotation Vector needed
- **BNO086 DMP reprogramming**: Use default firmware
- **External EKF for BNO086**: On-chip fusion is sufficient
- **ICM-20948/MPU-9250 driver updates**: Retain existing code but not active development
- **Raw sensor data exposure**: Quaternion output is sufficient for navigation

## Appendix

### References

**BNO086 Resources:**

- [BNO086 Datasheet](https://www.ceva-ip.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf)
- [Hillcrest Labs SH-2 Reference Manual](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-9-dof-orientation-imu-fusion-breakout-bno085.pdf)
- [SHTP Protocol Specification](https://cdn-learn.adafruit.com/assets/assets/000/076/762/original/1000-3925-Sensor-Hub-Transport-Protocol-v1.7.pdf)
- [Adafruit BNO085 Guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085)
- [SparkFun BNO086 Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-9dof-imu-breakout---bno086-qwiic-hookup-guide)

**Existing Rust Crates (reference only):**

- [bno080 crate](https://crates.io/crates/bno080) - Blocking, may need async adaptation

**Protocol Documentation:**

- Report ID 0x05: Rotation Vector
- Report ID 0x08: Game Rotation Vector (no magnetometer)
- Report ID 0x06: Gravity Vector
- Report ID 0x04: Linear Acceleration

### SHTP Report Enable Command

To enable Rotation Vector at 100Hz (10ms interval):

```
Set Feature Command (Report ID 0xFD):
+--------+--------+--------+--------+--------+--------+--------+--------+
| 0xFD   | RptID  | Flags  |Change  |Report  |Report  |Batch   |Sens-   |
|        |        |        |Sens    |Int_L   |Int_H   |Int_L   |Specific|
+--------+--------+--------+--------+--------+--------+--------+--------+
| 0xFD   | 0x05   | 0x00   | 0x00   | 0x10   | 0x27   | 0x00   | 0x00   |
                                       └──────┴──────┘
                                       10000µs = 10ms = 100Hz
```

### Pin Configuration

```
BNO086 Breakout    RP2350 Pico 2 W
─────────────────  ─────────────────
VIN                3.3V
GND                GND
SDA                GPIO4 (I2C0 SDA)
SCL                GPIO5 (I2C0 SCL)
INT                GPIO6 (Input, Pull-up)
RST                GPIO7 (Output)
```

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
