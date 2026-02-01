# AN-00028 BNO086 9-Axis IMU Integration

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00027-mpu9250-imu-and-ekf-integration](../analysis/AN-00027-mpu9250-imu-and-ekf-integration.md)
  - ~~[AN-00005-imu-sensor-selection](../analysis/AN-00005-imu-sensor-selection.md)~~ (Deprecated)
- Related Requirements:
  - [FR-00101-imu-sensor-trait](../requirements/FR-00101-imu-sensor-trait.md)
  - [NFR-00002-imu-sampling-rate](../requirements/NFR-00002-imu-sampling-rate.md)
  - [FR-00001-ahrs-attitude-estimation](../requirements/FR-00001-ahrs-attitude-estimation.md)
- Related ADRs:
  - [ADR-00026-mpu9250-i2c-driver-architecture](../adr/ADR-00026-mpu9250-i2c-driver-architecture.md)
  - [ADR-00025-ekf-ahrs-implementation](../adr/ADR-00025-ekf-ahrs-implementation.md)
  - [ADR-00032-ahrs-abstraction-architecture](../adr/ADR-00032-ahrs-abstraction-architecture.md)
- Related Tasks:
  - [T-00031-bno086-driver-implementation](../tasks/T-00031-bno086-driver-implementation/README.md)
  - [T-00032-ahrs-abstraction-layer](../tasks/T-00032-ahrs-abstraction-layer/README.md)
  - ~~[T-00024-icm20948-driver-implementation](../tasks/T-00024-icm20948-driver-implementation/README.md)~~ (Cancelled - counterfeit hardware)
  - ~~[T-00023-mpu9250-driver-implementation](../tasks/T-00023-mpu9250-driver-implementation/README.md)~~ (Cancelled)

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

- EKF AHRS design in progress (`ADR-00025-ekf-ahrs-implementation`)
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

- May need to update `ADR-00025-ekf-ahrs-implementation` since BNO086 provides quaternion directly (no external EKF needed for attitude)
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

## Current Implementation Status (2026-01)

**Status: Unstable (I2C communication issues on RP2350)**

BNO086 9-axis IMU integration is experiencing intermittent I2C communication failures on RP2350. Initial reads may succeed, but subsequent reads often fail with transport errors or timeouts. Success rate is approximately 2/3 with current software mitigations.

### Typical Failure Log

```
[INFO]  Initializing BNO086 with GPIO driver (INT-driven)...
[DEBUG] BNO086: Hardware reset
[TRACE] SHTP raw header: 00 00 00 00
[DEBUG] BNO086: clear_buffer complete
[TRACE] SHTP raw header: 1C 01 00 00
[TRACE] SHTP packet: len=284 ch=0 seq=0 payload=280   ← Success
[DEBUG] SHTP I2C read error (addr=0x4B, len=284)      ← Failure starts
[DEBUG] BNO086: Product ID read transport error, retrying...
[WARN]  SHTP I2C read timeout (addr=0x4B)
...
[ERROR] BNO086: All init attempts failed
```

### Log Output (When Working)

```
[INFO]  Using BNO086 at address 0x4B
[INFO]  Initializing BNO086 with GPIO driver (INT-driven)...
[DEBUG] BNO086: Hardware reset
[TRACE] SHTP packet: len=284 ch=0 seq=0 payload=280
[TRACE] SHTP packet: len=20 ch=2 seq=1 payload=16
[INFO]  BNO086: Product ID - SW 3.12.6
[INFO]  BNO086: Initialized successfully
[INFO]  BNO086 GPIO driver initialized!
[INFO]  AHRS task spawned (BNO086 External AHRS, INT-driven)
[INFO]  AHRS task started (BNO086 External AHRS, 100Hz)
[TRACE] SHTP packet: len=189 ch=3 seq=3 payload=185
[TRACE] SHTP packet: len=21 ch=3 seq=4 payload=17
...
[INFO]  AHRS converged after 100 samples
```

### Implementation Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    AHRS Task                            │
│  ┌───────────────────────────────────────────────────┐  │
│  │           Bno086ExternalAhrs<D>                   │  │
│  │         (generic over QuaternionSensor)           │  │
│  └───────────────────────────────────────────────────┘  │
│                          │                              │
│  ┌───────────────────────────────────────────────────┐  │
│  │      Bno086DriverWithGpio<T, INT, RST>            │  │
│  │  - INT-driven acquisition (no polling)            │  │
│  │  - Automatic hardware reset recovery              │  │
│  │  - Batched report (0xFB) parsing                  │  │
│  └───────────────────────────────────────────────────┘  │
│                          │                              │
│  ┌───────────────────────────────────────────────────┐  │
│  │              ShtpI2c Transport                    │  │
│  │          (280-byte packet buffer)                 │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### Key Implementation Features

- **INT-driven reads**: Waits for INT LOW before I2C read (no polling overhead)
- **Batched report parsing**: Handles 0xFB Base Timestamp Reference packets
- **Automatic recovery**: Hardware reset on consecutive errors (max 3 resets)
- **280-byte buffers**: Accommodates large BNO086 packets (up to 284 bytes)

## Implementation Findings (Empirical)

This section documents actual implementation experience and lessons learned during driver development.

### Hardware Configuration (Tested)

| Signal      | GPIO   | Notes                                                      |
| ----------- | ------ | ---------------------------------------------------------- |
| SDA         | GPIO4  | I2C0 SDA                                                   |
| SCL         | GPIO5  | I2C0 SCL                                                   |
| INT         | GPIO22 | Data ready (active low) - optional for polling mode        |
| RST         | GPIO17 | Hardware reset (active low) - **avoid pulsing at startup** |
| I2C Address | 0x4B   | SA0 = high (alternate: 0x4A)                               |
| I2C Speed   | 50kHz  | Lower speed improves stability after MCU reset             |

### Critical Findings

#### 1. RST Pin Handling - DO NOT Pulse at Startup

**Finding:** Pulsing RST at boot causes the BNO086 to enter an unstable state where I2C communication returns corrupted data (`PayloadTooLarge`, `TransportError`).

**Symptoms when RST is pulsed:**

- First I2C scan returns NACK (sensor not ready)
- Subsequent reads return garbage length values
- Alternating `PayloadTooLarge` and `TransportError` errors

**Recommendation:** Do NOT manipulate RST at startup. The BNO086 self-initializes properly on power-up. Only use RST for error recovery when I2C bus is completely stuck.

#### 2. BNO086 Outputs Default Reports Without SET_FEATURE

**Finding:** The BNO086 outputs batched reports (0xFB) containing Game Rotation Vector (0x08) by default, without requiring any SET_FEATURE command.

**Observed output pattern:**

```
Pkt 0: ch=3 len=71 report=0xFB  -> Embedded 0x08 at offset 15
Pkt 1: ch=3 len=137 report=0xFB -> Embedded 0x08 at offset 15
(alternating 71/137 byte packets)
```

**Implication:** Driver init can skip SET_FEATURE commands and immediately start reading quaternion data from the default reports.

#### 3. I2C Bus Recovery After MCU Reset

**Finding:** If the MCU resets while the BNO086 is mid-I2C-transaction, the sensor may hold SDA low, preventing further communication.

**Recovery procedure:**

1. Configure SCL as GPIO output, SDA as input with pull-up
2. Check if SDA is stuck low
3. Toggle SCL up to 16 times (50µs per pulse) to release slave
4. Generate STOP condition (SDA low→high while SCL high)
5. Wait 10ms before initializing I2C peripheral

**Code reference:** `examples/bno086_demo.rs` I2C Bus Recovery section

#### 4. Buffer Size Requirements

**Finding:** BNO086 sends large batched reports that require adequate buffer sizes.

| Buffer                | Size         | Notes                           |
| --------------------- | ------------ | ------------------------------- |
| SHTP I2C read buffer  | 280 bytes    | `src/communication/shtp/i2c.rs` |
| ShtpPacket\<N>        | 280 bytes    | Driver uses `ShtpPacket::<280>` |
| Observed packet sizes | 71-137 bytes | 0xFB batched reports            |

**Warning:** Using smaller buffers (e.g., 128-256 bytes) causes `PayloadTooLarge` errors during boot when sensor may send larger or corrupted packets. Always match ShtpPacket buffer size to the I2C transport buffer (280 bytes).

#### 5. Timeouts Are Essential

**Finding:** I2C reads can hang indefinitely due to clock stretching. All transport reads must have timeouts.

**Timeout implementation:**

```rust
// All I2C reads wrapped with timeout
let result = embassy_time::with_timeout(
    Duration::from_millis(100),  // 100ms timeout
    transport.read_packet(&mut packet),
).await;
```

**Affected functions:**

- `clear_buffer()`
- `read_product_id()`
- `configure_rotation_vector()`
- `read_and_process()`

#### 6. First I2C Scan Often Returns NACK

**Finding:** After power-on, the first I2C scan attempt frequently returns NACK. The sensor needs \~500ms to become responsive.

**Recommendation:** Implement retry logic with 500ms delay between attempts (3 attempts recommended).

#### 7. RST Recovery Requires Extended Boot Time

**Finding:** After RST pulse recovery (for SDA stuck low), the sensor needs 3+ seconds before SHTP protocol becomes fully operational.

**Symptoms with insufficient wait time (1s):**

- I2C ACK works (sensor detected at 0x4B)
- INT pin shows 0 edges (no data ready signals)
- First SHTP read returns `PayloadTooLarge` or garbage data

**Recommendation:**

- Wait 3 seconds after RST pulse before I2C initialization
- Monitor INT for 2 seconds; if 0 edges, wait additional 2 seconds
- Wait 500ms before initializing I2C peripheral after recovery

#### 8. INT Pin Initial State Issue (2026-01)

**Finding:** After MCU reset (without BNO086 power cycle), the INT pin may already be LOW because the sensor has pending data from before the reset. If the driver waits for a "falling edge", no edge occurs and the operation times out.

**Symptoms:**

- `wait_for_falling_edge()` never triggers because INT is already LOW
- Timeout fires (100-500ms depending on function)
- Subsequent I2C read fails due to timing mismatch
- Results in "SHTP I2C read error" followed by "TransportError"

**Affected functions:**

| Function                    | Timeout |
| --------------------------- | ------- |
| `init_internal()`           | 500ms   |
| `reinitialize()`            | 500ms   |
| `clear_buffer_with_retry()` | 200ms   |
| `clear_buffer()`            | 200ms   |
| `read_product_id()`         | 100ms   |

**Fix (implemented 2026-01-13):** Check `is_low()` before waiting for falling edge:

```rust
// Before (buggy):
let _ = with_timeout(200, self.int_pin.wait_for_falling_edge()).await;

// After (fixed):
if !self.int_pin.is_low() {
    let _ = with_timeout(200, self.int_pin.wait_for_falling_edge()).await;
}
```

**Code reference:** `src/devices/imu/bno086/driver_gpio.rs`

#### 9. BNO08x I2C Timing Bug (Hardware Issue)

**Finding:** The BNO08x series (BNO055, BNO080, BNO085, BNO086) has a known I2C timing bug where the SDA-to-SCL setup time occasionally violates the I2C specification when clock stretching ends.

**Reference:** [SparkFun Issue #72](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/72)

**Symptoms:**

- First few I2C reads may succeed, then subsequent reads fail
- TransportError and Timeout errors alternate
- Large packet reads (284 bytes) are more likely to fail

**Software mitigations attempted:**

| Mitigation                                 | Result                                  |
| ------------------------------------------ | --------------------------------------- |
| Reduced I2C speed (10kHz → 5kHz)           | Partial improvement                     |
| Added delays between reads (1ms → 20-50ms) | Partial improvement                     |
| Added INT wait before each read            | Partial improvement                     |
| Init retry logic (3 attempts)              | Allows recovery, doesn't fix root cause |

**Recommended hardware fix:** Asymmetric pull-up resistors:

```
SDA: 1KΩ pull-up to 3.3V (faster rise time)
SCL: 4.7KΩ pull-up to 3.3V (slower rise time)
```

This increases the SDA-to-SCL setup time margin, addressing the timing violation.

**Alternative:** Switch to SPI communication, which avoids I2C timing issues entirely.

### Batched Report (0xFB) Structure

The BNO086 outputs 0xFB "Base Timestamp Reference" reports containing multiple embedded sensor reports:

```
0xFB Report Structure:
Offset 0:    0xFB (Report ID)
Offset 1-4:  Base timestamp (4 bytes, little-endian)
Offset 5+:   First embedded report (no time delta prefix)
             [2-byte delta][next report]...

Embedded reports found at offset 15:
- 0x08: Game Rotation Vector (12 bytes) - quaternion without magnetometer
- 0x02: Gyroscope Calibrated (10 bytes)
- 0x05: Rotation Vector (14 bytes) - quaternion with magnetometer
```

### Polling Mode vs Interrupt Mode

**Tested configuration:** Polling mode without INT pin monitoring

**Observation:** Polling mode achieves \~13Hz with 50ms inter-read delay. For higher rates, INT-driven reads are recommended but require proper INT wiring.

**INT behavior observed:**

- INT goes LOW when data is ready
- Falling edges occur at \~10Hz (100ms interval) by default
- INT monitoring can be skipped for basic functionality
- **Critical:** After MCU reset, INT may already be LOW; check `is_low()` before waiting for falling edge

### Final Fixes Applied (2025-01)

#### Buffer Size Increase (128 → 280 bytes)

**Problem:** `Bno086DriverWithGpio` used 128-byte buffers, but BNO086 sends packets up to 284 bytes.

**Fix:** Changed all `ShtpPacket::<128>` to `ShtpPacket::<280>` in `driver_gpio.rs`.

#### Batched Report (0xFB) Parsing

**Problem:** `read_and_process()` only handled direct reports (0x05, 0x08), not batched reports.

**Fix:** Added 0xFB parsing to `read_and_process()`:

```rust
// 0xFB = Base Timestamp Reference - contains batched reports
// Format: [0xFB][4-byte timestamp][report][2-byte delta][report]...
if report_id == 0xFB && payload.len() > 7 {
    let mut offset = 5; // Skip 0xFB + 4-byte timestamp
    while offset < payload.len() {
        // Parse embedded 0x05/0x08 quaternion reports
        ...
    }
}
```

**Supported embedded report sizes:**

| Report ID | Type                 | Size (bytes) |
| --------- | -------------------- | ------------ |
| 0x01      | Accelerometer        | 10           |
| 0x02      | Gyroscope Calibrated | 10           |
| 0x03      | Magnetometer         | 10           |
| 0x04      | Linear Acceleration  | 10           |
| 0x05      | Rotation Vector      | 14           |
| 0x08      | Game Rotation Vector | 12           |

### Previous Integration Steps

#### Phase 1: Generic AHRS Interface

Made `Bno086ExternalAhrs` generic over any `QuaternionSensor` implementation:

```rust
// Before
pub struct Bno086ExternalAhrs<T: ShtpTransport> {
    driver: Bno086Driver<T>,
}

// After
pub struct Bno086ExternalAhrs<D: QuaternionSensor> {
    driver: D,
}
```

#### Phase 2: GPIO Pin Handling

1. **INT pin**: `EmbassyIntPin` wraps `Input<'static>` with pull-up
2. **RST pin**: `EmbassyRstPin` wraps `Flex<'static>` for high-Z release
   - `set_low()`: Drive LOW (assert reset)
   - `set_high()`: Release to high-Z (input mode with pull-up)

#### Phase 3: QuaternionSensor Trait

Added `angular_rate()` to `QuaternionSensor` trait for gyroscope data:

```rust
pub trait QuaternionSensor {
    async fn read_quaternion(&mut self) -> Result<QuaternionReading, QuaternionError>;
    fn is_healthy(&self) -> bool;
    fn last_update_us(&self) -> u64;
    fn angular_rate(&self) -> Vector3<f32> { Vector3::zeros() }  // Default
}
```

### Code Locations

| Component            | Path                                     |
| -------------------- | ---------------------------------------- |
| SHTP I2C Transport   | `src/communication/shtp/i2c.rs`          |
| BNO086 Driver        | `src/devices/imu/bno086/driver.rs`       |
| BNO086 GPIO Driver   | `src/devices/imu/bno086/driver_gpio.rs`  |
| GPIO Pin Wrappers    | `src/devices/imu/bno086/gpio.rs`         |
| Report Parsing       | `src/devices/imu/bno086/reports.rs`      |
| AHRS External Bridge | `src/subsystems/ahrs/external/bno086.rs` |
| Rover Example        | `examples/pico_trail_rover.rs`           |
| BNO086 Demo          | `examples/bno086_demo.rs`                |

## Risk Assessment

| Risk                                | Probability | Impact     | Mitigation Strategy                                     |
| ----------------------------------- | ----------- | ---------- | ------------------------------------------------------- |
| BNO086 hardware also counterfeit    | Low         | High       | Purchase from reputable source (Adafruit, SparkFun)     |
| SHTP protocol implementation errors | Medium      | Medium     | Follow datasheet closely, extensive testing             |
| I2C clock stretching issues         | Low         | Medium     | Configure generous timeout, test with scope             |
| INT pin timing issues               | Low         | Medium     | Use Embassy GPIO interrupt handling                     |
| Quaternion accuracy insufficient    | Low         | Medium     | BNO086 is industry-standard; calibrate properly         |
| Recovery loop (repeated resets)     | Low         | Medium     | Implement reset counter, alert on repeated failures     |
| **RST pulse destabilizes sensor**   | **High**    | **High**   | **Do NOT pulse RST at startup; use only for recovery**  |
| **I2C bus stuck after MCU reset**   | **Medium**  | **Medium** | **Implement SCL toggle recovery before I2C init**       |
| **INT already LOW after MCU reset** | **Medium**  | **Medium** | **Check `is_low()` before `wait_for_falling_edge()`**   |
| **BNO08x I2C timing bug**           | **High**    | **Medium** | **Asymmetric pull-ups (SDA 1KΩ, SCL 4.7KΩ) or use SPI** |

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
4. [ ] Update `ADR-00025-ekf-ahrs-implementation` to note BNO086 alternative
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

**Reference Implementations:**

- [SparkFun BNO08x Arduino Library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library) - Chunk-based I2C reads with INT wait
- [tstellanova/bno080](https://github.com/tstellanova/bno080) - Rust embedded-hal driver with SPI support
- [Adafruit CircuitPython BNO08x](https://github.com/adafruit/Adafruit_CircuitPython_BNO08x) - Python implementation with timing notes

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
