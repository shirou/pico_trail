# AN-00002 Core System Design for Embedded Autopilot

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-00001-ardupilot-analysis](AN-00001-ardupilot-analysis.md)
  - [AN-00004-platform-abstraction](AN-00004-platform-abstraction.md)
- Related Requirements:
  - [FR-00003-failsafe-mechanisms](../requirements/FR-00003-failsafe-mechanisms.md)
- Related ADRs:
  - [ADR-00004-storage-strategy](../adr/ADR-00004-storage-strategy.md)
  - [ADR-00005-task-scheduler-selection](../adr/ADR-00005-task-scheduler-selection.md)
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis examines the design of eight core systems required for a functional autopilot: Scheduler, Parameter System, Data Logger, Storage, Safety/Failsafe, Calibration, State Machine, and Notification. These systems provide the foundation upon which navigation, control, and communication subsystems are built. Drawing from ArduPilot's proven architecture while adapting to embedded Rust constraints, we propose designs that prioritize memory safety, real-time performance, and deterministic behavior.

Key findings: The Scheduler is the most critical component (affects all other systems), and the choice between Embassy async, RTIC, or a custom scheduler will drive many other design decisions. Parameter and Logger systems must carefully manage Flash wear leveling to avoid premature hardware failure. Safety and State Machine systems require formal verification to prevent catastrophic failures.

## Problem Space

### Current State

The project currently has:

- No task scheduling infrastructure
- No persistent storage management
- No parameter configuration system
- No data logging capability
- No safety checks or failsafe logic
- Basic panic handler from app-template

### Desired State

A complete set of core systems that provide:

- **Scheduler**: Deterministic execution of periodic tasks at 1Hz to 400Hz
- **Parameters**: Runtime configuration with Flash persistence and MAVLink access
- **Logger**: High-frequency sensor/control data logging to Flash
- **Storage**: Wear-leveled Flash access for parameters, missions, logs
- **Safety**: Failsafe triggers for GPS loss, RC loss, battery low, geofence violations
- **Calibration**: Guided procedures for IMU, compass, RC, ESC calibration
- **State Machine**: Disarmed → Pre-arm → Armed → Emergency state transitions
- **Notification**: LED/buzzer/GCS messages for status and warnings

### Gap Analysis

**Missing Infrastructure**:

- Task scheduling and execution framework
- Flash storage abstraction with wear leveling
- Parameter definition and serialization
- Binary log format and encoder
- Pre-arm check system
- Calibration data persistence

**Integration Challenges**:

- How does Scheduler interact with async Embassy or RTIC?
- How does Logger avoid blocking critical control loops?
- How does Storage prevent Flash wear from excessive parameter writes?
- How does State Machine coordinate with Safety checks?

## Stakeholder Analysis

| Stakeholder          | Interest/Need                                 | Impact | Priority |
| -------------------- | --------------------------------------------- | ------ | -------- |
| Control Loops        | Deterministic, low-jitter task execution      | High   | P0       |
| Operators            | Ability to tune parameters without reflashing | High   | P0       |
| Post-Flight Analysis | Comprehensive logs for debugging              | Medium | P1       |
| Safety Officers      | Reliable failsafe and pre-arm checks          | High   | P0       |
| Hardware             | Flash wear leveling to extend device lifespan | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - New project without user base.

### Competitive Analysis

**ArduPilot Core Systems**:

1. **AP_Scheduler**: Fixed-rate task execution (e.g., 400Hz IMU, 50Hz control, 10Hz telemetry), priority-based scheduling, task runtime monitoring, CPU load calculation
2. **AP_Param**: Parameter groups (e.g., `COMPASS_`, `PID_RATE_`), EEPROM storage with checksum, parameter tree for hierarchical naming, MAVLink parameter protocol
3. **AP_Logger**: Binary log format (FMT messages define structure), microSD card or Flash storage, log download via MAVLink, selective logging by flight mode
4. **AP_Arming**: Pre-arm checks (GPS lock, compass cal, accel cal, etc.), arming/disarming logic, safety switch support
5. **AP_Notify**: LED patterns (slow blink = disarmed, fast blink = no GPS, etc.), buzzer tones for events, text messages to GCS

**PX4 (Pixhawk) Approach**:

- Uses uORB (micro Object Request Broker) for inter-task communication
- Event-driven architecture with publish/subscribe model
- More complex than needed for single-MCU system
- Designed for multi-core, multi-process environments

**Recommendation**: Follow ArduPilot's simpler approach, adapted for Rust and Embassy/RTIC.

### Technical Investigation

**Scheduler Design Options**:

| Approach           | Pros                                                         | Cons                                       | Best For                |
| ------------------ | ------------------------------------------------------------ | ------------------------------------------ | ----------------------- |
| Embassy Async      | Modern async/await, efficient task switching, good ecosystem | Learning curve, larger binary, GC overhead | Complex I/O-heavy tasks |
| RTIC               | Zero-cost, compile-time scheduling, proven                   | Steeper learning curve, Cortex-M only      | Hard real-time systems  |
| Custom Timer-based | Full control, minimal overhead, tailored to needs            | High development effort, potential bugs    | Simple fixed-rate tasks |

**Parameter Storage Layout**:

```
Flash Layout:
[Parameter Block 1] (4 KB)
[Parameter Block 2] (4 KB) <- Redundant copy
[Mission Storage] (8 KB)
[Log Storage] (Remaining Flash)
```

- Use two blocks for redundancy (if Block 1 corrupted, load Block 2)
- Checksum each block (CRC32)
- Wear leveling: rotate writes across multiple blocks

**Log Format Design**:

ArduPilot uses a self-describing binary format:

```
FMT message: [Type, Length, Name, Format, Columns]
Data message: [Type, Timestamp, Field1, Field2, ...]
```

Example:

```
FMT: [1, 28, "IMU", "QffffffIIf", "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T"]
IMU: [1, 1234567, 0.01, 0.02, 0.03, 0.0, 0.0, 9.8, 0, 0, 25.0]
```

Advantages:

- Self-documenting (FMT messages describe structure)
- Efficient binary encoding (no JSON/XML overhead)
- Standard tools for reading (pymavlink, MAVExplorer)

### Data Analysis

N/A - No operational data available yet.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-13**: Scheduler shall execute tasks at configured rates (1Hz to 400Hz) with jitter < 1ms → Will become FR-013
  - Rationale: Control loops require predictable timing
  - Acceptance Criteria: 400Hz IMU task executes 400 ± 4 times per second, measured over 10-second window

- [ ] **FR-DRAFT-14**: Parameter system shall support at least 200 parameters with Flash persistence → Will become FR-014
  - Rationale: ArduPilot Rover uses \~300 parameters; 200 is a reasonable subset
  - Acceptance Criteria: Parameters persist across reboots, load time < 100ms

- [ ] **FR-DRAFT-15**: Logger shall record IMU data at 100Hz, GPS at 10Hz, control outputs at 50Hz → Will become FR-015
  - Rationale: Sufficient for post-flight analysis without excessive Flash usage
  - Acceptance Criteria: Log download via MAVLink, compatible with MAVExplorer

- [ ] **FR-DRAFT-16**: Storage system shall implement wear leveling to support 10,000 parameter save cycles → Will become FR-016
  - Rationale: RP2040/RP2350 Flash has limited write endurance (\~10K-100K cycles)
  - Acceptance Criteria: Wear leveling distributes writes across multiple blocks, verified via Flash block erase count monitoring

- [ ] **FR-DRAFT-17**: Safety system shall trigger failsafe within 1 second of GPS/RC signal loss → Will become FR-017
  - Rationale: Minimize unsafe autonomous operation with degraded sensors
  - Acceptance Criteria: Failsafe action (Hold/RTL) triggered within 1 second, validated in HIL testing

- [ ] **FR-DRAFT-18**: State machine shall prevent arming unless all pre-arm checks pass → Will become FR-018
  - Rationale: Safety-critical: never arm with uncalibrated sensors or low battery
  - Acceptance Criteria: Arming fails if any check fails (GPS lock, compass cal, accel cal, battery OK)

- [ ] **FR-DRAFT-19**: Calibration system shall guide user through IMU and compass calibration procedures → Will become FR-019
  - Rationale: Essential for accurate AHRS estimation
  - Acceptance Criteria: Calibration completes successfully, results persist to Flash, GCS shows progress

- [ ] **FR-DRAFT-20**: Notification system shall indicate system status via LED patterns and GCS messages → Will become FR-020
  - Rationale: User feedback for arming state, GPS lock, errors
  - Acceptance Criteria: LED blinks at distinct rates for different states, GCS receives text messages for errors

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-8**: Scheduler overhead shall not exceed 5% of CPU time → Will become NFR-008
  - Category: Performance
  - Rationale: Leave 95% of CPU for application tasks
  - Target: Measured via task profiling, scheduler uses < 7 ms per second @ 133 MHz

- [ ] **NFR-DRAFT-9**: Parameter load/save operations shall not block control loops → Will become NFR-009
  - Category: Real-time Performance
  - Rationale: Writing to Flash can take 100ms+, must not stall control
  - Target: Parameter writes execute in background task, control loops continue uninterrupted

- [ ] **NFR-DRAFT-10**: Flash write operations shall be minimized to extend hardware lifespan → Will become NFR-010
  - Category: Reliability
  - Rationale: Excessive writes degrade Flash (10K-100K cycle limit)
  - Target: Parameters written only on explicit save command, logs use circular buffer with batch writes

## Design Considerations

### Technical Constraints

1. **No Heap Allocation in Control Loops**: Must use static allocation or stack for real-time tasks
2. **Flash Write Latency**: Erasing a 4KB block can take \~100ms on RP2040/RP2350
3. **Task Stack Size**: Each Embassy/RTIC task requires dedicated stack space (\~1-4 KB per task)
4. **Interrupt Priority**: IMU sampling must have higher priority than telemetry to avoid jitter

### Potential Approaches

#### 1. Scheduler Design

**Option A: Embassy Async Executor**

```rust
#[embassy_executor::task]
async fn imu_task() {
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz
    loop {
        ticker.next().await;
        sample_imu().await;
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
```

- Pros: Modern async/await, easy to understand, good ecosystem support
- Cons: Async overhead, all tasks must be async
- Effort: Medium (learning async embedded patterns)

**Option B: RTIC**

```rust
#[rtic::app(device = rp2040_hal::pac, dispatchers = [TIMER_IRQ_0])]
mod app {
    #[task(priority = 3, binds = TIMER_IRQ_1)]
    fn imu_task(cx: imu_task::Context) {
        sample_imu();
        // Reschedule for next 400Hz tick
    }

    #[task(priority = 2)]
    fn control_task(cx: control_task::Context) {
        run_control_loop();
    }
}
```

- Pros: Zero-cost abstractions, compile-time scheduling guarantees, minimal overhead
- Cons: Steep learning curve, less flexible than async
- Effort: Medium-High (RTIC resource model)

**Recommendation**: Start with **Embassy** for faster development, switch to **RTIC** if performance is insufficient.

#### 2. Parameter System Design

**Option A: Simple Key-Value Store**

```rust
pub struct ParameterStore {
    params: &'static [Parameter],
}

pub struct Parameter {
    name: &'static str,
    value: AtomicU32, // f32 bits
    default: f32,
    min: f32,
    max: f32,
}
```

- Pros: Simple, fast access, compile-time parameter list
- Cons: No hierarchical naming, all parameters must be known at compile time
- Effort: Low

**Option B: ArduPilot-style Parameter Tree** ⭐ **Recommended**

```rust
pub struct ParamGroup {
    name: &'static str,
    params: &'static [Parameter],
}

// Example: COMPASS_DEV_ID, COMPASS_OFS_X, COMPASS_OFS_Y
const COMPASS_GROUP: ParamGroup = ParamGroup {
    name: "COMPASS",
    params: &[
        Parameter::new("DEV_ID", 0, 0, u16::MAX),
        Parameter::new("OFS_X", 0.0, -1000.0, 1000.0),
        Parameter::new("OFS_Y", 0.0, -1000.0, 1000.0),
    ],
};
```

- Pros: Organized parameters, compatible with ArduPilot conventions, supports future parameter discovery
- Cons: More complex to implement
- Effort: Medium

#### 3. Logger Design

**Option A: Synchronous Logging (Blocking)**

```rust
logger.log_imu(timestamp, gyro, accel); // Blocks until written to Flash
```

- Pros: Simple to implement
- Cons: Blocks control loops during Flash writes (unacceptable)
- Effort: Low

**Option B: Async Logging with Buffer** ⭐ **Recommended**

```rust
// Control loop
logger.queue_imu(timestamp, gyro, accel); // Writes to RAM buffer, returns immediately

// Background task
async fn logger_task() {
    loop {
        let batch = logger.get_batch();
        flash.write(batch).await; // Flush buffer to Flash periodically
    }
}
```

- Pros: Non-blocking, efficient batch writes
- Cons: Requires RAM buffer (4-16 KB), potential data loss if buffer overflows
- Effort: Medium

### Architecture Impact

This analysis will drive the following ADRs:

- **ADR-002**: Task scheduler selection (Embassy vs RTIC vs custom)
- **ADR-004**: Storage strategy (wear leveling, redundancy)
- **ADR-009**: Parameter system design (simple vs hierarchical)
- **ADR-010**: Data logging approach (synchronous vs async)
- **ADR-011**: State machine and safety system design

## Risk Assessment

| Risk                                                 | Probability | Impact | Mitigation Strategy                                                                         |
| ---------------------------------------------------- | ----------- | ------ | ------------------------------------------------------------------------------------------- |
| Flash wear leads to premature hardware failure       | Medium      | High   | Implement wear leveling, limit parameter writes, test with accelerated wear cycles          |
| Scheduler jitter causes control instability          | Low         | High   | Use hardware timers, profile task execution, switch to RTIC if Embassy jitter too high      |
| Logger buffer overflow loses critical data           | Medium      | Medium | Size buffer for worst-case logging rate, monitor buffer usage, drop low-priority logs first |
| Pre-arm checks too strict, prevent legitimate arming | Low         | Medium | Make checks configurable via parameters, test with diverse hardware setups                  |
| State machine logic error allows unsafe arming       | Low         | High   | Formal verification (model checking), extensive HIL testing                                 |

## Open Questions

- [ ] Should we use Embassy or RTIC for the scheduler? → Next step: Prototype both approaches with 400Hz IMU task, measure jitter and CPU overhead
- [ ] How large should the log buffer be (4 KB, 8 KB, 16 KB)? → Method: Calculate worst-case logging rate (IMU @ 100Hz, GPS @ 10Hz, etc.) and required buffer for 1-second window
- [ ] Should parameter groups be defined via macros or manual structs? → Next step: Draft ADR-009 evaluating macro-based vs manual approaches
- [ ] How do we test wear leveling without waiting for 10,000 write cycles? → Method: Create accelerated test that writes to Flash in tight loop, monitor block erase counts
- [ ] Should notification LEDs use dedicated GPIO or share with platform status LEDs? → Method: Review Pico W/2 W GPIO availability, ensure no conflicts with SPI/I2C/UART pins

## Recommendations

### Immediate Actions

1. Prototype minimal scheduler with Embassy (400Hz IMU task, 50Hz control task, 10Hz telemetry task)
2. Measure scheduler jitter and CPU overhead on Pico 2 W hardware
3. Design parameter storage layout (Flash blocks, redundancy, checksum)

### Next Steps

1. [ ] Create ADR-002 for scheduler selection
2. [ ] Create ADR-004 for storage strategy (wear leveling)
3. [ ] Create ADR-009 for parameter system design
4. [ ] Create ADR-010 for data logging approach
5. [ ] Create FR-013 through FR-020 for core system requirements
6. [ ] Create NFR-008 through NFR-010 for performance and reliability requirements
7. [ ] Create task T-003 for core systems implementation

### Out of Scope

The following are explicitly excluded from the initial core systems:

- **USB Logging**: Use UART/MAVLink for log download instead
- **Real-time Clock (RTC)**: Use GPS time instead of separate RTC chip
- **Over-the-Air (OTA) Firmware Updates**: Require physical reflashing initially
- **Multi-core Scheduling**: Run on single core; dual-core support can be added later
- **Advanced Parameter Types**: Support only f32 and u32 initially; enums/strings can be added later

## Appendix

### References

- ArduPilot Scheduler Documentation: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>
- ArduPilot Parameter System: <https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html>
- ArduPilot Logger: <https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html>
- Embassy Framework: <https://embassy.dev/>
- RTIC Framework: <https://rtic.rs/>
- RP2040 Flash Programming: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf> (Section 2.6)

### Raw Data

**Task Frequency Budget** (50Hz control loop):

| Task         | Frequency | Execution Time | CPU % (@ 133 MHz) |
| ------------ | --------- | -------------- | ----------------- |
| IMU Sampling | 400 Hz    | 0.1 ms         | 4%                |
| AHRS Update  | 100 Hz    | 0.5 ms         | 5%                |
| Control Loop | 50 Hz     | 2.0 ms         | 10%               |
| Navigation   | 10 Hz     | 1.0 ms         | 1%                |
| Telemetry    | 10 Hz     | 0.5 ms         | 0.5%              |
| Logger       | 5 Hz      | 5.0 ms         | 2.5%              |
| **Total**    | -         | -              | **23%**           |

Leaves 77% CPU headroom for spikes and future features.

**Flash Storage Allocation** (Pico 2 W, 4 MB Flash):

| Section           | Size   | Purpose                         |
| ----------------- | ------ | ------------------------------- |
| Firmware          | 256 KB | Application code                |
| Parameter Block 1 | 4 KB   | Current parameters              |
| Parameter Block 2 | 4 KB   | Backup parameters               |
| Mission Storage   | 8 KB   | Waypoints (200 waypoints @ 40B) |
| Log Storage       | 3.7 MB | Circular log buffer             |
