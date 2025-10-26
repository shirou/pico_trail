# AN-cp76d ArduPilot Feature Analysis for Embedded Rover/Boat Autopilot

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-kir7h-platform-abstraction](AN-kir7h-platform-abstraction.md)
  - [AN-5nucb-core-systems](AN-5nucb-core-systems.md)
- Related Requirements: N/A - Requirements will be created based on this analysis
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis examines the ArduPilot autopilot system to identify essential features for implementing a minimal viable autopilot for rover and boat vehicles on Raspberry Pi Pico W and Pico 2 W. ArduPilot is a mature, feature-rich autopilot with 153+ libraries supporting copters, planes, rovers, boats, and submarines. Given the Pico's resource constraints (264-520 KB RAM, 2-4 MB Flash), we must carefully select a subset of features that provide core autonomous navigation capabilities while fitting within hardware limitations.

Key findings: ArduPilot's architecture is centered around 8 core systems (Scheduler, AHRS, Parameter System, Logger, Storage, Safety/Failsafe, Calibration, and Notification), 5 key subsystems (Navigation, Control, Communication, Sensors, Actuators), and vehicle-specific mode implementations. For rover/boat operation, the L1 controller, simplified AHRS (DCM or basic EKF), and MAVLink communication are critical components.

## Problem Space

### Current State

The project currently has:

- Basic embedded Rust template from knurling-rs/app-template
- RP2040 (Pico W) and RP2350 (Pico 2 W) HAL dependencies configured
- Example binaries for testing basic functionality
- No autopilot functionality implemented

### Desired State

A functional autopilot system that enables:

- Autonomous waypoint navigation for rovers and boats
- MAVLink communication with ground control stations (GCS)
- Safe operation with failsafe mechanisms
- Sensor fusion for accurate position and attitude estimation
- Configurable parameters without reflashing firmware
- Data logging for flight analysis and debugging

### Gap Analysis

**Missing Core Systems**:

- Task scheduling and execution framework
- Sensor drivers (GPS, IMU, compass)
- Actuator control (motors, servos via PWM)
- Attitude estimation (AHRS/DCM/EKF)
- Navigation algorithms (L1 controller, waypoint following)
- MAVLink protocol implementation
- Parameter storage and management
- Safety and failsafe logic
- Data logging system

**Resource Constraints**:

- ArduPilot targets STM32F4/F7 with 192-512 KB RAM
- Pico W has only 264 KB RAM (50% of typical ArduPilot target)
- Must use simplified algorithms (e.g., 6-state EKF instead of 15-state)

## Stakeholder Analysis

| Stakeholder            | Interest/Need                                  | Impact | Priority |
| ---------------------- | ---------------------------------------------- | ------ | -------- |
| Rover/Boat Operators   | Reliable autonomous navigation                 | High   | P0       |
| GCS Users              | Standard MAVLink protocol compatibility        | High   | P0       |
| System Developers      | Clear architecture, testable components        | Medium | P1       |
| Hardware Experimenters | Support for multiple sensor configurations     | Medium | P2       |
| Data Analysts          | Comprehensive logging for post-flight analysis | Low    | P2       |

## Research & Discovery

### User Feedback

N/A - New project without existing user base. Requirements derived from ArduPilot community standards and common rover/boat use cases.

### Competitive Analysis

**ArduPilot Rover/Boat Features**:

- **Navigation**: S-curve path planning (ArduPilot 4.3+ primary method) for smooth waypoint transitions with velocity/acceleration limits, Position controller to follow S-curve generated paths, L1 controller (legacy) for simple path following
- **Sensors**: GPS (u-blox, NMEA), IMU (MPU6xxx, BMI xxx), Compass (HMC5883, QMC5883), Barometer (optional for altitude), Rangefinder (optional for obstacle detection)
- **Control**: Steering control (Ackermann, skid-steer), Throttle control with acceleration limits, Pivot turns for skid-steer vehicles
- **Modes**: Manual, Hold, Loiter, Auto (mission), RTL, Guided, Steering, Acro
- **Safety**: GPS failsafe, RC failsafe, Battery failsafe, Geofence (polygon, circle), Pre-arm safety checks
- **Communication**: MAVLink 1.0/2.0, Parameter protocol, Mission protocol, Telemetry streaming
- **Storage**: Parameters in EEPROM, Mission waypoints in Flash, Calibration data persistence

**Pixhawk (PX4) Approach**:

- Similar feature set to ArduPilot
- More modular architecture with uORB messaging
- Stricter real-time requirements
- Higher complexity, may be overkill for simple rovers/boats

**Recommendation**: Follow ArduPilot's architecture but with significant simplification for embedded constraints.

### Technical Investigation

**ArduPilot Core Libraries Analysis**:

1. **AP_HAL**: Hardware abstraction layer for portability across boards
2. **AP_Scheduler**: Task management with fixed-rate loops (e.g., 400Hz IMU, 50Hz control, 10Hz telemetry)
3. **AP_AHRS**: Attitude estimation using DCM or EKF, sensor fusion (gyro, accel, mag, GPS)
4. **AP_GPS**: GPS driver with NMEA and u-blox support
5. **AP_InertialSensor**: IMU data acquisition and calibration
6. **AP_Compass**: Magnetometer interface and calibration
7. **AP_InertialNav**: Position estimation combining IMU and GPS
8. **AC_PID / AR_AttitudeControl**: PID control loops for attitude and position
9. **AR_WPNav / AR_PosControl**: S-curve path planning and position control (ArduPilot 4.3+), **AP_L1_Control**: L1 navigation controller (legacy path following)
10. **GCS_MAVLink**: MAVLink protocol implementation
11. **AP_Mission**: Mission storage and execution
12. **AP_Param**: Parameter system with EEPROM storage
13. **AP_Logger**: High-frequency binary data logging
14. **AP_Motors / SRV_Channel**: Motor mixing and servo control

**Memory Requirements (Estimated)**:

| Component         | RAM Usage    | Notes                                          |
| ----------------- | ------------ | ---------------------------------------------- |
| AHRS (DCM)        | \~2 KB       | Simpler than EKF                               |
| AHRS (EKF simple) | \~8 KB       | 6-9 state EKF                                  |
| Navigation        | \~4-6 KB     | S-curve planner + position control + waypoints |
| Control (PID)     | \~1 KB       | Multiple PID controllers                       |
| MAVLink           | \~8 KB       | Message buffers + state                        |
| Parameters        | \~2 KB       | In-memory parameter cache                      |
| Logger            | \~4-16 KB    | Circular buffer                                |
| Task Stack        | \~8 KB       | Multiple task stacks                           |
| **Total Core**    | **37-53 KB** | Leaves 211-227 KB for application              |

**Feasibility**: Pico W (264 KB RAM) is marginal but feasible with careful memory management. Pico 2 W (520 KB RAM) provides comfortable headroom.

### Data Analysis

N/A - No operational data available yet.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall maintain a task scheduler with configurable task rates (1Hz to 400Hz) → Will become FR-001
  - Rationale: Critical for real-time control loop execution
  - Acceptance Criteria: Tasks execute within 5% of target period, no missed deadlines under normal load

- [ ] **FR-DRAFT-2**: System shall implement AHRS using DCM or simplified EKF for attitude estimation → Will become FR-002
  - Rationale: Accurate attitude required for navigation and control
  - Acceptance Criteria: Roll/pitch accuracy within 2 degrees, heading within 5 degrees (with mag calibration)

- [ ] **FR-DRAFT-3**: System shall support GPS navigation with waypoint following using S-curve path planning → Will become FR-003
  - Rationale: Core autonomous navigation capability, S-curves provide smoother paths than L1 for rovers/boats
  - Acceptance Criteria: Path following error < 2 meters at speeds up to 5 m/s, smooth acceleration/deceleration within configured limits

- [ ] **FR-DRAFT-4**: System shall implement MAVLink protocol for GCS communication → Will become FR-004
  - Rationale: Standard interface for mission planning and telemetry
  - Acceptance Criteria: Compatible with QGroundControl and Mission Planner

- [ ] **FR-DRAFT-5**: System shall provide runtime parameter configuration via MAVLink → Will become FR-005
  - Rationale: Tuning without reflashing firmware
  - Acceptance Criteria: Parameters persist across reboots, changeable via GCS

- [ ] **FR-DRAFT-6**: System shall log sensor data and control outputs to Flash storage → Will become FR-006
  - Rationale: Essential for debugging and performance analysis
  - Acceptance Criteria: Log at 50Hz minimum, download via MAVLink

- [ ] **FR-DRAFT-7**: System shall implement GPS and RC failsafe mechanisms → Will become FR-007
  - Rationale: Critical safety feature
  - Acceptance Criteria: Failsafe triggers within 1 second of signal loss, vehicle executes safe action (Hold or RTL)

- [ ] **FR-DRAFT-8**: System shall support at minimum 5 vehicle modes: Manual, Hold, Auto, RTL, Guided → Will become FR-008
  - Rationale: Essential operational modes for rover/boat
  - Acceptance Criteria: Mode transitions complete within 100ms, no mode switching during critical maneuvers

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Control loop latency shall not exceed 20ms (50Hz minimum) → Will become NFR-001
  - Category: Performance
  - Rationale: Required for stable control at typical rover/boat speeds
  - Target: 20ms max latency from sensor read to actuator output

- [ ] **NFR-DRAFT-2**: System shall operate within 200 KB RAM on Pico W, 400 KB on Pico 2 W → Will become NFR-002
  - Category: Resource Constraints
  - Rationale: Leave headroom for future features and stack growth
  - Target: Measured via runtime memory profiling

- [ ] **NFR-DRAFT-3**: System shall not use `unsafe` Rust except in HAL layer → Will become NFR-003
  - Category: Safety / Reliability
  - Rationale: Prevent memory safety issues, align with project principles
  - Target: Zero unsafe blocks in application/subsystem layers (audit via `grep`)

- [ ] **NFR-DRAFT-4**: Platform-specific code shall be isolated to `src/platform/` directory → Will become NFR-004
  - Category: Portability
  - Rationale: Enable support for additional platforms (ESP32, STM32)
  - Target: Zero hardware-specific code outside platform layer (enforced via code review)

- [ ] **NFR-DRAFT-5**: Critical sensor data shall be sampled at 400Hz minimum (IMU) → Will become NFR-005
  - Category: Performance
  - Rationale: ArduPilot standard, ensures smooth attitude estimation
  - Target: 400Hz IMU sampling with jitter < 1ms

## Design Considerations

### Technical Constraints

1. **Memory Limitations**:
   - Pico W: 264 KB RAM (requires aggressive optimization)
   - Pico 2 W: 520 KB RAM (comfortable for planned features)
   - No dynamic heap allocation in critical paths

2. **CPU Performance**:
   - Pico W: RP2040 @ 133 MHz (Cortex-M0+, no FPU)
   - Pico 2 W: RP2350 @ 150 MHz (Cortex-M33, with FPU)
   - FPU critical for control and navigation math

3. **Storage**:
   - Flash wear leveling required for parameter/log storage
   - Limited write cycles (10,000-100,000 depending on Flash type)

4. **Real-time Requirements**:
   - No operating system (bare metal or RTOS)
   - Deterministic task scheduling essential
   - Interrupt-driven sensor sampling

### Potential Approaches

1. **Option A: Embassy Async Framework**
   - Pros: Modern async/await, efficient task management, good community support, works on Cortex-M0+ and M33
   - Cons: Learning curve for async embedded, larger binary size
   - Effort: Medium (framework setup, async driver development)

2. **Option B: RTIC (Real-Time Interrupt-driven Concurrency)**
   - Pros: Zero-cost abstractions, compile-time scheduling, minimal overhead, proven in production
   - Cons: Steeper learning curve, less flexible than async, Cortex-M only
   - Effort: Medium (RTIC resource configuration, task priority tuning)

3. **Option C: Custom Bare-Metal Scheduler**
   - Pros: Full control, minimal overhead, tailored to exact needs
   - Cons: High development effort, reinventing the wheel, potential bugs in scheduler
   - Effort: High (scheduler implementation, testing, debugging)

**Recommendation**: Start with **Embassy** for its async model and active development, falling back to RTIC if performance/memory issues arise.

### Architecture Impact

This analysis will drive the following ADRs:

- **ADR-001**: Overall layered architecture design
- **ADR-002**: Task scheduling approach (Embassy vs RTIC vs custom)
- **ADR-003**: AHRS algorithm selection (DCM vs simplified EKF vs Madgwick)
- **ADR-004**: Storage strategy for parameters and logs (Flash wear leveling)
- **ADR-005**: MAVLink implementation approach (rust-mavlink crate vs custom)

## Risk Assessment

| Risk                                           | Probability | Impact | Mitigation Strategy                                                                                            |
| ---------------------------------------------- | ----------- | ------ | -------------------------------------------------------------------------------------------------------------- |
| Insufficient RAM on Pico W                     | Medium      | High   | Optimize memory usage, prioritize Pico 2 W, provide build-time feature flags to disable non-essential features |
| AHRS drift without magnetometer                | Medium      | Medium | Require magnetometer for heading estimation, implement mag calibration                                         |
| Flash wear from excessive parameter writes     | Low         | Medium | Implement wear leveling, cache parameters in RAM, only write on explicit save command                          |
| Scheduler jitter affecting control performance | Low         | High   | Use hardware timers for critical tasks, measure and optimize task execution times                              |
| MAVLink incompatibility with GCS               | Low         | High   | Use standard mavlink crate, test with QGroundControl and Mission Planner early                                 |

## Open Questions

- [ ] Should we support both Cortex-M0+ (Pico W) and Cortex-M33 (Pico 2 W) from day one, or focus on Pico 2 W initially? → Next step: Draft ADR-002 to evaluate task scheduler options and their platform requirements
- [ ] What is the minimum viable sensor set? GPS + IMU only, or also magnetometer? → Method: Prototype AHRS with and without mag to assess heading accuracy
- [ ] Should we implement full MAVLink mission protocol, or start with simpler waypoint list? → Next step: Create FR-004 for MAVLink requirements with phased implementation approach
- [ ] What Flash wear leveling strategy is most appropriate for RP2040/RP2350? → Method: Research rp2040-flash and embedded-storage crates, benchmark write patterns

## Recommendations

### Immediate Actions

1. Create architectural decision records (ADRs) for core system choices (scheduler, AHRS, storage)
2. Define formal requirements based on FR-DRAFT and NFR-DRAFT items above
3. Prototype minimal AHRS (DCM) on Pico 2 W to validate memory and CPU feasibility

### Next Steps

1. [ ] Create formal requirements: FR-001 through FR-008, NFR-001 through NFR-005
2. [ ] Draft ADR-001 for layered architecture
3. [ ] Draft ADR-002 for task scheduler selection (Embassy vs RTIC)
4. [ ] Draft ADR-003 for AHRS algorithm
5. [ ] Create task T-001 for foundation implementation (platform abstraction + scheduler)

### Out of Scope

The following ArduPilot features are explicitly excluded to reduce complexity and memory usage:

- **Full 15-state EKF**: Too memory-intensive, use DCM or 6-state EKF instead
- **Optical Flow**: Not required for outdoor rover/boat navigation
- **Terrain Following**: Not applicable to ground/water vehicles
- **Advanced Missions**: Spline waypoints, rally points, complex conditionals
- **Lua Scripting**: Insufficient RAM for embedded Lua interpreter
- **Multiple GPS Support**: Single GPS sufficient for rover/boat
- **CAN Bus Peripherals**: Use I2C/SPI sensors instead
- **Advanced Tuning**: Autotune, notch filters for propeller noise

## Appendix

### References

- ArduPilot Documentation: <https://ardupilot.org/dev/>
- ArduPilot Rover Documentation: <https://ardupilot.org/rover/>
- ArduPilot Libraries: <https://github.com/ArduPilot/ardupilot/tree/master/libraries>
- ArduPilot S-Curve Implementation: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_WPNav>
- MAVLink Protocol: <https://mavlink.io/en/>
- Embassy Async Framework: <https://embassy.dev/>
- RTIC Framework: <https://rtic.rs/>
- L1 Controller Paper: "A new nonlinear guidance logic for trajectory tracking" by Park, Deyst, and How (MIT)

### Raw Data

**ArduPilot Core Libraries** (from <https://ardupilot.org/dev/docs/apmcopter-programming-libraries.html>):

Core libraries:

- AP_AHRS - attitude estimation using DCM or EKF
- AP_Common - core includes required by all sketches and libraries
- AP_Math - vector manipulation functions
- AC_PID - PID controller library
- AP_InertialNav - inertial navigation blending accel/GPS/baro
- AC_AttitudeControl - attitude and position control
- AC_WPNav - waypoint navigation library
- AP_Motors - motor mixing for multicopter/helicopter
- RC_Channel - convert PWM input/output to angles
- AP_HAL - Hardware abstraction layer

Sensor libraries:

- AP_InertialSensor - gyro and accelerometer interface
- AP_RangeFinder - sonar/IR distance sensors
- AP_Baro - barometer interface
- AP_GPS - GPS interface
- AP_Compass - magnetometer interface
- AP_OpticalFlow - optical flow sensor interface

Other libraries:

- AP_Mount, AP_Camera, AP_Relay - camera control
- AP_Mission - mission storage/retrieval from EEPROM

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
