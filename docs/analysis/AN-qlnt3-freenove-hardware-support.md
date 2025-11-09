# AN-qlnt3 Freenove 4WD Car (Standard Wheel) Hardware Support

## Metadata

- Type: Analysis
- Status: Approved
- Decision: Option B - New Motor Driver Module

## Links

- Related Analyses:
  - [AN-kir7h-platform-abstraction](AN-kir7h-platform-abstraction.md)
- Related Requirements: N/A - Requirements will be created based on this analysis
- Related ADRs:
  - [ADR-4hx3d-motor-driver-abstraction](../adr/ADR-4hx3d-motor-driver-abstraction.md)
  - [ADR-2l5fh-differential-drive-kinematics](../adr/ADR-2l5fh-differential-drive-kinematics.md)
  - [ADR-mcg03-pin-configuration-management](../adr/ADR-mcg03-pin-configuration-management.md)
- Related Tasks:
  - [T-po5ns-pin-configuration](../tasks/T-po5ns-pin-configuration/README.md)
  - [T-l7czt-differential-drive-kinematics](../tasks/T-l7czt-differential-drive-kinematics/README.md)
  - [T-vf57h-hbridge-motor-control](../tasks/T-vf57h-hbridge-motor-control/README.md)

## Executive Summary

This analysis examines the requirements and approach for supporting the Freenove 4WD Car (FNK0089) Standard Wheel hardware platform. The current pico_trail implementation assumes PWM-based servo/ESC control (typical for hobby RC vehicles), but the Freenove platform uses H-bridge motor drivers (DRV8837) with 4 independent DC motors arranged in a differential drive (skid-steer) configuration.

Key findings: The project requires an H-bridge motor driver abstraction with PWM-based speed control and differential drive kinematics implementation. The Freenove platform uses 8 GPIO pins for motor control (2 pins per motor × 4 motors) with PWM signals for speed regulation. A new motor control layer is needed that translates steering/throttle commands into left/right motor group commands with PWM duty cycle control.

## Problem Space

### Current State

The pico_trail project currently supports:

- **Actuator Model**: PWM-based servo (steering) and ESC (throttle) control via `src/libraries/srv_channel/`
- **Control Channels**: 2-channel control (steering + throttle)
- **Output Interface**: PWM duty cycle (1000-2000μs pulse widths converted to 0.0-1.0 duty cycle)
- **Platform Support**: RP2350 (Pico 2 W) with abstracted PWM interface
- **Pin Usage**: Platform trait defines SPI0 on GPIO16/18/19 (conflicts with Freenove hardware)

### Desired State

Support for Freenove 4WD Car (Standard Wheel) hardware:

- **Hardware Specifications**:
  - 4× DC motors with H-bridge drivers (DRV8837 - Texas Instruments Low-Voltage H-Bridge)
  - Motor arrangement: M1/M2 (left side), M3/M4 (right side)
  - Control: 2 PWM-capable GPIO pins per motor (IN1, IN2) for direction and speed
  - Speed control: PWM duty cycle on IN1/IN2 pins (0-100% speed range)
  - Additional components: Buzzer (GPIO2), WS2812 LED (GPIO16), Battery ADC (GPIO26)

- **Pin Assignments** (from Freenove documentation):
  - M1 (left-front): GPIO18 (IN1), GPIO19 (IN2)
  - M2 (left-rear): GPIO20 (IN1), GPIO21 (IN2)
  - M3 (right-front): GPIO6 (IN1), GPIO7 (IN2)
  - M4 (right-rear): GPIO8 (IN1), GPIO9 (IN2)

- **Control Model**: Differential drive (skid-steer) where steering/throttle commands are converted to left/right motor group speeds

### Gap Analysis

**Missing Components**:

1. **H-bridge Motor Driver Abstraction**: No PWM-based H-bridge motor control implementation exists
2. **Differential Drive Kinematics**: No steering/throttle → left/right conversion logic
3. **Motor Group Management**: No abstraction for grouping motors into left/right sides
4. **PWM Speed Control**: Need to support variable speed via PWM duty cycle (not just on/off)
5. **Pin Configuration**: Current platform traits hardcode SPI0 pins that conflict with motor pins

**Implementation Gaps**:

- `src/libraries/srv_channel/`: Designed for PWM actuators, not H-bridge motors
- `src/platform/rp2350/platform.rs:86-100`: SPI0 uses GPIO16/18/19 (conflicts with LED and M1 motor)
- No motor driver traits or implementations in platform layer
- No differential drive control mode implementation

## Stakeholder Analysis

| Stakeholder             | Interest/Need                                           | Impact | Priority |
| ----------------------- | ------------------------------------------------------- | ------ | -------- |
| Freenove Hardware Users | Ability to test pico_trail on available hardware        | High   | P0       |
| Developers              | Clear separation between motor driver and control logic | High   | P0       |
| Future Hardware Ports   | Reusable H-bridge abstraction for other platforms       | Medium | P1       |
| Testing/CI              | Example configurations for hardware testing             | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - This is the first hardware platform being tested with pico_trail.

### Competitive Analysis

**ArduPilot Rover Differential Drive**:

- Supports differential drive via `SkidSteerOutput` class
- Steering/throttle mixing: `left = throttle + steering`, `right = throttle - steering`
- Motor abstraction via `SRV_Channel` (supports both PWM and DShot)
- Pros: Well-tested mixing algorithm, supports multiple motor protocols
- Cons: Complex class hierarchy, C++ implementation

**PX4 Differential Drive Controller**:

- Uses `DifferentialDriveKinematics` for mixing
- Supports both PWM and UAVCAN motor control
- Implements rate limiting and deadband handling
- Pros: Modern architecture, clean separation of concerns
- Cons: Requires significant infrastructure (UAVCAN stack for non-PWM)

**embedded-motor-driver Crate**:

- Rust traits for motor control: `Motor`, `DirectionalMotor`
- H-bridge support via `HBridge` struct
- Pros: Type-safe Rust interface, simple API
- Cons: Limited adoption, minimal documentation

### Technical Investigation

**H-bridge Control Logic** (DRV8837 - Texas Instruments):

| IN1 | IN2 | Motor State                                |
| --- | --- | ------------------------------------------ |
| 0   | 0   | Coast (High-Z, motor freewheels)           |
| PWM | 0   | Forward (speed = PWM duty cycle)           |
| 0   | PWM | Reverse (speed = PWM duty cycle)           |
| 1   | 1   | Brake (short brake, both terminals to GND) |
| PWM | PWM | Brake with PWM (for regenerative braking)  |

**Key Differences from TB6612FNG**:

- DRV8837 uses PWM on IN1/IN2 for speed control (not separate PWM + direction pins)
- 0/0 state = Coast (not Brake), allows motor to freewheel
- 1/1 state = Brake (short circuit braking)

**Differential Drive Mixing Algorithm**:

```
// Input: steering (-1.0 to +1.0), throttle (-1.0 to +1.0)
// Output: left_speed, right_speed (-1.0 to +1.0)

left_speed = throttle + steering
right_speed = throttle - steering

// Normalize to [-1.0, +1.0] range
max_magnitude = max(abs(left_speed), abs(right_speed))
if max_magnitude > 1.0 {
    left_speed /= max_magnitude
    right_speed /= max_magnitude
}
```

**Freenove Hardware Validation** (from documentation and hardware inspection):

- Hardware uses **DRV8837** H-bridge motor drivers (Texas Instruments)
- 4 motors wired in left/right groups
- **PWM speed control required**: RP2040_PWM library used in Freenove examples
- Speed control: -100 to +100 range (positive = forward, negative = reverse)
- PWM duty cycle directly controls motor speed (higher duty = higher speed)

### Data Analysis

N/A - No usage metrics available for new hardware platform.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: H-bridge motor driver interface with PWM speed control → Will become FR-<id>
  - Rationale: Need PWM-based H-bridge motor control for Freenove DRV8837 hardware
  - Acceptance Criteria:
    - Support forward, reverse, brake, coast states
    - Control via 2 PWM-capable GPIO pins per motor (IN1, IN2)
    - Variable speed control via PWM duty cycle (0-100% range)
    - Type-safe interface preventing invalid state combinations
    - Platform-agnostic trait implementation

- [ ] **FR-DRAFT-2**: Differential drive kinematics → Will become FR-<id>
  - Rationale: Convert steering/throttle to left/right motor commands
  - Acceptance Criteria:
    - Accept steering (-1.0 left, +1.0 right) and throttle (-1.0 reverse, +1.0 forward)
    - Output left/right motor speeds with proper normalization
    - Handle edge cases (full steering + full throttle)
    - Match ArduPilot's differential drive behavior

- [ ] **FR-DRAFT-3**: Motor group abstraction → Will become FR-<id>
  - Rationale: Manage multiple motors as coordinated groups (left side, right side)
  - Acceptance Criteria:
    - Group 2+ motors with identical speed commands
    - Support independent per-motor control if needed (future: individual motor trim)
    - Enforce armed state check (safety requirement)

- [ ] **FR-DRAFT-4**: Freenove pin configuration → Will become FR-<id>
  - Rationale: Provide validated pin mappings for Freenove Standard Wheel platform
  - Acceptance Criteria:
    - Document all GPIO assignments (motors, buzzer, LED, ADC)
    - Validate no pin conflicts with required peripherals (UART, I2C)
    - Provide example initialization code
    - Include hardware validation test

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Zero-cost motor abstraction → Will become NFR-<id>
  - Category: Performance
  - Rationale: Embedded system requires minimal runtime overhead
  - Target: H-bridge trait calls should compile to direct GPIO writes (no virtual dispatch)

- [ ] **NFR-DRAFT-2**: Safety enforcement → Will become NFR-<id>
  - Category: Safety
  - Rationale: Motors must not run when system is disarmed
  - Target: Armed state check enforced at motor driver layer (cannot be bypassed)

- [ ] **NFR-DRAFT-3**: Platform portability → Will become NFR-<id>
  - Category: Maintainability
  - Rationale: H-bridge support should work on any platform implementing GPIO traits
  - Target: Motor driver implementation independent of RP2350-specific code

## Design Considerations

### Technical Constraints

**Hardware Constraints**:

- Freenove uses 8 PWM-capable GPIO pins for motor control (scarce resource on RP2350)
- Hardware PWM required for speed control (RP2350 has 12 PWM channels, sufficient for 8 motors)
- Left/right motor groups must be synchronized (parallel PWM writes)
- PWM frequency: Typically 20-50 kHz for DC motors (avoid audible noise)

**Software Constraints**:

- Must work with existing Platform abstraction (`src/platform/traits/`)
- Must integrate with SystemState armed checks (`src/communication/mavlink/state.rs`)
- Must support PWM-based speed control from initial implementation (required by DRV8837)

**Safety Constraints**:

- Motors must stop immediately when disarmed (hard requirement)
- Invalid H-bridge states (both IN pins high during transitions) should be prevented

### Potential Approaches

#### Option A: Extend srv_channel for H-bridge Support

**Description**: Add H-bridge backend to existing `ActuatorInterface` in `src/libraries/srv_channel/`

**Pros**:

- Reuses existing actuator abstraction and safety checks
- Minimal architectural changes
- Differential drive could be implemented as a new control mode

**Cons**:

- `srv_channel` is conceptually PWM-focused (pulse widths, duty cycles)
- Name "srv_channel" (servo channel) misleading for DC motors
- Mixes two different actuator paradigms in one module

**Effort**: Low

#### Option B: New Motor Driver Module (Recommended)

**Description**: Create `src/libraries/motor_driver/` with H-bridge and differential drive support

**Pros**:

- Clean separation: `srv_channel` for PWM actuators, `motor_driver` for H-bridge motors
- Clear naming that reflects purpose
- Easy to add future motor types (DShot ESCs, stepper motors)
- Better aligns with platform abstraction philosophy

**Cons**:

- Requires new module and trait definitions
- Slightly more initial implementation effort
- Need to decide where differential drive logic lives (motor_driver vs control layer)

**Effort**: Medium

#### Option C: Platform-Specific Motor Implementation

**Description**: Implement H-bridge control directly in `src/platform/rp2350/motor.rs`

**Pros**:

- Fast to implement for immediate testing
- No cross-cutting abstraction design needed

**Cons**:

- Not portable to other platforms
- Violates platform abstraction principles
- Duplicates safety logic if other platforms need motors
- Technical debt that will require refactoring later

**Effort**: Low (short-term), High (long-term refactoring cost)

### Architecture Impact

**New ADRs Required**:

1. **ADR: Motor Driver Abstraction Strategy**
   - Decision: Choose between Option A (extend srv_channel) vs Option B (new motor_driver module)
   - Context: Need to support both PWM actuators (future RC vehicles) and H-bridge motors (Freenove)
   - Recommendation: Option B (new module) for clean separation

2. **ADR: Differential Drive Implementation Location**
   - Decision: Where does steering/throttle → left/right mixing live?
   - Options:
     - In motor_driver library (tightly coupled to motor control)
     - In rover/mode layer (control logic)
     - As a standalone kinematics library (reusable)
   - Recommendation: Standalone kinematics library (`src/libraries/kinematics/differential_drive.rs`)

3. **ADR: Pin Configuration Management**
   - Decision: How to handle platform-specific pin assignments?
   - Options:
     - Hardcoded in platform module
     - Configuration file/const
     - Runtime configuration via parameters (ArduPilot style)
   - Recommendation: Const definitions per platform in `src/platform/rp2350/pins/freenove.rs`

**Module Structure Impact**:

```
src/
├── libraries/
│   ├── motor_driver/          # NEW: H-bridge motor control
│   │   ├── mod.rs             # Motor traits and safety
│   │   └── hbridge.rs         # H-bridge implementation
│   ├── kinematics/            # NEW: Drive kinematics
│   │   ├── mod.rs
│   │   └── differential.rs    # Differential drive mixing
│   └── srv_channel/           # EXISTING: PWM actuators (unchanged)
├── platform/
│   └── rp2350/
│       ├── pins/              # NEW: Pin configuration presets
│       │   ├── mod.rs
│       │   └── freenove.rs    # Freenove Standard Wheel pins
│       └── motor.rs           # NEW: RP2350 motor driver implementation
```

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                                      |
| -------------------------------------------- | ----------- | ------ | -------------------------------------------------------- |
| H-bridge abstraction too complex             | Medium      | Medium | Start with minimal trait (forward/reverse/brake only)    |
| Pin conflicts break existing functionality   | High        | High   | Remove unused SPI0 initialization, validate pin mappings |
| Differential drive mixing incorrect          | Medium      | High   | Unit test against ArduPilot reference implementation     |
| Motor safety bypass possible                 | Low         | High   | Enforce armed checks in motor driver trait impl          |
| Performance overhead from abstraction layers | Low         | Medium | Use const generics and inline functions for zero-cost    |

## Open Questions

- [x] Should H-bridge support PWM speed control initially, or just on/off (full speed)?
  - **Decision**: PWM speed control required from initial implementation
  - **Rationale**: DRV8837 uses PWM on IN1/IN2 for speed regulation, Freenove examples demonstrate this
  - **Status**: Confirmed via hardware inspection and documentation review

- [ ] Do we need per-motor trim adjustments for straight-line driving?
  - Investigation: Check if Freenove examples include trim/calibration
  - Next step: Test hardware to measure motor variation

- [ ] Should buzzer and LED be part of this implementation or separate?
  - Decision: Treat as optional peripherals, not required for motor testing
  - Next step: Defer to future analysis/task

- [ ] How to handle transition from PWM-based (srv_channel) to motor-based examples?
  - Decision: Keep both implementations, add feature flags to select actuator backend
  - Next step: Draft ADR for feature flag structure

## Recommendations

### Immediate Actions

1. **Create Motor Driver Module** (`src/libraries/motor_driver/`)
   - Define `Motor` trait with forward/reverse/brake/coast methods
   - Implement `HBridgeMotor` using 2 PWM-capable GPIO pins (DRV8837)
   - Support variable speed via PWM duty cycle (0-100%)
   - Add safety enforcement (armed state check)

2. **Fix Pin Conflicts**
   - Remove SPI0 initialization from `src/platform/rp2350/platform.rs`
   - Create Freenove pin configuration in `src/platform/rp2350/pins/freenove.rs`

3. **Implement Differential Drive**
   - Create `src/libraries/kinematics/differential_drive.rs`
   - Implement steering/throttle → left/right mixing with normalization
   - Add unit tests against ArduPilot reference values

### Next Steps

1. [ ] Create formal requirements: FR-<id> for motor driver, differential drive, pin config
2. [ ] Draft ADR for: Motor driver abstraction strategy (Option B recommended)
3. [ ] Draft ADR for: Differential drive implementation location (kinematics library recommended)
4. [ ] Draft ADR for: Pin configuration management (const definitions recommended)
5. [ ] Create task for: Freenove hardware support implementation
6. [ ] Create task for: Hardware validation testing (motor movement, safety checks)

### Out of Scope

**Explicitly excluded from this analysis**:

- **Advanced motor control**: PID tuning, motor feedback (encoders), current limiting
  - Rationale: PWM speed control is included in initial implementation, but advanced features deferred

- **Buzzer and LED support**: GPIO2 (buzzer), GPIO16 (WS2812 LED)
  - Rationale: Not required for motor testing, can be added as separate peripherals later

- **Battery monitoring**: GPIO26 ADC for voltage sensing
  - Rationale: Separate from motor control, should be handled by telemetry/failsafe systems

- **4-wheel independent control**: Individual speed for each motor (like mecanum wheels)
  - Rationale: Standard wheels use differential drive (left/right groups only)

## Appendix

### References

**Freenove Documentation**:

- Product: FNK0089 4WD Smart Car Kit for Raspberry Pi Pico
- Module Test Documentation: <https://docs.freenove.com/projects/fnk0089/en/latest/fnk0089/codes/Standard/2_Module_test_.html>
- Pin assignments validated from official documentation

**ArduPilot References**:

- SkidSteer Output: `libraries/AP_Motors/AP_MotorsUGV.cpp`
- Differential drive mixing: `void AP_MotorsUGV::output_skid_steering()`
- Parameter reference: <https://ardupilot.org/rover/docs/parameters.html#skid-steer-output-parameters>

**Rust Crates**:

- `embedded-hal` GPIO traits: <https://docs.rs/embedded-hal/latest/embedded_hal/digital/>
- `rp235x-hal` GPIO implementation: <https://docs.rs/rp235x-hal/latest/rp235x_hal/gpio/>

### Raw Data

**Freenove Motor Pin Mapping** (verified from documentation):

```rust
// Left side (M1, M2)
const M1_IN1: u8 = 18;  // GPIO18
const M1_IN2: u8 = 19;  // GPIO19
const M2_IN1: u8 = 20;  // GPIO20
const M2_IN2: u8 = 21;  // GPIO21

// Right side (M3, M4)
const M3_IN1: u8 = 6;   // GPIO6
const M3_IN2: u8 = 7;   // GPIO7
const M4_IN1: u8 = 8;   // GPIO8
const M4_IN2: u8 = 9;   // GPIO9

// Optional peripherals
const BUZZER: u8 = 2;   // GPIO2
const LED_WS2812: u8 = 16;  // GPIO16
const BAT_ADC: u8 = 26;  // GPIO26 (ADC0)
```

**H-bridge Truth Table** (DRV8837):

```
IN1  | IN2  | Function
-----|------|------------------------------------------------------
 L   |  L   | Coast (High-Z, motor freewheels)
 PWM |  L   | Forward (speed = PWM duty cycle)
 L   | PWM  | Reverse (speed = PWM duty cycle)
 H   |  H   | Brake (short brake, both motor terminals to GND)
 PWM | PWM  | Brake with PWM (regenerative braking)
```

**DRV8837 Specifications**:

- Output current: Up to 1.8A per channel
- Motor voltage (VM): 0-11V
- Logic voltage (VCC): 1.8-7V
- Sleep mode: 120 nA (via nSLEEP pin)
- Package: WSON-8

**Differential Drive Test Cases** (for unit tests):

```
steering=0.0, throttle=0.5   => left=0.5, right=0.5    (straight forward)
steering=0.5, throttle=0.5   => left=1.0, right=0.0    (forward + right turn)
steering=1.0, throttle=0.0   => left=1.0, right=-1.0   (spin right in place)
steering=-1.0, throttle=0.0  => left=-1.0, right=1.0   (spin left in place)
```

---

## Template Usage

This analysis follows the structure defined in [docs/templates/analysis.md](../templates/analysis.md).
