# AN-00167 Hold Mode

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00012-mode-capability-system](AN-00012-mode-capability-system.md)
  - [AN-00013-mode-entry-validation](AN-00013-mode-entry-validation.md)
  - [AN-00014-mode-lifecycle-management](AN-00014-mode-lifecycle-management.md)
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
- Related Requirements:
  - [FR-00168-hold-mode-actuator-stop](../requirements/FR-00168-hold-mode-actuator-stop.md)
  - [FR-00169-hold-mode-identity](../requirements/FR-00169-hold-mode-identity.md)
  - [NFR-00170-hold-mode-performance](../requirements/NFR-00170-hold-mode-performance.md)
- Related ADRs: N/A – straightforward implementation, no architectural decision needed
- Related Tasks:
  - [T-00171-hold-mode](../tasks/T-00171-hold-mode/README.md)

## Executive Summary

This analysis examines the implementation of Hold mode as a `Mode` trait implementation struct. The `FlightMode::Hold` enum variant already exists in the codebase and is used by the battery failsafe system, motor control runner, and MAVLink state reporting (custom mode number 4). However, no `HoldMode` struct implementing the `Mode` trait exists, meaning Hold mode cannot be selected via `ModeExecutor::set_mode()`.

Hold mode is the simplest possible mode: zero all actuator outputs and remain stationary. It requires no GPS, no navigation controller, and no RC input. This makes it the ideal failsafe fallback for situations where other modes cannot be entered.

## Problem Space

### Current State

**FlightMode::Hold enum variant:**

- Defined in `crates/core/src/autopilot/state.rs` with custom mode number 4
- Referenced by `MotorControlRunner::step()` which returns zero motor output with `should_stop: true`
- Used as battery failsafe target (`BatteryFailsafeAction::Hold`)
- Mapped in MAVLink `to_custom_mode()` / `from_custom_mode()` conversions
- GCS can request Hold mode via `SET_MODE` command

**Missing `HoldMode` struct:**

- No `crates/core/src/mode/hold.rs` file exists
- No struct implementing `Mode` trait for Hold behavior
- Cannot be instantiated via `ModeExecutor::set_mode(Box::new(HoldMode::new(...)))`

**Existing mode implementations for reference:**

- `ManualMode` in `crates/core/src/mode/manual.rs` — simplest existing mode (RC passthrough)
- All modes implement `Mode` trait with `enter()`, `update()`, `exit()`, `name()` methods
- Modes take `Box<dyn ActuatorInterface>` for actuator control

**Where Hold is used today (without a Mode implementation):**

- `MotorControlRunner::step()` short-circuits to zero output for `FlightMode::Hold`
- Battery failsafe sets `FlightMode::Hold` directly in `SystemState`
- SITL `execute_mode()` falls through to ModeExecutor for non-Guided/Auto modes

### Desired State

A `HoldMode` struct that:

- Implements the `Mode` trait
- Zeros all actuator outputs on `enter()` and `update()`
- Can be instantiated and passed to `ModeExecutor::set_mode()`
- Serves as the reliable fallback mode when other modes fail to enter
- Works without GPS, RC input, or any external dependencies

### Gap Analysis

| Component                | Current State                         | Desired State                      | Gap         |
| ------------------------ | ------------------------------------- | ---------------------------------- | ----------- |
| `FlightMode::Hold` enum  | Exists (custom mode 4)                | No change needed                   | None        |
| `HoldMode` struct        | Does not exist                        | `Mode` trait implementation        | New file    |
| Motor control            | `MotorControlRunner` handles Hold     | No change needed                   | None        |
| ModeExecutor integration | Cannot create `HoldMode` instance     | Instantiable via `HoldMode::new()` | Constructor |
| Failsafe fallback        | Sets `FlightMode::Hold` in state only | Also transition `ModeExecutor`     | Integration |
| SITL mode dispatch       | Falls to ModeExecutor wildcard        | No change needed                   | None        |

## Stakeholder Analysis

| Stakeholder     | Interest/Need                              | Impact | Priority |
| --------------- | ------------------------------------------ | ------ | -------- |
| Failsafe System | Reliable fallback mode for battery/RC loss | High   | P0       |
| Field Operators | Safe stop without switching to Manual      | High   | P0       |
| GCS Users       | Standard ArduPilot Hold mode via SET_MODE  | Medium | P1       |
| Arming System   | Safe mode during arm/disarm procedures     | Medium | P1       |

## Research & Discovery

### User Feedback

N/A — Derived from ArduPilot standard functionality and feature backlog (FB-016).

### Competitive Analysis

**ArduPilot Hold Mode (Rover):**

From official documentation:

- Vehicle stops all movement
- Steering outputs are set to SERVOx_TRIM values (neutral)
- Motor outputs are set to SERVOx_TRIM values (zero throttle)
- No GPS required
- No active position correction (contrast with Loiter)
- Primary use cases: arming/disarming safely, failsafe destination
- ArduPilot custom mode number: 4

**Key Behavioral Properties:**

| Property                   | Value                            |
| -------------------------- | -------------------------------- |
| GPS required               | No                               |
| RC input required          | No                               |
| Active position correction | No                               |
| Actuator state             | Neutral (steering=0, throttle=0) |
| Entry conditions           | Always succeeds                  |
| ArduPilot mode number      | 4                                |

**Difference from Related Modes:**

| Aspect               | Hold Mode               | Manual Mode     | Loiter Mode        |
| -------------------- | ----------------------- | --------------- | ------------------ |
| Motor output         | Always zero             | RC passthrough  | Zero or correction |
| GPS requirement      | None                    | None            | Required           |
| RC requirement       | None                    | Required        | None               |
| Position holding     | Passive (no correction) | N/A             | Active (Type 1)    |
| Complexity           | Minimal                 | Low             | Medium             |
| Failsafe suitability | Excellent               | Poor (needs RC) | Requires GPS       |

### Technical Investigation

#### HoldMode Structure

Hold mode is architecturally identical to the simplest case: zero all outputs on every cycle.

```rust
pub struct HoldMode {
    actuators: Box<dyn ActuatorInterface>,
}
```

**Why this is so simple:**

1. No state machine needed — always in the same state (stopped)
2. No external data dependencies — no GPS, no RC, no navigation
3. No parameters — no `HOLD_*` ArduPilot parameters exist
4. `enter()` always succeeds — no preconditions to validate
5. `update()` is trivially zero output — idempotent
6. `exit()` is also zero output — same as update

#### Integration Points

**1. ModeExecutor (mode transition):**

```rust
// Creating and switching to Hold mode
let hold = Box::new(HoldMode::new(actuators));
mode_executor.set_mode(hold)?;
```

**2. MotorControlRunner (already handled):**

The `MotorControlRunner::step()` already returns zero output for `FlightMode::Hold`. The `HoldMode::update()` will also set actuators to zero, providing defense-in-depth.

**3. Battery Failsafe:**

Currently sets `FlightMode::Hold` in `SystemState`. The `HoldMode` struct enables proper mode transition via `ModeExecutor::set_mode()` in addition to the state flag.

**4. SITL `execute_mode()`:**

Hold falls into the wildcard `_ =>` arm which runs `ModeExecutor`. Once `HoldMode` is set as the active mode, the executor will call `HoldMode::update()`.

### Data Analysis

**Memory Requirements:**

| Component         | Size  | Notes                                          |
| ----------------- | ----- | ---------------------------------------------- |
| `HoldMode` struct | \~16B | One `Box<dyn ActuatorInterface>` (fat pointer) |
| Total             | \~16B | Smallest possible mode                         |

**CPU Requirements:**

| Operation  | Cost                | Notes                      |
| ---------- | ------------------- | -------------------------- |
| `enter()`  | \~2 actuator writes | Set steering=0, throttle=0 |
| `update()` | \~2 actuator writes | Set steering=0, throttle=0 |
| `exit()`   | \~2 actuator writes | Set steering=0, throttle=0 |

## Discovered Requirements

### Functional Requirements

- [x] **FR-00168-hold-mode-actuator-stop**: System shall provide a Hold mode that stops all actuator output
  - See: [FR-00168-hold-mode-actuator-stop](../requirements/FR-00168-hold-mode-actuator-stop.md)

- [x] **FR-00169-hold-mode-identity**: Hold mode shall report its identity as "Hold" for telemetry and logging
  - See: [FR-00169-hold-mode-identity](../requirements/FR-00169-hold-mode-identity.md)

### Non-Functional Requirements

- [x] **NFR-00170-hold-mode-performance**: Hold mode shall use minimal memory and execute in constant time
  - See: [NFR-00170-hold-mode-performance](../requirements/NFR-00170-hold-mode-performance.md)

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- Mode trait interface: `enter()`, `update(dt)`, `exit()`, `name()`
- `ActuatorInterface` with `set_steering()` and `set_throttle()`
- `ModeExecutor` manages mode lifecycle with `Box<dyn Mode>`
- All modes live in `crates/core/src/mode/`

**No Parameters Needed:**

ArduPilot's Hold mode has no dedicated parameters. The behavior is fixed: zero all outputs. This aligns with the project's principle of following ArduPilot parameter standards — no custom parameters should be created for Hold mode.

**No `can_enter()` Needed:**

Unlike `AutoMode::can_enter()` or `GuidedMode::can_enter()` which validate GPS, Hold mode has no preconditions. The `enter()` method always succeeds, making a separate `can_enter()` validation unnecessary.

### Potential Approaches

#### Approach A: Standalone HoldMode Struct (Recommended)

**Description:** Create a minimal `HoldMode` struct with `ActuatorInterface` dependency, following the same pattern as `ManualMode`.

**Pros:**

- Consistent with existing mode patterns
- Clear, self-contained implementation
- Easy to test in isolation
- Works as ModeExecutor-managed mode

**Cons:**

- None significant

**Effort:** Very Low (< 1 hour)

#### Approach B: Shared NullMode / StoppedMode Generic

**Description:** Create a generic "stopped" mode that could be reused for any mode needing zero output.

**Pros:**

- Could share code with initial state of other modes

**Cons:**

- Over-engineering for a trivial implementation
- Each mode has unique lifecycle needs
- Adds unnecessary abstraction

**Effort:** Low

### Architecture Impact

**New Components:**

- `crates/core/src/mode/hold.rs`: HoldMode implementation (\~50 lines)

**Modified Components:**

- `crates/core/src/mode/mod.rs`: Add `pub mod hold;` and re-export `HoldMode`

**No Impact On:**

- `FlightMode` enum (already has `Hold` variant)
- `MotorControlRunner` (already handles `FlightMode::Hold`)
- MAVLink state mapping (already has custom mode 4)
- Parameters (no Hold-specific parameters)

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                           |
| -------------------------------------------- | ----------- | ------ | --------------------------------------------- |
| Actuator writes fail silently                | Very Low    | Low    | Log errors from `set_steering`/`set_throttle` |
| Conflict with MotorControlRunner zero output | Very Low    | None   | Both produce same result (defense-in-depth)   |

## Open Questions

- [x] Does ArduPilot Hold mode have any parameters? → No, behavior is fixed
- [x] Should Hold mode require GPS? → No, it must work without any sensors
- [x] Is an ADR needed? → No, this is a straightforward implementation following existing patterns
- [x] Should `can_enter()` be implemented? → No, Hold mode has no preconditions

## Recommendations

### Immediate Actions

1. **Create `HoldMode` struct:**
   - New file: `crates/core/src/mode/hold.rs`
   - Implement `Mode` trait: zero actuators on enter/update/exit
   - Constructor: `HoldMode::new(actuators: Box<dyn ActuatorInterface>)`

2. **Register module:**
   - Add `pub mod hold;` to `crates/core/src/mode/mod.rs`
   - Re-export `HoldMode` from the module

3. **Add unit tests:**
   - Test enter always succeeds
   - Test update zeros actuators
   - Test exit zeros actuators
   - Test name returns "Hold"

### Next Steps

1. [x] Create formal requirements after analysis approval
   - Created: FR-00168, FR-00169, NFR-00170
2. [x] Create task package with design and plan
   - Created: [T-00171-hold-mode](../tasks/T-00171-hold-mode/README.md)
3. [x] Implement HoldMode (single phase, very small scope)
4. [x] Verify embedded build passes

### Out of Scope

- **Position holding**: Hold mode is passive stop only; Loiter mode handles position correction
- **Parameter configuration**: ArduPilot Hold mode has no parameters
- **Mode switching logic**: How/when to switch to Hold mode is handled by failsafe and command handler systems
- **Brake functionality**: ArduPilot has a separate "Brake" concept; Hold simply zeros outputs

## Appendix

### References

**ArduPilot Documentation:**

- [Hold Mode (Rover)](https://ardupilot.org/rover/docs/hold-mode.html)

**Related Project Documents:**

- [AN-00014-mode-lifecycle-management](AN-00014-mode-lifecycle-management.md): Mode lifecycle patterns
- [AN-00011-failsafe-system](AN-00011-failsafe-system.md): Failsafe system using Hold as fallback
- [FB-016 Feature Backlog](../feature-backlog.md): Original Hold mode feature request

### Raw Data

**HoldMode Implementation Sketch:**

```rust
pub struct HoldMode {
    actuators: Box<dyn ActuatorInterface>,
}

impl HoldMode {
    pub fn new(actuators: Box<dyn ActuatorInterface>) -> Self {
        Self { actuators }
    }
}

impl Mode for HoldMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Hold"
    }
}
```

**Existing MotorControlRunner Hold handling (defense-in-depth):**

```rust
// crates/core/src/motor/runner.rs, line 73
FlightMode::Hold => {
    return MotorControlOutput {
        motor_speeds: [0.0; 4],
        should_stop: true,
    };
}
```
