# AN-00044 Guided Mode Heading Oscillation

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00037-ahrs-navigation-control-integration](AN-00037-ahrs-navigation-control-integration.md)
  - [AN-00038-compass-calibration-via-mission-planner](AN-00038-compass-calibration-via-mission-planner.md)
  - [AN-00003-navigation-approach](AN-00003-navigation-approach.md)
  - [AN-00023-position-target-navigation](AN-00023-position-target-navigation.md)
- Related Requirements:
  - [FR-00083-guided-mode-navigation](../requirements/FR-00083-guided-mode-navigation.md)
  - [FR-00084-navigation-controller](../requirements/FR-00084-navigation-controller.md)
- Related ADRs:
  - [ADR-00033-heading-source-integration](../adr/ADR-00033-heading-source-integration.md)
  - [ADR-00022-navigation-controller-architecture](../adr/ADR-00022-navigation-controller-architecture.md)
- Related Tasks:
  - [T-00033-heading-source-navigation-integration](../tasks/T-00033-heading-source-navigation-integration/README.md)
  - [T-00038-fix-guided-mode-heading-oscillation](../tasks/T-00038-fix-guided-mode-heading-oscillation/README.md)
  - [T-00039-break-spin-feedback-loop](../tasks/T-00039-break-spin-feedback-loop/README.md)
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Executive Summary

This analysis investigates a critical field-observed bug: the rover spins in circles during Guided mode navigation. Yaw changes are confirmed correct in Mission Planner telemetry, and `compass_yaw_offset` has been applied to `FusedHeadingSource`, yet the spinning persists. The rover occasionally navigates straight, providing a key diagnostic clue.

**Key Findings:**

1. **P-only steering control** with no damping causes oscillation under any heading noise
2. **No hysteresis** on the AHRS/GPS COG heading source switch (1.0 m/s threshold) causes rapid heading flipping
3. **No smoothing** between heading sources creates discontinuities that trigger full steering corrections
4. **Throttle is independent of heading error** — the rover drives at full speed even while turning at maximum steering
5. **GPS COG timing lag** (1 Hz) vs AHRS (100 Hz) creates stale heading data when switching sources

**Why it sometimes goes straight:** Navigation works when the rover maintains steady speed above 1.0 m/s with stable GPS COG. The oscillation loop triggers when speed fluctuates around the 1.0 m/s threshold or when GPS COG updates arrive with stale data.

## Problem Space

### Observed Behavior

- **Symptom**: Rover spins in circles during Guided mode, unable to reach the target waypoint
- **Confirmed working**: Yaw axis changes correctly in Mission Planner telemetry
- **Partial fix applied**: `compass_yaw_offset` now applied to AHRS heading in `FusedHeadingSource`
- **Intermittent success**: Rover occasionally navigates straight toward the target
- **Environment**: Field testing with BNO086 IMU, GPS (typical \~1 Hz update), differential drive (4WD)

### Current Architecture

```text
                    ┌──────────────┐
                    │   GPS (1Hz)  │
                    │  COG, speed  │
                    └──────┬───────┘
                           │
┌──────────────────┐       │       ┌────────────────────────────┐
│   BNO086 (100Hz) │       │       │    FusedHeadingSource      │
│   AHRS yaw       │───────┼──────>│  speed >= 1.0? → GPS COG  │
│ + compass_offset │       │       │  speed <  1.0? → AHRS yaw │
└──────────────────┘       │       │  (hard switch, no blend)   │
                           │       └────────────┬───────────────┘
                           │                    │
                           │                    ▼ heading (degrees)
                           │       ┌────────────────────────────┐
                           │       │ SimpleNavigationController  │
                           │       │  heading_error = bearing    │
                           │       │                - heading    │
                           │       │  steering = error / 90°    │
                           │       │  (P-only, no damping)      │
                           │       └────────────┬───────────────┘
                           │                    │
                           │                    ▼ steering [-1, 1]
                           │       ┌────────────────────────────┐
                           │       │    DifferentialDrive::mix   │
                           │       │  left  = throttle+steering │
                           │       │  right = throttle-steering │
                           │       └────────────────────────────┘
```

### Components Verified as Correct

| Component                            | File                                                       | Status                                     |
| ------------------------------------ | ---------------------------------------------------------- | ------------------------------------------ |
| `DifferentialDrive::mix`             | `crates/core/src/kinematics/differential_drive.rs`         | Correct — sign conventions verified        |
| Motor control task                   | `crates/firmware/examples/pico_trail_rover.rs:1070`        | Correct — NAV_OUTPUT applied properly      |
| AHRS yaw convention                  | `crates/core/src/ahrs/dcm.rs:200`                          | Correct — 0°=North, CW positive            |
| `calculate_bearing` convention       | `crates/core/src/navigation/geo.rs:34`                     | Correct — 0°=North, CW positive, \[0, 360] |
| `wrap_180`                           | `crates/core/src/navigation/geo.rs:104`                    | Correct — wraps to \[-180, +180]           |
| `compass_yaw_offset` in AHRS heading | `crates/firmware/src/subsystems/navigation/heading.rs:118` | Correct — offset applied                   |
| AHRS task convergence                | `crates/firmware/src/subsystems/ahrs/task.rs:171`          | Correct — healthy after 5s                 |

## Root Cause Analysis

### Root Cause 1: P-Only Steering Controller (Critical)

**Location**: `crates/core/src/navigation/controller.rs:96-100`

```rust
fn calculate_steering(&self, heading_error: f32) -> f32 {
    let steering = heading_error / self.config.max_heading_error;
    steering.clamp(-1.0, 1.0)
}
```

The steering controller is purely proportional with `max_heading_error = 90.0°` (default in `types.rs:74`). This means:

- 45° heading error → 0.5 steering (50% turn)
- 90° heading error → 1.0 steering (full turn)
- Any heading noise directly maps to steering oscillation

**Why this causes spinning**: Without a derivative term (D), there is no damping to resist rapid heading changes. Without an integral term (I), persistent small errors never accumulate to a correction. The controller has zero resistance to oscillation.

**ArduPilot comparison**: ArduPilot's `AR_AttitudeControl` uses PID with configurable `ATC_STR_RAT_P`, `ATC_STR_RAT_I`, `ATC_STR_RAT_D` and includes rate limiting.

### Root Cause 2: Heading Source Hard-Switching Without Hysteresis (Critical)

**Location**: `crates/firmware/src/subsystems/navigation/heading.rs:138-147`

```rust
fn get_heading(&self) -> Option<f32> {
    // Priority 1: GPS COG when speed >= 1.0 m/s
    if gps.speed >= self.speed_threshold {
        return Some(cog);
    }
    // Priority 2: AHRS yaw
    return Some(ahrs_heading);
}
```

The heading source switches instantly at 1.0 m/s with no hysteresis band. The oscillation cycle:

1. Rover stationary → AHRS heading (e.g., 30°)
2. Rover accelerates past 1.0 m/s → GPS COG (e.g., 60° from stale GPS)
3. Heading jumps 30° → `heading_error` changes → steering correction
4. Rover slows from sharp turn → falls below 1.0 m/s → back to AHRS
5. Heading jumps again → another steering correction
6. → **Oscillation loop**

### Root Cause 3: GPS COG Timing Lag (High)

GPS position/COG updates at \~1 Hz while the navigation loop runs at 50 Hz. When the heading source switches to GPS COG:

- The COG value may be up to 1 second old
- During that 1 second, the rover may have turned 90°+
- Navigation sees a stale heading and applies a large correction in the wrong direction

### Root Cause 4: Throttle Independent of Heading Error (Medium)

**Location**: `crates/core/src/navigation/controller.rs:79-94`

```rust
fn calculate_throttle(&self, distance: f32) -> f32 {
    if distance < self.config.wp_radius {
        0.0
    } else if distance < self.config.approach_dist {
        // Linear ramp based on distance only
    } else {
        1.0  // Full throttle regardless of heading error!
    }
}
```

When the target is more than 10 m away, throttle is always 1.0 (full). Even if heading error is 180° (target is behind), the rover drives at full speed while applying maximum steering. This:

- Increases turn radius and momentum
- Makes oscillation faster and harder to recover from
- Prevents the rover from "stopping to think" when confused

ArduPilot reduces throttle proportionally to heading error (see `AR_AttitudeControl::get_throttle_out_stop()`).

### Root Cause 5: No Heading Source Blending (Medium)

**Location**: `crates/firmware/src/subsystems/navigation/heading.rs:138-163`

The `FusedHeadingSource` selects one source exclusively. There is no weighted blending (e.g., linearly interpolate between AHRS and GPS COG as speed crosses the threshold). A hard switch between two sources that may disagree by several degrees creates a step-function in heading that the P-only controller cannot handle gracefully.

## Oscillation Cycle Diagram

```text
   ┌─ Stationary: AHRS heading (correct after offset)
   │
   ▼
  Accelerate → speed > 1.0 m/s
   │
   ▼
  Switch to GPS COG (possibly stale/different)
   │
   ▼
  Large heading_error → full steering correction
   │
   ▼
  Sharp turn → speed drops < 1.0 m/s
   │
   ▼
  Switch back to AHRS (different heading!)
   │
   ▼
  Large heading_error → full steering in OTHER direction
   │
   ▼
  Speed rises again → ─────────────────── loops back ──┐
                                                        │
  ◄─────────────────────────────────────────────────────┘
```

## Why It Sometimes Works

The rover navigates correctly when all of the following are true simultaneously:

1. **Steady speed** well above 1.0 m/s (no threshold crossing)
2. **Fresh GPS COG** that closely matches actual heading
3. **Target roughly ahead** (small heading error, proportional response is adequate)
4. **No sharp turns needed** (P-only controller works for small corrections)

This explains the intermittent success: when initial conditions happen to align, the rover can make it to the target before oscillation builds up.

## Stakeholder Analysis

| Stakeholder       | Interest/Need                   | Impact | Priority |
| ----------------- | ------------------------------- | ------ | -------- |
| Rover Operator    | Reliable Guided mode navigation | High   | P0       |
| Navigation System | Stable heading input            | High   | P0       |
| Safety            | Predictable vehicle behavior    | High   | P0       |

## Discovered Requirements

### Functional Requirements

- [ ] **FR-TBD-1**: Navigation controller shall include derivative (D) term for steering damping
  - Rationale: P-only control oscillates under heading noise
  - Acceptance Criteria: Steering output is damped against rapid heading changes; rover converges to heading within 5 seconds

- [ ] **FR-TBD-2**: Heading source switching shall use hysteresis band
  - Rationale: Hard switching at 1.0 m/s threshold causes heading oscillation
  - Acceptance Criteria: Heading source does not switch more than once per second under steady-state conditions; uses hysteresis (e.g., switch to GPS COG at 1.5 m/s, back to AHRS at 0.8 m/s)

- [ ] **FR-TBD-3**: Navigation controller shall reduce throttle proportionally to heading error
  - Rationale: Full throttle during large heading error increases turn momentum and oscillation
  - Acceptance Criteria: Throttle is reduced when heading error exceeds a configurable threshold (e.g., 50% throttle at 45° error, 0% at 90°+)

### Non-Functional Requirements

- [ ] **NFR-TBD-1**: Heading source transitions shall not cause steering discontinuity greater than 0.3 (on -1.0 to 1.0 scale)
  - Category: Stability
  - Rationale: Large steering jumps trigger oscillation cycles
  - Target: Steering change per control cycle < 0.3 during heading source transitions

## Design Considerations

### Option A: Minimal Fix — Add Heading Error Rate Limiting

Add a maximum steering change rate (slew rate limit) to the navigation controller output. This prevents the step-function response to heading jumps.

**Pros**: Simple to implement, minimal code change
**Cons**: Masks the root cause; may slow legitimate turns

### Option B: PD Controller + Hysteresis

Replace P-only with PD (proportional-derivative) steering and add speed hysteresis to heading source.

**Pros**: Addresses root causes 1 and 2 directly; standard control approach
**Cons**: Requires tuning D-gain; derivative of noisy heading requires filtering

### Option C: Full Heading Fusion + PID + Throttle Scaling (Recommended)

1. **PID steering controller** with configurable gains (aligned with ArduPilot `ATC_STR_RAT_*`)
2. **Hysteresis band** for heading source switching (e.g., 0.8/1.5 m/s)
3. **Weighted blending** of AHRS and GPS COG near the threshold
4. **Throttle reduction** proportional to heading error
5. **Steering rate limiting** as a safety net

**Pros**: Comprehensive; addresses all root causes; follows ArduPilot patterns
**Cons**: Larger implementation scope; requires field tuning

## Risk Assessment

| Risk                                        | Probability | Impact | Mitigation Strategy                                                    |
| ------------------------------------------- | ----------- | ------ | ---------------------------------------------------------------------- |
| PID tuning requires field iteration         | High        | Medium | Start with conservative gains; provide ArduPilot-compatible parameters |
| GPS COG blending introduces jitter          | Low         | Low    | Use exponential moving average filter                                  |
| Throttle reduction prevents reaching target | Low         | Low    | Minimum throttle floor (e.g., 20%) when heading error < 90°            |

## Open Questions

- [ ] What is the actual GPS update rate in the current hardware configuration? (Assumed \~1 Hz)
- [ ] Is BNO086 yaw drift significant enough to cause issues even without heading source switching?
- [ ] Should heading rate limiting be configurable via ArduPilot parameters (e.g., `ATC_STR_RAT_MAX`)?

## Recommendations

### Immediate Priority

1. Add **steering rate limiting** (slew rate) as the fastest mitigation
2. Add **hysteresis** to heading source threshold (0.8 / 1.5 m/s)
3. Add **throttle reduction** proportional to heading error

### Follow-Up

4. Implement **PD controller** for steering (D-term for damping)
5. Add **heading source blending** near the speed threshold
6. Expose tuning parameters via ArduPilot `ATC_*` parameter family

### Out of Scope

- L1 or S-curve path following (separate feature, covered in AN-00003)
- Full EKF heading estimation (beyond current hardware capability)
- GPS RTK integration (hardware not available)

## Appendix

### ArduPilot Reference: Steering Rate Control

ArduPilot Rover uses `AR_AttitudeControl` with the following parameters:

| Parameter         | Description                   | Typical Value |
| ----------------- | ----------------------------- | ------------- |
| `ATC_STR_RAT_P`   | Steering rate P gain          | 0.2           |
| `ATC_STR_RAT_I`   | Steering rate I gain          | 0.2           |
| `ATC_STR_RAT_D`   | Steering rate D gain          | 0.0           |
| `ATC_STR_RAT_MAX` | Maximum steering rate (deg/s) | 120           |
| `ATC_SPEED_P`     | Speed P gain                  | 0.1           |

### Navigation Data Flow Trace

```text
navigation_task (50Hz)
  ├── heading_source.get_heading()
  │     ├── GPS speed >= 1.0? → gps.course_over_ground
  │     └── else → ahrs_state.get_yaw() + compass_offset → wrap_360()
  ├── controller.update(lat, lon, target, heading, dt)
  │     ├── calculate_bearing(current → target)         → [0, 360]
  │     ├── heading_error = wrap_180(bearing - heading)  → [-180, 180]
  │     ├── steering = error / 90.0, clamped [-1, 1]     ← P-only
  │     └── throttle = f(distance), ignores heading error
  └── NAV_OUTPUT = { steering, throttle }

motor_control_task (50Hz)
  ├── reads NAV_OUTPUT.steering, NAV_OUTPUT.throttle
  ├── DifferentialDrive::mix(steering, throttle)
  │     ├── left  = throttle + steering
  │     └── right = throttle - steering
  └── apply to 4 motors [left, left, right, right]
```
