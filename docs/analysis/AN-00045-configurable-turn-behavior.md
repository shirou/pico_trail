# AN-00045 Configurable Turn Behavior

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00044-guided-mode-heading-oscillation](AN-00044-guided-mode-heading-oscillation.md)
  - [AN-00023-position-target-navigation](AN-00023-position-target-navigation.md)
  - [AN-00003-navigation-approach](AN-00003-navigation-approach.md)
- Related Requirements:
  - [FR-00144-configurable-pivot-turn-threshold](../requirements/FR-00144-configurable-pivot-turn-threshold.md)
  - [FR-00145-arc-turn-minimum-throttle](../requirements/FR-00145-arc-turn-minimum-throttle.md)
  - [NFR-00093-arc-pivot-transition-continuity](../requirements/NFR-00093-arc-pivot-transition-continuity.md)
  - [FR-00084-navigation-controller](../requirements/FR-00084-navigation-controller.md)
  - [FR-00083-guided-mode-navigation](../requirements/FR-00083-guided-mode-navigation.md)
  - [FR-00082-auto-mode-mission-execution](../requirements/FR-00082-auto-mode-mission-execution.md)
  - [FR-00063-differential-drive-kinematics](../requirements/FR-00063-differential-drive-kinematics.md)
- Related Tasks:
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Executive Summary

The rover performs pivot turns (spin-in-place) in all autonomous navigation modes (Guided, Auto, RTL, SmartRTL, Loiter, Circle) when heading error is large. The current `calculate_throttle()` reduces throttle to 0 at `throttle_heading_error_max` (90°), and when throttle reaches 0, `DifferentialDrive::mix(steering, 0)` produces opposite wheel rotation — a pivot turn. A spin-in-place cap (`max_spin_steering=0.3`) slows the pivot but does not eliminate it.

ArduPilot addresses this with `WP_PIVOT_ANGLE` — a configurable threshold that determines when pivot turns are allowed versus arc turns. Below the threshold, the rover maintains forward throttle and turns in arcs; above it, pivot turns are permitted.

This analysis recommends adding configurable pivot/arc turn behavior to the navigation controller via two new `SimpleNavConfig` fields: `pivot_turn_angle` and `arc_turn_min_throttle`. Because all autonomous modes share the same `SimpleNavigationController`, the change applies universally — no per-mode modifications are required.

## Problem Space

### Observed Behavior

- When heading error is large (e.g., 60°+), throttle drops significantly due to `heading_scale`
- At 90° heading error, throttle reaches 0 and the rover pivots in place
- Pivot turns work for large errors (target behind the rover) but are undesirable for moderate corrections (30°–60°)
- The rover should maintain forward motion and arc-turn for moderate heading corrections

### Current Architecture

**Key code paths:**

1. `SimpleNavigationController::update()` calls `calculate_throttle(distance, heading_error_abs)` and `calculate_steering(heading_error)`
2. `calculate_throttle()` computes `distance_throttle * heading_scale` where `heading_scale = 1.0 - (error / throttle_heading_error_max)`
3. When `heading_error_abs >= throttle_heading_error_max` (90°), `heading_scale = 0.0` → throttle = 0
4. `motor_control_task` calls `DifferentialDrive::mix(steering, throttle)` → motor speeds
5. `mix(steering, 0.0)` produces `(+steering, -steering)` — opposite wheels, pivot turn

**Key files:**

- `crates/core/src/navigation/controller.rs:96-120` — `calculate_throttle()`
- `crates/core/src/navigation/types.rs:55-78` — `SimpleNavConfig`
- `crates/core/src/kinematics/differential_drive.rs:35-92` — `DifferentialDrive::mix()`

### Affected Modes

All autonomous navigation modes share `SimpleNavigationController` and are affected by this change. Manual mode uses direct RC input and does not go through the navigation controller.

| Mode     | Uses `SimpleNavigationController` | File                                            |
| -------- | --------------------------------- | ----------------------------------------------- |
| Guided   | Yes                               | `crates/firmware/src/rover/mode/guided.rs:51`   |
| Auto     | Yes                               | `crates/firmware/src/rover/mode/auto.rs:87`     |
| RTL      | Yes                               | `crates/firmware/src/rover/mode/rtl.rs:60`      |
| SmartRTL | Yes                               | `crates/firmware/src/rover/mode/smartrtl.rs:57` |
| Loiter   | Yes                               | `crates/firmware/src/rover/mode/loiter.rs:97`   |
| Circle   | Yes                               | `crates/firmware/src/rover/mode/circle.rs:122`  |
| Manual   | No (direct RC)                    | N/A                                             |

### Current Default Configuration

| Parameter                    | Value | Effect                                    |
| ---------------------------- | ----- | ----------------------------------------- |
| `throttle_heading_error_max` | 90.0° | Throttle reaches 0 at 90° heading error   |
| `max_spin_steering`          | 0.3   | Caps steering magnitude when throttle ≈ 0 |
| `spin_throttle_threshold`    | 0.1   | Threshold below which spin cap applies    |

### The Throttle-Heading Problem

The current throttle curve is linear from 1.0 (at 0° error) to 0.0 (at 90° error). This means:

- At 45° error: throttle = 0.5 — reasonable, but rover is already turning with reduced speed
- At 60° error: throttle = 0.33 — rover barely moves forward, approaching pivot behavior
- At 90° error: throttle = 0.0 — full pivot turn

For a differential drive rover, moderate heading corrections (30°–60°) are better handled by maintaining forward throttle and using arc turns. Pivot turns should be reserved for large errors where the target is behind or nearly behind the rover.

## ArduPilot Reference

ArduPilot Rover uses `WP_PIVOT_ANGLE` to control this behavior:

| Parameter        | Description                                  | Default |
| ---------------- | -------------------------------------------- | ------- |
| `WP_PIVOT_ANGLE` | Heading error threshold for pivot turns      | 60°     |
| `WP_PIVOT_RATE`  | Maximum turn rate during pivot turns (deg/s) | 60      |

**ArduPilot behavior:**

- `heading_error < WP_PIVOT_ANGLE`: Rover maintains forward speed, turns in arcs
- `heading_error >= WP_PIVOT_ANGLE`: Rover is allowed to stop and pivot
- `WP_PIVOT_ANGLE = 0`: Pivots always allowed (current pico_trail behavior)
- `WP_PIVOT_ANGLE = 180`: Pivots never allowed, always arc turns

## Stakeholder Analysis

| Stakeholder       | Interest/Need                        | Impact | Priority |
| ----------------- | ------------------------------------ | ------ | -------- |
| Rover Operator    | Smooth, predictable turning behavior | High   | P1       |
| Navigation System | Efficient path to waypoints          | Medium | P1       |
| Safety            | No abrupt throttle discontinuities   | High   | P0       |

## Design Considerations

### Option A: Pivot Angle + Arc Turn Throttle in `SimpleNavConfig` (Recommended)

Modify `calculate_throttle()` to enforce a minimum throttle floor when heading error is below `pivot_turn_angle` and the rover is not at the target:

```rust
// After computing distance_throttle * heading_scale:
if heading_error_abs < self.config.pivot_turn_angle && distance > self.config.wp_radius {
    throttle = throttle.max(self.config.arc_turn_min_throttle);
}
```

**New config fields:**

- `pivot_turn_angle: f32` — Heading error threshold for pivot turns (default 60.0°, matching ArduPilot `WP_PIVOT_ANGLE`)
- `arc_turn_min_throttle: f32` — Minimum throttle during arc turns (default 0.15)

**Pros:**

- Minimal code change (2 new fields, \~5 lines of logic)
- Follows ArduPilot's `WP_PIVOT_ANGLE` concept
- `pivot_turn_angle=0` preserves current behavior
- Transition is naturally continuous with default configuration (see Continuity Analysis below)

**Cons:**

- Does not modify `DifferentialDrive::mix()` — arc vs pivot is controlled entirely by throttle
- `arc_turn_min_throttle` is not an ArduPilot standard parameter (ArduPilot handles this internally)

### Option B: Modify `DifferentialDrive::mix()` to Limit Inner Wheel Reversal

Add logic to `mix()` to prevent the inner wheel from reversing direction when throttle > 0.

**Pros:** Works at the kinematics level, affects all modes
**Cons:** Complex, changes Manual mode behavior, violates current `mix()` contract

### Option C: Full Turn Rate PID (ArduPilot ATC_STR_RAT\_\*)

Implement a separate turn rate controller with P/I/D gains for pivot and arc turn modes.

**Pros:** Full ArduPilot compatibility
**Cons:** Over-engineering for current needs, requires significant field tuning

### Continuity Analysis (Option A)

With default configuration (`pivot_turn_angle=60°`, `throttle_heading_error_max=90°`):

At the `pivot_turn_angle` boundary (60° heading error):

- `heading_scale = 1.0 - (60.0 / 90.0) = 0.333`
- `distance_throttle = 1.0` (far from target)
- Natural throttle = `1.0 * 0.333 = 0.333`
- `arc_turn_min_throttle = 0.15`
- Since `0.333 > 0.15`, the floor does not activate at the boundary

This means:

- Just below 60°: throttle = 0.333 (natural curve, above floor)
- Just above 60°: throttle = 0.333 (natural curve, floor no longer applies)
- **Zero discontinuity at the boundary** with default settings
- The floor only activates deeper in the arc turn range (heading error > \~77° where natural throttle drops below 0.15)

## Risk Assessment

| Risk                                            | Probability | Impact | Mitigation                                                 |
| ----------------------------------------------- | ----------- | ------ | ---------------------------------------------------------- |
| Arc turn min throttle too high for tight spaces | Low         | Low    | Default 0.15 is conservative; configurable per deployment  |
| Discontinuity at pivot angle boundary           | Low         | Medium | Continuity analysis shows zero discontinuity with defaults |
| Pivot turn needed but delayed by arc behavior   | Low         | Low    | `pivot_turn_angle=0` restores current behavior             |

## Discovered Requirements

### Functional Requirements

- [ ] **FR-TBD-1**: Navigation controller shall support a configurable heading error threshold (`pivot_turn_angle`) above which pivot turns are allowed; below this threshold, forward throttle is maintained for arc turns. This applies to all modes using `SimpleNavigationController` (Guided, Auto, RTL, SmartRTL, Loiter, Circle).
  - Rationale: Pivot turns are appropriate only for large heading errors; moderate corrections should use arc turns
  - Acceptance Criteria: Default 60.0° matching ArduPilot `WP_PIVOT_ANGLE`; `pivot_turn_angle=0` preserves current behavior; `pivot_turn_angle=180` prevents all pivot turns; behavior consistent across all autonomous modes

- [ ] **FR-TBD-2**: During arc turns, the navigation controller shall maintain a configurable minimum forward throttle (`arc_turn_min_throttle`) to ensure forward motion while turning. This applies to all modes using `SimpleNavigationController`.
  - Rationale: Forward throttle during arc turns produces more efficient path to waypoints than pivot turns
  - Acceptance Criteria: Default 0.15; floor applies only when heading error < pivot_turn_angle AND distance > wp_radius; floor does not apply at target

### Non-Functional Requirements

- [ ] **NFR-TBD-1**: The transition between arc turn and pivot turn behavior shall be continuous, with no abrupt throttle discontinuity at the `pivot_turn_angle` boundary
  - Category: Stability
  - Rationale: Throttle discontinuities cause jerky vehicle behavior
  - Target: Throttle values at heading error just below and just above `pivot_turn_angle` differ by < 0.1

## Open Questions

- [ ] Should `pivot_turn_angle` be exposed as an ArduPilot `WP_PIVOT_ANGLE` parameter via MAVLink, or remain a compile-time config?
- [ ] Is the default `arc_turn_min_throttle` of 0.15 appropriate for the Freenove 4WD rover's motor characteristics?

## Recommendations

### Recommended Approach

**Option A** — Add `pivot_turn_angle` and `arc_turn_min_throttle` to `SimpleNavConfig` and modify `calculate_throttle()`.

This is the minimal effective solution:

- 2 new config fields
- \~5 lines of logic in `calculate_throttle()`
- Follows ArduPilot `WP_PIVOT_ANGLE` concept
- Backward compatible (`pivot_turn_angle=0` preserves current behavior)
- Naturally continuous with default configuration

### Out of Scope

- `WP_PIVOT_RATE` implementation (turn rate during pivot turns — defer to future analysis)
- `DifferentialDrive::mix()` modifications (unnecessary for this feature)
- Full turn rate PID controller (covered separately in AN-00044 recommendations)

## Appendix

### ArduPilot Reference: WP_PIVOT_ANGLE

From ArduPilot Rover documentation:

- **Parameter**: `WP_PIVOT_ANGLE`
- **Range**: 0–180 degrees
- **Default**: 60°
- **Description**: Controls the heading error threshold at which the rover is allowed to pivot turn. Set to 0 to always allow pivot turns. Set to 180 to never allow pivot turns.
- **Link**: <https://ardupilot.org/rover/docs/parameters.html#wp-pivot-angle>

### Throttle Curve Visualization

```text
Throttle
  1.0 ─┐
       │ ╲                          Current: linear 1.0→0.0
       │   ╲                        Proposed: floor at arc_turn_min_throttle
  0.33 ─│─ ─ ─╲─ ─ ─ ─ ─ ─ ─ ─ ─ ─  Natural throttle at 60° (pivot boundary)
       │       ╲
  0.15 ─│─ ─ ─ ─ ╲━━━━━━━━━━━━━━━━  arc_turn_min_throttle floor
       │           ╲  ↑floor active
  0.0 ─│─ ─ ─ ─ ─ ─ ─╲─ ─ ─ ─ ─ ─  Below floor: pivot allowed
       └──────────────────────────→
       0°    30°   60°   90°   Heading Error
                    ↑
              pivot_turn_angle
```
