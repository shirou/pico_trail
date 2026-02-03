# T-00042 Configurable Turn Behavior Design

## Metadata

- Type: Design
- Status: Implementation Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Add two config fields to `SimpleNavConfig` and a throttle floor to `calculate_throttle()`. When heading error is below `pivot_turn_angle` and the rover is not at the target, a minimum throttle (`arc_turn_min_throttle`) is enforced, preventing pivot turns for moderate heading corrections.

## Success Metrics

- [x] `pivot_turn_angle` field in `SimpleNavConfig` with default 60.0°
- [x] `arc_turn_min_throttle` field in `SimpleNavConfig` with default 0.15
- [x] Throttle floor enforced during arc turns
- [x] Transition at boundary is continuous (< 0.1 throttle delta)
- [x] `pivot_turn_angle=0` disables feature (current behavior preserved)
- [x] All existing tests pass; no regressions
- [x] `NavigationParams` registered in parameter store with all 12 fields
- [x] `to_nav_config()` converts `NavigationParams` → `SimpleNavConfig`
- [x] Parameters configurable at runtime via MAVLink GCS

## Background and Current State

- Context: Rover pivot-turns for moderate heading corrections (30°-60°) where arc turns would be more efficient
- Current behavior:
  - `calculate_throttle()` linearly scales throttle from 1.0 at 0° to 0.0 at 90° heading error
  - When throttle=0, `DifferentialDrive::mix(steering, 0)` produces pivot turn
  - Spin-in-place cap (`max_spin_steering=0.3`) slows pivots but doesn't prevent them
- Pain points: Inefficient path to waypoints, unnecessary spin-in-place for moderate corrections
- Constraints: Must remain no_std compatible, f32-only, < 1ms update on RP2350
- ArduPilot equivalent: `WP_PIVOT_ANGLE` (default 60°)

## Proposed Design

### High-Level Architecture

All modifications are within existing structs and methods. No new files or structs needed.

```text
SimpleNavConfig
+-- ADD: pivot_turn_angle: f32       (default: 60.0)
+-- ADD: arc_turn_min_throttle: f32  (default: 0.15)

SimpleNavigationController
+-- calculate_throttle()
    +-- CHANGE: apply throttle floor when heading_error < pivot_turn_angle
```

### Component: Throttle Floor in `calculate_throttle()`

**File**: `crates/core/src/navigation/controller.rs`

Current `calculate_throttle()` returns `(distance_throttle * heading_scale).clamp(0.0, 1.0)`. Add a floor condition after this calculation:

```rust
fn calculate_throttle(&self, distance: f32, heading_error_abs: f32) -> f32 {
    // ... existing distance_throttle and heading_scale logic ...

    let mut throttle = (distance_throttle * heading_scale).clamp(0.0, 1.0);

    // Arc turn throttle floor: maintain forward motion for moderate heading errors
    if heading_error_abs < self.config.pivot_turn_angle
        && distance > self.config.wp_radius
    {
        throttle = throttle.max(self.config.arc_turn_min_throttle);
    }

    throttle
}
```

### Config Fields

**File**: `crates/core/src/navigation/types.rs`

```rust
// Add to SimpleNavConfig struct:

/// Heading error threshold for pivot turns (degrees, ArduPilot: WP_PIVOT_ANGLE).
/// Below this angle, arc_turn_min_throttle is enforced.
/// Set to 0 to always allow pivot turns (current behavior).
/// Set to 180 to never allow pivot turns.
pub pivot_turn_angle: f32,

/// Minimum throttle during arc turns (0.0-1.0).
/// Only applies when heading error < pivot_turn_angle and distance > wp_radius.
pub arc_turn_min_throttle: f32,
```

Default values in `impl Default for SimpleNavConfig`:

```rust
pivot_turn_angle: 60.0,
arc_turn_min_throttle: 0.15,
```

### Interaction with Existing Features

**Spin-in-place cap**: `arc_turn_min_throttle` (0.15) > `spin_throttle_threshold` (0.1), so the spin cap does not activate during arc turns. This is correct — arc turns need full steering with forward motion.

**Approach zone**: The floor uses `max()`, so approach zone throttle reduction still applies when it produces higher throttle. The floor only raises throttle, never lowers it.

**At target**: The `distance > wp_radius` guard ensures the floor does not prevent stopping at the target.

### Continuity Analysis

With defaults (`pivot_turn_angle=60°`, `throttle_heading_error_max=90°`):

| Heading Error | Natural Throttle | Floor Active | Effective Throttle |
| ------------- | ---------------- | ------------ | ------------------ |
| 0°            | 1.00             | Yes (idle)   | 1.00               |
| 30°           | 0.67             | Yes (idle)   | 0.67               |
| 59.9°         | 0.334            | Yes (idle)   | 0.334              |
| 60.1°         | 0.332            | No           | 0.332              |
| 77°           | 0.14             | No           | 0.14               |
| 90°           | 0.00             | No           | 0.00               |

At the boundary (60°), natural throttle is 0.333, well above the 0.15 floor. The floor only activates at \~77° where natural throttle drops below 0.15, but above 60° the floor is already disabled. **Zero discontinuity.**

### Error Handling

- `pivot_turn_angle = 0.0`: Floor condition `heading_error_abs < 0.0` is always false → feature disabled, preserves current behavior
- `pivot_turn_angle = 180.0`: Floor always applies when not at target → no pivot turns
- `arc_turn_min_throttle = 0.0`: Floor is effectively disabled (max with 0 changes nothing)

### Performance Considerations

- 1 comparison + 1 `max()` added to `calculate_throttle()` per update
- Zero additional memory (2 f32 config fields = 8 bytes in struct, already stack-allocated)
- Negligible CPU impact (< 1us on RP2350)

### Component: `NavigationParams` Parameter Group

**File**: `crates/core/src/parameters/navigation.rs` (new)

Follows the established `LoiterParams` pattern. Provides `register_defaults()`, `from_store()`, `to_nav_config()`, and `is_valid()`.

#### Parameter Name Mapping

| SimpleNavConfig Field        | Parameter Name   | ArduPilot Standard | Default | Range      |
| ---------------------------- | ---------------- | ------------------ | ------- | ---------- |
| `wp_radius`                  | `WP_RADIUS`      | Yes                | 2.0     | 0.5–100.0  |
| `approach_dist`              | `WP_APPR_DIST`   | No                 | 10.0    | 1.0–200.0  |
| `max_heading_error`          | `ATC_HDG_ERR`    | No                 | 90.0    | 10.0–180.0 |
| `min_approach_throttle`      | `WP_APPR_THR`    | No                 | 0.2     | 0.0–1.0    |
| `steering_d_gain`            | `ATC_STR_RAT_D`  | Yes                | 0.05    | 0.0–1.0    |
| `max_steering_rate`          | `ATC_STR_SMAX`   | No                 | 2.0     | 0.0–10.0   |
| `throttle_heading_error_max` | `ATC_THR_HERR`   | No                 | 90.0    | 10.0–180.0 |
| `max_spin_steering`          | `ATC_SPIN_MAX`   | No                 | 0.3     | 0.0–1.0    |
| `spin_throttle_threshold`    | `ATC_SPIN_THR`   | No                 | 0.1     | 0.0–1.0    |
| `heading_filter_alpha`       | `ATC_HDG_FILT`   | No                 | 0.3     | 0.0–1.0    |
| `pivot_turn_angle`           | `WP_PIVOT_ANGLE` | Yes                | 60.0    | 0.0–180.0  |
| `arc_turn_min_throttle`      | `WP_ARC_THR`     | No                 | 0.15    | 0.0–1.0    |

#### Registration

Add to `ParamHandler::new()` in `crates/firmware/src/communication/mavlink/handlers/param.rs`:

```rust
let _ = crate::parameters::NavigationParams::register_defaults(&mut store);
```

#### Conversion

`NavigationParams::to_nav_config()` converts the parameter store representation to the runtime `SimpleNavConfig` used by `SimpleNavigationController`. This keeps navigation logic decoupled from parameter storage.

## Alternatives Considered

1. **Modify `DifferentialDrive::mix()`**: Prevent inner wheel reversal at kinematics level. Rejected — changes Manual mode behavior, violates mix() contract.
2. **Full turn rate PID**: Separate pivot/arc turn rate controllers. Rejected — over-engineering for this change.

### Decision Rationale

A throttle floor in `calculate_throttle()` is the simplest approach: 2 config fields and \~5 lines of logic. It works at the navigation level (not kinematics), applies to all modes uniformly, and is backward-compatible.

## Testing Strategy

### Unit Tests

- **Boundary behavior**: Heading error at 59°, 60°, 61° with default config → verify floor activation/deactivation
- **Continuity**: Throttle at 59.9° vs 60.1° → delta < 0.1
- **Feature disabled**: `pivot_turn_angle=0` → verify identical behavior to current implementation
- **No pivots mode**: `pivot_turn_angle=180` → verify floor always active
- **At target**: `distance <= wp_radius` → verify floor does not apply
- **Approach zone interaction**: Distance in approach zone with large heading error → verify correct throttle
- **Existing tests**: All current navigation tests pass unchanged

### Parameter Store Tests

- **Default registration**: `register_defaults()` populates all 12 parameters
- **Custom values**: `from_store()` reads overridden values correctly
- **Clamping**: Out-of-range values clamped to valid ranges
- **Conversion**: `to_nav_config()` produces matching `SimpleNavConfig`
- **Validation**: `is_valid()` rejects invalid parameter combinations

### Embedded Verification

- `./scripts/build-rp2350.sh pico_trail_rover` compiles successfully

## External References

- [ArduPilot WP_PIVOT_ANGLE](https://ardupilot.org/rover/docs/parameters.html#wp-pivot-angle)
