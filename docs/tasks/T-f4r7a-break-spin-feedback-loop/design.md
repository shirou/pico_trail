# T-f4r7a Break Spin-in-Place Positive Feedback Loop Design

## Metadata

- Type: Design
- Status: Implementation Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Field testing after T-w8x3p revealed a self-sustaining spin feedback loop. When heading error exceeds 90 degrees, throttle drops to 0 but steering remains at 1.0. `DifferentialDrive::mix(1.0, 0.0)` produces `left=1.0, right=-1.0`, causing maximum-speed spin in place. The resulting vibration corrupts AHRS yaw readings, sustaining the large heading error indefinitely.

This design addresses root causes 6-9 from AN-27568's post-T-w8x3p field testing analysis with four complementary mitigations.

## Success Metrics

- [ ] Spin-in-place steering capped: `|steering| <= max_spin_steering` when `throttle < spin_throttle_threshold`
- [ ] Slew rate active by default: `max_steering_rate = 2.0` (not 0.0)
- [ ] Heading EMA reduces noise: smoothed heading changes less than raw heading under simulated noise
- [ ] D-term uses gyroscope yaw rate: derivative input is direct measurement, not differenced heading
- [ ] All existing tests pass; no regressions

## Background and Current State

- Context: Rover spins in circles during Guided mode despite T-w8x3p PD steering, throttle reduction, and hysteresis
- Current behavior:
  - `SimpleNavConfig::default()` sets `max_steering_rate: 0.0`, disabling slew rate limiting
  - `steering_d_gain: 0.005` is too small for meaningful damping at normal heading rates
  - D-term computed from successive heading error differences, amplifying measurement noise
  - Raw AHRS heading passed directly to navigation controller without filtering
  - No steering cap when throttle is near zero, allowing full-speed spin in place
- Pain points: Positive feedback loop between spinning, vibration, and heading corruption
- Constraints: Must remain no_std compatible, f32-only, < 1ms update on RP2350

## Proposed Design

### High-Level Architecture

All modifications are within existing structs and methods. One new struct added for heading filtering.

```text
SimpleNavigationController              FusedHeadingSource
+-- calculate_steering()                +-- get_heading()
|   +-- CHANGE: accept yaw_rate param  |   (no changes)
|   +-- CHANGE: D-term from yaw_rate
+-- calculate_throttle()                HeadingFilter (NEW)
|   (no changes)                        +-- apply(raw_heading) -> smoothed
+-- update()
|   +-- CHANGE: accept yaw_rate param
|   +-- CHANGE: spin-in-place cap

SimpleNavConfig
+-- CHANGE: max_steering_rate default 0.0 -> 2.0
+-- CHANGE: steering_d_gain default 0.005 -> 0.05
+-- ADD: max_spin_steering: f32
+-- ADD: spin_throttle_threshold: f32
+-- ADD: heading_filter_alpha: f32
```

### Component 1: Enable Slew Rate Limiting (Root Cause 7)

**File**: `crates/core/src/navigation/types.rs`

Change `SimpleNavConfig::default()`:

```rust
max_steering_rate: 2.0,  // was 0.0 (disabled)
```

At 50 Hz, this allows max steering change of 0.04 per frame (full range change in 1 second). This was the intended value per T-w8x3p design but was not set in the implementation default.

### Component 2: Spin-in-Place Steering Cap (Root Cause 6)

**File**: `crates/core/src/navigation/controller.rs`

After calculating steering and throttle in `update()`, apply a steering cap when throttle is near zero:

```rust
// Spin-in-place speed limit
let effective_steering = if throttle < self.config.spin_throttle_threshold {
    steering.clamp(-self.config.max_spin_steering, self.config.max_spin_steering)
} else {
    steering
};
```

New config fields in `SimpleNavConfig`:

```rust
/// Maximum steering when throttle is near zero (0.0-1.0)
pub max_spin_steering: f32,       // default: 0.3
/// Throttle threshold below which spin limiting applies
pub spin_throttle_threshold: f32, // default: 0.1
```

This reduces spin speed, reducing vibration, giving the AHRS a better chance to track yaw correctly. Even at 0.3 steering, the rover can rotate to align with the target, but slowly enough to avoid vibration-induced AHRS corruption.

### Component 3: Heading EMA Filter (Root Cause 9)

**File**: `crates/core/src/navigation/heading_filter.rs` (new file in `crates/core/src/navigation/`)

Exponential Moving Average filter with angle-aware smoothing:

```rust
pub struct HeadingFilter {
    alpha: f32,
    prev_heading: Option<f32>,
}

impl HeadingFilter {
    pub fn new(alpha: f32) -> Self {
        Self { alpha: alpha.clamp(0.0, 1.0), prev_heading: None }
    }

    pub fn apply(&mut self, heading: f32) -> f32 {
        match self.prev_heading {
            None => {
                self.prev_heading = Some(heading);
                heading
            }
            Some(prev) => {
                // Angle-aware interpolation
                let diff = wrap_180(heading - prev);
                let smoothed = wrap_360(prev + self.alpha * diff);
                self.prev_heading = Some(smoothed);
                smoothed
            }
        }
    }

    pub fn reset(&mut self) {
        self.prev_heading = None;
    }
}
```

Alpha of 0.3 at 50 Hz provides smoothing while maintaining responsiveness. Higher alpha = less smoothing, lower alpha = more smoothing.

New config field in `SimpleNavConfig`:

```rust
/// EMA filter alpha for heading smoothing (0.0 = max smoothing, 1.0 = no filter)
pub heading_filter_alpha: f32, // default: 0.3
```

The filter is applied in the firmware-level navigation task before passing heading to the controller. The `HeadingFilter` struct lives in `crates/core` to enable unit testing without embassy dependencies.

### Component 4: Gyroscope Yaw Rate for D-Term (Root Cause 8)

**Files**:

- `crates/core/src/navigation/controller.rs` - Accept yaw_rate parameter
- `crates/core/src/navigation/types.rs` - Update trait signature

Change `NavigationController::update()` and `SimpleNavigationController` to accept an optional yaw rate:

```rust
fn update(
    &mut self,
    current_lat: f32,
    current_lon: f32,
    target: &PositionTarget,
    heading: f32,
    dt: f32,
    yaw_rate_dps: Option<f32>,  // NEW: gyroscope yaw rate in deg/s
) -> NavigationOutput;
```

In `calculate_steering`, use gyroscope yaw rate when available:

```rust
fn calculate_steering(&mut self, heading_error: f32, dt: f32, yaw_rate_dps: Option<f32>) -> f32 {
    let p_term = heading_error / self.config.max_heading_error;

    // D-term: prefer gyroscope yaw rate over differenced heading error
    let d_term = if let Some(yaw_rate) = yaw_rate_dps {
        // Gyroscope provides clean yaw rate directly
        // Negative because positive yaw rate (turning right) should reduce
        // positive steering (already turning right = less correction needed)
        -yaw_rate * self.config.steering_d_gain
    } else if dt > 0.0 {
        // Fallback: differentiate heading error
        let error_rate = (heading_error - self.prev_heading_error) / dt;
        error_rate * self.config.steering_d_gain
    } else {
        0.0
    };
    self.prev_heading_error = heading_error;

    // ... rest unchanged (PD output + slew rate limiting)
}
```

Also increase `steering_d_gain` default from 0.005 to 0.05 so the D-term is effective at normal heading rates (10-50 deg/s).

The firmware-level navigation task reads `AHRS_STATE.get_angular_rate().z.to_degrees()` (already available in the current code for logging) and passes it to the controller.

### Data Flow (Updated)

```text
navigation_task (50Hz)
  +-- heading_source.get_heading()
  |     +-- GPS speed vs threshold WITH HYSTERESIS
  +-- heading_filter.apply(raw_heading)              <-- NEW: EMA filter
  |     +-- smoothed heading (reduces vibration noise)
  +-- yaw_rate = AHRS_STATE.get_angular_rate().z     <-- NEW: gyro input
  +-- controller.update(lat, lon, target, heading, dt, yaw_rate)
  |     +-- calculate_bearing(current -> target)
  |     +-- heading_error = wrap_180(bearing - heading)
  |     +-- steering = PD(heading_error, dt, yaw_rate) + slew rate
  |     +-- throttle = f(distance) * f(heading_error)
  |     +-- spin_limit: if throttle < 0.1, cap steering  <-- NEW
  +-- NAV_OUTPUT = { steering, throttle }
```

### Error Handling

- `HeadingFilter` first call returns raw heading (no smoothing on first sample)
- `yaw_rate_dps = None` falls back to differenced heading error (backward-compatible)
- `max_spin_steering = 1.0` effectively disables spin limiting
- `heading_filter_alpha = 1.0` disables filtering (pass-through)

### Performance Considerations

- HeadingFilter: 1 subtraction, 1 multiplication, 1 addition per update
- Gyroscope yaw rate: already read for logging, zero additional I2C cost
- Spin-in-place cap: 1 comparison, 1 clamp per update
- Total overhead: negligible (< 5us additional on RP2350)
- HeadingFilter struct: 8 bytes (1 f32 + 1 Option<f32>)

## Alternatives Considered

1. **Only fix defaults (slew rate + D-gain)**: Simplest, but does not address the fundamental feedback loop (spin → vibration → heading corruption → spin)
2. **Full complementary filter replacing EMA**: More accurate, but significantly more complex and requires IMU acceleration data
3. **Disable differential drive spin-in-place entirely**: Too restrictive; the rover needs to be able to rotate to align with targets

### Decision Rationale

The four mitigations attack the feedback loop at multiple points: slew rate prevents instant steering reversals, spin-in-place cap reduces vibration energy, EMA filter smooths residual noise, and gyroscope D-term provides clean rate feedback. Each is independently testable and has a simple disable mechanism.

## Testing Strategy

### Unit Tests

- Spin-in-place cap: verify steering limited when throttle < threshold
- Spin-in-place cap: verify steering unlimited when throttle >= threshold
- HeadingFilter: verify smoothing reduces noise amplitude
- HeadingFilter: verify angle wrapping (350 -> 10 degrees)
- HeadingFilter: verify alpha=1.0 produces pass-through
- HeadingFilter: verify reset clears state
- D-term with yaw_rate: verify gyroscope input used over differenced heading
- D-term without yaw_rate: verify fallback to differenced heading
- Default config: verify max_steering_rate is 2.0 (not 0.0)
- Default config: verify steering_d_gain is 0.05
- Backward compatibility: all existing tests pass with updated defaults

### Field Testing

- Guided mode to waypoint 50m away on flat ground
- Verify rover does not enter sustained spin at start
- Verify slow rotation (not fast spin) when heading error is large
- Verify stable heading tracking during straight-line navigation

## Open Questions

- [ ] Optimal `max_spin_steering` value: start with 0.3, tune via field testing
- [ ] Optimal `heading_filter_alpha`: start with 0.3, adjust based on vibration level
- [ ] Optimal `steering_d_gain` with gyroscope input: start with 0.05, may need different tuning than differenced heading

## External References

- [ArduPilot ATC_STR_RAT_D](https://ardupilot.org/rover/docs/parameters.html)
- [Exponential Moving Average](https://en.wikipedia.org/wiki/Exponential_smoothing)
