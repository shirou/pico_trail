# FR-00115 Loiter Type Parameter Support

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00118-rover-loiter-mode](FR-00118-rover-loiter-mode.md)
  - [FR-00006-runtime-parameters](FR-00006-runtime-parameters.md)
- Dependent Requirements:
  - [FR-00117-position-drift-detection](FR-00117-position-drift-detection.md)
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Requirement Statement

The system shall support the LOIT_TYPE parameter to control Loiter mode behavior, where Type 0 stops motors without position correction and Type 1 actively corrects position when drift exceeds LOIT_RADIUS threshold.

## Rationale

Different operating conditions require different Loiter behaviors:

- **Type 0 (Stop)**: Suitable for flat terrain where the rover naturally stays in place; conserves battery
- **Type 1 (Active Hold)**: Required for slopes, windy conditions, or when precise position holding is critical

Following ArduPilot's parameter convention ensures GCS compatibility and familiar user experience.

## User Story

As a rover operator, I want to configure whether Loiter mode actively holds position or simply stops, so that I can optimize behavior for my operating environment.

## Acceptance Criteria

- [ ] LOIT_TYPE parameter exists with valid range 0-1
- [ ] LOIT_TYPE=0: Motors stop, no GPS-based position correction
- [ ] LOIT_TYPE=1: Active position correction when outside LOIT_RADIUS
- [ ] Parameter is readable/writable via MAVLink PARAM_REQUEST_READ/PARAM_SET
- [ ] Parameter change takes effect immediately (no mode re-entry required)
- [ ] Default value is 0 (stop) matching ArduPilot default
- [ ] LOIT_RADIUS parameter exists with valid range 0.5-100m, default 2m

## Technical Details

### Functional Requirement Details

**Parameter Definitions:**

| Parameter     | Type | Range    | Default | Description                       |
| ------------- | ---- | -------- | ------- | --------------------------------- |
| `LOIT_TYPE`   | u8   | 0-1      | 0       | 0=stop, 1=active position hold    |
| `LOIT_RADIUS` | f32  | 0.5-100m | 2.0     | Drift threshold before correction |

**Behavior by Type:**

**Type 0 (Stop):**

```rust
fn update_type0(&mut self) -> Result<(), &'static str> {
    self.actuators.set_steering(0.0)?;
    self.actuators.set_throttle(0.0)?;
    Ok(())
}
```

**Type 1 (Active Hold):**

```rust
fn update_type1(&mut self, current_pos: &GpsPosition) -> Result<(), &'static str> {
    let distance = distance_m(current_pos, &self.loiter_point);

    if distance <= self.radius {
        // Within acceptable radius - stop
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
    } else {
        // Outside radius - navigate back
        self.navigate_to_loiter_point(current_pos)?;
    }
    Ok(())
}
```

**Parameter Storage:**

Parameters stored in `src/parameters/loiter.rs`:

```rust
pub struct LoiterParams {
    pub loit_type: u8,      // 0=stop, 1=active
    pub loit_radius: f32,   // meters
}

impl Default for LoiterParams {
    fn default() -> Self {
        Self {
            loit_type: 0,
            loit_radius: 2.0,
        }
    }
}
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                   | Validation             |
| ----------------------------- | ------ | ---------- | ---------------------------- | ---------------------- |
| Invalid parameter value       | Low    | Low        | Clamp to valid range on set  | Parameter validation   |
| Type change during correction | Low    | Low        | Check type each update cycle | Test parameter changes |

## Implementation Notes

- Use existing parameter infrastructure from `src/parameters/mod.rs`
- Register parameters with MAVLink parameter protocol handler
- Parameter names must exactly match ArduPilot: `LOIT_TYPE`, `LOIT_RADIUS`
- Consider adding `LOIT_SPEED` parameter for correction speed limit (future enhancement)

## External References

- [ArduPilot LOIT_TYPE](https://ardupilot.org/rover/docs/parameters.html#loit-type) - Parameter documentation
- [ArduPilot LOIT_RADIUS](https://ardupilot.org/rover/docs/parameters.html#loit-radius) - Parameter documentation
