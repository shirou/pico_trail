# FR-7d203 Circle Mode Parameters

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-8dqwi-circle-mode](../analysis/AN-8dqwi-circle-mode.md)
- Prerequisite Requirements:
  - [FR-a1cuu-runtime-parameters](FR-a1cuu-runtime-parameters.md)
- Dependent Requirements:
  - [FR-khjpl-circle-mode-implementation](FR-khjpl-circle-mode-implementation.md)
- Related ADRs:
  - [ADR-897ov-circle-mode-path-generation](../adr/ADR-897ov-circle-mode-path-generation.md)
- Related Tasks:
  - [T-u8q60-circle-mode-implementation](../tasks/T-u8q60-circle-mode-implementation/README.md)

## Requirement Statement

The system shall support ArduPilot-compatible Circle mode parameters (CIRC_RADIUS, CIRC_SPEED, CIRC_DIR) that are configurable via MAVLink PARAM_SET and persist across reboots.

## Rationale

Using ArduPilot-standard parameter names ensures:

- GCS compatibility (Mission Planner, QGroundControl recognize parameters)
- Consistent user experience for ArduPilot users
- Documentation alignment with ArduPilot reference
- Predictable behavior matching ArduPilot Rover

Parameters must persist so users can configure once and have settings retained.

## User Story (if applicable)

As a GCS user, I want to configure Circle mode parameters (radius, speed, direction) via Mission Planner or QGroundControl, so that I can set up the circle behavior before entering the mode.

## Acceptance Criteria

- [ ] CIRC_RADIUS parameter available (float, meters, default 20.0)
- [ ] CIRC_SPEED parameter available (float, m/s, default 2.0)
- [ ] CIRC_DIR parameter available (int, 0=CW/1=CCW, default 0)
- [ ] Parameters configurable via MAVLink PARAM_SET
- [ ] Parameters readable via MAVLink PARAM_VALUE
- [ ] Parameters persist across reboots (flash storage)
- [ ] Parameter validation enforces valid ranges
- [ ] Parameter changes require mode re-entry to take effect

## Technical Details (if applicable)

### Functional Requirement Details

**Parameter Definitions:**

| Parameter     | Type    | Min | Max    | Default | Unit | Description              |
| ------------- | ------- | --- | ------ | ------- | ---- | ------------------------ |
| `CIRC_RADIUS` | Float32 | 0.0 | 1000.0 | 20.0    | m    | Circle radius            |
| `CIRC_SPEED`  | Float32 | 0.0 | 10.0   | 2.0     | m/s  | Target speed             |
| `CIRC_DIR`    | Int8    | 0   | 1      | 0       | -    | 0=Clockwise, 1=CounterCW |

**Parameter Validation:**

```rust
impl CircleParameters {
    pub fn validate(&self) -> Result<(), ParameterError> {
        if self.radius < 0.0 || self.radius > 1000.0 {
            return Err(ParameterError::OutOfRange("CIRC_RADIUS"));
        }
        if self.speed < 0.0 || self.speed > 10.0 {
            return Err(ParameterError::OutOfRange("CIRC_SPEED"));
        }
        if self.direction != 0 && self.direction != 1 {
            return Err(ParameterError::OutOfRange("CIRC_DIR"));
        }
        Ok(())
    }
}
```

**Special Cases:**

- `CIRC_RADIUS = 0`: Vehicle remains stationary (no circular motion)
- `CIRC_SPEED = 0`: Vehicle stops on the circle perimeter
- Parameter changes while in Circle mode: Ignored until mode re-entry

**Parameter Storage:**

Parameters stored in flash using existing parameter persistence infrastructure. Grouped with other navigation parameters.

## Platform Considerations

### Pico W (RP2040)

Flash write operations are identical across platforms.

### Pico 2 W (RP2350)

Flash write operations are identical across platforms.

### Cross-Platform

Parameter storage uses the same flash abstraction layer.

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                    | Validation                    |
| ------------------------------- | ------ | ---------- | ----------------------------- | ----------------------------- |
| Invalid parameter causes crash  | High   | Low        | Validate on set, use defaults | Test with out-of-range values |
| Flash wear from frequent writes | Low    | Low        | Debounce parameter saves      | Monitor flash write counts    |
| GCS parameter mismatch          | Low    | Medium     | Use exact ArduPilot names     | Test with Mission Planner     |

## Implementation Notes

Preferred approaches:

- Add CIRC\_\* parameters to parameter registry
- Use existing parameter persistence infrastructure
- Validate parameters on PARAM_SET, reject invalid values
- Read parameters at Circle mode entry, not during update

Known pitfalls:

- Parameter names must match ArduPilot exactly (case-sensitive)
- Don't read parameters every update cycle (performance)
- Handle missing parameters gracefully (use defaults)

Related code areas:

- `src/core/parameters/` - Parameter system
- `src/communication/mavlink/handlers/param.rs` - PARAM_SET handler
- `src/rover/mode/circle.rs` - Parameter consumer

## External References

- [ArduPilot Circle Mode Parameters](https://ardupilot.org/rover/docs/circle-mode.html)
- [ArduPilot Parameter System](https://ardupilot.org/dev/docs/parameters.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
