# T-00040 Compass Yaw Offset Persistence

## Metadata

- Type: Design
- Status: Done

## Links

- Associated Plan Document:
  - [T-00040-compass-yaw-offset-persistence-plan](plan.md)

## Overview

Persist the compass yaw offset calculated by `MAV_CMD_FIXED_MAG_CAL_YAW` across reboots using the existing `ParameterStore` infrastructure. The offset is stored as the standard ArduPilot parameter `COMPASS_DEC` (per ADR-00035). On boot, `SystemState.compass_yaw_offset` is initialized from the stored parameter instead of hardcoded `0.0`.

## Success Metrics

- [x] `COMPASS_DEC` parameter registered in `CompassParams::register_defaults()`
- [x] `CompassParams::from_store()` returns the stored `COMPASS_DEC` value
- [x] `SystemState::from_param_store()` initializes `compass_yaw_offset` from `COMPASS_DEC`
- [x] `handle_fixed_mag_cal_yaw()` saves offset to ParameterStore after calibration
- [x] Doc comment correctly states "true north = 0"
- [x] All existing tests pass; RP2350 build succeeds

## Background and Current State

- Context: `MAV_CMD_FIXED_MAG_CAL_YAW` handler (T-00034) calculates `compass_yaw_offset = true_heading - ahrs_yaw` and stores it in `SystemState` (RAM only). The offset is lost on every reboot.
- Current behavior: `SystemState::from_param_store()` hardcodes `compass_yaw_offset: 0.0` at line 566. The handler at `command.rs:542` writes to `SystemState` but does not save to `ParameterStore`.
- Pain points: Operators must recalibrate after every power cycle.
- Constraints: Must use existing `ParameterStore` pattern. Per ADR-00035, the parameter is named `COMPASS_DEC`.
- Related ADRs: [ADR-00035](../../adr/ADR-00035-compass-yaw-offset-calibration.md)

## Proposed Design

### High-Level Architecture

```text
Boot Sequence:
  ParameterStore::load_from_flash()
    → CompassParams::from_store()  [reads COMPASS_DEC]
    → SystemState::from_param_store()  [sets compass_yaw_offset]
    → Navigation/Telemetry use offset immediately

Calibration Flow:
  MAV_CMD_FIXED_MAG_CAL_YAW received
    → Calculate yaw_offset
    → Write to SystemState.compass_yaw_offset (RAM)
    → Write to ParameterStore.set("COMPASS_DEC", offset) (RAM)
    → ParameterStore.save_to_flash() (flash)
    → COMMAND_ACK (ACCEPTED)
```

### Components

- `crates/firmware/src/parameters/compass.rs`:
  - Add `declination: f32` field to `CompassParams`
  - Register `COMPASS_DEC` in `register_defaults()` with default `0.0`
  - Load `COMPASS_DEC` in `from_store()`

- `crates/firmware/src/communication/mavlink/state.rs`:
  - In `from_param_store()`, read `compass_yaw_offset` from `CompassParams.declination`

- `crates/firmware/src/communication/mavlink/handlers/command.rs`:
  - In `handle_fixed_mag_cal_yaw()`, save offset to ParameterStore after computing
  - Fix doc comment: "magnetic north = 0" → "true north = 0"

### Data Flow

1. **Boot**: `ParameterStore::load_from_flash()` → `CompassParams::from_store()` reads `COMPASS_DEC` → `SystemState.compass_yaw_offset` initialized with stored value
2. **Calibration**: Handler computes offset → writes to `SystemState` (RAM) → writes to `ParameterStore` → saves to flash
3. **Usage**: Telemetry and navigation read `compass_yaw_offset` from `SystemState` (unchanged from current behavior)

### Data Models and Types

**CompassParams (updated):**

```rust
pub struct CompassParams {
    pub ofs_x: f32,
    pub ofs_y: f32,
    pub ofs_z: f32,
    pub declination: f32,  // NEW: COMPASS_DEC (yaw offset in radians)
}
```

**Parameter Registration:**

```rust
// In register_defaults()
let _ = store.register("COMPASS_DEC", ParamValue::Float(0.0), ParamFlags::empty());
```

**SystemState Initialization (updated):**

```rust
// In from_param_store()
let compass_params = CompassParams::from_store(param_store);
// ...
compass_yaw_offset: compass_params.declination,
```

**Calibration Save:**

```rust
// In handle_fixed_mag_cal_yaw(), after computing yaw_offset:
// Save to ParameterStore for persistence
// Note: ParameterStore access pattern depends on how the handler
// accesses the store (via PARAM_STORE global or passed reference)
```

### Error Handling

- If `COMPASS_DEC` is not in store (first boot), default to `0.0` — no calibration applied
- If flash save fails during calibration, offset is still in RAM for current session; log warning via STATUSTEXT
- If flash is corrupted on boot, `ParameterStore::load_from_flash()` returns defaults — `COMPASS_DEC` is `0.0`

### Security Considerations

- No security implications — calibration offset only affects heading output
- Parameter can be modified via MAVLink PARAM_SET (standard ArduPilot behavior)

### Performance Considerations

- Parameter registration: one-time cost at boot
- Flash save during calibration: <10ms (same as existing parameter saves)
- No change to hot path (telemetry/navigation reads from RAM `SystemState`)

### Platform Considerations

#### Cross-Platform

- Parameter registration and `from_store()` work on both host tests and RP2350
- Flash save (`save_to_flash`) only works on embedded targets
- Host tests use in-memory ParameterStore

## Alternatives Considered

1. **Store in a separate flash location (not ParameterStore)**
   - Pros: Independent of parameter system
   - Cons: Duplicates persistence logic, not inspectable via MAVLink
   - Rejected: ParameterStore already handles flash persistence

2. **Store as multiple parameters (declination + residual separately)**
   - Pros: More semantic accuracy
   - Cons: More complex, no practical benefit for single-offset approach
   - Rejected: ADR-00035 decided single `COMPASS_DEC` is sufficient

## Migration and Compatibility

- Backward compatibility: Existing systems without `COMPASS_DEC` in flash will use default `0.0` (same as current behavior)
- No breaking changes to existing MAVLink protocol or telemetry
- `COMPASS_DEC` will appear as a new parameter in Mission Planner's parameter list

## Testing Strategy

### Unit Tests

- Test `COMPASS_DEC` is registered in `CompassParams::register_defaults()`
- Test `CompassParams::from_store()` returns stored `COMPASS_DEC` value
- Test `CompassParams::from_store()` returns `0.0` when `COMPASS_DEC` is not set
- Test `SystemState::from_param_store()` uses `CompassParams.declination` for `compass_yaw_offset`

### Integration Tests

- Connect Mission Planner, perform Large Vehicle MagCal, verify `COMPASS_DEC` is set
- Power cycle, verify `COMPASS_DEC` is restored and heading is corrected

## Documentation Impact

- Update AN-00043 discovered requirements as fulfilled
- Update FR-00141 acceptance criteria checkboxes

## External References

- [ArduPilot COMPASS_DEC](https://ardupilot.org/rover/docs/parameters.html#compass-dec)
- [MAV_CMD_FIXED_MAG_CAL_YAW](https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW)

## Open Questions

- [ ] How does `handle_fixed_mag_cal_yaw` access the `ParameterStore` to save? Need to check if the handler has access via a global or if plumbing is needed.
