# FR-00143 Parameter Groups Host-Testable

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00142-parameter-store-core-migration](../analysis/AN-00142-parameter-store-core-migration.md)
- Prerequisite Requirements:
  - [FR-00142-parameter-store-host-testable](FR-00142-parameter-store-host-testable.md)
- Dependent Requirements:
  - [NFR-00092-parameter-migration-no-regression](NFR-00092-parameter-migration-no-regression.md)
- Related ADRs:
  - N/A – To be created during task design
- Related Tasks:
  - [T-00041-parameter-store-core-migration](../tasks/T-00041-parameter-store-core-migration/README.md)

## Requirement Statement

All architecture-independent parameter group types (`CompassParams`, `ArmingParams`, `BatteryParams`, `FailsafeParams`, `FenceParams`, `LoiterParams`, `CircleParams`, `WifiParams`) shall be defined in the `pico_trail_core` crate and testable via `cargo test --lib` on the host.

## Rationale

Parameter group types contain pure business logic: they register parameter names/defaults with `ParameterStore` and extract typed values via `from_store()`. AN-00142 confirmed that all parameter groups except `BoardParams` depend only on `ParameterStore` and a result type — no HAL dependency. Moving these types to core enables:

- Host-runnable unit tests for parameter registration and value extraction
- Faster CI feedback for parameter-related changes
- Clear separation of architecture-dependent code (`BoardParams` stays in firmware)

## User Story (if applicable)

As a developer, I want to add or modify parameter groups and immediately validate them via `cargo test --lib`, so that I do not need to wait for an embedded build to catch parameter registration errors.

## Acceptance Criteria

- [ ] `CompassParams` type is defined in `pico_trail_core` and testable on host
- [ ] `ArmingParams` type is defined in `pico_trail_core` and testable on host
- [ ] `BatteryParams` type is defined in `pico_trail_core` and testable on host
- [ ] `FailsafeParams` type is defined in `pico_trail_core` and testable on host
- [ ] `FenceParams` type is defined in `pico_trail_core` and testable on host
- [ ] `LoiterParams` type is defined in `pico_trail_core` and testable on host
- [ ] `CircleParams` type is defined in `pico_trail_core` and testable on host
- [ ] `WifiParams` type is defined in `pico_trail_core` and testable on host
- [ ] `BoardParams` remains in firmware crate (architecture-dependent)
- [ ] Each parameter group has `register_defaults()` and `from_store()` unit tests in core
- [ ] `cargo test --lib` runs and passes all parameter group tests
- [ ] Existing firmware code compiles without changes to parameter group usage patterns

## Technical Details (if applicable)

### Functional Requirement Details

**Parameter Group Pattern:**

All parameter groups follow the same pattern:

```rust
pub struct CompassParams {
    pub ofs_x: f32,
    pub ofs_y: f32,
    pub ofs_z: f32,
    pub declination: f32,
}

impl CompassParams {
    pub fn register_defaults(store: &mut ParameterStore) -> Result<(), ParameterError> {
        store.register("COMPASS_OFS_X", ParamValue::Float(0.0), ParamFlags::empty())?;
        // ...
        Ok(())
    }

    pub fn from_store(store: &ParameterStore) -> Self {
        Self {
            ofs_x: match store.get("COMPASS_OFS_X") {
                Some(ParamValue::Float(v)) => *v,
                _ => 0.0,
            },
            // ...
        }
    }
}
```

**Migration Scope:**

| Type             | Migrates to Core | Reason                                      |
| ---------------- | ---------------- | ------------------------------------------- |
| `CompassParams`  | Yes              | Only uses ParameterStore + Result           |
| `ArmingParams`   | Yes              | Only uses ParameterStore + Result           |
| `BatteryParams`  | Yes              | Only uses ParameterStore + Result           |
| `FailsafeParams` | Yes              | Only uses ParameterStore + Result           |
| `FenceParams`    | Yes              | Only uses ParameterStore + Result           |
| `LoiterParams`   | Yes              | Only uses ParameterStore + Result           |
| `CircleParams`   | Yes              | Only uses ParameterStore + Result           |
| `WifiParams`     | Yes              | Only uses ParameterStore + Result           |
| `BoardParams`    | **No**           | Uses `BOARD_CONFIG` (hwdef-generated const) |

**Test Coverage per Group:**

Each parameter group should have at minimum:

1. `test_register_defaults` — verifies all parameters are registered with correct defaults
2. `test_from_store_defaults` — verifies `from_store()` returns correct defaults
3. `test_from_store_custom_values` — verifies `from_store()` reads custom values

## Platform Considerations

### Cross-Platform

- All parameter groups must remain `no_std` compatible
- `BoardParams` stays in firmware due to `BOARD_CONFIG` dependency
- Firmware re-exports core parameter group types for existing import paths

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                              | Validation                                |
| --------------------------------------- | ------ | ---------- | ------------------------------------------------------- | ----------------------------------------- |
| Import path changes break firmware code | Medium | Medium     | Firmware re-exports core types at original module paths | RP2350 build verification                 |
| `BoardParams` accidentally moved        | High   | Low        | Explicit exclusion in migration plan                    | Code review + `BOARD_CONFIG` compile test |
| Missing test for a parameter group      | Low    | Low        | Checklist-driven migration with per-group verification  | Test count assertion                      |

## Implementation Notes

- Depends on FR-00142 (ParameterStore must be in core first)
- Firmware's `parameters/mod.rs` should re-export core types so existing `use crate::parameters::compass::CompassParams` paths continue to work
- `BoardParams` remains the only parameter group in firmware
- Existing firmware tests that compile-check parameter groups can be moved to core as executable tests

## External References

- [ArduPilot Parameter Reference](https://ardupilot.org/rover/docs/parameters.html) - Standard parameter names
