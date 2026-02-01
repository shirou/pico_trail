# NFR-00092 Parameter Migration No Regression

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00142-parameter-store-core-migration](../analysis/AN-00142-parameter-store-core-migration.md)
- Prerequisite Requirements:
  - [FR-00142-parameter-store-host-testable](FR-00142-parameter-store-host-testable.md)
  - [FR-00143-parameter-groups-host-testable](FR-00143-parameter-groups-host-testable.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00041-parameter-store-core-migration](../tasks/T-00041-parameter-store-core-migration/README.md)

## Requirement Statement

The migration of `ParameterStore` and parameter group types from the firmware crate to the core crate shall not change any public API signatures or break existing embedded builds.

## Rationale

The parameter subsystem is used throughout the firmware for configuration persistence, MAVLink parameter protocol handling, and boot-time initialization. Any regression in the RP2350 build or change in API signatures would break existing functionality. The migration must be transparent to firmware code that consumes parameter types.

## User Story (if applicable)

As a firmware developer, I want the parameter migration to be transparent, so that my existing code continues to compile and behave identically without requiring changes.

## Acceptance Criteria

- [ ] `./scripts/build-rp2350.sh pico_trail_rover` succeeds after migration
- [ ] All existing `cargo test --lib` tests continue to pass
- [ ] No firmware source files require import path changes (re-exports maintain compatibility)
- [ ] `ParameterStore` flash persistence (`load_from_flash`, `save_to_flash`) continues to work on RP2350
- [ ] Parameter values stored in flash before migration are readable after migration (no format change)
- [ ] MAVLink `PARAM_SET` / `PARAM_VALUE` protocol behavior is unchanged

## Technical Details (if applicable)

### Non-Functional Requirement Details

- **Reliability**: Zero regressions
  - RP2350 build must compile and produce identical binary behavior
  - Flash storage format must not change (existing parameters readable after firmware update)
  - MAVLink parameter protocol behavior must be identical

- **Compatibility**: API stability
  - Firmware code using `use crate::parameters::storage::ParameterStore` must continue to work
  - Firmware code using `use crate::parameters::compass::CompassParams` must continue to work
  - All public method signatures must remain identical

- **Maintainability**: Migration pattern
  - Clear documentation of which types moved where
  - Re-export pattern documented for future contributors

**Verification Strategy:**

1. **RP2350 Build**: Run `./scripts/build-rp2350.sh pico_trail_rover` after each migration phase
2. **Host Tests**: Run `cargo test --lib` after each phase — test count must not decrease
3. **Import Compatibility**: `cargo clippy --all-targets -- -D warnings` on firmware crate
4. **Flash Format**: No changes to serialization/deserialization logic in `load_from_flash`/`save_to_flash`

## Platform Considerations

### Cross-Platform

- RP2350 embedded build must succeed (primary validation target)
- Host `cargo test --lib` must pass (secondary validation target)
- No platform-specific behavior changes

## Risks & Mitigation

| Risk                                          | Impact | Likelihood | Mitigation                                          | Validation                     |
| --------------------------------------------- | ------ | ---------- | --------------------------------------------------- | ------------------------------ |
| Import path breakage in firmware              | High   | Medium     | Re-export core types at original firmware paths     | RP2350 build + clippy          |
| Flash format incompatibility                  | High   | Low        | Do not change serialization logic during migration  | Manual flash read verification |
| Missing re-export causes runtime behavior gap | High   | Low        | Phased migration with per-phase RP2350 verification | Build + test after each phase  |

## Implementation Notes

- Each migration phase must end with a successful RP2350 build
- Firmware's `parameters/mod.rs` should re-export all core types to maintain import compatibility
- Flash persistence code stays in firmware — only pure logic moves to core
- The `crc` crate and `FlashInterface` trait are not part of this migration scope

## External References

- N/A
