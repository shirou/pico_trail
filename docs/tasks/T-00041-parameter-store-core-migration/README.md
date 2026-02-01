# T-00041 Parameter Store Core Migration

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00142-parameter-store-core-migration](../../analysis/AN-00142-parameter-store-core-migration.md)
- Related Requirements:
  - [FR-00142-parameter-store-host-testable](../../requirements/FR-00142-parameter-store-host-testable.md)
  - [FR-00143-parameter-groups-host-testable](../../requirements/FR-00143-parameter-groups-host-testable.md)
  - [NFR-00092-parameter-migration-no-regression](../../requirements/NFR-00092-parameter-migration-no-regression.md)
- Related ADRs:
  - N/A – No ADR needed (straightforward migration following existing core/firmware separation pattern)
- Associated Design Document:
  - [T-00041-parameter-store-core-migration-design](design.md)
- Associated Plan Document:
  - [T-00041-parameter-store-core-migration-plan](plan.md)

## Summary

Migrate `ParameterStore`, `ParamValue`, `ParamFlags`, and all architecture-independent parameter group types (`CompassParams`, `ArmingParams`, `BatteryParams`, `FailsafeParams`, `FenceParams`, `LoiterParams`, `CircleParams`, `WifiParams`) from the firmware crate to the core crate. This enables host-side `cargo test --lib` execution for all parameter logic. Flash persistence methods (`load_from_flash`, `save_to_flash`) and `BoardParams` remain in the firmware crate.

## Scope

- In scope:
  - Create `ParameterError` enum in core crate for parameter operations
  - Unify `ParamValue` types (core's `Float/Uint32` + firmware's `String/Bool/Int/Float/Ipv4`)
  - Move `ParameterStore` core operations to core crate
  - Move `ParamFlags` and `ParamMetadata` to core crate
  - Move 8 parameter group types to core crate
  - Add re-exports in firmware for import path compatibility
  - Move existing firmware compile-only tests to core as executable tests
  - Add new unit tests for all migrated types
- Out of scope:
  - `BoardParams` migration (depends on hwdef `BOARD_CONFIG`)
  - `SystemState` migration (depends on firmware-specific types)
  - `FlashInterface` trait migration
  - Flash persistence method migration (`load_from_flash`, `save_to_flash`)
  - Removing firmware parameter module (still needed for flash ops and `BoardParams`)

## Success Metrics

- `cargo test --lib` runs and passes parameter store and parameter group tests
- `./scripts/build-rp2350.sh pico_trail_rover` succeeds with no regressions
- All existing firmware import paths continue to work via re-exports
- Test count increases (firmware compile-only tests become executable)
- No changes to flash storage format or MAVLink parameter protocol behavior
