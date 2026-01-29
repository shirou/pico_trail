# T-nazbq Firmware-to-Core Platform-Independent Code Migration

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-64ofl-firmware-to-core-migration](../../analysis/AN-64ofl-firmware-to-core-migration.md)
  - [AN-q7k2m-crate-workspace-separation](../../analysis/AN-q7k2m-crate-workspace-separation.md)
- Related Requirements:
  - [FR-5f7tx-core-crate-nostd-purity](../../requirements/FR-5f7tx-core-crate-nostd-purity.md)
  - [NFR-3y83q-zero-cfg-core-crate](../../requirements/NFR-3y83q-zero-cfg-core-crate.md)
- Related ADRs: N/A
- Associated Design Document:
  - [T-nazbq-design](./design.md)
- Associated Plan Document:
  - [T-nazbq-plan](./plan.md)

## Summary

Migrate platform-independent code from `crates/firmware` to `crates/core`, eliminating code duplication and moving pure algorithms (DCM, geographic calculations, path recording, navigation control, AHRS traits, mission state machine) to the core crate for improved testability and reuse.

## Scope

- In scope:
  - Delete duplicate files in firmware that are identical to core versions (arming/error.rs, kinematics/differential_drive.rs, parameters/block.rs, parameters/crc.rs)
  - Move pure algorithms to core (DCM, geographic calculations)
  - Extract platform-independent logic from mixed modules (path recorder, navigation controller, heading source, AHRS traits, mission state)
  - Define core-level position type for navigation algorithms
  - Add unit tests for all migrated code
  - Update firmware imports to use core modules

- Out of scope:
  - Rover mode implementations (tight coupling to SystemState, requires larger refactor)
  - MAVLink protocol code (inherently firmware-specific)
  - Device drivers (belong in firmware by design)
  - Scheduler async execution (Embassy-specific)

## Success Metrics

- **Zero duplication**: No functionally identical code exists in both crates
- **Host tests pass**: `cargo test -p pico_trail_core --lib --quiet` passes with migrated algorithm tests
- **Embedded build**: `./scripts/build-rp2350.sh pico_trail_rover` compiles successfully
- **Zero cfg in core**: `./scripts/check-core-no-cfg.sh` passes
- **Lines migrated**: \~1,600 lines moved from firmware to core or deleted as duplicates
