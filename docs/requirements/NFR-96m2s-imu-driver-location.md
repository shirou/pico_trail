# NFR-96m2s IMU Driver Location

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-svawa-imu-hal-abstraction](../requirements/FR-svawa-imu-hal-abstraction.md)
- Source Analysis:
  - [AN-aruub-imu-driver-hal-abstraction](../analysis/AN-aruub-imu-driver-hal-abstraction.md)
- Related Tasks:
  - [T-cj02a-imu-hal-refactor](../tasks/T-cj02a-imu-hal-refactor/README.md)

## Requirement Statement

Platform-agnostic device drivers shall reside in `src/devices/` rather than `src/platform/<target>/devices/`, with zero platform-specific imports in their implementation.

## Rationale

Placing generic device drivers under platform-specific directories:

- Misleads developers about driver portability
- Creates unnecessary coupling in module structure
- Violates separation of concerns between devices and platforms

## User Story (if applicable)

The system shall maintain clear architectural boundaries between platform-specific and platform-agnostic code to ensure maintainability and extensibility.

## Acceptance Criteria

- [ ] MPU9250 driver files located in `src/devices/imu/mpu9250/`
- [ ] ICM20948 driver files located in `src/devices/imu/icm20948/`
- [ ] No `embassy_rp::*` imports in IMU driver modules
- [ ] No `pico2_w` feature gates in IMU driver modules
- [ ] Platform re-exports from `src/platform/rp2350/devices/imu/` removed
- [ ] `src/devices/imu/mod.rs` exports drivers directly (no conditional re-export)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Directory structure after refactoring**:

```
src/devices/imu/
  mod.rs           # Exports Mpu9250Driver, Icm20948Driver, ImuSensor trait
  mpu9250/
    mod.rs
    driver.rs      # No embassy_rp imports
    config.rs
    registers.rs
  icm20948/
    mod.rs
    driver.rs      # No embassy_rp imports
    config.rs
    registers.rs
  mock.rs          # Mock for host tests
```

**Removed**:

```
src/platform/rp2350/devices/imu/  # Entire directory removed
```

## Platform Considerations

### Cross-Platform

This NFR ensures platform neutrality is visible in the code structure itself.

## Risks & Mitigation

| Risk                       | Impact | Likelihood | Mitigation                  | Validation    |
| -------------------------- | ------ | ---------- | --------------------------- | ------------- |
| Breaking imports elsewhere | Medium | Medium     | Update all `use` statements | `cargo check` |
| Missing re-exports         | Low    | Low        | Verify public API unchanged | Build + tests |

## Implementation Notes

- Move files before refactoring I2C types (or combine in one task)
- Update `src/devices/imu/mod.rs` to export directly
- Remove `#[cfg(feature = "pico2_w")]` from imu/mod.rs exports
