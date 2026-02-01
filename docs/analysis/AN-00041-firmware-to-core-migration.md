# AN-00041 Firmware-to-Core Platform-Independent Code Migration

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-00039-crate-workspace-separation](AN-00039-crate-workspace-separation.md)
  - [AN-00004-platform-abstraction](AN-00004-platform-abstraction.md)
  - [AN-00029-feature-gate-reduction](AN-00029-feature-gate-reduction.md)
  - [AN-00037-ahrs-navigation-control-integration](AN-00037-ahrs-navigation-control-integration.md)
- Related Requirements:
  - [FR-00130-core-crate-nostd-purity](../requirements/FR-00130-core-crate-nostd-purity.md)
  - [NFR-00089-zero-cfg-core-crate](../requirements/NFR-00089-zero-cfg-core-crate.md)
- Related ADRs: N/A
- Related Tasks:
  - [T-00036-firmware-to-core-migration](../tasks/T-00036-firmware-to-core-migration/README.md)

## Executive Summary

This analysis investigates platform-independent code currently residing in `crates/firmware` that should be migrated to `crates/core`. Following the workspace separation established by AN-00039, `crates/core` is designed to hold pure `no_std` business logic with zero `cfg` directives, while `crates/firmware` contains Embassy/RP2350-specific code.

The investigation identified three categories of findings: (1) duplicated code that exists identically in both crates (\~560 lines), (2) pure algorithms in firmware with zero platform dependencies ready for immediate migration (\~350 lines), and (3) mixed modules where business logic can be extracted after decoupling from platform-specific state management (\~700 lines). The total migration opportunity is approximately 1,600 lines, which would reduce firmware complexity and improve testability of core algorithms.

## Problem Space

### Current State

The workspace split (AN-00039) established two crates:

| Crate             | Purpose                    | Files     | Principle                      |
| ----------------- | -------------------------- | --------- | ------------------------------ |
| `crates/core`     | Pure no_std business logic | 27 files  | Zero `cfg`, zero Embassy/defmt |
| `crates/firmware` | Embassy/RP2350 entry point | 165 files | Platform-specific integration  |

However, `crates/firmware` still contains significant platform-independent code:

- **Pure mathematical algorithms** (DCM, haversine, path simplification) that have no Embassy or hardware dependencies
- **Duplicate implementations** of code already in core (arming errors, differential drive, parameter types)
- **Business logic mixed with state management** where algorithms are entangled with `EmbassyState` globals

### Desired State

- All platform-independent algorithms and types reside in `crates/core`
- `crates/firmware` contains only platform-specific integration: Embassy tasks, device drivers, global state wrappers, and hardware initialization
- No code duplication between crates
- Core algorithms are testable with `cargo test` without any feature flags

### Gap Analysis

| Aspect                | Current                         | Desired                        | Gap                                         |
| --------------------- | ------------------------------- | ------------------------------ | ------------------------------------------- |
| Duplicated code       | \~560 lines in both crates      | Single source of truth in core | Delete firmware duplicates                  |
| DCM algorithm         | In firmware only                | In core                        | Move to core (zero dependencies)            |
| Geographic algorithms | Split between core and firmware | Consolidated in core           | Merge firmware geo.rs with core nav.rs      |
| Path recorder         | In firmware only                | Algorithm in core              | Extract algorithm, keep globals in firmware |
| Navigation controller | In firmware only                | Trait and algorithm in core    | Extract with generic position type          |
| AHRS traits           | In firmware only                | Core traits in core            | Extract traits without SharedAhrsState      |
| Mission state machine | In firmware only                | State logic in core            | Extract state transitions to core           |

## Stakeholder Analysis

| Stakeholder      | Interest/Need                                  | Impact | Priority |
| ---------------- | ---------------------------------------------- | ------ | -------- |
| Developers       | Host-testable algorithms, reduced duplication  | High   | P0       |
| Maintainers      | Single source of truth, clear crate boundaries | High   | P0       |
| Future platforms | Reusable core algorithms for ESP32/STM32       | Medium | P1       |

## Research & Discovery

### Technical Investigation

#### Category 1: Duplicated Code (Identical in Both Crates)

Code that exists in both `crates/core` and `crates/firmware` with functionally identical implementations:

| Firmware File                                    | Core File                              | Lines | Differences                                                            |
| ------------------------------------------------ | -------------------------------------- | ----- | ---------------------------------------------------------------------- |
| `src/core/arming/error.rs`                       | `src/arming/error.rs`                  | 165   | Minor: core uses `copy_from_slice` (no_std), firmware uses `.to_vec()` |
| `src/libraries/kinematics/differential_drive.rs` | `src/kinematics/differential_drive.rs` | 95    | Doc path only: `pico_trail::` vs `pico_trail_core::`                   |
| `src/core/parameters/block.rs`                   | `src/parameters/block.rs`              | \~150 | Import style only                                                      |
| `src/core/parameters/crc.rs`                     | `src/parameters/crc.rs`                | \~150 | Minor: array init vs `.to_vec()` in test                               |

**Action**: Delete firmware versions and import from core. The core versions are already `no_std` compatible.

**Note on registry.rs**: The parameter registry is **intentionally split** - core provides type definitions (`ParamType`, `ParamValue`, `ParamMetadata`), firmware provides the full `ParameterRegistry` implementation with Flash persistence. This separation is correct and should be preserved.

#### Category 2: Pure Algorithms Ready for Immediate Migration

Code in firmware with zero platform dependencies (no Embassy, no defmt, no hardware):

**DCM Algorithm** (`firmware/src/subsystems/ahrs/dcm.rs`, \~200 lines)

- Direction Cosine Matrix attitude estimation
- Dependencies: `micromath::F32Ext`, `nalgebra` only
- Contains one `log_warn!()` call (trivially removable)
- Core already has `crates/core/src/ahrs/` module with `calibration.rs`
- **Action**: Move directly to `crates/core/src/ahrs/dcm.rs`

**Geographic Calculations** (`firmware/src/subsystems/navigation/geo.rs`, \~150 lines)

- `calculate_bearing()` - haversine initial bearing
- `calculate_distance()` - haversine great-circle distance
- `wrap_180()`, `wrap_360()` - angle normalization
- `offset_position()` - lat/lon offset calculation
- Dependencies: `libm` only
- Core already has `crates/core/src/mode/nav.rs` with `haversine_distance_bearing()` and `normalize_angle()`
- **Action**: Consolidate into `crates/core/src/navigation/geo.rs`, remove duplicates from `mode/nav.rs`

#### Category 3: Mixed Modules (Require Decoupling)

Code where business logic is entangled with platform-specific state management:

**Path Recorder** (`firmware/src/subsystems/navigation/path_recorder.rs`, \~200 lines)

- `PathPoint` struct and `PathRecorder` ring-buffer algorithm
- Pure algorithm using only `calculate_distance()` from geo.rs
- Platform dependency: `crate::log_info!()` / `crate::log_debug!()` macros only
- **Action**: Move algorithm to core, remove or abstract logging calls

**Navigation Controller** (`firmware/src/subsystems/navigation/controller.rs`, \~300 lines)

- `NavigationController` trait and `SimpleNavigationController` implementation
- Pure steering/throttle calculation using bearing and distance
- Platform dependency: `crate::devices::gps::GpsPosition` type
- **Blocker**: `GpsPosition` is defined in firmware's device layer
- **Action**: Define a core-level position type (lat/lon/alt - 3 fields), make controller generic over it or move `GpsPosition` to core

**Heading Source** (`firmware/src/subsystems/navigation/heading.rs`, \~200 lines)

- `HeadingSource` trait, `HeadingSourceType` enum, `FusedHeadingSource`
- Platform dependency: `SharedAhrsState` (critical-section-based synchronization)
- **Action**: Extract trait definitions and fusion algorithm to core, keep `SharedAhrsState` binding in firmware

**AHRS Traits** (`firmware/src/subsystems/ahrs/traits.rs`, \~150 lines)

- `Ahrs` trait and `AhrsState` struct
- `SharedAhrsState` wrapper using `critical_section`
- **Action**: Move `Ahrs` trait and `AhrsState` to core. Keep `SharedAhrsState` in firmware.

**Mission State Machine** (`firmware/src/core/mission/state.rs`, \~250 lines)

- Mission execution state transitions
- Platform dependency: `EmbassyState` for synchronized access
- **Action**: Extract state transition logic to core, keep global state wrapper in firmware

#### Already Correctly Separated (No Action Required)

| Module           | Core                                | Firmware                            | Status  |
| ---------------- | ----------------------------------- | ----------------------------------- | ------- |
| RC Channel       | Normalization logic, types          | `EmbassyState` global wrapper       | Correct |
| Servo/Actuator   | `ActuatorConfig`, pulse calculation | Platform PWM implementation         | Correct |
| Motor            | `Motor` trait, `MotorGroup`         | H-bridge driver implementation      | Correct |
| Scheduler        | Types, registry, statistics         | Async task executor, Embassy tasks  | Correct |
| AHRS Calibration | `CalibrationData` struct            | Parameter/Flash integration         | Correct |
| Sync/Time Traits | `SharedState`, `TimeSource` traits  | `EmbassyState`, `EmbassyTime` impls | Correct |

### Data Analysis

Summary of code distribution:

| Category                          | Lines       | Files  | Action             |
| --------------------------------- | ----------- | ------ | ------------------ |
| Duplicated (delete firmware copy) | \~560       | 4      | Delete             |
| Pure algorithms (move directly)   | \~350       | 2      | Move               |
| Mixed (extract after decoupling)  | \~700       | 5      | Refactor then move |
| Correctly separated               | N/A         | N/A    | None               |
| **Total migration opportunity**   | **\~1,600** | **11** |                    |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Core crate shall contain all platform-independent mathematical algorithms (DCM, haversine, path simplification) without duplication in firmware
  - Rationale: Eliminates code duplication, enables host testing of algorithms
  - Acceptance Criteria: No algorithm implementations exist in firmware that have zero platform dependencies

- [ ] **FR-DRAFT-2**: Core crate shall define a platform-independent position type (latitude, longitude, altitude) usable by navigation algorithms
  - Rationale: Navigation controller, path recorder, and heading source all need a position type but currently depend on firmware's `GpsPosition`
  - Acceptance Criteria: Core position type exists and is used by all navigation algorithms in core

- [ ] **FR-DRAFT-3**: Core crate shall define AHRS traits (`Ahrs`, `AhrsState`) without platform-specific synchronization wrappers
  - Rationale: AHRS trait definitions are platform-independent; synchronization is a firmware concern
  - Acceptance Criteria: Core AHRS traits compile without `critical_section` dependency

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: No functionally identical code shall exist in both `crates/core` and `crates/firmware`
  - Category: Maintainability
  - Rationale: Duplicate code leads to divergence and maintenance burden
  - Target: Zero duplicate implementations between crates

- [ ] **NFR-DRAFT-2**: All algorithms migrated to core shall have unit tests that run via `cargo test` without feature flags
  - Category: Testability
  - Rationale: Core algorithms must be testable on host without embedded toolchain
  - Target: 100% of migrated algorithms have corresponding unit tests in core

## Design Considerations

### Technical Constraints

1. **Orphan Rule**: Core types must be defined in core to allow trait implementations in both crates
2. **no_std Compatibility**: All migrated code must work without `std` (use `libm`, `heapless`, `nalgebra` with no_std)
3. **Logging Abstraction**: Code moved to core cannot use `defmt` or `crate::log_*!()` macros directly
4. **Zero cfg in Core**: Per NFR-00089, core must maintain zero `#[cfg(...)]` directives

### Potential Approaches

1. **Option A: Phased Migration (Recommended)**
   - Phase 1: Delete duplicates (zero risk)
   - Phase 2: Move pure algorithms (low risk)
   - Phase 3: Extract mixed module logic (medium risk)
   - Pros: Incremental, each phase independently verifiable, minimal regression risk
   - Cons: Multiple rounds of changes
   - Effort: Low per phase

2. **Option B: Single Migration**
   - Move all identified code in one batch
   - Pros: Done in one pass
   - Cons: Large changeset, harder to review, higher regression risk
   - Effort: Medium (concentrated)

### Architecture Impact

- Core crate gains \~1,600 lines of algorithms and types
- Firmware crate shrinks correspondingly
- A new core-level `GpsPosition` (or `Position`) type is needed as a shared data structure
- Navigation module in core needs restructuring to accommodate geo algorithms and path recording

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                                              |
| -------------------------------------------- | ----------- | ------ | ---------------------------------------------------------------- |
| Breaking firmware build during migration     | Medium      | High   | Phase migration, verify RP2350 build after each phase            |
| Subtle behavior differences in migrated code | Low         | Medium | Run existing tests, add new tests for migrated algorithms        |
| Core crate dependency bloat                  | Low         | Low    | Only add dependencies already used (libm, nalgebra, heapless)    |
| Logging loss in migrated algorithms          | Low         | Low    | Algorithms should not log; remove debug logging during migration |

## Open Questions

- [x] Are the duplicate files intentionally maintained or accidental? → Finding: Accidental divergence from the workspace split process. Core versions are the source of truth.
- [ ] Should `GpsPosition` be a new type in core or should the existing `PositionTarget` be extended? → Next step: Review core's `PositionTarget` struct to determine if it covers the same fields
- [ ] Should DCM algorithm in core depend on `nalgebra` or use a simpler matrix representation? → Next step: Check if core already depends on `nalgebra`
- [ ] Should path recorder logging be replaced with a callback/trait or simply removed? → Next step: Evaluate whether recording events need observability in core

## Recommendations

### Immediate Actions

1. Delete duplicate files in firmware that are identical to core versions (arming/error.rs, kinematics/differential_drive.rs, parameters/block.rs, parameters/crc.rs)
2. Update firmware imports to reference `pico_trail_core` for these modules

### Next Steps

1. [ ] Create formal requirements from FR-DRAFT-1 through FR-DRAFT-3 and NFR-DRAFT-1 through NFR-DRAFT-2
2. [ ] Draft ADR for: Core position type design and navigation module restructuring
3. [ ] Create task for: Phase 1 (duplicate deletion), Phase 2 (pure algorithm migration), Phase 3 (mixed module extraction)
4. [ ] Resolve open questions about `GpsPosition` type and `nalgebra` dependency

### Out of Scope

- Rover mode implementations - These have tight coupling to communication/`SystemState` and require a larger architectural refactor (deferred to separate analysis)
- MAVLink protocol code - Communication layer is inherently firmware-specific
- Device drivers - By design, these belong in firmware
- Scheduler async execution - Embassy task executor is firmware-specific by nature

## Appendix

### References

- [AN-00039-crate-workspace-separation](AN-00039-crate-workspace-separation.md) - Original workspace split analysis
- [AN-00004-platform-abstraction](AN-00004-platform-abstraction.md) - Platform abstraction strategy
- [FR-00130-core-crate-nostd-purity](../requirements/FR-00130-core-crate-nostd-purity.md) - Core crate purity requirement
- [NFR-00089-zero-cfg-core-crate](../requirements/NFR-00089-zero-cfg-core-crate.md) - Zero cfg directive requirement

### Migration Detail by File

| Firmware Path                                    | Target Core Path                  | Lines | Dependency          | Phase |
| ------------------------------------------------ | --------------------------------- | ----- | ------------------- | ----- |
| `src/core/arming/error.rs`                       | (delete, already in core)         | 165   | None                | 1     |
| `src/libraries/kinematics/differential_drive.rs` | (delete, already in core)         | 95    | None                | 1     |
| `src/core/parameters/block.rs`                   | (delete, already in core)         | 150   | None                | 1     |
| `src/core/parameters/crc.rs`                     | (delete, already in core)         | 150   | None                | 1     |
| `src/subsystems/ahrs/dcm.rs`                     | `src/ahrs/dcm.rs`                 | 200   | nalgebra, micromath | 2     |
| `src/subsystems/navigation/geo.rs`               | `src/navigation/geo.rs`           | 150   | libm                | 2     |
| `src/subsystems/navigation/path_recorder.rs`     | `src/navigation/path_recorder.rs` | 200   | geo.rs              | 3     |
| `src/subsystems/navigation/controller.rs`        | `src/navigation/controller.rs`    | 300   | Position type       | 3     |
| `src/subsystems/ahrs/traits.rs`                  | `src/ahrs/traits.rs`              | 150   | nalgebra            | 3     |
| `src/subsystems/navigation/heading.rs`           | `src/navigation/heading.rs`       | 200   | AHRS traits         | 3     |
| `src/core/mission/state.rs`                      | `src/mission/state.rs`            | 250   | Mission types       | 3     |
