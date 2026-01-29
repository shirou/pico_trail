# AN-q7k2m Crate Workspace Separation for CFG Elimination

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-li4m8-feature-gate-reduction](../analysis/AN-li4m8-feature-gate-reduction.md)
  - [AN-kir7h-platform-abstraction](../analysis/AN-kir7h-platform-abstraction.md)
- Related Requirements:
  - [FR-5f7tx-core-crate-nostd-purity](../requirements/FR-5f7tx-core-crate-nostd-purity.md)
  - [FR-mna5g-trait-abstractions-platform-services](../requirements/FR-mna5g-trait-abstractions-platform-services.md)
  - [FR-lmy7w-external-impl-observability](../requirements/FR-lmy7w-external-impl-observability.md)
  - [NFR-3y83q-zero-cfg-core-crate](../requirements/NFR-3y83q-zero-cfg-core-crate.md)
  - [NFR-11puw-ci-core-purity-lint](../requirements/NFR-11puw-ci-core-purity-lint.md)
- Related ADRs:
  - (To be created after approval)
- Related Tasks:
  - [T-3n2ej-workspace-separation](../tasks/T-3n2ej-workspace-separation/README.md)
  - [T-nazbq-firmware-to-core-migration](../tasks/T-nazbq-firmware-to-core-migration/README.md)

## Executive Summary

This analysis evaluates the feasibility and approach for refactoring `pico_trail` from a single-crate architecture into a Cargo Workspace with two primary crates: `crates/core` (pure `no_std` business logic) and `crates/firmware` (Embassy/RP2040 entry point). The goal is to completely eliminate `#[cfg(feature = "...")]` flags from core business logic, enabling straightforward host testing without embedded feature flags.

Previous analysis (AN-li4m8) identified \~150 feature gates and proposed conservative cleanup. This analysis explores the more aggressive workspace split approach, which offers cleaner architectural boundaries at the cost of significant refactoring effort. The key insight is applying "inversion of control" - async and logging are firmware concerns, not core concerns.

## Problem Space

### Current State

The pico_trail codebase exhibits the following characteristics:

| Metric                                       | Value                     |
| -------------------------------------------- | ------------------------- |
| Total Rust files                             | 164 files in `src/`       |
| Total lines of code                          | \~46,000 lines            |
| Files using `#[cfg(feature = "embassy")]`    | 34 files, 120 occurrences |
| Files using `#[cfg(feature = "pico2_w")]`    | 13 files, 51 occurrences  |
| Files using `#[cfg_attr(feature = "defmt")]` | 12 files, 13 occurrences  |

#### Core Problems

1. **Embedded observability in type definitions**:

   ```rust
   // PROBLEM: Type definition includes embedded logging concern
   #[cfg_attr(feature = "defmt", derive(defmt::Format))]
   pub struct NavigationState { ... }
   ```

2. **Async runtime coupled to business logic**:

   ```rust
   // PROBLEM: Control algorithm requires Embassy async
   #[cfg(feature = "embassy")]
   pub async fn execute_control_loop(&mut self) {
       Timer::after_millis(10).await;
       // ... control logic
   }
   ```

3. **Duplicate stub implementations**:

   ```rust
   #[cfg(feature = "embassy")]
   async fn delay_ms(ms: u64) { embassy_time::Timer::after_millis(ms).await; }

   #[cfg(not(feature = "embassy"))]
   async fn delay_ms(_ms: u64) { /* stub */ }
   ```

4. **Host tests require stubbing**:
   - `cargo test --lib` requires mock implementations
   - Test coverage is limited by feature gate complexity
   - CI must maintain two compilation paths

### Desired State

1. **Pure `no_std` core crate**: Zero `cfg` flags, zero Embassy imports, zero `defmt` imports
2. **Clear firmware boundary**: All Embassy/HAL code isolated in firmware crate
3. **Native host testing**: `cargo test` in core crate without any feature flags
4. **Trait-based integration**: Core defines traits, firmware implements them

### Gap Analysis

| Aspect               | Current            | Desired              | Gap                           |
| -------------------- | ------------------ | -------------------- | ----------------------------- |
| Core crate cfg usage | \~150 gates        | 0                    | Complete elimination required |
| Logging in core      | defmt via cfg_attr | None (trait-based)   | Logging facade needed         |
| Async in core        | Embassy async fn   | Tick-based sync API  | Inversion of control          |
| Type observability   | Embedded in types  | External impl blocks | Decouple Format from core     |
| Workspace structure  | Single crate       | 2 crates minimum     | Directory restructure         |

## Stakeholder Analysis

| Stakeholder      | Interest/Need                            | Impact | Priority |
| ---------------- | ---------------------------------------- | ------ | -------- |
| Developers       | Simpler host testing, faster iteration   | High   | P0       |
| Maintainers      | Reduced cfg complexity, clear boundaries | High   | P0       |
| CI/CD            | Single test path, no feature flags       | Medium | P1       |
| Future platforms | Clean abstraction layer for ESP32/STM32  | High   | P1       |

## Research & Discovery

### Technical Investigation

#### Reference Architecture

```
pico_trail/
├── Cargo.toml            # Workspace root
├── crates/
│   ├── core/             # Pure no_std logic
│   │   ├── Cargo.toml    # Minimal deps: no embassy, no defmt
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── navigation/
│   │       ├── control/
│   │       └── ...
│   └── firmware/         # Embassy binary
│       ├── Cargo.toml    # Depends on: core, embassy-*, defmt
│       └── src/
│           ├── main.rs
│           ├── tasks/
│           └── ...
```

#### Defmt Problem and Solution

**Current (wrong)**:

```rust
// core/src/navigation.rs
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct State { pub x: i32 }
```

**Proposed (correct)**:

```rust
// crates/core/src/navigation.rs - NO attributes
pub struct State { pub x: i32 }

// crates/firmware/src/formatters.rs - External impl
impl defmt::Format for core::State {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "State {{ x: {} }}", self.x)
    }
}
```

#### Async Problem and Solution

**Current (wrong)**:

```rust
// core/src/control.rs
#[cfg(feature = "embassy")]
pub async fn control_loop(&mut self) {
    loop {
        self.update();
        Timer::after_millis(10).await;
    }
}
```

**Proposed (correct)**:

```rust
// crates/core/src/control.rs - Sync tick API
pub trait Tick {
    fn tick(&mut self, dt_ms: u32);
}

impl Tick for Controller {
    fn tick(&mut self, dt_ms: u32) {
        self.update(dt_ms);
    }
}

// crates/firmware/src/tasks/control.rs - Async wrapper
#[embassy_executor::task]
async fn control_task(mut ctrl: Controller) {
    loop {
        ctrl.tick(10);
        Timer::after_millis(10).await;
    }
}
```

#### Code Migration Analysis

Based on current module structure, the following migration is proposed:

| Current Module               | Target Crate            | Rationale                               |
| ---------------------------- | ----------------------- | --------------------------------------- |
| `src/core/traits/`           | `crates/core`           | Pure trait definitions                  |
| `src/core/arming/`           | `crates/core`           | State machine logic                     |
| `src/core/mission/`          | `crates/core`           | Mission state management                |
| `src/core/parameters/`       | `crates/core`           | Parameter definitions (not persistence) |
| `src/core/scheduler/`        | `crates/core` (partial) | Task definitions, not executor          |
| `src/libraries/`             | `crates/core`           | Kinematics, algorithms                  |
| `src/subsystems/ahrs/`       | `crates/core`           | AHRS math (not sensor drivers)          |
| `src/subsystems/navigation/` | `crates/core`           | Navigation algorithms                   |
| `src/rover/mode/`            | `crates/core`           | Mode state machines                     |
| `src/communication/mavlink/` | `crates/core` (partial) | Message handling, not transport         |
| `src/platform/`              | `crates/firmware`       | All HAL code                            |
| `src/devices/`               | `crates/firmware`       | Device drivers                          |
| `examples/`                  | `crates/firmware`       | Embassy entry points                    |

#### Core Crate Rules (Strict)

The core crate must adhere to these constraints:

| Allowed                   | Forbidden                       |
| ------------------------- | ------------------------------- |
| `#![no_std]`              | `std`                           |
| `libm` for math           | `embassy-*`                     |
| Pure data types           | `defmt`                         |
| Traits and impls          | HAL types                       |
| `heapless` collections    | `#[cfg(feature = ...)]`         |
| Algorithm implementations | `async fn` (executor-dependent) |

#### Firmware Crate Rules (Embassy-First)

The firmware crate targets Embassy/RP2350 exclusively. Key design principle:

**All Embassy dependencies are non-optional.** Since firmware always targets embedded (no host tests for firmware logic), feature flags for Embassy/defmt/HAL are unnecessary overhead.

| Approach                   | Rationale                                       |
| -------------------------- | ----------------------------------------------- |
| Embassy deps non-optional  | Firmware is Embassy-only, no host test target   |
| defmt always enabled       | RTT logging is standard for embedded debugging  |
| RP2350 HAL always included | Single target platform, no multi-platform gates |
| Minimal feature flags      | Only for vehicle type (rover) or device options |

**Acceptable features in firmware:**

- `rover` - Vehicle type selection (future: `boat`, `copter`)
- `gps-ublox` - GPS vendor-specific initialization
- `usb_serial` - Optional USB serial console

**Eliminated features:**

- `pico2_w` - Redundant (firmware is RP2350-only)
- `embassy` - Redundant (firmware is Embassy-only)
- `mock` - Not needed (mocks belong in core crate tests)

### Competitive Analysis

Similar embedded Rust projects use this pattern:

- **Embedded-HAL**: Defines traits in core, implementations in HAL crates
- **RTIC**: Separates app logic from runtime
- **Embassy examples**: Many split business logic from hardware code

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Core crate must be pure `no_std` with no Embassy dependencies
  - Rationale: Enable host testing without embedded toolchain
  - Acceptance Criteria: `cargo test` passes in `crates/core` without any feature flags

- [ ] **FR-DRAFT-2**: Core crate must define trait abstractions for all platform services
  - Rationale: Decouple business logic from specific implementations
  - Acceptance Criteria: Traits exist for Time, Sync, IO operations

- [ ] **FR-DRAFT-3**: Firmware crate must implement core traits using Embassy
  - Rationale: Integrate core logic with actual hardware
  - Acceptance Criteria: All core traits have Embassy implementations

- [ ] **FR-DRAFT-4**: External impl blocks must provide observability (defmt/Debug) for core types
  - Rationale: Keep core types clean while enabling debugging
  - Acceptance Criteria: No `#[cfg_attr]` or `derive(defmt::Format)` in core

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Core crate must have zero `#[cfg(feature = ...)]` directives
  - Category: Maintainability
  - Rationale: Eliminate conditional compilation complexity
  - Target: 0 cfg directives in `crates/core/`

- [ ] **NFR-DRAFT-2**: Host test execution must not require any feature flags
  - Category: Testability
  - Rationale: Simplify CI and developer workflow
  - Target: `cd crates/core && cargo test` with no `--features`

- [ ] **NFR-DRAFT-3**: CI must enforce zero cfg usage in core crate
  - Category: Maintainability
  - Rationale: Prevent regression of cfg leakage into core after refactoring
  - Target: CI lint script fails if any `#[cfg(` pattern found in `crates/core/`

- [ ] **NFR-DRAFT-4**: Firmware build time should not regress more than 10%
  - Category: Performance
  - Rationale: Workspace overhead should be minimal
  - Target: Incremental build within 10% of current time

## Design Considerations

### Technical Constraints

1. **Orphan Rule**: Cannot impl external traits for external types
   - Impact: May need newtype wrappers for some types
   - Mitigation: Design core types to be owned by core crate

2. **Async Trait Limitations**: `async fn` in traits requires `async_trait` or boxing
   - Impact: Core cannot use async traits directly
   - Mitigation: Use sync tick-based API pattern

3. **Embassy Task Macro**: `#[embassy_executor::task]` requires specific function signatures
   - Impact: Task definitions must stay in firmware
   - Mitigation: Core provides logic, firmware wraps in tasks

4. **Heapless/Alloc**: Core may need collections without std
   - Impact: Limited to fixed-size structures or `heapless`
   - Mitigation: Already using `heapless` in current codebase

### Potential Approaches

#### Option A: Two-Crate Split (Recommended)

Split into `crates/core` and `crates/firmware`:

- Pros: Clear boundary, straightforward migration, matches common patterns
- Cons: Large initial refactoring effort
- Effort: High

#### Option B: Three-Crate Split

Add `crates/embassy-integration` between core and firmware:

- Pros: Finer separation, Embassy traits reusable across platforms
- Cons: More complex dependency graph, more boilerplate
- Effort: Very High

#### Option C: Incremental Module Extraction

Extract modules one-by-one to core, keeping single crate:

- Pros: Lower risk, gradual migration
- Cons: Incomplete separation, cfg still needed during transition
- Effort: Medium (but ongoing)

### Architecture Impact

This change requires a new ADR to document:

1. Workspace structure decision
2. Trait abstraction patterns
3. Migration strategy (phased vs big-bang)
4. CI/CD changes for multi-crate builds

## Risk Assessment

| Risk                              | Probability | Impact | Mitigation Strategy                            |
| --------------------------------- | ----------- | ------ | ---------------------------------------------- |
| Breaking existing functionality   | Medium      | High   | Comprehensive test suite before migration      |
| Extended timeline                 | High        | Medium | Phase migration by module                      |
| Inter-crate dependency complexity | Medium      | Medium | Careful API design, minimize cross-crate types |
| Build system complexity           | Low         | Low    | Workspace handles most complexity              |
| Team unfamiliarity with pattern   | Low         | Low    | Document patterns, provide examples            |

## Open Questions

- [ ] Should we include a third `crates/mavlink` for protocol-only code? → Next step: Evaluate MAVLink coupling during implementation
- [ ] How to handle parameters that need persistence (firmware) vs definition (core)? → Next step: Design parameter abstraction trait
- [ ] Should rover modes be in core or firmware given async requirements? → Next step: Prototype tick-based mode implementation
- [ ] What is the minimum set of traits needed for core abstraction? → Next step: Inventory all Embassy usages in current core modules

## Recommendations

### Immediate Actions

1. **Create prototype**: Extract one module (e.g., `src/libraries/kinematics`) to validate pattern
2. **Inventory dependencies**: Document all Embassy/defmt usages that must be abstracted
3. **Define core traits**: Draft `TimeSource`, `SyncState`, and `Platform` traits
4. **Create CI lint**: Implement a script to verify core crate has no `cfg` attributes

### Next Steps

1. [ ] Create formal requirements: FR-xxxxx (workspace structure), NFR-xxxxx (zero cfg)
2. [ ] Draft ADR for: Workspace architecture and trait abstraction strategy
3. [ ] Create task for: Phase 1 - Workspace scaffolding and `crates/core` initialization
4. [ ] Prototype: Migrate `libraries/kinematics` to validate approach

### Out of Scope

- ESP32/STM32 platform support - Future work after separation
- Runtime trait selection - Compile-time only
- Dynamic feature loading - Not applicable for embedded

## Appendix

### References

- [AN-li4m8-feature-gate-reduction](../analysis/AN-li4m8-feature-gate-reduction.md) - Previous feature gate analysis
- [Embassy Book](https://embassy.dev/book/) - Embassy patterns and best practices
- [Rust Embedded Book](https://docs.rust-embedded.org/book/) - no_std patterns

### Migration Effort Estimate by Module

| Module                  | Files | LOC    | Embassy Deps          | Migration Complexity |
| ----------------------- | ----- | ------ | --------------------- | -------------------- |
| `libraries/kinematics`  | 3     | \~500  | 0                     | Low                  |
| `libraries/rc_channel`  | 2     | \~300  | 2 (Mutex)             | Medium               |
| `core/arming`           | 5     | \~800  | 4 (async)             | Medium               |
| `core/mission`          | 3     | \~600  | 3 (async)             | Medium               |
| `rover/mode`            | 7     | \~2000 | 7 (async)             | High                 |
| `subsystems/navigation` | 4     | \~1000 | 4 (async)             | High                 |
| `communication/mavlink` | 15    | \~3000 | 12 (async, transport) | Very High            |

### Tick-Based API Example

```rust
// crates/core/src/control.rs

/// Core controller with no async dependencies
pub struct Controller {
    state: ControlState,
    pid: PidController,
}

impl Controller {
    pub fn new(config: ControlConfig) -> Self {
        Self {
            state: ControlState::default(),
            pid: PidController::new(config.pid),
        }
    }

    /// Pure synchronous update - called by firmware at fixed rate
    pub fn tick(&mut self, dt_ms: u32, input: &SensorInput) -> ControlOutput {
        self.state.update(dt_ms);
        let error = self.state.compute_error(input);
        self.pid.update(error, dt_ms)
    }
}

// crates/firmware/src/tasks/control.rs

use core::Controller;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn control_task(mut ctrl: Controller, sensors: &'static SensorState) {
    const TICK_MS: u64 = 10;
    loop {
        let input = sensors.read().await;
        let output = ctrl.tick(TICK_MS as u32, &input);
        apply_output(output).await;
        Timer::after_millis(TICK_MS).await;
    }
}
```

### Proposed CI Lint Script

A lint script should be added to CI to enforce the zero-cfg rule in core crate:

```bash
#!/bin/bash
# scripts/check-core-no-cfg.sh
# Ensures crates/core has no #[cfg(...)] attributes

set -e

CORE_PATH="crates/core/src"

if [ ! -d "$CORE_PATH" ]; then
    echo "Core crate not found at $CORE_PATH"
    exit 0  # Skip if workspace not yet created
fi

# Search for cfg patterns (excluding cfg_attr for Debug which is allowed)
VIOLATIONS=$(grep -rn '#\[cfg(' "$CORE_PATH" --include="*.rs" || true)

if [ -n "$VIOLATIONS" ]; then
    echo "ERROR: Found #[cfg(...)] in core crate (forbidden):"
    echo "$VIOLATIONS"
    echo ""
    echo "Core crate must have zero cfg attributes."
    echo "Move conditional code to crates/firmware instead."
    exit 1
fi

# Also check for cfg_attr (defmt derives, etc.)
CFG_ATTR=$(grep -rn '#\[cfg_attr(' "$CORE_PATH" --include="*.rs" || true)

if [ -n "$CFG_ATTR" ]; then
    echo "ERROR: Found #[cfg_attr(...)] in core crate (forbidden):"
    echo "$CFG_ATTR"
    echo ""
    echo "Use external impl blocks in firmware crate instead."
    exit 1
fi

# Check for forbidden imports
FORBIDDEN_IMPORTS=$(grep -rn 'use embassy\|use defmt' "$CORE_PATH" --include="*.rs" || true)

if [ -n "$FORBIDDEN_IMPORTS" ]; then
    echo "ERROR: Found forbidden imports in core crate:"
    echo "$FORBIDDEN_IMPORTS"
    echo ""
    echo "Core crate must not depend on embassy or defmt."
    exit 1
fi

echo "✓ Core crate is clean: no cfg, cfg_attr, or forbidden imports found."
```

This script should be integrated into the CI workflow alongside existing checks:

```yaml
# .github/workflows/ci.yml (proposed addition)
- name: Check core crate purity
  run: ./scripts/check-core-no-cfg.sh
```
