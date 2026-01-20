# AN-li4m8 Feature Gate Reduction Analysis

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-kir7h-platform-abstraction](../analysis/AN-kir7h-platform-abstraction.md)
- Related Requirements:
  - [NFR-nmmu0-platform-code-isolation](../requirements/NFR-nmmu0-platform-code-isolation.md)
  - [FR-jpmdj-trait-based-async-abstraction](../requirements/FR-jpmdj-trait-based-async-abstraction.md)
  - [NFR-wl974-feature-gate-reduction](../requirements/NFR-wl974-feature-gate-reduction.md)
- Related ADRs:
  - [ADR-3ciu6-trait-based-async-abstraction](../adr/ADR-3ciu6-trait-based-async-abstraction.md)
- Related Tasks:
  - [T-d9rim-trait-based-async-abstraction](../tasks/T-d9rim-trait-based-async-abstraction/README.md)
  - [T-3n2ej-workspace-separation](../tasks/T-3n2ej-workspace-separation/README.md)

## Executive Summary

The pico_trail codebase currently contains approximately **150+ feature gates** (`#[cfg(feature = "...")]`), primarily using `pico2_w` and `embassy` features. Analysis reveals significant semantic confusion between platform-specific code (RP2350 hardware) and async runtime code (Embassy framework), leading to code duplication, maintenance burden, and unclear architecture boundaries.

This analysis proposes a **3-layer separation strategy**: (1) Platform-independent core, (2) Async runtime abstraction, (3) Platform-specific implementations. This approach could reduce feature gates by **60-70%** while improving code clarity, testability, and future platform support.

## Problem Space

### Current State

#### Feature Gate Distribution

Searching `src/` reveals the following feature gate usage:

| Feature            | Count | Primary Purpose                    |
| ------------------ | ----- | ---------------------------------- |
| `pico2_w`          | \~80  | RP2350 platform + mixed async code |
| `embassy`          | \~50  | Embassy async primitives           |
| `embassy-executor` | \~5   | Async task definitions             |
| `embassy-time`     | \~3   | Time-specific code                 |
| `defmt`            | \~2   | Logging                            |

#### Key Problems

1. **Semantic Confusion**: `pico2_w` is used for code that should work on any Embassy-supported platform

   ```rust
   // Example: This is Embassy code, not RP2350-specific
   #[cfg(feature = "pico2_w")]
   #[embassy_executor::task]
   pub async fn mavlink_task(...) { ... }
   ```

2. **Duplicate Implementations**: Many modules have parallel implementations:
   - `src/communication/mavlink/transport/udp.rs`: Two complete `UdpTransport` structs
   - `src/rover/mode/manual.rs`: Two `new()` constructors
   - `src/core/arming/tasks.rs`: Async functions + stub module

3. **Inconsistent Boundaries**: Platform code leaks outside `src/platform/`:
   - `src/devices/imu/icm20948.rs` - Uses RP2350 I2C HAL directly
   - `src/core/parameters/saver.rs` - Uses RP2350 flash directly
   - `src/platform/rp2350/motor.rs` - Contains both HAL code and generic motor logic

4. **Testing Friction**: Host tests require stub implementations gated with `#[cfg(not(feature = "embassy"))]`, creating maintenance burden

### Desired State

1. **Clear Layer Separation**:
   - Core logic: No feature gates, pure Rust
   - Async abstraction: Single `embassy` feature for all async code
   - Platform HAL: Platform features (`pico2_w`, future `esp32`) confined to `src/platform/`

2. **Reduced Feature Gates**: Target 50-60 feature gates (60-70% reduction)

3. **Improved Testability**: Core logic testable without platform features

4. **Future Platform Support**: Adding ESP32 or STM32 requires only implementing `src/platform/<platform>/` traits

### Gap Analysis

| Aspect                    | Current        | Desired  | Gap                            |
| ------------------------- | -------------- | -------- | ------------------------------ |
| Feature gate count        | \~150          | \~50     | -100 gates                     |
| Platform code isolation   | Partial        | Complete | HAL leakage in devices/, core/ |
| Async code clarity        | Mixed          | Clear    | pico2_w/embassy confusion      |
| Duplicate implementations | Many           | None     | Trait-based abstraction needed |
| Host test coverage        | Stub-dependent | Native   | Remove async stubs             |

## Stakeholder Analysis

| Stakeholder           | Interest/Need                        | Impact | Priority |
| --------------------- | ------------------------------------ | ------ | -------- |
| Maintainers           | Reduced complexity, clear boundaries | High   | P0       |
| Contributors          | Understandable architecture          | Medium | P1       |
| Future platform ports | Clean abstraction layer              | High   | P1       |

## Research & Discovery

### Technical Investigation

#### Feature Gate Categories

Analysis of all `#[cfg(feature = "...")]` patterns in `src/`:

**Category 1: Correctly Platform-Specific** (\~30 gates)

- `src/platform/mod.rs`: `#[cfg(feature = "pico2_w")] pub mod rp2350;`
- `src/platform/rp2350/*`: All RP2350 HAL code
- Correctly isolated, no changes needed

**Category 2: Misclassified as Platform-Specific** (\~50 gates)

```rust
// Should be #[cfg(feature = "embassy")], not pico2_w
#[cfg(feature = "pico2_w")]
use embassy_time::{Duration, Instant};

#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
pub async fn control_loop_task(...) { ... }
```

Files affected:

- `src/communication/mavlink/task.rs`
- `src/core/scheduler/tasks/control.rs`
- `src/communication/mavlink/transport_router.rs`
- `src/communication/mavlink/transport/udp.rs`

**Category 3: Async Runtime Abstraction** (\~40 gates)

```rust
// Correct use of embassy feature
#[cfg(feature = "embassy")]
pub static RC_INPUT: Mutex<CriticalSectionRawMutex, RcInput> = Mutex::new(RcInput::new());
```

Files affected:

- `src/libraries/rc_channel/mod.rs`
- `src/core/mission/state.rs`
- `src/subsystems/navigation/mod.rs`

**Category 4: Duplicate Stubs for Host Testing** (\~30 gates)

```rust
#[cfg(feature = "embassy")]
pub async fn handle_rc_channels(...) { ... }

#[cfg(not(feature = "embassy"))]
pub async fn handle_rc_channels(...) {
    // Stub: does nothing, just for cargo test
}
```

Files affected:

- `src/communication/mavlink/handlers/rc_input.rs`
- `src/communication/mavlink/handlers/navigation.rs`
- `src/communication/mavlink/handlers/command.rs`

#### Proposed Layer Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│ Layer 0: Platform-Independent Core (NO feature gates)          │
│ - Data structures (Waypoint, RcInput, NavigationOutput)        │
│ - Algorithms (PID, S-curve, AHRS math)                         │
│ - Protocol logic (MAVLink message handling, parameter defs)    │
│ - Pure functions (coordinate transforms, unit conversions)     │
├─────────────────────────────────────────────────────────────────┤
│ Layer 1: Async Runtime Abstraction (#[cfg(feature = "embassy")])│
│ - Global state (Mutex<CriticalSectionRawMutex, T>)             │
│ - Async functions (async fn, .await)                           │
│ - Time operations (embassy_time::Instant, Duration)            │
│ - Synchronization (Channel, Signal)                            │
├─────────────────────────────────────────────────────────────────┤
│ Layer 2: Platform HAL (#[cfg(feature = "pico2_w")] etc.)       │
│ - src/platform/rp2350/*: UART, I2C, SPI, PWM, GPIO, Flash      │
│ - Device drivers using HAL (moved from src/devices/)           │
│ - Embassy executor tasks (#[embassy_executor::task])           │
│ - Network stack (cyw43, embassy-net)                           │
└─────────────────────────────────────────────────────────────────┘
```

#### Alternative: Trait-Based Abstraction

Instead of feature gates for async code, use traits:

```rust
// Core trait (no feature gates)
pub trait TimeSource {
    fn now_ms(&self) -> u64;
}

// Embassy implementation
#[cfg(feature = "embassy")]
pub struct EmbassyTime;

#[cfg(feature = "embassy")]
impl TimeSource for EmbassyTime {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
}

// Host test implementation
#[cfg(test)]
pub struct MockTime(pub u64);

#[cfg(test)]
impl TimeSource for MockTime {
    fn now_ms(&self) -> u64 { self.0 }
}
```

This approach eliminates duplicate function implementations but requires dependency injection.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Platform code must be isolated to `src/platform/` directory
  - Rationale: Enables clean multi-platform support and prevents HAL leakage
  - Acceptance Criteria: No HAL imports outside `src/platform/`; CI script passes

- [ ] **FR-DRAFT-2**: Async runtime code must use `embassy` feature, not platform features
  - Rationale: Embassy code works across platforms (RP2350, ESP32, STM32)
  - Acceptance Criteria: `#[cfg(feature = "pico2_w")]` only in `src/platform/`

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Feature gate count should not exceed 60
  - Category: Maintainability
  - Rationale: Reduces cognitive load and maintenance burden
  - Target: ≤60 feature gates total (from current \~150)

- [ ] **NFR-DRAFT-2**: Host tests must run without embedded features
  - Category: Testability
  - Rationale: CI efficiency and developer experience
  - Target: `cargo test --lib` passes without feature flags

## Design Considerations

### Technical Constraints

1. **Embassy Dependency**: Embassy crates are embedded-only (`no_std`), cannot compile on host
2. **Global State Pattern**: Current architecture uses static Mutex for shared state
3. **Backward Compatibility**: Existing examples must continue to work
4. **Build Time**: Conditional compilation affects incremental build cache

### Potential Approaches

#### Option A: Feature Gate Cleanup (Conservative)

Reclassify existing feature gates without architectural changes:

- Change `#[cfg(feature = "pico2_w")]` to `#[cfg(feature = "embassy")]` where appropriate

- Remove duplicate stub implementations

- Move HAL-dependent code to `src/platform/`

- Pros: Low risk, incremental changes, preserves current patterns

- Cons: Still requires some feature gates for host tests

- Effort: Medium

#### Option B: Trait-Based Abstraction (Moderate)

Introduce traits for time, sync primitives, and I/O:

- Define platform-agnostic traits in core

- Implement for Embassy and Mock

- Use dependency injection instead of global static

- Pros: Eliminates most duplicate implementations, cleaner architecture

- Cons: Requires significant refactoring, changes API surface

- Effort: High

#### Option C: Workspace Split (Aggressive)

Split into multiple crates:

- `pico_trail_core`: Platform-independent logic (no features)

- `pico_trail_embassy`: Embassy runtime bindings

- `pico_trail_rp2350`: RP2350 HAL bindings

- Pros: Cleanest separation, explicit dependencies

- Cons: Major restructure, complex inter-crate dependencies

- Effort: Very High

### Architecture Impact

- Option A: No new ADRs needed, updates NFR-nmmu0 compliance
- Option B: Requires ADR for trait-based abstraction pattern
- Option C: Requires ADR for workspace structure

## Risk Assessment

| Risk                            | Probability | Impact | Mitigation Strategy               |
| ------------------------------- | ----------- | ------ | --------------------------------- |
| Breaking existing examples      | Medium      | High   | Comprehensive test before/after   |
| Increased compile time          | Low         | Medium | Measure build times before/after  |
| Incomplete abstraction          | Medium      | Medium | Start with one module as POC      |
| Regression in embedded behavior | Medium      | High   | Hardware validation after changes |

## Open Questions

- [ ] Should async handler functions use traits or continue with feature-gated implementations?
- [ ] How to handle `embassy_executor::task` macro which requires platform feature?
- [ ] Should device drivers (GPS, IMU) use platform traits or embedded-hal traits directly?

## Recommendations

### Immediate Actions

1. **Audit and Reclassify**: Change `pico2_w` to `embassy` for non-platform-specific async code
2. **Consolidate Stubs**: Remove duplicate stub implementations where possible
3. **Enforce HAL Isolation**: Update `scripts/check-platform-isolation.sh` to catch violations

### Next Steps

1. [ ] Create formal requirements: FR-xxxxx (platform isolation), NFR-xxxxx (feature gate limit)
2. [ ] Draft ADR for: Feature gate reduction approach (Option A vs B)
3. [ ] Create task for: Phase 1 cleanup (reclassify feature gates)
4. [ ] POC: Refactor one module (e.g., `rc_channel`) using trait-based approach

### Out of Scope

- Workspace split (Option C) - Too disruptive for current project stage
- ESP32/STM32 platform support - Future work after architecture cleanup
- Runtime feature detection - Not applicable for embedded targets

## Appendix

### Feature Gate Inventory

Files with highest feature gate count:

| File                                            | Gates | Notes                       |
| ----------------------------------------------- | ----- | --------------------------- |
| `src/core/mission/state.rs`                     | 18    | Mostly embassy async state  |
| `src/communication/mavlink/handlers/command.rs` | 12    | Mixed embassy/pico2_w       |
| `src/platform/rp2350/motor.rs`                  | 11    | Correctly platform-specific |
| `src/core/log_router.rs`                        | 10    | Embassy logging abstraction |
| `src/communication/mavlink/transport/udp.rs`    | 10    | Duplicate implementations   |

### References

- [Embassy Book: Platform Support](https://embassy.dev/book/#_supported_targets)
- [Rust Feature Flags Best Practices](https://doc.rust-lang.org/cargo/reference/features.html)
- [ArduPilot HAL Architecture](https://ardupilot.org/dev/docs/apmcopter-code-overview.html)
