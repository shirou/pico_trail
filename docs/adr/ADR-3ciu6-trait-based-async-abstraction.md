# ADR-3ciu6 Trait-Based Async Abstraction

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-jpmdj-trait-based-async-abstraction](../requirements/FR-jpmdj-trait-based-async-abstraction.md)
  - [NFR-wl974-feature-gate-reduction](../requirements/NFR-wl974-feature-gate-reduction.md)
  - [NFR-nmmu0-platform-code-isolation](../requirements/NFR-nmmu0-platform-code-isolation.md)
- Related Tasks:
  - [T-d9rim-trait-based-async-abstraction](../tasks/T-d9rim-trait-based-async-abstraction/README.md)

## Context

The pico_trail codebase currently has approximately 150 feature gates (`#[cfg(feature = "...")]`), primarily for `pico2_w` and `embassy` features. This creates several problems:

1. **Semantic confusion**: `pico2_w` is used for both platform-specific HAL code and general Embassy async code
2. **Code duplication**: Many modules have parallel implementations for embassy and non-embassy builds
3. **Testing friction**: Host tests require complex stub implementations with inverse feature gates
4. **Maintenance burden**: Changes often require updating multiple conditional blocks

The current pattern looks like:

```rust
// Current: Duplicate implementations
#[cfg(feature = "embassy")]
pub async fn handle_message(&self) -> bool {
    let state = STATE.lock().await;
    // Real implementation
}

#[cfg(not(feature = "embassy"))]
pub async fn handle_message(&self) -> bool {
    // Stub for cargo test
    false
}
```

We need a solution that:

- Separates platform-specific code from async runtime code
- Enables host testing without embedded dependencies
- Reduces feature gate count significantly (target: ≤60)
- Maintains zero-cost abstraction for embedded targets

## Success Metrics

- Feature gate count reduced from \~150 to ≤60
- `cargo test --lib` passes without feature flags
- No performance regression in control loop (50Hz maintained)
- All existing examples compile and run correctly

## Decision

We will introduce trait-based abstractions for async runtime operations, enabling core logic to remain runtime-agnostic.

### Decision Drivers

- Maintainability: Reduce cognitive load from feature gates
- Testability: Enable host testing without embedded features
- Portability: Support future async runtimes (e.g., ESP32's async HAL)
- Rust idioms: Prefer traits over conditional compilation for polymorphism

### Considered Options

- Option A: Feature Gate Cleanup (reclassify gates, keep current pattern)
- Option B: Trait-Based Abstraction (introduce runtime traits)
- Option C: Workspace Split (separate crates for core/embassy/platform)

### Option Analysis

- Option A — Pros: Low risk, incremental | Cons: Still requires many stubs, limited improvement
- Option B — Pros: Clean abstraction, eliminates duplicates, testable | Cons: Refactoring effort, API changes
- Option C — Pros: Cleanest separation | Cons: Major restructure, complex inter-crate deps, premature

## Rationale

Option B provides the best balance between improvement and effort:

1. **Eliminates code duplication**: One implementation per function, behavior varies via trait
2. **Clear boundaries**: Traits define what core needs, implementations are swappable
3. **Incremental adoption**: Can migrate one module at a time
4. **Future-proof**: Adding new platforms/runtimes requires only new trait implementations

Option A was rejected because it doesn't solve the fundamental problem of duplicate implementations. Option C was rejected as premature—the single-crate structure is adequate once trait boundaries are established.

## Consequences

### Positive

- Feature gate count reduced by 60-70%
- Host tests run without embedded dependencies
- Clear separation between core logic and runtime specifics
- Easier onboarding for new contributors
- Foundation for future platform support (ESP32, STM32)

### Negative

- Initial refactoring effort (estimated 2-3 phases)
- Slight API surface changes for internal modules
- Learning curve for trait-based patterns

### Neutral

- Compile time may slightly increase due to more generic code (mitigated by monomorphization)
- Some modules may need restructuring to accept trait parameters

## Implementation Notes

### Phase 1: Time Abstraction

Define and implement `TimeSource` trait:

```rust
// src/core/traits/time.rs
pub trait TimeSource: Clone {
    fn now_ms(&self) -> u64;
    fn now_us(&self) -> u64;
}

// Embassy implementation (feature-gated)
#[cfg(feature = "embassy")]
#[derive(Clone, Copy)]
pub struct EmbassyTime;

#[cfg(feature = "embassy")]
impl TimeSource for EmbassyTime {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }

    fn now_us(&self) -> u64 {
        embassy_time::Instant::now().as_micros()
    }
}

// Mock implementation (always available)
#[derive(Clone)]
pub struct MockTime {
    current_us: core::cell::Cell<u64>,
}

impl MockTime {
    pub fn new() -> Self {
        Self { current_us: core::cell::Cell::new(0) }
    }

    pub fn advance(&self, us: u64) {
        self.current_us.set(self.current_us.get() + us);
    }
}

impl TimeSource for MockTime {
    fn now_ms(&self) -> u64 {
        self.current_us.get() / 1000
    }

    fn now_us(&self) -> u64 {
        self.current_us.get()
    }
}
```

### Phase 2: State Abstraction

Define `SharedState` trait for synchronized access:

```rust
// src/core/traits/sync.rs
pub trait SharedState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R;

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R;
}

// Embassy implementation using Mutex
#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

#[cfg(feature = "embassy")]
impl<T> SharedState<T> for Mutex<CriticalSectionRawMutex, core::cell::RefCell<T>> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        self.lock(|cell| f(&cell.borrow()))
    }

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        self.lock(|cell| f(&mut cell.borrow_mut()))
    }
}

// Mock implementation using RefCell
pub struct MockState<T> {
    inner: core::cell::RefCell<T>,
}

impl<T> SharedState<T> for MockState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        f(&self.inner.borrow())
    }

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        f(&mut self.inner.borrow_mut())
    }
}
```

### Phase 3: Module Migration

Migrate high-impact modules in order:

1. `src/core/mission/state.rs` (18 gates) - Mission state management
2. `src/core/log_router.rs` (10 gates) - Logging infrastructure
3. `src/communication/mavlink/handlers/` (mixed) - Message handlers
4. `src/libraries/rc_channel/mod.rs` (3 gates) - RC input processing
5. `src/subsystems/navigation/mod.rs` (6 gates) - Navigation state

### Directory Structure

```
src/core/traits/
├── mod.rs          # pub use time::*, sync::*;
├── time.rs         # TimeSource trait + implementations
└── sync.rs         # SharedState trait + implementations
```

### Migration Pattern

Before (with feature gates):

```rust
#[cfg(feature = "embassy")]
pub static MISSION_STATE: Mutex<CriticalSectionRawMutex, MissionState> = ...;

#[cfg(feature = "embassy")]
pub fn get_state() -> MissionState {
    critical_section::with(|cs| MISSION_STATE.borrow(cs).clone())
}

#[cfg(not(feature = "embassy"))]
pub fn get_state() -> MissionState {
    MissionState::default() // stub
}
```

After (with traits):

```rust
pub struct MissionManager<S: SharedState<MissionState>> {
    state: S,
}

impl<S: SharedState<MissionState>> MissionManager<S> {
    pub fn get_state(&self) -> MissionState {
        self.state.with(|s| s.clone())
    }
}

// Usage in embedded:
// let manager = MissionManager { state: &EMBASSY_MISSION_STATE };

// Usage in tests:
// let manager = MissionManager { state: MockState::new(MissionState::default()) };
```

## Examples

```rust
// Example: Control loop using TimeSource trait
pub fn run_control_loop<T: TimeSource>(time: &T, last_time: &mut u64) {
    let now = time.now_us();
    let dt = now - *last_time;
    *last_time = now;

    // Control logic using dt...
}

// Embedded usage
#[cfg(feature = "embassy")]
let time = EmbassyTime;

// Test usage
let time = MockTime::new();
time.advance(20_000); // Simulate 20ms
```

## Open Questions

- [x] Should we use `async-trait` crate or wait for stable native async traits? → Use sync traits first, async later if needed
- [ ] How to handle global static state that's currently feature-gated? → Evaluate lazy_static or OnceCell patterns
- [ ] Should trait bounds be added to existing public types? → Minimize public API changes, use internal wrappers

## External References

- [Embassy Async Framework](https://embassy.dev/) - Target async runtime
- [Rust API Guidelines - Traits](https://rust-lang.github.io/api-guidelines/flexibility.html) - Trait design patterns
- [Dependency Injection in Rust](https://blog.logrocket.com/dependency-injection-rust/) - DI patterns for Rust
