# T-00025 Trait-Based Async Abstraction Design

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [T-00025-plan](./plan.md)

## Overview

This design introduces trait-based abstractions for async runtime operations to eliminate feature gate proliferation and enable host testing. By defining `TimeSource` and `SharedState<T>` traits, core logic becomes runtime-agnostic while maintaining zero-cost abstraction for embedded targets through Rust's monomorphization.

## Success Metrics

- [x] Feature gate count reduced to ≤60 (from \~150)
- [x] `cargo test --lib` passes without any feature flags
- [x] Control loop maintains 50Hz performance (≤20ms period)
- [x] All existing examples compile and run without modification

## Background and Current State

- Context: pico_trail uses Embassy async runtime for embedded targets (RP2350), with `#[cfg(feature)]` gates to enable host testing
- Current behavior: \~150 feature gates with many duplicate implementations for `embassy` / `not(embassy)` configurations
- Pain points:
  - Semantic confusion: `pico2_w` used for Embassy code, not just platform code
  - Code duplication: Same function implemented twice with different feature gates
  - Testing friction: Host tests require stub implementations
- Constraints: Must maintain backward compatibility with existing examples
- Related ADRs: [ADR-00027-trait-based-async-abstraction](../../adr/ADR-00027-trait-based-async-abstraction.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    Application Layer                             │
│  (examples/, rover/, communication/mavlink/handlers/)           │
│                           │                                      │
│                           ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              Core Traits (no feature gates)              │    │
│  │  ┌──────────────┐  ┌─────────────────────────────────┐  │    │
│  │  │ TimeSource   │  │ SharedState<T>                  │  │    │
│  │  │ + now_ms()   │  │ + with(f: Fn(&T) -> R)         │  │    │
│  │  │ + now_us()   │  │ + with_mut(f: Fn(&mut T) -> R) │  │    │
│  │  └──────────────┘  └─────────────────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────┘    │
│                           │                                      │
│          ┌────────────────┴────────────────┐                    │
│          ▼                                 ▼                    │
│  ┌──────────────────────┐    ┌──────────────────────────────┐  │
│  │ Embassy Impl         │    │ Mock Impl                     │  │
│  │ #[cfg(feature =      │    │ (always available)            │  │
│  │   "embassy")]        │    │                               │  │
│  │                      │    │ MockTime, MockState<T>        │  │
│  │ EmbassyTime,         │    │                               │  │
│  │ EmbassyState<T>      │    │                               │  │
│  └──────────────────────┘    └──────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Components

#### `src/core/traits/mod.rs`

Root module for core traits:

```rust
pub mod time;
pub mod sync;

pub use time::{TimeSource, MockTime};
pub use sync::{SharedState, MockState};

#[cfg(feature = "embassy")]
pub use time::EmbassyTime;

#[cfg(feature = "embassy")]
pub use sync::EmbassyState;
```

#### `src/core/traits/time.rs`

Time abstraction trait:

```rust
/// Platform-agnostic time source for control loops and timing
pub trait TimeSource: Clone + Send + Sync {
    /// Returns current time in milliseconds
    fn now_ms(&self) -> u64;

    /// Returns current time in microseconds
    fn now_us(&self) -> u64;

    /// Returns elapsed time since reference point
    fn elapsed_since(&self, reference_us: u64) -> u64 {
        self.now_us().saturating_sub(reference_us)
    }
}

// Embassy implementation
#[cfg(feature = "embassy")]
#[derive(Clone, Copy, Default)]
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
#[derive(Clone, Default)]
pub struct MockTime {
    current_us: core::cell::Cell<u64>,
}

impl MockTime {
    pub fn new() -> Self {
        Self { current_us: core::cell::Cell::new(0) }
    }

    pub fn set(&self, us: u64) {
        self.current_us.set(us);
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

#### `src/core/traits/sync.rs`

Synchronized state abstraction:

```rust
/// Platform-agnostic synchronized state access
pub trait SharedState<T> {
    /// Access state immutably
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R;

    /// Access state mutably
    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R;
}

// Embassy implementation using critical_section Mutex
#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};

#[cfg(feature = "embassy")]
pub struct EmbassyState<T> {
    inner: Mutex<CriticalSectionRawMutex, core::cell::RefCell<T>>,
}

#[cfg(feature = "embassy")]
impl<T> EmbassyState<T> {
    pub const fn new(value: T) -> Self {
        Self {
            inner: Mutex::new(core::cell::RefCell::new(value)),
        }
    }
}

#[cfg(feature = "embassy")]
impl<T> SharedState<T> for EmbassyState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R,
    {
        self.inner.lock(|cell| f(&cell.borrow()))
    }

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R,
    {
        self.inner.lock(|cell| f(&mut cell.borrow_mut()))
    }
}

// Mock implementation using RefCell (single-threaded)
pub struct MockState<T> {
    inner: core::cell::RefCell<T>,
}

impl<T> MockState<T> {
    pub fn new(value: T) -> Self {
        Self {
            inner: core::cell::RefCell::new(value),
        }
    }
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

### Data Flow

1. **Initialization**: Application creates appropriate trait implementations based on target
2. **Injection**: Trait objects passed to modules that need time/state access
3. **Usage**: Core logic calls trait methods without knowing concrete type
4. **Monomorphization**: Compiler generates specialized code for each concrete type

### Migration Pattern

Before (feature-gated duplicates):

```rust
#[cfg(feature = "embassy")]
pub static STATE: Mutex<CriticalSectionRawMutex, RefCell<MyState>> = ...;

#[cfg(feature = "embassy")]
pub fn get_value() -> u32 {
    STATE.lock(|cell| cell.borrow().value)
}

#[cfg(not(feature = "embassy"))]
pub fn get_value() -> u32 {
    0 // stub
}
```

After (trait-based):

```rust
pub struct MyModule<S: SharedState<MyState>> {
    state: S,
}

impl<S: SharedState<MyState>> MyModule<S> {
    pub fn get_value(&self) -> u32 {
        self.state.with(|s| s.value)
    }
}

// Global instances for backward compatibility
#[cfg(feature = "embassy")]
pub static MODULE: MyModule<EmbassyState<MyState>> = MyModule {
    state: EmbassyState::new(MyState::default()),
};
```

### Error Handling

- Trait methods are infallible (no `Result` return types)
- Panic on borrow conflicts in MockState (test-only, indicates bug)
- Embassy implementation uses critical sections to prevent races

### Performance Considerations

- **Zero-cost abstraction**: Generics with trait bounds compile to direct calls via monomorphization
- **No dynamic dispatch**: Avoid `dyn Trait` in hot paths; use generics
- **Inline hints**: Mark trait methods with `#[inline]` for cross-crate optimization
- **Benchmark gates**: Measure control loop timing before/after migration

### Platform Considerations

#### Embedded (RP2350)

- Embassy implementations use `CriticalSectionRawMutex` for interrupt safety
- `EmbassyTime` uses hardware timer via Embassy time driver
- No heap allocation in trait implementations

#### Host (x86_64 tests)

- `MockTime` allows deterministic time control in tests
- `MockState` uses `RefCell` (single-threaded, no synchronization overhead)
- Tests can advance time and verify timing-dependent behavior

## Alternatives Considered

1. **Keep feature gates, just reclassify**
   - Pros: Lower effort
   - Cons: Still requires stub implementations, doesn't solve duplication

2. **Use `async-trait` crate for async methods**
   - Pros: Enables async trait methods
   - Cons: Adds allocation, complexity; sync traits sufficient for current needs

3. **Global type aliases instead of generics**
   - Pros: Simpler API surface
   - Cons: Less flexible, harder to test, still needs feature gates

Decision Rationale: Generics with trait bounds provide the cleanest solution with zero runtime cost and maximum flexibility for testing.

## Testing Strategy

### Unit Tests

- Test `MockTime` time advancement and elapsed calculation
- Test `MockState` with and without concurrent borrows
- Test Embassy implementations compile with feature enabled

### Integration Tests

- Migrate one module (e.g., `rc_channel`) as POC
- Verify all existing tests pass before/after
- Verify embedded build succeeds

### Performance & Benchmarks

- Measure control loop period before migration (baseline)
- Measure after each phase to detect regressions
- Target: ≤20ms average, ≤25ms max

## Documentation Impact

- Update `docs/architecture.md` with trait abstraction layer
- Update `CLAUDE.md` feature gate guidelines
- Add inline documentation for new traits

## Open Questions

- [x] Should `TimeSource` require `Clone`? → Yes, for flexibility in passing to multiple components
- [x] Should `SharedState` support async locking? → No, use sync for now; async can be added later if needed
- [x] How to handle global static instances for backward compatibility? → Use const constructors with feature gates

## Appendix

### Module Migration Priority

| Module                        | Gates | Impact | Complexity |
| ----------------------------- | ----- | ------ | ---------- |
| `mission/state.rs`            | 18    | High   | Medium     |
| `log_router.rs`               | 10    | Medium | Low        |
| `mavlink/handlers/command.rs` | 12    | Medium | High       |
| `rc_channel/mod.rs`           | 3     | Low    | Low        |
| `navigation/mod.rs`           | 6     | Medium | Medium     |

### Feature Gate Reclassification

| Current                    | Should Be          | Files                |
| -------------------------- | ------------------ | -------------------- |
| `pico2_w` (async code)     | `embassy`          | \~50 locations       |
| `pico2_w` (executor tasks) | `embassy-executor` | \~10 locations       |
| `pico2_w` (HAL code)       | `pico2_w` (keep)   | `src/platform/` only |
