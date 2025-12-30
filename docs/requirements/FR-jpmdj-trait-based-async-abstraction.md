# FR-jpmdj Trait-Based Async Abstraction

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [NFR-nmmu0-platform-code-isolation](../requirements/NFR-nmmu0-platform-code-isolation.md)
- Dependent Requirements:
  - [NFR-wl974-feature-gate-reduction](../requirements/NFR-wl974-feature-gate-reduction.md)
- Related ADRs:
  - [ADR-3ciu6-trait-based-async-abstraction](../adr/ADR-3ciu6-trait-based-async-abstraction.md)
- Related Tasks:
  - [T-d9rim-trait-based-async-abstraction](../tasks/T-d9rim-trait-based-async-abstraction/README.md)

## Requirement Statement

The system shall provide trait-based abstractions for async runtime operations (time, synchronization, I/O) to decouple core logic from specific async runtime implementations (Embassy).

## Rationale

Currently, async runtime code is scattered throughout the codebase with feature gates (`#[cfg(feature = "embassy")]` and `#[cfg(feature = "pico2_w")]`), creating:

1. **Code duplication**: Many functions have two implementations (embassy + stub)
2. **Semantic confusion**: Platform features used for non-platform-specific async code
3. **Testing friction**: Host tests require complex stub implementations
4. **Portability barriers**: Adding new async runtimes requires touching many files

Trait-based abstraction allows core logic to remain runtime-agnostic while enabling different implementations for embedded targets and host tests.

## User Story (if applicable)

As a maintainer, I want async operations abstracted behind traits, so that I can test core logic without embedded dependencies and support future async runtimes without modifying business logic.

## Acceptance Criteria

- [ ] Time abstraction trait (`TimeSource`) defined with `now_ms()` and `now_us()` methods
- [ ] Mutex abstraction trait (`SharedState<T>`) defined for thread-safe state access
- [ ] Embassy implementations provided for embedded targets
- [ ] Mock implementations provided for host tests (no feature gates required)
- [ ] Core modules (`mission/state.rs`, `rc_channel/mod.rs`, `navigation/mod.rs`) refactored to use traits
- [ ] `cargo test --lib` passes without any embassy features enabled
- [ ] Existing embedded functionality preserved (no behavioral changes)

## Technical Details

### Functional Requirement Details

#### Trait Definitions

```rust
// src/core/traits/time.rs
pub trait TimeSource {
    fn now_ms(&self) -> u64;
    fn now_us(&self) -> u64;
}

// src/core/traits/sync.rs
pub trait SharedState<T> {
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R;

    fn with_mut<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut T) -> R;
}

// Optional: Async variant
pub trait AsyncSharedState<T> {
    async fn with_async<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&T) -> R;
}
```

#### Implementation Strategy

1. **Embassy Implementation** (feature-gated):

   ```rust
   #[cfg(feature = "embassy")]
   pub struct EmbassyTime;

   #[cfg(feature = "embassy")]
   impl TimeSource for EmbassyTime {
       fn now_ms(&self) -> u64 {
           embassy_time::Instant::now().as_millis()
       }
   }
   ```

2. **Mock Implementation** (always available):

   ```rust
   pub struct MockTime {
       current_ms: core::cell::Cell<u64>,
   }

   impl TimeSource for MockTime {
       fn now_ms(&self) -> u64 {
           self.current_ms.get()
       }
   }
   ```

3. **Dependency Injection**: Core structs accept trait objects or generics:
   ```rust
   pub struct MissionState<T: TimeSource> {
       time: T,
       // ...
   }
   ```

## Platform Considerations

### Embedded (RP2350, future ESP32)

- Embassy implementations use platform-specific timing and sync primitives
- Feature gates only in implementation files, not in core logic

### Host (x86_64 tests)

- Mock implementations enable testing without embedded features
- Test code can control time advancement for deterministic behavior

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                                   | Validation                               |
| --------------------------------------- | ------ | ---------- | ------------------------------------------------------------ | ---------------------------------------- |
| Performance overhead from trait objects | Medium | Low        | Use generics with monomorphization, benchmark critical paths | Measure control loop timing before/after |
| API surface changes break examples      | High   | Medium     | Maintain backward-compatible wrappers during transition      | All examples compile and run             |
| Incomplete abstraction coverage         | Medium | Medium     | Start with highest-impact modules (mission, rc_channel)      | Track feature gate count reduction       |

## Implementation Notes

- Start with `TimeSource` trait as POC (simplest, most widely used)
- Prioritize modules with highest feature gate count: `mission/state.rs` (18 gates), `log_router.rs` (10 gates)
- Consider using `core::cell::RefCell` for mock implementations to enable interior mutability
- Async traits may require `async-trait` crate or Rust 1.75+ native async traits

## External References

- [Embassy Async Framework](https://embassy.dev/) - Target async runtime
- [Rust Async Book](https://rust-lang.github.io/async-book/) - Async Rust patterns
