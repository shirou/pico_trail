# FR-00132 Trait Abstractions for Platform Services

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00130-core-crate-nostd-purity](../requirements/FR-00130-core-crate-nostd-purity.md)
- Dependent Requirements:
  - [NFR-00089-zero-cfg-core-crate](../requirements/NFR-00089-zero-cfg-core-crate.md)
- Related Analyses:
  - [AN-00039-crate-workspace-separation](../analysis/AN-00039-crate-workspace-separation.md)
- Related Tasks:
  - [T-00035-workspace-separation](../tasks/T-00035-workspace-separation/README.md)

## Requirement Statement

The core crate shall define trait abstractions for all platform services (time, synchronization, I/O) that enable business logic to operate without direct dependencies on Embassy or HAL implementations.

## Rationale

Current business logic directly uses Embassy types (`embassy_time::Instant`, `Mutex<CriticalSectionRawMutex, T>`), creating tight coupling between algorithms and the async runtime. This prevents:

1. Host testing without Embassy feature flags
2. Future async runtime migration
3. Clean separation of concerns

Trait-based abstraction allows:

- Core logic to remain runtime-agnostic
- Different implementations for embedded targets and host tests
- Inversion of control: firmware drives core, not vice versa

## User Story (if applicable)

As a maintainer, I want platform services abstracted behind traits, so that core algorithms can be tested with mock implementations and ported to different async runtimes without modification.

## Acceptance Criteria

- [ ] `TimeSource` trait defined in `crates/core/src/traits/time.rs`
- [ ] `Tick` trait defined for tick-based state machine updates
- [ ] Mock implementations provided in core crate (no feature gates)
- [ ] Firmware crate provides Embassy implementations
- [ ] Core modules use traits instead of concrete Embassy types
- [ ] `cargo test -p pico_trail_core` passes using mock implementations

## Technical Details

### Functional Requirement Details

#### Trait Definitions

```rust
// crates/core/src/traits/time.rs

/// Time source abstraction for core logic
pub trait TimeSource {
    /// Returns current time in milliseconds since boot
    fn now_ms(&self) -> u64;

    /// Returns current time in microseconds since boot
    fn now_us(&self) -> u64;
}

/// Mock implementation for host tests
pub struct MockTime {
    current_ms: core::cell::Cell<u64>,
}

impl MockTime {
    pub fn new() -> Self {
        Self { current_ms: core::cell::Cell::new(0) }
    }

    pub fn advance(&self, ms: u64) {
        self.current_ms.set(self.current_ms.get() + ms);
    }
}

impl TimeSource for MockTime {
    fn now_ms(&self) -> u64 { self.current_ms.get() }
    fn now_us(&self) -> u64 { self.current_ms.get() * 1000 }
}
```

```rust
// crates/core/src/traits/tick.rs

/// Tick-based update trait for state machines
pub trait Tick {
    /// Input type for each tick
    type Input;
    /// Output type for each tick
    type Output;

    /// Process one tick with given delta time
    fn tick(&mut self, dt_ms: u32, input: &Self::Input) -> Self::Output;
}
```

#### Usage Pattern

```rust
// crates/core/src/control/controller.rs
use crate::traits::{Tick, TimeSource};

pub struct Controller<T: TimeSource> {
    time: T,
    state: ControlState,
}

impl<T: TimeSource> Controller<T> {
    pub fn new(time: T) -> Self {
        Self { time, state: ControlState::default() }
    }
}

impl<T: TimeSource> Tick for Controller<T> {
    type Input = SensorInput;
    type Output = ControlOutput;

    fn tick(&mut self, dt_ms: u32, input: &Self::Input) -> Self::Output {
        // Pure logic, no async, no Embassy
        self.state.update(dt_ms, input)
    }
}
```

#### Firmware Integration

```rust
// crates/firmware/src/tasks/control.rs
use pico_trail_core::{Controller, Tick};
use embassy_time::Timer;

pub struct EmbassyTime;

impl pico_trail_core::traits::TimeSource for EmbassyTime {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }
    fn now_us(&self) -> u64 {
        embassy_time::Instant::now().as_micros()
    }
}

#[embassy_executor::task]
async fn control_task(mut ctrl: Controller<EmbassyTime>) {
    loop {
        let input = read_sensors().await;
        let output = ctrl.tick(10, &input);
        apply_output(output).await;
        Timer::after_millis(10).await;
    }
}
```

## Platform Considerations

### Embedded (RP2350)

- Embassy implementations in firmware crate
- Zero-cost abstraction via monomorphization

### Host (x86_64 tests)

- Mock implementations in core crate
- Tests can control time advancement for deterministic behavior

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                                   | Validation                    |
| ---------------------------------- | ------ | ---------- | -------------------------------------------- | ----------------------------- |
| Performance overhead from generics | Low    | Low        | Monomorphization eliminates virtual dispatch | Benchmark control loop timing |
| Incomplete trait coverage          | Medium | Medium     | Inventory all Embassy usages first           | Checklist of required traits  |
| Complex type signatures            | Medium | Medium     | Use type aliases for common combinations     | Code review for readability   |

## Implementation Notes

- Start with `TimeSource` trait (most widely used abstraction)
- Avoid async traits in core - use tick-based patterns instead
- Consider providing a `Platform` trait that bundles all services
- Use generics with monomorphization, not trait objects

## External References

- [Rust Traits](https://doc.rust-lang.org/book/ch10-02-traits.html) - Trait fundamentals
- [Embassy Time](https://docs.embassy.dev/embassy-time/) - Embassy time primitives
