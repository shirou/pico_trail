# NFR-e82vp STATUSTEXT API no_std Compatibility

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-onj2m-statustext-public-api](../requirements/FR-onj2m-statustext-public-api.md)
  - [NFR-ssp9q-statustext-performance-memory](../requirements/NFR-ssp9q-statustext-performance-memory.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-eiuvv-statustext-implementation](../tasks/T-eiuvv-statustext-implementation/README.md)

## Requirement Statement

The STATUSTEXT notification API shall be usable in no_std embedded environments without requiring the Rust standard library, depending only on core and embedded-hal libraries to ensure compatibility with bare-metal targets like RP2350.

## Rationale

Embedded targets (RP2350 Pico 2 W) do not provide the Rust standard library (std). Code requiring std cannot run on bare-metal. The notification API must work in this constrained environment using only core (always available) and carefully selected no_std crates (heapless, embedded-hal). This ensures the notification system is usable throughout the codebase, including in scheduler, interrupt handlers, and device drivers.

## User Story

The system shall **implement the STATUSTEXT notification API using only no_std compatible dependencies** to ensure **it works on bare-metal embedded targets without requiring the Rust standard library**.

## Acceptance Criteria

- [ ] Notification module builds with `#![no_std]` attribute
- [ ] Zero dependencies on `std::*` types (String, Vec, HashMap, etc.)
- [ ] Uses only `core::*` and `embedded-hal` compatible types
- [ ] Builds successfully for `thumbv8m.main-none-eabihf` target (RP2350)
- [ ] No compilation errors with `--no-default-features` flag
- [ ] Compatible with `embassy` async runtime (no_std)
- [ ] Works in interrupt context (no OS dependencies)
- [ ] `cargo clippy` reports no std dependencies
- [ ] Binary links successfully without newlib or other C standard library

## Technical Details

### Non-Functional Requirement Details

**Portability:**

- Target: Bare-metal embedded (RP2350, ARM Cortex-M33)
- Runtime: No OS, embassy async executor
- Memory: Static and stack only (no heap, see NFR-ssp9q)

**Compatibility:**

- Allowed dependencies:
  - `core::*` - Always available in no_std
  - `heapless::*` - no_std collections
  - `embedded-hal` - Hardware abstraction
  - `embassy-*` - Async runtime for embedded
  - `mavlink` (with appropriate features)
  - `defmt` - Embedded logging

- Disallowed dependencies:
  - `std::*` - Standard library (not available)
  - `alloc::*` - Heap allocation (conflicts with NFR-ssp9q)
  - OS-specific types (File, Thread, etc.)

**Module Structure:**

```rust
#![no_std]

use core::fmt::Write;
use heapless::{Deque, String};
use mavlink::common::{MavSeverity, STATUSTEXT_DATA};

pub struct StatusNotifier {
    queue: Deque<QueuedMessage, 16>,
    drops: u32,
}

// No std dependencies anywhere
```

**Verification:**

```bash
# Verify no_std build
cargo build --target thumbv8m.main-none-eabihf --lib

# Check for std symbols (should be empty)
arm-none-eabi-nm target/thumbv8m.main-none-eabihf/debug/libpico_trail.a | grep -i "std::"
```

## Platform Considerations

### Embedded (RP2350)

- Critical requirement for RP2350 bare-metal
- Must work with embassy async executor
- Must work in interrupt handlers
- No OS, no threads, no heap

### Host Tests

- Host tests can use std for test infrastructure
- Notification implementation itself still no_std
- Conditional compilation for test utilities:

  ```rust
  #[cfg(test)]
  use std::vec::Vec;  // OK in tests

  #[cfg(not(test))]
  // Production code must be no_std
  ```

### Cross-Platform

- Notification core logic identical (no_std)
- Platform-specific integration uses conditional compilation
- Test doubles for std environments

## Risks & Mitigation

| Risk                                            | Impact | Likelihood | Mitigation                                          | Validation                            |
| ----------------------------------------------- | ------ | ---------- | --------------------------------------------------- | ------------------------------------- |
| Developer accidentally adds std dependency      | High   | Medium     | CI builds with `--target thumbv8m.main-none-eabihf` | Automated build checks, code review   |
| Third-party crate requires std                  | High   | Low        | Audit dependencies, choose no_std alternatives      | `cargo tree` analysis, feature flags  |
| String formatting requires std (format! macro)  | Medium | Medium     | Use `write!()` with heapless::String                | Replace format! with core::fmt::Write |
| Error handling requires std::error::Error trait | Low    | Low        | Use simple error enums without std trait            | Custom error types with no_std        |
| Time/timing requires std::time                  | Medium | Low        | Use embassy_time for embedded timing                | Use embassy_time::Instant instead     |

## Implementation Notes

**Preferred Patterns:**

- Use `write!()` macro with `heapless::String` for formatting:

  ```rust
  use core::fmt::Write;
  use heapless::String;

  let mut text = String::<200>::new();
  write!(text, "PreArm: Battery voltage {} below {}", 9.8, 10.5).unwrap();
  send_error(&text);
  ```

- Use `core::fmt::Display` for type formatting

- Use `defmt` for debug logging (not println!)

**Known Pitfalls:**

- Do not use `format!()` macro (requires std/alloc)
- Do not use `std::sync::Mutex` (use critical-section or embassy-sync)
- Do not use `println!()` (use defmt or project logging macros)
- Do not use `.to_string()` (allocates via std)

**Migration Pattern:**

```rust
// BAD (uses std)
let msg = format!("Error: value {}", x);
send_error(&msg);

// GOOD (no_std)
use core::fmt::Write;
let mut msg = heapless::String::<200>::new();
write!(msg, "Error: value {}", x).unwrap();
send_error(&msg);
```

**Related Code:**

- Project logging: `src/core/logging.rs` (already no_std compatible)
- Scheduler: `src/core/scheduler.rs` (no_std, embassy-based)

## External References

- [Embedded Rust Book - no_std](https://docs.rust-embedded.org/book/intro/no-std.html)
- [heapless crate documentation](https://docs.rs/heapless/)
- [embedded-hal documentation](https://docs.rs/embedded-hal/)
- [Embassy documentation](https://embassy.dev/)
