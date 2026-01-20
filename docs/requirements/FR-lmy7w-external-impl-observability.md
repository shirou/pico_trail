# FR-lmy7w External Impl Blocks for Observability

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-5f7tx-core-crate-nostd-purity](../requirements/FR-5f7tx-core-crate-nostd-purity.md)
- Dependent Requirements:
  - [NFR-3y83q-zero-cfg-core-crate](../requirements/NFR-3y83q-zero-cfg-core-crate.md)
- Related Analyses:
  - [AN-q7k2m-crate-workspace-separation](../analysis/AN-q7k2m-crate-workspace-separation.md)
- Related Tasks:
  - [T-3n2ej-workspace-separation](../tasks/T-3n2ej-workspace-separation/README.md)

## Requirement Statement

The system shall implement observability traits (`defmt::Format`, `Debug`, `Display`) for core types via external impl blocks in the firmware crate, not via derive macros or cfg_attr in the core crate.

## Rationale

Current core types embed observability concerns:

```rust
// PROBLEM: Observability embedded in type definition
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug)]
pub struct NavigationState { ... }
```

This pattern:

1. Couples type definition to logging implementation
2. Requires `cfg_attr` which we aim to eliminate from core
3. Forces defmt dependency awareness in core crate

External impl blocks separate concerns:

- Core crate defines pure data types
- Firmware crate adds observability for embedded targets
- Host tests add Debug/Display for test output

## User Story (if applicable)

As a developer, I want core types to be pure data structures without logging concerns, so that observability implementation can vary by target without modifying core code.

## Acceptance Criteria

- [ ] No `#[cfg_attr(feature = "defmt", derive(defmt::Format))]` in `crates/core/`
- [ ] No `use defmt` or `defmt::` in `crates/core/`
- [ ] Core types export all necessary public fields for external formatting
- [ ] Firmware crate provides `defmt::Format` impls for core types
- [ ] Host tests use `Debug` impls (can be derived in core)
- [ ] All types currently using defmt have equivalent firmware impls

## Technical Details

### Functional Requirement Details

#### Before (Current Pattern)

```rust
// src/navigation/state.rs
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct NavigationState {
    pub position: Position,
    pub heading: f32,
    pub speed: f32,
}
```

#### After (External Impl Pattern)

```rust
// crates/core/src/navigation/state.rs
#[derive(Debug, Clone)]  // Debug is allowed (std/core trait)
pub struct NavigationState {
    pub position: Position,
    pub heading: f32,
    pub speed: f32,
}

// crates/firmware/src/formatters/navigation.rs
use pico_trail_core::navigation::NavigationState;

impl defmt::Format for NavigationState {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "NavState {{ pos: {}, hdg: {}, spd: {} }}",
            self.position, self.heading, self.speed)
    }
}
```

#### Type Visibility Requirements

For external impl blocks to work, core types must:

1. Be `pub` (public visibility)
2. Have `pub` fields, OR
3. Provide accessor methods for all formatted data

```rust
// Option 1: Public fields (preferred for data types)
pub struct Position {
    pub lat: f64,
    pub lon: f64,
}

// Option 2: Accessor methods (for encapsulated types)
pub struct SensorReading {
    value: f32,
    timestamp: u64,
}

impl SensorReading {
    pub fn value(&self) -> f32 { self.value }
    pub fn timestamp(&self) -> u64 { self.timestamp }
}
```

#### Firmware Formatter Module Structure

```
crates/firmware/src/
├── formatters/
│   ├── mod.rs           # Re-exports all Format impls
│   ├── navigation.rs    # NavigationState, Position, etc.
│   ├── arming.rs        # ArmingState, ArmingError, etc.
│   ├── mission.rs       # MissionState, Waypoint, etc.
│   └── sensor.rs        # SensorData, ImuReading, etc.
```

#### Derive Allowances in Core

| Derive             | Allowed in Core | Notes                   |
| ------------------ | --------------- | ----------------------- |
| `Debug`            | Yes             | Standard library trait  |
| `Clone`, `Copy`    | Yes             | Standard library traits |
| `PartialEq`, `Eq`  | Yes             | Standard library traits |
| `Default`          | Yes             | Standard library trait  |
| `defmt::Format`    | NO              | External impl only      |
| `serde::Serialize` | NO              | External impl only      |

## Platform Considerations

### Embedded (RP2350)

- `defmt::Format` impls in firmware crate
- Used for RTT logging via probe-rs

### Host (x86_64 tests)

- `Debug` trait from core provides test output
- Can add `Display` impls if human-readable output needed

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                                     | Validation          |
| ------------------------------- | ------ | ---------- | ---------------------------------------------- | ------------------- |
| Missing Format impl for type    | Medium | Medium     | Generate impl list from current cfg_attr usage | Compile-time check  |
| Private fields block formatting | Low    | Low        | Audit types for field visibility               | Code review         |
| Verbose impl blocks             | Low    | Medium     | Consider macro for common patterns             | Code clarity review |

## Implementation Notes

- Inventory all current `#[cfg_attr(feature = "defmt", ...)]` usages first
- Create formatters module in firmware before removing from core
- Consider a macro `impl_defmt_format!` for repetitive patterns
- Nested types require Format impls for all components

## External References

- [defmt User Guide](https://defmt.ferrous-systems.com/) - defmt documentation
- [Rust Orphan Rule](https://doc.rust-lang.org/book/ch10-02-traits.html#implementing-a-trait-on-a-type) - Trait impl restrictions
