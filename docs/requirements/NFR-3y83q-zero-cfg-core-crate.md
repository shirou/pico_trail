# NFR-3y83q Zero CFG in Core Crate

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-5f7tx-core-crate-nostd-purity](../requirements/FR-5f7tx-core-crate-nostd-purity.md)
  - [FR-mna5g-trait-abstractions-platform-services](../requirements/FR-mna5g-trait-abstractions-platform-services.md)
  - [FR-lmy7w-external-impl-observability](../requirements/FR-lmy7w-external-impl-observability.md)
- Dependent Requirements:
  - [NFR-11puw-ci-core-purity-lint](../requirements/NFR-11puw-ci-core-purity-lint.md)
- Related Analyses:
  - [AN-q7k2m-crate-workspace-separation](../analysis/AN-q7k2m-crate-workspace-separation.md)
- Related Tasks:
  - [T-3n2ej-workspace-separation](../tasks/T-3n2ej-workspace-separation/README.md)
  - [T-nazbq-firmware-to-core-migration](../tasks/T-nazbq-firmware-to-core-migration/README.md)

## Requirement Statement

The `crates/core` directory shall contain zero instances of `#[cfg(feature = ...)]` or `#[cfg_attr(feature = ...)]` conditional compilation directives.

## Rationale

Conditional compilation in core crate indicates leaked platform concerns:

1. **Design smell**: cfg in core means business logic depends on build configuration
2. **Testing complexity**: Multiple code paths require multiple test configurations
3. **Cognitive overhead**: Developers must mentally track which code applies when
4. **Maintenance burden**: Changes may require updating multiple conditional branches

The workspace split exists specifically to eliminate cfg from business logic. If cfg appears in core, the architectural boundary has been violated.

## User Story (if applicable)

The system shall maintain zero conditional compilation directives in the core crate to ensure all business logic is platform-independent and uniformly testable.

## Acceptance Criteria

- [ ] `grep -r "#\[cfg(" crates/core/src/ | wc -l` returns 0
- [ ] `grep -r "#\[cfg_attr(" crates/core/src/ | wc -l` returns 0
- [ ] Core crate compiles identically regardless of feature flags
- [ ] No `#[cfg(test)]` workarounds that duplicate production logic
- [ ] CI script enforces zero cfg count

## Technical Details

### Non-Functional Requirement Details

#### Forbidden Patterns

```rust
// ALL of these are FORBIDDEN in crates/core/:

#[cfg(feature = "embassy")]
use embassy_time::Instant;

#[cfg(feature = "pico2_w")]
pub fn platform_specific() { }

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MyType { }

#[cfg(not(feature = "embassy"))]
fn stub_implementation() { }
```

#### Allowed Patterns

```rust
// These are ALLOWED in crates/core/:

#[cfg(test)]
mod tests {
    // Test-only code is allowed
}

#[derive(Debug, Clone)]  // Standard derives
pub struct MyType { }

pub trait TimeSource {  // Trait abstractions
    fn now_ms(&self) -> u64;
}
```

#### Measurement Method

```bash
# Must return 0
grep -rn "#\[cfg(feature" crates/core/src/ | wc -l

# Must return 0
grep -rn "#\[cfg_attr(feature" crates/core/src/ | wc -l

# cfg(test) is allowed - excluded from count
grep -rn "#\[cfg(" crates/core/src/ | grep -v "cfg(test)" | wc -l
```

#### Where CFG Belongs

| Location                        | CFG Allowed | Purpose              |
| ------------------------------- | ----------- | -------------------- |
| `crates/core/`                  | NO          | Pure business logic  |
| `crates/firmware/`              | YES         | Platform integration |
| `crates/firmware/src/platform/` | YES         | HAL bindings         |
| `examples/`                     | YES         | Demo applications    |

- Maintainability: Zero cfg simplifies code understanding and modification
- Testability: Single code path tested on all configurations
- Reliability: No risk of cfg-related logic bugs

## Platform Considerations

N/A - This requirement is platform-agnostic by definition.

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                        | Validation                     |
| ------------------------------- | ------ | ---------- | --------------------------------- | ------------------------------ |
| Legitimate need for cfg in core | Medium | Low        | Design review to find alternative | Architecture review            |
| cfg(test) confusion             | Low    | Medium     | Document allowed patterns clearly | Lint script excludes cfg(test) |
| Regression during development   | Medium | Medium     | CI enforcement on every PR        | Automated check                |

## Implementation Notes

- Run cfg check before each migration step
- If cfg seems necessary in core, redesign with traits instead
- The presence of cfg in core is an architectural red flag
- Consider pre-commit hook for immediate feedback

### Alternative Patterns

Instead of cfg, use these patterns:

| Instead of...                                 | Use...                          |
| --------------------------------------------- | ------------------------------- |
| `#[cfg(feature = "embassy")] async fn`        | Trait with tick-based sync API  |
| `#[cfg_attr(feature = "defmt", derive(...))]` | External impl block in firmware |
| `#[cfg(feature = "pico2_w")] use hal::*`      | Platform trait in firmware      |
| `#[cfg(not(...))] fn stub()`                  | Trait with mock impl            |

## External References

- [Rust Conditional Compilation](https://doc.rust-lang.org/reference/conditional-compilation.html) - cfg reference
