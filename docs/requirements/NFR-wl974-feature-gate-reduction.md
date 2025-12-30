# NFR-wl974 Feature Gate Reduction

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-jpmdj-trait-based-async-abstraction](../requirements/FR-jpmdj-trait-based-async-abstraction.md)
  - [NFR-nmmu0-platform-code-isolation](../requirements/NFR-nmmu0-platform-code-isolation.md)
- Related ADRs:
  - [ADR-3ciu6-trait-based-async-abstraction](../adr/ADR-3ciu6-trait-based-async-abstraction.md)
- Related Tasks:
  - [T-d9rim-trait-based-async-abstraction](../tasks/T-d9rim-trait-based-async-abstraction/README.md)

## Requirement Statement

The codebase shall contain no more than 60 feature gates (`#[cfg(feature = "...")]`), reduced from the current \~150, while maintaining full functionality for all supported platforms.

## Rationale

Excessive feature gates create:

1. **Cognitive load**: Developers must track which code paths apply to which configurations
2. **Maintenance burden**: Changes often require updating multiple conditional blocks
3. **Testing complexity**: Each feature combination requires separate validation
4. **Code duplication**: Parallel implementations for different features

Reducing feature gates improves code clarity, simplifies maintenance, and makes the architecture more understandable for contributors.

## User Story (if applicable)

The system shall minimize conditional compilation directives to ensure maintainability and reduce cognitive overhead for developers working across different platform configurations.

## Acceptance Criteria

- [ ] Total feature gate count ≤ 60 (measured by `grep -r "#\[cfg(feature" src/ | wc -l`)
- [ ] `pico2_w` feature gates used only in `src/platform/rp2350/` directory
- [ ] `embassy` feature gates used only for async runtime integration (not platform-specific code)
- [ ] No duplicate function implementations (removed `#[cfg(feature)]` + `#[cfg(not(feature))]` pairs)
- [ ] Host tests (`cargo test --lib`) pass without feature flags
- [ ] Embedded builds (`./scripts/build-rp2350.sh`) succeed without regression
- [ ] CI script added to enforce feature gate count limit

## Technical Details

### Non-Functional Requirement Details

#### Current State (Baseline)

| Category                  | Count      | Target              |
| ------------------------- | ---------- | ------------------- |
| Total feature gates       | \~150      | ≤60                 |
| `pico2_w` gates           | \~80       | ≤25 (platform only) |
| `embassy` gates           | \~50       | ≤30 (async runtime) |
| Duplicate implementations | \~15 pairs | 0                   |

#### Measurement Method

```bash
# Total feature gates
grep -r "#\[cfg(feature" src/ | wc -l

# By feature
grep -r '#\[cfg(feature = "pico2_w"' src/ | wc -l
grep -r '#\[cfg(feature = "embassy"' src/ | wc -l

# Platform isolation check
grep -r '#\[cfg(feature = "pico2_w"' src/ | grep -v "src/platform/" | wc -l
# Target: 0
```

#### Feature Gate Classification

| Feature            | Allowed Locations           | Purpose                     |
| ------------------ | --------------------------- | --------------------------- |
| `pico2_w`          | `src/platform/rp2350/` only | RP2350 HAL implementations  |
| `embassy`          | Anywhere (minimized)        | Async runtime integration   |
| `embassy-executor` | `src/platform/` only        | Executor task definitions   |
| `gps-ublox`        | `src/devices/gps/`          | Optional GPS vendor support |

- Performance: No measurable impact on compile time or runtime performance
- Reliability: All existing tests must continue passing
- Compatibility: All supported platforms (RP2350, future ESP32) must build correctly

## Platform Considerations

### Embedded Platforms

- Platform-specific code confined to `src/platform/<platform>/`
- Async runtime code uses `embassy` feature (not platform features)
- Device drivers use traits, implementations in platform directories

### Host Testing

- Core logic testable without any features enabled
- Mock implementations provided without feature gates
- CI runs `cargo test --lib` without feature flags

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                                      | Validation                        |
| ---------------------------------- | ------ | ---------- | ----------------------------------------------- | --------------------------------- |
| Breaking embedded builds           | High   | Medium     | Run embedded build after each refactoring phase | `./scripts/build-rp2350.sh` in CI |
| Missing platform-specific behavior | High   | Low        | Comprehensive test coverage before refactoring  | All tests pass before/after       |
| Regression in feature gate count   | Medium | Medium     | CI enforcement script                           | Automated count check in PR       |

## Implementation Notes

- Refactoring should be done in phases, one module at a time
- Prioritize modules with highest feature gate count first
- Use `#[cfg(test)]` for test-only code instead of `#[cfg(not(feature = "embassy"))]`
- Consider extracting platform-independent submodules from mixed modules

### CI Enforcement Script

```bash
#!/bin/bash
# scripts/check-feature-gates.sh
MAX_GATES=60
ACTUAL=$(grep -r "#\[cfg(feature" src/ | wc -l)

if [ "$ACTUAL" -gt "$MAX_GATES" ]; then
    echo "Feature gate count ($ACTUAL) exceeds limit ($MAX_GATES)"
    exit 1
fi

# Check platform isolation
LEAKED=$(grep -r '#\[cfg(feature = "pico2_w"' src/ | grep -v "src/platform/" | wc -l)
if [ "$LEAKED" -gt 0 ]; then
    echo "pico2_w feature gates found outside src/platform/:"
    grep -r '#\[cfg(feature = "pico2_w"' src/ | grep -v "src/platform/"
    exit 1
fi

echo "Feature gate check passed: $ACTUAL gates"
```

## External References

- [Rust Conditional Compilation](https://doc.rust-lang.org/reference/conditional-compilation.html) - Rust reference
- [Cargo Features](https://doc.rust-lang.org/cargo/reference/features.html) - Feature flag best practices
