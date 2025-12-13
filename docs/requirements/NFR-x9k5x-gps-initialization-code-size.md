# NFR-x9k5x GPS Initialization Code Size Minimization

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Source Analysis:
  - [AN-vzkte-gps-initialization-abstraction](../analysis/AN-vzkte-gps-initialization-abstraction.md)
- Prerequisite Requirements:
  - [FR-pcr03-gps-initialization-separation](../requirements/FR-pcr03-gps-initialization-separation.md)
  - [FR-xpfte-gps-vendor-extensibility](../requirements/FR-xpfte-gps-vendor-extensibility.md)
- Dependent Requirements: (none)
- Related ADRs:
  - [ADR-27cz1-gps-initialization-module-structure](../adr/ADR-27cz1-gps-initialization-module-structure.md)
- Related Tasks:
  - [T-djbdm-gps-initialization-module](../tasks/T-djbdm-gps-initialization-module/README.md)

## Requirement Statement

The GPS initialization system shall use compile-time feature flags to exclude unused vendor initialization code from the final binary.

## Rationale

Embedded targets (RP2040, RP2350) have limited flash memory. Including initialization code for all GPS vendors when only one is used wastes valuable flash space. Feature flags enable dead code elimination at compile time, ensuring only the required vendor code is included in the binary.

## User Story (if applicable)

The system shall exclude unused GPS vendor initialization code at compile time to ensure minimal flash memory usage on embedded targets.

## Acceptance Criteria

- [ ] Each GPS vendor initialization module is gated by its own feature flag
- [ ] Building without a vendor feature excludes that vendor's code from the binary
- [ ] Default feature configuration includes only `gps-ublox` (most common)
- [ ] Binary size with single vendor is smaller than binary with all vendors
- [ ] Feature flags follow naming convention: `gps-<vendor>` (e.g., `gps-ublox`, `gps-mtk`)

## Technical Details (if applicable)

### Non-Functional Requirement Details

- Performance: N/A (compile-time optimization)
- Security: N/A
- Reliability: N/A
- Compatibility: All embedded targets (RP2040, RP2350)
- Usability: Developers select vendor via `--features gps-<vendor>` flag

**Feature Flag Configuration:**

```toml
# Cargo.toml
[features]
default = ["gps-ublox"]  # u-blox enabled by default

gps-ublox = []           # u-blox UBX protocol
gps-mtk = []             # MediaTek PMTK protocol (future)
gps-sirf = []            # SiRF binary protocol (future)
gps-generic = []         # No initialization (module works with defaults)
```

**Conditional Compilation:**

```rust
// src/devices/gps/init/mod.rs
#[cfg(feature = "gps-ublox")]
pub mod ublox;

#[cfg(feature = "gps-mtk")]
pub mod mtk;
```

**Build Command Examples:**

```bash
# Default (u-blox only)
cargo build --release

# MTK only
cargo build --release --no-default-features --features gps-mtk

# Multiple vendors (development/testing)
cargo build --release --features gps-ublox,gps-mtk
```

## Platform Considerations

### Embedded (RP2040/RP2350)

Flash memory is limited (2MB on Pico). Feature flags ensure minimal footprint.

### Host Tests

All vendor modules may be compiled for comprehensive testing. Use `--all-features` for test builds.

## Risks & Mitigation

| Risk                           | Impact | Likelihood | Mitigation                        | Validation             |
| ------------------------------ | ------ | ---------- | --------------------------------- | ---------------------- |
| Default features misconfigured | Medium | Low        | Document default in README        | CI build with defaults |
| Feature combination untested   | Low    | Medium     | CI matrix for common combinations | Add CI job             |

## Implementation Notes

- Use `#[cfg(feature = "...")]` on module declarations, not individual functions
- Avoid `#[cfg(any(...))]` patterns that couple vendors together
- Document feature flags in crate-level documentation

## External References

- [Rust Conditional Compilation](https://doc.rust-lang.org/reference/conditional-compilation.html) - Feature flag syntax reference

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
