# FR-00094 GPS Vendor Extensibility

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Source Analysis:
  - [AN-00025-gps-initialization-abstraction](../analysis/AN-00025-gps-initialization-abstraction.md)
- Prerequisite Requirements:
  - [FR-00092-gps-initialization-separation](../requirements/FR-00092-gps-initialization-separation.md)
- Dependent Requirements:
  - [NFR-00071-gps-initialization-code-size](../requirements/NFR-00071-gps-initialization-code-size.md)
- Related ADRs:
  - [ADR-00024-gps-initialization-module-structure](../adr/ADR-00024-gps-initialization-module-structure.md)
- Related Tasks:
  - [T-00020-gps-initialization-module](../tasks/T-00020-gps-initialization-module/README.md)

## Requirement Statement

The GPS initialization module structure shall enable adding new GPS vendor support by creating a new module file and feature flag, without modifying existing code in `GpsDriver` or other vendor modules.

## Rationale

Different GPS vendors use different configuration protocols (u-blox uses UBX binary, MTK uses PMTK text commands, SiRF uses proprietary binary). A modular structure allows each vendor's initialization to be implemented independently, with feature flags ensuring only needed vendor code is compiled into the final binary.

## User Story (if applicable)

As a developer adding support for a new GPS vendor (e.g., MTK), I want to create a new module without touching existing code, so that I can contribute vendor support without risk of breaking existing functionality.

## Acceptance Criteria

- [ ] Adding a new GPS vendor requires only:
  - Creating `src/devices/gps/init/<vendor>.rs`
  - Adding feature flag `gps-<vendor>` to `Cargo.toml`
  - Adding conditional export in `src/devices/gps/init/mod.rs`
- [ ] No changes required to `GpsDriver` implementation
- [ ] No changes required to existing vendor modules (e.g., `ublox.rs`)
- [ ] Each vendor module exports `pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()>`
- [ ] Documentation exists for adding new vendor support

## Technical Details (if applicable)

### Functional Requirement Details

**Module Structure:**

```
src/devices/gps/init/
├── mod.rs          # Feature-gated re-exports
├── ublox.rs        # u-blox UBX protocol
├── mtk.rs          # MediaTek PMTK protocol (future)
├── sirf.rs         # SiRF binary protocol (future)
└── README.md       # Instructions for adding vendors
```

**Feature Flag Pattern in mod.rs:**

```rust
// src/devices/gps/init/mod.rs

#[cfg(feature = "gps-ublox")]
pub mod ublox;

#[cfg(feature = "gps-mtk")]
pub mod mtk;

// Future vendors follow same pattern
```

**Standard Initialization Function Signature:**

```rust
/// Initialize GPS module for <vendor>
///
/// Sends vendor-specific commands to configure NMEA output.
/// Call once after creating GpsDriver.
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
    // Vendor-specific initialization
}
```

**Adding New Vendor Checklist:**

1. Create `src/devices/gps/init/<vendor>.rs`
2. Implement `pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()>`
3. Add to `Cargo.toml`:
   ```toml
   [features]
   gps-<vendor> = []
   ```
4. Add to `src/devices/gps/init/mod.rs`:
   ```rust
   #[cfg(feature = "gps-<vendor>")]
   pub mod <vendor>;
   ```
5. Add unit tests for vendor-specific commands
6. Document in README

## Platform Considerations

N/A – Platform agnostic. Vendor modules are pure Rust with no platform dependencies.

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                     | Validation  |
| -------------------------------- | ------ | ---------- | ------------------------------ | ----------- |
| Inconsistent function signatures | Medium | Low        | Document standard signature    | Code review |
| Feature flag naming conflicts    | Low    | Low        | Use `gps-` prefix consistently | CI check    |

## Implementation Notes

- Start with u-blox as the reference implementation
- Keep each vendor module self-contained (no cross-vendor dependencies)
- Consider adding a `generic` module that does nothing (for modules that work with defaults)

## External References

- [MTK NMEA Extension Commands](https://cdn.sparkfun.com/datasheets/GPS/GlobalTop_PMTK_Command_List.pdf) - MTK PMTK protocol reference
- [u-blox Receiver Description](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) - UBX protocol reference
