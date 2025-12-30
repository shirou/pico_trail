# T-djbdm GPS Initialization Module Refactoring

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-vzkte-gps-initialization-abstraction](../../analysis/AN-vzkte-gps-initialization-abstraction.md)
- Related Requirements:
  - [FR-pcr03-gps-initialization-separation](../../requirements/FR-pcr03-gps-initialization-separation.md)
  - [FR-njq2p-gps-ublox-initialization](../../requirements/FR-njq2p-gps-ublox-initialization.md)
  - [FR-xpfte-gps-vendor-extensibility](../../requirements/FR-xpfte-gps-vendor-extensibility.md)
  - [NFR-x9k5x-gps-initialization-code-size](../../requirements/NFR-x9k5x-gps-initialization-code-size.md)
- Related ADRs:
  - [ADR-27cz1-gps-initialization-module-structure](../../adr/ADR-27cz1-gps-initialization-module-structure.md)
- Associated Design Document:
  - [T-djbdm-gps-initialization-module-design](design.md)
- Associated Plan Document:
  - [T-djbdm-gps-initialization-module-plan](plan.md)

## Summary

Refactor GPS initialization code from `GpsDriver` into a separate `src/devices/gps/init/` module directory with standalone functions per vendor (starting with u-blox), gated by feature flags. This enables multi-vendor GPS support while keeping the core driver focused on NMEA parsing.

## Scope

- In scope:
  - Extract `init_ublox()`, `build_cfg_msg()`, `ubx_checksum()` from `GpsDriver` to `src/devices/gps/init/ublox.rs`
  - Create `src/devices/gps/init/mod.rs` with feature-gated exports
  - Add `gps-ublox` feature flag to `Cargo.toml`
  - Ensure `GpsDriver::uart_mut()` is public for initialization access
  - Update existing example code to use new initialization path
  - Maintain backward compatibility for NMEA parsing functionality
- Out of scope:
  - MTK, SiRF, or CASIC vendor implementations (future work)
  - Auto-detection of GPS vendor
  - Async initialization (keep sync for simplicity)
  - Changing NMEA parsing logic

## Success Metrics

- All existing GPS unit tests pass without modification
- Embedded build (`./scripts/build-rp2350.sh`) compiles successfully
- Example code works with explicit initialization call
- Binary size does not increase significantly when only one vendor is enabled

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
