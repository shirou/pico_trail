# T-djbdm GPS Initialization Module | Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-djbdm-gps-initialization-module-plan](plan.md)

## Overview

This design describes the refactoring of GPS vendor-specific initialization code from `GpsDriver` into a separate module structure. The goal is to enable multi-vendor GPS support through compile-time feature flags while keeping the core NMEA parsing driver vendor-agnostic.

## Success Metrics

- [ ] All existing GPS unit tests pass without modification
- [ ] Binary size increase is negligible (<500 bytes) for single-vendor builds
- [ ] Embedded build compiles successfully with `gps-ublox` feature

## Background and Current State

- Context: GPS initialization is currently tightly coupled with `GpsDriver` in `src/devices/gps.rs`
- Current behavior:
  - `GpsDriver::init_ublox()` sends UBX-CFG-MSG commands to enable NMEA sentences
  - `build_cfg_msg()` and `ubx_checksum()` are private helper functions
  - All code is compiled regardless of GPS vendor used
- Pain points:
  - Cannot easily add support for MTK, SiRF, or other GPS vendors
  - u-blox specific code is always included in binary
  - Single responsibility principle violated (parsing + initialization mixed)
- Constraints:
  - Must maintain backward compatibility for existing example code
  - Must work in no_std environment (no heap allocation)
  - Must integrate with Embassy async runtime
- Related ADRs: [ADR-27cz1-gps-initialization-module-structure](../../adr/ADR-27cz1-gps-initialization-module-structure.md)

## Proposed Design

### High-Level Architecture

```text
src/devices/gps/
├── mod.rs              # Re-exports (GpsDriver, GpsPosition, etc.)
├── driver.rs           # GpsDriver - NMEA parsing only (current gps.rs)
├── position.rs         # GpsPosition, GpsFixType types
└── init/
    ├── mod.rs          # Feature-gated vendor module exports
    └── ublox.rs        # u-blox UBX initialization functions

Usage Flow:
┌─────────────────┐     ┌──────────────────┐
│  Application    │     │  gps::init::     │
│  Example Code   │────>│  ublox::         │
│                 │     │  initialize()    │
└────────┬────────┘     └────────┬─────────┘
         │                       │
         │                       │ (writes UBX commands)
         v                       v
┌─────────────────────────────────────────────┐
│              GpsDriver                       │
│  ┌─────────────┐    ┌─────────────────────┐ │
│  │ uart: U     │<───│ uart_mut() accessor │ │
│  └─────────────┘    └─────────────────────┘ │
│  ┌───────────────────────────────────────┐  │
│  │ NMEA parsing (GGA, RMC, VTG)          │  │
│  └───────────────────────────────────────┘  │
└─────────────────────────────────────────────┘
```

### Components

- `src/devices/gps/mod.rs` - Module root with re-exports for backward compatibility
- `src/devices/gps/driver.rs` - `GpsDriver<U>` struct, NMEA parsing, `uart_mut()` accessor
- `src/devices/gps/position.rs` - `GpsPosition`, `GpsFixType`, `GpsInternalState` types
- `src/devices/gps/init/mod.rs` - Feature-gated vendor module exports
- `src/devices/gps/init/ublox.rs` - `initialize()`, `build_cfg_msg()`, `ubx_checksum()` functions

### Data Flow

1. Application creates `GpsDriver::new(uart)`
2. Application calls `gps::init::ublox::initialize(driver.uart_mut())?` (feature-gated)
3. `initialize()` sends UBX-CFG-MSG commands via UART to enable GGA, RMC, VTG
4. Application calls `driver.update()` or `driver.read_position()` for NMEA data
5. GPS module outputs configured NMEA sentences
6. `GpsDriver` parses sentences and updates internal state

### Data Models and Types

**Existing types (no changes):**

```rust
pub struct GpsPosition {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub speed: f32,
    pub course_over_ground: f32,
    pub fix_type: GpsFixType,
    pub satellites: u8,
}

pub enum GpsFixType {
    NoFix,
    Fix2D,
    Fix3D,
}
```

**New module structure:**

```rust
// src/devices/gps/init/mod.rs
#[cfg(feature = "gps-ublox")]
pub mod ublox;

// src/devices/gps/init/ublox.rs
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()>;
fn build_cfg_msg(msg_class: u8, msg_id: u8, rate: u8) -> [u8; 11];
fn ubx_checksum(data: &[u8]) -> (u8, u8);
```

### Error Handling

- `initialize()` returns `Result<()>` with UART write errors propagated
- No new error types required; uses existing `crate::Error` type
- Errors are not recoverable; initialization failure should halt GPS setup

### Security Considerations

- N/A - No security implications for GPS initialization commands

### Performance Considerations

- Zero runtime overhead: feature flags exclude unused vendor code at compile time
- `initialize()` is called once at startup; not performance critical
- No heap allocations; all buffers are stack-allocated arrays

### Platform Considerations

#### Embedded (RP2350)

- Feature flag `gps-ublox` included in `pico2_w` feature set
- UBX commands sent via `embassy_rp::uart::BufferedUart`

#### Host Tests

- Mock UART captures commands for verification
- All vendor modules can be compiled for testing with `--all-features`

## Alternatives Considered

1. Trait-Based Abstraction (`GpsInitializer` trait)
   - Pros: Type-safe interface enforcement
   - Cons: More complex, requires generic parameter threading
2. Enum-Based Dispatch (`GpsVendor` enum with match)
   - Pros: Simple, familiar pattern
   - Cons: All vendor code compiled in; violates code size requirement

Decision Rationale

- Standalone functions with feature flags chosen per ADR-27cz1 for simplicity and zero-cost abstraction

## Migration and Compatibility

- Backward compatibility: `GpsDriver::init_ublox()` can be deprecated but kept as a thin wrapper initially
- Forward compatibility: New vendors added by creating new module + feature flag
- Deprecation plan: Remove `GpsDriver::init_ublox()` wrapper after one release cycle

## Testing Strategy

### Unit Tests

- Test `ubx_checksum()` with known inputs (existing tests)
- Test `build_cfg_msg()` produces correct byte sequences
- Existing NMEA parsing tests remain unchanged

### Integration Tests

- Verify example code compiles and runs with new initialization path
- Test with real NEO-M8N hardware on RP2350

## Documentation Impact

- Update example code comments to show new initialization pattern
- Add inline documentation for `gps::init::ublox::initialize()`

## External References

- [u-blox NEO-M8 Interface Description (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) - UBX protocol specification

## Open Questions

- [x] Should `GpsDriver::init_ublox()` be kept as deprecated wrapper? - Yes, for one release cycle

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
