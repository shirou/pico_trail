# AN-vzkte GPS Initialization Abstraction

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../analysis/AN-xfiyr-gps-hardware-integration.md)
- Related Requirements:
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-pcr03-gps-initialization-separation](../requirements/FR-pcr03-gps-initialization-separation.md)
  - [FR-njq2p-gps-ublox-initialization](../requirements/FR-njq2p-gps-ublox-initialization.md)
  - [FR-xpfte-gps-vendor-extensibility](../requirements/FR-xpfte-gps-vendor-extensibility.md)
  - [NFR-x9k5x-gps-initialization-code-size](../requirements/NFR-x9k5x-gps-initialization-code-size.md)
- Related ADRs:
  - [ADR-27cz1-gps-initialization-module-structure](../adr/ADR-27cz1-gps-initialization-module-structure.md)
- Related Tasks:
  - [T-djbdm-gps-initialization-module](../tasks/T-djbdm-gps-initialization-module/README.md)

## Executive Summary

The current GPS driver (`src/devices/gps.rs`) has ublox-specific initialization code (`init_ublox`) hardcoded into `GpsDriver`. This works for NEO-M8N modules but prevents easy support for other GPS chipsets (MTK, SiRF, CASIC, etc.). This analysis evaluates approaches to abstract GPS initialization, enabling multi-vendor support while maintaining the existing NMEA parsing infrastructure.

## Problem Space

### Current State

The GPS driver architecture has:

- **Platform-independent NMEA parsing**: `GpsDriver<U: UartInterface>` handles NMEA sentences (GGA, RMC, VTG)
- **Ublox-specific initialization**: `init_ublox()` method sends UBX-CFG-MSG commands to enable required NMEA messages
- **Tight coupling**: Initialization logic is embedded directly in `GpsDriver`

```rust
// Current implementation (src/devices/gps.rs:362-374)
pub fn init_ublox(&mut self) -> Result<()> {
    let enable_gga = Self::build_cfg_msg(0xF0, 0x00, 1); // NMEA-GGA
    let enable_rmc = Self::build_cfg_msg(0xF0, 0x04, 1); // NMEA-RMC
    let enable_vtg = Self::build_cfg_msg(0xF0, 0x05, 1); // NMEA-VTG
    self.uart.write(&enable_gga)?;
    self.uart.write(&enable_rmc)?;
    self.uart.write(&enable_vtg)?;
    Ok(())
}
```

### Desired State

- **Vendor-agnostic GPS driver**: `GpsDriver` focuses on NMEA parsing only
- **Pluggable initialization**: Different GPS vendors supported via initialization modules
- **Easy extensibility**: Adding new GPS vendor support requires minimal code changes
- **Compile-time selection**: Vendor support selectable via feature flags (no runtime overhead)

### Gap Analysis

| Aspect            | Current                       | Desired                         |
| ----------------- | ----------------------------- | ------------------------------- |
| Initialization    | Hardcoded ublox UBX commands  | Pluggable per-vendor modules    |
| Adding new vendor | Modify `GpsDriver` directly   | Add new module, no core changes |
| Code organization | Mixed concerns in single file | Separated by vendor             |
| Feature selection | None (always ublox)           | Feature flags for each vendor   |

## Stakeholder Analysis

| Stakeholder              | Interest/Need                    | Impact | Priority |
| ------------------------ | -------------------------------- | ------ | -------- |
| Developers               | Easy addition of new GPS vendors | High   | P0       |
| Users with non-ublox GPS | Support for MTK, SiRF modules    | High   | P0       |
| Maintainers              | Clean separation of concerns     | Medium | P1       |
| Testing                  | Per-vendor unit testing          | Medium | P1       |

## Research & Discovery

### GPS Vendor Initialization Protocols

| Vendor      | Protocol     | Init Purpose                    | Message Format                                |
| ----------- | ------------ | ------------------------------- | --------------------------------------------- |
| **u-blox**  | UBX binary   | Enable/disable NMEA messages    | `0xB5 0x62 <class> <id> <len> <payload> <ck>` |
| **MTK**     | PMTK text    | Configure update rate, messages | `$PMTK<cmd>,<params>*<checksum>`              |
| **SiRF**    | SiRF binary  | Set protocols, message rates    | Binary with 0xA0 0xA2 sync                    |
| **CASIC**   | CASIC binary | Similar to ublox                | Proprietary binary format                     |
| **Generic** | None         | Most modules work with defaults | N/A                                           |

### Common Initialization Tasks

1. **Enable required NMEA sentences**: GGA (position), RMC (speed/COG), VTG (speed/COG backup)
2. **Disable unnecessary sentences**: GSV, GSA, GLL (reduce UART traffic)
3. **Set update rate**: 1Hz (default), 5Hz, 10Hz
4. **Configure baud rate**: 9600 (default) or higher for faster updates
5. **Set navigation mode**: Automotive, pedestrian, stationary, etc.

### ArduPilot Approach

ArduPilot uses a similar pattern:

- `AP_GPS_Backend` base class with virtual methods
- `AP_GPS_UBLOX`, `AP_GPS_MTK`, `AP_GPS_SBF` derived classes
- Auto-detection via probing each protocol

### Rust Idiomatic Approaches

1. **Trait-based**: Define `GpsInitializer` trait, implement per vendor
2. **Enum-based**: `GpsVendor` enum with match-based dispatch
3. **Module-based**: Separate modules with standalone `init()` functions
4. **Feature-gated functions**: Vendor-specific code behind feature flags

## Discovered Requirements

### Functional Requirements

- [x] **FR-DRAFT-1** → [FR-pcr03-gps-initialization-separation](../requirements/FR-pcr03-gps-initialization-separation.md)
  - GPS initialization shall be separated from NMEA parsing

- [x] **FR-DRAFT-2** → [FR-njq2p-gps-ublox-initialization](../requirements/FR-njq2p-gps-ublox-initialization.md)
  - System shall support ublox GPS initialization

- [x] **FR-DRAFT-3** → [FR-xpfte-gps-vendor-extensibility](../requirements/FR-xpfte-gps-vendor-extensibility.md)
  - System shall provide extensible GPS vendor support

### Non-Functional Requirements

- [x] **NFR-DRAFT-1** → [NFR-x9k5x-gps-initialization-code-size](../requirements/NFR-x9k5x-gps-initialization-code-size.md)
  - GPS initialization code size shall be minimized via feature flags

## Design Considerations

### Technical Constraints

- **no_std environment**: No heap allocation, static dispatch preferred
- **Embedded flash limits**: Feature flags to exclude unused vendor code
- **UART interface**: All vendors use same UART for commands
- **Async compatibility**: Must work with Embassy async runtime

### Potential Approaches

#### Option A: Trait-Based Abstraction

```rust
// src/devices/gps/initializer.rs
pub trait GpsInitializer {
    fn initialize<U: UartInterface>(uart: &mut U) -> Result<()>;
}

// src/devices/gps/ublox.rs
pub struct UbloxInitializer;
impl GpsInitializer for UbloxInitializer {
    fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
        // UBX commands...
    }
}

// src/devices/gps/mtk.rs
pub struct MtkInitializer;
impl GpsInitializer for MtkInitializer {
    fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
        // PMTK commands...
    }
}
```

- **Pros**: Type-safe, extensible, idiomatic Rust
- **Cons**: Slightly more complex, requires generic parameter
- **Effort**: Medium

#### Option B: Standalone Functions with Feature Flags

```rust
// src/devices/gps/init/mod.rs
#[cfg(feature = "gps-ublox")]
pub mod ublox;

#[cfg(feature = "gps-mtk")]
pub mod mtk;

// src/devices/gps/init/ublox.rs
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> {
    // UBX commands...
}
```

- **Pros**: Simple, zero-cost abstraction, easy to add vendors
- **Cons**: No compile-time enforcement of interface
- **Effort**: Low

#### Option C: Enum-Based Dispatch

```rust
pub enum GpsVendor {
    Ublox,
    Mtk,
    Generic,
}

impl GpsDriver<U> {
    pub fn initialize(&mut self, vendor: GpsVendor) -> Result<()> {
        match vendor {
            GpsVendor::Ublox => self.init_ublox(),
            GpsVendor::Mtk => self.init_mtk(),
            GpsVendor::Generic => Ok(()),
        }
    }
}
```

- **Pros**: Very simple, familiar pattern
- **Cons**: All vendor code compiled in, switch-case grows with vendors
- **Effort**: Low

### Recommended Approach

**Option B (Standalone Functions with Feature Flags)** is recommended because:

1. **Simplicity**: Minimal abstraction overhead, easy to understand
2. **Zero-cost**: Feature flags exclude unused vendor code from binary
3. **Extensibility**: Adding vendor = adding module + feature flag
4. **Consistency**: Matches project's existing feature flag patterns (`pico2_w`, `embassy`)

### Proposed Directory Structure

```
src/devices/gps/
├── mod.rs          # Re-exports, GpsDriver definition
├── driver.rs       # Core NMEA parsing (current gps.rs content)
├── position.rs     # GpsPosition, GpsFixType types
└── init/
    ├── mod.rs      # Feature-gated re-exports
    ├── ublox.rs    # u-blox UBX initialization
    └── mtk.rs      # MTK PMTK initialization (future)
```

### Architecture Impact

**ADR Required:**

- **ADR-\<id>-gps-initialization-abstraction**: Document decision to separate initialization from parsing

**Changes Required:**

1. Refactor `src/devices/gps.rs` into module structure
2. Move `init_ublox()`, `build_cfg_msg()`, `ubx_checksum()` to `init/ublox.rs`
3. Add feature flag `gps-ublox` (default enabled)
4. Update example code to call initialization explicitly

## Risk Assessment

| Risk                            | Probability | Impact | Mitigation Strategy                    |
| ------------------------------- | ----------- | ------ | -------------------------------------- |
| Breaking existing functionality | Low         | High   | Preserve public API, extensive testing |
| Increased complexity            | Low         | Medium | Keep abstraction minimal (Option B)    |
| Feature flag misconfiguration   | Low         | Medium | Document clearly, add CI checks        |

## Open Questions

- [x] Which abstraction approach to use? → Recommend Option B (standalone functions)
- [ ] Should we add auto-detection of GPS vendor? → Defer to future enhancement
- [ ] Should initialization be async? → Recommend sync for simplicity

## Recommendations

### Immediate Actions

1. **Create module structure**: Refactor `gps.rs` into `gps/` directory
2. **Extract ublox code**: Move initialization to `gps/init/ublox.rs`
3. **Add feature flag**: `gps-ublox` feature, enabled by default
4. **Update imports**: Ensure backward compatibility

### Next Steps

1. [ ] Create formal requirements: FR-\<id> for GPS initialization abstraction
2. [ ] Draft ADR: GPS initialization module architecture
3. [ ] Create task: T-\<id>-gps-initialization-abstraction
4. [ ] Implement refactoring with backward compatibility

### Out of Scope

- **Auto-detection**: Probing multiple protocols adds complexity
- **MTK implementation**: Focus on abstraction first, add vendors later
- **Runtime vendor selection**: Compile-time feature flags sufficient

## Appendix

### References

- [u-blox NEO-M8 Interface Description](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
- [MTK NMEA Extension Reference](https://cdn.sparkfun.com/datasheets/GPS/GlobalTop_PMTK_Command_List.pdf)
- [ArduPilot GPS Backend](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_GPS)

### MTK PMTK Command Examples

```
# Enable GGA, RMC, VTG only (disable others)
$PMTK314,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29

# Set update rate to 5Hz
$PMTK220,200*2C

# Set baud rate to 38400
$PMTK251,38400*27
```

### UBX Command Reference (Current Implementation)

```
# UBX-CFG-MSG structure
Sync: 0xB5 0x62
Class: 0x06 (CFG)
ID: 0x01 (MSG)
Length: 3 bytes
Payload: [msgClass, msgID, rate]
Checksum: CK_A, CK_B (Fletcher-16)

# NMEA message IDs (class 0xF0)
GGA: 0x00
RMC: 0x04
VTG: 0x05
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
