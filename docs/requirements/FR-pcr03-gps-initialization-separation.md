# FR-pcr03 GPS Initialization Separation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Source Analysis:
  - [AN-vzkte-gps-initialization-abstraction](../analysis/AN-vzkte-gps-initialization-abstraction.md)
- Prerequisite Requirements:
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
- Dependent Requirements:
  - [FR-njq2p-gps-ublox-initialization](../requirements/FR-njq2p-gps-ublox-initialization.md)
  - [FR-xpfte-gps-vendor-extensibility](../requirements/FR-xpfte-gps-vendor-extensibility.md)
  - [NFR-x9k5x-gps-initialization-code-size](../requirements/NFR-x9k5x-gps-initialization-code-size.md)
- Related ADRs:
  - [ADR-27cz1-gps-initialization-module-structure](../adr/ADR-27cz1-gps-initialization-module-structure.md)
- Related Tasks:
  - [T-djbdm-gps-initialization-module](../tasks/T-djbdm-gps-initialization-module/README.md)

## Requirement Statement

The GPS driver shall separate initialization logic from NMEA parsing logic, with initialization implemented as standalone modules independent of the core `GpsDriver` struct.

## Rationale

The current `GpsDriver` implementation has ublox-specific initialization code (`init_ublox()`) embedded directly in the driver. This tight coupling prevents easy support for other GPS chipsets (MTK, SiRF, CASIC) without modifying the core driver. Separating initialization from parsing follows the single responsibility principle and enables vendor-agnostic GPS operation.

## User Story (if applicable)

As a developer integrating a non-ublox GPS module, I want initialization logic separated from NMEA parsing, so that I can add support for my GPS vendor without modifying the core driver.

## Acceptance Criteria

- [ ] `GpsDriver` struct contains no vendor-specific initialization code
- [ ] `init_ublox()`, `build_cfg_msg()`, and `ubx_checksum()` functions are moved to a separate module
- [ ] `GpsDriver::new()` and NMEA parsing methods remain unchanged in functionality
- [ ] Initialization functions accept a mutable reference to `UartInterface` for sending commands
- [ ] Existing example code continues to work with updated import paths
- [ ] Unit tests for NMEA parsing pass without modification

## Technical Details (if applicable)

### Functional Requirement Details

**Current Structure (before):**

```rust
// src/devices/gps.rs
pub struct GpsDriver<U: UartInterface> { ... }

impl<U: UartInterface> GpsDriver<U> {
    pub fn new(uart: U) -> Self { ... }
    pub fn read_position(&mut self) -> Result<Option<GpsPosition>> { ... }
    pub fn init_ublox(&mut self) -> Result<()> { ... }  // Vendor-specific
    fn build_cfg_msg(...) -> [u8; 11] { ... }           // Vendor-specific
    fn ubx_checksum(...) -> (u8, u8) { ... }            // Vendor-specific
}
```

**Target Structure (after):**

```rust
// src/devices/gps/mod.rs
pub mod driver;
pub mod position;
pub mod init;

pub use driver::GpsDriver;
pub use position::{GpsPosition, GpsFixType};

// src/devices/gps/driver.rs
pub struct GpsDriver<U: UartInterface> { ... }

impl<U: UartInterface> GpsDriver<U> {
    pub fn new(uart: U) -> Self { ... }
    pub fn read_position(&mut self) -> Result<Option<GpsPosition>> { ... }
    pub fn uart_mut(&mut self) -> &mut U { ... }  // For initialization access
}

// src/devices/gps/init/ublox.rs
pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()> { ... }
```

**Usage Pattern:**

```rust
let mut gps = GpsDriver::new(uart);

// Initialize for ublox module
#[cfg(feature = "gps-ublox")]
gps::init::ublox::initialize(gps.uart_mut())?;

// Then use normally
if let Some(pos) = gps.read_position()? { ... }
```

## Platform Considerations

N/A – Platform agnostic. The separation applies to all targets (RP2040, RP2350, host tests).

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                                     | Validation           |
| ---------------------------------- | ------ | ---------- | ---------------------------------------------- | -------------------- |
| Breaking API changes               | Medium | Low        | Preserve public API, add `uart_mut()` accessor | Run existing tests   |
| Import path changes break examples | Low    | Medium     | Update examples, document migration            | Compile all examples |

## Implementation Notes

- The `GpsDriver` should expose `uart_mut()` to allow initialization modules to send commands
- Consider making `uart_mut()` public (not just `#[cfg(test)]`) for initialization access
- Keep backward compatibility by re-exporting types from `src/devices/gps/mod.rs`

## External References

N/A – No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
