# ADR-00024 GPS Initialization Module Structure

## Metadata

- Type: ADR (Lite)
- Status: Approved

## Links

- Source Analysis:
  - [AN-00025-gps-initialization-abstraction](../analysis/AN-00025-gps-initialization-abstraction.md)
- Impacted Requirements:
  - [FR-00092-gps-initialization-separation](../requirements/FR-00092-gps-initialization-separation.md)
  - [FR-00093-gps-ublox-initialization](../requirements/FR-00093-gps-ublox-initialization.md)
  - [FR-00094-gps-vendor-extensibility](../requirements/FR-00094-gps-vendor-extensibility.md)
  - [NFR-00071-gps-initialization-code-size](../requirements/NFR-00071-gps-initialization-code-size.md)
- Related Tasks:
  - [T-00020-gps-initialization-module](../tasks/T-00020-gps-initialization-module/README.md)

## Context

- Current `GpsDriver` has u-blox specific `init_ublox()` method hardcoded, preventing easy support for other GPS vendors (MTK, SiRF, CASIC)
- Different GPS vendors use different configuration protocols (UBX binary, PMTK text, SiRF binary)
- Embedded targets have limited flash; unused vendor code should be excluded at compile time
- Need to maintain backward compatibility with existing example code

## Decision

We will refactor GPS initialization into a separate `src/devices/gps/init/` module directory with standalone functions per vendor, gated by feature flags (e.g., `gps-ublox`). `GpsDriver` will focus solely on NMEA parsing and expose `uart_mut()` for initialization access.

## Consequences

- Positive: Adding new GPS vendor support requires only creating a new module file and feature flag
- Positive: Unused vendor code excluded from binary via feature flags (minimal flash footprint)
- Positive: Clear separation of concerns between NMEA parsing and vendor-specific initialization
- Positive: Each vendor module can be unit tested independently
- Negative: Slight increase in module count and import paths
- Negative: Users must explicitly call initialization function (not automatic)

## External References

- [u-blox Receiver Description (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) - UBX protocol specification
- [MTK PMTK Command List](https://cdn.sparkfun.com/datasheets/GPS/GlobalTop_PMTK_Command_List.pdf) - MTK protocol reference
