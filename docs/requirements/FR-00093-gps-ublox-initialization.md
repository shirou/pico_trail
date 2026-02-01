# FR-00093 GPS u-blox Initialization

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Source Analysis:
  - [AN-00025-gps-initialization-abstraction](../analysis/AN-00025-gps-initialization-abstraction.md)
- Prerequisite Requirements:
  - [FR-00092-gps-initialization-separation](../requirements/FR-00092-gps-initialization-separation.md)
- Dependent Requirements: (none)
- Related ADRs:
  - [ADR-00024-gps-initialization-module-structure](../adr/ADR-00024-gps-initialization-module-structure.md)
- Related Tasks:
  - [T-00020-gps-initialization-module](../tasks/T-00020-gps-initialization-module/README.md)

## Requirement Statement

The system shall provide a u-blox initialization module that sends UBX-CFG-MSG commands to enable required NMEA sentences (GGA, RMC, VTG) on u-blox GPS modules (NEO-M8N and compatible).

## Rationale

u-blox GPS modules may not output all required NMEA sentences by default. The NEO-M8N in particular may only output GPGSV without explicit configuration. The UBX binary protocol allows enabling specific NMEA messages (GGA for position, RMC for speed/COG, VTG as backup) to ensure the GPS driver receives complete navigation data.

## User Story (if applicable)

As a user with a u-blox NEO-M8N GPS module, I want the system to configure the module to output required NMEA sentences, so that position and speed data are available for navigation.

## Acceptance Criteria

- [ ] `gps::init::ublox::initialize()` function sends UBX-CFG-MSG commands via UART
- [ ] GGA message (0xF0, 0x00) enabled at rate 1 (every navigation solution)
- [ ] RMC message (0xF0, 0x04) enabled at rate 1
- [ ] VTG message (0xF0, 0x05) enabled at rate 1
- [ ] UBX checksum calculated correctly (Fletcher-16 algorithm)
- [ ] Function returns `Result<()>` with UART write errors propagated
- [ ] Functionality equivalent to current `GpsDriver::init_ublox()`

## Technical Details (if applicable)

### Functional Requirement Details

**UBX-CFG-MSG Command Structure:**

```
Byte 0-1: Sync chars (0xB5, 0x62)
Byte 2:   Class (0x06 = CFG)
Byte 3:   ID (0x01 = MSG)
Byte 4-5: Length (3, little endian)
Byte 6:   Message class (0xF0 = NMEA standard)
Byte 7:   Message ID (0x00=GGA, 0x04=RMC, 0x05=VTG)
Byte 8:   Rate (1 = every navigation solution)
Byte 9-10: Checksum (CK_A, CK_B)
```

**Checksum Algorithm (Fletcher-16):**

```rust
fn ubx_checksum(data: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &byte in data {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}
```

**Module Location:**

```
src/devices/gps/init/
├── mod.rs      # #[cfg(feature = "gps-ublox")] pub mod ublox;
└── ublox.rs    # initialize(), build_cfg_msg(), ubx_checksum()
```

## Platform Considerations

N/A – Platform agnostic. UBX commands are pure byte sequences sent via any `UartInterface`.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                       | Validation                  |
| ----------------------------------------- | ------ | ---------- | -------------------------------- | --------------------------- |
| UBX commands rejected by non-ublox module | Low    | Low        | Feature flag isolates ublox code | Test with NEO-M8N hardware  |
| Checksum calculation error                | High   | Low        | Port existing tested code        | Unit test checksum function |

## Implementation Notes

- Port existing `init_ublox()`, `build_cfg_msg()`, and `ubx_checksum()` from `GpsDriver`
- No functional changes required; this is a code reorganization
- Consider adding configurable message rates in future enhancement

## External References

- [u-blox NEO-M8 Interface Description (UBX-13003221)](https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf) - UBX protocol specification
