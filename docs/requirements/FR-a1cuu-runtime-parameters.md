# FR-a1cuu Runtime Parameter Configuration

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall provide a runtime parameter configuration system supporting at least 200 parameters with Flash persistence, allowing tuning and configuration via MAVLink protocol without requiring firmware reflashing.

## Rationale

Parameter systems enable tuning and configuration without recompilation:

- **Field Tuning**: Adjust PID gains, navigation speeds, failsafe thresholds on-site
- **Vehicle Customization**: Configure for different vehicle types, sensor setups, missions
- **Persistence**: Parameters survive reboots and power cycles
- **GCS Integration**: Change parameters via QGroundControl or Mission Planner
- **Debugging**: Enable/disable features, adjust logging levels

ArduPilot uses \~300 parameters for rovers. Supporting 200 parameters covers essential functionality while respecting embedded constraints.

## User Story (if applicable)

As an operator, I want to adjust navigation speeds, control gains, and failsafe thresholds via the ground control station, so that I can tune the autopilot for different vehicles and conditions without reflashing firmware.

## Acceptance Criteria

- [ ] System supports at least 200 configurable parameters
- [ ] Parameters accessible via MAVLink parameter protocol (PARAM_REQUEST_LIST, PARAM_SET)
- [ ] Parameters persist to Flash storage and survive reboots
- [ ] Parameter load time from Flash < 100ms during system initialization
- [ ] Parameter names follow ArduPilot conventions (e.g., `COMPASS_OFS_X`, `PID_RATE_P`)
- [ ] Parameters organized into groups (e.g., `COMPASS_*`, `PID_*`, `WP_*`)
- [ ] Each parameter has: name, default value, min/max bounds, data type (f32 or u32)
- [ ] Parameter changes take effect immediately without reboot (where applicable)
- [ ] Invalid parameter values rejected (out of bounds, wrong type)

## Technical Details (if applicable)

### Functional Requirement Details

**Parameter Organization:**

Group parameters by subsystem using prefix convention:

```
COMPASS_DEV_ID: Compass device ID (u32)
COMPASS_OFS_X: X-axis offset (f32, -1000.0 to 1000.0)
COMPASS_OFS_Y: Y-axis offset (f32, -1000.0 to 1000.0)

PID_RATE_P: Rate controller P gain (f32, 0.0 to 2.0)
PID_RATE_I: Rate controller I gain (f32, 0.0 to 1.0)
PID_RATE_D: Rate controller D gain (f32, 0.0 to 0.1)

WP_SPEED: Waypoint navigation speed (f32, 0.0 to 10.0 m/s)
WP_ACCEL: Maximum acceleration (f32, 0.1 to 5.0 m/s²)
WP_RADIUS: Waypoint acceptance radius (f32, 0.5 to 10.0 m)
```

**Storage Layout:**

```
Flash Layout:
[Parameter Block 1] (4 KB) - Active parameters
[Parameter Block 2] (4 KB) - Backup parameters
```

Each block contains:

- Magic header (4 bytes): `0x50415241` ("PARA")
- Version (2 bytes): Parameter format version
- Parameter count (2 bytes)
- Parameter data (name + value pairs)
- CRC32 checksum (4 bytes)

**Parameter Types:**

- `f32`: Floating-point (e.g., gains, speeds, offsets)
- `u32`: Unsigned integer (e.g., device IDs, enable flags, bitmasks)

**Flash Write Strategy:**

- Cache all parameters in RAM (2 KB typical)
- Write to Flash only on explicit save command (avoid wear)
- Use redundant blocks (Block 1 and Block 2) for corruption recovery
- Implement wear leveling by rotating writes across multiple blocks

**Parameter Access:**

```rust
// Get parameter value
let speed = params.get_f32("WP_SPEED")?;

// Set parameter value
params.set_f32("WP_SPEED", 2.5)?;

// Save to Flash (explicit, not automatic)
params.save_to_flash().await?;
```

## Platform Considerations

### Pico W (RP2040)

2 MB Flash - Adequate for parameters, missions, and limited logging. Wear leveling essential.

### Pico 2 W (RP2350)

4 MB Flash - More comfortable for parameters and extensive logging.

### Cross-Platform

Parameter system must be platform-independent. Use platform Flash abstraction.

## Risks & Mitigation

| Risk                                       | Impact | Likelihood | Mitigation                                                   | Validation                                    |
| ------------------------------------------ | ------ | ---------- | ------------------------------------------------------------ | --------------------------------------------- |
| Flash wear from excessive parameter saves  | High   | Medium     | Write only on explicit save command, implement wear leveling | Monitor Flash erase count in accelerated test |
| Parameter corruption during Flash write    | High   | Low        | Use redundant blocks, CRC checksums, validate on load        | Test with power loss during write             |
| Parameter name conflicts (duplicate names) | Medium | Low        | Enforce unique names at compile time via macro               | Unit test parameter registration              |
| RAM usage too high (>2 KB cache)           | Medium | Low        | Limit to 200 parameters, use compact encoding                | Measure actual RAM usage during runtime       |

## Implementation Notes

Preferred approaches:

- Use **hierarchical parameter groups** (ArduPilot-style) for organization
- Define parameters via **macro** for compile-time validation:
  ```rust
  param_group! {
      COMPASS {
          DEV_ID: u32 = 0,
          OFS_X: f32 = 0.0 (-1000.0, 1000.0),
          OFS_Y: f32 = 0.0 (-1000.0, 1000.0),
      }
  }
  ```
- Implement **async Flash writes** to avoid blocking control loops

Known pitfalls:

- Parameter names limited to 16 characters (MAVLink constraint)
- Flash write can block for 100ms+ - must not stall control loops
- Default values must be sane (vehicle should operate safely without tuning)
- Some parameters require reboot to take effect (document which ones)

Related code areas:

- `src/core/parameters/` - Parameter system implementation
- `src/platform/*/flash.rs` - Flash storage abstraction
- `src/communication/mavlink/param.rs` - MAVLink parameter protocol

Suggested libraries:

- `embedded-storage` for Flash abstraction
- `serde` (with `no_std`) for parameter serialization
- `crc` for checksum calculation

## External References

- ArduPilot Parameter System: <https://ardupilot.org/dev/docs/code-overview-adding-a-new-parameter.html>
- MAVLink Parameter Protocol: <https://mavlink.io/en/services/parameter.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
