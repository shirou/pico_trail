# FR-uyie8 GPS MAVLink Telemetry Messages

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-3ik7l-gps-operation-data-management](../requirements/FR-3ik7l-gps-operation-data-management.md)
  - [FR-4z22l-gps-course-over-ground](../requirements/FR-4z22l-gps-course-over-ground.md)
- Dependent Requirements:
  - [NFR-5utah-gps-telemetry-performance](../requirements/NFR-5utah-gps-telemetry-performance.md)
- Related Tasks:
  - [T-vxtxn-uart0-gps-integration](../tasks/T-vxtxn-uart0-gps-integration/README.md)
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Requirement Statement

The system shall transmit GPS position data via MAVLink GPS_RAW_INT and GLOBAL_POSITION_INT messages, enabling GCS applications to display vehicle position on maps and log telemetry data.

## Rationale

GCS applications (QGroundControl, Mission Planner) require GPS telemetry to display vehicle position on maps, calculate distance/bearing to home, and log flight data. The current GPS_RAW_INT implementation sends placeholder zeros. Real GPS data must be integrated to enable effective vehicle monitoring and autonomous navigation support.

## User Story (if applicable)

As a GCS operator, I want to see the vehicle's real-time position on the map, so that I can monitor its location and plan navigation waypoints.

## Acceptance Criteria

- [ ] GPS_RAW_INT message contains real GPS data from NEO-M8N receiver
- [ ] GPS_RAW_INT.lat and .lon are populated in degE7 format (degrees \* 1e7)
- [ ] GPS_RAW_INT.alt is populated in mm (millimeters above MSL)
- [ ] GPS_RAW_INT.fix_type reflects actual GPS fix status (NO_FIX, 2D_FIX, 3D_FIX)
- [ ] GPS_RAW_INT.satellites_visible shows actual satellite count
- [ ] GPS_RAW_INT.vel is populated in cm/s (centimeters per second)
- [ ] GLOBAL_POSITION_INT message is transmitted with position data
- [ ] GLOBAL_POSITION_INT.lat, .lon, .alt match GPS_RAW_INT values
- [ ] GLOBAL_POSITION_INT.relative_alt is 0 when home not set, calculated when home is set
- [ ] GLOBAL_POSITION_INT.vx, .vy populated from speed and COG when available
- [ ] GLOBAL_POSITION_INT.hdg populated from COG or AHRS when available (UINT16_MAX if unknown)
- [ ] Messages transmitted at SR_POSITION rate (default 5Hz)

## Technical Details (if applicable)

### Functional Requirement Details

**GPS_RAW_INT (Message ID: 24):**

| Field              | Source                 | Conversion                          |
| ------------------ | ---------------------- | ----------------------------------- |
| time_usec          | SystemState.uptime_us  | Direct                              |
| lat                | GpsPosition.latitude   | `(lat * 1e7) as i32`                |
| lon                | GpsPosition.longitude  | `(lon * 1e7) as i32`                |
| alt                | GpsPosition.altitude   | `(alt * 1000.0) as i32`             |
| eph                | -                      | 9999 (unknown, until HDOP added)    |
| epv                | -                      | 9999 (unknown, until VDOP added)    |
| vel                | GpsPosition.speed      | `(speed * 100.0) as u16`            |
| cog                | GpsPosition.course     | `(course * 100.0) as u16` (0-35999) |
| fix_type           | GpsPosition.fix_type   | Enum mapping                        |
| satellites_visible | GpsPosition.satellites | Direct                              |

**GLOBAL_POSITION_INT (Message ID: 33):**

| Field        | Source                       | Conversion                           |
| ------------ | ---------------------------- | ------------------------------------ |
| time_boot_ms | SystemState.uptime_us / 1000 | `(uptime_us / 1000) as u32`          |
| lat          | GpsPosition.latitude         | `(lat * 1e7) as i32`                 |
| lon          | GpsPosition.longitude        | `(lon * 1e7) as i32`                 |
| alt          | GpsPosition.altitude         | `(alt * 1000.0) as i32`              |
| relative_alt | alt - home_alt               | `(rel_alt * 1000.0) as i32` or 0     |
| vx           | speed \* cos(cog)            | `(vx * 100.0) as i16` (cm/s North)   |
| vy           | speed \* sin(cog)            | `(vy * 100.0) as i16` (cm/s East)    |
| vz           | 0                            | GPS vertical rate unreliable         |
| hdg          | course or AHRS heading       | `(hdg * 100.0) as u16` or UINT16_MAX |

**Fix Type Mapping:**

| GpsFixType | MAVLink GPS_FIX_TYPE    |
| ---------- | ----------------------- |
| NoFix      | GPS_FIX_TYPE_NO_FIX (0) |
| Fix2D      | GPS_FIX_TYPE_2D_FIX (2) |
| Fix3D      | GPS_FIX_TYPE_3D_FIX (3) |

## Platform Considerations

### Embedded (RP2350)

- GPS data accessed from shared state (SystemState or dedicated GPS state)
- Unit conversion performed in telemetry handler
- No heap allocation in message building

### Host Tests

- Mock GPS data for telemetry handler unit tests
- Verify unit conversions with known values

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                                        | Validation                                           |
| -------------------------------- | ------ | ---------- | ------------------------------------------------- | ---------------------------------------------------- |
| GPS data stale when message sent | Low    | Medium     | Include freshness check, send last known position | Unit test with timestamp validation                  |
| Unit conversion overflow         | Medium | Low        | Use saturating arithmetic for edge cases          | Test with boundary values (90/-90 lat, 180/-180 lon) |
| COG invalid at low speed         | Low    | High       | Set vx/vy to 0 when speed < 0.5 m/s               | Unit test low speed case                             |

## Implementation Notes

- Modify `TelemetryStreamer::build_gps()` in `src/communication/mavlink/handlers/telemetry.rs`
- Add `build_global_position_int()` method to TelemetryStreamer
- GPS data source: SystemState.gps_position (to be added)
- Consider adding helper functions for unit conversion (reusable across modules)

## External References

- [MAVLink GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT)
- [MAVLink GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
