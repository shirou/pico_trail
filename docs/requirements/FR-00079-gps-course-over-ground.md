# FR-00079 GPS Course Over Ground Extraction

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00077-gps-uart-driver](../requirements/FR-00077-gps-uart-driver.md)
- Dependent Requirements:
  - [FR-00080-gps-mavlink-telemetry](../requirements/FR-00080-gps-mavlink-telemetry.md)
- Related Tasks:
  - [T-00017-gps-state-telemetry](../tasks/T-00017-gps-state-telemetry/README.md)

## Requirement Statement

The GPS driver shall extract Course Over Ground (COG) from GPRMC sentences to provide heading information for velocity decomposition and navigation.

## Rationale

GLOBAL_POSITION_INT requires velocity in North-East-Down (NED) frame (vx, vy, vz). To decompose ground speed into vx/vy components, heading or course information is needed. The NEO-M8N GPS provides Course Over Ground (track angle) in GPRMC sentences, which can be used until AHRS heading is available.

## User Story (if applicable)

As a telemetry system, I want to know the vehicle's course over ground, so that I can decompose ground speed into north and east velocity components.

## Acceptance Criteria

- [ ] GpsPosition struct includes course_over_ground field (Option<f32> in degrees)
- [ ] GPRMC sentence parsing extracts track angle (field 8)
- [ ] COG is set to None when speed < 0.5 m/s (GPS COG unreliable at low speed)
- [ ] COG is set to None when GPRMC status is 'V' (void/invalid)
- [ ] COG range is validated (0.0 to 360.0 degrees)
- [ ] Unit tests verify COG extraction from valid GPRMC sentences
- [ ] Unit tests verify COG is None for low-speed/invalid cases

## Technical Details (if applicable)

### Functional Requirement Details

**GPRMC Sentence Format:**

```
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
       │      │  │         │           │     │     │      │
       │      │  │         │           │     │     │      └─ Magnetic variation
       │      │  │         │           │     │     └─ Date (DDMMYY)
       │      │  │         │           │     └─ Track angle (COG) in degrees
       │      │  │         │           └─ Speed over ground in knots
       │      │  │         └─ Longitude
       │      │  └─ Latitude
       │      └─ Status: A=Active, V=Void
       └─ UTC Time
```

**GpsPosition Extension:**

```rust
pub struct GpsPosition {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub speed: f32,               // m/s (converted from knots)
    pub course_over_ground: Option<f32>,  // NEW: degrees (0-360), None if invalid
    pub fix_type: GpsFixType,
    pub satellites: u8,
}
```

**Velocity Decomposition (in telemetry handler):**

```rust
if let Some(cog) = gps.course_over_ground {
    let cog_rad = cog.to_radians();
    let vx = gps.speed * cog_rad.cos();  // North (m/s)
    let vy = gps.speed * cog_rad.sin();  // East (m/s)
}
```

**Low Speed Threshold:**

COG is unreliable when the vehicle is stationary or moving very slowly. The GPS receiver may report random COG values. Threshold: 0.5 m/s (approximately 1 knot).

## Platform Considerations

### Embedded (RP2350)

- COG parsing uses existing NMEA parser infrastructure
- No additional memory overhead (Option<f32> is 8 bytes)
- Trigonometric functions available via micromath crate

### Host Tests

- Test GPRMC parsing with various COG values
- Test low-speed COG invalidation
- Test invalid status handling

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                              | Validation              |
| -------------------------------- | ------ | ---------- | --------------------------------------- | ----------------------- |
| COG jumps when vehicle turns     | Low    | Medium     | Application smoothing if needed         | GPS simulation test     |
| COG undefined during first fix   | Low    | High       | COG is Option, handle None case         | Unit test initial state |
| Magnetic vs true north confusion | Medium | Low        | GPS COG is true north, document clearly | Code comments           |

## Implementation Notes

- Modify `parse_gprmc()` in `src/devices/gps.rs`
- GPRMC already parsed for speed; add COG extraction from field 8
- Speed threshold check: `if speed < 0.5 { cog = None }`
- Consider adding `parse_cog()` helper function for clarity

## External References

- [NMEA GPRMC Sentence](https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_RMC.html)
