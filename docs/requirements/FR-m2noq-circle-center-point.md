# FR-m2noq Circle Center Point Determination

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-8dqwi-circle-mode](../analysis/AN-8dqwi-circle-mode.md)
- Prerequisite Requirements:
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-eyuh8-ahrs-attitude-estimation](FR-eyuh8-ahrs-attitude-estimation.md)
- Dependent Requirements:
  - [FR-khjpl-circle-mode-implementation](FR-khjpl-circle-mode-implementation.md)
- Related ADRs:
  - [ADR-897ov-circle-mode-path-generation](../adr/ADR-897ov-circle-mode-path-generation.md)
- Related Tasks:
  - [T-u8q60-circle-mode-implementation](../tasks/T-u8q60-circle-mode-implementation/README.md)

## Requirement Statement

The system shall determine the circle center point on Circle mode entry by offsetting CIRC_RADIUS meters in front of the current vehicle position in the direction of the current heading.

## Rationale

ArduPilot standard behavior places the circle center ahead of the vehicle when entering Circle mode:

- Vehicle immediately begins curving toward the circle perimeter
- Intuitive behavior: "circle around what's in front of me"
- Avoids requiring the vehicle to turn around to reach the circle
- Compatible with GCS expectations

This differs from a "circle around current position" approach, which would require the vehicle to first move outward to reach the perimeter.

## User Story (if applicable)

As a rover operator, I want the circle center to be placed in front of my vehicle when I engage Circle mode, so that the vehicle begins orbiting immediately without needing to travel to a distant perimeter.

## Acceptance Criteria

- [ ] Center point calculated as CIRC_RADIUS meters ahead of current position
- [ ] Current heading used for offset direction (from AHRS or GPS COG)
- [ ] Center point stored and maintained throughout Circle mode operation
- [ ] Center point reported in telemetry (future: custom message)
- [ ] Center point calculation uses geodetic (great-circle) offset
- [ ] Invalid heading (no GPS COG, AHRS not ready) prevents mode entry

## Technical Details (if applicable)

### Functional Requirement Details

**Center Point Calculation:**

```rust
fn calculate_center_on_entry(current: &GpsPosition, heading_deg: f32, radius: f32) -> GpsPosition {
    // Offset center point in front of vehicle
    offset_position(current, radius, heading_deg)
}

fn offset_position(origin: &GpsPosition, distance: f32, bearing_deg: f32) -> GpsPosition {
    const EARTH_RADIUS: f32 = 6_371_000.0;

    let bearing_rad = bearing_deg.to_radians();
    let lat1 = origin.latitude.to_radians();
    let lon1 = origin.longitude.to_radians();
    let angular_distance = distance / EARTH_RADIUS;

    let lat2 = (lat1.sin() * angular_distance.cos()
        + lat1.cos() * angular_distance.sin() * bearing_rad.cos())
        .asin();

    let lon2 = lon1
        + (bearing_rad.sin() * angular_distance.sin() * lat1.cos())
            .atan2(angular_distance.cos() - lat1.sin() * lat2.sin());

    GpsPosition {
        latitude: lat2.to_degrees(),
        longitude: lon2.to_degrees(),
        altitude: origin.altitude,
    }
}
```

**Heading Source Priority:**

1. AHRS heading (if available and valid)
2. GPS Course Over Ground (COG) if vehicle is moving
3. Reject mode entry if no valid heading available

**Validation Constraints:**

- Heading must be between 0° and 360°
- GPS position must have 3D fix
- CIRC_RADIUS must be > 0 (0 = stationary mode, no center offset needed)

## Platform Considerations

### Pico W (RP2040)

Trigonometric calculations for offset may need optimization. Consider using lookup tables if CPU-bound.

### Pico 2 W (RP2350)

Hardware FPU handles trigonometric calculations efficiently.

### Cross-Platform

Geodetic offset calculation is identical across platforms.

## Risks & Mitigation

| Risk                         | Impact | Likelihood | Mitigation                           | Validation                   |
| ---------------------------- | ------ | ---------- | ------------------------------------ | ---------------------------- |
| Heading source unavailable   | Medium | Low        | Require valid heading for mode entry | Test entry without GPS fix   |
| Offset calculation error     | High   | Low        | Unit test with known coordinates     | Compare against online tools |
| Large radius causes overflow | Low    | Low        | Limit CIRC_RADIUS to reasonable max  | Test with extreme values     |

## Implementation Notes

Preferred approaches:

- Calculate center once on mode entry, store in CircleMode state
- Reuse geodetic utilities from navigation subsystem
- Prefer AHRS heading over GPS COG when available

Known pitfalls:

- GPS COG is invalid when vehicle is stationary (speed < threshold)
- Ensure heading is in degrees, not radians
- Longitude offset must account for latitude (cosine correction)

Related code areas:

- `src/rover/mode/circle.rs` - Circle mode entry
- `src/core/math/geodetic.rs` - Geodetic calculations
- `src/subsystems/ahrs/` - Heading source

## External References

- [ArduPilot Circle Mode (Rover)](https://ardupilot.org/rover/docs/circle-mode.html) - "Circle will orbit a point located CIRC_RADIUS meters in front of the vehicle"
- [Movable Type Scripts - Destination Point](https://www.movable-type.co.uk/scripts/latlong.html) - Geodetic calculations

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
