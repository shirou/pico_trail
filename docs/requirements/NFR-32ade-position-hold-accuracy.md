# NFR-32ade Position Hold Accuracy

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-aw3h3-rover-loiter-mode](FR-aw3h3-rover-loiter-mode.md)
  - [FR-9s9th-position-correction-navigation](FR-9s9th-position-correction-navigation.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-n24yy-rover-loiter-mode](../tasks/T-n24yy-rover-loiter-mode/README.md)

## Requirement Statement

The Loiter mode position hold accuracy shall be within LOIT_RADIUS plus GPS horizontal accuracy (CEP), typically achieving 2-5 meters with consumer-grade GPS.

## Rationale

Position hold accuracy is fundamentally limited by GPS accuracy. Setting accuracy expectations correctly prevents users from configuring unrealistically small LOIT_RADIUS values that would cause constant correction attempts due to GPS noise rather than actual drift.

## User Story

The system shall achieve position hold accuracy appropriate to the GPS hardware quality to ensure stable operation without excessive corrections caused by measurement noise.

## Acceptance Criteria

- [ ] Position maintained within LOIT_RADIUS + GPS_HDOP \* 2 meters (practical bound)
- [ ] No correction oscillation when stationary with good GPS fix
- [ ] Minimum recommended LOIT_RADIUS documented as 2m (default)
- [ ] Position hold logged for post-flight accuracy analysis
- [ ] Type 1 correction reduces drift below LOIT_RADIUS after stabilization

## Technical Details

### Non-Functional Requirement Details

**Accuracy Factors:**

| Factor         | Typical Value | Notes                             |
| -------------- | ------------- | --------------------------------- |
| GPS CEP (50%)  | 1-3m          | Horizontal position accuracy      |
| LOIT_RADIUS    | 2m (default)  | Configurable threshold            |
| Total Expected | 3-5m          | LOIT_RADIUS + GPS noise           |
| RTK GPS        | 0.1-0.5m      | With RTK, smaller radius possible |

**GPS Accuracy Integration:**

```rust
fn calculate_effective_radius(loit_radius: f32, gps_hdop: f32) -> f32 {
    // HDOP * 2.5 gives approximate horizontal accuracy in meters
    let gps_accuracy = gps_hdop * 2.5;

    // Effective radius is the larger of configured radius or GPS accuracy
    loit_radius.max(gps_accuracy)
}
```

**Accuracy Validation:**

Post-flight analysis can verify accuracy:

1. Log position samples during Loiter mode
2. Calculate distance from loiter point for each sample
3. 95th percentile should be < LOIT_RADIUS + 2 \* GPS_accuracy

**User Guidance:**

- LOIT_RADIUS < 2m: May cause oscillation with consumer GPS
- LOIT_RADIUS 2-5m: Good balance for typical use
- LOIT_RADIUS > 10m: Very loose hold, mainly useful for boats

## Platform Considerations

N/A - Platform agnostic (depends on GPS hardware, not processor)

## Risks & Mitigation

| Risk                              | Impact | Likelihood | Mitigation                      | Validation                   |
| --------------------------------- | ------ | ---------- | ------------------------------- | ---------------------------- |
| GPS multipath causes large errors | Medium | Medium     | Clear sky operation recommended | Test in various environments |
| User sets LOIT_RADIUS too small   | Low    | Medium     | Document minimum, warn in GCS   | User education               |
| GPS accuracy varies over time     | Low    | High       | Use HDOP for adaptive threshold | Monitor HDOP in telemetry    |

## Implementation Notes

- Consider warning user if LOIT_RADIUS < 2m (via STATUSTEXT)
- Log GPS HDOP alongside position during Loiter for accuracy analysis
- Future enhancement: adaptive LOIT_RADIUS based on GPS quality
- RTK GPS users can safely use LOIT_RADIUS down to 0.5m

## External References

- [GPS Accuracy](https://www.gps.gov/systems/gps/performance/accuracy/) - GPS accuracy specifications
- [ArduPilot GPS](https://ardupilot.org/rover/docs/common-positioning-landing-page.html) - GPS configuration guidance
