# NFR-00034 Graceful Degradation with Partial Sensor Loss

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00054-sensor-health-monitoring](FR-00054-sensor-health-monitoring.md)
- Dependent Requirements: N/A – No dependent requirements

## Requirement Statement

Monitoring system shall support graceful degradation, allowing continued safe operation in Manual mode with reduced sensor set (RC + IMU + Battery only) when non-critical sensors (GPS, Compass) fail.

## Rationale

System reliability improved when vehicle continues operating with partial sensor failures. Manual mode operation requires only RC, IMU, and Battery - GPS and Compass are optional.

## Acceptance Criteria

- [ ] Continue Manual mode operation with RC + IMU + Battery only
- [ ] Mark GPS/Compass as non-critical for Manual mode
- [ ] Prevent mode switch to Auto/RTL if GPS unavailable
- [ ] Send warning to GCS when operating with reduced sensor set
- [ ] Log sensor unavailability events

## Technical Details (if applicable)

**Sensor Criticality**:

- **Critical** (all modes): RC, IMU, Battery
- **Non-Critical** (Manual mode): GPS, Compass
- **Critical** (Auto/RTL modes): GPS (required for navigation)

**Behavior**:

- GPS fail in Manual → Continue operation, warn operator
- GPS fail in Auto → Trigger failsafe (cannot navigate without GPS)

## External References

- Analysis: [AN-00009-armed-state-monitoring](../analysis/AN-00009-armed-state-monitoring.md)
