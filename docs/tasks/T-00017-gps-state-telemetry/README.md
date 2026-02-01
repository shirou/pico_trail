# T-00017 GPS State Management and Telemetry Integration

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00022-gps-position-telemetry-and-autonomous-navigation-foundation](../../analysis/AN-00022-gps-position-telemetry-and-autonomous-navigation-foundation.md)
- Related Requirements:
  - [FR-00080-gps-mavlink-telemetry](../../requirements/FR-00080-gps-mavlink-telemetry.md)
  - [FR-00081-gps-navigation-state-access](../../requirements/FR-00081-gps-navigation-state-access.md)
  - [FR-00079-gps-course-over-ground](../../requirements/FR-00079-gps-course-over-ground.md)
  - [NFR-00068-gps-telemetry-performance](../../requirements/NFR-00068-gps-telemetry-performance.md)
  - [NFR-00067-gps-state-thread-safety](../../requirements/NFR-00067-gps-state-thread-safety.md)
- Related ADRs:
  - [ADR-00021-gps-state-management](../../adr/ADR-00021-gps-state-management.md)
- Associated Design Document:
  - [T-00017-gps-state-telemetry-design](./design.md)
- Associated Plan Document:
  - [T-00017-gps-state-telemetry-plan](./plan.md)

## Summary

Extend SystemState with GPS fields and implement real GPS data in MAVLink telemetry messages (GPS_RAW_INT, GLOBAL_POSITION_INT). Enable thread-safe GPS position sharing across telemetry, navigation, and control subsystems.

## Scope

- In scope:
  - Extend SystemState with `gps_position: Option<GpsPosition>` and `gps_timestamp_us: u64`
  - Add `course_over_ground: Option<f32>` to GpsPosition struct
  - Extract Course Over Ground (COG) from GPRMC sentences
  - Update GPS driver to set SystemState after successful NMEA parse
  - Implement GPS_RAW_INT with real GPS data in TelemetryStreamer
  - Implement GLOBAL_POSITION_INT message building
  - Unit tests for GPS state access, COG parsing, and telemetry message building
- Out of scope:
  - HDOP/VDOP extraction (future enhancement)
  - Home position management (future task)
  - AHRS heading integration (separate subsystem)
  - Lock-free alternatives (start with Mutex, optimize if needed)

## Success Metrics

- GPS_RAW_INT and GLOBAL_POSITION_INT display real GPS data in QGroundControl
- GPS state access latency < 100us (no control loop impact)
- COG correctly extracted from GPRMC when speed > 0.5 m/s
- All unit tests pass; no clippy warnings; embedded build succeeds
- Messages transmitted at SR_POSITION rate (default 5Hz)
