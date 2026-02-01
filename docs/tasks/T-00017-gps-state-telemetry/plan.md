# T-00017 GPS State Management and Telemetry Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-00017-gps-state-telemetry-design](./design.md)

## Overview

Implement GPS state sharing via SystemState and real GPS telemetry messages. This enables GCS to display vehicle position and provides GPS data access for navigation subsystem.

## Success Metrics

- [x] GPS_RAW_INT displays real position in QGroundControl
- [x] GLOBAL_POSITION_INT displays real position and velocity
- [x] GPS state access latency < 100us
- [x] COG correctly parsed when speed > 0.5 m/s
- [x] All existing tests pass; no regressions

## Scope

- Goal: Thread-safe GPS state sharing and real GPS telemetry messages
- Non-Goals: HDOP/VDOP, home position, AHRS heading, lock-free optimization
- Assumptions: GPS driver works correctly (T-00016 completed)
- Constraints: Embassy async, no heap allocation, single Mutex pattern

## ADR & Legacy Alignment

- [x] ADR-00021 (GPS State Management Pattern) governs this work
- [x] No legacy patterns to retire

## Plan Summary

- Phase 1 - GPS data model extension (GpsPosition.course_over_ground, SystemState GPS fields)
- Phase 2 - GPS driver integration (state update after NMEA parse)
- Phase 3 - Telemetry implementation (GPS_RAW_INT, GLOBAL_POSITION_INT)
- Phase 4 - Testing and verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: GPS Data Model Extension

### Goal

- Extend GpsPosition with course_over_ground field
- Extend SystemState with GPS position and timestamp fields
- Add COG extraction from GPRMC sentences

### Inputs

- Documentation:
  - `docs/requirements/FR-00079-gps-course-over-ground.md` - COG requirements
  - `docs/adr/ADR-00021-gps-state-management.md` - State pattern decision
- Source Code to Modify:
  - `src/devices/gps.rs` - GpsPosition struct, GPRMC parsing
  - `src/communication/mavlink/state.rs` - SystemState struct
- Dependencies:
  - Internal: `src/devices/gps.rs` - existing GpsPosition
  - External crates: None

### Tasks

- [x] **Extend GpsPosition struct**
  - [x] Add `course_over_ground: Option<f32>` field to GpsPosition
  - [x] Update Default impl to set course_over_ground to None
  - [x] Update any existing GpsPosition constructors

- [x] **Extract COG from GPRMC sentences**
  - [x] Modify `parse_gprmc()` to extract track angle (field 8)
  - [x] Set COG to None when speed < 0.5 m/s (unreliable at low speed)
  - [x] Set COG to None when GPRMC status is 'V' (void/invalid)
  - [x] Validate COG range (0.0 to 360.0 degrees)

- [x] **Extend SystemState struct**
  - [x] Add `gps_position: Option<GpsPosition>` field
  - [x] Add `gps_timestamp_us: u64` field
  - [x] Update Default impl to set gps_position to None, timestamp to 0

- [x] **Add GPS state access helpers**
  - [x] Add `is_gps_fresh()` method to check timestamp age
  - [x] Add `update_gps()` helper method for GPS driver

### Deliverables

- Extended GpsPosition with COG field
- Extended SystemState with GPS fields
- COG parsing in GPRMC handler

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet gps
```

### Acceptance Criteria (Phase Gate)

- GpsPosition has course_over_ground field
- SystemState has gps_position and gps_timestamp_us fields
- GPRMC parsing extracts COG correctly
- Host build and tests pass

### Rollback/Fallback

- Revert struct changes if compilation fails
- Keep COG as Option to handle missing data gracefully

---

## Phase 2: GPS Driver Integration

### Phase 2 Goal

- Update GPS driver to set SystemState after successful NMEA parse
- Ensure thread-safe state updates

### Phase 2 Inputs

- Dependencies:
  - Phase 1: GpsPosition and SystemState extensions
  - `src/devices/gps_operation.rs` - GpsOperationManager
- Source Code to Modify:
  - `src/devices/gps_operation.rs` - state update call

### Phase 2 Tasks

- [x] **Integrate state update in GpsOperationManager**
  - [x] Import SYSTEM_STATE from mavlink/state.rs
  - [x] After successful position read, call state update
  - [x] Set gps_timestamp_us to current uptime

- [x] **Handle state update failures gracefully**
  - [x] Use critical_section for thread-safe update
  - [x] Log info when fix acquired

### Phase 2 Deliverables

- GPS driver updates SystemState on successful NMEA parse
- Timestamp set for freshness validation

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet gps
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- GPS driver compiles with state update
- Embedded build succeeds
- No mutex deadlock (single writer pattern)

### Phase 2 Rollback/Fallback

- Comment out state update if integration issues arise
- Verify mutex access pattern is correct

---

## Phase 3: Telemetry Implementation

### Phase 3 Goal

- Implement GPS_RAW_INT with real GPS data
- Implement GLOBAL_POSITION_INT message building
- Add unit conversion helpers

### Phase 3 Inputs

- Dependencies:
  - Phase 2: GPS state updates working
  - `src/communication/mavlink/handlers/telemetry.rs` - TelemetryStreamer
- Source Code to Modify:
  - `src/communication/mavlink/handlers/telemetry.rs` - message builders

### Phase 3 Tasks

- [x] **Add unit conversion helpers**
  - [x] Add `degrees_to_deg_e7()` - convert f32 degrees to i32 degE7
  - [x] Add `meters_to_mm()` - convert f32 meters to i32 mm
  - [x] Add `mps_to_cms()` - convert f32 m/s to u16 cm/s
  - [x] Add `degrees_to_cdeg()` - convert f32 degrees to u16 cdeg

- [x] **Update build_gps() for GPS_RAW_INT**
  - [x] Read GPS position from SystemState
  - [x] Convert lat/lon to degE7 format
  - [x] Convert altitude to mm
  - [x] Convert speed to cm/s
  - [x] Convert COG to cdeg (or u16::MAX if None)
  - [x] Map fix_type to MAVLink GPS_FIX_TYPE enum
  - [x] Set eph/epv to 9999 (unknown, until HDOP added)

- [x] **Implement build_global_position_int()**
  - [x] Read GPS position from SystemState
  - [x] Convert lat/lon/alt same as GPS_RAW_INT
  - [x] Set relative_alt to 0 (home not implemented)
  - [x] Calculate vx = speed \* cos(cog) when COG valid
  - [x] Calculate vy = speed \* sin(cog) when COG valid
  - [x] Set vz to 0 (vertical rate unreliable from GPS)
  - [x] Set hdg from COG or UINT16_MAX if unknown

- [x] **Handle edge cases**
  - [x] GPS no fix: send with fix_type=0, position zeros
  - [x] No COG (speed < 0.5 m/s): set vx=0, vy=0, hdg=UINT16_MAX
  - [x] Use clamping for overflow protection

### Phase 3 Deliverables

- GPS_RAW_INT with real GPS data
- GLOBAL_POSITION_INT with position and velocity
- Unit conversion helper functions

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet telemetry
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- GPS_RAW_INT builds with real position data
- GLOBAL_POSITION_INT builds with velocity decomposition
- Unit conversions handle boundary values correctly
- Embedded build succeeds

### Phase 3 Rollback/Fallback

- Revert to placeholder values if conversion issues
- Add fallback for missing COG

---

## Phase 4: Testing and Verification

### Phase 4 Goal

- Comprehensive unit tests for new functionality
- Verify integration end-to-end

### Phase 4 Tasks

- [x] **Unit tests for COG parsing**
  - [x] Test valid GPRMC with COG extraction
  - [x] Test low speed COG invalidation (< 0.5 m/s)
  - [x] Test threshold speed COG validation (>= 0.5 m/s)
  - [x] Test GPGGA without COG (correctly None)

- [x] **Unit tests for unit conversions**
  - [x] Test degE7 conversion with boundary values (90/-90, 180/-180)
  - [x] Test mm conversion with typical altitude values
  - [x] Test cm/s conversion with various speeds
  - [x] Test cdeg conversion with 0, 180, 359.99 degrees

- [x] **Unit tests for telemetry messages**
  - [x] Test GPS_RAW_INT with mock GPS data
  - [x] Test GLOBAL_POSITION_INT velocity decomposition
  - [x] Test no-fix case (zeros)
  - [x] Test no-COG case (vx=0, vy=0, hdg=UINT16_MAX)

- [x] **Integration verification**
  - [x] Verify embedded build succeeds
  - [ ] Test with hardware if available (deferred)

### Phase 4 Deliverables

- Comprehensive unit test coverage for GPS telemetry
- All tests passing

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- All unit tests pass
- No clippy warnings
- Embedded build succeeds
- Documentation updated

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Traceability updated (`bun scripts/trace-status.ts --write`)
- [x] No `unsafe` and no vague naming

## Open Questions

- [ ] Should we add HDOP/VDOP to GpsPosition? → Defer to future task (use 9999 for now)
- [ ] Should we implement home position for relative_alt? → Defer to future task (use 0 for now)
