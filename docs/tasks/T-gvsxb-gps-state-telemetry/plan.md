# T-gvsxb GPS State Management and Telemetry Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-gvsxb-gps-state-telemetry-design](./design.md)

## Overview

Implement GPS state sharing via SystemState and real GPS telemetry messages. This enables GCS to display vehicle position and provides GPS data access for navigation subsystem.

## Success Metrics

- [ ] GPS_RAW_INT displays real position in QGroundControl
- [ ] GLOBAL_POSITION_INT displays real position and velocity
- [ ] GPS state access latency < 100us
- [ ] COG correctly parsed when speed > 0.5 m/s
- [ ] All existing tests pass; no regressions

## Scope

- Goal: Thread-safe GPS state sharing and real GPS telemetry messages
- Non-Goals: HDOP/VDOP, home position, AHRS heading, lock-free optimization
- Assumptions: GPS driver works correctly (T-vxtxn completed)
- Constraints: Embassy async, no heap allocation, single Mutex pattern

## ADR & Legacy Alignment

- [x] ADR-xqqbl (GPS State Management Pattern) governs this work
- [ ] No legacy patterns to retire

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
  - `docs/requirements/FR-4z22l-gps-course-over-ground.md` - COG requirements
  - `docs/adr/ADR-xqqbl-gps-state-management.md` - State pattern decision
- Source Code to Modify:
  - `src/devices/gps.rs` - GpsPosition struct, GPRMC parsing
  - `src/communication/mavlink/state.rs` - SystemState struct
- Dependencies:
  - Internal: `src/devices/gps.rs` - existing GpsPosition
  - External crates: None

### Tasks

- [ ] **Extend GpsPosition struct**
  - [ ] Add `course_over_ground: Option<f32>` field to GpsPosition
  - [ ] Update Default impl to set course_over_ground to None
  - [ ] Update any existing GpsPosition constructors

- [ ] **Extract COG from GPRMC sentences**
  - [ ] Modify `parse_gprmc()` to extract track angle (field 8)
  - [ ] Set COG to None when speed < 0.5 m/s (unreliable at low speed)
  - [ ] Set COG to None when GPRMC status is 'V' (void/invalid)
  - [ ] Validate COG range (0.0 to 360.0 degrees)

- [ ] **Extend SystemState struct**
  - [ ] Add `gps_position: Option<GpsPosition>` field
  - [ ] Add `gps_timestamp_us: u64` field
  - [ ] Update Default impl to set gps_position to None, timestamp to 0

- [ ] **Add GPS state access helpers**
  - [ ] Add `is_gps_fresh()` method to check timestamp age
  - [ ] Add `update_gps_state()` helper function for GPS driver

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

- [ ] **Integrate state update in GpsOperationManager**
  - [ ] Import SYSTEM_STATE from mavlink/state.rs
  - [ ] After successful position read, call state update
  - [ ] Set gps_timestamp_us to current uptime

- [ ] **Handle state update failures gracefully**
  - [ ] Use try_lock if needed for non-blocking update
  - [ ] Log warning if lock acquisition fails (should not happen)

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

- [ ] **Add unit conversion helpers**
  - [ ] Add `degrees_to_deg_e7()` - convert f32 degrees to i32 degE7
  - [ ] Add `meters_to_mm()` - convert f32 meters to i32 mm
  - [ ] Add `mps_to_cms()` - convert f32 m/s to u16 cm/s
  - [ ] Add `degrees_to_cdeg()` - convert f32 degrees to u16 cdeg

- [ ] **Update build_gps() for GPS_RAW_INT**
  - [ ] Read GPS position from SYSTEM_STATE
  - [ ] Convert lat/lon to degE7 format
  - [ ] Convert altitude to mm
  - [ ] Convert speed to cm/s
  - [ ] Convert COG to cdeg (or 0 if None)
  - [ ] Map fix_type to MAVLink GPS_FIX_TYPE enum
  - [ ] Set eph/epv to 9999 (unknown, until HDOP added)

- [ ] **Implement build_global_position_int()**
  - [ ] Read GPS position from SYSTEM_STATE
  - [ ] Convert lat/lon/alt same as GPS_RAW_INT
  - [ ] Set relative_alt to 0 (home not implemented)
  - [ ] Calculate vx = speed \* cos(cog) when COG valid
  - [ ] Calculate vy = speed \* sin(cog) when COG valid
  - [ ] Set vz to 0 (vertical rate unreliable from GPS)
  - [ ] Set hdg from COG or UINT16_MAX if unknown

- [ ] **Handle edge cases**
  - [ ] GPS no fix: send with fix_type=0, position zeros
  - [ ] No COG (speed < 0.5 m/s): set vx=0, vy=0, hdg=UINT16_MAX
  - [ ] Use saturating arithmetic for overflow protection

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

- [ ] **Unit tests for COG parsing**
  - [ ] Test valid GPRMC with COG extraction
  - [ ] Test low speed COG invalidation (< 0.5 m/s)
  - [ ] Test void status COG invalidation
  - [ ] Test COG range validation

- [ ] **Unit tests for unit conversions**
  - [ ] Test degE7 conversion with boundary values (90/-90, 180/-180)
  - [ ] Test mm conversion with typical altitude values
  - [ ] Test cm/s conversion with various speeds
  - [ ] Test cdeg conversion with 0, 180, 359.99 degrees

- [ ] **Unit tests for telemetry messages**
  - [ ] Test GPS_RAW_INT with mock GPS data
  - [ ] Test GLOBAL_POSITION_INT velocity decomposition
  - [ ] Test no-fix case (zeros)
  - [ ] Test no-COG case (vx=0, vy=0)

- [ ] **Integration verification**
  - [ ] Verify embedded build succeeds
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

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Traceability updated (`bun scripts/trace-status.ts --write`)
- [ ] No `unsafe` and no vague naming

## Open Questions

- [ ] Should we add HDOP/VDOP to GpsPosition? → Defer to future task (use 9999 for now)
- [ ] Should we implement home position for relative_alt? → Defer to future task (use 0 for now)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
