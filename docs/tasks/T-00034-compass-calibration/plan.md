# T-00034 Compass Calibration via Mission Planner

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-00034-compass-calibration-design](design.md)

## Overview

Implement Mission Planner's "Large Vehicle MagCal" support by adding capability advertisement and command handler. Phase 1 trusts BNO086's internal calibration and validates GPS fix before accepting commands.

## Success Metrics

- [x] Mission Planner shows enabled "Large Vehicle MagCal" button (manual verification)
- [x] Command returns ACCEPTED with valid 3D GPS fix
- [x] Command returns DENIED without GPS fix
- [x] Response time < 1 second (sync handler, no async operations)
- [x] All existing tests pass; no regressions in MAVLink handling (178 tests passed)

## Scope

- Goal: Enable Mission Planner compass calibration UI and handle calibration commands
- Non-Goals: WMM implementation, hard iron offset calculation (Phase 2)
- Assumptions: BNO086 internal calibration is sufficient for Phase 1
- Constraints: No additional ROM overhead for WMM tables

## ADR & Legacy Alignment

- [x] No ADRs required - simple implementation
- [x] No legacy patterns to migrate

## Plan Summary

- Phase 1 – Add COMPASS_CALIBRATION capability flag
- Phase 2 – Implement MAV_CMD_FIXED_MAG_CAL_YAW handler
- Phase 3 – Testing and verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Add Capability Flag

### Goal

- Add `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` to AUTOPILOT_VERSION response

### Inputs

- Source Code to Modify:
  - `src/communication/mavlink/handlers/command.rs` – AUTOPILOT_VERSION handler (line \~699)

### Tasks

- [x] **Add capability flag**
  - [x] Locate `handle_request_autopilot_capabilities()` function
  - [x] Add `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` to capabilities bitmask
  - [x] Verify capability value 4096 (0x1000) is correctly OR'd

### Deliverables

- Modified `command.rs` with new capability flag

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet mavlink
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Capability flag compiles without errors
- Existing MAVLink tests pass
- RP2350 build succeeds

### Rollback/Fallback

- Remove the single line if issues arise

---

## Phase 2: Implement Command Handler

### Phase 2 Goal

- Implement `MAV_CMD_FIXED_MAG_CAL_YAW` command handler with GPS validation

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Capability flag added
  - GPS state access via `SystemState`
  - STATUSTEXT via `status_notifier` module
- Source Code to Modify:
  - `src/communication/mavlink/handlers/command.rs` – Command dispatcher

### Phase 2 Tasks

- [x] **Add command case to dispatcher**
  - [x] Add `MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW` match arm in `handle_command_long()`
  - [x] Route to new handler function
- [x] **Implement handler function**
  - [x] Create `handle_fixed_mag_cal_yaw(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult`
  - [x] Check GPS fix availability via `self.state.gps_position()`
  - [x] Validate fix type >= GpsFixType::Fix3D
  - [x] Return DENIED with warning STATUSTEXT if no GPS
  - [x] Return ACCEPTED with info STATUSTEXT if GPS available
  - [x] Log yaw angle for debugging

### Phase 2 Deliverables

- `handle_fixed_mag_cal_yaw()` function
- Command dispatcher integration

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet mavlink
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Command handler compiles without errors
- Handler returns correct MavResult based on GPS state
- STATUSTEXT messages are sent

### Phase 2 Rollback/Fallback

- Remove command case and handler function

---

## Phase 3: Testing & Verification

### Phase 3 Goal

- Add unit tests and verify integration with Mission Planner

### Phase 3 Tasks

- [x] **Unit tests**
  - [x] Test capability flag is set in AUTOPILOT_VERSION
  - [x] Test handler returns ACCEPTED with GPS fix
  - [x] Test handler returns DENIED without GPS fix
  - [x] Test handler returns DENIED with 2D GPS fix (additional)
- [x] **Integration verification** (manual testing required)
  - [x] Connect Mission Planner to pico_trail
  - [x] Verify "Large Vehicle MagCal" button is enabled
  - [x] Execute calibration with GPS fix, verify ACCEPTED
  - [x] Verify STATUSTEXT appears in Messages tab

### Phase 3 Deliverables

- Unit tests in `command.rs`
- Integration test documentation

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All unit tests pass
- Mission Planner integration verified
- No regressions in existing functionality

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Mission Planner shows enabled calibration button (manual verification)
- [x] Command returns correct result based on GPS state
- [x] STATUSTEXT messages displayed in GCS
- [x] No `unsafe` and no vague naming

## Open Questions

- [x] Does Mission Planner require specific response beyond COMMAND_ACK? → **Answer:** Yes, Mission Planner requires `COMPASS_OFS_X` parameter to exist, and SYS_STATUS must include `MAV_SYS_STATUS_SENSOR_3D_MAG` flag. Additionally, the yaw offset must be applied to ATTITUDE and GLOBAL_POSITION_INT heading values for the calibration to have visible effect.
