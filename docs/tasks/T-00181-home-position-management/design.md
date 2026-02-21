# T-00181 Home Position Management Design

## Metadata

- Type: Design
- Status: Done

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Add home position lifecycle management to the pico_trail rover: auto-set on GPS fix, disarmed refinement, on-change broadcast, GCS query support, pre-arm check, home lock, and relative altitude computation. All changes follow existing `MavlinkLoopRunner` and `CommandHandler` patterns.

## Success Metrics

- [x] Home is auto-set on first GPS 3D fix and broadcast to GCS
- [x] Home is refined at 1 Hz while disarmed (>0.5m movement, unlocked)
- [x] GCS-set home is locked and preserved from auto-update
- [x] Arming blocked with "waiting for home" when home is not set
- [x] GCS can request home via REQUEST_MESSAGE(242) and GET_HOME_POSITION(410)
- [x] GLOBAL_POSITION_INT `relative_alt` shows altitude above home
- [x] All tests pass, clippy clean, embedded build succeeds

## Background and Current State

**Working components:**

- `HomePosition` struct in `state.rs:351` with lat/lon/alt fields
- `HomePosition::from_gps()` and `HomePosition::from_command_int()`
- `SystemState.home_position: Option<HomePosition>` with `set_home()`, `set_home_to_current()`, `has_home()`
- `MAV_CMD_DO_SET_HOME` handler in `command.rs:571` via COMMAND_INT
- `build_home_position_message()` in `CommandHandler` (instance method)
- HOME_POSITION sent as part of DO_SET_HOME response
- `MavlinkLoopRunner` in `mavlink_runner.rs` with battery and GCS failsafe checks

**Gaps (from AN-00047):**

- Home is never auto-set (always `None` unless GCS sends DO_SET_HOME)
- No disarmed home refinement
- No HOME_POSITION broadcast on auto-set or disarmed update
- `handle_request_message()` does not handle message ID 242
- No `MAV_CMD_GET_HOME_POSITION` handler
- No pre-arm check for home
- No home lock mechanism
- `relative_alt` hardcoded to 0 in `telemetry.rs:308`

## Proposed Design

### 1. SystemState Changes (`crates/core/src/autopilot/state.rs`)

**Add `home_locked` field:**

```rust
pub struct SystemState {
    // ... existing fields ...
    pub home_position: Option<HomePosition>,
    pub home_locked: bool,  // NEW: GCS-set home is protected from auto-update
    // ...
}
```

Default: `false`. Set to `true` by `MAV_CMD_DO_SET_HOME`. Reset on power cycle (no persistence).

### 2. Shared `build_home_position_message()` (`crates/core/src/autopilot/state.rs`)

Move `build_home_position_message()` from `CommandHandler` (instance method) to a standalone function on `HomePosition`:

```rust
impl HomePosition {
    pub fn to_mavlink_message(&self) -> HOME_POSITION_DATA {
        HOME_POSITION_DATA {
            latitude: (self.latitude * 1e7) as i32,
            longitude: (self.longitude * 1e7) as i32,
            altitude: (self.altitude * 1000.0) as i32,
            x: 0.0, y: 0.0, z: 0.0,
            q: [f32::NAN, f32::NAN, f32::NAN, f32::NAN],
            approach_x: 0.0, approach_y: 0.0, approach_z: 0.0,
            time_usec: 0,
        }
    }
}
```

The existing `CommandHandler::build_home_position_message()` delegates to this method. This allows the control loop (MavlinkLoopRunner caller) to build HOME_POSITION without access to CommandHandler.

### 3. Home Auto-Set and Disarmed Update

Extend `MavlinkLoopRunner` with home management methods:

```rust
/// Return value for home management operations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HomeAction {
    /// No action needed
    None,
    /// Home was set or updated; caller should broadcast HOME_POSITION
    Broadcast(HomePosition),
}

impl MavlinkLoopRunner {
    /// Check if home should be auto-set on first GPS fix.
    /// Called on each control loop iteration.
    pub fn check_home_auto_set(
        &self,
        has_home: bool,
        gps_position: Option<&GpsPosition>,
        gps_fix_type: GpsFixType,
    ) -> bool {
        !has_home && gps_position.is_some() && gps_fix_type >= GpsFixType::Fix3D
    }

    /// Check if home should be updated while disarmed.
    /// Called at 1 Hz while disarmed.
    pub fn check_home_update(
        &self,
        home: &HomePosition,
        gps: &GpsPosition,
        home_locked: bool,
    ) -> bool {
        if home_locked { return false; }
        let distance = calculate_distance(
            home.latitude, home.longitude,
            gps.latitude as f32, gps.longitude as f32,
        );
        distance >= DISTANCE_HOME_MINCHANGE
    }
}

const DISTANCE_HOME_MINCHANGE: f32 = 0.5; // meters, matching ArduPilot
```

**Caller responsibility** (firmware `pico_trail_rover.rs` and SITL `autopilot.rs`):

1. Call `check_home_auto_set()` every iteration; if true, call `set_home_to_current()` in critical section, set `home_locked = false`, build and send HOME_POSITION
2. At 1 Hz while disarmed, call `check_home_update()`; if true, call `set_home_to_current()`, build and send HOME_POSITION
3. The caller already has access to both SYSTEM_STATE and the MAVLink sender (established pattern from GCS failsafe)

**Distance calculation:**

Simple flat-earth approximation (sufficient for 0.5m threshold):

```rust
fn calculate_distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    let dlat = (lat2 - lat1).to_radians() * EARTH_RADIUS;
    let dlon = (lon2 - lon1).to_radians() * EARTH_RADIUS * (lat1.to_radians().cos());
    (dlat * dlat + dlon * dlon).sqrt()
}
const EARTH_RADIUS: f32 = 6_371_000.0; // meters
```

### 4. Command Handler Changes (`crates/core/src/communication/handlers/command.rs`)

**4a. REQUEST_MESSAGE for HOME_POSITION (param1=242):**

Handle message ID 242 at the `handle_command_long` level (where extra_messages Vec is available), before delegating to `handle_request_message`:

```rust
MavCmd::MAV_CMD_REQUEST_MESSAGE => {
    let msg_id = cmd.param1 as u32;
    match msg_id {
        242 => {  // MAVLINK_MSG_ID_HOME_POSITION
            let result = critical_section::with(|cs| {
                let state = SYSTEM_STATE.borrow_ref(cs);
                state.home_position.as_ref().map(|h| h.to_mavlink_message())
            });
            if let Some(home_msg) = result {
                let mut msgs = Vec::new();
                let _ = msgs.push(MavMessage::HOME_POSITION(home_msg));
                (MavResult::MAV_RESULT_ACCEPTED, false, msgs)
            } else {
                (MavResult::MAV_RESULT_FAILED, false, Vec::new())
            }
        }
        _ => (self.handle_request_message(cmd), false, Vec::new()),
    }
}
```

**4b. MAV_CMD_GET_HOME_POSITION (command 410):**

Add new match arm in `handle_command_long`:

```rust
MavCmd::MAV_CMD_GET_HOME_POSITION => {
    let result = critical_section::with(|cs| {
        let state = SYSTEM_STATE.borrow_ref(cs);
        state.home_position.as_ref().map(|h| h.to_mavlink_message())
    });
    if let Some(home_msg) = result {
        let mut msgs = Vec::new();
        let _ = msgs.push(MavMessage::HOME_POSITION(home_msg));
        (MavResult::MAV_RESULT_ACCEPTED, false, msgs)
    } else {
        (MavResult::MAV_RESULT_FAILED, false, Vec::new())
    }
}
```

**4c. DO_SET_HOME sets `home_locked = true`:**

In `handle_set_home()`, after successfully setting home, set `state.home_locked = true`:

```rust
// Both branches (use_current and specified location):
state.set_home(...);
state.home_locked = true;  // GCS-set home is locked
```

### 5. Pre-Arm Check (`crates/core/src/arming/checks.rs`)

Add a new `HomePositionCheck` using the existing `PreArmCheck` trait:

```rust
pub struct HomePositionCheck;

static HOME_CHECK: HomePositionCheck = HomePositionCheck;

impl PreArmCheck for HomePositionCheck {
    fn check(&self, state: &SystemState) -> CheckResult {
        if state.home_position.is_none() {
            return Err(ArmingError::CheckFailed {
                reason: "waiting for home",
                category: CheckCategory::Gps,
            });
        }
        Ok(())
    }

    fn name(&self) -> &'static str { "Home Position" }
    fn category(&self) -> CheckCategory { CheckCategory::Gps }
}
```

Register in `create_default_checker()`:

```rust
let _ = checker.register(&HOME_CHECK);
```

The check is in the GPS category, so it is skippable via `ARMING_CHECK` bitmask (GPS bit = `1 << 2`).

### 6. GLOBAL_POSITION_INT `relative_alt` Fix

Update `build_global_position_int()`:

```rust
// Before (line 308):
relative_alt: 0,

// After:
relative_alt: if let Some(home) = &state.home_position {
    if let Some(gps) = &state.gps_position {
        ((gps.altitude as f32 - home.altitude) * 1000.0) as i32
    } else { 0 }
} else { 0 },
```

### Data Flow

```
Power on --> SystemState.home_position = None, home_locked = false

GPS acquires 3D fix:
    --> Control loop: check_home_auto_set() returns true
    --> critical_section: set_home_to_current(), home_locked = false
    --> Build HOME_POSITION via HomePosition::to_mavlink_message()
    --> Send HOME_POSITION on MAVLink channel
    --> log_info!("Home set to {lat}, {lon}")

While disarmed (1 Hz):
    --> check_home_update(home, gps, home_locked)
    --> If unlocked and moved >0.5m: update home, send HOME_POSITION

GCS sends MAV_CMD_DO_SET_HOME:
    --> handle_set_home(): set_home(), home_locked = true
    --> Send HOME_POSITION (already implemented)

GCS sends MAV_CMD_REQUEST_MESSAGE(param1=242):
    --> Return HOME_POSITION or MAV_RESULT_FAILED

GCS sends MAV_CMD_GET_HOME_POSITION:
    --> Return HOME_POSITION or MAV_RESULT_FAILED

Arm attempt:
    --> Pre-arm checks: HomePositionCheck fails "waiting for home" if None
    --> Home is set (from auto-set), so check passes

Telemetry (GLOBAL_POSITION_INT):
    --> relative_alt = (gps_alt - home_alt) * 1000 mm
```

## Alternatives Considered

See [AN-00047](../../analysis/AN-00047-home-position-management.md) for full analysis:

1. **Option A: Minimal** -- auto-set + broadcast only. Lacks REQUEST_MESSAGE, home lock, pre-arm check. Insufficient for full GCS compatibility
2. **Option C: Full ArduPilot parity with EKF origin** -- premature; pico_trail has no EKF

Selected approach (Option B) matches ArduPilot behavior closely while staying within pico_trail's current architecture.

## Testing Strategy

### Unit Tests (`crates/core/src/autopilot/state.rs`)

- `home_locked` defaults to `false`
- `set_home()` does not change `home_locked` (caller responsibility)
- `HomePosition::to_mavlink_message()` produces correct degE7/mm values

### Unit Tests (`crates/core/src/autopilot/mavlink_runner.rs`)

- `check_home_auto_set()`: returns true when no home + GPS 3D fix, false otherwise
- `check_home_update()`: returns true when unlocked + moved >0.5m, false when locked, false when <0.5m
- Distance calculation accuracy at various latitudes

### Unit Tests (`crates/core/src/arming/checks.rs`)

- `HomePositionCheck` passes when home is set
- `HomePositionCheck` fails with "waiting for home" when home is None
- Check is in GPS category and respects ARMING_CHECK bitmask

### Unit Tests (`crates/core/src/communication/handlers/command.rs`)

- REQUEST_MESSAGE param1=242 returns HOME_POSITION when home set
- REQUEST_MESSAGE param1=242 returns FAILED when home not set
- GET_HOME_POSITION returns HOME_POSITION when home set
- GET_HOME_POSITION returns FAILED when home not set
- DO_SET_HOME sets `home_locked = true`
- Existing REQUEST_MESSAGE tests (148, 300) still pass

### Unit Tests (`crates/core/src/communication/handlers/telemetry.rs`)

- `relative_alt` computed from home altitude when home set
- `relative_alt` is 0 when home not set (fallback)

## Open Questions

- [x] Where should `build_home_position_message()` live? --> `HomePosition::to_mavlink_message()` on the struct itself. CommandHandler delegates to it
- [x] Should distance calculation be a separate utility? --> Keep as private function in `mavlink_runner.rs` or `state.rs`. Only used for 0.5m threshold check
- [x] Should `check_home_auto_set` and `check_home_update` be on MavlinkLoopRunner or standalone functions? --> Standalone functions or methods on MavlinkLoopRunner; either works since they are stateless. Methods keep them co-located with other control loop checks
