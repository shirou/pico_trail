# T-00034 Compass Calibration via Mission Planner

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [T-00034-compass-calibration-plan](plan.md)

## Overview

Enable Mission Planner's "Large Vehicle MagCal" feature for pico_trail rovers. This Phase 1 implementation adds the `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` capability flag and implements a `MAV_CMD_FIXED_MAG_CAL_YAW` command handler that trusts the BNO086's internal magnetometer calibration. The handler validates GPS availability before accepting the calibration command.

## Success Metrics

- [x] Mission Planner shows enabled "Large Vehicle MagCal" button when connected
- [x] Command returns `MAV_RESULT_ACCEPTED` with valid GPS fix
- [x] Command returns `MAV_RESULT_DENIED` without GPS fix
- [x] Response time < 1 second from command to ACK
- [x] STATUSTEXT message appears in Mission Planner console

## Background and Current State

- Context: pico_trail uses BNO086 for AHRS, which has internal auto-calibration. Mission Planner's compass calibration UI is currently disabled because we don't advertise the capability.
- Current behavior: `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` is not set in AUTOPILOT_VERSION. Command handler does not recognize `MAV_CMD_FIXED_MAG_CAL_YAW`.
- Pain points: Users cannot verify compass calibration status via Mission Planner. UI shows disabled calibration buttons.
- Constraints: Phase 1 trusts BNO086 internal calibration. WMM implementation deferred to Phase 2.
- Related ADRs: None - simple implementation with no architectural decisions.

## Proposed Design

### High-Level Architecture

```text
Mission Planner                              pico_trail
      │                                           │
      │  AUTOPILOT_VERSION request                │
      ├──────────────────────────────────────────>│
      │                                           │
      │  AUTOPILOT_VERSION                        │
      │  capabilities: COMPASS_CALIBRATION        │
      │<──────────────────────────────────────────┤
      │                                           │
      │  [UI enables "Large Vehicle MagCal"]      │
      │                                           │
      │  MAV_CMD_FIXED_MAG_CAL_YAW               │
      │  param1: yaw_deg                          │
      ├──────────────────────────────────────────>│
      │                                           │  ┌─────────────────────┐
      │                                           │──│ Check GPS fix       │
      │                                           │  │ (GpsFixType >= 3D)  │
      │                                           │  └─────────────────────┘
      │                                           │           │
      │  COMMAND_ACK (ACCEPTED/DENIED)            │<──────────┘
      │<──────────────────────────────────────────┤
      │                                           │
      │  STATUSTEXT "Mag cal accepted/failed"     │
      │<──────────────────────────────────────────┤
```

### Components

- `src/communication/mavlink/handlers/command.rs`:
  - Add `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` to capabilities bitmask
  - Add `MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW` case to command handler
  - Implement `handle_fixed_mag_cal_yaw()` function

### Data Flow

1. Mission Planner requests AUTOPILOT_VERSION
2. pico_trail responds with capabilities including COMPASS_CALIBRATION
3. Mission Planner enables calibration UI
4. User clicks "Large Vehicle MagCal" and enters heading
5. Mission Planner sends `MAV_CMD_FIXED_MAG_CAL_YAW` with yaw angle
6. pico_trail checks GPS fix via `SystemState`
7. pico_trail returns COMMAND_ACK (ACCEPTED or DENIED)
8. pico_trail sends STATUSTEXT with result

### Data Models and Types

**Command Parameters:**

```rust
// MAV_CMD_FIXED_MAG_CAL_YAW (42006)
struct FixedMagCalYawParams {
    yaw_deg: f32,      // param1: Vehicle yaw in earth frame (0-360)
    compass_mask: f32, // param2: 0 = all compasses
    latitude: f32,     // param3: 0 = use current GPS
    longitude: f32,    // param4: 0 = use current GPS
}
```

**GPS Fix Check:**

```rust
use crate::devices::gps::GpsFixType;

fn has_valid_gps_fix(state: &SystemState) -> bool {
    state.gps_position()
        .map(|pos| pos.fix_type >= GpsFixType::Fix3D)
        .unwrap_or(false)
}
```

### Error Handling

- No GPS fix: Return `MAV_RESULT_DENIED`, send STATUSTEXT warning
- GPS fix available: Return `MAV_RESULT_ACCEPTED`, send STATUSTEXT info
- All messages in English following ArduPilot conventions

### Security Considerations

- No security implications - calibration command only affects compass offset storage
- GPS validation prevents calibration in unknown location

### Performance Considerations

- Simple GPS state check - no complex calculations
- Target response time < 1 second (actual \~50ms expected)
- No heap allocations in hot path

### Platform Considerations

#### Cross-Platform

- MAVLink message handling is platform-agnostic
- GPS state access via shared `SystemState`
- Same implementation on RP2350 and host tests

## Alternatives Considered

1. **Full WMM Implementation (Phase 2)**
   - Pros: Accurate hard iron offset calculation
   - Cons: Requires WMM tables (\~15KB ROM), more complex

2. **Reject All Calibration Commands**
   - Pros: No implementation needed
   - Cons: Mission Planner UI remains disabled

**Decision Rationale**: Phase 1 approach enables the UI with minimal implementation. BNO086's internal calibration is generally sufficient. Phase 2 (FR-00102) can be implemented later if field testing shows issues.

## Migration and Compatibility

- Backward compatibility: No changes to existing behavior
- Rollout plan: Feature enabled immediately upon deployment
- Deprecation plan: N/A

## Testing Strategy

### Unit Tests

- Test capability flag is set correctly
- Test command handler returns ACCEPTED with GPS fix
- Test command handler returns DENIED without GPS fix
- Test STATUSTEXT messages are queued

### Integration Tests

- Connect Mission Planner and verify calibration UI is enabled
- Execute calibration command and verify response

## Documentation Impact

- Update analysis document AN-00038 with task link
- Update requirements with task link

## External References

- [MAV_CMD_FIXED_MAG_CAL_YAW](https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW)
- [MAV_PROTOCOL_CAPABILITY](https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY)
- [Large Vehicle MagCal documentation](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html#large-vehicle-magcal)

## Open Questions

- [x] Is MAV_CMD_FIXED_MAG_CAL_YAW in common.xml? Yes, ID 42006
- [x] Is COMPASS_CALIBRATION capability in common.xml? Yes, value 4096
- [ ] Does Mission Planner require specific response beyond COMMAND_ACK? To be verified in testing
