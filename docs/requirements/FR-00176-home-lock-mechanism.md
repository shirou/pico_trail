# FR-00176 Home Lock Mechanism

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
- Dependent Requirements:
  - [FR-00172-home-update-disarmed](FR-00172-home-update-disarmed.md)
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall support a home lock mechanism that distinguishes between auto-set (unlocked) and GCS-set (locked) home positions. A locked home position shall not be overwritten by automatic home updates while disarmed. The lock shall reset on power cycle.

## Rationale

ArduPilot distinguishes "locked" (GCS-set) vs "unlocked" (auto-set) home. `update_home()` skips locked home via `home_is_locked()` check (`Rover/commands.cpp:44`). This prevents the 1 Hz disarmed refinement from overwriting a home position that the operator intentionally set to a specific location via `MAV_CMD_DO_SET_HOME`. Without this mechanism, a GCS-set home would drift to the vehicle's current position within seconds.

## User Story

As an operator, I want my manually-set home position to be preserved and not overwritten by automatic GPS updates, so that I can designate a specific landing point that differs from the vehicle's current location.

## Acceptance Criteria

- [ ] Add `home_locked: bool` field to `SystemState` (default `false`)
- [ ] `MAV_CMD_DO_SET_HOME` sets `home_locked = true`
- [ ] Auto-set on GPS fix (FR-00171) sets `home_locked = false`
- [ ] Disarmed home update (FR-00172) skips if `home_locked == true`
- [ ] Lock resets on power cycle (no persistence needed; default `false`)
- [ ] No API exists to unlock home explicitly (matches ArduPilot behavior)

## Technical Details

### Functional Requirement Details

**SystemState changes:**

```rust
pub struct SystemState {
    // ... existing fields ...
    pub home_position: Option<HomePosition>,
    pub home_locked: bool,  // NEW: GCS-set home is locked
}

impl SystemState {
    pub fn new() -> Self {
        Self {
            // ...
            home_position: None,
            home_locked: false,
        }
    }
}
```

**Lock behavior matrix:**

| Action                    | Sets home_locked | Behavior                               |
| ------------------------- | ---------------- | -------------------------------------- |
| Auto-set on GPS fix       | `false`          | Home is unlocked, can be refined       |
| Disarmed 1 Hz update      | `false`          | Home is unlocked, can be refined again |
| GCS `MAV_CMD_DO_SET_HOME` | `true`           | Home is locked, disarmed update skips  |
| Power cycle               | `false` (reset)  | Lock is not persisted                  |

**ArduPilot reference:**

- `ahrs.lock_home()` at `AP_AHRS.cpp:3078` sets the lock
- `home_is_locked()` at `Rover/commands.cpp:44` checked before update
- No unlock API exists -- lock resets only on power cycle

**Memory impact:**

| Component           | RAM Usage | Notes               |
| ------------------- | --------- | ------------------- |
| `home_locked: bool` | 1 byte    | Negligible overhead |

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                                                   | Validation             |
| -------------------------------------------- | ------ | ---------- | ---------------------------------------------------------------------------- | ---------------------- |
| Operator forgets home is locked              | Low    | Low        | HOME_POSITION broadcast shows current home; operator can re-send DO_SET_HOME | Document behavior      |
| Lock prevents correction of bad GCS-set home | Low    | Low        | Operator can send another DO_SET_HOME to update; power cycle resets lock     | Test override scenario |

## Implementation Notes

- Single boolean field addition to SystemState
- Checked by disarmed update logic (FR-00172)
- Set by `MAV_CMD_DO_SET_HOME` handler in `command.rs`
- No persistence mechanism needed (acceptable for pico_trail)

## External References

- ArduPilot `lock_home()`: `repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3078`
- ArduPilot `home_is_locked()` check: `repo/ardupilot/Rover/commands.cpp:44`
