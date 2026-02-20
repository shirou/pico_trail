# FR-00041 GCS Communication Loss Failsafe

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
  - [FR-00037-failsafe-parameters](FR-00037-failsafe-parameters.md)
  - [FR-00062-control-modes](FR-00062-control-modes.md)

- Dependent Requirements:
  - [FR-00034-failsafe-action-priority](FR-00034-failsafe-action-priority.md)
  - [FR-00038-failsafe-recovery](FR-00038-failsafe-recovery.md)
  - [NFR-00029-failsafe-detection-latency](NFR-00029-failsafe-detection-latency.md)
  - [NFR-00044-no-false-failsafe-triggers](NFR-00044-no-false-failsafe-triggers.md)

- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Requirement Statement

The system shall detect GCS communication loss using a two-stage timeout system -- `FS_GCS_TIMEOUT` (default 5.0s) for detection and `FS_TIMEOUT` (default 1.5s) for condition persistence -- and execute a configured failsafe action (`FS_ACTION`, ArduPilot numbering: 0=None, 1=RTL, 2=Hold). The system shall only check when the vehicle is armed, shall not trigger if no GCS heartbeat has ever been received (never-seen guard), shall skip action execution when already in Hold, RTL, or SmartRTL modes, and shall send STATUSTEXT notifications on both activation and recovery.

## Rationale

GCS loss is common during autonomous missions beyond RC range. The two-stage timeout (ArduPilot pattern) prevents transient false triggers from brief WiFi dropouts while still providing timely safety response. The never-seen guard prevents false positives during startup or when operating without GCS. Mode exemptions avoid redundant actions when the vehicle is already in a safe state. ArduPilot-aligned parameter numbering ensures operator familiarity.

## User Story (if applicable)

As an operator, I want the vehicle to automatically enter Hold or RTL mode when GCS connection is lost during autonomous operation, so that the vehicle is safe if I lose telemetry over WiFi.

## Acceptance Criteria

### Detection

- [ ] Track timestamp of last received HEARTBEAT from GCS
- [ ] Only check GCS failsafe when vehicle is armed (skip when disarmed)
- [ ] Detect GCS loss when heartbeat age exceeds `FS_GCS_TIMEOUT` (default 5.0s)
- [ ] Require condition to persist for `FS_TIMEOUT` (default 1.5s) before executing action
- [ ] Total time from last heartbeat to action: `FS_GCS_TIMEOUT + FS_TIMEOUT` (default 6.5s)
- [ ] Check at 10 Hz (100ms interval) in vehicle control loop

### Never-Seen Guard

- [ ] Track whether at least one GCS heartbeat has been received
- [ ] Do not trigger GCS failsafe if no heartbeat has ever been received
- [ ] Begin monitoring only after first heartbeat is received

### Mode Exemptions

- [ ] Check current `FlightMode` before executing failsafe action
- [ ] Skip action execution if mode is Hold, RTL, or SmartRTL
- [ ] Still track failsafe state (for notification/logging) even if action is skipped

### Action Execution

- [ ] Execute action from `FS_ACTION` parameter (ArduPilot numbering: 0=None, 1=RTL, 2=Hold)
- [ ] Apply action via `set_mode()` on system state
- [ ] Do not re-trigger action if failsafe is already active

### Notifications

- [ ] Queue STATUSTEXT "Failsafe: GCS Lost - {action}" with severity WARNING on activation
- [ ] Queue STATUSTEXT "GCS Failsafe Cleared" with severity INFO on recovery
- [ ] Log failsafe events locally via `log_warn!` / `log_info!` macros

### Configuration

- [ ] `FS_GCS_ENABLE`: Enable GCS failsafe (u8, default 1=Enabled)
- [ ] `FS_GCS_TIMEOUT`: GCS detection timeout (float, default 5.0s, range 2-120s)
- [ ] `FS_TIMEOUT`: Condition persistence timeout (float, default 1.5s, range 1-100s)
- [ ] `FS_ACTION`: Failsafe action (u8, default 2=Hold, 0=None, 1=RTL, 2=Hold)
- [ ] GCS failsafe enabled by default (differs from ArduPilot; pico_trail operates primarily over WiFi)

## Technical Details (if applicable)

### Two-Stage Timeout System (ArduPilot Pattern)

1. **`FS_GCS_TIMEOUT`** (default 5.0s): Time since last GCS HEARTBEAT before condition is detected
2. **`FS_TIMEOUT`** (default 1.5s): Time the condition must persist before action executes

The second timeout prevents transient failsafe activation on brief signal drops common on WiFi links.

### Architecture

- `GcsFailsafeChecker` in `crates/core/src/autopilot/gcs_failsafe.rs` (following `BatteryFailsafeChecker` pattern)
- Checker method: `check_gcs(last_heartbeat_us, heartbeat_count, current_time_us, is_armed, param_store)` -- caller passes data as parameters
- Wired into `MavlinkLoopRunner` control loop alongside battery failsafe check
- Core crate only (testable in SITL and host tests)

### Parameter Alignment with ArduPilot

`FS_ACTION` values match ArduPilot Rover numbering:

| Value | Action | Behavior                         |
| ----- | ------ | -------------------------------- |
| 0     | None   | No mode change, warning only     |
| 1     | RTL    | Return to launch                 |
| 2     | Hold   | Stop and hold position (default) |

### Mode-Specific Behavior

- Active in: Manual, Auto, Guided, and other non-safe modes
- Exempt: Hold, RTL, SmartRTL (vehicle already in safe state)

## Platform Considerations

### Cross-Platform

GCS timeout detection is platform-independent (core crate). Heartbeat timestamp is passed as a parameter from the caller, which reads it from the transport-layer dispatcher.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                                              |
| ----------------------------------------- | ------ | ---------- | ----------------------------------------------------------------------- |
| False trigger from WiFi dropout           | Medium | Medium     | Two-stage timeout (5s + 1.5s = 6.5s) accommodates typical WiFi dropouts |
| GCS never connected causes trigger        | Medium | Medium     | Never-seen guard: skip if no heartbeat ever received                    |
| GCS loss during critical maneuver         | High   | Low        | Mode exemption prevents redundant mode changes in safe modes            |
| FS_ACTION numbering change breaks configs | Medium | Low        | Document migration; default Hold still works                            |

## Implementation Notes

- Follow `BatteryFailsafeChecker` pattern: checker struct in `crates/core/src/autopilot/`, integration in `MavlinkLoopRunner`
- Remove duplicate GCS heartbeat tracking from firmware `ArmedStateMonitor` (core checker is authoritative)
- Checker state: `failsafe_active`, `condition_detected`, `condition_start_us` (\~16 bytes RAM)

Related code areas:

- `crates/core/src/autopilot/gcs_failsafe.rs` (new)
- `crates/core/src/autopilot/mavlink_runner.rs` (wire check)
- `crates/core/src/communication/dispatcher.rs` (heartbeat data source)
- `crates/core/src/parameters/failsafe.rs` (parameter definitions)

## External References

- ArduPilot GCS Failsafe: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- Analysis: [AN-00046-communication-lost-action](../analysis/AN-00046-communication-lost-action.md)
- Parent analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
