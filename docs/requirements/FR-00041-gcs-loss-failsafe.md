# FR-00041 GCS Communication Loss Failsafe

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
  - [FR-00062-control-modes](FR-00062-control-modes.md)

- Dependent Requirements:
  - [FR-00034-failsafe-action-priority](FR-00034-failsafe-action-priority.md)
  - [FR-00038-failsafe-recovery](FR-00038-failsafe-recovery.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall detect GCS communication loss within 5 seconds of the last HEARTBEAT message and trigger a configured failsafe action, but shall not trigger while in RTL or Hold modes to avoid redundant actions.

## Rationale

GCS loss is common during autonomous missions beyond RC range. 5 second timeout (longer than RC) accounts for telemetry link delays while still providing timely safety response. Disabling in RTL/Hold prevents unnecessary mode changes when vehicle is already in safe state.

## User Story (if applicable)

As an operator, I want the vehicle to automatically enter RTL or Hold mode when GCS connection is lost for more than 5 seconds during autonomous operation, so that the vehicle returns home if I lose telemetry.

## Acceptance Criteria

- [ ] Monitor GCS HEARTBEAT message arrival time
- [ ] Trigger failsafe if no HEARTBEAT received within `FS_GCS_TIMEOUT` seconds (default 5.0s)
- [ ] Do not trigger in RTL or Hold modes (already safe)
- [ ] Execute configured action from `FS_GCS_ACTION` parameter
- [ ] Send STATUSTEXT: "Failsafe: GCS Lost" with severity WARNING
- [ ] Log failsafe event with timestamp and action
- [ ] Clear failsafe when HEARTBEAT messages resume
- [ ] Default: GCS failsafe disabled (`FS_GCS_ENABLE=false`)

## Technical Details (if applicable)

### Functional Requirement Details

**ArduPilot Parameters**:

- `FS_GCS_ENABLE`: Enable GCS failsafe (bool, default false)
- `FS_GCS_TIMEOUT`: Timeout in seconds (float, default 5.0, range 0.0-30.0)
- `FS_GCS_ACTION`: Same values as `FS_ACTION` (0=None, 1=Hold, 2=RTL, etc.)

**Mode-Specific Behavior**:

- Active in: Manual, Auto, Guided modes
- Disabled in: Hold, RTL, SmartRTL modes

## Platform Considerations

### Cross-Platform

GCS timeout detection is platform-independent.

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                         |
| --------------------------------------- | ------ | ---------- | -------------------------------------------------- |
| False trigger from telemetry link delay | Medium | Medium     | 5s timeout allows for delays, disable by default   |
| GCS loss during critical maneuver       | High   | Low        | RTL/Hold exemption prevents redundant mode changes |

## Implementation Notes

- Check GCS timestamp age at 10 Hz
- Update GCS timestamp on every HEARTBEAT from GCS system ID
- Follow ArduPilot: disabled by default, requires explicit opt-in

Related code areas:

- `src/vehicle/failsafe/checkers/gcs_loss.rs`
- `src/communication/mavlink/router.rs` - HEARTBEAT timestamp update

## External References

- ArduPilot GCS Failsafe: <https://ardupilot.org/rover/docs/apms-failsafe-function.html>
- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
