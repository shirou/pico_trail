# FR-zcnqw Mission Item Reached

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-l8emi-mission-execution-gaps](../analysis/AN-l8emi-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
  - [ADR-0yapl-mission-execution-telemetry-architecture](../adr/ADR-0yapl-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements:
  - [FR-m2c9z-mission-waypoint-navigation](FR-m2c9z-mission-waypoint-navigation.md)
- Dependent Requirements:
  - [NFR-at4uq-mission-telemetry-latency](NFR-at4uq-mission-telemetry-latency.md)
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall send a MISSION_ITEM_REACHED message when a NAV command waypoint is reached, before advancing to the next waypoint.

## Rationale

MISSION_ITEM_REACHED (message #46) notifies the GCS that a specific waypoint has been reached. This enables:

- Progress logging in Mission Planner
- Triggering GCS-side actions on waypoint arrival
- Verifying mission execution correctness

ArduPilot sends this message after hold time expires and before advancing to the next item.

## User Story (if applicable)

As a GCS operator, I want to receive a notification when the rover reaches each waypoint, so that I can verify the mission is progressing correctly and log waypoint arrival times.

## Acceptance Criteria

- [ ] MISSION_ITEM_REACHED sent when NAV command completes (after hold time if applicable)
- [ ] Sent before advancing to next waypoint
- [ ] `seq` field contains the reached waypoint sequence number
- [ ] Only sent for NAV commands (command ID <= 95), not DO commands
- [ ] Not sent if mission is aborted before waypoint is reached

## Technical Details (if applicable)

### Functional Requirement Details

**MISSION_ITEM_REACHED Message (ID: 46):**

| Field | Type   | Description                             |
| ----- | ------ | --------------------------------------- |
| `seq` | uint16 | Sequence number of reached mission item |

**Emission Timing:**

```
1. Navigation detects at_target (within WP_RADIUS)
2. Hold time elapses (if param1 > 0)
3. Send MISSION_ITEM_REACHED with seq = current_index
4. Advance to next waypoint
```

**Integration Point:**

Emit from Auto mode `update()` in `crates/firmware/src/rover/mode/auto.rs` when waypoint arrival is confirmed and hold time has elapsed. The message must be queued for async send since Auto mode update is synchronous.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                       | Impact | Likelihood | Mitigation                    | Validation                       |
| -------------------------- | ------ | ---------- | ----------------------------- | -------------------------------- |
| Message lost in send queue | Low    | Low        | Use reliable message queue    | Test message delivery under load |
| Duplicate send on re-entry | Low    | Low        | Track sent state per waypoint | Test rapid waypoint transitions  |

## Implementation Notes

Preferred approaches:

- Queue message via `status_notifier` or similar async send mechanism
- Emit in Auto mode after hold time check, before `advance_waypoint()`

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs:223-241` - Waypoint arrival handling
- `crates/firmware/src/communication/mavlink/handlers/telemetry.rs` - Message sending

## External References

- [MAVLink MISSION_ITEM_REACHED](https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED)
