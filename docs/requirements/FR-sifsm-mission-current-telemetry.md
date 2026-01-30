# FR-sifsm Mission Current Telemetry

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
  - [FR-v6571-mission-execution-state](FR-v6571-mission-execution-state.md)
- Dependent Requirements:
  - [NFR-at4uq-mission-telemetry-latency](NFR-at4uq-mission-telemetry-latency.md)
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall stream MISSION_CURRENT messages at 1Hz during mission execution and send immediately when the current waypoint changes.

## Rationale

Mission Planner and other GCS applications rely on MISSION_CURRENT (message #42) to display real-time mission progress. Without this message, the GCS cannot show which waypoint the rover is targeting, making mission monitoring impossible for operators.

## User Story (if applicable)

As a GCS operator, I want to see which waypoint the rover is currently targeting, so that I can monitor mission progress in real-time.

## Acceptance Criteria

- [ ] MISSION_CURRENT message streamed at 1Hz during mission execution
- [ ] MISSION_CURRENT sent immediately when current waypoint index changes
- [ ] `seq` field reports current NAV command index
- [ ] `total` field reports total mission item count
- [ ] `mission_state` field maps from MissionState enum (0=Unknown, 4=Active per MAVLink spec)
- [ ] Message not sent when no mission is loaded (or sent with `total` = `UINT16_MAX`)

## Technical Details (if applicable)

### Functional Requirement Details

**MISSION_CURRENT Message (ID: 42):**

| Field           | Type   | Description                                          |
| --------------- | ------ | ---------------------------------------------------- |
| `seq`           | uint16 | Current target mission item sequence number          |
| `total`         | uint16 | Total items; `UINT16_MAX` = no mission               |
| `mission_state` | uint8  | MISSION_STATE enum (0=Unknown, 1=Complete, 4=Active) |
| `mission_mode`  | uint8  | 0=Unknown, 1=In mission mode, 2=Suspended            |

**Streaming Integration:**

Add MISSION_CURRENT to `TelemetryStreamer` in `telemetry.rs`. The current message buffer is `heapless::Vec<MavMessage, 6>` and may need capacity increase or a separate emission path.

**Event-triggered Emission:**

When `advance_waypoint()` changes the current index, emit MISSION_CURRENT immediately in addition to the periodic stream.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                                      | Validation                    |
| ----------------------------- | ------ | ---------- | ----------------------------------------------- | ----------------------------- |
| Message buffer overflow       | Medium | Low        | Increase heapless Vec capacity or separate path | Test telemetry buffer at load |
| Increased telemetry bandwidth | Low    | Low        | 1Hz rate is minimal (< 20 bytes/s)              | Measure bandwidth usage       |
| Stale seq after mode change   | Low    | Medium     | Reset seq on mission clear/mode change          | Test mode transition behavior |

## Implementation Notes

Preferred approaches:

- Add to existing `TelemetryStreamer::update()` in `crates/firmware/src/communication/mavlink/handlers/telemetry.rs`
- Use `get_mission_state()` and `MISSION_STORAGE` to populate fields
- Consider separate 1Hz counter or use existing telemetry tick

Related code areas:

- `crates/firmware/src/communication/mavlink/handlers/telemetry.rs:196-252` - TelemetryStreamer
- `crates/firmware/src/core/mission/state.rs` - MISSION_STATE, MISSION_STORAGE

## External References

- [MAVLink MISSION_CURRENT](https://mavlink.io/en/messages/common.html#MISSION_CURRENT)
- [MAVLink MISSION_STATE enum](https://mavlink.io/en/messages/common.html#MISSION_STATE)
