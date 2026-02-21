# NFR-00180 HOME_POSITION Broadcast Bandwidth

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00173-home-position-broadcast](FR-00173-home-position-broadcast.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

HOME_POSITION broadcast bandwidth shall be near zero under normal operation. The on-change-only broadcast pattern shall produce fewer than 10 HOME_POSITION messages per typical flight session.

## Rationale

HOME_POSITION is broadcast on-change only (not periodic), matching ArduPilot behavior. HOME_POSITION_DATA is approximately 52 bytes per message. With on-change broadcasting, typical message count is: 1 (initial GPS fix) + a few (disarmed refinements) + 0-1 (GCS override) = fewer than 10 messages per session. This ensures HOME_POSITION does not consume meaningful bandwidth on the WiFi telemetry link, leaving capacity for higher-frequency telemetry streams (ATTITUDE at 10 Hz, GLOBAL_POSITION_INT at 2 Hz, etc.).

## User Story

The system shall ensure home position broadcasting does not consume noticeable telemetry bandwidth, so that other critical telemetry streams are not impacted.

## Acceptance Criteria

- [ ] HOME_POSITION is not in any periodic telemetry stream
- [ ] Fewer than 10 HOME_POSITION messages per typical flight session
- [ ] Total HOME_POSITION bandwidth per session < 520 bytes (\~52 bytes x 10 messages)
- [ ] No periodic HOME_POSITION timer or stream configuration exists

## Technical Details

### Non-Functional Requirement Details

**Expected message count per session:**

| Event                  | Count   | When                                                |
| ---------------------- | ------- | --------------------------------------------------- |
| First GPS fix auto-set | 1       | Once on power-up                                    |
| Disarmed refinements   | 0-5     | GPS accuracy convergence, \~1 Hz, stops when stable |
| GCS override           | 0-1     | Optional operator action                            |
| **Total**              | **1-7** | **Well under 10**                                   |

**Bandwidth calculation:**

- HOME_POSITION_DATA: \~52 bytes
- Maximum 10 messages: 520 bytes total per session
- Comparison: ATTITUDE at 10 Hz = \~280 bytes/sec = \~1 MB per hour

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                                        | Validation                |
| -------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------- | ------------------------- |
| GPS jitter causes excessive disarmed updates | Low    | Low        | 0.5m threshold (FR-00172) filters normal GPS noise                | Count messages in SITL    |
| Future periodic stream added accidentally    | Low    | Low        | No stream config for HOME_POSITION; code review catches additions | Review telemetry streamer |

## Implementation Notes

- Verify HOME_POSITION is not added to `StreamConfig` in TelemetryStreamer
- Disarmed update frequency (1 Hz) with 0.5m threshold naturally limits message count
- Can be verified by counting HOME_POSITION messages in SITL test logs

## External References

- ArduPilot `STREAM_POSITION_msgs[]` (does not include MSG_HOME): `repo/ardupilot/libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp:272`
