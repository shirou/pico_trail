# NFR-00044 No False Failsafe Triggers Under Normal Operation

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
  - [FR-00041-gcs-loss-failsafe](FR-00041-gcs-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Requirement Statement

Failsafe system shall not introduce false triggers under normal operation (< 0.1% false trigger rate), ensuring reliability and operator confidence in the safety system. GCS failsafe shall not produce false triggers under normal WiFi operation with up to 3% packet loss.

## Rationale

False failsafe activation disrupts missions and reduces operator confidence. Conservative timeouts, the two-stage timeout system, and hysteresis prevent false triggers while maintaining safety response. pico_trail operates primarily over WiFi where brief dropouts (100-500ms) are common.

## Acceptance Criteria

- [ ] False trigger rate < 0.1% (measured via test flights)
- [ ] No false triggers when RC/GCS signals healthy
- [ ] No false GCS triggers under normal WiFi operation with up to 3% packet loss at 1 Hz heartbeat rate
- [ ] Two-stage timeout (5s detection + 1.5s persistence) prevents transient GCS false triggers
- [ ] Hysteresis prevents flapping (1 second stable before RC clear; immediate clear for GCS per ArduPilot)
- [ ] Conservative timeout defaults (1.5s RC persistence, 5.0s GCS detection)
- [ ] Battery voltage filtering prevents transient false triggers
- [ ] Never-seen guard prevents false trigger when GCS has never connected

## Technical Details (if applicable)

### Target

< 0.1% false trigger rate

### Measurement

Test flights with healthy signals, count unintended failsafe activations.

### Mitigation

- ArduPilot-proven timeouts
- Two-stage timeout system for GCS (detection + persistence) per [AN-00046](../analysis/AN-00046-communication-lost-action.md)
- Never-seen guard: skip GCS failsafe if no heartbeat ever received
- Hysteresis on RC recovery
- Battery voltage filtering
- 10 second delay on battery LOW failsafe

### WiFi Dropout Analysis (per AN-00046)

| Timeout | False Trigger Risk | Use Case                      |
| ------- | ------------------ | ----------------------------- |
| 2s      | High (WiFi drops)  | Not recommended for WiFi      |
| 5s      | Low                | Good for WiFi links (default) |
| 10s     | Very Low           | Conservative, long missions   |

## External References

- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
- Analysis: [AN-00046-communication-lost-action](../analysis/AN-00046-communication-lost-action.md)
