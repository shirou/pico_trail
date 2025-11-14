# FR-dbjjx ArduPilot STATUSTEXT Conventions

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-onj2m-statustext-public-api](../requirements/FR-onj2m-statustext-public-api.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-eiuvv-statustext-implementation](../tasks/T-eiuvv-statustext-implementation/README.md)

## Requirement Statement

The system shall follow ArduPilot STATUSTEXT message conventions for prefixes, severity mapping, and message formatting to ensure compatibility with ground control station software (Mission Planner, QGroundControl) and familiarity for operators experienced with ArduPilot systems.

## Rationale

GCS operators expect familiar message formats and prefixes from ArduPilot-compatible systems. Consistent prefixes ("PreArm:", "Failsafe:", "Mode:") enable operators to quickly identify message categories. Following ArduPilot severity mapping ensures messages appear with correct visual indicators (colors, icons) in GCS software. Deviating from conventions confuses operators and reduces system usability.

## User Story

As an **operator using Mission Planner or QGroundControl**, I want **status messages to follow ArduPilot formatting conventions**, so that **I can quickly recognize message types and severity without learning a new format**.

## Acceptance Criteria

- [ ] Pre-arm check failures prefixed with "PreArm: " (e.g., "PreArm: Battery voltage low")
- [ ] Failsafe events prefixed with "Failsafe: " (e.g., "Failsafe: GCS connection lost")
- [ ] Mode changes prefixed with "Mode: " (e.g., "Mode: Changed to MANUAL")
- [ ] Parameter changes prefixed with "Param: " (e.g., "Param: ARMING_CHECK set to 1")
- [ ] Error conditions prefixed with "Error: " (e.g., "Error: IMU initialization failed")
- [ ] Severity levels mapped to RFC-5424 standards (0=EMERGENCY, 1=ALERT, 2=CRITICAL, 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG)
- [ ] Critical pre-arm failures use CRITICAL or ERROR severity
- [ ] Failsafe triggers use WARNING severity
- [ ] Mode changes use NOTICE severity
- [ ] Informational messages (startup, calibration) use INFO severity
- [ ] Messages display with correct severity colors in Mission Planner
- [ ] Messages display with correct severity icons in QGroundControl

## Technical Details

### Functional Requirement Details

**Message Prefix Standards:**

| Prefix      | Usage                                         | Example                                                 | Severity     |
| ----------- | --------------------------------------------- | ------------------------------------------------------- | ------------ |
| `PreArm:`   | Pre-arm check failures preventing arming      | "PreArm: Battery voltage 9.8V below 10.5V"              | ERROR        |
| `Failsafe:` | Failsafe trigger events                       | "Failsafe: GCS connection lost"                         | WARNING      |
| `Mode:`     | Mode change notifications and failures        | "Mode: Changed to MANUAL", "Mode: Unable to enter AUTO" | NOTICE/ERROR |
| `Param:`    | Parameter change confirmations                | "Param: ARMING_CHECK set to 1"                          | INFO         |
| `Error:`    | Error conditions (hardware, software)         | "Error: IMU initialization failed"                      | ERROR        |
| `Warning:`  | Warning conditions (non-critical)             | "Warning: Low battery, return to base recommended"      | WARNING      |
| `Armed`     | Arming state change (no prefix)               | "Armed", "Disarmed"                                     | NOTICE       |
| (none)      | Informational messages (startup, calibration) | "EKF2 IMU0 is using GPS", "Compass calibration started" | INFO         |

**RFC-5424 Severity Mapping:**

| Level | Name      | ArduPilot Usage                                           | Color (typical) |
| ----- | --------- | --------------------------------------------------------- | --------------- |
| 0     | EMERGENCY | Critical system failure requiring immediate landing       | Red/blinking    |
| 1     | ALERT     | Serious error requiring immediate attention               | Red             |
| 2     | CRITICAL  | Critical hardware failure (motor, sensor)                 | Red             |
| 3     | ERROR     | Error conditions (GPS lost, pre-arm failure)              | Red             |
| 4     | WARNING   | Warning conditions (battery low, failsafe triggered)      | Yellow/orange   |
| 5     | NOTICE    | Normal but significant events (mode change, armed status) | Green           |
| 6     | INFO      | Informational messages (startup, calibration)             | White/gray      |
| 7     | DEBUG     | Debug-level messages (usually disabled in production)     | Gray            |

**Usage Examples:**

```rust
// Pre-arm check failure
send_error("PreArm: Battery voltage 9.8V below minimum 10.5V");

// Failsafe trigger
send_warning("Failsafe: GCS connection lost");

// Mode change success
send_notice("Mode: Changed to MANUAL");

// Mode change failure
send_error("Mode: Unable to enter AUTO - no mission loaded");

// Parameter change
send_info("Param: ARMING_CHECK set to 1");

// Hardware error
send_critical("Error: Motor driver fault detected");

// Arming state
send_notice("Armed");
send_notice("Disarmed");
```

**Message Formatting Guidelines:**

- Keep messages concise but descriptive
- Include numeric values when relevant (e.g., voltage, threshold)
- Use consistent terminology (e.g., "GCS" not "ground station")
- Avoid excessive punctuation or formatting
- Use sentence case (first word capitalized, rest lowercase unless proper noun)

## Platform Considerations

### Embedded (RP2350)

- Prefix strings stored as compile-time constants (no allocation)
- Format strings at call site using stack buffers
- No special platform considerations

### Host Tests

- Test suite verifies correct prefix usage
- Mock GCS can verify severity mapping
- Automated tests check message format compliance

### Cross-Platform

- Message format and conventions identical on all platforms
- GCS compatibility verified on both Windows and Linux

## Risks & Mitigation

| Risk                                                       | Impact | Likelihood | Mitigation                                          | Validation                            |
| ---------------------------------------------------------- | ------ | ---------- | --------------------------------------------------- | ------------------------------------- |
| Developers use wrong prefix or severity                    | Medium | Medium     | Provide helper functions with correct defaults      | Code review, automated linting        |
| Message format differs from ArduPilot (operator confusion) | Medium | Medium     | Document conventions, provide examples in docs      | Cross-reference with ArduPilot source |
| GCS displays messages with wrong color/icon                | Low    | Low        | Test with actual Mission Planner and QGroundControl | Manual GCS testing during development |
| Severity mapping inconsistent with ArduPilot               | Medium | Low        | Follow RFC-5424 exactly as ArduPilot does           | Compare with ArduPilot message logs   |
| Long prefix + message exceeds 50 chars too often           | Low    | Medium     | Use concise prefixes, rely on chunking (FR-7qki7)   | Analyze typical message lengths       |

## Implementation Notes

**Preferred Patterns:**

- Provide helper functions for common message types:

  ```rust
  pub fn send_prearm_error(message: &str) {
      send_error(&format!("PreArm: {}", message));
  }

  pub fn send_failsafe_warning(message: &str) {
      send_warning(&format!("Failsafe: {}", message));
  }

  pub fn send_mode_notice(message: &str) {
      send_notice(&format!("Mode: {}", message));
  }
  ```

- Document prefix conventions in API docs

- Provide examples for each common message type

**Known Pitfalls:**

- Do not use custom prefixes (e.g., "PreCheck:", "FailSafe:")
- Do not mix severity levels (e.g., sending pre-arm failures as WARNING)
- Do not forget prefix for categorized messages

**Related Code:**

- Arming system: `src/system/arming/` (pre-arm checks)
- Mode system: `src/system/mode/` (mode changes)
- Future failsafe system (to be implemented)

## External References

- [ArduPilot STATUSTEXT Examples](https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink)
- [RFC-5424 Syslog Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
- [Mission Planner Message Display](https://ardupilot.org/planner/)
- [QGroundControl Message Handling](https://docs.qgroundcontrol.com/)
