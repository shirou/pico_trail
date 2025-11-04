# FR-sp3at Vehicle Operational Modes

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-333ym-gps-waypoint-navigation](FR-333ym-gps-waypoint-navigation.md)
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-q2sjt-vehicle-mode-framework](FR-q2sjt-vehicle-mode-framework.md)
  - [FR-uk0us-manual-mode](FR-uk0us-manual-mode.md)
  - [FR-uo1p5-actuator-abstraction](FR-uo1p5-actuator-abstraction.md)

- Dependent Requirements:
  - [FR-2e00g-mode-execution-framework](FR-2e00g-mode-execution-framework.md)
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
  - [FR-lusbw-rc-signal-loss-failsafe](FR-lusbw-rc-signal-loss-failsafe.md)
  - [FR-meyja-manual-mode-implementation](FR-meyja-manual-mode-implementation.md)
  - [FR-sxsvw-failsafe-mechanisms](FR-sxsvw-failsafe-mechanisms.md)
  - [FR-zwsr5-gcs-loss-failsafe](FR-zwsr5-gcs-loss-failsafe.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall support at minimum 5 vehicle operational modes (Manual, Hold, Auto, RTL, Guided) with mode transitions completing within 100ms and no mode switching during critical maneuvers.

## Rationale

Multiple operational modes provide flexibility for different mission phases and safety scenarios:

- **Manual**: Direct operator control via RC - for testing, recovery, manual operation
- **Hold**: Stop in place - for troubleshooting, waiting, emergency stop
- **Auto**: Follow waypoint mission - for autonomous navigation
- **RTL (Return to Launch)**: Navigate back to launch point - for safe return, battery failsafe
- **Guided**: Accept real-time navigation commands from GCS - for dynamic missions

Mode-based architecture is standard in ArduPilot and PX4, providing clear separation of behaviors.

## User Story (if applicable)

As an operator, I want to switch between operational modes (Manual for direct control, Auto for waypoint missions, RTL to return home), so that I can adapt the vehicle's behavior to different mission phases and respond to changing conditions.

## Acceptance Criteria

- [ ] System implements Manual, Hold, Auto, RTL, and Guided modes
- [ ] Mode transitions complete within 100ms
- [ ] Mode changes commanded via RC switch or MAVLink command
- [ ] Current mode visible in MAVLink telemetry (HEARTBEAT message)
- [ ] Mode changes logged to Flash with timestamp
- [ ] No mode switching during critical maneuvers (e.g., mid-turn at high speed)
- [ ] Invalid mode transitions rejected (e.g., Auto without GPS fix)
- [ ] Mode-specific pre-entry checks (e.g., Auto requires valid mission uploaded)

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Descriptions:**

**Manual Mode:**

- RC input directly controls steering and throttle
- No autonomous behavior
- Operator has full control
- Use case: Testing, manual driving, emergency override

**Hold Mode:**

- Vehicle stops in place (zero throttle, center steering)
- Maintains current position (if GPS available)
- No autonomous navigation
- Use case: Pause mission, troubleshooting, emergency stop

**Auto Mode:**

- Follow uploaded waypoint mission
- Use S-curve path planning for navigation
- Automatically progress through waypoints
- Use case: Autonomous missions, surveys, delivery

**RTL (Return to Launch) Mode:**

- Navigate back to launch point (location where vehicle was armed)
- Use straight-line path (not full mission waypoint following)
- Automatically disarm upon reaching launch point
- Use case: Battery failsafe, end of mission, safe return

**Guided Mode:**

- Accept real-time navigation commands from GCS via MAVLink
- Navigate to commanded position using S-curve planner
- Allow dynamic mission changes without uploading full mission
- Use case: Manual GCS control, dynamic missions, obstacle avoidance

**Mode Transitions:**

```
                    Manual
                      |
        +-------------+-------------+
        |             |             |
      Hold          Auto          RTL
        |             |             |
        +---------- Guided ---------+
```

**Transition Rules:**

- From any mode to Manual: Always allowed (safety override)
- From Manual to Auto: Requires GPS fix + valid mission uploaded
- From Auto to Hold: Always allowed
- From Hold to Auto: Requires GPS fix + valid mission
- From any mode to RTL: Requires GPS fix + known launch point
- From any mode to Guided: Requires GPS fix

**Pre-entry Checks:**

- Auto mode: GPS 3D fix, mission uploaded, pre-arm checks passed
- RTL mode: GPS 3D fix, launch point recorded
- Guided mode: GPS 3D fix

**Mode Transition Events:**

- Log mode change: "Mode changed: Manual → Auto"
- Send MAVLink message: `STATUSTEXT` with new mode
- Update `HEARTBEAT` custom_mode field

**Critical Maneuver Detection:**

Prevent mode changes during:

- High-speed turns (lateral acceleration > 0.5 m/s²)
- Rapid deceleration (deceleration > 2 m/s²)
- Waypoint transition (within 5 meters of waypoint)

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (mode logic independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Mode implementation should be platform-independent. Use platform abstraction for RC input and actuator output.

## Risks & Mitigation

| Risk                                          | Impact | Likelihood | Mitigation                                                | Validation                                 |
| --------------------------------------------- | ------ | ---------- | --------------------------------------------------------- | ------------------------------------------ |
| Mode transition during critical maneuver      | High   | Low        | Detect critical maneuvers, defer transition until safe    | Test mode switches during turns            |
| Invalid mode entered without required sensors | High   | Medium     | Implement pre-entry checks, reject invalid transitions    | Attempt Auto mode without GPS fix          |
| Mode confusion (operator unaware of mode)     | Medium | Medium     | Send mode change notifications via MAVLink, log changes   | Verify GCS displays current mode correctly |
| Slow mode transition (>100ms)                 | Low    | Low        | Profile mode initialization code, optimize critical paths | Measure mode transition latency            |

## Implementation Notes

Preferred approaches:

- Implement **mode as trait** with common interface:
  ```rust
  pub trait VehicleMode {
      fn enter(&mut self) -> Result<()>;
      fn exit(&mut self) -> Result<()>;
      fn update(&mut self, dt: f32) -> Result<()>;
      fn name(&self) -> &'static str;
  }
  ```
- Use **state machine** for mode transitions with guard conditions
- Implement **mode manager** to handle transitions and enforce rules

Known pitfalls:

- Mode transition must be atomic (no partial transitions)
- Some modes require cleanup on exit (e.g., RTL must clear navigation target)
- Mode changes should not lose state (e.g., Auto mode should remember mission progress)
- Manual mode must always be reachable (safety override)

Related code areas:

- `src/vehicle/modes/` - Mode implementations (manual.rs, hold.rs, auto.rs, rtl.rs, guided.rs)
- `src/vehicle/mode_manager.rs` - Mode transition logic
- `src/communication/mavlink/command.rs` - Mode change commands

## External References

- ArduPilot Rover Modes: <https://ardupilot.org/rover/docs/rover-modes.html>
- MAVLink Custom Mode Field: <https://mavlink.io/en/messages/common.html#HEARTBEAT>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
