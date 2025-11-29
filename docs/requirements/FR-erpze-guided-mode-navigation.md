# FR-erpze Guided Mode Navigation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
- Prerequisite Requirements:
  - [FR-obwjs-position-target-command-handler](FR-obwjs-position-target-command-handler.md)
  - [FR-2vbe8-navigation-controller](FR-2vbe8-navigation-controller.md)
  - [FR-tmibt-position-target-state](FR-tmibt-position-target-state.md)
  - [FR-q2sjt-control-mode-framework](FR-q2sjt-control-mode-framework.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement Guided mode that accepts real-time position targets from GCS via SET_POSITION_TARGET_GLOBAL_INT and navigates to the commanded position using the navigation controller.

## Rationale

Guided mode enables dynamic vehicle control without pre-planned missions:

- GCS operators can click on map to command vehicle movement
- Companion computers can send real-time navigation commands
- Enables interactive testing and manual waypoint control
- Foundation for more complex autonomous behaviors

This is a standard mode in ArduPilot Rover and expected by GCS applications.

## User Story (if applicable)

As a GCS operator, I want to use Guided mode to command the vehicle to specific positions in real-time, so that I can manually direct the vehicle without creating and uploading a full mission.

## Acceptance Criteria

- [ ] Guided mode entry requires valid GPS fix (3D fix minimum)
- [ ] Mode accepts position targets from SET_POSITION_TARGET_GLOBAL_INT
- [ ] Vehicle navigates toward target position using navigation controller
- [ ] Vehicle stops within waypoint acceptance radius (WP_RADIUS parameter)
- [ ] New position target updates navigation destination immediately
- [ ] Mode transitions to Hold if GPS fix is lost
- [ ] Mode exit clears current navigation target
- [ ] HEARTBEAT custom_mode reports Guided mode (mode 3)
- [ ] STATUSTEXT sent on mode entry: "Guided mode entered"

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Entry Conditions:**

- GPS fix type >= 3D_FIX
- Position target provided (via SET_POSITION_TARGET_GLOBAL_INT)
- Vehicle not in emergency stop state

**Mode Behavior:**

1. On entry: Validate GPS fix, initialize navigation state
2. On position target received: Update navigation destination
3. Each update cycle:
   - Get current GPS position from NavigationState
   - Calculate steering/throttle via navigation controller
   - Output commands to actuators
4. On target reached (distance < WP_RADIUS): Hold position
5. On exit: Clear navigation target, stop actuators

**Navigation Parameters (ArduPilot-compatible):**

- `WP_RADIUS`: Waypoint acceptance radius (default 2.0 m)
- `WP_SPEED`: Maximum navigation speed (default 2.0 m/s)

**State Diagram:**

```
               [Entry with GPS fix]
                       |
                       v
              +----------------+
              |   Navigating   |<--- New target received
              +----------------+
                |          |
    Target      |          | GPS lost
    reached     |          |
                v          v
          +---------+  +--------+
          | Holding |  | -> Hold|
          +---------+  +--------+
```

**Error Handling:**

- GPS fix lost during navigation: Transition to Hold mode
- No position target: Remain in Guided but hold position
- Target outside safe area: Reject and maintain current target

## Platform Considerations

### Pico W (RP2040)

Navigation calculations may need optimization for non-FPU processor. Consider fixed-point math or reduced update rate.

### Pico 2 W (RP2350)

Full floating-point navigation calculations supported at 50Hz.

### Cross-Platform

Mode logic identical across platforms; only navigation performance may vary.

## Risks & Mitigation

| Risk                                | Impact | Likelihood | Mitigation                            | Validation                          |
| ----------------------------------- | ------ | ---------- | ------------------------------------- | ----------------------------------- |
| GPS position lag causes overshoot   | Medium | Medium     | Tune WP_RADIUS, reduce approach speed | Test approach behavior at waypoints |
| No target causes undefined behavior | Medium | Low        | Hold position if no valid target      | Test mode entry without target      |
| Mode transition during movement     | Low    | Medium     | Graceful stop on mode exit            | Test rapid mode switches            |

## Implementation Notes

Preferred approaches:

- Implement as `GuidedMode` struct implementing `Mode` trait
- Use existing `NavigationState` for GPS position access
- Delegate steering/throttle calculation to navigation controller
- Follow existing Manual mode pattern for actuator output

Known pitfalls:

- Mode should not crash if entered without position target (hold instead)
- GPS update rate (1-10Hz) is slower than control loop (50Hz)
- Ensure atomic mode transitions

Related code areas:

- `src/rover/mode/` - Mode implementations
- `src/subsystems/navigation/` - Navigation controller
- `src/communication/mavlink/state.rs` - NavigationState

## External References

- [ArduPilot Guided Mode](https://ardupilot.org/rover/docs/guided-mode.html)
- [Rover Commands in Guided Mode](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
