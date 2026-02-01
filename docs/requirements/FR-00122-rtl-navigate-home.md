# FR-00122 Direct RTL Navigate to Home Position

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00120-rtl-entry-validation](FR-00120-rtl-entry-validation.md)
- Dependent Requirements:
  - [FR-00119-rtl-arrival-stop](FR-00119-rtl-arrival-stop.md)
  - [FR-00121-rtl-gps-loss-handling](FR-00121-rtl-gps-loss-handling.md)
  - [FR-00126-smartrtl-rtl-fallback](FR-00126-smartrtl-rtl-fallback.md)
  - [NFR-00085-rtl-update-rate](NFR-00085-rtl-update-rate.md)
  - [NFR-00084-rtl-memory-overhead](NFR-00084-rtl-memory-overhead.md)
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall navigate the vehicle directly to the stored home position when direct RTL mode is activated (as fallback when SmartRTL path is unavailable), using the SimpleNavigationController to calculate steering and throttle commands.

**Note:** Direct RTL is the fallback mode. SmartRTL (FR-00125) is the default RTL behavior that retraces the recorded path.

## Rationale

Direct RTL provides straight-line navigation to home as a fallback when SmartRTL cannot operate. This capability is essential for:

- Fallback: When SmartRTL path buffer is empty or unavailable
- Safety: Returning the vehicle when communication is lost
- Failsafe actions: Battery, RC, and GCS loss recovery
- First arm: Before any path has been recorded

## User Story

As an operator, I want the vehicle to navigate back to the launch point when RTL mode is activated, so that I can recover the vehicle safely from any location.

## Acceptance Criteria

- [ ] RTL mode uses SimpleNavigationController for navigation calculations
- [ ] Navigation target is set to stored HomePosition (latitude, longitude)
- [ ] Steering and throttle outputs are calculated each update cycle
- [ ] Navigation continues until vehicle reaches home (within WP_RADIUS)
- [ ] RTL mode can be entered via GCS mode switch command
- [ ] RTL mode can be triggered by failsafe actions

## Technical Details

### Functional Requirement Details

**Mode Entry:**

- Validate entry conditions via `RtlMode::can_enter()`
- Retrieve HomePosition from SYSTEM_STATE
- Set navigation target to home coordinates
- Reset navigation controller state

**Navigation Loop:**

- Get current GPS position from SYSTEM_STATE
- Call `nav_controller.update(&current, &target, dt)`
- Apply steering/throttle from NavigationOutput to actuators
- Check `at_target` flag for arrival detection

**Integration:**

- RtlMode implements Mode trait (enter/update/exit lifecycle)
- Exported from `src/rover/mode/mod.rs`
- Registered in mode switching logic

## Platform Considerations

N/A - Platform agnostic (navigation algorithms are cross-platform)

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                     | Validation                       |
| --------------------------- | ------ | ---------- | ------------------------------ | -------------------------------- |
| GPS fix lost during RTL     | High   | Medium     | Transition to Hold mode        | Test with GPS disconnection      |
| Navigation oscillation      | Medium | Low        | Use appropriate WP_RADIUS (2m) | Test arrival detection threshold |
| Home position not reachable | Medium | Low        | Operator manual override       | Document operator procedures     |

## Implementation Notes

- RTL mode implementation: `src/rover/mode/rtl.rs`
- Uses existing SimpleNavigationController from `src/subsystems/navigation/controller.rs`
- HomePosition stored in `src/communication/mavlink/state.rs`
- Mode trait defined in `src/rover/mode/mod.rs`

## External References

- [ArduPilot Rover RTL Mode](https://ardupilot.org/rover/docs/rtl-mode.html) - Reference implementation
