# FR-00121 RTL GPS Loss Handling

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00122-rtl-navigate-home](FR-00122-rtl-navigate-home.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall transition to Hold mode and stop the vehicle if GPS fix is lost during RTL navigation, sending a STATUSTEXT message indicating the reason for the mode change.

## Rationale

GPS is essential for RTL navigation. If GPS is lost mid-navigation:

- Cannot determine current position
- Cannot calculate bearing/distance to home
- Continuing blindly is dangerous

Stopping in place (Hold mode) is the safest response, allowing GPS to potentially recover.

## User Story

As an operator, I want the vehicle to stop safely if GPS is lost during RTL, so that the vehicle does not continue moving without position awareness.

## Acceptance Criteria

- [ ] GPS fix loss detected when `gps.position()` returns None
- [ ] Vehicle transitions to Hold mode on GPS loss
- [ ] Throttle and steering set to zero immediately
- [ ] STATUSTEXT sent: "RTL: GPS lost, entering Hold"
- [ ] Log message recorded for diagnostics
- [ ] Mode transition follows standard mode switching logic

## Technical Details

### Functional Requirement Details

**Detection Logic:**

```rust
fn update(&mut self, dt: f32) -> Result<(), &'static str> {
    let state = SYSTEM_STATE.lock();
    let gps = state.gps.position().ok_or("GPS fix lost")?;
    // ... navigation continues if GPS available
}
```

**Response Sequence:**

1. `update()` returns `Err("GPS fix lost")`
2. Mode controller catches error
3. Initiates transition to Hold mode
4. Hold mode stops vehicle immediately
5. STATUSTEXT sent to GCS

**Integration with Mode Controller:**

- RTL update() returning Err triggers mode switch
- Mode controller has logic to select fallback mode (Hold)
- Follows existing mode switching patterns

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                         | Impact | Likelihood | Mitigation                    | Validation                  |
| ---------------------------- | ------ | ---------- | ----------------------------- | --------------------------- |
| Brief GPS dropout triggers   | Low    | Medium     | GPS has internal smoothing    | Test with GPS disconnection |
| Hold mode not available      | High   | Very Low   | Hold is always available mode | Verify Hold mode exists     |
| Operator unaware of GPS loss | Medium | Low        | STATUSTEXT notification       | Verify GCS receives message |

## Implementation Notes

- GPS loss detection in RTL update() method
- Fallback mode selection handled by mode controller
- Hold mode must be implemented before RTL GPS loss handling
- Consider: Hysteresis to prevent rapid mode switching on marginal GPS

## External References

- [ArduPilot GPS Failsafe](https://ardupilot.org/rover/docs/rover-failsafes.html) - GPS failsafe behavior reference
