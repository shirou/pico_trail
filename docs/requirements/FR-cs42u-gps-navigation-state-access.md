# FR-cs42u GPS Navigation State Access

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-3ik7l-gps-operation-data-management](../requirements/FR-3ik7l-gps-operation-data-management.md)
- Dependent Requirements:
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
  - [FR-333ym-gps-waypoint-navigation](../requirements/FR-333ym-gps-waypoint-navigation.md)
  - [FR-erpze-guided-mode-navigation](../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-obwjs-position-target-command-handler](../requirements/FR-obwjs-position-target-command-handler.md)
  - [FR-tmibt-position-target-state](../requirements/FR-tmibt-position-target-state.md)
  - [NFR-wwke9-gps-state-thread-safety](../requirements/NFR-wwke9-gps-state-thread-safety.md)
- Related Tasks:
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Requirement Statement

The system shall provide a shared GPS state accessible to the navigation subsystem and control loop, enabling position-based autonomous navigation.

## Rationale

Waypoint navigation (FR-333ym) requires continuous GPS position data for path following. The GPS driver runs as an independent async task, while the navigation subsystem and control loop need to access the latest position. A shared state mechanism bridges these components without tight coupling.

## User Story (if applicable)

As a navigation algorithm, I want to read the current GPS position, so that I can calculate the heading and distance to the next waypoint.

## Acceptance Criteria

- [ ] GPS position is accessible from navigation subsystem without blocking
- [ ] GPS position includes timestamp for freshness validation
- [ ] Navigation can detect stale GPS data (>1 second since last update)
- [ ] GPS state includes fix type for validity checking
- [ ] GPS state is updated when new valid position received from GPS driver
- [ ] Multiple consumers can read GPS state concurrently
- [ ] GPS state access does not introduce latency to control loop (>1ms)

## Technical Details (if applicable)

### Functional Requirement Details

**GPS State Structure:**

```rust
pub struct GpsState {
    pub position: Option<GpsPosition>,
    pub last_update_us: u64,  // Timestamp in microseconds
    pub fix_type: GpsFixType,
}
```

**Access Pattern:**

- GPS driver task: Updates state when valid position received
- Telemetry task: Reads state for MAVLink message generation
- Navigation subsystem: Reads state for waypoint following
- Control loop: Reads state for Auto mode position feedback

**Freshness Check:**

```rust
impl GpsState {
    pub fn is_fresh(&self, current_time_us: u64, max_age_us: u64) -> bool {
        current_time_us.saturating_sub(self.last_update_us) < max_age_us
    }
}
```

## Platform Considerations

### Embedded (RP2350)

- State sharing via Embassy Mutex or atomic operations
- No heap allocation for state access
- Consider lock-free read for control loop (single writer, multiple readers)

### Host Tests

- Mock GPS state for navigation unit tests
- Test concurrent access patterns

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                            | Validation                |
| ------------------------------------ | ------ | ---------- | ------------------------------------- | ------------------------- |
| Mutex contention delays control loop | High   | Low        | Use try_lock or lock-free pattern     | Benchmark access latency  |
| Stale data used for navigation       | Medium | Medium     | Timestamp validation, GPS failsafe    | Unit test freshness check |
| Race condition on state update       | Medium | Low        | Single writer pattern (GPS task only) | Code review, stress test  |

## Implementation Notes

- Extend SystemState with GPS fields (consistent with battery, attitude pattern)
- Alternative: Dedicated static `Mutex<GpsState>` if SystemState becomes too large
- GPS driver calls state update after successful NMEA parse
- Navigation reads state at waypoint calculation frequency (10-50Hz)

## External References

N/A - No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
