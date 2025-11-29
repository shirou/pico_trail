# FR-tmibt Position Target State Management

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
- Prerequisite Requirements:
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
- Dependent Requirements:
  - [FR-erpze-guided-mode-navigation](FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-obwjs-position-target-command-handler](FR-obwjs-position-target-command-handler.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall maintain position target state that stores the current navigation destination and provides target validity, distance, and bearing information to the navigation subsystem.

## Rationale

Position target state management is essential for:

- Sharing navigation target between MAVLink handler and navigation controller
- Tracking target validity (valid target vs no target)
- Providing computed navigation data (distance, bearing) to modes
- Enabling stale target detection based on timestamps
- Supporting both Guided mode (external targets) and Auto mode (mission waypoints)

## User Story (if applicable)

As the navigation system, I need a consistent way to access the current navigation target and related computed values, so that I can navigate to the target regardless of whether it came from GCS commands or mission waypoints.

## Acceptance Criteria

- [ ] Position target stored with latitude, longitude, altitude
- [ ] Target includes timestamp for staleness detection
- [ ] Target validity indicated (valid/invalid/expired)
- [ ] Distance to target calculated and accessible
- [ ] Bearing to target calculated and accessible
- [ ] Target can be set from SET_POSITION_TARGET_GLOBAL_INT handler
- [ ] Target can be set from mission waypoint in Auto mode
- [ ] Target cleared on mode exit
- [ ] Unit tests verify state management operations

## Technical Details (if applicable)

### Functional Requirement Details

**Position Target Structure:**

```rust
pub struct PositionTarget {
    /// Target latitude in degrees
    pub latitude: f32,
    /// Target longitude in degrees
    pub longitude: f32,
    /// Target altitude in meters (MSL)
    pub altitude: f32,
    /// Timestamp when target was set (ms since boot)
    pub timestamp_ms: u64,
    /// Target source for debugging
    pub source: TargetSource,
}

pub enum TargetSource {
    /// From SET_POSITION_TARGET_GLOBAL_INT
    GcsCommand,
    /// From mission waypoint
    MissionWaypoint(u16),  // waypoint sequence number
    /// From companion computer
    Companion,
}

pub enum TargetValidity {
    /// No target set
    NoTarget,
    /// Valid target available
    Valid,
    /// Target expired (velocity command timeout)
    Expired,
}
```

**NavigationState Extension:**

```rust
impl NavigationState {
    /// Current position target (if any)
    pub fn target(&self) -> Option<&PositionTarget>;

    /// Set new position target
    pub fn set_target(&mut self, target: PositionTarget);

    /// Clear current target
    pub fn clear_target(&mut self);

    /// Check if target is valid (not expired)
    pub fn target_validity(&self, current_time_ms: u64) -> TargetValidity;

    /// Distance to target in meters (if target valid)
    pub fn distance_to_target(&self) -> Option<f32>;

    /// Bearing to target in degrees (if target valid)
    pub fn bearing_to_target(&self) -> Option<f32>;
}
```

**Target Expiration:**

- Position targets: Never expire (persist until cleared)
- Velocity targets: Expire after 3 seconds (future enhancement)
- Expiration checked via `target_validity()` method

**Computed Values:**

Distance and bearing are computed on-demand from current GPS position:

```rust
impl NavigationState {
    pub fn distance_to_target(&self) -> Option<f32> {
        let current = self.gps_position()?;
        let target = self.target()?;
        Some(haversine_distance(current, target))
    }

    pub fn bearing_to_target(&self) -> Option<f32> {
        let current = self.gps_position()?;
        let target = self.target()?;
        Some(calculate_bearing(current, target))
    }
}
```

## Platform Considerations

### Pico W (RP2040)

PositionTarget struct is small (\~24 bytes), no memory concerns.

### Pico 2 W (RP2350)

No platform-specific considerations.

### Cross-Platform

Position target management is platform-independent.

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                         | Validation                        |
| ---------------------------------- | ------ | ---------- | ---------------------------------- | --------------------------------- |
| Stale target used for navigation   | Medium | Low        | Check validity before using target | Test with expired targets         |
| Race condition on target update    | Medium | Low        | Use atomic operations or mutex     | Test concurrent access            |
| Memory leak if targets not cleared | Low    | Low        | Clear target on mode exit          | Verify memory after mode switches |

## Implementation Notes

Preferred approaches:

- Extend existing `NavigationState` in `src/communication/mavlink/state.rs`
- Store `Option<PositionTarget>` for target
- Compute distance/bearing on-demand (not cached) to ensure freshness
- Use existing GPS position from NavigationState

Known pitfalls:

- Don't cache computed values - GPS position updates frequently
- Ensure target cleared on all mode exit paths
- Handle case where GPS position unavailable

Related code areas:

- `src/communication/mavlink/state.rs` - NavigationState
- `src/communication/mavlink/handlers/position_target.rs` - Handler that sets target
- `src/rover/mode/` - Modes that use target

## External References

- [MAVLink Position Target Messages](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
