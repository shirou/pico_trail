# AN-7ix56 Navigation Approach: S-Curve vs L1 Controller

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-cp76d-ardupilot-analysis](AN-cp76d-ardupilot-analysis.md)
  - [AN-5nucb-core-systems](AN-5nucb-core-systems.md)
- Related Requirements: N/A - Requirements will be created based on this analysis
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis examines two navigation approaches for rover and boat autonomous navigation: S-curve path planning (ArduPilot 4.3+ primary method) and L1 controller (legacy method). ArduPilot has transitioned to S-curve path planning as the primary navigation method for ground and water vehicles due to its superior handling of velocity/acceleration constraints and smoother operation. We recommend implementing S-curve as the primary method with optional L1 fallback for resource-constrained scenarios or simple point-to-point navigation.

Key findings: S-curve provides smoother acceleration profiles, better corner handling, and explicit consideration of vehicle dynamics, making it ideal for rovers and boats. L1 is simpler and less resource-intensive, making it useful as a fallback or for initial implementation. A phased approach (L1 first, then S-curve) balances development complexity with ultimate functionality.

## Problem Space

### Current State

The project currently has:

- No navigation algorithm implementation
- Basic ArduPilot analysis identifying navigation as a core requirement
- Understanding that ArduPilot 4.3+ uses S-curve as the primary method

### Desired State

A navigation system that:

- Provides smooth, safe waypoint navigation for rovers and boats
- Respects vehicle velocity and acceleration limits
- Handles corners gracefully without excessive overshoot
- Compatible with standard GCS tools (QGroundControl, Mission Planner)
- Efficient enough to run on Raspberry Pi Pico W/2W

### Gap Analysis

**Missing Implementation**:

- S-curve path generation algorithm
- Position controller to follow S-curve trajectories
- L1 controller implementation
- Parameter system for navigation tuning
- Testing infrastructure for comparing navigation methods

**Key Decision**:

Which navigation approach to implement first, and whether to support both methods.

## Stakeholder Analysis

| Stakeholder                | Interest/Need                                  | Impact | Priority |
| -------------------------- | ---------------------------------------------- | ------ | -------- |
| Rover Operators            | Smooth acceleration, no wheel slip             | High   | P0       |
| Boat Operators             | Reduced propeller cavitation, stable turns     | High   | P0       |
| GCS Users                  | Compatibility with modern ArduPilot behavior   | High   | P0       |
| Developers                 | Clear implementation path, testable components | Medium | P1       |
| Resource-Constrained Users | Option for lighter L1 controller on Pico W     | Medium | P2       |

## Research & Discovery

### User Feedback

N/A - New project without user base. Requirements derived from ArduPilot community standards.

### Competitive Analysis

**ArduPilot Evolution**:

- **Pre-4.3**: L1 controller was the primary navigation method
  - Proven algorithm based on MIT research
  - Converts origin-destination line to lateral acceleration
  - Simple, computationally efficient
  - Good for point-to-point navigation

- **ArduPilot 4.3+**: S-curve path planning is now primary
  - Generates smooth trajectories respecting velocity/acceleration limits
  - Better handling of waypoint transitions
  - Reduces mechanical stress on vehicles
  - Preferred method for rovers and boats

**Why S-Curve is Better for Rovers/Boats**:

1. **Smooth Acceleration/Deceleration**:
   - Gradual speed changes reduce drivetrain stress
   - Prevents wheel slip on rovers (especially on low-friction surfaces)
   - Reduces propeller cavitation on boats
   - More comfortable for passengers/cargo

2. **Velocity and Acceleration Limits**:
   - Explicitly considers `SPEED_MAX` and `ATC_ACCEL_MAX` parameters
   - Prevents unrealistic maneuvers the vehicle cannot execute
   - Respects physical constraints of the platform

3. **Continuous Path**:
   - Generates continuous position and velocity targets
   - No abrupt direction changes at waypoints
   - Vehicle smoothly transitions through waypoints without stopping

4. **Better Corner Handling**:
   - Automatically reduces speed before sharp corners
   - Smoother turn radius calculations
   - Prevents skidding on rovers, drift on boats

### Technical Investigation

**S-Curve Path Planning**:

Algorithm Steps:

1. **Path Generation**: Calculate smooth curve connecting waypoints
2. **Velocity Profile**: Determine speed along path respecting limits
3. **Trajectory Output**: Generate position + velocity targets at each timestep
4. **Position Control**: Calculate steering/throttle to follow trajectory

Visual Comparison:

```
Waypoint A -----> Waypoint B -----> Waypoint C

Without S-Curve (L1):
Speed: |‾‾‾‾‾‾‾‾|__|‾‾‾‾‾‾‾‾|__|
       ^急激な減速 ^急激な加速

With S-Curve:
Speed: |‾‾‾╲___/‾‾‾╲___/‾‾‾|
       ^滑らかな減速/加速
```

**Resource Requirements**:

| Method  | RAM Usage | CPU Load | Best For                                          |
| ------- | --------- | -------- | ------------------------------------------------- |
| S-Curve | \~4-6 KB  | Moderate | Smooth operation, modern ArduPilot compatibility  |
| L1      | \~2 KB    | Low      | Simple navigation, resource-constrained platforms |

**L1 Controller**:

```
Current Position
     ●
      \
       \  Lateral Acceleration
        \     Command
         \
          \
           ─────────────────────────
          Origin          Destination
```

The L1 controller calculates the lateral acceleration needed to converge to the line between origin and destination.

### Data Analysis

N/A - No operational data available yet.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-21**: System shall implement S-curve path planning for waypoint navigation → Will become FR-021
  - Rationale: Primary navigation method in ArduPilot 4.3+, provides smoother operation for rovers/boats
  - Acceptance Criteria: Generates smooth paths with configurable velocity/acceleration limits, path following error < 2 meters

- [ ] **FR-DRAFT-22**: System shall optionally support L1 controller as fallback navigation method → Will become FR-022
  - Rationale: Simpler alternative for resource-constrained scenarios or debugging
  - Acceptance Criteria: Selectable via parameter, provides basic waypoint navigation

- [ ] **FR-DRAFT-23**: System shall support navigation parameter configuration (speed, acceleration, jerk limits) → Will become FR-023
  - Rationale: Allow tuning for different vehicle types and operating conditions
  - Acceptance Criteria: Parameters configurable via MAVLink, persist across reboots

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-11**: S-curve path planning shall complete within 50ms for typical waypoint transitions → Will become NFR-011
  - Category: Performance
  - Rationale: Path updates must not stall control loops
  - Target: Measured via profiling on Pico 2 W hardware

- [ ] **NFR-DRAFT-12**: Navigation implementation shall be selectable at compile-time to reduce binary size → Will become NFR-012
  - Category: Resource Efficiency
  - Rationale: Allow building L1-only version for Pico W if S-curve too heavy
  - Target: Cargo features for `scurve` and `l1` navigation

## Design Considerations

### Technical Constraints

1. **Memory Limitations**: S-curve requires \~4-6 KB RAM for path buffers and state
2. **CPU Performance**: Path calculations must complete within control loop period (20ms @ 50Hz)
3. **ArduPilot Compatibility**: Should match ArduPilot 4.3+ parameter naming and behavior
4. **Testing Complexity**: Need simulation environment to validate navigation algorithms

### Potential Approaches

**Option A: S-Curve Only**

- Pros: Modern standard, best performance, simpler codebase (no dual implementation)
- Cons: Higher resource usage, more complex initial implementation
- Effort: Medium (S-curve implementation)

**Option B: L1 Only**

- Pros: Simple, low resource usage, proven algorithm
- Cons: Not aligned with modern ArduPilot, worse performance for rovers/boats
- Effort: Low (L1 implementation)

**Option C: Both S-Curve and L1** ⭐ **Recommended**

- Pros: Maximum flexibility, graceful degradation, easier debugging (start with L1)
- Cons: More code to maintain, larger binary
- Effort: Medium-High (both implementations + switching logic)

**Implementation Strategy**:

**Phase 1: L1 Controller**

- Implement simple L1 controller first
- Validate basic waypoint navigation
- Establish testing methodology
- Lower risk, faster initial results

**Phase 2: S-Curve Path Planning**

- Add S-curve path generation
- Implement position controller
- Benchmark memory and CPU usage

**Phase 3: Integration**

- Make S-curve the default
- Keep L1 as fallback (selectable via parameter `WP_NAV_TYPE`)
- Test both methods in real-world scenarios

### Architecture Impact

This analysis will drive the following ADRs:

- **ADR-012**: Navigation algorithm selection (S-curve primary, L1 fallback)
- **ADR-013**: Navigation parameter naming and structure
- **ADR-014**: Navigation testing strategy (SITL simulation)

## Risk Assessment

| Risk                                                    | Probability | Impact | Mitigation Strategy                                                  |
| ------------------------------------------------------- | ----------- | ------ | -------------------------------------------------------------------- |
| S-curve too resource-intensive for Pico W               | Medium      | Medium | Implement L1 first, benchmark S-curve, make L1 fallback selectable   |
| S-curve implementation complexity delays development    | Medium      | Low    | Start with L1 to validate navigation architecture, add S-curve later |
| Navigation parameter incompatibility with ArduPilot GCS | Low         | Medium | Use ArduPilot 4.3+ parameter names, test with QGroundControl early   |
| L1 performance inadequate for user needs                | Low         | Medium | Clearly document S-curve as primary method, implement both           |

## Open Questions

- [ ] Should we implement S-curve first or L1 first? → Next step: Draft ADR-012 evaluating phased approach (L1 then S-curve recommended)
- [ ] What is the minimum acceptable path following accuracy? → Method: Review ArduPilot tuning guides, test with different waypoint patterns
- [ ] How do we test navigation algorithms without physical hardware? → Next step: Research ArduPilot SITL (Software-In-The-Loop) simulation for embedded platforms
- [ ] Should navigation method be runtime-selectable or compile-time only? → Method: Evaluate binary size impact of dual implementation

## Recommendations

### Immediate Actions

1. Implement L1 controller as Phase 1 (simpler, validates navigation architecture)
2. Define navigation parameters following ArduPilot conventions
3. Set up basic waypoint navigation testing infrastructure

### Next Steps

1. [ ] Create FR-021 for S-curve path planning requirement
2. [ ] Create FR-022 for L1 controller fallback requirement
3. [ ] Create FR-023 for navigation parameter configuration
4. [ ] Create NFR-011 for navigation performance requirement
5. [ ] Create NFR-012 for navigation build-time selection
6. [ ] Create ADR-012 for navigation algorithm selection
7. [ ] Create task T-XXX for Phase 1 (L1 controller implementation)

### Out of Scope

The following advanced navigation features are explicitly excluded from initial implementation:

- **Spline Waypoints**: Only support straight-line segments initially
- **Rally Points**: Not required for basic rover/boat operation
- **Do-Land-Start**: Not applicable to ground/water vehicles
- **Terrain Following**: Not required for initial implementation
- **Obstacle Avoidance**: Separate feature, not part of core navigation

## Appendix

### References

- ArduPilot Rover S-Curve Documentation: <https://ardupilot.org/rover/docs/rover-tuning-navigation.html>
- ArduPilot AR_WPNav Library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_WPNav>
- ArduPilot AR_PosControl Library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_PosControl>
- L1 Controller Paper: "A new nonlinear guidance logic for trajectory tracking" by Park, Deyst, and How (AIAA 2004)
- ArduPilot Navigation Tuning: <https://ardupilot.org/rover/docs/rover-tuning-navigation.html>

### Raw Data

**Proposed Parameters** (ArduPilot-compatible naming):

```
WP_NAV_TYPE: 0=L1, 1=S-Curve (default)
WP_SPEED: Maximum speed (m/s)
WP_ACCEL: Maximum acceleration (m/s²)
WP_RADIUS: Waypoint acceptance radius (m)
WP_OVERSHOOT: Maximum overshoot distance (m)

# L1 specific
WP_L1_PERIOD: L1 controller period (s)
WP_L1_DAMPING: L1 damping ratio

# S-Curve specific
WP_SCURVE_TC: S-curve time constant (s)
WP_SCURVE_JERK: Maximum jerk limit (m/s³)
```

**Testing Plan**:

1. **Simulation**: Test in Software-In-The-Loop (SITL) simulation
2. **Waypoint Patterns**: Square, circle, figure-8 paths
3. **Speed Variations**: Test at different speeds (0.5 m/s to 5 m/s)
4. **Comparison Metrics**:
   - Cross-track error
   - Speed smoothness (jerk)
   - Waypoint overshoot
   - CPU usage
   - Memory usage

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
