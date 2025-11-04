# FR-eeit0 Emergency Stop Tier System

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
- Dependent Requirements:
  - [FR-mg2bv-failsafe-integration](FR-mg2bv-failsafe-integration.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall support three emergency stop tiers with different severity levels (Controlled, Aggressive, EmergencyDisarm), automatically selecting the appropriate tier based on trigger source and situation criticality.

## Rationale

Different emergency situations require different stop responses. A low battery warning requires gentle controlled stop to preserve battery, while a critical hardware failure requires immediate motor cutoff. The tier system provides:

- Appropriate response matching situation severity
- Graduated escalation from gentle to aggressive stops
- Flexibility for different vehicle types and missions
- Clear selection criteria for automated systems

## User Story (if applicable)

As a system integrator, I want the emergency stop system to automatically select the appropriate stop tier based on the situation severity, so that the vehicle responds appropriately to different failure modes.

## Acceptance Criteria

- [ ] Tier 1 (Controlled): Gradual deceleration, 2-3 seconds, P gain = 10.0
- [ ] Tier 2 (Aggressive): Rapid deceleration, 0.5-1 second, P gain = 30.0
- [ ] Tier 3 (EmergencyDisarm): Immediate motor cutoff, < 100ms
- [ ] Tier selection logic based on trigger source (RC switch → Controlled, GCS emergency → Aggressive, hardware failure → EmergencyDisarm)
- [ ] Tier selection configurable via parameter for specific triggers
- [ ] Log selected tier with emergency stop event
- [ ] Prevent tier downgrade once higher tier selected (e.g., cannot switch from Aggressive to Controlled mid-stop)

## Technical Details (if applicable)

### Functional Requirement Details

**Tier Selection Criteria**:

| Situation                  | Stop Tier       | Rationale                                         |
| -------------------------- | --------------- | ------------------------------------------------- |
| Hold mode entry            | Controlled      | Normal operation, no emergency                    |
| RC loss failsafe           | Controlled      | Loss of manual control, but not critical          |
| Battery low failsafe       | Controlled      | Warning level, time for safe stop                 |
| Battery critical failsafe  | Aggressive      | Emergency level, rapid stop needed                |
| GCS emergency stop command | Aggressive      | Operator-initiated emergency                      |
| RC emergency stop switch   | Controlled      | Operator-initiated, but allow smooth deceleration |
| Motor/ESC failure          | EmergencyDisarm | Hardware failure, cannot control deceleration     |

**Tier Configuration**:

```rust
pub enum StopTier {
    None,            // Normal operation
    Controlled,      // Gradual (2-3s), P gain = 10.0
    Aggressive,      // Rapid (0.5-1s), P gain = 30.0
    EmergencyDisarm, // Immediate (<100ms)
}
```

**Stop Performance Targets**:

| Stop Tier       | Deceleration Time | Max Deceleration | Use Cases                    |
| --------------- | ----------------- | ---------------- | ---------------------------- |
| Controlled      | 2-3 seconds       | \~0.5 m/s²       | Normal failsafes, Hold mode  |
| Aggressive      | 0.5-1 second      | \~2.0 m/s²       | Critical failsafes, GCS stop |
| EmergencyDisarm | Immediate <100ms  | N/A (cutoff)     | Hardware failure only        |

## Platform Considerations

### Cross-Platform

Tier selection logic is platform-independent. Deceleration performance depends on vehicle mass and surface friction, which should be characterized during testing.

## Risks & Mitigation

| Risk                                           | Impact | Likelihood | Mitigation                                                           | Validation                                    |
| ---------------------------------------------- | ------ | ---------- | -------------------------------------------------------------------- | --------------------------------------------- |
| Wrong tier selected (too aggressive or gentle) | High   | Low        | Clear documented criteria, allow operator override via parameter     | Test all trigger sources, verify correct tier |
| Aggressive stop causes rollover                | High   | Medium     | Tune P gain conservatively, test on actual vehicle at various speeds | Field testing with instrumentation            |
| EmergencyDisarm used inappropriately           | High   | Low        | Restrict to hardware failure only, require explicit configuration    | Code review, safety testing                   |

## Implementation Notes

- Default to Controlled tier when uncertain
- Document tier selection rationale clearly
- Allow per-trigger tier override via parameters (e.g., `FS_RC_STOP_TIER`)
- Log tier selection reason with emergency stop event

Related code areas:

- `src/vehicle/emergency_stop/types.rs` - StopTier enum
- `src/vehicle/emergency_stop/manager.rs` - Tier selection logic

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
