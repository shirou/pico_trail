# NFR-s7fsz Controlled Stop Completion Time

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Controlled emergency stop shall complete within 3 seconds from typical operating velocity (1 m/s), ensuring predictable stop behavior and preventing extended unsafe operation.

## Rationale

3 second stop time balances safety (quick stop) with vehicle stability (no rollover). Longer stops allow too much distance traveled; shorter stops risk vehicle instability.

## Acceptance Criteria

- [ ] Stop completes (velocity < 0.1 m/s) within 3 seconds from 1 m/s typical velocity
- [ ] Automated test measures stop time across velocity range (0.5-2.0 m/s)
- [ ] Target: < 3s (95th percentile), < 4s (max)
- [ ] Stop time logged for every emergency stop event

## Technical Details (if applicable)

**Performance**: Stop time < 3 seconds at 1 m/s velocity

**Measurement**: Time from stop initiation to velocity < 0.1 m/s threshold

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
