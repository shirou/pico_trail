# FR-00052 RC Input Health Monitoring During Armed Operation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00014-rc-channels-processing](FR-00014-rc-channels-processing.md)

- Dependent Requirements:
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
  - [FR-00057-system-health-status-tracking](FR-00057-system-health-status-tracking.md)
  - [NFR-00042-monitoring-cpu-overhead](NFR-00042-monitoring-cpu-overhead.md)
  - [NFR-00036-high-frequency-monitor-detection-time](NFR-00036-high-frequency-monitor-detection-time.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall continuously monitor RC input health at 50 Hz during armed operation, tracking RC_CHANNELS message age and marking RC input as Warning (age > 100ms) or Unhealthy (age > 500ms).

## Rationale

Continuous RC monitoring during flight detects signal degradation before complete loss, allowing early warning and failsafe trigger. 50 Hz check rate matches RC_CHANNELS message rate for timely detection.

## Acceptance Criteria

- [ ] Track timestamp of last RC_CHANNELS message
- [ ] Check RC signal age every 20ms (50 Hz)
- [ ] Mark RC input as Healthy if age < 100ms
- [ ] Mark RC input as Warning if age 100-500ms
- [ ] Mark RC input as Unhealthy if age > 500ms
- [ ] Integrate with failsafe system to trigger RC loss failsafe
- [ ] Update RC health status in system health structure

## Technical Details (if applicable)

**Health Levels**:

- Healthy: Fresh RC data (< 100ms old)
- Warning: Degraded but present (100-500ms old)
- Unhealthy: Signal lost (> 500ms old) → triggers failsafe

**Check Frequency**: 50 Hz (every 20ms)

## External References

- Analysis: [AN-00009-armed-state-monitoring](../analysis/AN-00009-armed-state-monitoring.md)
