# T-00167 GCS Communication Lost Failsafe Implementation

## Metadata

- Type: Task
- Status: Done

## Links

- Related Analyses:
  - [AN-00046-communication-lost-action](../../analysis/AN-00046-communication-lost-action.md)
  - [AN-00011-failsafe-system](../../analysis/AN-00011-failsafe-system.md)
- Related Requirements:
  - [FR-00041-gcs-loss-failsafe](../../requirements/FR-00041-gcs-loss-failsafe.md) (primary)
  - [FR-00037-failsafe-parameters](../../requirements/FR-00037-failsafe-parameters.md)
  - [FR-00038-failsafe-recovery](../../requirements/FR-00038-failsafe-recovery.md)
  - [NFR-00029-failsafe-detection-latency](../../requirements/NFR-00029-failsafe-detection-latency.md)
  - [NFR-00044-no-false-failsafe-triggers](../../requirements/NFR-00044-no-false-failsafe-triggers.md)
- Related ADRs: N/A -- follows existing BatteryFailsafeChecker pattern, no new architecture decision needed
- Prerequisite Tasks: N/A
- Dependent Tasks: N/A
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement end-to-end GCS communication lost failsafe using a two-stage timeout system (`FS_GCS_TIMEOUT` + `FS_TIMEOUT`), following the established `BatteryFailsafeChecker` pattern. This adds a `GcsFailsafeChecker` in the core crate that detects heartbeat loss, executes configurable actions (Hold/RTL), sends STATUSTEXT notifications, and recovers when heartbeats resume. Parameter definitions are aligned with ArduPilot numbering and the duplicate GCS detection in firmware's `ArmedStateMonitor` is removed.

## Scope

- In scope:
  - Fix `FailsafeAction` enum to match ArduPilot numbering (0=None, 1=RTL, 2=Hold)
  - Rename `FS_TIMEOUT` (5.0s) to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT` (1.5s)
  - Create `GcsFailsafeChecker` in `crates/core/src/autopilot/gcs_failsafe.rs`
  - Wire GCS failsafe into `MavlinkLoopRunner` (alongside battery failsafe)
  - Mode exemptions (Hold, RTL, SmartRTL)
  - Never-seen guard (skip if no heartbeat ever received)
  - Recovery logic (clear on heartbeat resume, no auto mode switch)
  - STATUSTEXT notifications and local logging
  - Remove GCS heartbeat tracking from firmware `ArmedStateMonitor`
  - Unit tests for checker and runner integration
- Out of scope:
  - RC failsafe (separate feature)
  - Continue-mission mode (`FS_GCS_ENABLE=2`)
  - `FS_OPTIONS` bitmask
  - Unified failsafe framework
  - Terminate action
  - `RC_CHANNELS_OVERRIDE`/`MANUAL_CONTROL` as heartbeat reset

## Success Metrics

| Metric                      | Target                                                                |
| --------------------------- | --------------------------------------------------------------------- |
| Core tests pass             | `cargo test -p pico_trail_core --features embassy --lib` passes       |
| Core (no embassy) unchanged | `cargo test -p pico_trail_core --lib` passes                          |
| Firmware builds             | `./scripts/build-rp2350.sh pico_trail_rover` compiles                 |
| Detection timing            | Action within `FS_GCS_TIMEOUT + FS_TIMEOUT + 200ms` of last heartbeat |
| No false triggers           | Zero triggers with 3% packet loss at 1 Hz heartbeat                   |
| Parameter alignment         | `FS_ACTION` matches ArduPilot numbering (0=None, 1=RTL, 2=Hold)       |
