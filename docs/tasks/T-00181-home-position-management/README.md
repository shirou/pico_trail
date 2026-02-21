# T-00181 Home Position Management Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00047-home-position-management](../../analysis/AN-00047-home-position-management.md)
- Related Requirements:
  - [FR-00171-home-auto-set-gps-fix](../../requirements/FR-00171-home-auto-set-gps-fix.md) (primary)
  - [FR-00172-home-update-disarmed](../../requirements/FR-00172-home-update-disarmed.md)
  - [FR-00173-home-position-broadcast](../../requirements/FR-00173-home-position-broadcast.md)
  - [FR-00174-request-message-home-position](../../requirements/FR-00174-request-message-home-position.md)
  - [FR-00175-home-prearm-check](../../requirements/FR-00175-home-prearm-check.md)
  - [FR-00176-home-lock-mechanism](../../requirements/FR-00176-home-lock-mechanism.md)
  - [FR-00177-global-position-int-relative-alt](../../requirements/FR-00177-global-position-int-relative-alt.md)
  - [FR-00178-get-home-position-command](../../requirements/FR-00178-get-home-position-command.md)
  - [NFR-00179-home-auto-set-latency](../../requirements/NFR-00179-home-auto-set-latency.md)
  - [NFR-00180-home-broadcast-bandwidth](../../requirements/NFR-00180-home-broadcast-bandwidth.md)
- Related ADRs: N/A -- follows existing MavlinkLoopRunner pattern, no new architecture decision needed
- Prerequisite Tasks: N/A
- Dependent Tasks: N/A
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement complete home position management for the pico_trail rover: auto-set home on first GPS 3D fix, refine home at 1 Hz while disarmed, broadcast HOME_POSITION on change, support GCS home queries (REQUEST_MESSAGE and GET_HOME_POSITION), add pre-arm check requiring home, implement home lock for GCS overrides, and fix GLOBAL_POSITION_INT `relative_alt` to compute altitude relative to home.

This resolves Mission Planner's "failed to set home" error when entering Guided mode, and ensures RTL/failsafe actions always have a valid home destination.

## Scope

- In scope:
  - Add `home_locked: bool` field to `SystemState`
  - Auto-set home on first GPS 3D fix in `MavlinkLoopRunner` control loop
  - Disarmed home update at 1 Hz with 0.5m threshold
  - HOME_POSITION broadcast on every home change (on-change only, no periodic stream)
  - `MAV_CMD_REQUEST_MESSAGE` param1=242 support in `handle_command_long`
  - `MAV_CMD_GET_HOME_POSITION` (command 410) support in `handle_command_long`
  - Home lock: `MAV_CMD_DO_SET_HOME` sets `home_locked = true`
  - Pre-arm check: "waiting for home" in GPS check category
  - Fix `build_global_position_int()` to compute `relative_alt` from home altitude
  - Move `build_home_position_message()` to shared location accessible from both command handler and control loop
  - Wire home management in both firmware (pico_trail_rover) and SITL
  - Unit tests for all new logic
- Out of scope:
  - EKF origin management / GPS_GLOBAL_ORIGIN message
  - `MAV_CMD_DO_SET_HOME` via COMMAND_LONG (COMMAND_INT only)
  - `SET_HOME_POSITION` message handler (ID 243)
  - Home persistence across power cycles
  - Mission waypoint 0 = home
  - Rally points
  - Barometric altitude calibration

## Success Metrics

| Metric                    | Target                                                                   |
| ------------------------- | ------------------------------------------------------------------------ |
| Core tests pass           | `cargo test -p pico_trail_core --lib --quiet` passes                     |
| Core (embassy) tests pass | `cargo test -p pico_trail_core --features embassy --lib --quiet` passes  |
| Firmware builds           | `./scripts/build-rp2350.sh pico_trail_rover` compiles                    |
| Home auto-set             | Home set within one control loop iteration of first GPS 3D fix           |
| Disarmed update           | Home refined at 1 Hz when moved >0.5m while disarmed                     |
| Pre-arm check             | Arming blocked with "waiting for home" when home is None                 |
| Broadcast                 | HOME_POSITION sent on every home change (auto-set, update, GCS override) |
| REQUEST_MESSAGE           | HOME_POSITION returned for param1=242                                    |
| GET_HOME_POSITION         | HOME_POSITION returned for command 410                                   |
| Home lock                 | GCS-set home not overwritten by disarmed update                          |
| relative_alt              | GLOBAL_POSITION_INT shows altitude relative to home                      |
| Clippy clean              | `cargo clippy --all-targets -- -D warnings` passes                       |
