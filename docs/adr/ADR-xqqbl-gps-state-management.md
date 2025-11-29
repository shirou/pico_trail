# ADR-xqqbl GPS State Management Pattern

## Metadata

- Type: ADR (Lite)
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-uyie8-gps-mavlink-telemetry](../requirements/FR-uyie8-gps-mavlink-telemetry.md)
  - [FR-cs42u-gps-navigation-state-access](../requirements/FR-cs42u-gps-navigation-state-access.md)
  - [NFR-5utah-gps-telemetry-performance](../requirements/NFR-5utah-gps-telemetry-performance.md)
  - [NFR-wwke9-gps-state-thread-safety](../requirements/NFR-wwke9-gps-state-thread-safety.md)
- Related Tasks:
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Context

- GPS driver runs as independent async task, updating position at 1-10Hz
- Multiple consumers need GPS data: telemetry handler, navigation subsystem, control loop
- Existing SystemState pattern (battery, mode, armed) uses static Mutex for shared state
- GPS data is small (GpsPosition \~24 bytes) and suitable for copy semantics
- Thread-safe access required without blocking control loop (50Hz)

## Success Metrics (optional)

- GPS state access latency < 100us
- No control loop timing regression (50Hz maintained)
- Review date: 2025-12-15

## Decision

We will extend SystemState with GPS fields (`gps_position: Option<GpsPosition>`, `gps_timestamp_us: u64`) following the existing battery/mode pattern. GPS driver updates state after successful NMEA parse; consumers read via existing `SYSTEM_STATE` static Mutex.

## Consequences

- Positive: Consistent with existing architecture (battery, mode in SystemState)
- Positive: Single source of truth for system state, simplifies testing
- Positive: No new synchronization primitives needed (reuse existing Mutex)
- Positive: GpsPosition is Copy-able, no allocation overhead
- Negative: SystemState grows slightly (\~32 bytes for GPS fields)
- Negative: All GPS reads go through same Mutex as battery/mode (acceptable contention)

## Open Questions (optional)

- [x] Should we use dedicated GPS Mutex vs SystemState Mutex? → Decided: Use SystemState for consistency
- [ ] Should GpsPosition include course_over_ground? → Next step: Implement per FR-4z22l in task design

## External References (optional)

- [Embassy Mutex](https://docs.embassy.dev/embassy-sync/git/default/mutex/struct.Mutex.html) - Embassy synchronization primitive documentation

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
