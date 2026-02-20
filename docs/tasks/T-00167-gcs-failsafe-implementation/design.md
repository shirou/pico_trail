# T-00167 GCS Communication Lost Failsafe Design

## Metadata

- Type: Design
- Status: Done

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Add a `GcsFailsafeChecker` to the core crate following the established `BatteryFailsafeChecker` pattern, fix `FailsafeAction` enum numbering to match ArduPilot, implement two-stage timeout detection (`FS_GCS_TIMEOUT` + `FS_TIMEOUT`), and wire the checker into `MavlinkLoopRunner` alongside the existing battery failsafe.

## Success Metrics

- [x] `FailsafeAction` enum matches ArduPilot: 0=None, 1=RTL, 2=Hold
- [x] Two-stage timeout: `FS_GCS_TIMEOUT` (5.0s) + `FS_TIMEOUT` (1.5s)
- [x] GCS failsafe triggers only when armed
- [x] Never-seen guard prevents false trigger when GCS never connected
- [x] Mode exemption returns `Activated` in Hold/RTL/SmartRTL (caller sends STATUSTEXT but skips mode change)
- [x] Recovery clears immediately on heartbeat resume
- [x] STATUSTEXT notifications sent on activation and recovery
- [x] Firmware `ArmedStateMonitor` GCS tracking removed

## Background and Current State

- **Battery failsafe pattern**: `BatteryFailsafeChecker` (checker struct) + `MavlinkLoopRunner` (runner integration) is the established pattern for failsafe logic in `crates/core/src/autopilot/`
- **`FailsafeAction` enum**: Currently `None=0, Hold=1, RTL=2, Disarm=3` -- diverges from ArduPilot's `None=0, RTL=1, Hold=2`
- **`FailsafeParams`**: Has `FS_ACTION` (default Hold=1), `FS_TIMEOUT` (5.0s), `FS_GCS_ENABLE` (1) -- `FS_TIMEOUT` conflates ArduPilot's detection and persistence timeouts
- **`ConnectionState`**: In `crates/core/src/communication/dispatcher.rs` with `last_heartbeat_us`, `heartbeat_count`, `connected` fields -- data source for checker
- **`ArmedStateMonitor`**: In `crates/firmware/src/core/arming/monitoring.rs` has duplicate `gcs_last_heartbeat_ms` tracking with TODO for failsafe action -- to be removed
- **`MavlinkLoopRunner`**: Currently owns only `BatteryFailsafeChecker` and `was_armed` -- will be extended with `GcsFailsafeChecker`

## Proposed Design

### Parameter Changes (`crates/core/src/parameters/failsafe.rs`)

**`FailsafeAction` enum** -- reorder to match ArduPilot:

```rust
pub enum FailsafeAction {
    None = 0,
    RTL = 1,    // was 2
    Hold = 2,   // was 1
    Disarm = 3, // unchanged
}
```

**`FailsafeParams` struct** -- add `gcs_timeout` field:

```rust
pub struct FailsafeParams {
    pub action: u8,        // FS_ACTION (default 2=Hold)
    pub timeout: f32,      // FS_TIMEOUT (default 1.5s, persistence)
    pub gcs_timeout: f32,  // FS_GCS_TIMEOUT (default 5.0s, detection)
    pub gcs_enable: bool,  // FS_GCS_ENABLE (default true)
}
```

**Parameter registration changes**:

- `FS_ACTION`: default `FailsafeAction::Hold as i32` (now 2, was 1)
- `FS_TIMEOUT`: rename registration from 5.0s to 1.5s (persistence timeout)
- `FS_GCS_TIMEOUT`: new parameter, 5.0s default (detection timeout)
- `FS_GCS_ENABLE`: unchanged (default 1)

### GCS Failsafe Checker (`crates/core/src/autopilot/gcs_failsafe.rs`)

New module following `BatteryFailsafeChecker` pattern:

```rust
/// Action returned by GCS failsafe check
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GcsFailsafeAction {
    /// No action needed
    None,
    /// Set vehicle mode (Hold or RTL)
    SetMode(FlightMode),
    /// Failsafe activated but no mode change needed (already in Hold/RTL/SmartRTL)
    Activated,
    /// Failsafe cleared (heartbeat resumed)
    Cleared,
}

/// Configuration for GCS failsafe
#[derive(Debug, Clone)]
pub struct GcsFailsafeConfig {
    pub enabled: bool,
    pub gcs_timeout_us: u64,  // FS_GCS_TIMEOUT in microseconds
    pub persistence_us: u64,  // FS_TIMEOUT in microseconds
    pub action: FailsafeAction,
}

/// Stateful GCS failsafe checker
pub struct GcsFailsafeChecker {
    config: GcsFailsafeConfig,
    /// Whether the GCS loss condition has been detected (stage 1)
    condition_detected: bool,
    /// Timestamp when condition was first detected
    condition_start_us: u64,
    /// Whether failsafe action has been executed (sticky until cleared)
    failsafe_active: bool,
}
```

**`check()` method signature** (follows `BatteryFailsafeChecker::check()` pattern -- caller passes data as parameters):

```rust
pub fn check(
    &mut self,
    last_heartbeat_us: u64,
    heartbeat_count: u32,
    current_time_us: u64,
    is_armed: bool,
    current_mode: FlightMode,
) -> GcsFailsafeAction
```

**Two-stage detection logic**:

1. Skip if `!is_armed` or `!config.enabled` or `heartbeat_count == 0` (never-seen guard)
2. Calculate `heartbeat_age = current_time_us - last_heartbeat_us`
3. If `heartbeat_age >= config.gcs_timeout_us`:
   - Stage 1: Set `condition_detected = true`, record `condition_start_us`
   - Stage 2: If `current_time_us - condition_start_us >= config.persistence_us`:
     - If `!failsafe_active`: set `failsafe_active = true`
       - Mode exemption: if current mode is Hold/RTL/SmartRTL, return `Activated` (failsafe state tracked but no mode change)
       - Otherwise: return `SetMode` based on `config.action`
4. If heartbeat is within threshold (age < gcs_timeout):
   - Clear `condition_detected`
   - If `failsafe_active` was true: set `failsafe_active = false`, return `Cleared`
5. Return `None`

### MavlinkLoopRunner Integration (`crates/core/src/autopilot/mavlink_runner.rs`)

Extend the runner to own a `GcsFailsafeChecker`:

```rust
pub struct MavlinkLoopRunner {
    battery_failsafe: BatteryFailsafeChecker,
    gcs_failsafe: GcsFailsafeChecker,
    was_armed: bool,
}
```

New `check_gcs_failsafe()` method:

```rust
pub fn check_gcs_failsafe(
    &mut self,
    last_heartbeat_us: u64,
    heartbeat_count: u32,
    current_time_us: u64,
    is_armed: bool,
    current_mode: FlightMode,
    param_store: &ParameterStore,
) -> GcsFailsafeAction
```

- On arm transition: reload `GcsFailsafeConfig` from `FailsafeParams` and reset checker
- Delegate to `GcsFailsafeChecker::check()`

### STATUSTEXT Notifications

Use existing `status_notifier` pattern (already in core):

- On `SetMode(mode)`: `send_warning("Failsafe: GCS Lost - Hold")` or `"- RTL"` based on action
- On `Activated`: `send_warning("Failsafe: GCS Lost - already in {mode}")` -- notifies even though no mode change occurred
- On `Cleared`: `send_info("GCS Failsafe Cleared")`
- On `SetMode`/`Activated`: `log_warn!("GCS failsafe triggered: {action}")`
- On `Cleared`: `log_info!("GCS failsafe cleared")`

Notifications are sent from the caller (where `check_gcs_failsafe()` is called) based on the returned `GcsFailsafeAction`, not from inside the checker. This keeps the checker pure and testable.

**Important**: The `Activated` variant ensures the caller always sends an activation STATUSTEXT when failsafe triggers, even if the vehicle is already in a safe mode. Without this, recovery would send "GCS Failsafe Cleared" without a prior activation notification, confusing the operator.

### Firmware Cleanup (`crates/firmware/src/core/arming/monitoring.rs`)

Remove `gcs_last_heartbeat_ms` field from `ArmedStateMonitor` and any GCS-related logic in `update_slow()`. Core's `GcsFailsafeChecker` is now the single source of truth for GCS failsafe detection.

### Data Flow

```
HEARTBEAT received in MessageDispatcher::dispatch()
    --> ConnectionState.last_heartbeat_us updated
    --> ConnectionState.heartbeat_count incremented

Every 100ms (in caller's control loop):
    caller reads:
        last_heartbeat_us = dispatcher.connection().last_heartbeat_us
        heartbeat_count = dispatcher.connection().heartbeat_count
        is_armed = SYSTEM_STATE.is_armed()
        current_mode = SYSTEM_STATE.mode()

    MavlinkLoopRunner::check_gcs_failsafe(...)
        --> GcsFailsafeChecker::check(...)
        --> Returns GcsFailsafeAction

    Caller handles action:
        SetMode(mode) --> SYSTEM_STATE.set_mode(mode) + STATUSTEXT warning
        Activated --> STATUSTEXT warning (no mode change, already safe)
        Cleared --> STATUSTEXT info "GCS Failsafe Cleared"
        None --> no-op
```

### Error Handling

- Invalid `FS_ACTION` value (> 2 and != 3): treated as `FailsafeAction::Hold` (safe default)
- **`FS_ACTION = 3` (Disarm) for GCS failsafe**: treated as `FailsafeAction::Hold` (safe fallback). Disarming on GCS loss while in motion is dangerous. The Disarm action is retained in the `FailsafeAction` enum for battery critical failsafe but is not a valid GCS failsafe action. The checker maps Disarm → Hold when building `GcsFailsafeConfig`
- `last_heartbeat_us == 0` with `heartbeat_count > 0`: should not occur; treated as valid data
- `current_time_us < last_heartbeat_us` (clock non-monotonicity): treated as no timeout (heartbeat age would underflow to a large value with unsigned subtraction, but checker uses `current_time_us.saturating_sub(last_heartbeat_us)` to return 0, preventing false trigger)
- Timer overflow: u64 microseconds won't overflow for \~584,942 years

### Parameter Migration

The `FailsafeAction` enum reorder (Hold: 1→2, RTL: 2→1) is a **breaking change** for stored parameters. If a user previously saved `FS_ACTION=1` (Hold), after update it would be interpreted as RTL.

**Mitigation**: `from_store()` validates the loaded `FS_ACTION` value against the enum range. Since the project is pre-release with no deployed fleet, the migration strategy is:

1. Document the enum change in release notes
2. `from_store()` uses the loaded integer value directly (no special migration logic)
3. Users who have customized `FS_ACTION` must re-set it after flashing updated firmware
4. Default value (2=Hold) is safe regardless of stored state because `register_defaults()` is called on fresh parameter store initialization

### Concurrent Failsafe Behavior

When both GCS and battery failsafes activate simultaneously:

- **Both default to Hold**: No conflict. The first to trigger sets Hold mode; the second triggers but `set_mode(Hold)` is a no-op since already in Hold
- **GCS=RTL + Battery=Hold**: First-activated failsafe's mode takes effect. The second failsafe triggers `set_mode()` which may override. Since battery failsafe checks are more frequent (every voltage sample) and battery critical triggers immediately, battery typically wins. This is acceptable: Hold is a safer action than RTL for low battery
- **Unified priority framework**: Deferred to future work if a third failsafe type is added. Current behavior (last-write-wins via `set_mode()`) is safe because all supported actions (Hold, RTL) are safe modes

## Alternatives Considered

See [AN-00046](../../analysis/AN-00046-communication-lost-action.md) for full analysis:

1. **Extend ArmedStateMonitor (firmware only)** -- rejected: firmware-only, not SITL-testable, breaks architecture
2. **Unified failsafe framework** -- rejected: over-engineering for current needs (battery + GCS only)

Selected approach (Option B: GcsFailsafeChecker in core) follows the proven battery pattern and keeps logic in core for testability.

## Testing Strategy

### Unit Tests (`crates/core/src/autopilot/gcs_failsafe.rs`)

- Normal operation: heartbeats within timeout, no trigger
- Two-stage timeout: condition detected then persists to trigger
- Condition clears before persistence: no trigger
- Armed guard: disarmed state never triggers
- Never-seen guard: `heartbeat_count == 0` never triggers
- Disabled guard: `config.enabled == false` never triggers
- Mode exemption: Hold/RTL/SmartRTL return `Activated` (not `SetMode`)
- Mode exemption STATUSTEXT: `Activated` followed by `Cleared` on recovery (both notifications sent)
- Sticky trigger: only fires once per failsafe activation
- Recovery: failsafe clears when heartbeats resume, returns `Cleared`
- Arm transition: reset clears sticky flags and reloads config
- Rapid arm/disarm cycle: arm → timeout detection starts → disarm → re-arm → state correctly reset
- Disarm action fallback: `FS_ACTION=3` maps to Hold in `GcsFailsafeConfig`
- Clock non-monotonicity: `current_time_us < last_heartbeat_us` does not false-trigger (saturating_sub)

### Integration Tests (`crates/core/src/autopilot/mavlink_runner.rs`)

- `check_gcs_failsafe()` with normal heartbeats
- `check_gcs_failsafe()` with timeout triggering Hold action
- Arm transition resets GCS failsafe state
- Config reload from parameter store on arm

### SITL Integration Tests (`crates/sitl/`)

- Heartbeat loss triggers mode change to Hold (end-to-end: stop heartbeat → wait `FS_GCS_TIMEOUT + FS_TIMEOUT` → verify mode is Hold)
- Heartbeat resume clears failsafe (verify STATUSTEXT "GCS Failsafe Cleared")

### Parameter Tests (`crates/core/src/parameters/failsafe.rs`)

- `FailsafeAction` enum values match ArduPilot numbering
- `FS_GCS_TIMEOUT` registration and loading
- `FS_TIMEOUT` new default (1.5s)
- `FS_ACTION` new default (2=Hold in ArduPilot numbering)

## Open Questions

- [x] Should GCS failsafe checker have its own config struct or reuse `FailsafeParams`? --> Own `GcsFailsafeConfig` struct (following `BatteryFailsafeConfig` pattern) for encapsulation and pre-conversion of timeout values to microseconds
- [x] Should mode exemption be in the checker or the caller? --> In the checker (checker receives `current_mode` as parameter), keeping caller logic simple
