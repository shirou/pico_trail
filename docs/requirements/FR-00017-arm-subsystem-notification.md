# FR-00017 Arm Event Subsystem Notification

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00015-actuator-armed-initialization](FR-00015-actuator-armed-initialization.md)
  - [FR-00049-post-arm-event-recording](FR-00049-post-arm-event-recording.md)

- Dependent Requirements: N/A - Notification is terminal post-arm step

- Related Analysis:
  - [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
  - [AN-00009-armed-state-monitoring](../analysis/AN-00009-armed-state-monitoring.md)
  - [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)

- Related Tasks:
  - [T-00008-arming-system-implementation](../tasks/T-00008-arming-system-implementation/README.md)

## Requirement Statement

The system shall notify all subsystems (monitoring, failsafe, mode system) of the arm event immediately after actuator initialization, with all notifications completing before `arm()` returns, enabling subsystems to reset baselines, enable armed-only detection, and adjust behavior for armed operation.

## Rationale

Subsystem notification synchronizes the entire system with the armed state:

- **Monitoring system**: Reset health baselines (battery voltage, RC signal quality) to current values at arm time, avoiding false failsafe triggers from pre-arm transients
- **Failsafe system**: Enable RC/GCS loss detection (failsafes typically disabled when disarmed, enabled when armed)
- **Mode system**: Adjust behavior for armed operation (e.g., Auto mode may only execute waypoints when armed)
- **State consistency**: All subsystems have consistent view of armed state, preventing race conditions

ArduPilot calls `on_successful_arming()` after post-arm initialization, allowing vehicle-specific code to react to arm event. Subsystems also subscribe to armed state changes via observer pattern.

## User Story (if applicable)

As a subsystem (monitoring, failsafe, mode), I want to be notified when the vehicle arms, so that I can adjust my behavior for armed operation, reset baselines, and enable armed-only safety features.

## Acceptance Criteria

- [ ] Notify monitoring system to reset health baselines (battery voltage, RC signal quality, sensor health)
- [ ] Notify failsafe system to enable RC/GCS loss detection (per FS_GCS_ENABLE, FS_CRASH_CHECK)
- [ ] Notify mode system to adjust behavior for armed operation (e.g., Auto starts executing waypoints)
- [ ] All notifications complete before `arm()` returns (synchronous notification)
- [ ] Notification includes arm timestamp (for timeout calculations) and arm method (for logging/debug)
- [ ] If notification fails (subsystem error), log warning but continue arming (non-critical)
- [ ] Test coverage: verify each subsystem receives notification, verify baselines reset, verify failsafes enabled

## Technical Details (if applicable)

### Functional Requirement Details

**Notification Mechanism:**

Phase 1 uses simple function calls to subsystems (synchronous notification):

```rust
impl SystemState {
    /// Post-arm initialization sequence
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp
        let arm_time_ms = get_time_ms();
        self.post_arm_state = PostArmState { arm_time_ms, arm_method: method, /* ... */ };

        // 2. Log arming event
        self.log_arm_event(arm_time_ms, method, checks_performed)?;

        // 3. Initialize actuators
        self.initialize_actuators()?;

        // 4. Notify subsystems (after actuators ready)
        self.notify_subsystems_armed(arm_time_ms, method)?;
        self.post_arm_state.subsystems_notified = true;

        // 5. Continue with other post-arm steps...
        Ok(())
    }

    /// Notify subsystems of arm event
    fn notify_subsystems_armed(&mut self, arm_time_ms: u32, method: ArmMethod)
                                -> Result<(), &'static str> {
        // Notify monitoring system to reset health baselines
        if let Err(e) = self.monitoring.on_armed(arm_time_ms) {
            warn!("Monitoring notification failed: {}", e);
            // Continue (non-critical failure)
        }

        // Notify failsafe system to enable RC/GCS loss detection
        if let Err(e) = self.failsafe.on_armed(arm_time_ms) {
            warn!("Failsafe notification failed: {}", e);
            // Continue (non-critical failure)
        }

        // Notify mode system to adjust behavior for armed operation
        if let Err(e) = self.mode_manager.on_armed(method) {
            warn!("Mode notification failed: {}", e);
            // Continue (non-critical failure)
        }

        info!("Subsystems notified of arm event");
        Ok(())
    }
}
```

**Monitoring System Response:**

```rust
impl MonitoringSystem {
    /// Called when vehicle arms
    pub fn on_armed(&mut self, arm_time_ms: u32) -> Result<(), &'static str> {
        // Reset battery voltage baseline to current value
        self.battery_baseline = self.battery.voltage();

        // Reset RC signal quality baseline
        self.rc_quality_baseline = self.rc.signal_quality();

        // Reset sensor health baselines (IMU, GPS, compass)
        self.sensor_health_baseline = self.sensors.health();

        // Store arm time for timeout calculations
        self.arm_time_ms = Some(arm_time_ms);

        info!("Monitoring baselines reset at arm");
        Ok(())
    }
}
```

**Failsafe System Response:**

```rust
impl FailsafeSystem {
    /// Called when vehicle arms
    pub fn on_armed(&mut self, arm_time_ms: u32) -> Result<(), &'static str> {
        // Enable RC loss detection (per FS_GCS_ENABLE parameter)
        if self.params.fs_gcs_enable > 0 {
            self.rc_loss_enabled = true;
        }

        // Enable GCS loss detection (per FS_GCS_ENABLE parameter)
        if self.params.fs_gcs_enable > 0 {
            self.gcs_loss_enabled = true;
        }

        // Enable battery failsafe (always enabled when armed)
        self.battery_failsafe_enabled = true;

        // Store arm time for failsafe timeout calculations
        self.arm_time_ms = Some(arm_time_ms);

        info!("Failsafes enabled at arm");
        Ok(())
    }
}
```

**Mode System Response:**

```rust
impl ModeManager {
    /// Called when vehicle arms
    pub fn on_armed(&mut self, method: ArmMethod) -> Result<(), &'static str> {
        // Notify current mode of arm event
        self.current_mode.on_armed(method)?;

        // Auto mode: start executing waypoints (if in Auto)
        // Manual mode: no behavior change
        // Hold mode: maintain position (if position control available)

        info!("Mode system notified of arm (method: {:?})", method);
        Ok(())
    }
}
```

**Notification Order:**

Order matters for safety:

1. **Monitoring first**: Reset baselines so failsafe doesn't trigger immediately after arm
2. **Failsafe second**: Enable detection after baselines reset
3. **Mode last**: Mode behavior may depend on monitoring/failsafe state

**Error Handling:**

Notification failures are non-critical:

- Log warning (don't propagate error to caller)
- Continue with arm operation
- Rationale: Notification failure shouldn't prevent arming (degraded operation better than no operation)

**Future Enhancement (Phase 2):**

Consider event bus for decoupled notification:

```rust
// Event-based notification (Phase 2)
self.event_bus.publish(Event::Armed { time_ms: arm_time_ms, method });

// Subsystems subscribe to armed event
self.monitoring.subscribe(EventType::Armed);
self.failsafe.subscribe(EventType::Armed);
self.mode_manager.subscribe(EventType::Armed);
```

Pros: Decoupled, extensible, supports async notification
Cons: Higher complexity, requires event bus infrastructure

Phase 1: Simple function calls (sufficient for initial implementation)

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

Notification mechanism platform-agnostic. Subsystem interfaces abstracted via traits.

## Risks & Mitigation

| Risk                                                    | Impact | Likelihood | Mitigation                                                                              | Validation                                                      |
| ------------------------------------------------------- | ------ | ---------- | --------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| Notification order wrong, failsafe triggers immediately | High   | Low        | Document notification order in ADR, monitoring first (reset baselines), then failsafe   | Test: arm with low battery, verify failsafe doesn't trigger     |
| Subsystem notification blocks arm operation (latency)   | Medium | Low        | Keep notifications fast (< 1 ms each), profile if latency issue                         | Benchmark: measure notification time, verify < 5 ms total       |
| Notification failure leaves subsystem in inconsistent   | Medium | Low        | Log warnings on failure, continue arming (degraded operation better than no operation)  | Test: simulate notification error, verify warning logged        |
| Race condition: subsystem accesses arm state before set | Medium | Very Low   | Arm state set BEFORE notification (synchronous), subsystems see consistent state        | Unit test: verify armed state visible in notification callbacks |
| Monitoring baseline reset too late, false failsafe      | High   | Low        | Monitor notification FIRST (before failsafe), reset baselines before enabling detection | Test: arm with transient battery drop, verify no false failsafe |
| Mode system assumes armed before actuators initialized  | High   | Very Low   | Notify mode AFTER actuators initialized, mode can safely command actuators              | Test: verify actuators ready when mode.on_armed() called        |
| Notification failure unnoticed (silent failure)         | Medium | Low        | Log warning for every notification failure (don't fail silently)                        | Review logs: verify warnings present for notification errors    |
| Subsystem not implemented, notification panics          | High   | Low        | Use Option or trait checks before calling on_armed(), graceful degradation              | Test: run with missing subsystems, verify no panics             |

## Implementation Notes

Preferred approaches:

- **Synchronous notification**: Simple function calls, Phase 1 sufficient
- **Order matters**: Monitoring → Failsafe → Mode (documented in ADR)
- **Non-critical failures**: Log warnings, continue arming (degraded operation acceptable)
- **Defense in depth**: Subsystems should be robust to late/missing notifications

Known pitfalls:

- **Notification order**: Wrong order can cause false failsafes (monitor baselines before enabling detection)
- **Blocking calls**: Keep notifications fast (< 1 ms each), avoid I/O or complex logic
- **Missing subsystems**: Check if subsystem exists before calling (graceful degradation)
- **State visibility**: Ensure armed state set BEFORE notification (subsystems expect consistent state)

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState post_arm_init() calls notify_subsystems_armed()
- `src/vehicle/monitoring/` - MonitoringSystem on_armed() implementation
- `src/vehicle/failsafe/` - FailsafeSystem on_armed() implementation
- `src/vehicle/modes/` - ModeManager and VehicleMode on_armed() implementation

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
  N/A - No external references
