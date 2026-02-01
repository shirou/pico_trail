# NFR-00015 Arm Event Logging Non-Blocking

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Arm event logging shall not block arm operation or prevent vehicle arming in case of logging subsystem failure, ensuring that logging issues do not compromise operational availability.

## Rationale

Arm event logging provides critical audit trail data for post-flight analysis and safety investigations. However, logging must not prevent arming if the logging subsystem fails:

- **Operational Availability**: Vehicle must be operable even if SD card full, flash worn out, or logger crashed
- **Safety Priority**: Flying with degraded logging is safer than not flying when operationally necessary
- **Graceful Degradation**: Warn operator of logging failure but allow mission to proceed
- **ArduPilot Pattern**: ArduPilot logs arm events but continues arming even if logger fails

Flash writes can be slow (5-20ms) and unpredictable due to wear leveling and erase cycles. Asynchronous or timeout-based logging prevents these delays from blocking arm operation.

## User Story (if applicable)

The system shall write arm events to the log without blocking arm operation, and shall warn the operator but still complete arming if the logging subsystem fails, ensuring logging issues do not prevent critical missions.

## Acceptance Criteria

- [ ] Arm operation completes successfully even if logging subsystem is unavailable
- [ ] Arm operation completes successfully even if flash is full or write fails
- [ ] Log write is asynchronous (queued to background task) OR has timeout (< 15ms) if synchronous
- [ ] Operator receives warning (MAVLink STATUSTEXT) if log write fails
- [ ] No memory leak if log queue fills (drop oldest entries or limit queue size)
- [ ] Log write failure does not cause panic or abort arm sequence
- [ ] Performance verified: arm latency < 50ms even if log subsystem slow

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Reliability:**

- **Fault Tolerance**: Logging failure does not prevent arming
- **Graceful Degradation**: Warn operator but continue operation
- **Error Handling**: All log write errors caught and handled
- **Resource Management**: No memory leak if log queue fills

**Performance:**

- **Non-Blocking**: Log write queued or timeout-based
- **Timeout Budget**: Synchronous logging must complete within 15ms or timeout
- **Async Queue**: If asynchronous, queue should hold at least 100 entries (prevent overflow during burst logging)

**Implementation Strategies:**

**Option A: Asynchronous Logging with Queue (Recommended)**

```rust
/// Queue-based async arm event logging
pub struct ArmEventQueue {
    /// Ring buffer for arm events
    events: heapless::Deque<ArmEvent, 100>,
    /// Logger task handle
    logger_task: Option<embassy_executor::TaskHandle>,
}

impl SystemState {
    /// Log arm event (non-blocking)
    fn log_arm_event(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        let event = ArmEvent {
            timestamp_ms: self.post_arm_state.arm_time_ms,
            method,
            checks_performed,
        };

        // Attempt to queue event
        match self.log_queue.events.push_back(event) {
            Ok(()) => {
                defmt::debug!("Arm event queued for logging");
                Ok(())
            }
            Err(_) => {
                // Queue full - drop oldest event and retry
                self.log_queue.events.pop_front();
                self.log_queue.events.push_back(event)
                    .expect("Queue should have space after pop");

                defmt::warn!("Log queue full, dropped oldest arm event");

                // Warn operator
                self.send_warning("Log queue full, events dropped")?;
                Ok(())
            }
        }
    }
}

/// Background logging task
#[embassy_executor::task]
async fn log_writer_task(mut queue: ArmEventQueue, logger: &'static mut Logger) {
    loop {
        // Wait for event or timeout
        let event = match queue.events.pop_front() {
            Some(event) => event,
            None => {
                // No events, sleep briefly
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
        };

        // Write event to persistent storage
        match logger.write_arm_event(&event).await {
            Ok(()) => {
                defmt::debug!("Arm event logged: {:?}", event);
            }
            Err(e) => {
                defmt::error!("Failed to log arm event: {:?}", e);
                // Do not retry - continue processing queue
            }
        }
    }
}
```

**Pros**: Non-blocking, arm completes immediately (< 1us logging overhead)
**Cons**: More complex, requires background task and queue management

**Option B: Synchronous with Timeout (Fallback)**

```rust
/// Synchronous logging with timeout
fn log_arm_event(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    let event = ArmEvent {
        timestamp_ms: self.post_arm_state.arm_time_ms,
        method,
        checks_performed,
    };

    // Attempt write with 15ms timeout
    let write_start = timer.now_micros();
    let result = match timeout(
        Duration::from_millis(15),
        logger.write_arm_event(&event)
    ).await {
        Ok(Ok(())) => {
            let duration = timer.now_micros() - write_start;
            defmt::debug!("Arm event logged ({}us)", duration);
            Ok(())
        }
        Ok(Err(e)) => {
            defmt::error!("Log write failed: {:?}", e);
            self.send_warning("Arm event logging failed")?;
            Ok(()) // Continue arming despite log failure
        }
        Err(_timeout) => {
            defmt::warn!("Log write timeout (>15ms)");
            self.send_warning("Arm logging slow")?;
            Ok(()) // Continue arming despite timeout
        }
    };

    result
}
```

**Pros**: Simpler, no background task required
**Cons**: Still blocks for up to 15ms, timeout may fire during legitimate slow write

**Error Handling Strategy:**

```rust
/// Handle all logging errors gracefully
fn handle_log_error(&mut self, error: LogError) -> Result<(), &'static str> {
    match error {
        LogError::FlashFull => {
            defmt::error!("Flash full, cannot log arm event");
            self.send_warning("Log storage full")?;
        }
        LogError::WriteFailed(reason) => {
            defmt::error!("Log write failed: {}", reason);
            self.send_warning("Log write failed")?;
        }
        LogError::QueueFull => {
            defmt::warn!("Log queue full, events dropped");
            self.send_warning("Log queue overflow")?;
        }
    }

    // Always return Ok - logging failure does not prevent arming
    Ok(())
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- Flash write may be slower (15-20ms)
- Asynchronous logging strongly recommended to avoid blocking
- Limited RAM (264KB) - keep log queue small (< 100 entries, \~4KB)

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz:

- Faster flash write (10-15ms)
- More RAM available for log queue
- Synchronous with timeout may be acceptable if < 15ms verified

### Cross-Platform

Arm event logging must not block arm operation on either platform. Use asynchronous logging or synchronous with timeout depending on platform capabilities.

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                  | Validation                            |
| ---------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ------------------------------------- |
| Log write blocks arm operation > 50ms    | High   | Medium     | Use asynchronous logging or timeout-based synchronous write | Measure arm latency with slow flash   |
| Log queue overflow loses events          | Medium | Medium     | Drop oldest events, warn operator, size queue appropriately | Test with burst arming (100x in loop) |
| Background logger task crashes           | Medium | Low        | Watchdog monitoring, restart logger task if crash detected  | Inject logger faults, verify recovery |
| Flash write failure prevents arming      | High   | Low        | Catch all errors, continue arming, warn operator            | Test with full flash, write-protected |
| Memory leak if log queue grows unbounded | Medium | Low        | Use fixed-size queue (heapless::Deque), drop oldest on full | Verify no allocation, test queue fill |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Arm event for logging
#[derive(Clone, Copy, Debug)]
pub struct ArmEvent {
    pub timestamp_ms: u32,
    pub method: ArmMethod,
    pub checks_performed: bool,
}

/// Log queue for asynchronous arm event logging
pub struct ArmEventQueue {
    /// Fixed-size ring buffer (no heap allocation)
    events: heapless::Deque<ArmEvent, 100>,
}

impl ArmEventQueue {
    /// Queue arm event for background logging (non-blocking)
    pub fn queue_event(&mut self, event: ArmEvent) -> Result<(), &'static str> {
        match self.events.push_back(event) {
            Ok(()) => Ok(()),
            Err(_) => {
                // Queue full - drop oldest
                self.events.pop_front();
                self.events.push_back(event)
                    .expect("Should have space after pop");
                Err("Log queue full, oldest event dropped")
            }
        }
    }

    /// Pop next event for logging (called by background task)
    pub fn pop_event(&mut self) -> Option<ArmEvent> {
        self.events.pop_front()
    }
}

/// Background logger task
#[embassy_executor::task]
async fn arm_event_logger(
    queue: &'static mut ArmEventQueue,
    logger: &'static mut Logger
) {
    loop {
        // Wait for event
        if let Some(event) = queue.pop_event() {
            // Write to persistent storage (may block, but this is background task)
            match logger.write_arm_event(&event).await {
                Ok(()) => {
                    defmt::info!("Arm event logged: {:?}", event);
                }
                Err(e) => {
                    defmt::error!("Log write failed: {:?} (event lost)", e);
                    // Continue processing queue - do not retry
                }
            }
        } else {
            // No events, sleep briefly
            Timer::after(Duration::from_millis(10)).await;
        }
    }
}
```

**Integration with Post-Arm Init:**

```rust
impl SystemState {
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp
        self.record_timestamp()?;

        // 2. Queue arm event for logging (non-blocking, < 1us)
        let event = ArmEvent {
            timestamp_ms: self.post_arm_state.arm_time_ms,
            method,
            checks_performed,
        };

        if let Err(e) = self.log_queue.queue_event(event) {
            defmt::warn!("{}", e);
            self.send_warning("Log queue full")?;
        }

        // 3. Continue with other init steps
        self.initialize_actuators()?;
        self.notify_subsystems()?;
        self.update_gpio()?;

        if !checks_performed {
            self.send_warning("Arming checks disabled")?;
        }

        Ok(())
    }
}
```

**Testing Strategy:**

```rust
#[test]
fn test_arm_with_log_failure() {
    let mut state = SystemState::new();

    // Simulate logging failure
    state.logger.set_mode(LogMode::AlwaysFail);

    // Arm should still succeed
    let result = state.arm(ArmMethod::GcsCommand, true);
    assert!(result.is_ok(), "Arm should succeed even if logging fails");

    // Verify armed state
    assert!(state.is_armed());

    // Verify warning sent
    assert!(state.warnings.contains("Log write failed"));
}

#[test]
fn test_arm_with_log_timeout() {
    let mut state = SystemState::new();

    // Simulate slow logging (> 15ms)
    state.logger.set_mode(LogMode::SlowWrite(Duration::from_millis(20)));

    let start = timer.now_micros();
    let result = state.arm(ArmMethod::GcsCommand, true);
    let duration = timer.now_micros() - start;

    // Arm should succeed within timeout
    assert!(result.is_ok());
    assert!(duration < 50_000, "Arm should not block > 50ms");

    // Verify armed state
    assert!(state.is_armed());
}

#[test]
fn test_log_queue_overflow() {
    let mut state = SystemState::new();
    let mut queue = ArmEventQueue::new();

    // Fill queue to capacity
    for i in 0..100 {
        let event = ArmEvent {
            timestamp_ms: i,
            method: ArmMethod::GcsCommand,
            checks_performed: true,
        };
        assert!(queue.queue_event(event).is_ok());
    }

    // Next event should drop oldest
    let event = ArmEvent {
        timestamp_ms: 100,
        method: ArmMethod::GcsCommand,
        checks_performed: true,
    };
    let result = queue.queue_event(event);
    assert!(result.is_err()); // Error indicates oldest dropped
    assert_eq!(queue.events.len(), 100); // Queue still at capacity
    assert_eq!(queue.events.front().unwrap().timestamp_ms, 1); // Oldest (0) dropped
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::post_arm_init()
- `src/core/logging/` - Logger implementation and ArmEventQueue
- `src/core/logging/tasks.rs` - Background logging task

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
  N/A - No external references
