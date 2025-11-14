# FR-onj2m Public API for STATUSTEXT Messages

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-7qki7-statustext-chunking](../requirements/FR-7qki7-statustext-chunking.md)
  - [FR-e9yu9-statustext-message-queue](../requirements/FR-e9yu9-statustext-message-queue.md)
  - [FR-dbjjx-statustext-ardupilot-conventions](../requirements/FR-dbjjx-statustext-ardupilot-conventions.md)
  - [NFR-ssp9q-statustext-performance-memory](../requirements/NFR-ssp9q-statustext-performance-memory.md)
  - [NFR-e82vp-statustext-nostd](../requirements/NFR-e82vp-statustext-nostd.md)
  - [NFR-fc2tw-statustext-host-tests](../requirements/NFR-fc2tw-statustext-host-tests.md)
- Related Tasks:
  - [T-eiuvv-statustext-implementation](../tasks/T-eiuvv-statustext-implementation/README.md)

## Requirement Statement

The system shall provide a public API for sending MAVLink STATUSTEXT messages to ground control stations, accessible to all system components (arming system, failsafes, mode controllers, sensors).

## Rationale

Currently, only the CommandHandler can generate STATUSTEXT messages during force-arm/disarm operations. System components need the ability to report status, errors, and warnings to GCS operators for troubleshooting and situational awareness. Without a public API, critical information (pre-arm failures, failsafe triggers, sensor errors) cannot be communicated to operators.

## User Story

As a **rover system component** (arming checker, failsafe handler, mode controller), I want **to send status messages to the ground control station**, so that **operators receive real-time notifications about system state, errors, and warnings**.

## Acceptance Criteria

- [ ] Public functions exist for each RFC-5424 severity level: `send_emergency()`, `send_alert()`, `send_critical()`, `send_error()`, `send_warning()`, `send_notice()`, `send_info()`, `send_debug()`
- [ ] Each function accepts `&str` message text parameter
- [ ] API is accessible from arming system (`src/system/arming/`)
- [ ] API is accessible from failsafe system (to be implemented)
- [ ] API is accessible from mode system (`src/system/mode/`)
- [ ] API is thread-safe for use from interrupt handlers (with appropriate guards)
- [ ] Messages appear in Mission Planner and QGroundControl GCS software

## Technical Details

### Functional Requirement Details

**API Design:**

```rust
// Severity-specific convenience functions
pub fn send_emergency(text: &str);
pub fn send_alert(text: &str);
pub fn send_critical(text: &str);
pub fn send_error(text: &str);
pub fn send_warning(text: &str);
pub fn send_notice(text: &str);
pub fn send_info(text: &str);
pub fn send_debug(text: &str);

// Or generic function with severity parameter
pub fn send_statustext(severity: MavSeverity, text: &str);
```

**Integration Points:**

- Arming system: Report pre-arm check failures
- Failsafe system: Report failsafe triggers (GCS loss, battery low)
- Mode system: Announce mode changes and failures
- Sensor drivers: Report hardware initialization errors

**Error Conditions:**

- Queue full: Log warning internally, drop oldest message
- Message too long: Automatically chunk via FR-7qki7
- Invalid UTF-8: Replace invalid bytes with replacement character

## Platform Considerations

### Embedded (RP2350)

- Must work in no_std environment (see NFR-e82vp)
- Must not allocate heap memory (see NFR-ssp9q)
- Must complete quickly (<100µs, see NFR-ajg5b)

### Host Tests

- Must provide mock sink for capturing messages in tests (see NFR-fc2tw)
- Test framework can verify message content and severity

### Cross-Platform

- API signature and behavior identical on all targets
- Message content and format consistent across platforms

## Risks & Mitigation

| Risk                                               | Impact | Likelihood | Mitigation                                           | Validation                                     |
| -------------------------------------------------- | ------ | ---------- | ---------------------------------------------------- | ---------------------------------------------- |
| Queue overflow during burst of error messages      | Medium | Medium     | Implement priority queue, bypass for EMERGENCY level | Test with intentional message flood            |
| Mutex deadlock when calling from interrupt context | High   | Low        | Provide try_lock variant, document usage patterns    | Review lock acquisition paths, test edge cases |
| Message corruption if called concurrently          | High   | Low        | Use proper synchronization primitives                | Concurrent stress tests                        |
| API misuse (wrong severity level, formatting)      | Low    | Medium     | Provide clear documentation and examples             | Code review, developer documentation           |
| Performance impact on real-time scheduler (400 Hz) | Medium | Low        | Ensure queue push is O(1), measure actual overhead   | Benchmark with profiler, verify <100µs         |

## Implementation Notes

**Preferred Patterns:**

- Global static notifier with `Mutex<RefCell<StatusNotifier>>` for accessibility (per analysis Option A)
- Severity-specific helper functions for developer convenience
- Follow existing project patterns for global state management

**Known Pitfalls:**

- Do not call directly from interrupt context without try_lock variant
- Do not assume message is sent immediately (async via queue)
- Do not send messages faster than 10/second (rate limiting recommended)

**Related Code:**

- Existing private function: `src/communication/mavlink/handlers/command.rs:275`
- MAVLink state: `src/communication/mavlink/state.rs`
- Message dispatcher: `src/communication/mavlink/dispatcher.rs`

## External References

- [MAVLink STATUSTEXT Message Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [RFC-5424 Syslog Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
