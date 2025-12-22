# FR-c6zp6 Log to STATUSTEXT Routing

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-nzsiz-logging-storage-and-routing](../analysis/AN-nzsiz-logging-storage-and-routing.md)
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-e301z-log-ring-buffer](../requirements/FR-e301z-log-ring-buffer.md)
  - [FR-onj2m-statustext-public-api](../requirements/FR-onj2m-statustext-public-api.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-mrt6j-log-storage-routing](../tasks/T-mrt6j-log-storage-routing/design.md)

## Requirement Statement

The system shall route log messages with WARNING or ERROR severity to MAVLink STATUSTEXT for transmission to ground control stations. This threshold is fixed and not configurable.

## Rationale

GCS operators need visibility into system warnings and errors without being overwhelmed by informational or debug messages. WARNING and ERROR levels indicate conditions that may require operator attention. INFO, DEBUG, and TRACE levels are developer-focused and would flood the GCS display, reducing situational awareness.

## User Story

As a **GCS operator monitoring the rover via Mission Planner**, I want **to see warning and error messages in the HUD**, so that **I can quickly identify problems requiring attention without being distracted by debug output**.

## Acceptance Criteria

- [ ] WARNING level (LogLevel::Warn) routed to STATUSTEXT
- [ ] ERROR level (LogLevel::Error) routed to STATUSTEXT
- [ ] INFO level (LogLevel::Info) NOT routed to STATUSTEXT
- [ ] DEBUG level (LogLevel::Debug) NOT routed to STATUSTEXT
- [ ] TRACE level (LogLevel::Trace) NOT routed to STATUSTEXT
- [ ] Routing threshold hardcoded (no runtime configuration)
- [ ] Integration with StatusNotifier from AN-e8x8h
- [ ] Messages appear in Mission Planner HUD

## Technical Details

### Functional Requirement Details

**Routing Logic:**

```rust
impl LogRouter {
    pub fn route(&mut self, msg: &LogMessage) {
        // Always buffer all messages
        self.buffer_sink.push(msg.clone());

        // USB serial: all levels (when usb_serial feature enabled)
        #[cfg(feature = "usb_serial")]
        if let Some(ref mut usb) = self.usb_sink {
            usb.push(msg);
        }

        // STATUSTEXT: WARNING and above only (fixed threshold)
        if msg.level >= LogLevel::Warn {
            if let Some(ref mut st) = self.statustext_sink {
                st.push(msg);
            }
        }
    }
}
```

**LogLevel Ordering:**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Trace = 0,  // Lowest severity
    Debug = 1,
    Info = 2,
    Warn = 3,   // Routed to STATUSTEXT
    Error = 4,  // Routed to STATUSTEXT (highest)
}
```

**Severity Mapping to MAVLink:**

| LogLevel | MAVLink Severity         | Routed to STATUSTEXT |
| -------- | ------------------------ | -------------------- |
| Error    | MAV_SEVERITY_ERROR (3)   | Yes                  |
| Warn     | MAV_SEVERITY_WARNING (4) | Yes                  |
| Info     | MAV_SEVERITY_INFO (6)    | No                   |
| Debug    | MAV_SEVERITY_DEBUG (7)   | No                   |
| Trace    | N/A                      | No                   |

## Platform Considerations

### Embedded (RP2350)

- StatusTextSink wraps StatusNotifier from AN-e8x8h
- Message forwarded to STATUSTEXT queue for async transmission
- No blocking in routing path

### Host Tests

- Mock StatusTextSink captures routed messages
- Verify only WARN and ERROR levels are forwarded
- Test boundary conditions (INFO should not route)

### Cross-Platform

- Routing logic identical on all platforms
- Threshold comparison uses LogLevel ordering

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                | Validation                       |
| ----------------------------------------- | ------ | ---------- | ----------------------------------------- | -------------------------------- |
| STATUSTEXT queue overflow from log burst  | Low    | Medium     | STATUSTEXT has its own queue management   | Test with rapid WARN/ERROR burst |
| INFO accidentally routed (comparison bug) | Medium | Low        | Use PartialOrd derive, unit test boundary | Test INFO level explicitly       |
| Message truncation (log > 50 chars)       | Low    | Medium     | STATUSTEXT chunking handles long messages | Test long warning messages       |

## Implementation Notes

**Preferred Patterns:**

- Derive `PartialOrd` for LogLevel for clean comparison
- Use single `>=` comparison for routing decision
- Bridge to StatusNotifier without duplicating queue logic

**Known Pitfalls:**

- Do not add runtime configuration - threshold is intentionally fixed
- Do not duplicate STATUSTEXT queue - use existing StatusNotifier

**Related Code:**

- StatusNotifier: `src/communication/mavlink/status_notifier.rs`
- Current logging: `src/core/logging.rs`

## External References

- [MAVLink STATUSTEXT](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [RFC-5424 Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
