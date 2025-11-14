# NFR-fc2tw STATUSTEXT Host Test Support

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-onj2m-statustext-public-api](../requirements/FR-onj2m-statustext-public-api.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-eiuvv-statustext-implementation](../tasks/T-eiuvv-statustext-implementation/README.md)

## Requirement Statement

The STATUSTEXT notification system shall work in host test environments (x86_64 / aarch64 with std), providing a mock notification sink for capturing and verifying messages without requiring embedded hardware or MAVLink transport.

## Rationale

Developers need to test notification logic without hardware. Host tests run quickly on development machines, enabling rapid iteration and CI automation. A mock notification sink allows tests to verify message content, severity levels, and notification counts without full embedded build or physical hardware. This accelerates development and improves test coverage.

## User Story

As a **developer writing tests**, I want **to capture STATUSTEXT messages in host tests**, so that **I can verify notification behavior without requiring embedded hardware or full system integration**.

## Acceptance Criteria

- [ ] Notification API compiles and runs in host tests (x86_64 target)
- [ ] Mock notification sink captures messages during test execution
- [ ] Tests can retrieve captured messages (content, severity, timestamp)
- [ ] Tests can verify message count
- [ ] Tests can verify message order (FIFO)
- [ ] Tests can verify truncation behavior (200-char limit)
- [ ] Tests can verify severity level mapping
- [ ] Mock sink has same API as production notifier
- [ ] No embedded-specific dependencies required in test mode
- [ ] `cargo test --lib` runs successfully with notification tests

## Technical Details

### Non-Functional Requirement Details

**Usability:**

- Simple test API for capturing and verifying messages
- Clear error messages when assertions fail
- Minimal setup required in test code

**Compatibility:**

- Works with `cargo test` (std environment)
- Compatible with Rust test framework
- No special build flags required

**Mock Notification Sink:**

```rust
#[cfg(test)]
pub mod test_support {
    use super::*;

    pub struct MockNotifier {
        messages: Vec<CapturedMessage>,
    }

    pub struct CapturedMessage {
        pub severity: MavSeverity,
        pub text: String,
    }

    impl MockNotifier {
        pub fn new() -> Self {
            Self { messages: Vec::new() }
        }

        pub fn capture(&mut self, severity: MavSeverity, text: &str) {
            self.messages.push(CapturedMessage {
                severity,
                text: text.to_string(),
            });
        }

        pub fn get_messages(&self) -> &[CapturedMessage] {
            &self.messages
        }

        pub fn clear(&mut self) {
            self.messages.clear();
        }

        pub fn count(&self) -> usize {
            self.messages.len()
        }
    }
}
```

**Test Usage Example:**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prearm_error_notification() {
        let mut notifier = MockNotifier::new();

        // Simulate pre-arm check failure
        arming_check_battery(&mut notifier);

        // Verify notification sent
        assert_eq!(notifier.count(), 1);
        let msg = &notifier.get_messages()[0];
        assert_eq!(msg.severity, MavSeverity::ERROR);
        assert!(msg.text.starts_with("PreArm: Battery"));
    }

    #[test]
    fn test_message_truncation() {
        let mut notifier = MockNotifier::new();

        // Send message > 200 chars
        let long_msg = "A".repeat(250);
        notifier.capture(MavSeverity::ERROR, &long_msg);

        // Verify truncation
        let msg = &notifier.get_messages()[0];
        assert_eq!(msg.text.len(), 200);
        assert!(msg.text.ends_with("..."));
    }
}
```

**Conditional Compilation:**

```rust
// Production (embedded target)
#[cfg(feature = "pico2_w")]
pub fn send_error(text: &str) {
    NOTIFIER.lock(|n| n.borrow_mut().enqueue(MavSeverity::ERROR, text));
}

// Test (host target)
#[cfg(all(test, not(feature = "pico2_w")))]
pub fn send_error(text: &str) {
    TEST_NOTIFIER.with(|n| n.borrow_mut().capture(MavSeverity::ERROR, text));
}
```

## Platform Considerations

### Host (x86_64 / aarch64)

- Critical for developer productivity
- Uses std (Vec, String) for simplicity in tests
- No performance constraints (tests run on powerful machines)

### Embedded (RP2350)

- Production code uses heapless structures (see NFR-ssp9q)
- Test code not included in embedded binary
- Conditional compilation separates test from production code

### Cross-Platform

- Test API identical on Linux, macOS, Windows
- Mock notifier behavior consistent across platforms

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                            | Validation                                |
| ------------------------------------------------------ | ------ | ---------- | ----------------------------------------------------- | ----------------------------------------- |
| Test code accidentally included in embedded binary     | High   | Low        | Use `#[cfg(test)]` guards strictly                    | Check binary size, verify no test symbols |
| Mock notifier behavior differs from production         | Medium | Medium     | Keep mock API identical to production                 | Integration tests on hardware             |
| Tests pass but production fails (behavior mismatch)    | High   | Medium     | Supplement host tests with embedded integration tests | Hardware-in-loop testing                  |
| Conditional compilation complexity confuses developers | Low    | Medium     | Document patterns clearly, provide examples           | Developer documentation, code comments    |
| Test dependencies conflict with embedded dependencies  | Medium | Low        | Use feature flags to separate test/production deps    | CI builds both configurations             |

## Implementation Notes

**Preferred Patterns:**

- Use `#[cfg(test)]` for test-only code
- Use thread-local storage for test notifier (`thread_local!`)
- Keep mock API identical to production API
- Use feature flags for conditional compilation

**Known Pitfalls:**

- Do not rely solely on host tests (also test on hardware)
- Do not use `#[cfg(test)]` in production code paths
- Do not assume test behavior matches embedded behavior exactly

**Test Organization:**

```rust
// src/communication/mavlink/status_notifier.rs

// Production code (no_std)
#[cfg(not(test))]
pub struct StatusNotifier {
    queue: heapless::Deque<QueuedMessage, 16>,
}

// Test support (std)
#[cfg(test)]
pub mod test_support {
    pub struct MockNotifier {
        messages: Vec<CapturedMessage>,
    }
}

// Tests
#[cfg(test)]
mod tests {
    use super::test_support::*;

    #[test]
    fn test_notification() {
        // Test code here
    }
}
```

**Related Code:**

- Existing test patterns: `src/core/logging.rs` (already has host/embedded split)
- Test utilities: `tests/` directory

## External References

- [Rust Conditional Compilation](https://doc.rust-lang.org/reference/conditional-compilation.html)
- [Embedded Testing Patterns](https://docs.rust-embedded.org/book/start/qemu.html)
- [Rust Testing Documentation](https://doc.rust-lang.org/book/ch11-00-testing.html)
