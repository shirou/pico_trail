# NFR-1wo70 STATUSTEXT Message Length Limits

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-7qki7-statustext-chunking](../requirements/FR-7qki7-statustext-chunking.md)
  - [NFR-ssp9q-statustext-performance-memory](../requirements/NFR-ssp9q-statustext-performance-memory.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-eiuvv-statustext-implementation](../tasks/T-eiuvv-statustext-implementation/README.md)

## Requirement Statement

The STATUSTEXT notification system shall enforce a maximum message length of 200 characters to limit chunking overhead, queue memory consumption, and transmission time, truncating longer messages with an ellipsis indicator.

## Rationale

Long messages consume queue capacity (16 messages), transmission bandwidth, and chunking overhead. A 200-character limit balances expressiveness with resource constraints:

- 200 chars = 4 chunks maximum (4 × 50 bytes)
- Typical error messages are 50-100 characters
- Excessive length indicates poor message design
- Fixed limit enables fixed-size heapless storage

ArduPilot similarly limits messages to practical lengths, rarely exceeding 150 characters for diagnostic messages.

## User Story

The system shall **truncate STATUSTEXT messages longer than 200 characters** to ensure **bounded resource usage and prevent excessive transmission overhead**.

## Acceptance Criteria

- [ ] Messages ≤200 characters sent as-is (no truncation)
- [ ] Messages >200 characters truncated to 197 characters + "..."
- [ ] Truncation occurs at enqueue time (before queueing)
- [ ] Truncated messages still chunk correctly via FR-7qki7 (4 chunks max)
- [ ] Internal warning logged when truncation occurs
- [ ] Message storage uses `heapless::String<200>` (fixed capacity)
- [ ] API documentation warns developers about 200-char limit
- [ ] Example messages in docs demonstrate good length (<150 chars)
- [ ] No runtime panics or errors for oversized messages

## Technical Details

### Non-Functional Requirement Details

**Reliability:**

- Maximum 4 chunks per message (200 chars ÷ 50 bytes)
- Queue of 16 messages = max 64 chunks worst-case
- Bounded transmission time: 64 chunks @ 57600 baud ≈ 1 second worst-case

**Performance:**

- Truncation check: O(1) (compare length)
- Truncation operation: O(1) (slice + append "...")
- No reallocation needed (fixed-size buffer)

**Usability:**

- Developer feedback via documentation and examples
- Internal warning when truncation occurs
- Ellipsis indicator shows message was truncated

**Truncation Algorithm:**

```rust
pub fn send_error(text: &str) {
    const MAX_LEN: usize = 200;
    let truncated = if text.len() > MAX_LEN {
        log_warn!("STATUSTEXT truncated: {} chars", text.len());
        let mut msg = heapless::String::<MAX_LEN>::new();
        msg.push_str(&text[..197]).unwrap();
        msg.push_str("...").unwrap();
        msg
    } else {
        heapless::String::from(text)
    };

    enqueue(MavSeverity::ERROR, &truncated);
}
```

**Example Messages:**

```rust
// Good: 89 characters (< 200)
send_error("PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT");

// Truncated: 250 characters -> 200
send_error("PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter. Please charge battery above minimum threshold or adjust parameter to lower value if battery is known good. Current configuration requires...");
// Becomes: "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter. Please charge battery above minimum threshold or adjust parameter to lower va..."
```

## Platform Considerations

### Embedded (RP2350)

- Critical for RAM conservation (264 KB total)
- 200-char limit enables fixed heapless::String<200>
- Truncation overhead negligible (<10 µs)

### Host Tests

- Test suite can verify truncation behavior
- Can test exact 200-char boundary
- Can verify ellipsis appended correctly

### Cross-Platform

- Truncation behavior identical on all platforms
- Length limit enforced consistently

## Risks & Mitigation

| Risk                                                      | Impact | Likelihood | Mitigation                                            | Validation                                    |
| --------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------- | --------------------------------------------- |
| 200-char limit too small for useful messages              | Medium | Low        | Analysis shows typical messages are <150 chars        | Review ArduPilot message lengths              |
| Truncation loses critical diagnostic information          | Medium | Medium     | Encourage concise messages, provide examples          | Code review, documentation                    |
| Developer unaware of limit, confused by truncation        | Low    | Medium     | Document limit prominently, log warning on truncation | API docs, examples, internal logging          |
| Truncation at wrong character boundary (UTF-8 corruption) | High   | Low        | Rust str slicing is UTF-8 safe                        | Test with multi-byte characters (emoji, etc.) |
| Queue capacity insufficient for 4-chunk messages          | Low    | Low        | 16 messages × 4 chunks = 64 chunks fits comfortably   | Worst-case queue analysis                     |

## Implementation Notes

**Preferred Patterns:**

- Truncate at enqueue time, not transmission time
- Use Rust's UTF-8-safe string slicing
- Log warning when truncation occurs (for developer feedback)
- Provide good examples of concise messages in docs

**Known Pitfalls:**

- Do not slice at arbitrary byte boundary (Rust prevents this)
- Do not truncate without ellipsis indicator (confusing)
- Do not increase limit beyond 200 without analysis (memory impact)

**Message Design Guidelines:**

- Keep messages concise (<100 chars ideal)
- Include critical values (voltage, threshold)
- Omit unnecessary words ("please", "the", "a")
- Use abbreviations when clear (GCS, IMU, EKF)
- Reference parameter names for additional context

**Examples:**

```rust
// Good (89 chars)
"PreArm: Battery voltage 9.8V below minimum 10.5V (BATT_ARM_VOLT)"

// Better (61 chars)
"PreArm: Battery 9.8V < 10.5V minimum (BATT_ARM_VOLT)"

// Excellent (48 chars)
"PreArm: Battery 9.8V < min 10.5V (BATT_ARM_VOLT)"
```

**Related Code:**

- Message queue: `heapless::String<200>`
- Chunking: FR-7qki7 (splits at 50-byte boundaries)

## External References

- [ArduPilot Message Examples](https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink)
- [Technical Writing - Conciseness](https://developers.google.com/tech-writing/one/words)
- [heapless::String documentation](https://docs.rs/heapless/latest/heapless/struct.String.html)
