# NFR-00023 Disarm Event Logging Detail

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Disarm events shall be logged with full context for post-flight analysis and safety investigations, including timestamp, disarm method, disarm reason, armed duration, and all information needed to reconstruct the disarm sequence.

## Rationale

Disarm event logs are critical for post-flight analysis and safety investigations. Insufficient logging detail makes it impossible to determine:

- **When did the vehicle disarm?** (timestamp for timeline reconstruction)
- **How was it disarmed?** (GCS command, RC switch, failsafe, forced)
- **Why was it disarmed?** (normal operation, battery failsafe, RC loss, crash, emergency stop)
- **How long was it armed?** (armed duration for flight time tracking)

ArduPilot logs comprehensive disarm events with timestamp, method, reason, and armed duration. This enables investigators to:

- Reconstruct timeline of events leading to incident
- Identify unsafe disarm patterns (repeated failsafe disarms, frequent emergency stops)
- Correlate disarm events with sensor data (battery voltage, RC signal strength)
- Validate operator procedures during post-flight review
- Track vehicle usage (armed duration for maintenance scheduling)

## User Story (if applicable)

The system shall log disarm events with timestamp, disarm method, disarm reason, armed duration, and any relevant context to enable post-flight investigators to reconstruct the disarm sequence and identify any unsafe patterns or contributing factors.

## Acceptance Criteria

- [ ] Log entry includes timestamp (milliseconds since boot, u32)
- [ ] Log entry includes disarm method (enum: GcsCommand, RcSwitch, Failsafe, ForceDisarm)
- [ ] Log entry includes disarm reason (enum: Normal, RcLoss, BatteryLow, BatteryCritical, GcsLoss, EmergencyStop, Forced)
- [ ] Log entry includes armed duration (milliseconds, u32)
- [ ] Log format is human-readable ASCII (comma-separated values)
- [ ] Log entry is parseable by automated tools (fixed format, no variations)
- [ ] Log entry fits within 80 characters (single line, easy to read)
- [ ] Log is written to persistent storage (survives power loss)
- [ ] Log is readable post-flight (no corruption, proper line endings)
- [ ] Logging failure does not prevent disarm from completing (safety priority)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Reliability:**

- **Auditability**: Complete disarm event history
- **Forensics**: Sufficient detail for incident investigation
- **Compliance**: Meets regulatory requirements for vehicle logging

**Usability:**

- **Human-Readable**: ASCII text, comma-separated values
- **Tool-Parseable**: Fixed format for automated analysis
- **Compact**: Single line per event, < 80 characters

**Log Format Specification:**

```
DISARM,<timestamp_ms>,<method>,<reason>,<duration_ms>\n

Where:
- timestamp_ms: u32 milliseconds since boot (e.g., "12345")
- method: String enum ("Unknown", "GcsCommand", "RcSwitch", "Failsafe", "ForceDisarm")
- reason: String enum ("Unknown", "Normal", "RcLoss", "BatteryLow", "BatteryCritical", "GcsLoss", "EmergencyStop", "Forced")
- duration_ms: u32 milliseconds vehicle was armed (e.g., "5000" = 5 seconds)

Example:
DISARM,12345,GcsCommand,Normal,5000
DISARM,23456,Failsafe,BatteryCritical,8500
DISARM,34567,ForceDisarm,EmergencyStop,120
```

**Implementation:**

```rust
/// Log disarm event with full detail
fn log_disarm_event(&mut self, timestamp_ms: u32, method: DisarmMethod,
                    reason: DisarmReason, duration_ms: u32)
                    -> Result<(), &'static str> {
    // Format method as string
    let method_str = match method {
        DisarmMethod::Unknown => "Unknown",
        DisarmMethod::GcsCommand => "GcsCommand",
        DisarmMethod::RcSwitch => "RcSwitch",
        DisarmMethod::Failsafe => "Failsafe",
        DisarmMethod::ForceDisarm => "ForceDisarm",
    };

    // Format reason as string
    let reason_str = match reason {
        DisarmReason::Unknown => "Unknown",
        DisarmReason::Normal => "Normal",
        DisarmReason::RcLoss => "RcLoss",
        DisarmReason::BatteryLow => "BatteryLow",
        DisarmReason::BatteryCritical => "BatteryCritical",
        DisarmReason::GcsLoss => "GcsLoss",
        DisarmReason::EmergencyStop => "EmergencyStop",
        DisarmReason::Forced => "Forced",
    };

    // Format log entry
    let log_entry = format!(
        "DISARM,{},{},{},{}\n",
        timestamp_ms,
        method_str,
        reason_str,
        duration_ms
    );

    // Write to persistent storage
    self.logger.write(log_entry.as_bytes())?;

    defmt::info!("Disarm event logged: {}", log_entry.trim_end());
    Ok(())
}
```

**Extended Format (Phase 2, optional):**

```
DISARM,<timestamp_ms>,<method>,<reason>,<duration_ms>,<battery_mv>,<mode>\n

Example:
DISARM,12345,GcsCommand,Normal,5000,12400,Manual
DISARM,23456,Failsafe,BatteryCritical,8500,10200,Auto

Where:
- battery_mv: Battery voltage in millivolts (e.g., "12400" = 12.4V)
- mode: Current flight mode ("Manual", "Hold", "Auto", etc.)
```

**Log Parsing Example:**

```rust
/// Parse disarm event log entry
#[derive(Debug, PartialEq)]
pub struct DisarmLogEntry {
    pub timestamp_ms: u32,
    pub method: DisarmMethod,
    pub reason: DisarmReason,
    pub duration_ms: u32,
}

impl DisarmLogEntry {
    /// Parse log line: "DISARM,12345,GcsCommand,Normal,5000"
    pub fn parse(line: &str) -> Result<Self, &'static str> {
        let parts: Vec<&str> = line.trim().split(',').collect();

        if parts.len() != 5 || parts[0] != "DISARM" {
            return Err("Invalid DISARM log format");
        }

        let timestamp_ms = parts[1]
            .parse::<u32>()
            .map_err(|_| "Invalid timestamp")?;

        let method = match parts[2] {
            "Unknown" => DisarmMethod::Unknown,
            "GcsCommand" => DisarmMethod::GcsCommand,
            "RcSwitch" => DisarmMethod::RcSwitch,
            "Failsafe" => DisarmMethod::Failsafe,
            "ForceDisarm" => DisarmMethod::ForceDisarm,
            _ => return Err("Invalid disarm method"),
        };

        let reason = match parts[3] {
            "Unknown" => DisarmReason::Unknown,
            "Normal" => DisarmReason::Normal,
            "RcLoss" => DisarmReason::RcLoss,
            "BatteryLow" => DisarmReason::BatteryLow,
            "BatteryCritical" => DisarmReason::BatteryCritical,
            "GcsLoss" => DisarmReason::GcsLoss,
            "EmergencyStop" => DisarmReason::EmergencyStop,
            "Forced" => DisarmReason::Forced,
            _ => return Err("Invalid disarm reason"),
        };

        let duration_ms = parts[4]
            .parse::<u32>()
            .map_err(|_| "Invalid duration")?;

        Ok(DisarmLogEntry {
            timestamp_ms,
            method,
            reason,
            duration_ms,
        })
    }
}

#[test]
fn test_log_parsing() {
    // Valid entry (normal disarm)
    let entry = DisarmLogEntry::parse("DISARM,12345,GcsCommand,Normal,5000").unwrap();
    assert_eq!(entry.timestamp_ms, 12345);
    assert_eq!(entry.method, DisarmMethod::GcsCommand);
    assert_eq!(entry.reason, DisarmReason::Normal);
    assert_eq!(entry.duration_ms, 5000);

    // Failsafe disarm
    let entry = DisarmLogEntry::parse("DISARM,23456,Failsafe,BatteryCritical,8500").unwrap();
    assert_eq!(entry.timestamp_ms, 23456);
    assert_eq!(entry.method, DisarmMethod::Failsafe);
    assert_eq!(entry.reason, DisarmReason::BatteryCritical);
    assert_eq!(entry.duration_ms, 8500);

    // Force disarm
    let entry = DisarmLogEntry::parse("DISARM,34567,ForceDisarm,EmergencyStop,120").unwrap();
    assert_eq!(entry.timestamp_ms, 34567);
    assert_eq!(entry.method, DisarmMethod::ForceDisarm);
    assert_eq!(entry.reason, DisarmReason::EmergencyStop);
    assert_eq!(entry.duration_ms, 120);

    // Invalid format
    assert!(DisarmLogEntry::parse("DISARM,invalid,data").is_err());
}
```

**Log Storage Requirements:**

- **Persistent Storage**: Flash or SD card (survives power loss)
- **Write Durability**: Immediate flush or sync after write
- **Capacity**: Minimum 1000 disarm events (assuming 50 bytes per event = 50KB)
- **Retention**: Logs preserved until explicitly cleared by operator
- **Corruption Protection**: Use checksums or journaling if available

**Log Analysis Tools (Post-Flight):**

```python
#!/usr/bin/env python3
"""Analyze disarm event logs for safety patterns"""

import sys
from collections import Counter

def analyze_disarm_log(log_file):
    with open(log_file, 'r') as f:
        lines = f.readlines()

    disarm_events = []
    for line in lines:
        if line.startswith("DISARM,"):
            parts = line.strip().split(',')
            disarm_events.append({
                'timestamp_ms': int(parts[1]),
                'method': parts[2],
                'reason': parts[3],
                'duration_ms': int(parts[4])
            })

    print(f"Total disarm events: {len(disarm_events)}")

    # Count by method
    methods = Counter(e['method'] for e in disarm_events)
    print(f"By method: {dict(methods)}")

    # Count by reason
    reasons = Counter(e['reason'] for e in disarm_events)
    print(f"By reason: {dict(reasons)}")

    # Calculate total armed time
    total_armed_ms = sum(e['duration_ms'] for e in disarm_events)
    total_armed_sec = total_armed_ms / 1000.0
    print(f"Total armed time: {total_armed_sec:.1f}s ({total_armed_sec/60:.1f}min)")

    # Detect failsafe disarms (warning sign)
    failsafe_disarms = [e for e in disarm_events if e['method'] == 'Failsafe']
    if len(failsafe_disarms) > 5:
        print(f"WARNING: {len(failsafe_disarms)} failsafe disarms detected!")
        failsafe_reasons = Counter(e['reason'] for e in failsafe_disarms)
        print(f"  Failsafe reasons: {dict(failsafe_reasons)}")

    # Detect emergency stops (critical warning)
    emergency_stops = [e for e in disarm_events if e['reason'] == 'EmergencyStop']
    if len(emergency_stops) > 0:
        print(f"CRITICAL: {len(emergency_stops)} emergency stop events detected!")

    # Detect short armed durations (< 1 second, potential issue)
    short_arms = [e for e in disarm_events if e['duration_ms'] < 1000]
    if len(short_arms) > 10:
        print(f"WARNING: {len(short_arms)} short armed durations (< 1s), potential arming issues")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <log_file>")
        sys.exit(1)

    analyze_disarm_log(sys.argv[1])
```

**Logging Persistence Policy:**

```rust
/// Determine if logging should persist after disarm
fn should_persist_logging(&self, reason: DisarmReason) -> bool {
    // Continue logging for failsafe-triggered disarms
    matches!(reason,
        DisarmReason::RcLoss |
        DisarmReason::BatteryCritical |
        DisarmReason::GcsLoss |
        DisarmReason::EmergencyStop
    )
}

/// Extend logging duration for post-disarm analysis
fn extend_logging_duration(&mut self) -> Result<(), &'static str> {
    // Keep logging active for additional time after disarm
    // to capture full failure sequence

    // ArduPilot uses LOG_DISARMED parameter:
    // - 0 = disable logging on disarm
    // - 1 = continue logging
    // - 2 = continue logging if armed log created

    // For pico_trail Phase 1:
    // - Normal disarm: stop logging after 5 seconds
    // - Failsafe disarm: continue logging for 30 seconds

    const EXTENDED_LOGGING_MS: u32 = 30_000; // 30 seconds

    self.logger.set_extended_duration(EXTENDED_LOGGING_MS)?;

    defmt::info!("Logging extended for {}ms for failsafe analysis", EXTENDED_LOGGING_MS);
    Ok(())
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- Flash storage for logs (2MB flash available)
- Use embedded-storage crate for flash abstraction
- Write durability via flash sync after critical events

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz:

- More flash available (4MB or more)
- Faster flash write operations
- Same logging format and tools

### Cross-Platform

Disarm event logging format must be identical on both platforms to enable consistent post-flight analysis tools.

## Risks & Mitigation

| Risk                                    | Impact   | Likelihood | Mitigation                                                        | Validation                                    |
| --------------------------------------- | -------- | ---------- | ----------------------------------------------------------------- | --------------------------------------------- |
| Log format changes break analysis tools | High     | Medium     | Freeze log format, version log file if format changes             | Document format, version log files            |
| Flash corruption loses log data         | High     | Low        | Use checksums, journaling, or redundant storage                   | Test with power loss during write             |
| Log capacity exceeded (> 1000 events)   | Medium   | Low        | Rotate logs or warn operator when near capacity                   | Test with 10,000 disarm events                |
| Insufficient detail for investigation   | High     | Low        | Include all critical fields (timestamp, method, reason, duration) | Review past incident reports                  |
| Log not human-readable (binary format)  | Medium   | Medium     | Use ASCII CSV format, test readability with text editor           | Open log with text editor, verify readable    |
| Logging failure prevents disarm         | Critical | Low        | Log errors but complete disarm, safety first                      | Test with full flash, verify disarm completes |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Log disarm event with full detail
fn log_disarm_event(&mut self, timestamp_ms: u32, method: DisarmMethod,
                    reason: DisarmReason, duration_ms: u32)
                    -> Result<(), &'static str> {
    // Format log entry (human-readable CSV)
    let mut buffer = [0u8; 64]; // 64 bytes for log entry
    let method_str = match method {
        DisarmMethod::Unknown => "Unknown",
        DisarmMethod::GcsCommand => "GcsCommand",
        DisarmMethod::RcSwitch => "RcSwitch",
        DisarmMethod::Failsafe => "Failsafe",
        DisarmMethod::ForceDisarm => "ForceDisarm",
    };

    let reason_str = match reason {
        DisarmReason::Unknown => "Unknown",
        DisarmReason::Normal => "Normal",
        DisarmReason::RcLoss => "RcLoss",
        DisarmReason::BatteryLow => "BatteryLow",
        DisarmReason::BatteryCritical => "BatteryCritical",
        DisarmReason::GcsLoss => "GcsLoss",
        DisarmReason::EmergencyStop => "EmergencyStop",
        DisarmReason::Forced => "Forced",
    };

    // Use core::fmt::Write to format into buffer (no heap)
    use core::fmt::Write;
    let mut writer = BufferWriter::new(&mut buffer);
    write!(
        &mut writer,
        "DISARM,{},{},{},{}\n",
        timestamp_ms,
        method_str,
        reason_str,
        duration_ms
    ).map_err(|_| "Log format failed")?;

    // Write to persistent storage
    self.logger.write(writer.as_bytes())?;

    // Flush to ensure durability (critical for disarm events)
    self.logger.flush()?;

    defmt::info!("Disarm event logged: {} bytes", writer.as_bytes().len());
    Ok(())
}
```

**Testing Strategy:**

```rust
#[test]
fn test_log_format() {
    let mut state = SystemState::new();

    // Arm and disarm with GCS command, normal reason
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    delay_ms(5000); // Armed for 5 seconds
    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();

    // Read log entry
    let log = state.logger.read_last_entry().unwrap();

    // Verify format
    assert!(log.starts_with("DISARM,"));
    assert!(log.contains(",GcsCommand,"));
    assert!(log.contains(",Normal,"));
    assert!(log.ends_with("\n"));

    // Parse back
    let entry = DisarmLogEntry::parse(&log).unwrap();
    assert_eq!(entry.method, DisarmMethod::GcsCommand);
    assert_eq!(entry.reason, DisarmReason::Normal);
    assert!(entry.duration_ms >= 5000 && entry.duration_ms < 5100); // ~5 seconds
}

#[test]
fn test_log_all_reasons() {
    let mut state = SystemState::new();

    let reasons = [
        DisarmReason::Normal,
        DisarmReason::RcLoss,
        DisarmReason::BatteryLow,
        DisarmReason::BatteryCritical,
        DisarmReason::GcsLoss,
        DisarmReason::EmergencyStop,
        DisarmReason::Forced,
    ];

    for reason in reasons {
        state.arm(ArmMethod::GcsCommand, true).unwrap();
        delay_ms(1000); // Armed for 1 second
        state.disarm(DisarmMethod::Failsafe, reason).unwrap();

        let log = state.logger.read_last_entry().unwrap();
        let entry = DisarmLogEntry::parse(&log).unwrap();
        assert_eq!(entry.reason, reason);
        assert!(entry.duration_ms >= 1000 && entry.duration_ms < 1100);
    }
}

#[test]
fn test_log_durability() {
    let mut state = SystemState::new();

    // Arm and disarm
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    delay_ms(2000);
    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();

    // Simulate power loss
    state.logger.simulate_power_loss();

    // Reinitialize system
    let mut state = SystemState::new();

    // Verify log entry survived
    let log = state.logger.read_last_entry().unwrap();
    let entry = DisarmLogEntry::parse(&log).unwrap();
    assert_eq!(entry.method, DisarmMethod::GcsCommand);
    assert_eq!(entry.reason, DisarmReason::Normal);
    assert!(entry.duration_ms >= 2000 && entry.duration_ms < 2100);
}

#[test]
fn test_logging_failure_does_not_prevent_disarm() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate full flash (logging will fail)
    state.logger.set_full(true);

    // Disarm should still succeed
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should complete despite logging failure");

    // Verify disarmed state
    assert!(!state.is_armed());
}
```

Related code areas:

- `src/core/logging/` - Log manager and file handling
- `src/communication/mavlink/state.rs` - Disarm event generation
- `tools/log_analysis/` - Post-flight log analysis scripts

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- ArduPilot Log Format: <https://ardupilot.org/copter/docs/logmessages.html>
- MAVLink Logging Standards: <https://mavlink.io/en/services/logging.html>
- ArduPilot LOG_DISARMED parameter: <https://ardupilot.org/copter/docs/parameters.html#log-disarmed>
