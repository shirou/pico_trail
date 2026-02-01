# NFR-00014 Arm Event Logging Detail

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

Arm events shall be logged with sufficient detail for post-flight analysis and safety investigations, including timestamp, arming method, checks status, and any relevant context needed to reconstruct the arm sequence.

## Rationale

Arm event logs are critical for post-flight analysis and safety investigations. Insufficient logging detail makes it impossible to determine:

- **When did the vehicle arm?** (timestamp for timeline reconstruction)
- **How was it armed?** (RC rudder, GCS command, switch, forced)
- **Were safety checks performed?** (checks enabled/disabled)
- **What was the vehicle state?** (battery voltage, GPS status, mode)

ArduPilot logs comprehensive arm events with timestamp, method, checks status, and vehicle state. This enables investigators to:

- Reconstruct timeline of events leading to incident
- Identify unsafe arming patterns (repeated force-arm, checks disabled)
- Correlate arm events with sensor data (battery, GPS, RC signal)
- Validate operator procedures during post-flight review

## User Story (if applicable)

The system shall log arm events with timestamp, arming method, checks status, and vehicle state to enable post-flight investigators to reconstruct the arm sequence and identify any unsafe arming patterns or contributing factors.

## Acceptance Criteria

- [ ] Log entry includes timestamp (milliseconds since boot, u32)
- [ ] Log entry includes arming method (enum: GcsCommand, RcRudder, RcSwitch, ForceArm)
- [ ] Log entry includes checks status (bool: checks performed or bypassed)
- [ ] Log format is human-readable ASCII (comma-separated values)
- [ ] Log entry is parseable by automated tools (fixed format, no variations)
- [ ] Log entry fits within 80 characters (single line, easy to read)
- [ ] Log is written to persistent storage (survives power loss)
- [ ] Log is readable post-flight (no corruption, proper line endings)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Reliability:**

- **Auditability**: Complete arm event history
- **Forensics**: Sufficient detail for incident investigation
- **Compliance**: Meets regulatory requirements for vehicle logging

**Usability:**

- **Human-Readable**: ASCII text, comma-separated values
- **Tool-Parseable**: Fixed format for automated analysis
- **Compact**: Single line per event, < 80 characters

**Log Format Specification:**

```
ARM,<timestamp_ms>,<method>,<checks>\n

Where:
- timestamp_ms: u32 milliseconds since boot (e.g., "12345")
- method: String enum ("Unknown", "RcRudder", "GcsCommand", "RcSwitch", "ForceArm")
- checks: bool ("0" = bypassed, "1" = performed)

Example:
ARM,12345,GcsCommand,1
ARM,23456,ForceArm,0
```

**Implementation:**

```rust
/// Log arm event with full detail
fn log_arm_event(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    let timestamp_ms = self.post_arm_state.arm_time_ms;

    // Format method as string
    let method_str = match method {
        ArmMethod::Unknown => "Unknown",
        ArmMethod::RcRudder => "RcRudder",
        ArmMethod::GcsCommand => "GcsCommand",
        ArmMethod::RcSwitch => "RcSwitch",
        ArmMethod::ForceArm => "ForceArm",
    };

    // Format checks as 0/1
    let checks_byte = if checks_performed { '1' } else { '0' };

    // Format log entry
    let log_entry = format!(
        "ARM,{},{},{}\n",
        timestamp_ms,
        method_str,
        checks_byte
    );

    // Write to persistent storage
    self.logger.write(log_entry.as_bytes())?;

    defmt::info!("Arm event logged: {}", log_entry.trim_end());
    Ok(())
}
```

**Extended Format (Phase 2, optional):**

```
ARM,<timestamp_ms>,<method>,<checks>,<battery_mv>,<mode>\n

Example:
ARM,12345,GcsCommand,1,12400,Manual
ARM,23456,ForceArm,0,11800,Hold

Where:
- battery_mv: Battery voltage in millivolts (e.g., "12400" = 12.4V)
- mode: Current flight mode ("Manual", "Hold", "Auto", etc.)
```

**Log Parsing Example:**

```rust
/// Parse arm event log entry
#[derive(Debug, PartialEq)]
pub struct ArmLogEntry {
    pub timestamp_ms: u32,
    pub method: ArmMethod,
    pub checks_performed: bool,
}

impl ArmLogEntry {
    /// Parse log line: "ARM,12345,GcsCommand,1"
    pub fn parse(line: &str) -> Result<Self, &'static str> {
        let parts: Vec<&str> = line.trim().split(',').collect();

        if parts.len() != 4 || parts[0] != "ARM" {
            return Err("Invalid ARM log format");
        }

        let timestamp_ms = parts[1]
            .parse::<u32>()
            .map_err(|_| "Invalid timestamp")?;

        let method = match parts[2] {
            "Unknown" => ArmMethod::Unknown,
            "RcRudder" => ArmMethod::RcRudder,
            "GcsCommand" => ArmMethod::GcsCommand,
            "RcSwitch" => ArmMethod::RcSwitch,
            "ForceArm" => ArmMethod::ForceArm,
            _ => return Err("Invalid arm method"),
        };

        let checks_performed = match parts[3] {
            "0" => false,
            "1" => true,
            _ => return Err("Invalid checks status"),
        };

        Ok(ArmLogEntry {
            timestamp_ms,
            method,
            checks_performed,
        })
    }
}

#[test]
fn test_log_parsing() {
    // Valid entry
    let entry = ArmLogEntry::parse("ARM,12345,GcsCommand,1").unwrap();
    assert_eq!(entry.timestamp_ms, 12345);
    assert_eq!(entry.method, ArmMethod::GcsCommand);
    assert_eq!(entry.checks_performed, true);

    // Force arm without checks
    let entry = ArmLogEntry::parse("ARM,23456,ForceArm,0").unwrap();
    assert_eq!(entry.timestamp_ms, 23456);
    assert_eq!(entry.method, ArmMethod::ForceArm);
    assert_eq!(entry.checks_performed, false);

    // Invalid format
    assert!(ArmLogEntry::parse("ARM,invalid,data").is_err());
}
```

**Log Storage Requirements:**

- **Persistent Storage**: Flash or SD card (survives power loss)
- **Write Durability**: Immediate flush or sync after write
- **Capacity**: Minimum 1000 arm events (assuming 40 bytes per event = 40KB)
- **Retention**: Logs preserved until explicitly cleared by operator
- **Corruption Protection**: Use checksums or journaling if available

**Log Analysis Tools (Post-Flight):**

```python
#!/usr/bin/env python3
"""Analyze arm event logs for safety patterns"""

import sys
from collections import Counter

def analyze_arm_log(log_file):
    with open(log_file, 'r') as f:
        lines = f.readlines()

    arm_events = []
    for line in lines:
        if line.startswith("ARM,"):
            parts = line.strip().split(',')
            arm_events.append({
                'timestamp_ms': int(parts[1]),
                'method': parts[2],
                'checks': parts[3] == '1'
            })

    print(f"Total arm events: {len(arm_events)}")

    # Count by method
    methods = Counter(e['method'] for e in arm_events)
    print(f"By method: {dict(methods)}")

    # Count checks bypassed
    checks_bypassed = sum(1 for e in arm_events if not e['checks'])
    print(f"Checks bypassed: {checks_bypassed} ({100*checks_bypassed/len(arm_events):.1f}%)")

    # Detect repeated force-arm (warning sign)
    force_arms = [e for e in arm_events if e['method'] == 'ForceArm']
    if len(force_arms) > 5:
        print(f"WARNING: {len(force_arms)} force-arm events detected!")

    # Detect frequent arming (potential issue)
    if len(arm_events) > 100:
        print(f"WARNING: {len(arm_events)} arm events (potential arming issues)")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <log_file>")
        sys.exit(1)

    analyze_arm_log(sys.argv[1])
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

Arm event logging format must be identical on both platforms to enable consistent post-flight analysis tools.

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                                              | Validation                                 |
| --------------------------------------- | ------ | ---------- | ------------------------------------------------------- | ------------------------------------------ |
| Log format changes break analysis tools | High   | Medium     | Freeze log format, version log file if format changes   | Document format, version log files         |
| Flash corruption loses log data         | High   | Low        | Use checksums, journaling, or redundant storage         | Test with power loss during write          |
| Log capacity exceeded (> 1000 events)   | Medium | Low        | Rotate logs or warn operator when near capacity         | Test with 10,000 arm events                |
| Insufficient detail for investigation   | High   | Low        | Include all critical fields (timestamp, method, checks) | Review past incident reports               |
| Log not human-readable (binary format)  | Medium | Medium     | Use ASCII CSV format, test readability with text editor | Open log with text editor, verify readable |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Log arm event with full detail
fn log_arm_event(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    let timestamp_ms = self.post_arm_state.arm_time_ms;

    // Format log entry (human-readable CSV)
    let mut buffer = [0u8; 64]; // 64 bytes for log entry
    let method_str = match method {
        ArmMethod::Unknown => "Unknown",
        ArmMethod::RcRudder => "RcRudder",
        ArmMethod::GcsCommand => "GcsCommand",
        ArmMethod::RcSwitch => "RcSwitch",
        ArmMethod::ForceArm => "ForceArm",
    };

    let checks_char = if checks_performed { '1' } else { '0' };

    // Use core::fmt::Write to format into buffer (no heap)
    use core::fmt::Write;
    let mut writer = BufferWriter::new(&mut buffer);
    write!(
        &mut writer,
        "ARM,{},{},{}\n",
        timestamp_ms,
        method_str,
        checks_char
    ).map_err(|_| "Log format failed")?;

    // Write to persistent storage
    self.logger.write(writer.as_bytes())?;

    // Flush to ensure durability (critical for arm events)
    self.logger.flush()?;

    defmt::info!("Arm event logged: {} bytes", writer.as_bytes().len());
    Ok(())
}
```

**Testing Strategy:**

```rust
#[test]
fn test_log_format() {
    let mut state = SystemState::new();

    // Arm with GCS command, checks performed
    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Read log entry
    let log = state.logger.read_last_entry().unwrap();

    // Verify format
    assert!(log.starts_with("ARM,"));
    assert!(log.contains(",GcsCommand,"));
    assert!(log.ends_with(",1\n"));

    // Parse back
    let entry = ArmLogEntry::parse(&log).unwrap();
    assert_eq!(entry.method, ArmMethod::GcsCommand);
    assert_eq!(entry.checks_performed, true);
}

#[test]
fn test_log_all_methods() {
    let mut state = SystemState::new();

    let methods = [
        ArmMethod::GcsCommand,
        ArmMethod::RcRudder,
        ArmMethod::RcSwitch,
        ArmMethod::ForceArm,
    ];

    for method in methods {
        state.arm(method, true).unwrap();
        let log = state.logger.read_last_entry().unwrap();
        let entry = ArmLogEntry::parse(&log).unwrap();
        assert_eq!(entry.method, method);
        state.disarm().unwrap();
    }
}

#[test]
fn test_log_durability() {
    let mut state = SystemState::new();

    // Arm and immediately power cycle (simulate power loss)
    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate power loss
    state.logger.simulate_power_loss();

    // Reinitialize system
    let mut state = SystemState::new();

    // Verify log entry survived
    let log = state.logger.read_last_entry().unwrap();
    let entry = ArmLogEntry::parse(&log).unwrap();
    assert_eq!(entry.method, ArmMethod::GcsCommand);
}
```

**Log File Management:**

```rust
/// Log file manager
pub struct LogManager {
    /// Current log file
    current_file: LogFile,

    /// Total events logged
    event_count: usize,

    /// Maximum events before rotation
    max_events: usize,
}

impl LogManager {
    /// Write arm event
    pub fn write_arm_event(&mut self, event: &ArmEvent) -> Result<(), &'static str> {
        // Format event
        let log_entry = format!(
            "ARM,{},{},{}\n",
            event.timestamp_ms,
            event.method_str(),
            if event.checks_performed { '1' } else { '0' }
        );

        // Write to current file
        self.current_file.write(log_entry.as_bytes())?;
        self.current_file.flush()?;

        self.event_count += 1;

        // Rotate if needed
        if self.event_count >= self.max_events {
            self.rotate_log()?;
        }

        Ok(())
    }

    /// Rotate log file
    fn rotate_log(&mut self) -> Result<(), &'static str> {
        // Close current file
        self.current_file.close()?;

        // Rename current to timestamped backup
        let backup_name = format!("arm_log_{}.txt", get_time_ms());
        self.current_file.rename(&backup_name)?;

        // Open new file
        self.current_file = LogFile::create("arm_log.txt")?;
        self.event_count = 0;

        defmt::info!("Log rotated to {}", backup_name);
        Ok(())
    }
}
```

Related code areas:

- `src/core/logging/` - Log manager and file handling
- `src/communication/mavlink/state.rs` - Arm event generation
- `tools/log_analysis/` - Post-flight log analysis scripts

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- ArduPilot Log Format: <https://ardupilot.org/copter/docs/logmessages.html>
- MAVLink Logging Standards: <https://mavlink.io/en/services/logging.html>
