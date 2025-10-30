# FR-4e922 Data Logging to Flash Storage

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall log sensor data, control outputs, and system events to Flash storage with configurable rates (minimum 50Hz), supporting log download via MAVLink protocol for post-flight analysis and debugging.

## Rationale

Data logging is essential for:

- **Debugging**: Analyze failures, investigate unexpected behavior
- **Performance Tuning**: Review control loop performance, identify oscillations
- **Incident Analysis**: Reconstruct sequence of events leading to problems
- **Compliance**: Record operational data for safety reviews

ArduPilot's binary log format is industry-standard, compatible with analysis tools like MAVExplorer and Mission Planner.

## User Story (if applicable)

As an operator, I want the autopilot to record sensor readings, control outputs, and mode changes to onboard storage, so that I can download and analyze the logs after each mission to understand vehicle behavior and diagnose issues.

## Acceptance Criteria

- [ ] System logs IMU data at 100Hz minimum
- [ ] System logs GPS data at 10Hz minimum
- [ ] System logs control outputs (steering, throttle) at 50Hz minimum
- [ ] System logs mode changes, arming events, and errors as they occur
- [ ] Log format compatible with ArduPilot binary log format (self-describing FMT messages)
- [ ] Logs downloadable via MAVLink LOG_REQUEST_LIST/LOG_REQUEST_DATA protocol
- [ ] Logs compatible with MAVExplorer and Mission Planner log analysis tools
- [ ] Logging does not block control loops (async/buffered writes)
- [ ] Circular buffer prevents Flash overflow (oldest logs overwritten when full)

## Technical Details (if applicable)

### Functional Requirement Details

**Log Message Types:**

```
FMT: Format definition (describes structure of other messages)
IMU: Gyro, accel, timestamp
GPS: Latitude, longitude, altitude, fix type, velocity
ATT: Roll, pitch, yaw (attitude)
CTUN: Control outputs (steering, throttle), desired vs actual
MODE: Mode change events
ERR: Error events and warnings
PARM: Parameter values at startup
```

**ArduPilot Binary Log Format:**

```
[FMT message]: Type, Length, Name, Format, Columns
[Data message]: Type, Timestamp, Field1, Field2, ...
```

Example:

```
FMT: [1, 28, "IMU", "QffffffIIf", "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T"]
IMU: [1, 1234567, 0.01, 0.02, 0.03, 0.0, 0.0, 9.8, 0, 0, 25.0]
```

**Storage Allocation:**

```
Pico W (2 MB Flash):
  - Firmware: 256 KB
  - Parameters: 8 KB
  - Missions: 8 KB
  - Logs: ~1.7 MB (circular buffer)

Pico 2 W (4 MB Flash):
  - Firmware: 256 KB
  - Parameters: 8 KB
  - Missions: 8 KB
  - Logs: ~3.7 MB (circular buffer)
```

**Logging Rates:**

- IMU: 100Hz (10ms period)
- GPS: 10Hz (100ms period)
- Attitude: 50Hz (20ms period)
- Control: 50Hz (20ms period)
- Mode/Events: As they occur

**Estimated Data Rates:**

- IMU (28 bytes @ 100Hz): 2.8 KB/s
- GPS (64 bytes @ 10Hz): 640 B/s
- ATT (16 bytes @ 50Hz): 800 B/s
- CTUN (32 bytes @ 50Hz): 1.6 KB/s
- **Total**: \~6 KB/s → \~360 KB/min → \~21 MB/hour

**Log Buffer:**

- RAM buffer: 8-16 KB (1-2 seconds of data)
- Flush to Flash every 1 second or when buffer 75% full
- If buffer overflows, drop oldest data or low-priority messages

**Log Download:**

Via MAVLink protocol:

1. GCS sends `LOG_REQUEST_LIST` - Get list of logs
2. Autopilot responds with `LOG_ENTRY` for each log
3. GCS sends `LOG_REQUEST_DATA` - Request log chunks
4. Autopilot sends `LOG_DATA` - Log data chunks (90 bytes max per packet)

## Platform Considerations

### Pico W (RP2040)

Limited Flash (2 MB) - Logs fill up quickly. Recommend downloading logs frequently or reducing log rates.

### Pico 2 W (RP2350)

More Flash (4 MB) - Can log for longer periods before circular buffer overwrites.

### Cross-Platform

Logger must use platform Flash abstraction. Logging rates may need adjustment based on available Flash.

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                                  | Validation                                |
| ------------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ----------------------------------------- |
| Log buffer overflow during high-rate events | Medium | Medium     | Size buffer for worst-case rate, drop low-priority messages | Stress test with maximum sensor rates     |
| Flash write latency blocks control loops    | High   | Low        | Use async writes with RAM buffer, background Flash writes   | Verify control loop jitter during logging |
| Logs fill Flash too quickly                 | Low    | Medium     | Implement circular buffer, allow rate adjustment via params | Calculate log duration at various rates   |
| Log corruption during power loss            | Medium | Low        | Flush buffer periodically, accept some data loss            | Test with abrupt power removal            |

## Implementation Notes

Preferred approaches:

- Use **ArduPilot binary log format** for compatibility with existing tools
- Implement **async logging** with RAM buffer to avoid blocking control loops
- Use **circular buffer** in Flash to prevent overflow
- Define log messages via **macro** for compile-time validation:
  ```rust
  log_message! {
      IMU {
          TimeUS: u64,
          GyrX: f32, GyrY: f32, GyrZ: f32,
          AccX: f32, AccY: f32, AccZ: f32,
      }
  }
  ```

Known pitfalls:

- Flash writes are slow (100ms for 4KB block erase) - must use buffering
- Circular buffer must track read/write pointers correctly
- Log download via MAVLink is slow (57.6 KB/s @ 115200 baud) - be patient
- Log timestamps must be monotonic (use microsecond timestamps)

Related code areas:

- `src/core/logger/` - Logger implementation
- `src/platform/*/flash.rs` - Flash storage abstraction
- `src/communication/mavlink/log.rs` - MAVLink log download protocol

Suggested libraries:

- `embedded-storage` for Flash abstraction
- `heapless` for fixed-size log buffer (no_std compatible)

## External References

- ArduPilot Logger: <https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html>
- MAVLink Log Download Protocol: <https://mavlink.io/en/services/log_transfer.html>
- MAVExplorer Tool: <https://ardupilot.org/dev/docs/using-mavexplorer-for-log-analysis.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
