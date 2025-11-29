# T-gvsxb GPS State Management and Telemetry Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-gvsxb-gps-state-telemetry-plan](./plan.md)

## Overview

Integrate GPS position data into the system state and implement real GPS telemetry in MAVLink messages. The GPS driver runs as an async task, updating position at 1-10Hz. Multiple consumers (telemetry handler, navigation, control loop) need thread-safe access to this data. This design extends the existing SystemState pattern (battery, mode, armed) with GPS fields, following ADR-xqqbl.

## Success Metrics

- [ ] GPS_RAW_INT contains real GPS data (lat, lon, alt, satellites, fix_type)
- [ ] GLOBAL_POSITION_INT contains position and velocity data
- [ ] GPS state access latency < 100us
- [ ] COG correctly parsed from GPRMC sentences
- [ ] No control loop timing regression (50Hz maintained)

## Background and Current State

- Context: GPS driver exists (`src/devices/gps.rs`) and parses NMEA sentences via UART0. The GpsOperationManager polls position but doesn't share state globally.
- Current behavior: TelemetryStreamer sends GPS_RAW_INT with placeholder zeros. No GPS data reaches GCS.
- Pain points: GCS cannot display vehicle position; navigation subsystem has no GPS access.
- Constraints: Embassy async runtime, no heap allocation for state access, single Mutex for SystemState.
- Related ADRs: ADR-xqqbl (GPS State Management Pattern)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────┐
│   GPS Driver    │  (UART0, NMEA parsing)
│ GpsOperationMgr │
└────────┬────────┘
         │ update_gps_state()
         v
┌─────────────────────────────────────────────────┐
│               SYSTEM_STATE                       │
│  Mutex<CriticalSectionRawMutex, SystemState>    │
│  - armed, mode, battery (existing)              │
│  - gps_position: Option<GpsPosition> (NEW)      │
│  - gps_timestamp_us: u64 (NEW)                  │
└───────┬─────────────────┬─────────────────┬─────┘
        │                 │                 │
        v                 v                 v
┌───────────────┐ ┌───────────────┐ ┌─────────────┐
│   Telemetry   │ │  Navigation   │ │ Control Loop│
│    Handler    │ │  Subsystem    │ │   (50Hz)    │
└───────────────┘ └───────────────┘ └─────────────┘
```

### Components

- **GpsPosition (extended)**: Add `course_over_ground: Option<f32>` field for COG in degrees
- **SystemState (extended)**: Add `gps_position: Option<GpsPosition>` and `gps_timestamp_us: u64`
- **GPS Driver Update**: After successful NMEA parse, call `update_gps_state()` to set SYSTEM_STATE
- **TelemetryStreamer**: Build GPS_RAW_INT and GLOBAL_POSITION_INT from SystemState.gps_position

### Data Flow

1. GPS driver receives NMEA sentence via UART0
2. Parser extracts GpsPosition (including COG from GPRMC)
3. GPS driver acquires SYSTEM_STATE mutex, updates gps_position and gps_timestamp_us
4. Telemetry handler acquires SYSTEM_STATE mutex (short lock), reads gps_position
5. TelemetryStreamer builds GPS_RAW_INT / GLOBAL_POSITION_INT with unit conversions
6. Messages sent to GCS at SR_POSITION rate

### Data Models and Types

**Extended GpsPosition:**

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsPosition {
    pub latitude: f32,           // degrees (-90 to +90)
    pub longitude: f32,          // degrees (-180 to +180)
    pub altitude: f32,           // meters above MSL
    pub speed: f32,              // m/s
    pub course_over_ground: Option<f32>,  // NEW: degrees (0-360), None if invalid
    pub fix_type: GpsFixType,
    pub satellites: u8,
}
```

**Extended SystemState:**

```rust
pub struct SystemState {
    // ... existing fields ...
    pub gps_position: Option<GpsPosition>,  // NEW
    pub gps_timestamp_us: u64,              // NEW
}
```

**Unit Conversions (in telemetry handler):**

| Field      | Source                   | Conversion                    |
| ---------- | ------------------------ | ----------------------------- |
| lat/lon    | GpsPosition.latitude/lon | `(val * 1e7) as i32` (degE7)  |
| alt        | GpsPosition.altitude     | `(val * 1000.0) as i32` (mm)  |
| vel        | GpsPosition.speed        | `(val * 100.0) as u16` (cm/s) |
| cog        | GpsPosition.course       | `(val * 100.0) as u16` (cdeg) |
| vx (North) | speed \* cos(cog)        | `(vx * 100.0) as i16` (cm/s)  |
| vy (East)  | speed \* sin(cog)        | `(vy * 100.0) as i16` (cm/s)  |

### Error Handling

- If GPS has no fix: send GPS_RAW_INT with fix_type=0, zeros for position
- If GPS data stale (>1s): include last known position with stale indicator
- If COG invalid (speed < 0.5 m/s): set vx=0, vy=0, hdg=UINT16_MAX
- Use saturating arithmetic for unit conversions to prevent overflow

### Security Considerations

- No external input validation needed (GPS data from hardware)
- No heap allocation in state access path

### Performance Considerations

- GPS state access: single Mutex lock, \~10us typical
- Unit conversions: FPU-accelerated on Cortex-M33
- Message building: no allocation, struct population only
- Total telemetry overhead: < 1ms per cycle (NFR-5utah)

### Platform Considerations

#### Embedded (RP2350)

- Embassy `Mutex<CriticalSectionRawMutex, T>` for thread safety
- Critical sections are short (disable interrupts during access)
- micromath crate for sin/cos (no_std compatible)

#### Host Tests

- std::sync::Mutex for equivalent behavior
- Mock GPS data for telemetry handler tests
- Test concurrent access patterns

## Alternatives Considered

1. Dedicated GPS Mutex (separate from SystemState)
   - Pros: Reduced contention with battery/mode access
   - Cons: Additional synchronization primitive, inconsistent pattern

2. Lock-free triple buffering
   - Pros: Zero blocking for readers
   - Cons: More complex, premature optimization

Decision Rationale

- Use SystemState for consistency with existing architecture (battery, mode)
- GpsPosition is Copy (\~32 bytes), acceptable for mutex-based sharing
- Optimize to lock-free only if contention proves problematic

## Testing Strategy

### Unit Tests

- GpsPosition COG parsing from GPRMC sentences
- Unit conversion functions (degE7, mm, cm/s, cdeg)
- GPS_RAW_INT message building with mock GPS data
- GLOBAL_POSITION_INT velocity decomposition (vx, vy from speed+COG)
- Low speed COG invalidation (speed < 0.5 m/s)

### Integration Tests

- GPS state update and read cycle
- Telemetry message generation with real GpsPosition data

## Documentation Impact

- Update `docs/architecture.md` if GPS state flow needs documentation
- Traceability matrix will include this task

## External References

- [MAVLink GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT)
- [MAVLink GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)
- [Embassy Mutex](https://docs.embassy.dev/embassy-sync/git/default/mutex/struct.Mutex.html)

## Open Questions

- [x] Should we use dedicated GPS Mutex vs SystemState Mutex? → Decided: Use SystemState for consistency (ADR-xqqbl)
- [ ] Should GpsPosition include HDOP/VDOP? → Defer to future enhancement (set eph/epv=9999 for now)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
