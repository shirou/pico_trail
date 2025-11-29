# AN-r86b9 GPS Position Telemetry and Autonomous Navigation Foundation

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../analysis/AN-xfiyr-gps-hardware-integration.md)
- Related Requirements:
  - [FR-333ym-gps-waypoint-navigation](../requirements/FR-333ym-gps-waypoint-navigation.md)
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-3ik7l-gps-operation-data-management](../requirements/FR-3ik7l-gps-operation-data-management.md)
  - [FR-uyie8-gps-mavlink-telemetry](../requirements/FR-uyie8-gps-mavlink-telemetry.md)
  - [FR-cs42u-gps-navigation-state-access](../requirements/FR-cs42u-gps-navigation-state-access.md)
  - [FR-4z22l-gps-course-over-ground](../requirements/FR-4z22l-gps-course-over-ground.md)
  - [NFR-5utah-gps-telemetry-performance](../requirements/NFR-5utah-gps-telemetry-performance.md)
  - [NFR-wwke9-gps-state-thread-safety](../requirements/NFR-wwke9-gps-state-thread-safety.md)
- Related ADRs:
  - [ADR-8tp69-uart0-gps-allocation](../adr/ADR-8tp69-uart0-gps-allocation.md)
  - [ADR-xqqbl-gps-state-management](../adr/ADR-xqqbl-gps-state-management.md)
- Related Tasks:
  - [T-vxtxn-uart0-gps-integration](../tasks/T-vxtxn-uart0-gps-integration/README.md)
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Executive Summary

This analysis examines the next steps for utilizing GPS position data acquired via UART0 from the NEO-M8N GPS module. The completed T-vxtxn task established the foundational GPS driver and UART integration. This analysis explores two key capabilities: (1) transmitting GPS position via MAVLink GLOBAL_POSITION_INT message for GCS integration, and (2) establishing the foundation for autonomous navigation by making GPS data available to the control and navigation subsystems.

The recommended approach is to extend the existing telemetry system to include real GPS data in GLOBAL_POSITION_INT messages (replacing current placeholder zeros), and create a shared GPS state mechanism that can be consumed by future autonomous navigation features.

## Problem Space

### Current State

**GPS Driver Implementation (Complete):**

- `src/devices/gps.rs`: UART-based GPS driver with NMEA parsing (GPGGA, GPRMC)
- `GpsPosition` struct provides: latitude, longitude, altitude, speed, fix_type, satellites
- `GpsDriver::update()` returns `Option<GpsPosition>` when valid NMEA sentence parsed
- Hardware verified: NEO-M8N GPS on UART0 (GPIO 0 RX, GPIO 5 TX) at 9600 baud

**MAVLink Telemetry (Partial):**

- `src/communication/mavlink/handlers/telemetry.rs`: TelemetryStreamer sends GPS_RAW_INT
- Current `build_gps()` function returns placeholder zeros (lat: 0, lon: 0, etc.)
- GLOBAL_POSITION_INT message not yet implemented

**Navigation Subsystem (Planned):**

- FR-333ym specifies GPS waypoint navigation with S-curve path planning
- No current mechanism to share GPS position with navigation/control subsystems

### Desired State

1. **MAVLink GPS Telemetry:**
   - GPS_RAW_INT populated with real GPS data (latitude, longitude, altitude, satellites, fix type)
   - GLOBAL_POSITION_INT message providing filtered position (lat, lon, alt, vx, vy, vz, hdg)
   - GCS (QGroundControl, Mission Planner) displays real vehicle position on map

2. **Autonomous Navigation Foundation:**
   - Shared GPS state accessible to control loop and navigation subsystem
   - GPS position updates available at 1-10Hz for waypoint following
   - Foundation for Auto mode implementation (FR-333ym)

### Gap Analysis

| Component           | Current State     | Desired State                | Gap                        |
| ------------------- | ----------------- | ---------------------------- | -------------------------- |
| GPS_RAW_INT         | Placeholder zeros | Real GPS data                | Data integration           |
| GLOBAL_POSITION_INT | Not implemented   | Provides position + velocity | New message implementation |
| Shared GPS state    | None              | Accessible to navigation     | State sharing mechanism    |
| Navigation input    | None              | GPS position for waypoints   | Integration layer          |

## Stakeholder Analysis

| Stakeholder          | Interest/Need                          | Impact | Priority |
| -------------------- | -------------------------------------- | ------ | -------- |
| GCS User             | View vehicle position on map           | High   | P0       |
| Navigation subsystem | GPS data for waypoint following        | High   | P0       |
| Control loop         | Position feedback for autonomous modes | High   | P0       |
| Telemetry system     | Accurate GPS reporting                 | Medium | P1       |
| System integrator    | Clean architecture for GPS data flow   | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - This is a continuation of GPS integration driven by existing requirements (FR-333ym).

### Competitive Analysis

**ArduPilot GPS Telemetry:**

- Sends both GPS_RAW_INT (raw GPS data) and GLOBAL_POSITION_INT (filtered EKF position)
- GPS_RAW_INT: Direct GPS receiver output
- GLOBAL_POSITION_INT: Fused position from EKF combining GPS, IMU, barometer
- Typical rates: GPS_RAW_INT at GPS native rate (1-10Hz), GLOBAL_POSITION_INT at EKF rate (50-400Hz)

**PX4 Autopilot:**

- Similar dual-message approach
- Uses uORB for internal GPS data sharing
- Position estimator fuses GPS with other sensors

**This Project's Context:**

- No EKF or sensor fusion currently implemented
- GLOBAL_POSITION_INT can initially mirror GPS_RAW_INT data
- Future: GLOBAL_POSITION_INT will reflect fused/filtered position when AHRS/EKF added

### Technical Investigation

#### MAVLink Message Analysis

**GPS_RAW_INT (ID: 24):**

```
time_usec     (uint64) - Timestamp since boot (us)
lat           (int32)  - Latitude (degE7)
lon           (int32)  - Longitude (degE7)
alt           (int32)  - Altitude MSL (mm)
eph           (uint16) - GPS HDOP (cm) [0=unknown]
epv           (uint16) - GPS VDOP (cm) [0=unknown]
vel           (uint16) - Ground speed (cm/s)
cog           (uint16) - Course over ground (cdeg, 0-35999)
fix_type      (uint8)  - GPS fix type enum
satellites_visible (uint8) - Satellites used
```

**GLOBAL_POSITION_INT (ID: 33):**

```
time_boot_ms  (uint32) - Timestamp since boot (ms)
lat           (int32)  - Latitude (degE7)
lon           (int32)  - Longitude (degE7)
alt           (int32)  - Altitude MSL (mm)
relative_alt  (int32)  - Altitude above home (mm)
vx            (int16)  - Ground X speed (cm/s, North)
vy            (int16)  - Ground Y speed (cm/s, East)
vz            (int16)  - Ground Z speed (cm/s, Down)
hdg           (uint16) - Heading (cdeg, 0-35999, UINT16_MAX if unknown)
```

**Key Differences:**

- GPS_RAW_INT: Raw GPS data with accuracy estimates (eph/epv)
- GLOBAL_POSITION_INT: Filtered position with velocity vector and heading
- GLOBAL_POSITION_INT requires: home position (for relative_alt), velocity decomposition

#### Unit Conversion Requirements

**GpsPosition (current) → MAVLink:**

| Field      | GpsPosition | MAVLink Unit | Conversion               |
| ---------- | ----------- | ------------ | ------------------------ |
| latitude   | f32 degrees | int32 degE7  | `(lat * 1e7) as i32`     |
| longitude  | f32 degrees | int32 degE7  | `(lon * 1e7) as i32`     |
| altitude   | f32 meters  | int32 mm     | `(alt * 1000.0) as i32`  |
| speed      | f32 m/s     | uint16 cm/s  | `(speed * 100.0) as u16` |
| satellites | u8          | uint8        | Direct copy              |

#### Velocity Decomposition for GLOBAL_POSITION_INT

GLOBAL_POSITION_INT requires velocity in NED (North-East-Down) frame:

- vx: Velocity North (cm/s)
- vy: Velocity East (cm/s)
- vz: Velocity Down (cm/s)

**Current GPS data provides:**

- speed: Ground speed magnitude (m/s)
- No heading/course from GPS directly (need GPRMC parsing enhancement or AHRS)

**Options for velocity:**

1. **Option A**: Set vx/vy/vz to 0 until AHRS provides heading (current speed available but direction unknown)
2. **Option B**: Parse course over ground (COG) from GPRMC to decompose speed into vx/vy
3. **Option C**: Wait for AHRS integration to provide heading

**Recommendation:** Option B - Enhance GPS driver to extract COG from GPRMC, then decompose:

- vx = speed \* cos(cog)
- vy = speed \* sin(cog)
- vz = 0 (GPS doesn't provide vertical rate reliably)

### Data Analysis

**GPS Update Rate Budget:**

| Component                     | Rate    | Notes                           |
| ----------------------------- | ------- | ------------------------------- |
| GPS hardware (NEO-M8N)        | 1-10Hz  | Default 1Hz, configurable       |
| GPS_RAW_INT telemetry         | 1-5Hz   | Match GPS rate                  |
| GLOBAL_POSITION_INT telemetry | 1-5Hz   | Initially same as GPS           |
| Navigation subsystem          | 10-50Hz | May need interpolation          |
| Control loop                  | 50Hz    | Position feedback for Auto mode |

**Latency Considerations:**

- GPS fix latency: \~100-200ms (internal processing)
- UART transmission: \~83ms for 80-byte NMEA at 9600 baud
- Total GPS latency: \~200-300ms from real-world position to `GpsPosition`
- Navigation should account for this latency (dead reckoning, prediction)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: GPS_RAW_INT message shall contain real GPS data from NEO-M8N
  - Rationale: GCS needs accurate GPS data for map display and logging
  - Acceptance Criteria:
    - lat/lon populated from GpsPosition (converted to degE7)
    - alt populated from GpsPosition (converted to mm)
    - fix_type reflects actual GPS fix status
    - satellites_visible shows actual satellite count
    - eph/epv set to 9999 (unknown) until HDOP/VDOP parsing added

- [ ] **FR-DRAFT-2**: GLOBAL_POSITION_INT message shall provide filtered position data
  - Rationale: Standard MAVLink message expected by GCS for vehicle position
  - Acceptance Criteria:
    - lat/lon/alt populated from GPS (initially unfiltered)
    - relative_alt calculated from home position (0 if home not set)
    - vx/vy/vz populated when heading available (0 otherwise)
    - hdg populated from AHRS or COG when available (UINT16_MAX otherwise)

- [ ] **FR-DRAFT-3**: GPS position shall be accessible to navigation subsystem
  - Rationale: Waypoint navigation (FR-333ym) requires GPS position input
  - Acceptance Criteria:
    - Shared GPS state updated when new position received
    - Navigation subsystem can read latest GPS position
    - Stale position detection (timestamp-based)

- [ ] **FR-DRAFT-4**: Course over ground (COG) shall be extracted from GPRMC
  - Rationale: Required for velocity decomposition in GLOBAL_POSITION_INT
  - Acceptance Criteria:
    - GPRMC sentence parsing extracts track angle (course)
    - COG available in GpsPosition struct
    - Invalid COG indicated when speed < threshold

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: GPS telemetry shall not increase control loop latency by more than 1ms
  - Category: Performance
  - Rationale: 50Hz control loop (20ms period) cannot be delayed by telemetry
  - Target: GPS data access and MAVLink message building < 1ms

- [ ] **NFR-DRAFT-2**: GPS state sharing shall be thread-safe without blocking
  - Category: Reliability
  - Rationale: Multiple consumers (telemetry, navigation, control) may access concurrently
  - Target: Use atomic or lock-free mechanism for GPS state sharing

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- TelemetryStreamer generates MAVLink messages from SystemState
- SystemState currently has no GPS field
- GPS driver runs independently, not integrated with SystemState

**Embassy Async Runtime:**

- GPS polling runs as async task
- Telemetry runs as separate async task
- Need inter-task communication for GPS data sharing

**Memory Constraints:**

- no_std environment, no heap allocation in critical paths
- GpsPosition is 20 bytes (small, suitable for copying)

### Potential Approaches

#### Approach A: Extend SystemState with GPS

**Description:** Add GPS fields to SystemState struct, update during GPS task, read during telemetry.

**Pros:**

- Consistent with existing pattern (battery, attitude in SystemState)
- Single source of truth for system state
- Simple integration with telemetry

**Cons:**

- SystemState currently passed by reference, may need Mutex
- GPS update rate may not match telemetry rate

**Effort:** Medium

#### Approach B: Dedicated GPS State with Channel

**Description:** Create separate GpsState struct, GPS task sends updates via channel, consumers receive.

**Pros:**

- Decouples GPS from SystemState
- Natural async pattern with channels
- Multiple consumers possible

**Cons:**

- Additional complexity (channel management)
- Potential message backpressure

**Effort:** Medium

#### Approach C: Static Mutex-Protected GPS State

**Description:** Global static `Mutex<GpsState>` accessible to all tasks.

**Pros:**

- Simple access pattern
- Works well for single-reader scenarios
- Existing pattern used elsewhere in codebase

**Cons:**

- Global state (less testable)
- Potential contention with multiple readers

**Effort:** Low

**Recommendation:** Approach A (Extend SystemState) for initial implementation, with potential migration to Approach B for full navigation integration.

### Architecture Impact

**ADR Required:**

- ADR for GPS state management pattern (SystemState extension vs. dedicated state)

**Module Changes:**

- `src/communication/mavlink/handlers/telemetry.rs`: Update build_gps(), add build_global_position_int()
- `src/communication/mavlink/state.rs`: Add GPS fields to SystemState
- `src/devices/gps.rs`: Add course_over_ground field to GpsPosition
- New integration layer: Connect GPS driver output to SystemState

## Risk Assessment

| Risk                                   | Probability | Impact | Mitigation Strategy                                |
| -------------------------------------- | ----------- | ------ | -------------------------------------------------- |
| GPS data stale when telemetry sends    | Medium      | Low    | Include timestamp in GPS state, validate freshness |
| Control loop blocked by GPS access     | Low         | High   | Use non-blocking state access (atomic/try_lock)    |
| Position jumps cause navigation issues | Medium      | Medium | Add position sanity checks, reject outliers        |
| COG invalid at low speed               | High        | Low    | Set COG to unknown when speed < 0.5 m/s            |
| Home position not set for relative_alt | Medium      | Low    | Default relative_alt to 0, warn in telemetry       |

## Open Questions

- [ ] Should GLOBAL_POSITION_INT wait for AHRS to provide heading, or use COG from GPS?
  - Recommendation: Use COG initially, switch to AHRS heading when available
- [ ] What is the minimum GPS update rate needed for waypoint navigation?
  - Next step: Review FR-333ym acceptance criteria for position update requirements
- [ ] Should GPS position be interpolated/predicted between updates for 50Hz control?
  - Recommendation: Defer to AHRS/EKF implementation, use raw GPS for now

## Recommendations

### Immediate Actions

1. **Extend SystemState with GPS fields:**
   - Add `gps_position: Option<GpsPosition>` and `gps_timestamp: u64` to SystemState
   - Update telemetry task to populate SystemState from GPS driver

2. **Implement real GPS_RAW_INT:**
   - Modify `build_gps()` to use SystemState.gps_position
   - Add unit conversion functions (degrees → degE7, meters → mm)

3. **Add GLOBAL_POSITION_INT message:**
   - New `build_global_position_int()` function in TelemetryStreamer
   - Initially mirror GPS_RAW_INT data, velocity/heading as placeholders

### Next Steps

1. [ ] Create formal requirements: FR for GPS telemetry, FR for navigation GPS access
2. [ ] Draft ADR: GPS state management pattern decision
3. [ ] Create task: T-<id>-gps-position-telemetry
   - Phase 1: SystemState GPS integration
   - Phase 2: GPS_RAW_INT with real data
   - Phase 3: GLOBAL_POSITION_INT implementation
   - Phase 4: Navigation foundation (shared GPS access pattern)

### Out of Scope

- **EKF/sensor fusion**: GLOBAL_POSITION_INT initially uses raw GPS, fusion is future work
- **GPS velocity vertical component**: NEO-M8N doesn't provide reliable vertical speed
- **RTK/differential GPS**: Not required for 2m waypoint acceptance radius
- **Multi-GPS redundancy**: Single GPS sufficient for rover

## Appendix

### References

**MAVLink Messages:**

- [GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT)
- [GLOBAL_POSITION_INT](https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT)

**ArduPilot Implementation:**

- [ArduPilot GCS_MAVLINK](https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp)
- [GPS Backend](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_GPS/AP_GPS.cpp)

### Raw Data

**Current GpsPosition struct:**

```rust
pub struct GpsPosition {
    pub latitude: f32,      // Decimal degrees (-90 to +90)
    pub longitude: f32,     // Decimal degrees (-180 to +180)
    pub altitude: f32,      // Meters above sea level
    pub speed: f32,         // Meters per second
    pub fix_type: GpsFixType,
    pub satellites: u8,
}
```

**Unit Conversion Examples:**

```rust
// Latitude: 35.6812 degrees -> 356812000 degE7
let lat_e7 = (35.6812_f32 * 1e7) as i32; // 356812000

// Altitude: 125.5 meters -> 125500 mm
let alt_mm = (125.5_f32 * 1000.0) as i32; // 125500

// Speed: 5.5 m/s -> 550 cm/s
let speed_cms = (5.5_f32 * 100.0) as u16; // 550
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
