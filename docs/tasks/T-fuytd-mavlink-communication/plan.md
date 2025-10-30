# T-fuytd MAVLink Protocol Communication

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-fuytd-mavlink-communication-design](design.md)
- Related ADRs:
  - [ADR-ggou4-mavlink-implementation](../../adr/ADR-ggou4-mavlink-implementation.md)
- Related Requirements:
  - [FR-gpzpz-mavlink-protocol](../../requirements/FR-gpzpz-mavlink-protocol.md)
  - [FR-a1cuu-runtime-parameters](../../requirements/FR-a1cuu-runtime-parameters.md)
  - [NFR-z2iuk-memory-limits](../../requirements/NFR-z2iuk-memory-limits.md)

## Overview

Implement MAVLink 2.0 protocol communication using rust-mavlink crate for GCS integration. Support telemetry streaming (HEARTBEAT, ATTITUDE, GPS_RAW_INT), parameter management (PARAM\_\* messages), mission protocol (MISSION\_\* messages), and command execution (COMMAND_LONG) compatible with QGroundControl and Mission Planner.

## Success Metrics

- [ ] Compatible with QGroundControl 4.x (tested connection and telemetry display)
- [ ] Compatible with Mission Planner 1.3.x (tested connection and telemetry display)
- [ ] MAVLink state < 10 KB RAM (verified via defmt memory logging)
- [ ] 10Hz telemetry streams without dropped messages (verified via GCS logs)
- [ ] < 0.1% message corruption rate (CRC validation tracking over 1000 messages)
- [ ] All existing tests pass; no regressions in scheduler or platform

## Scope

- Goal: Production-ready MAVLink communication with essential message support
- Non-Goals:
  - Message signing (deferred to future task)
  - Parameter persistence to flash (deferred to FR-a1cuu task)
  - Full mission waypoint execution (basic protocol only, execution in FR-333ym)
  - USB CDC transport (UART only for initial implementation)
  - ArduPilot-specific message extensions (use common dialect only)
- Assumptions:
  - Platform abstraction (T-egg4f) provides UART interface
  - Task scheduler (T-g729p) provides periodic task execution
  - UART configured at 115200 baud (11.5 KB/s bandwidth)
- Constraints:
  - no_std environment
  - < 10 KB RAM budget for MAVLink state and buffers
  - UART bandwidth limited to 11.5 KB/s at 115200 baud
  - Must work on both RP2040 (Pico W) and RP2350 (Pico 2 W)

## ADR & Legacy Alignment

- [x] ADR-ggou4-mavlink-implementation governs this work
- [x] No legacy code conflicts (greenfield implementation)
- [x] Platform abstraction (T-egg4f) provides UART interface
- [x] Task scheduler (T-g729p) provides task execution framework

## Plan Summary

- Phase 1 – Core MAVLink infrastructure (rust-mavlink integration, message parsing/writing)
- Phase 2 – Parameter protocol (parameter registry, PARAM\_\* handlers in RAM only)
- Phase 3 – Telemetry streaming (HEARTBEAT, ATTITUDE, GPS_RAW_INT, stream rates)
- Phase 4 – Command protocol (COMMAND_LONG, COMMAND_ACK, arm/disarm)
- Phase 5 – Mission protocol (MISSION_COUNT, MISSION_ITEM_INT, basic upload/download)
- Phase 6 – Hardware validation and GCS compatibility (QGroundControl, Mission Planner testing)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Core MAVLink Infrastructure

### Goal

- Integrate rust-mavlink crate into project
- Implement async message parsing from UART
- Implement async message writing to UART
- Create MAVLink router scaffold for message dispatch

### Inputs

- Documentation:
  - `/docs/adr/ADR-ggou4-mavlink-implementation.md` – Architecture decision
  - `/docs/requirements/FR-gpzpz-mavlink-protocol.md` – Protocol requirement
- Source Code to Create:
  - `/src/communication/` – New communication module
  - `/src/communication/mavlink/` – MAVLink implementation
- Dependencies:
  - External crates: `mavlink` (0.13+) with `common` dialect only, `heapless` (for no_std buffers), `embassy-time`
  - Internal: `src/platform/traits/uart.rs` (from T-egg4f)

### Tasks

- [ ] **Setup module structure**
  - [ ] Create `src/communication/` directory
  - [ ] Create `src/communication/mavlink/` directory
  - [ ] Create `src/communication/mavlink/mod.rs` with module exports
  - [ ] Create `src/communication/mod.rs`
  - [ ] Update `src/lib.rs` to include `communication` module
- [ ] **Add dependencies**
  - [ ] Add `mavlink` crate to `Cargo.toml` with features `["embedded", "common"]`
  - [ ] Add `heapless` crate for fixed-size buffers (no_std)
  - [ ] Verify `embassy-time` already present (from T-g729p)
- [ ] **Implement message parser**
  - [ ] Create `src/communication/mavlink/parser.rs`
  - [ ] Define `read_mavlink_message()` async function using rust-mavlink `read_v2_msg()`
  - [ ] Add RX buffer (512 bytes, use heapless::Vec)
  - [ ] Handle parse errors (CRC failures, incomplete messages)
  - [ ] Add statistics tracking (messages received, parse errors)
- [ ] **Implement message writer**
  - [ ] Create `src/communication/mavlink/writer.rs`
  - [ ] Define `write_mavlink_message()` async function using rust-mavlink `serialize()`
  - [ ] Add TX buffer (1024 bytes, use heapless::Vec or ring buffer)
  - [ ] Handle buffer full (drop message, log warning)
  - [ ] Add statistics tracking (messages sent, buffer overflows)
- [ ] **Create MAVLink router**
  - [ ] Create `src/communication/mavlink/router.rs`
  - [ ] Define `MavlinkRouter` struct with handler placeholders
  - [ ] Implement `handle_message()` dispatcher (match on message type)
  - [ ] Add connection state tracking (last heartbeat time from GCS)
- [ ] **Define system state**
  - [ ] Create `src/communication/mavlink/state.rs`
  - [ ] Define `SystemState` struct (armed, mode, battery voltage/current)
  - [ ] Define `ConnectionState` struct (connected, last_heartbeat)
  - [ ] Add accessors for state queries
- [ ] **Create MAVLink task**
  - [ ] Create `src/communication/mavlink/task.rs`
  - [ ] Define `mavlink_task()` Embassy task
  - [ ] Initialize UART (from platform abstraction)
  - [ ] Loop: read message, dispatch to router, send telemetry (placeholder)
  - [ ] Add to scheduler example for testing

### Deliverables

- rust-mavlink integrated with Embassy async UART
- Message parsing and writing functions
- MAVLink router scaffold
- Basic MAVLink task running on hardware

### Verification

```bash
cargo check --features pico2_w
cargo fmt
cargo clippy --all-targets --features pico2_w -- -D warnings
cargo doc --no-deps --features pico2_w  # Verify documentation renders
```

### Acceptance Criteria (Phase Gate)

- rust-mavlink dependency compiles successfully
- MAVLink parser can read HEARTBEAT message from test buffer
- MAVLink writer can serialize HEARTBEAT message to buffer
- MAVLink task compiles and runs on hardware (verified via defmt logs)
- No unsafe code outside `src/platform/`

### Rollback/Fallback

- If rust-mavlink integration issues, consult crate examples and documentation
- If UART async issues, verify Embassy UART driver functionality with simple echo test
- If memory issues, reduce buffer sizes (RX: 280 bytes min, TX: 512 bytes min)

---

## Phase 2: Parameter Protocol

### Phase 2 Goal

- Implement parameter registry (static list of parameters in RAM)
- Implement PARAM_REQUEST_LIST, PARAM_REQUEST_READ handlers
- Implement PARAM_SET, PARAM_VALUE handlers
- Add parameter validation (bounds checking, type conversion)

### Phase 2 Inputs

- Dependencies:
  - Phase 1: MAVLink router, message parser/writer
- Source Code to Create:
  - `/src/core/parameters/` – Parameter system
  - `/src/communication/mavlink/handlers/param.rs` – Parameter handler

### Phase 2 Tasks

- [ ] **Create parameter system**
  - [ ] Create `src/core/parameters/` directory
  - [ ] Create `src/core/parameters/mod.rs`
  - [ ] Define `ParamMetadata` struct (name, type, value, default, min, max)
  - [ ] Define `ParamType` enum (Uint8, Int32, Float, etc.)
  - [ ] Define `ParamValue` enum (union of parameter types)
- [ ] **Implement parameter registry**
  - [ ] Create `src/core/parameters/registry.rs`
  - [ ] Define static parameter array (use `static` with `Mutex` for thread safety)
  - [ ] Add initial parameters:
    - `SR_EXTRA1` (Uint8, default 10, range 0-50)
    - `SR_POSITION` (Uint8, default 5, range 0-50)
    - `SR_RC_CHAN` (Uint8, default 5, range 0-50)
    - `SR_RAW_SENS` (Uint8, default 5, range 0-50)
    - `SYSID_THISMAV` (Uint8, default 1, range 1-255)
  - [ ] Implement `get_param_by_name()` function
  - [ ] Implement `get_param_by_index()` function
  - [ ] Implement `set_param()` function with validation
- [ ] **Integrate Flash persistence** (depends on T-ex2h7 Phase 4)
  - [ ] Add `FlashParamStorage` field to parameter registry
  - [ ] Call `load_from_flash()` during registry initialization
  - [ ] Trigger `save_to_flash()` after parameter changes (via debounced save task)
  - [ ] Handle Flash load/save errors gracefully (fall back to defaults on load failure)
  - [ ] Test Flash persistence integration (save → reboot → load → verify)
- [ ] **Implement parameter handler**
  - [ ] Create `src/communication/mavlink/handlers/` directory
  - [ ] Create `src/communication/mavlink/handlers/mod.rs`
  - [ ] Create `src/communication/mavlink/handlers/param.rs`
  - [ ] Define `ParamHandler` struct
  - [ ] Implement `handle_param_request_list()` – send all parameters
  - [ ] Implement `handle_param_request_read()` – send specific parameter
  - [ ] Implement `handle_param_set()` – validate and set parameter, send PARAM_VALUE response
  - [ ] Add to MAVLink router dispatcher
- [ ] **Write unit tests**
  - [ ] Test parameter registry lookup by name and index
  - [ ] Test parameter bounds validation (reject out-of-range values)
  - [ ] Test parameter type conversion (int to float, etc.)
  - [ ] Test PARAM_VALUE message serialization

### Phase 2 Deliverables

- Parameter registry with initial parameters (stream rates, system ID) in RAM
- PARAM\_\* message handlers
- Unit tests for parameter system

### Phase 2 Verification

```bash
cargo check --features pico2_w
cargo fmt
cargo clippy --all-targets --features pico2_w -- -D warnings
cargo test --lib --quiet parameters
cargo test --lib --quiet communication::mavlink::handlers::param
```

### Phase 2 Acceptance Criteria

- Parameter registry compiles and allows get/set operations
- PARAM_REQUEST_LIST sends all parameters (verified via GCS or simulated request)
- PARAM_SET validates bounds and rejects invalid values
- Parameters stored in RAM (reset to defaults on reboot, persistence deferred to FR-a1cuu)
- Unit tests pass for parameter validation logic
- No unsafe code

### Phase 2 Rollback/Fallback

- If bounds validation causes issues, log warning and clamp values instead of rejecting
- If parameter count exceeds available RAM, reduce number of parameters or optimize storage

---

## Phase 3: Telemetry Streaming

### Phase 3 Goal

- Implement telemetry streamer (HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS)
- Implement stream rate control (SR\_\* parameters)
- Integrate with application state (scheduler stats, placeholder sensor data)
- Run telemetry task at appropriate rate

### Phase 3 Tasks

- [ ] **Implement telemetry streamer**
  - [ ] Create `src/communication/mavlink/handlers/telemetry.rs`
  - [ ] Define `TelemetryStreamer` struct with stream rates and last send times
  - [ ] Implement `should_send()` helper (checks if time to send based on rate)
  - [ ] Implement `send_heartbeat()` – MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, armed status
  - [ ] Implement `send_attitude()` – roll, pitch, yaw from AHRS (placeholder: zeros for now)
  - [ ] Implement `send_gps()` – lat, lon, alt from GPS (placeholder: zeros for now)
  - [ ] Implement `send_sys_status()` – battery voltage/current, CPU load from scheduler
- [ ] **Integrate with system state**
  - [ ] Update `SystemState` to include battery voltage/current (placeholder: 12.0V, 0.0A)
  - [ ] Query scheduler stats for CPU load (use monitor task stats from T-g729p)
  - [ ] Add accessors for telemetry queries
- [ ] **Add telemetry task**
  - [ ] Update MAVLink task to call `telemetry.update()` periodically
  - [ ] Run telemetry update at 50Hz (highest stream rate is 10Hz, oversample)
  - [ ] Send messages based on SR\_\* parameter rates
- [ ] **Write unit tests**
  - [ ] Test `should_send()` logic for 1Hz, 5Hz, 10Hz rates
  - [ ] Test HEARTBEAT message construction (verify fields)
  - [ ] Test ATTITUDE message construction
  - [ ] Test stream rate parameter integration

### Phase 3 Deliverables

- Telemetry streamer with HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS
- Stream rate control via SR\_\* parameters
- Telemetry task integrated with MAVLink router

### Phase 3 Verification

```bash
cargo check --features pico2_w
cargo fmt
cargo clippy --all-targets --features pico2_w -- -D warnings
cargo test --lib --quiet communication::mavlink::handlers::telemetry
# Hardware test: Build and flash, monitor UART output
./scripts/build-rp2350.sh --release mavlink_demo
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
```

### Phase 3 Acceptance Criteria

- HEARTBEAT sent at 1Hz (verified via defmt logs or UART capture)
- ATTITUDE sent at 10Hz (default SR_EXTRA1 rate)
- GPS_RAW_INT sent at 5Hz (default SR_POSITION rate)
- Changing SR_EXTRA1 parameter changes ATTITUDE rate dynamically
- No telemetry dropped messages under normal load (< 75% CPU)

---

## Phase 4: Command Protocol

### Phase 4 Goal

- Implement command handler for COMMAND_LONG messages
- Support essential commands (arm/disarm, mode change)
- Send COMMAND_ACK responses
- Add safety checks (prevent arming if unsafe)

### Phase 4 Tasks

- [ ] **Implement command handler**
  - [ ] Create `src/communication/mavlink/handlers/command.rs`
  - [ ] Define `CommandHandler` struct
  - [ ] Implement `handle_command_long()` dispatcher
  - [ ] Implement `MAV_CMD_COMPONENT_ARM_DISARM` (param1: 1=arm, 0=disarm)
  - [ ] Implement `MAV_CMD_DO_SET_MODE` (set flight mode, placeholder: accept any mode)
  - [ ] Implement `MAV_CMD_PREFLIGHT_CALIBRATION` (placeholder: send ACK accepted)
  - [ ] Send `COMMAND_ACK` with MAV_RESULT_ACCEPTED/DENIED/UNSUPPORTED
- [ ] **Add safety checks**
  - [ ] Arm only if not already armed (prevent double arming)
  - [ ] Disarm only if already armed
  - [ ] Reject arm if battery voltage too low (< 10.0V, placeholder threshold)
  - [ ] Log command rejections via defmt
- [ ] **Update system state**
  - [ ] Add `armed` flag to `SystemState`
  - [ ] Add `mode` field to `SystemState` (enum: Stabilize, Loiter, Auto, etc.)
  - [ ] Update state in command handler
  - [ ] Reflect armed status in HEARTBEAT message
- [ ] **Write unit tests**
  - [ ] Test arm command acceptance (valid conditions)
  - [ ] Test arm command rejection (already armed, low battery)
  - [ ] Test disarm command
  - [ ] Test unsupported command (verify MAV_RESULT_UNSUPPORTED)

### Phase 4 Deliverables

- Command handler for arm/disarm and mode change
- Safety checks preventing unsafe arming
- Unit tests for command logic

### Phase 4 Verification

```bash
cargo check --features pico2_w
cargo fmt
cargo clippy --all-targets --features pico2_w -- -D warnings
cargo test --lib --quiet communication::mavlink::handlers::command
# Hardware test with GCS or simulated COMMAND_LONG messages
```

### Phase 4 Acceptance Criteria

- Arm command successfully arms vehicle (armed flag set, HEARTBEAT reflects armed)
- Disarm command successfully disarms vehicle
- Arm command rejected if battery low (COMMAND_ACK with MAV_RESULT_DENIED)
- Unsupported commands return MAV_RESULT_UNSUPPORTED
- No unsafe code

---

## Phase 5: Mission Protocol

### Phase 5 Goal

- Implement basic mission protocol (MISSION_COUNT, MISSION_ITEM, MISSION_ACK)
- Support mission upload from GCS (store waypoints in memory)
- Support mission download to GCS (send stored waypoints)
- Defer mission execution to FR-333ym implementation

### Phase 5 Tasks

- [ ] **Create mission storage**
  - [ ] Create `src/core/mission/` directory
  - [ ] Create `src/core/mission/mod.rs`
  - [ ] Define `Waypoint` struct (seq, lat, lon, alt, command)
  - [ ] Define static mission array (max 50 waypoints, use heapless::Vec)
  - [ ] Implement `add_waypoint()`, `get_waypoint()`, `clear_mission()` functions
- [ ] **Implement mission handler**
  - [ ] Create `src/communication/mavlink/handlers/mission.rs`
  - [ ] Define `MissionHandler` struct with upload/download state
  - [ ] Implement `handle_mission_request_list()` – send MISSION_COUNT
  - [ ] Implement `handle_mission_request_int()` – send MISSION_ITEM_INT for requested seq
  - [ ] Implement `handle_mission_count()` – prepare for upload, request first item
  - [ ] Implement `handle_mission_item_int()` – store waypoint, request next or send ACK
  - [ ] Implement `handle_mission_ack()` – finalize upload/download
- [ ] **Add mission state machine**
  - [ ] Track upload/download state (Idle, UploadInProgress, DownloadInProgress)
  - [ ] Handle timeouts (if GCS doesn't send next item within 5 seconds, abort)
  - [ ] Add to MAVLink router dispatcher
- [ ] **Write unit tests**
  - [ ] Test mission storage (add, retrieve, clear)
  - [ ] Test upload flow (MISSION_COUNT → MISSION_ITEM sequence)
  - [ ] Test download flow (MISSION_REQUEST_LIST → MISSION_COUNT → MISSION_REQUEST)
  - [ ] Test timeout handling

### Phase 5 Deliverables

- Mission storage (in-memory waypoint array)
- Mission protocol handlers (upload/download)
- Unit tests for mission logic

### Phase 5 Verification

```bash
cargo check --features pico2_w
cargo fmt
cargo clippy --all-targets --features pico2_w -- -D warnings
cargo test --lib --quiet communication::mavlink::handlers::mission
cargo test --lib --quiet core::mission
```

### Phase 5 Acceptance Criteria

- Mission upload from GCS successfully stores waypoints (verified via logs)
- Mission download to GCS sends stored waypoints
- Mission timeout aborts upload if GCS unresponsive
- Unit tests pass for mission state machine
- No unsafe code

---

## Phase 6: Hardware Validation and GCS Compatibility

### Phase 6 Goal

- Deploy MAVLink implementation to Pico 2 W hardware
- Test with QGroundControl 4.x
- Test with Mission Planner 1.3.x
- Measure memory usage and message rates
- Optimize if needed
- Document usage

### Phase 6 Tasks

- [ ] **Hardware deployment**
  - [ ] Build MAVLink demo for RP2350 target
  - [ ] Flash to Pico 2 W using UF2
  - [ ] Connect UART to USB-serial adapter (UART0 TX/RX, GND)
  - [ ] Verify MAVLink traffic via serial monitor or mavproxy
- [ ] **QGroundControl testing**
  - [ ] Install QGroundControl 4.x on test machine
  - [ ] Connect to Pico 2 W via serial port (115200 baud)
  - [ ] Verify vehicle connects (HEARTBEAT received)
  - [ ] Verify telemetry display (attitude, GPS, battery)
  - [ ] Test parameter editing (change SR_EXTRA1, verify rate change)
  - [ ] Test arm/disarm commands
  - [ ] Test mission upload (upload 5 waypoints, verify storage)
  - [ ] Document any compatibility issues
- [ ] **Mission Planner testing**
  - [ ] Install Mission Planner 1.3.x on Windows test machine
  - [ ] Connect to Pico 2 W via COM port (115200 baud)
  - [ ] Verify vehicle connects and telemetry displays
  - [ ] Test parameter editing
  - [ ] Test arm/disarm commands
  - [ ] Document any compatibility issues
- [ ] **Memory usage measurement**
  - [ ] Add defmt logging for heap/stack usage (if available)
  - [ ] Measure MAVLink static memory (buffers, state structs)
  - [ ] Verify < 10 KB RAM target met
  - [ ] Document actual memory usage
- [ ] **Message rate measurement**
  - [ ] Capture UART output for 60 seconds
  - [ ] Count messages per type (HEARTBEAT, ATTITUDE, GPS_RAW_INT)
  - [ ] Verify 1Hz HEARTBEAT, 10Hz ATTITUDE, 5Hz GPS (within ±5%)
  - [ ] Check for dropped messages (gaps in seq numbers)
- [ ] **CPU load measurement**
  - [ ] Enable scheduler CPU load monitoring (from T-g729p)
  - [ ] Measure CPU load with all telemetry streams active
  - [ ] Verify < 10% CPU overhead target
  - [ ] Document actual CPU load
- [ ] **CRC validation tracking**
  - [ ] Add statistics for CRC failures in message parser
  - [ ] Run for 1000+ messages
  - [ ] Verify < 0.1% corruption rate
  - [ ] Document error rate
- [ ] **Optimization (if needed)**
  - [ ] Profile hot paths (message serialization, telemetry update)
  - [ ] Optimize if CPU load > 10% or memory > 10 KB
  - [ ] Consider reducing telemetry rates or buffer sizes
- [ ] **Documentation**
  - [ ] Create `docs/mavlink.md` with usage guide (wiring, GCS setup, supported messages)
  - [ ] Update `docs/architecture.md` with MAVLink component diagram
  - [ ] Add code examples in handler files
  - [ ] Update `README.md` with GCS compatibility section
- [ ] **RP2040 validation (Pico W)**
  - [ ] Build and flash to Pico W (RP2040)
  - [ ] Re-run QGroundControl connection test
  - [ ] Verify performance acceptable on Cortex-M0+ (no FPU)
  - [ ] Document any platform-specific issues

### Phase 6 Deliverables

- MAVLink validated on both Pico W and Pico 2 W
- QGroundControl and Mission Planner compatibility confirmed
- Performance measurements documented
- Usage documentation (`docs/mavlink.md`)

### Phase 6 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
# Build for both platforms
./scripts/build-rp2350.sh --release mavlink_demo
# Flash and test manually on hardware
probe-rs run --chip RP2350 --release target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
```

### Phase 6 Acceptance Criteria

- QGroundControl 4.x connects successfully, displays telemetry
- Mission Planner 1.3.x connects successfully, displays telemetry
- Memory usage < 10 KB (measured and documented)
- Telemetry rates within ±5% of target (1Hz, 5Hz, 10Hz)
- CPU load < 10% during full telemetry streaming
- CRC error rate < 0.1% over 1000+ messages
- Documentation complete and accurate

---

## Definition of Done

- [ ] `cargo check --features pico2_w`
- [ ] `cargo check --features pico_w` (if RP2040 hardware available)
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet` (all tests pass)
- [ ] `docs/mavlink.md` created with usage guide
- [ ] `docs/architecture.md` updated with MAVLink component
- [ ] Hardware validation completed on Pico 2 W (QGC + MP tested)
- [ ] All performance targets met (FR-gpzpz, NFR-z2iuk)
- [ ] No `unsafe` code outside `src/platform/`
- [ ] All `unsafe` blocks have SAFETY comments
- [ ] No vague naming (no "manager"/"util")

## Open Questions

- [ ] Should we implement all MAVLink commands or start with essential subset (arm, mode)? → Next step: Start with arm/disarm and mode change, add others in future tasks as needed
- [ ] How to handle parameter storage persistence (flash writes)? → Decision: Defer to FR-a1cuu task, keep parameters in RAM only for this task
- [ ] Should we support USB CDC as alternative to UART? → Method: USB support as future enhancement, UART-only for initial implementation
- [ ] What system ID and component ID should we use? → Decision: System ID 1 (configurable via SYSID_THISMAV parameter), Component ID MAV_COMP_ID_AUTOPILOT1 (1)
- [ ] Should we implement mavproxy-style routing (multiple GCS endpoints)? → Decision: Defer to future task, single GCS connection for v1

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
