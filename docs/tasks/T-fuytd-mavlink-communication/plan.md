# T-fuytd MAVLink Protocol Communication

## Metadata

- Type: Implementation Plan
- Status: Complete

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

- [x] Compatible with QGroundControl 4.x (tested connection and telemetry display)
- [x] Compatible with Mission Planner 1.3.x (tested connection and telemetry display)
- [x] MAVLink state < 10 KB RAM (verified via defmt memory logging)
- [x] 10Hz telemetry streams without dropped messages (verified via GCS logs)
- [x] < 0.1% message corruption rate (CRC validation tracking over 1000 messages)
- [x] All existing tests pass; no regressions in scheduler or platform

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

- [x] **Setup module structure**
  - [x] Create `src/communication/` directory
  - [x] Create `src/communication/mavlink/` directory
  - [x] Create `src/communication/mavlink/mod.rs` with module exports
  - [x] Create `src/communication/mod.rs`
  - [x] Update `src/lib.rs` to include `communication` module
- [x] **Add dependencies**
  - [x] Add `mavlink` crate to `Cargo.toml` with features `["embedded", "common"]`
  - [x] Add `heapless` crate for fixed-size buffers (no_std) - already present
  - [x] Verify `embassy-time` already present (from T-g729p) - confirmed
- [x] **Implement message parser**
  - [x] Create `src/communication/mavlink/parser.rs`
  - [x] Define `read_mavlink_message()` async function using rust-mavlink `read_v2_msg()`
  - [x] Add RX buffer (512 bytes, use heapless::Vec)
  - [x] Handle parse errors (CRC failures, incomplete messages)
  - [x] Add statistics tracking (messages received, parse errors)
- [x] **Implement message writer**
  - [x] Create `src/communication/mavlink/writer.rs`
  - [x] Define `write_mavlink_message()` async function using rust-mavlink `serialize()`
  - [x] Add TX buffer (1024 bytes, use heapless::Vec or ring buffer)
  - [x] Handle buffer full (drop message, log warning)
  - [x] Add statistics tracking (messages sent, buffer overflows)
- [x] **Create MAVLink router**
  - [x] Create `src/communication/mavlink/router.rs`
  - [x] Define `MavlinkRouter` struct with handler placeholders
  - [x] Implement `handle_message()` dispatcher (match on message type)
  - [x] Add connection state tracking (last heartbeat time from GCS)
- [x] **Define system state**
  - [x] Create `src/communication/mavlink/state.rs`
  - [x] Define `SystemState` struct (armed, mode, battery voltage/current)
  - [x] Define `ConnectionState` struct (connected, last_heartbeat) - in router.rs
  - [x] Add accessors for state queries
- [x] **Create MAVLink task**
  - [x] Create `src/communication/mavlink/task.rs`
  - [x] Define `mavlink_task_placeholder()` Embassy task
  - [x] Initialize UART (placeholder for actual implementation)
  - [x] Loop: read message, dispatch to router, send telemetry (placeholder structure)
  - [x] Add unit tests for task context

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

- [x] rust-mavlink dependency compiles successfully
- [x] MAVLink parser implemented with statistics tracking (3 unit tests passing)
- [x] MAVLink writer implemented with sequence counter (5 unit tests passing)
- [x] MAVLink router scaffold with HEARTBEAT handling (5 unit tests passing)
- [x] System state structures with arm/disarm logic (7 unit tests passing)
- [x] MAVLink task context and placeholder (4 unit tests passing)
- [x] All 24 unit tests passing
- [x] No unsafe code outside `src/platform/`
- [x] cargo fmt and cargo clippy pass with zero warnings

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

- [x] **Create parameter system**
  - [x] Create `src/core/parameters/` directory (completed in T-ex2h7)
  - [x] Create `src/core/parameters/mod.rs` (completed in T-ex2h7)
  - [x] Define `ParamMetadata` struct (name, type, value, default, min, max) (completed in T-ex2h7)
  - [x] Define `ParamType` enum (Float, Uint32) (completed in T-ex2h7)
  - [x] Define `ParamValue` enum (union of parameter types) (completed in T-ex2h7)
- [x] **Implement parameter registry**
  - [x] Create `src/core/parameters/registry.rs` (completed in T-ex2h7)
  - [x] Define parameter storage with heapless::Vec (completed in T-ex2h7)
  - [x] Add initial parameters via ParamHandler initialization:
    - `SR_EXTRA1` (Uint32, default 10, range 0-50)
    - `SR_POSITION` (Uint32, default 5, range 0-50)
    - `SR_RC_CHAN` (Uint32, default 5, range 0-50)
    - `SR_RAW_SENS` (Uint32, default 5, range 0-50)
    - `SYSID_THISMAV` (Uint32, default 1, range 1-255)
  - [x] Implement `get_param_by_name()` function (completed in T-ex2h7)
  - [x] Implement `get_param_by_index()` function (completed in T-ex2h7)
  - [x] Implement `set_param()` function with validation (completed in T-ex2h7)
- [x] **Integrate Flash persistence** (depends on T-ex2h7 Phase 4)
  - [x] Add `FlashParamStorage` field to parameter registry (completed in T-ex2h7)
  - [x] Call `load_from_flash()` during registry initialization (in ParamHandler::new)
  - [x] Flash save mechanism available via `save_to_flash()` (manual or via ParamSaver task)
  - [x] Handle Flash load/save errors gracefully (fall back to defaults on load failure)
  - [x] Flash persistence tested in T-ex2h7
- [x] **Implement parameter handler**
  - [x] Create `src/communication/mavlink/handlers/` directory
  - [x] Create `src/communication/mavlink/handlers/mod.rs`
  - [x] Create `src/communication/mavlink/handlers/param.rs`
  - [x] Define `ParamHandler` struct with ParameterRegistry
  - [x] Implement `handle_request_list()` – send all parameters
  - [x] Implement `handle_request_read()` – send specific parameter by name or index
  - [x] Implement `handle_set()` – validate and set parameter, send PARAM_VALUE response
  - [x] Add to MAVLink router dispatcher (MavlinkRouter now owns ParamHandler)
- [x] **Write unit tests**
  - [x] Test parameter registry lookup by name and index (8 tests in param.rs)
  - [x] Test parameter bounds validation (reject out-of-range values)
  - [x] Test PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET message handling
  - [x] Test PARAM_VALUE message serialization
  - [x] Test integration with router (test_param_set_integration)

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

- [x] Parameter registry compiles and allows get/set operations
- [x] PARAM_REQUEST_LIST sends all parameters (verified via unit tests)
- [x] PARAM_SET validates bounds and rejects invalid values (test_param_set_out_of_bounds)
- [x] Parameters integrated with Flash persistence (T-ex2h7)
- [x] Unit tests pass for parameter validation logic (8 tests in param.rs, 6 in router.rs)
- [x] No unsafe code in handlers or parameter integration

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

- [x] **Implement telemetry streamer**
  - [x] Create `src/communication/mavlink/handlers/telemetry.rs`
  - [x] Define `TelemetryStreamer` struct with stream rates and last send times
  - [x] Implement `should_send()` helper (checks if time to send based on rate)
  - [x] Implement `build_heartbeat()` – MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC, armed status
  - [x] Implement `build_attitude()` – roll, pitch, yaw from AHRS (placeholder: zeros for now)
  - [x] Implement `build_gps()` – lat, lon, alt from GPS (placeholder: zeros for now)
  - [x] Implement `build_sys_status()` – battery voltage/current, CPU load from scheduler
- [x] **Integrate with system state**
  - [x] Update `SystemState` to include cpu_load field
  - [x] Battery voltage/current already in SystemState (from Phase 1)
  - [x] Add telemetry message builders using system state
- [x] **Add telemetry integration**
  - [x] Add TelemetryStreamer to MavlinkRouter
  - [x] Implement `update_telemetry()` in router (reads SR\_\* parameters and calls streamer)
  - [x] Stream rates automatically updated from parameters (SR_EXTRA1, SR_POSITION)
- [x] **Write unit tests**
  - [x] Test `should_send()` logic for 1Hz, 5Hz, 10Hz rates (5 tests)
  - [x] Test HEARTBEAT message construction (verify armed status)
  - [x] Test ATTITUDE message construction (placeholder zeros)
  - [x] Test SYS_STATUS message construction (battery, CPU load)
  - [x] Test GPS_RAW_INT message construction (placeholder zeros)
  - [x] Test stream rate control (simulate 1 second at 50Hz updates)
  - [x] Test telemetry integration with router (2 tests in router.rs)
  - [x] All 15 telemetry tests passing

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

- [x] HEARTBEAT sent at 1Hz (verified via unit tests)
- [x] ATTITUDE sent at 10Hz (default SR_EXTRA1 rate, verified via test_stream_rate_control)
- [x] GPS_RAW_INT sent at 5Hz (default SR_POSITION rate, verified via test_stream_rate_control)
- [x] SYS_STATUS sent at 1Hz (SR_EXTRA1 clamped to max 1Hz)
- [x] Changing SR_EXTRA1 parameter changes ATTITUDE rate dynamically (test_telemetry_rate_from_parameters)
- [x] Stream rate control working correctly (test_stream_rate_control simulates 1s of updates)
- [x] All 15 telemetry unit tests passing
- [x] All 47 MAVLink module tests passing
- [x] No unsafe code in telemetry implementation

---

## Phase 4: Command Protocol

### Phase 4 Goal

- Implement command handler for COMMAND_LONG messages
- Support essential commands (arm/disarm, mode change)
- Send COMMAND_ACK responses
- Add safety checks (prevent arming if unsafe)

### Phase 4 Tasks

- [x] **Implement command handler**
  - [x] Create `src/communication/mavlink/handlers/command.rs`
  - [x] Define `CommandHandler` struct
  - [x] Implement `handle_command_long()` dispatcher
  - [x] Implement `MAV_CMD_COMPONENT_ARM_DISARM` (param1: 1=arm, 0=disarm)
  - [x] Implement `MAV_CMD_DO_SET_MODE` (set flight mode, placeholder: accept any mode)
  - [x] Implement `MAV_CMD_PREFLIGHT_CALIBRATION` (placeholder: send ACK accepted)
  - [x] Send `COMMAND_ACK` with MAV_RESULT_ACCEPTED/DENIED/UNSUPPORTED
- [x] **Add safety checks**
  - [x] Arm only if not already armed (prevent double arming)
  - [x] Disarm only if already armed
  - [x] Reject arm if battery voltage too low (< 10.0V, placeholder threshold)
  - [x] Log command rejections via defmt (with test environment stubs)
- [x] **Update system state**
  - [x] Add `armed` flag to `SystemState` (already present from Phase 1)
  - [x] Add `mode` field to `SystemState` (already present from Phase 1)
  - [x] Update state in command handler
  - [x] Reflect armed status in HEARTBEAT message (via TelemetryStreamer)
- [x] **Write unit tests**
  - [x] Test arm command acceptance (valid conditions)
  - [x] Test arm command rejection (already armed, low battery)
  - [x] Test disarm command
  - [x] Test unsupported command (verify MAV_RESULT_UNSUPPORTED)

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

- [x] Arm command successfully arms vehicle (armed flag set, HEARTBEAT reflects armed)
- [x] Disarm command successfully disarms vehicle
- [x] Arm command rejected if battery low (COMMAND_ACK with MAV_RESULT_DENIED)
- [x] Unsupported commands return MAV_RESULT_UNSUPPORTED
- [x] Mode change command updates system state
- [x] All 10 command handler unit tests passing
- [x] All 3 router integration tests passing (arm, disarm, mode change)
- [x] No unsafe code

---

## Phase 5: Mission Protocol

### Phase 5 Goal

- Implement basic mission protocol (MISSION_COUNT, MISSION_ITEM, MISSION_ACK)
- Support mission upload from GCS (store waypoints in memory)
- Support mission download to GCS (send stored waypoints)
- Defer mission execution to FR-333ym implementation

### Phase 5 Tasks

- [x] **Create mission storage**
  - [x] Create `src/core/mission/` directory
  - [x] Create `src/core/mission/mod.rs`
  - [x] Define `Waypoint` struct (seq, lat, lon, alt, command)
  - [x] Define static mission array (max 50 waypoints, use heapless::Vec)
  - [x] Implement `add_waypoint()`, `get_waypoint()`, `clear_mission()` functions
- [x] **Implement mission handler**
  - [x] Create `src/communication/mavlink/handlers/mission.rs`
  - [x] Define `MissionHandler` struct with upload/download state
  - [x] Implement `handle_mission_request_list()` – send MISSION_COUNT
  - [x] Implement `handle_mission_request_int()` – send MISSION_ITEM_INT for requested seq
  - [x] Implement `handle_mission_count()` – prepare for upload, request first item
  - [x] Implement `handle_mission_item_int()` – store waypoint, request next or send ACK
  - [x] Implement `handle_mission_ack()` – finalize upload/download
- [x] **Add mission state machine**
  - [x] Track upload/download state (Idle, UploadInProgress, DownloadInProgress)
  - [x] Handle timeouts (if GCS doesn't send next item within 5 seconds, abort)
  - [x] Add to MAVLink router dispatcher
- [x] **Write unit tests**
  - [x] Test mission storage (add, retrieve, clear)
  - [x] Test upload flow (MISSION_COUNT → MISSION_ITEM sequence)
  - [x] Test download flow (MISSION_REQUEST_LIST → MISSION_COUNT → MISSION_REQUEST)
  - [x] Test timeout handling

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

- [x] Mission upload from GCS successfully stores waypoints (verified via unit tests)
- [x] Mission download to GCS sends stored waypoints (verified via unit tests)
- [x] Mission timeout aborts upload if GCS unresponsive (test_timeout passes)
- [x] All 7 mission handler unit tests passing
- [x] All 12 mission storage unit tests passing
- [x] All 2 router integration tests passing (upload, download)
- [x] No unsafe code

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

- [x] **Hardware deployment**
  - [x] Build MAVLink demo for RP2350 target
  - [x] Flash to Pico 2 W using UF2
  - [x] Connect UART to USB-serial adapter (UART0 TX/RX, GND)
  - [x] Verify MAVLink traffic via serial monitor or mavproxy
- [x] **QGroundControl testing**
  - [x] Install QGroundControl 4.x on test machine
  - [x] Connect to Pico 2 W via serial port (115200 baud)
  - [x] Verify vehicle connects (HEARTBEAT received)
  - [x] Verify telemetry display (attitude, GPS, battery)
  - [x] Test parameter editing (change SR_EXTRA1, verify rate change)
  - [x] Test arm/disarm commands
  - [x] Test mission upload (upload 5 waypoints, verify storage)
  - [x] Document any compatibility issues
- [x] **Mission Planner testing**
  - [x] Install Mission Planner 1.3.x on Windows test machine
  - [x] Connect to Pico 2 W via COM port (115200 baud)
  - [x] Verify vehicle connects and telemetry displays
  - [x] Test parameter editing
  - [x] Test arm/disarm commands
  - [x] Document any compatibility issues
- [x] **Memory usage measurement**
  - [x] Add defmt logging for heap/stack usage (if available)
  - [x] Measure MAVLink static memory (buffers, state structs)
  - [x] Verify < 10 KB RAM target met
  - [x] Document actual memory usage
- [x] **Message rate measurement**
  - [x] Capture UART output for 60 seconds
  - [x] Count messages per type (HEARTBEAT, ATTITUDE, GPS_RAW_INT)
  - [x] Verify 1Hz HEARTBEAT, 10Hz ATTITUDE, 5Hz GPS (within ±5%)
  - [x] Check for dropped messages (gaps in seq numbers)
- [x] **CPU load measurement**
  - [x] Enable scheduler CPU load monitoring (from T-g729p)
  - [x] Measure CPU load with all telemetry streams active
  - [x] Verify < 10% CPU overhead target
  - [x] Document actual CPU load
- [x] **CRC validation tracking**
  - [x] Add statistics for CRC failures in message parser
  - [x] Run for 1000+ messages
  - [x] Verify < 0.1% corruption rate
  - [x] Document error rate
- [x] **Optimization (if needed)**
  - [x] Profile hot paths (message serialization, telemetry update)
  - [x] Optimize if CPU load > 10% or memory > 10 KB
  - [x] Consider reducing telemetry rates or buffer sizes
- [x] **Documentation**
  - [x] Create `docs/mavlink.md` with usage guide (wiring, GCS setup, supported messages)
  - [x] Update `docs/architecture.md` with MAVLink component diagram
  - [x] Add code examples in handler files
  - [x] Update `README.md` with GCS compatibility section
- [x] **RP2040 validation (Pico W)**
  - [x] Build and flash to Pico W (RP2040)
  - [x] Re-run QGroundControl connection test
  - [x] Verify performance acceptable on Cortex-M0+ (no FPU)
  - [x] Document any platform-specific issues

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

- [x] QGroundControl 4.x connects successfully, displays telemetry
- [x] Mission Planner 1.3.x connects successfully, displays telemetry
- [x] Memory usage < 10 KB (measured and documented)
- [x] Telemetry rates within ±5% of target (1Hz, 5Hz, 10Hz)
- [x] CPU load < 10% during full telemetry streaming
- [x] CRC error rate < 0.1% over 1000+ messages
- [x] Documentation complete and accurate

---

## Definition of Done

- [x] `cargo check --features pico2_w`
- [x] `cargo check --features pico_w` (if RP2040 hardware available)
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet` (all tests pass)
- [x] `docs/mavlink.md` created with usage guide
- [x] `docs/architecture.md` updated with MAVLink component
- [x] Hardware validation completed on Pico 2 W (QGC + MP tested)
- [x] All performance targets met (FR-gpzpz, NFR-z2iuk)
- [x] No `unsafe` code outside `src/platform/`
- [x] All `unsafe` blocks have SAFETY comments
- [x] No vague naming (no "manager"/"util")

## Open Questions

- [x] Should we implement all MAVLink commands or start with essential subset (arm, mode)? → Decision: Start with arm/disarm and mode change, add others in future tasks as needed
- [x] How to handle parameter storage persistence (flash writes)? → Decision: Defer to FR-a1cuu task, keep parameters in RAM only for this task (now completed in T-ex2h7)
- [x] Should we support USB CDC as alternative to UART? → Decision: USB support as future enhancement (see below), UART-only for initial implementation
- [x] What system ID and component ID should we use? → Decision: System ID 1 (configurable via SYSID_THISMAV parameter), Component ID MAV_COMP_ID_AUTOPILOT1 (1)
- [x] Should we implement mavproxy-style routing (multiple GCS endpoints)? → Decision: Defer to future task, single GCS connection for v1

## Future Enhancements

### USB CDC Transport

**Rationale**: USB CDC (Communications Device Class) provides direct PC connection without external UART adapter, simplifying hardware setup and enabling higher bandwidth.

**Benefits**:

- No external USB-serial adapter required
- Higher bandwidth potential (12 Mbps USB Full Speed vs 115200 baud UART)
- Simpler wiring for end users
- Can coexist with UART for dual-channel operation

**Implementation Requirements**:

- Add USB CDC interface to Platform abstraction layer (src/platform/traits/usb_cdc.rs)
- Implement RP2350 USB CDC driver (src/platform/rp2350/usb_cdc.rs)
- Add transport selection in MAVLink router (UART vs USB CDC)
- Update examples with USB CDC configuration
- Test with QGroundControl/Mission Planner over USB

**Estimated Effort**: 1-2 weeks (depends on embassy-usb stability)

**Priority**: Medium (nice-to-have after core functionality validated)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
