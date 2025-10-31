# T-oq110 MAVLink Network Transport Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](./design.md)

## Overview

Implement UDP network transport for MAVLink communication using trait-based abstraction, enabling concurrent UART and UDP operation with production-grade reliability. WiFi configuration uses compile-time environment variables. Implementation follows ArduPilot's proven transport abstraction pattern adapted to Rust and Embassy async framework.

## Success Metrics

- [ ] Command round-trip latency < 110ms (COMMAND_LONG → COMMAND_ACK via UDP)
- [ ] Memory usage ≤ 50 KB RAM for network stack
- [ ] Packet loss < 1% on local WiFi network
- [ ] All existing UART tests pass; no regressions
- [ ] Works with QGroundControl/Mission Planner without configuration
- [ ] `cargo fmt`, `cargo clippy`, `cargo test --lib --quiet` all pass

## Scope

- Goal: Production-grade UDP network transport alongside existing UART transport
- Non-Goals:
  - TCP transport (deferred to future)
  - Runtime WiFi configuration (deferred to future)
  - WiFi AP mode (deferred to future)
  - Message encryption/authentication (not part of base MAVLink v2.0)
- Assumptions:
  - WiFi network available during development/testing
  - QGroundControl or Mission Planner available for testing
  - RP2350 Pico 2 W hardware available
- Constraints:
  - Memory budget: ≤50 KB RAM
  - Latency budget: ≤100ms additional latency
  - Platform: RP2040/RP2350 with CYW43439 WiFi

## ADR & Legacy Alignment

- [ ] Confirm the latest ADRs are referenced:
  - [ADR-ckv8z-transport-abstraction](../../adr/ADR-ckv8z-transport-abstraction.md) - Trait-based abstraction
  - [ADR-aul2v-udp-primary-transport](../../adr/ADR-aul2v-udp-primary-transport.md) - UDP port 14550
  - [ADR-dxdj0-wifi-config-strategy](../../adr/ADR-dxdj0-wifi-config-strategy.md) - Compile-time config
- [ ] Legacy patterns to address:
  - Current MAVLink code (`src/communication/mavlink/`) is UART-specific
  - Need to extract protocol logic from UART-specific code
  - Need to refactor existing UART code to use new transport trait

## Plan Summary

- Phase 1 – Transport Abstraction Foundation: Define trait, refactor UART to use trait, create router
- Phase 2 – WiFi & UDP Implementation: WiFi initialization, UDP transport, GCS tracking, concurrent operation
- Phase 3 – Testing & Validation: Hardware testing, QGroundControl integration, performance validation

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Transport Abstraction Foundation

### Goal

- Create transport trait abstraction
- Refactor existing UART code to implement trait
- Create MAVLink router for multi-transport support
- Verify no UART functionality regressions

### Inputs

- Documentation:
  - `docs/adr/ADR-ckv8z-transport-abstraction.md` – Trait design and rationale
  - `docs/analysis/AN-808o3-mavlink-network-transport.md` – ArduPilot pattern analysis
- Source Code to Modify:
  - `src/communication/mavlink/parser.rs` – MAVLink message parsing
  - `src/communication/mavlink/writer.rs` – MAVLink message encoding
  - `src/communication/mavlink/task.rs` – MAVLink task logic
- Dependencies:
  - Internal: `src/communication/mavlink/` – Existing MAVLink implementation
  - External crates: `embassy-futures`, `heapless`, `defmt`

### Tasks

- [ ] **Create transport abstraction module**
  - [ ] Create `src/communication/mavlink/transport/mod.rs`
  - [ ] Define `MavlinkTransport` trait with async methods (`available`, `read`, `write`, `flush`)
  - [ ] Define `TransportError` enum (IoError, Timeout, Disconnected)
  - [ ] Add module documentation with trait usage examples
- [ ] **Implement UART transport**
  - [ ] Create `src/communication/mavlink/transport/uart.rs`
  - [ ] Define `UartTransport` struct wrapping `UartTx` and `UartRx`
  - [ ] Implement `MavlinkTransport` trait for `UartTransport`
  - [ ] Add constructor `UartTransport::new(tx, rx)`
  - [ ] Verify UART read/write operations work correctly
- [ ] **Create MAVLink router**
  - [ ] Create `src/communication/mavlink/router.rs`
  - [ ] Define `MavlinkRouter` struct with transport vector (`heapless::Vec<Box<dyn MavlinkTransport>, 4>`)
  - [ ] Define `RouterError` enum
  - [ ] Implement `add_transport(&mut self, Box<dyn MavlinkTransport>)`
  - [ ] Implement `receive_message(&mut self) -> Result<(MavMessage, usize)>` using `embassy_futures::select_array`
  - [ ] Implement `send_message(&mut self, &MavMessage) -> Result<()>` broadcasting to all transports
  - [ ] Add transport statistics tracking (`TransportStats` struct)
- [ ] **Refactor existing MAVLink task**
  - [ ] Update `src/communication/mavlink/task.rs` to use `MavlinkRouter`
  - [ ] Remove direct UART dependencies from task logic
  - [ ] Register `UartTransport` with router at startup
  - [ ] Verify message handling logic unchanged

### Deliverables

- `src/communication/mavlink/transport/mod.rs` - Trait definition
- `src/communication/mavlink/transport/uart.rs` - UART implementation
- `src/communication/mavlink/router.rs` - Multi-transport router
- Updated `src/communication/mavlink/task.rs` - Using router instead of direct UART

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Build for RP2350
./scripts/build-rp2350.sh mavlink_demo

# Flash and verify UART still works
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/mavlink_demo
```

### Acceptance Criteria (Phase Gate)

- `MavlinkTransport` trait compiles and is well-documented
- `UartTransport` implements trait correctly
- `MavlinkRouter` can route messages to/from UART transport
- Existing UART MAVLink functionality works identically (no regressions)
- All clippy warnings resolved
- Code formatted with `cargo fmt`

### Rollback/Fallback

- If trait abstraction introduces regressions: Revert to direct UART implementation, investigate trait design issues
- If router adds significant overhead: Profile and optimize, or simplify router design

---

## Phase 2: WiFi & UDP Implementation

### Phase 2 Goal

- Implement WiFi initialization with compile-time configuration
- Implement UDP transport with GCS endpoint tracking
- Enable concurrent UART and UDP operation
- Verify bidirectional communication with QGroundControl

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `MavlinkTransport` trait, `MavlinkRouter`
  - `embassy-net`, `cyw43`, `cyw43-pio` crates
- Source Code to Modify:
  - `src/platform/rp2350/network.rs` – Create WiFi initialization module
  - `src/communication/mavlink/transport/udp.rs` – Create UDP transport
  - `Cargo.toml` – Add network-related dependencies
  - `scripts/build-rp2350.sh` – Add WiFi credential validation
- Documentation:
  - `docs/adr/ADR-dxdj0-wifi-config-strategy.md` – WiFi configuration approach
  - `docs/adr/ADR-aul2v-udp-primary-transport.md` – UDP transport design

### Phase 2 Tasks

- [ ] **Parameter storage foundation**
  - [ ] Create `src/parameters/storage.rs`
  - [ ] Implement `ParameterStore` with Flash-backed storage
  - [ ] Implement `load_from_flash()` to read parameters at boot
  - [ ] Implement `save_to_flash()` to persist parameters on change
  - [ ] Add parameter types: String, Bool, Int, Float, Ipv4
  - [ ] Implement `is_hidden()` for sensitive parameters (NET_PASS)
- [ ] **WiFi parameter registration**
  - [ ] Define WiFi parameters in `src/parameters/wifi.rs`
  - [ ] Register `NET_SSID` (String<32>)
  - [ ] Register `NET_PASS` (String<63>, hidden)
  - [ ] Register `NET_DHCP` (Bool, default: true)
  - [ ] Register `NET_IP` (Ipv4, default: 0.0.0.0)
  - [ ] Register `NET_NETMASK` (Ipv4, default: 255.255.255.0)
  - [ ] Register `NET_GATEWAY` (Ipv4, default: 0.0.0.0)
- [ ] **MAVLink parameter protocol**
  - [ ] Implement `PARAM_REQUEST_LIST` handler in MAVLink task
  - [ ] Implement `PARAM_REQUEST_READ` handler (respect `is_hidden()`)
  - [ ] Implement `PARAM_SET` handler with Flash persistence
  - [ ] Add parameter hiding logic for `NET_PASS`
  - [ ] Test parameter set/get via QGroundControl
- [ ] **WiFi configuration module**
  - [ ] Create `src/platform/rp2350/network.rs`
  - [ ] Define `WifiConfig` struct (runtime data, not compile-time)
  - [ ] Implement `WifiConfig::from_params(params)` to load from parameter storage
  - [ ] Add support for DHCP and static IP configuration
  - [ ] Handle empty SSID (skip WiFi, UART-only mode)
- [ ] **WiFi initialization function**
  - [ ] Implement `initialize_wifi(spawner, config) -> (Stack, Control)` in `network.rs`
  - [ ] Initialize CYW43439 WiFi driver (PIO, DMA setup)
  - [ ] Spawn network task on Embassy executor
  - [ ] Call `control.join_wpa2(ssid, password)` with 30-second timeout
  - [ ] Implement connection retry with exponential backoff (1s, 2s, 4s, 8s, 16s)
  - [ ] After 5 failures: Disable WiFi, continue UART-only
  - [ ] Configure DHCP client or static IP based on config
  - [ ] Return network stack and WiFi control handles
- [ ] **UDP transport implementation**
  - [ ] Create `src/communication/mavlink/transport/udp.rs`
  - [ ] Define `UdpTransport` struct with socket, GCS endpoint tracking
  - [ ] Implement `UdpTransport::new(stack, port)` to bind UDP socket on port 14550
  - [ ] Implement `track_endpoint(&mut self, SocketAddr)` for GCS discovery
  - [ ] Implement `cleanup_inactive(&mut self)` to remove GCS after 10s timeout
  - [ ] Implement `MavlinkTransport` trait for `UdpTransport`
  - [ ] In `read()`: Track sender endpoint, return received bytes
  - [ ] In `write()`: Broadcast to all active GCS endpoints
- [ ] **Integrate UDP transport**
  - [ ] Update `src/communication/mavlink/task.rs` to load parameters from Flash
  - [ ] Initialize WiFi with runtime parameter configuration
  - [ ] Create `UdpTransport` after WiFi connected
  - [ ] Register UDP transport with `MavlinkRouter`
  - [ ] Verify UART and UDP transports operate concurrently
- [ ] **Add Cargo dependencies**
  - [ ] Add `embassy-net` with UDP feature
  - [ ] Add `cyw43` and `cyw43-pio` for WiFi driver
  - [ ] Add Flash driver for parameter storage
  - [ ] Add serialization crate for parameter persistence (e.g., `postcard`)
  - [ ] Verify no conflicts with existing dependencies

### Phase 2 Deliverables

- `src/parameters/storage.rs` - Flash-backed parameter storage
- `src/parameters/wifi.rs` - WiFi parameter definitions
- `src/platform/rp2350/network.rs` - WiFi initialization and configuration
- `src/communication/mavlink/transport/udp.rs` - UDP transport implementation
- `src/communication/mavlink/parameters.rs` - MAVLink parameter protocol handlers
- Updated `Cargo.toml` - Network and Flash dependencies
- Updated `src/communication/mavlink/task.rs` - Concurrent UART + UDP with parameter loading

### Phase 2 Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Build for Pico 2 W
./scripts/build-rp2350.sh --release mavlink_demo

# Flash to Pico 2 W
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo

# Configure WiFi via QGroundControl (UART connection)
# 1. Connect QGroundControl to UART
# 2. Open Parameters tab
# 3. Set parameters:
#    - NET_SSID = "YourNetwork"
#    - NET_PASS = "YourPassword"
#    - NET_DHCP = 1
# 4. Write parameters (auto-saved to Flash)
# 5. Reboot Pico

# Verify WiFi connection in logs after reboot
# Expected: "WiFi connected", "IP address: <IP>"

# Connect QGroundControl via UDP
# 1. Open QGroundControl
# 2. Add UDP connection (port 14550, listening mode)
# 3. Verify HEARTBEAT messages appear
# 4. Send command, verify ACK received

# Verify UART still works concurrently
# Connect serial console, verify MAVLink messages

# Verify NET_PASS is hidden
# 1. Request parameter list in QGroundControl
# 2. Verify NET_PASS not visible in parameter list
# 3. Verify NET_SSID is visible and readable
```

### Phase 2 Acceptance Criteria

- Parameters successfully saved to Flash and loaded on boot
- WiFi parameters configurable via QGroundControl
- NET_PASS hidden from PARAM_REQUEST_READ/LIST
- WiFi connects successfully within 30 seconds using runtime parameters
- Empty SSID skips WiFi initialization (UART-only mode works)
- Connection retry with exponential backoff working (5 attempts)
- UDP socket binds to port 14550
- QGroundControl receives HEARTBEAT messages via UDP
- Commands sent from QGroundControl receive ACK via UDP
- UART transport continues to work simultaneously
- Memory usage ≤ 50 KB RAM (check via defmt logs, includes parameter storage)
- No clippy warnings

### Phase 2 Rollback/Fallback

- If parameter storage fails: Debug Flash driver, verify sector allocation, check serialization
- If WiFi initialization fails: Verify parameter values in Flash, check WiFi network availability, add debug logging
- If UDP transport fails: Verify port 14550 not blocked, check firewall settings, test with netcat
- If memory exceeds budget: Reduce buffer sizes, optimize parameter storage, profile with embassy-net stats

---

## Phase 3: Testing & Integration

### Phase 3 Goal

- Validate concurrent transport operation
- Measure latency and packet loss
- Test with multiple GCS simultaneously
- Create example configuration documentation
- Verify all success metrics achieved

### Phase 3 Tasks

- [ ] **Unit tests (host)**
  - [ ] Create mock transport implementation in `src/communication/mavlink/transport/mod.rs`
  - [ ] Test router with mock transports (message routing, error isolation)
  - [ ] Test WiFi config parsing with various env var combinations
  - [ ] Test UDP endpoint tracking (add, timeout, max 4 GCS)
- [ ] **Hardware integration tests**
  - [ ] Test UART-only operation (Phase 1 baseline)
  - [ ] Test UDP-only operation (disconnect UART)
  - [ ] Test concurrent UART + UDP operation
  - [ ] Test WiFi reconnection after disconnect
  - [ ] Test GCS disconnection and reconnection
  - [ ] Test multiple GCS (connect 2-4 QGroundControl instances)
- [ ] **Performance validation**
  - [ ] Measure command round-trip latency (COMMAND_LONG → COMMAND_ACK)
  - [ ] Target: < 110ms
  - [ ] Method: Add timestamps to MAVLink task, log latency via defmt
  - [ ] Measure memory usage with network stack active
  - [ ] Target: ≤ 50 KB RAM
  - [ ] Method: Check embassy-net statistics, add defmt memory logs
  - [ ] Monitor packet loss over 1-hour test
  - [ ] Target: < 1%
  - [ ] Method: Monitor MAVLink sequence number gaps
- [ ] **Documentation updates**
  - [ ] Update `docs/mavlink.md` with UDP transport usage
  - [ ] Add WiFi configuration example (environment variables)
  - [ ] Add QGroundControl connection instructions
  - [ ] Update `README.md` to mention network transport capability
  - [ ] Document known limitations (password in binary, no TCP)
- [ ] **Example configuration**
  - [ ] Create `.env.example` file with WiFi configuration template
  - [ ] Add to `.gitignore` to prevent credential leakage
  - [ ] Document in build script comments

### Phase 3 Deliverables

- Unit tests for router and transport abstraction
- Hardware test results (latency, memory, packet loss)
- Updated documentation (`docs/mavlink.md`, `README.md`)
- Example configuration (`.env.example`)
- Performance validation report (latency, memory, packet loss metrics)

### Phase 3 Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Run unit tests
cargo test --lib --quiet

# Build and flash for hardware testing
export WIFI_SSID="TestNetwork"
export WIFI_PASSWORD="TestPassword"
./scripts/build-rp2350.sh --release mavlink_demo
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo

# Performance testing
# 1. Connect QGroundControl via UDP
# 2. Send 100 commands, measure average round-trip time
# 3. Monitor defmt logs for memory usage
# 4. Run for 1 hour, count sequence number gaps
```

### Phase 3 Acceptance Criteria

- All unit tests pass
- Command round-trip latency < 110ms (measured)
- Memory usage ≤ 50 KB RAM (measured)
- Packet loss < 1% over 1-hour test (measured)
- QGroundControl and Mission Planner connect successfully
- UART and UDP operate concurrently without interference
- Documentation complete and accurate
- All clippy warnings resolved
- Code formatted

---

## Definition of Done

- [ ] `cargo check` passes
- [ ] `cargo fmt` applied
- [ ] `cargo clippy --all-targets -- -D warnings` passes
- [ ] `cargo test --lib --quiet` passes (unit tests)
- [ ] Hardware integration tested on Pico 2 W
- [ ] `docs/mavlink.md` updated with UDP transport usage
- [ ] README.md updated with network transport mention
- [ ] WiFi configuration documented with examples
- [ ] QGroundControl connection tested and working
- [ ] Mission Planner connection tested and working
- [ ] Performance metrics validated (latency < 110ms, memory ≤ 50 KB, packet loss < 1%)
- [ ] UART transport regression testing complete (no functionality lost)
- [ ] Concurrent UART + UDP operation verified
- [ ] Build script validates WiFi credentials
- [ ] No unsafe code introduced
- [ ] No vague naming (no "manager", "util" in new code)

## Open Questions

- [ ] Should we add UDP broadcast for GCS auto-discovery? → Next step: Evaluate after basic unicast implementation, add if user feedback indicates need
- [ ] How to handle WiFi disconnection during operation? → Method: Monitor WiFi state, clear GCS endpoints on disconnect, log reconnection attempts
- [ ] Should we implement transport prioritization? → Next step: Start with equal priority, profile performance, add if needed
- [ ] TCP support needed for specific use cases (log downloads)? → Decision: Defer to future based on user feedback and requirements

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
