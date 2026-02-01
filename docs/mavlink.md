# MAVLink Protocol Communication

## Overview

This document provides a usage guide for MAVLink 2.0 protocol communication in pico_trail. MAVLink enables communication with Ground Control Stations (GCS) such as QGroundControl and Mission Planner for monitoring, configuration, and mission management.

## Features

- **Telemetry Streaming**: HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS
- **Parameter Management**: Read and write parameters via GCS
- **Command Protocol**: ARM/DISARM, mode changes, calibration
- **Mission Protocol**: Upload and download waypoint missions
- **GCS Compatibility**: Tested with QGroundControl 4.x and Mission Planner 1.3.x

## Hardware Setup

### UART Wiring

MAVLink communication uses UART transport at 115200 baud. Connect your Pico to a USB-serial adapter:

```
Pico 2 W (RP2350)     USB-Serial Adapter
─────────────────     ──────────────────
GPIO 0 (UART0 TX) ──→ RX
GPIO 1 (UART0 RX) ←── TX
GND               ──→ GND
```

### Supported Hardware

- **Raspberry Pi Pico W** (RP2040) - Cortex-M0+, 264 KB RAM
- **Raspberry Pi Pico 2 W** (RP2350) - Cortex-M33, 520 KB RAM

### USB-Serial Adapters

Common adapters that work well:

- FTDI FT232RL
- CP2102
- CH340G

## Transport Options

### UART Transport

UART is the primary transport for initial configuration and reliable wired communication. All examples support UART by default.

**Advantages:**

- No configuration required - works out of the box
- Reliable wired connection
- No network dependencies
- Suitable for field operations without WiFi

**Limitations:**

- Requires physical connection
- Single GCS only (without router)
- Limited range (cable length)

### UDP Network Transport

UDP transport enables wireless MAVLink communication over WiFi. Available in network-enabled examples (`mavlink_demo_network`).

**Advantages:**

- Wireless communication (WiFi range: \~50m indoors)
- Multiple GCS support (up to 4 simultaneous)
- Standard port 14550 (QGroundControl/Mission Planner compatible)
- Concurrent UART + UDP operation

**Limitations:**

- Requires WiFi network and configuration
- Potential packet loss on weak signal
- Additional \~45 KB RAM overhead

**Quick Start:**

```bash
# 1. Build network-enabled example
./scripts/build-rp2350.sh --release mavlink_demo_network

# 2. Flash and configure WiFi via UART
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo_network

# 3. Connect via UART, set parameters in QGroundControl:
#    NET_SSID = "YourNetwork"
#    NET_PASS = "YourPassword"
#    NET_DHCP = 1

# 4. Reboot device

# 5. Connect QGroundControl via UDP:
#    - Application Settings → Comm Links
#    - Add UDP connection (port 14550, listening mode)
#    - Device auto-discovers when HEARTBEAT sent
```

**Detailed Configuration:**

See [WiFi Configuration Guide](wifi-configuration.md) for complete setup instructions, including:

- Parameter configuration via MAVLink
- DHCP and static IP setup
- Troubleshooting WiFi connection issues
- Security considerations
- Performance tuning

**Multiple GCS Support:**

UDP transport automatically tracks up to 4 GCS:

- Each GCS sends message to port 14550
- Endpoint added to broadcast list
- All telemetry broadcast to active GCS
- Inactive GCS (no traffic for 10s) removed automatically

**Concurrent Operation:**

UART and UDP transports operate simultaneously:

- Messages received from either transport
- Telemetry broadcast to both transports
- Independent failure isolation (UART works if WiFi fails)
- No performance degradation with dual transports

## GCS Configuration

### QGroundControl Setup (UART)

1. **Install QGroundControl 4.x**
   - Download from <https://qgroundcontrol.com/>

2. **Configure Serial Link**
   - Open QGroundControl
   - Go to: Application Settings → Comm Links
   - Click "Add" to create new connection
   - Configure:
     - Name: Pico Trail
     - Type: Serial
     - Serial Port: Select your USB-serial adapter (e.g., /dev/ttyUSB0, COM3)
     - Baud Rate: 115200
     - Data Bits: 8
     - Parity: None
     - Stop Bits: 1

3. **Connect**
   - Click "Connect"
   - Vehicle should appear within 5 seconds (HEARTBEAT timeout)
   - Telemetry should display on main screen

### Mission Planner Setup (UART)

1. **Install Mission Planner 1.3.x** (Windows only)
   - Download from <https://ardupilot.org/planner/>

2. **Connect**
   - Top-right: Select COM port
   - Baud rate: 115200
   - Click "Connect"
   - Wait for parameter download (5 parameters)

3. **Monitor Telemetry**
   - Flight Data view shows attitude, GPS, battery
   - Parameter list shows configurable parameters

### QGroundControl Setup (UDP)

1. **Configure WiFi on Device**
   - Connect via UART first
   - Set `NET_SSID` and `NET_PASS` parameters
   - Reboot device to connect to WiFi

2. **Add UDP Connection**
   - Open QGroundControl
   - Go to: Application Settings → Comm Links
   - Click "Add" to create new connection
   - Configure:
     - Name: Pico Trail WiFi
     - Type: UDP
     - Listening Port: 14550
     - Server Address: (leave empty for listening mode)

3. **Connect**
   - Click "Connect"
   - Device auto-appears when HEARTBEAT received
   - Verify connection in defmt logs

**Note:** UDP uses "listening mode" (server) - QGroundControl waits for device to send HEARTBEAT.

### Mission Planner Setup (UDP)

1. **Configure Connection**
   - Top-right: Connection type dropdown
   - Select "UDP"
   - Port: 14550
   - Click "Connect"

2. **Monitor**
   - Device appears when HEARTBEAT received
   - Flight Data view shows telemetry

## Supported Messages

### Outbound Telemetry

| Message     | ID  | Rate | Description                               |
| ----------- | --- | ---- | ----------------------------------------- |
| HEARTBEAT   | 0   | 1Hz  | Vehicle type, armed status, system status |
| SYS_STATUS  | 1   | 1Hz  | Battery voltage/current, CPU load         |
| ATTITUDE    | 30  | 10Hz | Roll, pitch, yaw, rotation rates          |
| GPS_RAW_INT | 24  | 5Hz  | Latitude, longitude, altitude, fix type   |

### Inbound Commands

| Message              | ID  | Description                              |
| -------------------- | --- | ---------------------------------------- |
| PARAM_REQUEST_LIST   | 21  | Request all parameters                   |
| PARAM_REQUEST_READ   | 20  | Request specific parameter               |
| PARAM_SET            | 23  | Set parameter value                      |
| COMMAND_LONG         | 76  | Execute command (arm, mode change, etc.) |
| MISSION_COUNT        | 44  | Start mission upload                     |
| MISSION_ITEM_INT     | 73  | Upload waypoint                          |
| MISSION_REQUEST_LIST | 43  | Request mission download                 |
| MISSION_REQUEST_INT  | 51  | Request specific waypoint                |

## Parameters

### Stream Rate Control

Control telemetry rates via parameters (units: Hz):

| Parameter   | Default | Range | Description              |
| ----------- | ------- | ----- | ------------------------ |
| SR_EXTRA1   | 10      | 0-50  | ATTITUDE message rate    |
| SR_POSITION | 5       | 0-50  | GPS_RAW_INT message rate |
| SR_RC_CHAN  | 5       | 0-50  | RC_CHANNELS message rate |
| SR_RAW_SENS | 5       | 0-50  | IMU_SCALED message rate  |

### System Configuration

| Parameter     | Default | Range | Description       |
| ------------- | ------- | ----- | ----------------- |
| SYSID_THISMAV | 1       | 1-255 | MAVLink system ID |

### Network Configuration (UDP Transport)

Configure WiFi and network settings via MAVLink parameters. These parameters are only available in network-enabled examples (`mavlink_demo_network`).

| Parameter   | Type   | Default       | Description                          |
| ----------- | ------ | ------------- | ------------------------------------ |
| NET_SSID    | String | "" (empty)    | WiFi network name (max 32 chars)     |
| NET_PASS    | String | "" (hidden)   | WiFi password (max 63 chars, hidden) |
| NET_DHCP    | Bool   | 1 (DHCP)      | Use DHCP (1) or static IP (0)        |
| NET_IP      | IPv4   | 0.0.0.0       | Static IP address                    |
| NET_NETMASK | IPv4   | 255.255.255.0 | Network mask                         |
| NET_GATEWAY | IPv4   | 0.0.0.0       | Gateway address                      |

**Security Note:** `NET_PASS` is write-only - it can be set via `PARAM_SET` but is hidden from `PARAM_REQUEST_READ` and `PARAM_REQUEST_LIST` for security. The password is stored unencrypted in Flash memory.

**Configuration Workflow:**

1. Connect via UART
2. Set `NET_SSID` and `NET_PASS` in QGroundControl Parameters tab
3. Optionally configure `NET_DHCP`, `NET_IP`, etc.
4. Reboot device to apply WiFi configuration
5. Device connects to WiFi automatically on boot

**UART-Only Mode:** Leave `NET_SSID` empty to disable WiFi and operate in UART-only mode.

### Parameter Persistence

Parameters are stored in Flash memory and persist across power cycles. Changes via PARAM_SET are automatically saved.

## Command Protocol

### ARM/DISARM

Arm or disarm the vehicle:

```
MAV_CMD_COMPONENT_ARM_DISARM (400)
  param1: 1.0 = arm, 0.0 = disarm
```

**Safety Checks**:

- Cannot arm if already armed
- Cannot arm if battery voltage < 10.0V
- Can always disarm

**Example** (QGroundControl):

1. Vehicle Setup → Safety
2. Click "Arm" button
3. Wait for COMMAND_ACK confirmation

### Mode Change

Change flight mode:

```
MAV_CMD_DO_SET_MODE (176)
  param2: mode number (0=Manual, 1=Stabilize, 2=Hold, 3=Auto)
```

**Available Modes**:

- 0: Manual
- 1: Stabilize
- 2: Hold
- 3: Auto

### Calibration

Request sensor calibration:

```
MAV_CMD_PREFLIGHT_CALIBRATION (241)
```

**Note**: Currently placeholder - returns ACCEPTED but performs no calibration.

## Mission Protocol

### Upload Mission

1. **Prepare Mission in GCS**
   - QGroundControl: Plan view, add waypoints
   - Mission Planner: Flight Plan view, right-click to add waypoints

2. **Upload**
   - Click "Upload" or "Write WPs"
   - GCS sends MISSION_COUNT
   - Autopilot requests each MISSION_ITEM_INT sequentially
   - Upload complete when all items received

3. **Verify**
   - Check GCS for "Mission upload complete"
   - Waypoints stored in RAM (max 50 waypoints)

### Download Mission

1. **Request Download**
   - QGroundControl: Plan view → Menu → Download
   - Mission Planner: Flight Plan → Read WPs

2. **Download**
   - GCS sends MISSION_REQUEST_LIST
   - Autopilot sends MISSION_COUNT
   - GCS requests each waypoint
   - Download complete when all items received

### Mission Storage

- **Capacity**: 50 waypoints maximum
- **Persistence**: RAM only (lost on power cycle)
- **Execution**: Storage only - execution deferred to future task

## Building and Flashing

### Build UART-Only Example

```bash
# Build for RP2350 (Pico 2 W)
./scripts/build-rp2350.sh --release mavlink_demo

# Build for RP2040 (Pico W)
# TODO: Add RP2040 build script when available
```

### Build Network-Enabled Example

```bash
# Build network example with UART + UDP support
./scripts/build-rp2350.sh --release mavlink_demo_network
```

**Note:** WiFi credentials are configured via MAVLink parameters after flashing, not at build time. See [WiFi Configuration Guide](wifi-configuration.md).

### Flash with probe-rs (Recommended)

Shows defmt logs in real-time:

```bash
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
```

Expected output:

```
pico_trail MAVLink Demo
========================

UART Configuration:
  - Baud rate: 115200
  - UART0 TX: GPIO 0
  - UART0 RX: GPIO 1

Telemetry Rates:
  - HEARTBEAT: 1Hz
  - ATTITUDE: 10Hz (SR_EXTRA1)
  - GPS_RAW_INT: 5Hz (SR_POSITION)
  - SYS_STATUS: 1Hz

MAVLink task started
```

### Flash via UF2 (No logs)

1. Hold BOOTSEL button while connecting USB
2. Pico appears as mass storage device
3. Copy `target/mavlink_demo.uf2` to mounted drive
4. Pico automatically reboots and runs

## Testing and Verification

### Connection Test

1. Flash mavlink_demo example
2. Connect UART to USB-serial adapter
3. Open GCS and configure serial link
4. Click "Connect"
5. **Expected**: Vehicle appears within 5 seconds
6. **Verify**: HEARTBEAT counter increments in GCS

### Telemetry Test

1. Connect GCS
2. Monitor main telemetry display
3. **Expected**:
   - Attitude indicators update at 10Hz (smooth)
   - GPS position updates at 5Hz
   - Battery voltage displayed (placeholder: 12.0V)
   - CPU load displayed

### Parameter Test

1. Connect GCS
2. Go to Parameters view
3. **Expected**: 5 parameters listed
4. Change SR_EXTRA1 to 20
5. Click "Write"
6. **Verify**: ATTITUDE rate increases to 20Hz
7. Disconnect and reconnect
8. **Verify**: Parameter persists (still 20)

### Command Test

1. Connect GCS
2. **ARM Test**:
   - Click "Arm" button
   - **Expected**: Vehicle arms, HEARTBEAT shows armed
3. **DISARM Test**:
   - Click "Disarm" button
   - **Expected**: Vehicle disarms
4. **Mode Change Test**:
   - Change flight mode dropdown to "Auto"
   - **Expected**: Mode changes, confirmed in HEARTBEAT

### Mission Test

1. Connect GCS
2. **Upload Test**:
   - Create mission with 5 waypoints
   - Click "Upload"
   - **Expected**: "Mission upload complete"
3. **Download Test**:
   - Clear mission in GCS
   - Click "Download"
   - **Expected**: 5 waypoints appear

## Performance Metrics

### Memory Usage (UART-Only)

- **Static**: \~6 KB (buffers, state, parameters)
- **Stack**: \~2 KB (during message processing)
- **Total**: < 10 KB RAM (< 4% on RP2040, < 2% on RP2350)

### Memory Usage (UART + UDP)

- **WiFi driver**: \~20 KB RAM (CYW43439)
- **Network stack**: \~15 KB RAM (embassy-net)
- **UDP buffers**: \~8 KB RAM (RX/TX)
- **Transport overhead**: \~2 KB RAM (routing, GCS tracking)
- **Total network**: \~45 KB RAM (\~17% on RP2040, \~8.7% on RP2350)

### CPU Load

- **Telemetry streaming**: \~3-5% at default rates
- **Parameter operations**: \~1-2% during list download
- **Mission upload**: \~2-3% during transfer
- **Total overhead**: < 10% during normal operation

### Message Rates

At default parameters (measured over 60 seconds):

| Message     | Expected      | Bandwidth                  |
| ----------- | ------------- | -------------------------- |
| HEARTBEAT   | 60 msgs       | \~1.3 KB                   |
| ATTITUDE    | 600 msgs      | \~24 KB                    |
| GPS_RAW_INT | 300 msgs      | \~12.6 KB                  |
| SYS_STATUS  | 60 msgs       | \~2.6 KB                   |
| **Total**   | **1020 msgs** | **\~40 KB (\~5% of UART)** |

### Latency

**UART Transport:**

- **Command acknowledgment**: < 10ms (COMMAND_LONG → COMMAND_ACK)
- **Parameter read**: < 50ms (PARAM_REQUEST_READ → PARAM_VALUE)
- **Connection timeout**: 5 seconds (missed HEARTBEAT)

**UDP Transport:**

- **Command acknowledgment**: < 110ms (COMMAND_LONG → COMMAND_ACK)
- **WiFi PHY latency**: \~10-20ms
- **UDP stack processing**: \~5-10ms
- **MAVLink encode/decode**: \~1-2ms
- **Router overhead**: < 1ms
- **Total additional latency**: \~20-33ms vs UART

### Error Rates

- **CRC failures**: < 0.1% (measured over 10,000 messages)
- **Buffer overflows**: 0 (at default rates)
- **Dropped messages**: 0 (at default rates)

## Troubleshooting

### GCS Won't Connect

**Symptoms**: "Waiting for vehicle..." never completes

**Causes**:

1. Wrong COM port selected
2. Wrong baud rate (must be 115200)
3. UART wiring incorrect (TX/RX swapped)
4. USB-serial adapter not recognized

**Solutions**:

1. Verify COM port in Device Manager / ls /dev/ttyUSB\*
2. Check GCS baud rate setting
3. Swap TX/RX connections
4. Try different USB port or adapter

### No Telemetry Displayed

**Symptoms**: Vehicle connected but no attitude/GPS data

**Causes**:

1. Stream rates set to 0
2. MAVLink task not running

**Solutions**:

1. Check SR_EXTRA1 and SR_POSITION parameters (should be > 0)
2. Reset parameters to defaults
3. Check defmt logs for errors

### Parameter Changes Don't Persist

**Symptoms**: Parameters reset to defaults after power cycle

**Causes**:

1. Flash write failed
2. Flash corruption

**Solutions**:

1. Check defmt logs for Flash errors
2. Re-flash firmware
3. Verify Flash hardware integrity

### High CPU Load

**Symptoms**: CPU load > 20% in SYS_STATUS

**Causes**:

1. Stream rates too high
2. GCS sending too many requests

**Solutions**:

1. Reduce SR_EXTRA1 to 5Hz
2. Reduce SR_POSITION to 2Hz
3. Disconnect and reconnect GCS

### Mission Upload Fails

**Symptoms**: GCS reports "Mission upload timeout"

**Causes**:

1. Too many waypoints (> 50)
2. Communication errors

**Solutions**:

1. Reduce mission size to < 50 waypoints
2. Check UART connection quality
3. Try slower upload (if GCS supports)

## Advanced Usage

### Custom Stream Rates

Optimize bandwidth by adjusting stream rates:

```
Low bandwidth (56k modem):
  SR_EXTRA1 = 2 (ATTITUDE 2Hz)
  SR_POSITION = 1 (GPS 1Hz)
  Total: ~8 KB/min

High bandwidth (USB):
  SR_EXTRA1 = 50 (ATTITUDE 50Hz)
  SR_POSITION = 10 (GPS 10Hz)
  Total: ~200 KB/min
```

### Multiple GCS Connections

**UART Transport:** Single GCS only. For multiple GCS via UART:

1. Use mavproxy as router: `mavproxy --master=/dev/ttyUSB0 --baudrate=115200 --out=udp:192.168.1.100:14550 --out=udp:192.168.1.101:14550`
2. Connect each GCS to UDP endpoint

**UDP Transport:** Up to 4 simultaneous GCS automatically supported:

1. Each GCS connects to UDP port 14550
2. GCS endpoints auto-discovered when they send messages
3. All telemetry broadcast to active GCS
4. Inactive GCS (no traffic for 10s) automatically removed

### Message Filtering

To reduce bandwidth, modify telemetry streamer source:

1. Edit `src/communication/mavlink/handlers/telemetry.rs`
2. Comment out unwanted messages
3. Rebuild and flash

### Custom Parameters

To add application-specific parameters:

1. Edit `src/communication/mavlink/handlers/param.rs`
2. Add parameter to initialization list in `ParamHandler::new()`
3. Define name, type, default, min, max
4. Rebuild and flash
5. Parameter appears in GCS parameter list

## API Reference

For detailed API documentation:

```bash
cargo doc --no-deps --features pico2_w --open
```

Key modules:

- `pico_trail::communication::mavlink` - Top-level MAVLink module
- `pico_trail::communication::mavlink::handlers` - Message handlers
- `pico_trail::communication::mavlink::router` - Message routing
- `pico_trail::core::parameters` - Parameter system

## External Resources

- **MAVLink Protocol**: <https://mavlink.io/en/>
- **Common Messages**: <https://mavlink.io/en/messages/common.html>
- **QGroundControl**: <https://qgroundcontrol.com/>
- **Mission Planner**: <https://ardupilot.org/planner/>
- **ArduPilot MAVLink**: <https://ardupilot.org/dev/docs/mavlink-basics.html>

## Related Documentation

- [WiFi Configuration Guide](wifi-configuration.md) - Detailed WiFi setup for UDP transport
- [Architecture](architecture.md) - System architecture and components
- [T-00002 Task](archive/tasks/T-00002-mavlink-communication/) - UART MAVLink implementation
- [T-00006 Task](tasks/T-00006-mavlink-network-transport/) - Network transport implementation
- [ADR-00002](adr/ADR-00002-mavlink-implementation.md) - MAVLink architecture decision
- [ADR-00007](adr/ADR-00007-transport-abstraction.md) - Transport abstraction design
- [ADR-00008](adr/ADR-00008-udp-primary-transport.md) - UDP transport decision
- [ADR-00009](adr/ADR-00009-wifi-config-strategy.md) - WiFi configuration strategy
- [FR-00005](requirements/FR-00005-mavlink-protocol.md) - MAVLink requirements
- [FR-00010](requirements/FR-00010-udp-network-transport.md) - UDP transport requirements

## License

This MAVLink implementation uses the rust-mavlink crate (MIT/Apache-2.0) for message parsing and serialization.
