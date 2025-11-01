# WiFi Configuration Guide

This guide explains how to configure WiFi for MAVLink network transport on Pico 2 W.

## Overview

WiFi credentials are stored in Flash memory using the parameter storage system. Configuration is performed via the MAVLink parameter protocol, allowing secure credential management without hardcoding in firmware.

## Configuration Parameters

| Parameter     | Type   | Description                          | Default       |
| ------------- | ------ | ------------------------------------ | ------------- |
| `NET_SSID`    | String | WiFi network name (max 32 chars)     | "" (empty)    |
| `NET_PASS`    | String | WiFi password (max 63 chars, hidden) | "" (empty)    |
| `NET_DHCP`    | Bool   | Use DHCP (1) or static IP (0)        | 1 (DHCP)      |
| `NET_IP`      | IPv4   | Static IP address                    | 0.0.0.0       |
| `NET_NETMASK` | IPv4   | Network mask                         | 255.255.255.0 |
| `NET_GATEWAY` | IPv4   | Gateway address                      | 0.0.0.0       |

## Quick Start

### Step 1: Build and Flash Firmware

```bash
# Build network-enabled example
./scripts/build-rp2350.sh --release mavlink_demo_network

# Flash to Pico 2 W (with defmt logs)
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo_network

# Alternative: Flash via UF2 (no logs)
# 1. Hold BOOTSEL button while connecting USB
# 2. Copy target/mavlink_demo_network.uf2 to mounted drive
```

### Step 2: Configure WiFi via MAVLink

Connect QGroundControl or Mission Planner to the UART port (115200 baud):

1. Open Parameters tab
2. Set `NET_SSID` to your WiFi network name
3. Set `NET_PASS` to your WiFi password
4. Optionally configure `NET_DHCP`, `NET_IP`, `NET_NETMASK`, `NET_GATEWAY`
5. Parameters are automatically saved to Flash

### Step 3: Reboot Device

Reboot the device to apply WiFi configuration:

- Power cycle the device, OR
- Send `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` command

### Step 4: Verify WiFi Connection

Monitor defmt logs (via probe-rs) for connection status:

```
WiFi configured - initializing network...
  SSID: YourNetwork
  DHCP: true
WiFi connected
UDP transport initialized on port 14550
Transport router initialized with UART + UDP
```

### Step 5: Connect via UDP

1. Open QGroundControl
2. Go to Application Settings → Comm Links
3. Add new UDP connection:
   - Port: 14550
   - Mode: Listening (server mode)
4. Connect
5. Device should appear automatically when HEARTBEAT is received

## Configuration Examples

### Example 1: Home Network with DHCP

```
NET_SSID = "MyHomeNetwork"
NET_PASS = "SecurePassword123"
NET_DHCP = 1
```

Device will obtain IP address automatically via DHCP.

### Example 2: Static IP Configuration

```
NET_SSID = "FieldNetwork"
NET_PASS = "DronePassword456"
NET_DHCP = 0
NET_IP = "192.168.1.100"
NET_NETMASK = "255.255.255.0"
NET_GATEWAY = "192.168.1.1"
```

Device will use the specified static IP address.

### Example 3: UART-Only Mode

```
NET_SSID = ""
```

Leave `NET_SSID` empty to disable WiFi. Device will operate in UART-only mode.

## Fallback Behavior

The firmware includes automatic fallback mechanisms:

### Empty SSID

- **Behavior**: WiFi initialization skipped
- **Result**: UART-only operation
- **Use case**: Testing without network, field operations without WiFi

### WiFi Connection Failure

- **Retry strategy**: Exponential backoff (1s, 2s, 4s, 8s, 16s)
- **Max attempts**: 5 retries
- **After 5 failures**: Disable WiFi, continue UART-only
- **Logging**: Connection errors logged via defmt

## Security Considerations

### Password Storage

- **Storage location**: Flash memory parameter block
- **Encryption**: Not encrypted in current implementation
- **Risk**: Firmware extraction reveals WiFi password
- **Mitigation**:
  - Use strong WiFi network password
  - Implement physical device security
  - Future enhancement: RP2350 Flash encryption support

### Parameter Privacy

- **NET_PASS visibility**: Hidden from MAVLink `PARAM_REQUEST_READ` and `PARAM_REQUEST_LIST`
- **Write-only**: Password can be set via `PARAM_SET` but not read remotely
- **Local access**: Password visible in Flash dump (physical access required)

### Network Security

- **MAVLink authentication**: Not implemented (MAVLink v2.0 base spec)
- **Recommended**: Use WPA2-secured WiFi network
- **Trusted network only**: Operate on trusted networks to prevent unauthorized GCS access

## Troubleshooting

### WiFi Not Connecting

1. **Verify SSID/password**:
   - Check parameter values in QGroundControl
   - Ensure no typos in network name or password
2. **Check network availability**:
   - Ensure WiFi network is active and in range
   - Verify 2.4 GHz network (CYW43439 does not support 5 GHz)
3. **Monitor logs**:
   - Use `probe-rs run` to view defmt logs
   - Look for connection error messages

### UDP Not Receiving Messages

1. **Verify WiFi connection**: Check logs for "WiFi connected" message
2. **Check firewall settings**:
   - Ensure UDP port 14550 is not blocked
   - Temporarily disable firewall for testing
3. **Verify GCS configuration**:
   - QGroundControl: Use "Listening" mode, not "Connect" mode
   - Port: 14550
   - Protocol: UDP
4. **Network routing**:
   - Ensure GCS and device are on same subnet
   - Check router settings for UDP forwarding

### Concurrent UART + UDP Issues

1. **UART still works**: UART transport is independent of WiFi status
2. **Message duplication**: Expected behavior - messages sent to both transports
3. **GCS confusion**: Some GCS may show duplicate vehicles - configure to filter by System ID

## Performance

### Expected Metrics

- **Latency**: < 110ms round-trip (COMMAND_LONG → COMMAND_ACK)
- **Memory usage**: \~45 KB RAM for network stack + transports
- **Packet loss**: < 1% on local WiFi (100 Mbps, 2.4 GHz)
- **Concurrent GCS**: Up to 4 simultaneous connections

### Monitoring

Monitor performance via defmt logs:

```
RX 42 bytes from transport 1  # UDP receive
TX HEARTBEAT (28 bytes to all transports)  # Broadcast send
```

Transport statistics available in code:

```rust
let stats = router.stats();
defmt::info!("Bytes RX: {}, TX: {}", stats.bytes_received, stats.bytes_sent);
```

## Advanced Configuration

### Multiple GCS Connections

UDP transport automatically tracks up to 4 GCS endpoints:

1. GCS sends message to port 14550
2. Endpoint added to active GCS list
3. All outbound messages broadcast to active GCS list
4. Inactive GCS (no traffic for 10s) automatically removed

### Network Diagnostics

Use MAVLink `COMMAND_LONG` with `MAV_CMD_REQUEST_MESSAGE` to request network statistics (future enhancement).

### Firmware Updates Over Network

Not currently supported. Use UART or UF2 bootloader for firmware updates.

## References

- [MAVLink Protocol](https://mavlink.io/)
- [QGroundControl](http://qgroundcontrol.com/)
- [Mission Planner](https://ardupilot.org/planner/)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [CYW43439 WiFi Chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/cyw43439/)
