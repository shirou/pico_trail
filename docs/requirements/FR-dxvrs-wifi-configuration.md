# FR-dxvrs WiFi Configuration

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [FR-ydttj-udp-network-transport](FR-ydttj-udp-network-transport.md)
- Related Tasks:
  - [T-oq110-\*](../tasks/T-oq110-*/README.md)

## Requirement Statement

The system shall provide WiFi connectivity configuration including SSID, password, and IP address settings to enable network transport operation.

## Rationale

WiFi configuration is essential for network transport functionality:

- **Network Access**: Must connect to existing WiFi network (Station mode)
- **Flexible Deployment**: Different networks for development, testing, production
- **IP Configuration**: Support both DHCP (auto) and static IP
- **Connection Status**: Visibility into WiFi state for troubleshooting

Without proper WiFi configuration, UDP/TCP transports cannot operate.

## User Story (if applicable)

As a system operator, I want to configure WiFi credentials and IP settings, so that the autopilot can connect to my local network and communicate with ground control stations over WiFi.

## Acceptance Criteria

- [ ] WiFi SSID configurable via MAVLink parameter (`NET_SSID`)
- [ ] WiFi password configurable via MAVLink parameter (`NET_PASS`)
- [ ] Password hidden from `PARAM_REQUEST_READ` and `PARAM_REQUEST_LIST` (write-only)
- [ ] IP address configuration: DHCP or static IP selectable via parameter (`NET_DHCP`)
- [ ] Static IP configuration: IP address (`NET_IP`), netmask (`NET_NETMASK`), gateway (`NET_GATEWAY`)
- [ ] Parameters persisted to Flash storage
- [ ] Parameters loaded from Flash on boot
- [ ] WiFi connection established on system startup using loaded parameters
- [ ] Connection retry on failure (exponential backoff, max 5 attempts)
- [ ] WiFi status visible in telemetry (connected, disconnected, IP address)
- [ ] Support WPA2 security (CYW43439 hardware requirement)
- [ ] Connection timeout: 30 seconds before retry
- [ ] MAC address accessible for network troubleshooting
- [ ] Configuration changeable without firmware reflash
- [ ] Works with QGroundControl parameter editor

## Technical Details (if applicable)

### Functional Requirement Details

**Configuration Storage**:

Runtime parameter configuration via MAVLink:

```rust
// Parameters registered with parameter system
pub struct WifiParams {
    ssid: ParamString<32>,      // NET_SSID (max 32 chars)
    password: ParamString<63>,  // NET_PASS (max 63 chars, WPA2 limit)
    use_dhcp: ParamBool,        // NET_DHCP (true=DHCP, false=static)
    static_ip: ParamIpv4,       // NET_IP (used if DHCP=false)
    netmask: ParamIpv4,         // NET_NETMASK
    gateway: ParamIpv4,         // NET_GATEWAY
}

impl Default for WifiParams {
    fn default() -> Self {
        Self {
            ssid: ParamString::new(""),
            password: ParamString::new(""),
            use_dhcp: ParamBool::new(true),
            static_ip: ParamIpv4::new(Ipv4Addr::UNSPECIFIED),
            netmask: ParamIpv4::new(Ipv4Addr::new(255, 255, 255, 0)),
            gateway: ParamIpv4::new(Ipv4Addr::UNSPECIFIED),
        }
    }
}
```

**Parameter Configuration via GCS**:

1. Connect to Pico via UART (WiFi not configured yet)
2. Open QGroundControl Parameters tab
3. Set WiFi parameters:
   - `NET_SSID` = "MyNetwork"
   - `NET_PASS` = "SecurePassword123"
   - `NET_DHCP` = 1 (or 0 for static IP)
4. Write parameters (saved to Flash automatically)
5. Reboot Pico → WiFi connects with new settings

**Connection Flow**:

```
1. System boot
2. Load parameters from Flash
3. Initialize WiFi driver (CYW43439)
4. Load WiFi configuration from parameters
5. If SSID empty: Skip WiFi initialization, UART-only mode
6. Attempt connection to SSID with password (timeout: 30s)
7. If DHCP: Wait for IP lease (timeout 30s)
8. If static: Configure IP/netmask/gateway
9. If success: Mark WiFi as connected, proceed to UDP transport init
10. If failure: Log error, retry after delay (1s, 2s, 4s, 8s, 16s backoff)
11. After 5 failures: Disable WiFi, continue with UART only
```

**WiFi Status Telemetry**:

Add to SYS_STATUS or custom MAVLink message:

```rust
struct WifiStatus {
    connected: bool,
    ssid: [u8; 32],
    ip_address: Ipv4Addr,
    rssi: i8,  // Signal strength in dBm
    mac_address: [u8; 6],
}
```

Send WiFi status at 1Hz when connected, or on state change.

**Security Considerations**:

- Password stored in Flash plaintext (no secure storage on RP2040/RP2350)
- **Password Hiding**: `NET_PASS` not returned in `PARAM_REQUEST_READ` or `PARAM_REQUEST_LIST`
- **Write-Only Parameter**: Can set password via `PARAM_SET`, never read back
- GCS verifies WiFi connection success without password readout
- WPA2 encryption handled by CYW43439 firmware
- **Security Trade-off**: Password still extractable via firmware dump (hardware limitation)

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

WiFi configuration must work on both Pico W (RP2040) and Pico 2 W (RP2350), both using CYW43439 WiFi chip. Use `cyw43` crate for unified driver interface.

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                           | Validation                               |
| -------------------------------------- | ------ | ---------- | ---------------------------------------------------- | ---------------------------------------- |
| Password stored in plaintext           | Medium | High       | Use compile-time config, document security trade-off | Security review, document in ADR         |
| Connection failure (wrong credentials) | High   | Medium     | Clear error messages, retry logic                    | Test with invalid credentials            |
| DHCP timeout on slow networks          | Medium | Low        | 30s timeout, fallback to retry                       | Test on congested network                |
| WiFi driver crash (CYW43439 firmware)  | High   | Low        | Watchdog timer, automatic reinitialization           | Stress test WiFi connection              |
| Parameter size limits (SSID/password)  | Low    | Low        | Validate length before storing                       | Test with 32-char SSID, 63-char password |

## Implementation Notes

Preferred approaches:

- Start with compile-time configuration (simpler, more secure)
- Use `cyw43` crate for WiFi driver (Embassy project)
- Implement exponential backoff for retries
- Add WiFi status to existing telemetry (extend SYS_STATUS or custom message)
- Store MAC address for network troubleshooting

Known pitfalls:

- CYW43439 requires firmware blobs (`43439A0.bin`, `43439A0_clm.bin`)
- WiFi initialization takes \~5 seconds (async, don't block main task)
- DHCP can timeout on busy networks (use longer timeout)
- WPA2 password max 63 characters, SSID max 32 characters
- Connection state machine needs proper async handling

Related code areas:

- `src/platform/*/network.rs` - WiFi driver initialization
- `src/core/parameters/` - Parameter storage (if runtime config)
- `src/communication/mavlink/handlers/telemetry.rs` - WiFi status telemetry

Suggested libraries:

- `cyw43` - CYW43439 WiFi driver (Embassy)
- `cyw43-pio` - PIO-based SPI driver for RP2040/RP2350
- `embassy-net` - Network stack (DHCP client included)

## External References

- CYW43439 WiFi: <https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/cyw43439/>
- Embassy CYW43 Driver: <https://github.com/embassy-rs/embassy/tree/main/cyw43>
- WPA2 Specification: <https://www.wi-fi.org/discover-wi-fi/security>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
