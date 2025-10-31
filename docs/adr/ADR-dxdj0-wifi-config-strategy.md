# ADR-dxdj0 WiFi Configuration via Compile-Time Constants

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-dxvrs-wifi-configuration](../requirements/FR-dxvrs-wifi-configuration.md)
- Supersedes ADRs: N/A
- Related Tasks: Will be created after approval

## Context

Network transport requires WiFi configuration including SSID, password, and IP settings. The configuration mechanism affects security, flexibility, and user experience.

### Problem

WiFi credentials must be provided to connect to network:

- **SSID**: Network name (public)
- **Password**: WPA2 passphrase (sensitive)
- **IP Configuration**: DHCP or static IP

Configuration options:

1. **Compile-time**: Hardcoded or environment variables at build
2. **Runtime parameters**: Stored in Flash, configurable via GCS
3. **WiFi AP mode**: Pico creates access point for initial setup
4. **Serial setup**: Configure via UART before WiFi enabled

### Prior Art

**ArduPilot**:

- Compile-time WiFi in early versions
- Later: Runtime parameters (NET_SSID, NET_PASS)
- Stored in Flash parameter storage
- Configurable via GCS

**ESP32-based autopilots**:

- WiFi Manager: AP mode for initial setup
- Web interface for configuration
- Stored in NVS (non-volatile storage)

**Consumer IoT devices**:

- BLE for initial pairing
- AP mode for WiFi setup
- Mobile app for configuration

### Constraints

- **Security**: Password stored in plaintext (RP2040/RP2350 have no secure storage)
- **Development**: Frequent network changes during testing
- **Production**: Fixed network or field reconfiguration needed
- **User Experience**: Setup process should be simple

## Success Metrics

- WiFi connects successfully on first boot (< 30 seconds)
- No password leakage via parameter readout
- Configuration change possible without full reflash
- Development workflow efficient (quick network changes)

## Decision

**We will use runtime parameters via MAVLink PARAM_SET for WiFi configuration, stored in Flash parameter storage with password obfuscation.**

### Configuration Method

WiFi credentials configurable at runtime via MAVLink parameters:

```rust
// Runtime parameter storage (Flash-backed)
pub struct WifiParams {
    pub ssid: ParamString<32>,      // NET_SSID (readable)
    pub password: ParamString<63>,  // NET_PASS (hidden from PARAM_REQUEST_READ)
    pub use_dhcp: ParamBool,        // NET_DHCP (default: true)
    pub static_ip: ParamIpv4,       // NET_IP (if DHCP disabled)
    pub netmask: ParamIpv4,         // NET_NETMASK
    pub gateway: ParamIpv4,         // NET_GATEWAY
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

### Parameter Configuration via QGroundControl

1. **Connect via UART** (WiFi not configured yet)
2. **Open Parameters tab** in QGroundControl
3. **Set WiFi parameters**:
   - `NET_SSID` = "MyNetwork"
   - `NET_PASS` = "SecurePassword123"
   - `NET_DHCP` = 1 (or 0 for static IP)
4. **Write parameters** (saved to Flash)
5. **Reboot Pico** (WiFi connects with new settings)

### Code Integration

```rust
// Load WiFi configuration from parameters
pub struct WifiConfig {
    pub ssid: String<32>,
    pub password: String<63>,
    pub use_dhcp: bool,
    pub static_ip: Option<Ipv4Address>,
    pub netmask: Option<Ipv4Address>,
    pub gateway: Option<Ipv4Address>,
}

impl WifiConfig {
    pub fn from_params(params: &WifiParams) -> Self {
        Self {
            ssid: params.ssid.get().clone(),
            password: params.password.get().clone(),
            use_dhcp: params.use_dhcp.get(),
            static_ip: if params.use_dhcp.get() {
                None
            } else {
                Some(params.static_ip.get())
            },
            netmask: if params.use_dhcp.get() {
                None
            } else {
                Some(params.netmask.get())
            },
            gateway: if params.use_dhcp.get() {
                None
            } else {
                Some(params.gateway.get())
            },
        }
    }
}

// Use at startup
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Load parameters from Flash
    let params = load_parameters().await;
    let wifi_config = WifiConfig::from_params(&params.wifi);

    // Initialize WiFi with runtime configuration
    let mut control = initialize_wifi().await;
    control.join_wpa2(&wifi_config.ssid, &wifi_config.password).await?;

    if wifi_config.use_dhcp {
        let config = Config::dhcpv4(Default::default());
        // ...
    } else {
        let config = Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(wifi_config.static_ip.unwrap(), 24),
            gateway: wifi_config.gateway,
            dns_servers: Vec::new(),
        });
        // ...
    }
}
```

### Parameter Storage Implementation

WiFi parameters stored in Flash using existing parameter storage system:

```rust
// Parameter definitions (src/parameters/mod.rs)
pub fn register_wifi_parameters(store: &mut ParameterStore) {
    store.register_string("NET_SSID", "", 32);          // SSID (max 32 chars)
    store.register_string("NET_PASS", "", 63);          // Password (max 63 chars, WPA2 limit)
    store.register_bool("NET_DHCP", true);              // Use DHCP (default: true)
    store.register_ipv4("NET_IP", Ipv4Addr::UNSPECIFIED);      // Static IP
    store.register_ipv4("NET_NETMASK", Ipv4Addr::new(255, 255, 255, 0));  // Netmask
    store.register_ipv4("NET_GATEWAY", Ipv4Addr::UNSPECIFIED);  // Gateway
}

// Password hiding (src/communication/mavlink/parameters.rs)
impl ParameterHandler {
    fn is_hidden(&self, param_id: &str) -> bool {
        match param_id {
            "NET_PASS" => true,  // Don't send in PARAM_VALUE response
            _ => false,
        }
    }

    fn handle_param_request_read(&mut self, param_id: &str) -> Option<ParamValue> {
        if self.is_hidden(param_id) {
            None  // Act as if parameter doesn't exist
        } else {
            self.store.get(param_id)
        }
    }
}

// Flash storage persistence
// Parameters automatically persisted to Flash on PARAM_SET
// Loaded from Flash on boot
```

### Password Security

**Obfuscation Strategy**:

- `NET_PASS` parameter marked as **hidden**
- Not returned in `PARAM_REQUEST_READ` or `PARAM_REQUEST_LIST` responses
- Can only be **written** via `PARAM_SET`, never read back
- GCS can verify connection success without reading password
- Stored in Flash in plaintext (RP2040/RP2350 limitation)

**Security Trade-offs**:

- ✅ Not readable via GCS (hidden from parameter list)
- ✅ Change without reflashing firmware
- ❌ Still in Flash memory (extractable via firmware dump)
- ❌ Not encrypted (RP2040 has no secure enclave)

### Decision Drivers

1. **User Experience**: Change WiFi settings without reflashing firmware
2. **Flexibility**: Single binary works with multiple networks
3. **GCS Integration**: Configure via familiar QGroundControl interface
4. **Security**: Password hidden from GCS readout (not returned in PARAM_VALUE)
5. **Production Ready**: Field reconfiguration without recompilation

### Considered Options

- **Option A: Compile-Time Environment Variables**
- **Option B: Runtime Parameters (MAVLink PARAM_SET)** ⭐ Selected
- **Option C: WiFi AP Mode with Web Interface**
- **Option D: Serial Console Configuration**

### Option Analysis

**Option A: Compile-Time Environment Variables**

- **Pros**:
  - Simple implementation
  - Secure (credentials not readable via GCS)
  - No Flash storage needed
  - Fast development iteration
- **Cons**:
  - Network change requires rebuild
  - Not user-friendly for field reconfiguration
  - Different binary per network
- **Effort**: Low

**Option B: Runtime Parameters (MAVLink PARAM_SET)**

- **Pros**:
  - Change network without reflash
  - User-friendly via GCS
  - Single binary for all networks
- **Cons**:
  - Password visible via PARAM_REQUEST_READ (security risk)
  - Must implement parameter hiding
  - Flash storage required
  - More complex implementation
- **Effort**: Medium

**Option C: WiFi AP Mode with Web Interface**

- **Pros**:
  - User-friendly setup
  - No GCS or UART needed
  - Common IoT pattern
- **Cons**:
  - Significant implementation effort (web server, AP mode, UI)
  - More memory (HTTP server + AP mode)
  - Complexity
- **Effort**: High

**Option D: Serial Console Configuration**

- **Pros**:
  - Interactive configuration
  - More secure than parameters
- **Cons**:
  - Requires UART access
  - Must implement CLI
  - Not user-friendly
- **Effort**: Medium

## Rationale

**Option B (runtime parameters via MAVLink) was chosen** because:

1. **User Experience**: Field reconfiguration without firmware rebuild dramatically improves usability
2. **Flexibility**: Single firmware binary works across multiple networks (development, testing, production)
3. **GCS Integration**: Familiar QGroundControl interface for configuration (no custom tools needed)
4. **Security Balance**: Password hiding prevents GCS readout while allowing updates
5. **Production Ready**: Essential for field deployments where network changes are needed

### Trade-offs Accepted

- **Flash Storage Required**: Need parameter persistence system (\~4 KB Flash)
- **Parameter Complexity**: Must implement parameter hiding for NET_PASS
- **Security Not Perfect**: Password still in Flash (but RP2040/RP2350 have no secure enclave anyway)
- **Implementation Effort**: Medium complexity vs compile-time (but worth usability gain)

### Why Not Other Options

**Option A (Compile-time) Rejected**:

- Network change requires rebuild (unacceptable for field use)
- Multiple firmware binaries needed for different sites
- Poor user experience for non-developers

**Option C (WiFi AP Mode) Rejected**:

- High implementation effort (HTTP server, Web UI)
- Significant memory overhead (\~20 KB RAM for HTTP + AP)
- Overkill when GCS already available via UART

**Option D (Serial Console) Rejected**:

- Must implement custom CLI
- Requires UART access (not always available in field)
- Less user-friendly than GCS parameters

## Consequences

### Positive

- **Flexible**: Change WiFi settings without reflashing firmware
- **User-Friendly**: Configure via familiar QGroundControl interface
- **Single Binary**: Same firmware works on all networks
- **Secure Enough**: Password hidden from GCS readout (though still in Flash)
- **Production Ready**: Field reconfiguration essential feature

### Negative

- **Flash Storage Required**: Need parameter persistence (\~4 KB Flash)
- **Parameter Hiding Complexity**: Must implement NET_PASS hiding logic
- **Flash Security**: Password extractable via firmware dump (RP2040/RP2350 limitation)
- **Implementation Effort**: Medium complexity (parameter storage + hiding)

### Neutral

- **Security Trade-off**: Password in Flash acceptable given hardware limitations
- **GCS Dependency**: Initial configuration requires UART connection to GCS

## Implementation Notes

### Phase 1: Parameter Storage Foundation

1. Implement Flash-backed parameter storage (`src/parameters/storage.rs`)
2. Add parameter registration system
3. Implement parameter persistence (write to Flash on change)
4. Add parameter loading from Flash on boot

### Phase 2: WiFi Parameter Registration

1. Define WiFi parameters (`NET_SSID`, `NET_PASS`, `NET_DHCP`, etc.)
2. Register parameters in parameter store
3. Add `WifiConfig::from_params()` in `src/platform/*/network.rs`
4. Load WiFi config from parameters at startup

### Phase 3: MAVLink Parameter Protocol

1. Implement `PARAM_REQUEST_LIST` handler
2. Implement `PARAM_REQUEST_READ` handler (with hiding logic)
3. Implement `PARAM_SET` handler (with Flash persistence)
4. Add `is_hidden()` check for `NET_PASS`

### Phase 4: Testing

1. Test parameter set via QGroundControl
2. Test WiFi connection with runtime parameters
3. Verify NET_PASS is hidden from PARAM_REQUEST_READ
4. Test parameter persistence across reboots

### Documentation

````markdown
## WiFi Configuration

### Quick Start

1. Set environment variables:
   ```bash
   export WIFI_SSID="YourNetwork"
   export WIFI_PASSWORD="YourPassword"
   ```
````

2. Build:

   ```bash
   ./scripts/build-rp2350.sh --release
   ```

3. Flash and connect

### Using .env File

Create `.env` (gitignored):

```bash
WIFI_SSID=MyNetwork
WIFI_PASSWORD=SecurePassword123
WIFI_DHCP=true
```

Load and build:

```bash
source .env
./scripts/build-rp2350.sh
```

### Static IP Configuration

```bash
export WIFI_SSID="MyNetwork"
export WIFI_PASSWORD="SecurePassword123"
export WIFI_DHCP=false
export WIFI_IP="192.168.1.100"
export WIFI_NETMASK="255.255.255.0"
export WIFI_GATEWAY="192.168.1.1"
```

````

## Examples

### Development Workflow

```bash
# Home network
export WIFI_SSID="HomeWiFi"
export WIFI_PASSWORD="homepass"
./scripts/build-rp2350.sh && probe-rs run target/uf2/example.uf2

# Office network
export WIFI_SSID="OfficeWiFi"
export WIFI_PASSWORD="officepass"
./scripts/build-rp2350.sh && probe-rs run target/uf2/example.uf2

# Test network with static IP
export WIFI_SSID="TestNet"
export WIFI_PASSWORD="testpass"
export WIFI_DHCP=false
export WIFI_IP="10.0.0.50"
./scripts/build-rp2350.sh
````

### Production Deployment

```bash
# Site A
export WIFI_SSID="SiteA-Network"
export WIFI_PASSWORD="SecurePass123"
./scripts/build-rp2350.sh --release
cp target/pico_trail.uf2 firmware/site-a-firmware.uf2

# Site B
export WIFI_SSID="SiteB-Network"
export WIFI_PASSWORD="DifferentPass456"
./scripts/build-rp2350.sh --release
cp target/pico_trail.uf2 firmware/site-b-firmware.uf2
```

## Platform Considerations

- **RP2040/RP2350**: No secure enclave for password storage
- **CYW43439**: WPA2 only (handled by firmware)
- **Flash Memory**: Credentials in firmware binary (encrypted by default on RP2350)

## Security & Privacy

### Security Considerations

**Password Storage**:

- Stored in firmware binary (Flash)
- Readable if binary extracted (firmware readout)
- RP2350: Optional Flash encryption (not implemented initially)
- **Recommendation**: Use strong network password, rotate if device lost

**Mitigation Strategies**:

1. **Physical Security**: Protect device from theft
2. **Network Security**: Use WPA2, strong password
3. **Future**: Implement Flash encryption on RP2350
4. **Future**: Runtime parameter with no-readout flag

**Not Mitigated**:

- Binary extraction → password recovery
- Accept this risk for initial release

### Privacy Considerations

- SSID visible in binary (public information, acceptable)
- Password visible in binary (risk accepted, document)
- No telemetry of WiFi credentials

## Monitoring & Logging

```rust
// Log WiFi connection status (not credentials)
defmt::info!("WiFi connecting to SSID: {}", wifi_config.ssid);

// Don't log password
// BAD: defmt::info!("Password: {}", wifi_config.password);
// GOOD: (don't log at all)

// Connection result
match control.join_wpa2(ssid, password).await {
    Ok(_) => defmt::info!("WiFi connected"),
    Err(e) => defmt::error!("WiFi connection failed: {:?}", e),
}

// IP address (safe to log)
defmt::info!("IP address: {:?}", ip_address);
```

## Open Questions

- [ ] Should we support WPA3 if CYW43439 firmware supports it? → Method: Check firmware capabilities, add if available
- [ ] Do we need WiFi connection retry logic? → Next step: Yes, implement exponential backoff (covered in FR-dxvrs)
- [ ] Should .env file be supported by default? → Decision: Yes, add to .gitignore, document usage

## External References

- Rust env! macro: <https://doc.rust-lang.org/std/macro.env.html>
- RP2350 Flash Encryption: <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf> (Chapter 12)
- WPA2 Security: <https://www.wi-fi.org/discover-wi-fi/security>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
