//! WiFi Parameter Definitions
//!
//! Defines WiFi-related parameters for network configuration.
//!
//! # Parameters
//!
//! - `NET_SSID` - WiFi network name (max 32 chars)
//! - `NET_PASS` - WiFi password (max 63 chars, hidden from MAVLink readout)
//! - `NET_DHCP` - Use DHCP (true) or static IP (false)
//! - `NET_IP` - Static IP address (used if NET_DHCP = false)
//! - `NET_NETMASK` - Network mask (used if NET_DHCP = false)
//! - `NET_GATEWAY` - Gateway address (used if NET_DHCP = false)
//!
//! # Security Note
//!
//! NET_PASS is marked as HIDDEN, preventing readout via MAVLink PARAM_REQUEST_READ.
//! However, the password is still stored in Flash and can be extracted from firmware binary.
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::parameters::{ParameterStore, ParamValue};
//! use pico_trail::parameters::wifi::WifiParams;
//! use pico_trail::platform::rp2350::Rp2350Flash;
//!
//! let mut flash = Rp2350Flash::new();
//! let mut store = ParameterStore::load_from_flash(&mut flash).unwrap();
//!
//! // Register WiFi parameters with defaults
//! WifiParams::register_defaults(&mut store);
//!
//! // Load WiFi configuration
//! let wifi_config = WifiParams::from_store(&store);
//! ```

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;
use heapless::String;

/// Maximum SSID length (IEEE 802.11 standard)
pub const MAX_SSID_LEN: usize = 32;

/// Maximum WiFi password length (WPA2 standard)
pub const MAX_PASSWORD_LEN: usize = 63;

/// WiFi parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct WifiParams {
    /// WiFi network SSID
    pub ssid: String<MAX_SSID_LEN>,
    /// WiFi password (WPA2)
    pub password: String<MAX_PASSWORD_LEN>,
    /// Use DHCP for IP configuration
    pub use_dhcp: bool,
    /// Static IP address (used if use_dhcp = false)
    pub static_ip: [u8; 4],
    /// Network mask (used if use_dhcp = false)
    pub netmask: [u8; 4],
    /// Gateway address (used if use_dhcp = false)
    pub gateway: [u8; 4],
}

impl WifiParams {
    /// Register WiFi parameters with default values
    ///
    /// Registers all WiFi parameters in the parameter store with sensible defaults.
    /// Parameters are only registered if they don't already exist.
    ///
    /// Default values can be provided at build time via environment variables:
    /// - `WIFI_SSID` - WiFi network name
    /// - `WIFI_PASSWORD` - WiFi password
    /// - `WIFI_DHCP` - Use DHCP (true/false)
    /// - `WIFI_IP` - Static IP address (e.g., "192.168.1.100")
    /// - `WIFI_NETMASK` - Network mask (e.g., "255.255.255.0")
    /// - `WIFI_GATEWAY` - Gateway address (e.g., "192.168.1.1")
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // Load defaults from build-time environment variables
        let default_ssid = env!("WIFI_SSID");
        let default_password = env!("WIFI_PASSWORD");
        let default_dhcp = env!("WIFI_DHCP");
        let default_ip = env!("WIFI_IP");
        let default_netmask = env!("WIFI_NETMASK");
        let default_gateway = env!("WIFI_GATEWAY");

        // NET_SSID - WiFi network name (from env or empty)
        let ssid_value = if default_ssid.is_empty() {
            String::new()
        } else {
            String::try_from(default_ssid).unwrap_or_else(|_| String::new())
        };
        store.register(
            "NET_SSID",
            ParamValue::String(ssid_value),
            ParamFlags::empty(),
        )?;

        // NET_PASS - WiFi password (from env or empty, HIDDEN from MAVLink readout)
        let pass_value = if default_password.is_empty() {
            String::new()
        } else {
            String::try_from(default_password).unwrap_or_else(|_| String::new())
        };
        store.register(
            "NET_PASS",
            ParamValue::String(pass_value),
            ParamFlags::HIDDEN,
        )?;

        // NET_DHCP - Use DHCP (from env or true)
        let dhcp_value = default_dhcp.parse::<bool>().unwrap_or(true);
        store.register(
            "NET_DHCP",
            ParamValue::Bool(dhcp_value),
            ParamFlags::empty(),
        )?;

        // NET_IP - Static IP address (from env or 0.0.0.0)
        let ip_value = parse_ipv4(default_ip).unwrap_or([0, 0, 0, 0]);
        store.register("NET_IP", ParamValue::Ipv4(ip_value), ParamFlags::empty())?;

        // NET_NETMASK - Network mask (from env or 255.255.255.0)
        let netmask_value = parse_ipv4(default_netmask).unwrap_or([255, 255, 255, 0]);
        store.register(
            "NET_NETMASK",
            ParamValue::Ipv4(netmask_value),
            ParamFlags::empty(),
        )?;

        // NET_GATEWAY - Gateway address (from env or 0.0.0.0)
        let gateway_value = parse_ipv4(default_gateway).unwrap_or([0, 0, 0, 0]);
        store.register(
            "NET_GATEWAY",
            ParamValue::Ipv4(gateway_value),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load WiFi parameters from parameter store
    ///
    /// Reads all WiFi parameters from the store and returns a WifiParams struct.
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to load from
    ///
    /// # Returns
    ///
    /// WifiParams with values from store, or defaults if parameters not found
    pub fn from_store(store: &ParameterStore) -> Self {
        let ssid = match store.get("NET_SSID") {
            Some(ParamValue::String(s)) => {
                // Convert from String<63> to String<32>
                let mut ssid = String::<MAX_SSID_LEN>::new();
                let copy_len = core::cmp::min(s.len(), MAX_SSID_LEN);
                ssid.push_str(&s[..copy_len]).ok();
                ssid
            }
            _ => String::new(),
        };

        let password = match store.get("NET_PASS") {
            Some(ParamValue::String(s)) => {
                // Convert from String<63> to String<63>
                let mut pass = String::<MAX_PASSWORD_LEN>::new();
                pass.push_str(s.as_str()).ok();
                pass
            }
            _ => String::new(),
        };

        let use_dhcp = match store.get("NET_DHCP") {
            Some(ParamValue::Bool(b)) => *b,
            _ => true,
        };

        let static_ip = match store.get("NET_IP") {
            Some(ParamValue::Ipv4(ip)) => *ip,
            _ => [0, 0, 0, 0],
        };

        let netmask = match store.get("NET_NETMASK") {
            Some(ParamValue::Ipv4(mask)) => *mask,
            _ => [255, 255, 255, 0],
        };

        let gateway = match store.get("NET_GATEWAY") {
            Some(ParamValue::Ipv4(gw)) => *gw,
            _ => [0, 0, 0, 0],
        };

        Self {
            ssid,
            password,
            use_dhcp,
            static_ip,
            netmask,
            gateway,
        }
    }

    /// Check if WiFi is configured
    ///
    /// Returns true if SSID is not empty (WiFi is configured).
    /// Empty SSID indicates WiFi should be skipped (UART-only mode).
    pub fn is_configured(&self) -> bool {
        !self.ssid.is_empty()
    }
}

impl Default for WifiParams {
    fn default() -> Self {
        Self {
            ssid: String::new(),
            password: String::new(),
            use_dhcp: true,
            static_ip: [0, 0, 0, 0],
            netmask: [255, 255, 255, 0],
            gateway: [0, 0, 0, 0],
        }
    }
}

/// Parse IPv4 address from string (e.g., "192.168.1.1")
///
/// # Arguments
///
/// * `s` - String in "a.b.c.d" format
///
/// # Returns
///
/// Ok([a, b, c, d]) if valid, Err if parsing fails
fn parse_ipv4(s: &str) -> core::result::Result<[u8; 4], ()> {
    let parts: heapless::Vec<&str, 4> = s.split('.').collect();
    if parts.len() != 4 {
        return Err(());
    }

    let mut result = [0u8; 4];
    for (i, part) in parts.iter().enumerate() {
        result[i] = part.parse::<u8>().map_err(|_| ())?;
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wifi_params_defaults() {
        let mut store = ParameterStore::new();
        WifiParams::register_defaults(&mut store).unwrap();

        // Check that parameters are registered
        assert!(store.get("NET_SSID").is_some());
        assert!(store.get("NET_PASS").is_some());
        assert!(store.get("NET_DHCP").is_some());
        assert!(store.get("NET_IP").is_some());
        assert!(store.get("NET_NETMASK").is_some());
        assert!(store.get("NET_GATEWAY").is_some());

        // Check that NET_PASS is hidden
        assert!(store.is_hidden("NET_PASS"));
    }

    #[test]
    fn test_wifi_params_from_store() {
        let mut store = ParameterStore::new();
        WifiParams::register_defaults(&mut store).unwrap();

        // Set some values
        store
            .set(
                "NET_SSID",
                ParamValue::String(String::try_from("TestNetwork").unwrap()),
            )
            .unwrap();
        store
            .set(
                "NET_PASS",
                ParamValue::String(String::try_from("password123").unwrap()),
            )
            .unwrap();
        store.set("NET_DHCP", ParamValue::Bool(false)).unwrap();
        store
            .set("NET_IP", ParamValue::Ipv4([192, 168, 1, 100]))
            .unwrap();

        // Load params
        let params = WifiParams::from_store(&store);

        assert_eq!(params.ssid.as_str(), "TestNetwork");
        assert_eq!(params.password.as_str(), "password123");
        assert!(!params.use_dhcp);
        assert_eq!(params.static_ip, [192, 168, 1, 100]);
    }

    #[test]
    fn test_wifi_params_is_configured() {
        let mut params = WifiParams::default();
        assert!(!params.is_configured()); // Empty SSID

        params.ssid = String::try_from("MyNetwork").unwrap();
        assert!(params.is_configured()); // Non-empty SSID
    }
}
